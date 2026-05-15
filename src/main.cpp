#include <Arduino.h>
#include <Wire.h>
#include <WiFiManager.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <sMQTTBroker.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "config.h"

// ── Display ──────────────────────────────────────────────────
Arduino_DataBus *spi_bus = new Arduino_SWSPI(
    GFX_NOT_DEFINED, TFT_SPI_CS, TFT_SPI_SCK, TFT_SPI_SDA, GFX_NOT_DEFINED);
Arduino_ESP32RGBPanel *rgb_bus = new Arduino_ESP32RGBPanel(
    TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK,
    TFT_R0,TFT_R1,TFT_R2,TFT_R3,TFT_R4,
    TFT_G0,TFT_G1,TFT_G2,TFT_G3,TFT_G4,TFT_G5,
    TFT_B0,TFT_B1,TFT_B2,TFT_B3,TFT_B4,
    1,10,4,20, 1,10,4,20, 1);
Arduino_RGB_Display *display = new Arduino_RGB_Display(
    TFT_WIDTH, TFT_HEIGHT, rgb_bus, 0, true,
    spi_bus, GFX_NOT_DEFINED,
    st7701_type5_init_operations, sizeof(st7701_type5_init_operations));

// ── Paleta ───────────────────────────────────────────────────
#define COL_BG       0x0A0A0A
#define COL_CARD     0x1E1C1C
#define COL_TEXT     0xFFFFFF
#define COL_MUTED    0x938E8E
#define COL_OK       0x58D130
#define COL_ALERT    0x303BFF
#define COL_RELAY_ON 0x0A9FFF
#define COL_OFF      0x3C3A3A

// ── Estado A6v3 ──────────────────────────────────────────────
struct { bool input[7]={}; bool output[7]={}; } a6v3;
static bool ui_needs_update = false;

// ── Alarma ───────────────────────────────────────────────────
static bool alarm_armed      = false;
static bool intruder_active  = false;
static bool critical_alert   = false;

// ── PIN ──────────────────────────────────────────────────────
#define PIN_CODE "1234"
static char pin_buf[5] = {};
static int  pin_len    = 0;
static bool pin_for_arm = true;

// ── Tiempo (Open-Meteo) ──────────────────────────────────────
struct { float temp=NAN; float wind=0; float rain=0; bool ok=false; } wx;
static unsigned long wx_last = 0;
#define WX_URL "https://api.open-meteo.com/v1/forecast" \
    "?latitude=42.80632457618044&longitude=-1.6290100870212767" \
    "&current=temperature_2m,wind_speed_10m" \
    "&hourly=precipitation,wind_speed_10m" \
    "&forecast_days=1&timezone=Europe%2FMadrid"

// ── MQTT ─────────────────────────────────────────────────────
#define MQTT_STATE "A6v3/30EDA03B1378/STATE"
#define MQTT_SET   "A6v3/30EDA03B1378/SET"

class DomusBroker : public sMQTTBroker {
public:
    bool onEvent(sMQTTEvent *ev) override {
        if (ev->Type() == NewClient_sMQTTEventType)
            Serial.println("Broker: cliente conectado");
        else if (ev->Type() == Public_sMQTTEventType) {
            auto *p = (sMQTTPublicClientEvent*)ev;
            if (p->Topic() != MQTT_STATE) return true;
            JsonDocument doc;
            if (deserializeJson(doc, p->Payload().c_str(), p->Payload().size())) return true;
            for (int i=1;i<=6;i++) {
                char k[10];
                snprintf(k,10,"input%d",i);
                if (!doc[k]["value"].isNull()) a6v3.input[i]=doc[k]["value"].as<bool>();
                snprintf(k,10,"output%d",i);
                if (!doc[k]["value"].isNull()) a6v3.output[i]=doc[k]["value"].as<bool>();
            }
            ui_needs_update = true;
        }
        return true;
    }
} broker;

static void relay_set(int n, bool v) {
    char buf[64];
    snprintf(buf,64,"{\"output%d\":{\"value\":%s}}",n,v?"true":"false");
    broker.publish(std::string(MQTT_SET), std::string(buf));
    Serial.printf("SET out%d=%d\n",n,v);
}

// ── PCF8574 / LCD ────────────────────────────────────────────
static void pcf8574_write(uint8_t v) {
    Wire.beginTransmission(PCF8574_ADDR); Wire.write(v); Wire.endTransmission();
}
static void lcd_power_on() {
    pcf8574_write(0xFF); delay(10);
    pcf8574_write(0xFF&~(1<<PCF_LCD_RESET)); delay(100);
    pcf8574_write(0xFF); delay(100);
}

// ── LVGL flush ───────────────────────────────────────────────
static lv_color_t *lvgl_buf = nullptr;
static void lvgl_flush_cb(lv_display_t *d, const lv_area_t *a, uint8_t *px) {
    display->draw16bitRGBBitmap(a->x1,a->y1,(uint16_t*)px,
        a->x2-a->x1+1, a->y2-a->y1+1);
    lv_display_flush_ready(d);
}

// ── Touch (CST8XX) ───────────────────────────────────────────
static uint8_t touch_i2c_addr = 0;

static void touch_init() {
    uint8_t candidates[] = {0x15, 0x38, 0x5D};
    for (uint8_t a : candidates) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) { touch_i2c_addr = a; break; }
    }
    Serial.printf("Touch addr: 0x%02X\n", touch_i2c_addr);
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    if (!touch_i2c_addr) { data->state = LV_INDEV_STATE_RELEASED; return; }
    Wire.beginTransmission(touch_i2c_addr);
    Wire.write(0x01);
    Wire.endTransmission(false);
    Wire.requestFrom(touch_i2c_addr, (uint8_t)6);
    if (Wire.available() >= 6) {
        Wire.read(); // gesture
        uint8_t fingers = Wire.read();
        uint8_t xh = Wire.read(), xl = Wire.read();
        uint8_t yh = Wire.read(), yl = Wire.read();
        if (fingers > 0) {
            data->point.x = ((xh & 0x0F) << 8) | xl;
            data->point.y = ((yh & 0x0F) << 8) | yl;
            data->state   = LV_INDEV_STATE_PRESSED;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
    } else {
        while (Wire.available()) Wire.read();
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ── Widgets globales ─────────────────────────────────────────
static lv_obj_t *tv;
static lv_obj_t *tile_home, *tile_sensors, *tile_relays, *tile_alarm, *tile_settings;

// Home
static lv_obj_t *lbl_alarm_badge;
static lv_obj_t *lbl_temp_ext, *lbl_weather, *lbl_temp_int;
// Indicadores radiales (home)
static lv_obj_t *hdot[5];   // h=home sensors, indices 1-4
static lv_obj_t *rdot[4];   // r=relays, indices 1-3
// Sensor tile
static lv_obj_t *sdot[5], *slbl[5];
// Relay tile
static lv_obj_t *rbtn[4], *rlbl[4];
// Alarm tile
static lv_obj_t *lbl_arm_state, *btn_arm;
// Overlays
static lv_obj_t *scr_alarm=nullptr, *scr_pin=nullptr, *scr_water=nullptr;
static lv_obj_t *lbl_alarm_type, *lbl_alarm_detail;
static lv_obj_t *lbl_pin_title, *lbl_pin_dots, *lbl_pin_msg;
static lv_obj_t *lbl_wvalve, *lbl_wflood, *btn_wopen;

// ── Navegación ───────────────────────────────────────────────
enum Screen { SCR_TV, SCR_ALARM_CRIT, SCR_PIN, SCR_WATER };
static Screen cur_scr = SCR_TV;
static Screen pend_scr = SCR_TV;
static bool   scr_change = false;
static void go_to(Screen s) { pend_scr=s; scr_change=true; }

// ── Helpers UI ───────────────────────────────────────────────
static lv_obj_t *make_dot_at(lv_obj_t *parent, int px, int py) {
    lv_obj_t *d = lv_obj_create(parent);
    lv_obj_set_size(d, 14, 14);
    lv_obj_set_pos(d, px, py);
    lv_obj_set_style_radius(d, 7, 0);
    lv_obj_set_style_border_width(d, 0, 0);
    lv_obj_set_style_bg_color(d, lv_color_hex(COL_OFF), 0);
    lv_obj_set_style_pad_all(d, 0, 0);
    lv_obj_clear_flag(d, LV_OBJ_FLAG_SCROLLABLE);
    return d;
}

static lv_obj_t *make_label_at(lv_obj_t *parent, const char *txt,
                                int px, int py, int w, bool right_align) {
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_width(l, w);
    lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(COL_MUTED), 0);
    lv_obj_set_style_text_align(l,
        right_align ? LV_TEXT_ALIGN_RIGHT : LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_pos(l, px, py);
    return l;
}

static lv_obj_t *make_big_btn(lv_obj_t *parent, const char *txt,
                               uint32_t col, int y, int w, int h,
                               lv_event_cb_t cb, void *ud) {
    lv_obj_t *b = lv_obj_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_align(b, LV_ALIGN_CENTER, 0, y);
    lv_obj_set_style_bg_color(b, lv_color_hex(col), 0);
    lv_obj_set_style_radius(b, h/2, 0);
    lv_obj_set_style_border_width(b, 0, 0);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(COL_TEXT), 0);
    lv_obj_center(l);
    return b;
}

static lv_obj_t *centered_label(lv_obj_t *parent, const char *txt,
                                 const lv_font_t *font, uint32_t col, int y) {
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(col), 0);
    lv_obj_align(l, LV_ALIGN_CENTER, 0, y);
    return l;
}

static void bg(lv_obj_t *o) {
    lv_obj_set_style_bg_color(o, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
}

// ── Indicadores radiales (posiciones precalculadas) ──────────
// Sensores: θ = 315°, 285°, 255°, 225° desde arriba CW, r=185
// dot_pos[i] = top-left del dot (14x14)
static const int S_PX[5] = {0, 102, 54, 54, 102};
static const int S_PY[5] = {0, 102, 185, 281, 364};
// Relés: θ = 45°, 90°, 135°, r=185
static const int R_PX[4] = {0, 364, 418, 364};
static const int R_PY[4] = {0, 102, 233, 364};
static const char *S_SHORT[5] = {"","INUND","PIR","HUMO","220V"};
static const char *R_SHORT[4] = {"","AGUA","CALEF","SIR"};

// ── BUILD TILE HOME ──────────────────────────────────────────
static void cb_alarm_badge(lv_event_t *e) {
    lv_tileview_set_tile(tv, tile_alarm, LV_ANIM_ON);
}
static void cb_settings_icon(lv_event_t *e) {
    lv_tileview_set_tile(tv, tile_settings, LV_ANIM_ON);
}

static void build_tile_home() {
    bg(tile_home);

    // Badge alarma (arriba, tappable)
    lbl_alarm_badge = lv_label_create(tile_home);
    lv_label_set_text(lbl_alarm_badge, "DESARMADA");
    lv_obj_set_style_text_font(lbl_alarm_badge, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_alarm_badge, lv_color_hex(COL_MUTED), 0);
    lv_obj_set_pos(lbl_alarm_badge, 190, 62);
    lv_obj_add_flag(lbl_alarm_badge, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lbl_alarm_badge, cb_alarm_badge, LV_EVENT_CLICKED, nullptr);

    // Temperatura exterior (centro arriba)
    lbl_temp_ext = centered_label(tile_home, "--.-°C", &lv_font_montserrat_40, COL_TEXT, -100);

    // Viento + lluvia
    lbl_weather = centered_label(tile_home, "-- km/h  -- mm", &lv_font_montserrat_14, COL_MUTED, -48);

    // Temperatura interior (centro abajo)
    centered_label(tile_home, "Interior", &lv_font_montserrat_14, COL_MUTED, 18);
    lbl_temp_int = centered_label(tile_home, "--.-°C", &lv_font_montserrat_20, COL_TEXT, 46);

    // Icono ajustes (abajo, tappable)
    lv_obj_t *lbl_cfg = lv_label_create(tile_home);
    lv_label_set_text(lbl_cfg, "AJUSTES");
    lv_obj_set_style_text_font(lbl_cfg, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_cfg, lv_color_hex(COL_MUTED), 0);
    lv_obj_set_pos(lbl_cfg, 200, 396);
    lv_obj_add_flag(lbl_cfg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lbl_cfg, cb_settings_icon, LV_EVENT_CLICKED, nullptr);

    // Indicadores radiales sensores (izquierda)
    for (int i=1;i<=4;i++) {
        hdot[i] = make_dot_at(tile_home, S_PX[i], S_PY[i]);
        // etiqueta a la derecha del dot, hacia el centro
        make_label_at(tile_home, S_SHORT[i],
            S_PX[i]+18, S_PY[i], 72, false);
    }

    // Indicadores radiales relés (derecha)
    for (int i=1;i<=3;i++) {
        rdot[i] = make_dot_at(tile_home, R_PX[i], R_PY[i]);
        // etiqueta a la izquierda del dot, hacia el centro
        make_label_at(tile_home, R_SHORT[i],
            R_PX[i]-76, R_PY[i], 72, true);
    }
}

// ── BUILD TILE SENSORES ──────────────────────────────────────
static const char *S_NAME[5] = {"","Inundacion","Movimiento","Humo","Red 220V"};

static void build_tile_sensors() {
    bg(tile_sensors);
    centered_label(tile_sensors, "SENSORES", &lv_font_montserrat_20, COL_TEXT, -175);
    for (int i=1;i<=4;i++) {
        int y = -100 + (i-1)*60;
        lv_obj_t *card = lv_obj_create(tile_sensors);
        lv_obj_set_size(card, 300, 44);
        lv_obj_align(card, LV_ALIGN_CENTER, 0, y);
        lv_obj_set_style_bg_color(card, lv_color_hex(COL_CARD), 0);
        lv_obj_set_style_border_width(card, 0, 0);
        lv_obj_set_style_radius(card, 10, 0);
        lv_obj_set_style_pad_all(card, 0, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        slbl[i] = lv_label_create(card);
        lv_label_set_text(slbl[i], S_NAME[i]);
        lv_obj_set_style_text_font(slbl[i], &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(slbl[i], lv_color_hex(COL_MUTED), 0);
        lv_obj_align(slbl[i], LV_ALIGN_LEFT_MID, 14, 0);

        sdot[i] = make_dot_at(card, 272, 15);
    }
}

// ── BUILD TILE RELÉS ─────────────────────────────────────────
static void cb_relay_btn(lv_event_t *e) {
    int n = (int)(intptr_t)lv_event_get_user_data(e);
    if (n == 1) { go_to(SCR_WATER); }
    else        { relay_set(n, !a6v3.output[n]); }
}
static const char *R_NAME[4] = {"","Agua","Calefaccion","Sirena"};

static void build_tile_relays() {
    bg(tile_relays);
    centered_label(tile_relays, "RELES", &lv_font_montserrat_20, COL_TEXT, -175);
    for (int i=1;i<=3;i++) {
        int y = -80 + (i-1)*80;
        rbtn[i] = lv_obj_create(tile_relays);
        lv_obj_set_size(rbtn[i], 280, 60);
        lv_obj_align(rbtn[i], LV_ALIGN_CENTER, 0, y);
        lv_obj_set_style_bg_color(rbtn[i], lv_color_hex(COL_OFF), 0);
        lv_obj_set_style_radius(rbtn[i], 12, 0);
        lv_obj_set_style_border_width(rbtn[i], 0, 0);
        lv_obj_set_style_pad_all(rbtn[i], 0, 0);
        lv_obj_clear_flag(rbtn[i], LV_OBJ_FLAG_SCROLLABLE);
        if (i < 3) { // sirena no interactiva
            lv_obj_add_flag(rbtn[i], LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(rbtn[i], cb_relay_btn, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        }
        rlbl[i] = lv_label_create(rbtn[i]);
        lv_label_set_text(rlbl[i], R_NAME[i]);
        lv_obj_set_style_text_font(rlbl[i], &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(rlbl[i], lv_color_hex(COL_TEXT), 0);
        lv_obj_center(rlbl[i]);
    }
}

// ── BUILD TILE ALARMA ────────────────────────────────────────
static void cb_go_pin(lv_event_t *e) {
    pin_for_arm = !alarm_armed;
    memset(pin_buf,0,5); pin_len=0;
    go_to(SCR_PIN);
}

static void build_tile_alarm() {
    bg(tile_alarm);
    centered_label(tile_alarm, "ALARMA INTRUSION", &lv_font_montserrat_20, COL_TEXT, -160);
    lbl_arm_state = centered_label(tile_alarm, "DESARMADA", &lv_font_montserrat_40, COL_MUTED, -60);
    btn_arm = make_big_btn(tile_alarm, "ARMAR", COL_OFF, 60, 240, 70, cb_go_pin, nullptr);
}

// ── BUILD TILE SETTINGS ──────────────────────────────────────
static void build_tile_settings() {
    bg(tile_settings);
    centered_label(tile_settings, "SISTEMA", &lv_font_montserrat_20, COL_TEXT, -160);
    char buf[64];
    snprintf(buf, sizeof(buf), "IP: %s", WiFi.localIP().toString().c_str());
    centered_label(tile_settings, buf, &lv_font_montserrat_14, COL_MUTED, -80);
    centered_label(tile_settings, "DOMUS v0.3", &lv_font_montserrat_14, COL_MUTED, -50);
    centered_label(tile_settings, "Sistema OK.", &lv_font_montserrat_14, COL_OK, -20);
}

// ── BUILD OVERLAY ALARMA CRÍTICA ─────────────────────────────
static void cb_deactivate(lv_event_t *e) {
    if (intruder_active) {
        pin_for_arm = false; memset(pin_buf,0,5); pin_len=0;
        go_to(SCR_PIN);
    } else {
        relay_set(3, false);
        critical_alert = false;
        go_to(SCR_TV);
    }
}

static void build_scr_alarm() {
    scr_alarm = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_alarm, lv_color_hex(0x1A0000), 0);
    lv_obj_set_style_bg_opa(scr_alarm, LV_OPA_COVER, 0);
    lbl_alarm_type   = centered_label(scr_alarm,"! ALARMA !",&lv_font_montserrat_40,COL_ALERT,-80);
    lbl_alarm_detail = centered_label(scr_alarm,"",&lv_font_montserrat_20,COL_TEXT,-20);
    make_big_btn(scr_alarm,"DESACTIVAR",COL_ALERT,70,260,80,cb_deactivate,nullptr);
}

// ── BUILD OVERLAY PIN ─────────────────────────────────────────
static void update_pin_dots() {
    char b[9]="_ _ _ _";
    for (int i=0;i<pin_len&&i<4;i++) b[i*2]='*';
    lv_label_set_text(lbl_pin_dots, b);
}

static void pin_check() {
    if (strncmp(pin_buf, PIN_CODE, 4)==0) {
        alarm_armed = pin_for_arm;
        if (!pin_for_arm) { intruder_active=false; relay_set(3,false); }
        lv_label_set_text(lbl_pin_msg,"");
        go_to(SCR_TV);
    } else {
        lv_label_set_text(lbl_pin_msg,"PIN incorrecto");
        memset(pin_buf,0,5); pin_len=0; update_pin_dots();
    }
}

static void cb_pin_key(lv_event_t *e) {
    int d = (int)(intptr_t)lv_event_get_user_data(e);
    if (d==-1) {
        if (pin_len>0) { pin_len--; pin_buf[pin_len]=0; }
        update_pin_dots();
    } else if (pin_len<4) {
        pin_buf[pin_len++]='0'+d;
        update_pin_dots();
        if (pin_len==4) pin_check();
    }
}
static void cb_pin_cancel(lv_event_t *e) { memset(pin_buf,0,5);pin_len=0;go_to(SCR_TV); }

static void build_scr_pin() {
    scr_pin = lv_obj_create(nullptr);
    bg(scr_pin);
    lbl_pin_title = centered_label(scr_pin,"PIN",&lv_font_montserrat_20,COL_TEXT,-175);
    lbl_pin_dots  = centered_label(scr_pin,"_ _ _ _",&lv_font_montserrat_40,COL_TEXT,-115);
    lbl_pin_msg   = centered_label(scr_pin,"",&lv_font_montserrat_14,COL_ALERT,-70);

    const int W=80,H=60,G=10;
    const int sx=-(3*W+2*G)/2+W/2, sy=-40;
    const int digits[11]={1,2,3,4,5,6,7,8,9,-1,0};
    const char *lbs[11]={"1","2","3","4","5","6","7","8","9","DEL","0"};
    for (int i=0;i<11;i++) {
        lv_obj_t *b=lv_obj_create(scr_pin);
        lv_obj_set_size(b,W,H);
        lv_obj_align(b,LV_ALIGN_CENTER,sx+(i%3)*(W+G),sy+(i/3)*(H+G));
        lv_obj_set_style_bg_color(b,lv_color_hex(COL_CARD),0);
        lv_obj_set_style_radius(b,12,0);
        lv_obj_set_style_border_width(b,0,0);
        lv_obj_set_style_pad_all(b,0,0);
        lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(b,LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(b,cb_pin_key,LV_EVENT_CLICKED,(void*)(intptr_t)digits[i]);
        lv_obj_t *l=lv_label_create(b);
        lv_label_set_text(l,lbs[i]);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
        lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
        lv_obj_center(l);
    }
    make_big_btn(scr_pin,"CANCELAR",COL_OFF,185,200,44,cb_pin_cancel,nullptr);
}

// ── BUILD OVERLAY AGUA ────────────────────────────────────────
static void cb_wopen(lv_event_t *e)  { relay_set(1,true);  go_to(SCR_TV); }
static void cb_wclose(lv_event_t *e) { relay_set(1,false); go_to(SCR_TV); }
static void cb_wback(lv_event_t *e)  { go_to(SCR_TV); }

static void build_scr_water() {
    scr_water = lv_obj_create(nullptr);
    bg(scr_water);
    centered_label(scr_water,"VALVULA AGUA",&lv_font_montserrat_20,COL_TEXT,-155);
    lbl_wvalve  = centered_label(scr_water,"Valvula: ?",&lv_font_montserrat_20,COL_TEXT,-80);
    lbl_wflood  = centered_label(scr_water,"",&lv_font_montserrat_14,COL_ALERT,-40);
    btn_wopen   = make_big_btn(scr_water,"ABRIR",COL_OK,20,260,70,cb_wopen,nullptr);
    make_big_btn(scr_water,"CERRAR",COL_ALERT,105,260,70,cb_wclose,nullptr);
    make_big_btn(scr_water,"VOLVER",COL_OFF,185,200,44,cb_wback,nullptr);
}

// ── UPDATE UI ─────────────────────────────────────────────────
static void update_home() {
    // sensores
    lv_obj_set_style_bg_color(hdot[1],lv_color_hex(a6v3.input[1]?COL_ALERT:COL_OFF),0);
    lv_obj_set_style_bg_color(hdot[2],lv_color_hex(!a6v3.input[4]?COL_ALERT:COL_OK),0);
    lv_obj_set_style_bg_color(hdot[3],lv_color_hex(!a6v3.input[5]?COL_ALERT:COL_OK),0);
    lv_obj_set_style_bg_color(hdot[4],lv_color_hex(a6v3.input[6]?COL_OK:COL_ALERT),0);
    // relés
    lv_obj_set_style_bg_color(rdot[1],lv_color_hex(a6v3.output[1]?COL_RELAY_ON:COL_OFF),0);
    lv_obj_set_style_bg_color(rdot[2],lv_color_hex(a6v3.output[2]?COL_RELAY_ON:COL_OFF),0);
    lv_obj_set_style_bg_color(rdot[3],lv_color_hex(a6v3.output[3]?COL_ALERT:COL_OFF),0);
    // badge alarma
    lv_label_set_text(lbl_alarm_badge, alarm_armed ? "ARMADA" : "DESARMADA");
    lv_obj_set_style_text_color(lbl_alarm_badge,
        lv_color_hex(alarm_armed?COL_ALERT:COL_MUTED), 0);
}

static void update_sensors_tile() {
    lv_obj_set_style_bg_color(sdot[1],lv_color_hex(a6v3.input[1]?COL_ALERT:COL_OFF),0);
    lv_obj_set_style_bg_color(sdot[2],lv_color_hex(!a6v3.input[4]?COL_ALERT:COL_OK),0);
    lv_obj_set_style_bg_color(sdot[3],lv_color_hex(!a6v3.input[5]?COL_ALERT:COL_OK),0);
    lv_obj_set_style_bg_color(sdot[4],lv_color_hex(a6v3.input[6]?COL_OK:COL_ALERT),0);
}

static void update_relays_tile() {
    lv_obj_set_style_bg_color(rbtn[1],lv_color_hex(a6v3.output[1]?COL_RELAY_ON:COL_OFF),0);
    lv_obj_set_style_bg_color(rbtn[2],lv_color_hex(a6v3.output[2]?COL_RELAY_ON:COL_OFF),0);
    lv_obj_set_style_bg_color(rbtn[3],lv_color_hex(a6v3.output[3]?COL_ALERT:COL_OFF),0);
}

static void update_alarm_tile() {
    if (alarm_armed) {
        lv_label_set_text(lbl_arm_state, "ARMADA");
        lv_obj_set_style_text_color(lbl_arm_state, lv_color_hex(COL_ALERT), 0);
        lv_label_set_text((lv_obj_t*)lv_obj_get_child(btn_arm, 0), "DESARMAR");
        lv_obj_set_style_bg_color(btn_arm, lv_color_hex(COL_ALERT), 0);
    } else {
        lv_label_set_text(lbl_arm_state, "DESARMADA");
        lv_obj_set_style_text_color(lbl_arm_state, lv_color_hex(COL_MUTED), 0);
        lv_label_set_text((lv_obj_t*)lv_obj_get_child(btn_arm, 0), "ARMAR");
        lv_obj_set_style_bg_color(btn_arm, lv_color_hex(COL_OFF), 0);
    }
}

static void update_weather_display() {
    if (!wx.ok) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "%.1f\xc2\xb0""C", wx.temp);
    lv_label_set_text(lbl_temp_ext, buf);
    snprintf(buf, sizeof(buf), "%d km/h  %.1f mm", (int)wx.wind, wx.rain);
    lv_label_set_text(lbl_weather, buf);
}

// ── ALARMAS ───────────────────────────────────────────────────
static void check_alarms() {
    bool flood = a6v3.input[1];
    bool smoke = !a6v3.input[5];
    if ((flood || smoke) && !critical_alert) {
        critical_alert = true;
        relay_set(3, true);
        lv_label_set_text(lbl_alarm_type,  flood ? "INUNDACION" : "HUMO");
        lv_label_set_text(lbl_alarm_detail, flood
            ? "Valvulas cerradas automaticamente"
            : "Sirena activada");
        go_to(SCR_ALARM_CRIT);
    }
    if (alarm_armed && !a6v3.input[4] && !intruder_active) {
        intruder_active = true;
        relay_set(3, true);
        lv_label_set_text(lbl_alarm_type,   "INTRUSION");
        lv_label_set_text(lbl_alarm_detail, "Introduce PIN para desactivar");
        go_to(SCR_ALARM_CRIT);
    }
}

// ── SCREEN SWITCH ─────────────────────────────────────────────
static void do_switch(Screen s) {
    lv_obj_t *target = nullptr;
    switch (s) {
        case SCR_TV:
            update_home();
            update_sensors_tile();
            update_relays_tile();
            update_alarm_tile();
            target = lv_tileview_get_tile_act(tv);  // ya está en el tileview
            lv_scr_load_anim(tv, LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, false);
            cur_scr = SCR_TV; return;
        case SCR_ALARM_CRIT: target = scr_alarm;  break;
        case SCR_PIN:
            lv_label_set_text(lbl_pin_title,
                pin_for_arm ? "Armar alarma" : "Desarmar alarma");
            update_pin_dots();
            lv_label_set_text(lbl_pin_msg,"");
            target = scr_pin; break;
        case SCR_WATER:
            lv_label_set_text(lbl_wvalve,
                a6v3.output[1] ? "Valvula: ABIERTA" : "Valvula: CERRADA");
            lv_label_set_text(lbl_wflood,
                a6v3.input[1] ? "SENSOR INUNDACION ACTIVO" : "");
            if (a6v3.input[1]) lv_obj_add_flag(btn_wopen, LV_OBJ_FLAG_HIDDEN);
            else               lv_obj_clear_flag(btn_wopen, LV_OBJ_FLAG_HIDDEN);
            target = scr_water; break;
    }
    if (target) { lv_scr_load_anim(target,LV_SCR_LOAD_ANIM_FADE_IN,300,0,false); cur_scr=s; }
}

// ── WEATHER FETCH ─────────────────────────────────────────────
static void fetch_weather() {
    if (!WiFi.isConnected()) return;
    Serial.println("Fetching weather...");
    WiFiClientSecure client; client.setInsecure();
    HTTPClient http;
    http.begin(client, WX_URL);
    http.setTimeout(10000);
    int code = http.GET();
    if (code == 200) {
        String body = http.getString();
        JsonDocument doc;
        if (!deserializeJson(doc, body)) {
            wx.temp = doc["current"]["temperature_2m"].as<float>();
            float r=0, wmax=0;
            for (int i=0;i<12;i++) {
                r    += doc["hourly"]["precipitation"][i].as<float>();
                float w = doc["hourly"]["wind_speed_10m"][i].as<float>();
                if (w>wmax) wmax=w;
            }
            wx.rain=r; wx.wind=wmax; wx.ok=true;
            Serial.printf("Wx: %.1fC %.0fkm/h %.1fmm\n",wx.temp,wx.wind,wx.rain);
            update_weather_display();
        }
    } else Serial.printf("Wx HTTP: %d\n", code);
    http.end();
    wx_last = millis();
}

// ── SETUP ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    unsigned long t0=millis();
    while(!Serial&&(millis()-t0)<5000) delay(10);
    Serial.println("DOMUS arranque...");

    Wire.begin(I2C_SDA, I2C_SCL);
    lcd_power_on();
    touch_init();

    ledcSetup(0,5000,8);
    ledcAttachPin(TFT_BL_PIN,0);
    ledcWrite(0,204);

    display->begin();
    lv_init();

    lvgl_buf=(lv_color_t*)heap_caps_malloc(
        TFT_WIDTH*10*sizeof(lv_color_t),MALLOC_CAP_SPIRAM);
    if (!lvgl_buf) lvgl_buf=(lv_color_t*)malloc(TFT_WIDTH*10*sizeof(lv_color_t));

    lv_display_t *disp=lv_display_create(TFT_WIDTH,TFT_HEIGHT);
    lv_display_set_flush_cb(disp,lvgl_flush_cb);
    lv_display_set_buffers(disp,lvgl_buf,nullptr,
        TFT_WIDTH*10*sizeof(lv_color_t),LV_DISPLAY_RENDER_MODE_PARTIAL);

    esp_timer_handle_t lt;
    esp_timer_create_args_t la={.callback=[](void*){lv_tick_inc(2);},.arg=nullptr,.name="lv"};
    esp_timer_create(&la,&lt);
    esp_timer_start_periodic(lt,2000);

    // Touch input device
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);

    // Tileview   col,row
    tv = lv_tileview_create(nullptr);
    lv_obj_set_style_bg_color(tv, lv_color_hex(COL_BG), 0);
    lv_obj_set_scrollbar_mode(tv, LV_SCROLLBAR_MODE_OFF);
    tile_alarm    = lv_tileview_add_tile(tv, 1, 0, LV_DIR_BOTTOM);
    tile_sensors  = lv_tileview_add_tile(tv, 0, 1, LV_DIR_RIGHT);
    tile_home     = lv_tileview_add_tile(tv, 1, 1, LV_DIR_ALL);
    tile_relays   = lv_tileview_add_tile(tv, 2, 1, LV_DIR_LEFT);
    tile_settings = lv_tileview_add_tile(tv, 1, 2, LV_DIR_TOP);

    // Construir tiles y overlays
    build_tile_home();
    build_tile_sensors();
    build_tile_relays();
    build_tile_alarm();
    build_tile_settings();
    build_scr_alarm();
    build_scr_pin();
    build_scr_water();

    lv_tileview_set_tile(tv, tile_home, LV_ANIM_OFF);
    lv_scr_load(tv);
    for (int i=0;i<20;i++){lv_timer_handler();delay(5);}
    Serial.println("Pantalla lista.");

    WiFiManager wm;
    if (wm.autoConnect("DOMUS-Setup"))
        Serial.printf("WiFi: %s\n", WiFi.localIP().toString().c_str());

    broker.init(1883);
    Serial.printf("Broker en %s:1883\n", WiFi.localIP().toString().c_str());

    fetch_weather();
}

// ── LOOP ──────────────────────────────────────────────────────
static unsigned long last_status=0;

void loop() {
    broker.update();

    if (ui_needs_update) {
        check_alarms();
        update_home();
        update_sensors_tile();
        update_relays_tile();
        update_alarm_tile();
        ui_needs_update = false;
    }

    if (scr_change) { scr_change=false; do_switch(pend_scr); }

    // Refresco weather cada 30 min
    if (millis()-wx_last > 30UL*60000UL) fetch_weather();

    if (millis()-last_status>=5000) {
        last_status=millis();
        Serial.printf("[S] flood=%d pir=%d smoke=%d pwr=%d | agua=%d calef=%d sir=%d | arm=%d Wx:%.1fC\n",
            a6v3.input[1],a6v3.input[4],a6v3.input[5],a6v3.input[6],
            a6v3.output[1],a6v3.output[2],a6v3.output[3],
            alarm_armed,wx.temp);
    }

    lv_timer_handler();
    delay(5);
}
