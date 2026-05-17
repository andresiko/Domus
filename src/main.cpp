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
#include <Preferences.h>
#include <mbedtls/md.h>
#include <time.h>
#include "config.h"
#include <LittleFS.h>

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

// ── Paleta (BGR-swapped) ──────────────────────────────────────
#define COL_BG         0x0A0A0A
#define COL_CARD       0x1E1C1C
#define COL_TEXT       0xFFFFFF
#define COL_MUTED      0x938E8E
#define COL_OK         0x58D130
#define COL_OK_DIM     0x285010
#define COL_ALERT      0x303BFF
#define COL_RELAY_ON   0x0A9FFF
#define COL_RELAY_DIM  0x054F80
#define COL_OFF        0x2A2828
#define COL_ZONE_ALARM  0x18103A
#define COL_ZONE_SET    0x10200A
#define COL_BROKER_OK   0xFFCC44  // azul claro en pantalla (BGR)
#define ARC_WIDTH      72
#define COL_CHART_TEMP 0x4040FF   // rojo en pantalla (BGR)
#define COL_CHART_RAIN 0xFF4000   // azul en pantalla (BGR)

// ── NVS ──────────────────────────────────────────────────────
static Preferences prefs;
static int cfg_brightness = 200;
static int cfg_dim_delay  = 5;

// ── Calefacción ───────────────────────────────────────────────
enum HeatMode { HM_OFF, HM_MANUAL, HM_CONSIGNA };
static HeatMode heat_mode     = HM_OFF;
static int      heat_setpoint = 20;    // °C entero
static float    tuya_temp_int = NAN;
static float    tuya_humidity = NAN;

// ── Auto-dim ─────────────────────────────────────────────────
static unsigned long last_touch_ms = 0;
static bool          screen_dimmed = false;

// ── Encoder ──────────────────────────────────────────────────
static volatile int  enc_delta    = 0;
static int           enc_accum    = 0;
static bool          enc_btn_prev = false;
static unsigned long enc_btn_last_ms = 0;

void IRAM_ATTR enc_isr() {
    static uint8_t old_AB = 3;
    static const int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    old_AB = ((old_AB << 2) | (digitalRead(ENC_A) << 1) | digitalRead(ENC_B)) & 0x0F;
    enc_delta -= enc_states[old_AB];
}

// ── Alarma ───────────────────────────────────────────────────
enum AlarmState { AS_OFF, AS_ARMING, AS_ARMED, AS_GRACE, AS_SOUNDING };
static AlarmState alarm_state   = AS_OFF;
static unsigned long alarm_ts   = 0;
static unsigned long arming_end_ms = 0;   // millis() cuando termina la cuenta atrás
static bool grace_beeps[3]      = {};

// ── Estado A6v3 ──────────────────────────────────────────────
struct { bool input[7]={}; bool output[7]={}; } a6v3;
static bool ui_needs_update = false;
static bool alarm_armed     = false;
static bool intruder_active = false;
static bool critical_alert  = false;

// ── PIN ──────────────────────────────────────────────────────
static char cfg_pin[5] = "1234";
static char pin_buf[5] = {};
static int  pin_len    = 0;
static bool pin_for_arm = true;

// ── Sirena / beeps ───────────────────────────────────────────
static unsigned long siren_off_at = 0;
struct BeepSeq { int total=0,done=0,dur_ms=100,pause_ms=200; unsigned long next=0; };
static BeepSeq beep_seq;
static void start_beep_seq(int n, int dur=100, int pause=200) {
    beep_seq.total=n; beep_seq.done=0;
    beep_seq.dur_ms=dur; beep_seq.pause_ms=pause;
    beep_seq.next=millis();
}

// ── Tiempo / Open-Meteo ──────────────────────────────────────
struct {
    float temp=NAN; float wind=0; float rain_now=0; bool ok=false;
    float h_temp[12]={}; float h_rain[12]={};
} wx;
static unsigned long wx_last = 0;
#define WX_URL "https://api.open-meteo.com/v1/forecast" \
    "?latitude=42.80632457618044&longitude=-1.6290100870212767" \
    "&current=temperature_2m,wind_speed_10m,precipitation" \
    "&hourly=precipitation,wind_speed_10m,temperature_2m" \
    "&forecast_days=1&timezone=Europe%2FMadrid"

// ── Tuya API ─────────────────────────────────────────────────
#define TUYA_CLIENT_ID     "df33cuper4k8ce7qstn8"
#define TUYA_CLIENT_SECRET "c86757d118de4dcda2f1c143936dc7bd"
#define TUYA_DEVICE_ID     "bf080169b9cf3e36f2khp2"
#define TUYA_BASE_URL      "https://openapi.tuyaeu.com"
#define TUYA_SHA256_EMPTY  "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"

static String        tuya_token     = "";
static unsigned long tuya_token_exp = 0;
static unsigned long tuya_last      = 0;

// ── HISTORIAL ────────────────────────────────────────────────
#define HIST_EPOCH_BASE  1735689600UL   // 2025-01-01 00:00:00 UTC
#define HIST_CAPACITY    170000UL       // ~1 MB en /hist.bin  (flash 8MB)
#define TEMP_CAPACITY    580000UL       // ~3.4 MB en /temp.bin (flash 8MB)
#define TEMP_LOG_MS      (5UL*60000UL)
#define HSRC_MANUAL      1
#define HSRC_AUTO        2

enum HistType : uint8_t {
    HT_RELAY_ON=0, HT_RELAY_OFF,         // data=relay(1-6) flags=HSRC
    HT_SENSOR_ON,  HT_SENSOR_OFF,         // data=sensor(1-6)
    HT_HEAT_MODE,                          // data=HeatMode nuevo
    HT_HEAT_RELAY_ON, HT_HEAT_RELAY_OFF,  // data=HeatMode activo
    HT_ALARM_ARM,  HT_ALARM_DISARM,       // data=0
    HT_ALARM_TRIGGER, HT_ALARM_CANCEL,    // data=sensor o 0
};
struct __attribute__((packed)) HistEvent  { uint8_t type,data,ts[3],flags; };
struct __attribute__((packed)) TempSample { int8_t t_int,t_ext; uint8_t hum,ts[3]; };
static_assert(sizeof(HistEvent)==6,"");
static_assert(sizeof(TempSample)==6,"");

static uint32_t hist_head=0, hist_count=0;
static uint32_t temp_head=0, temp_count=0;
static unsigned long temp_log_last=0;
static unsigned long hist_ready_at=0;
static bool relay_manual_pending[7]={};
static File hist_file_rw, temp_file_rw;

static void hist_pack_ts(uint8_t *b, time_t t) {
    uint32_t m=(t>(time_t)HIST_EPOCH_BASE)?(uint32_t)((t-(time_t)HIST_EPOCH_BASE)/60):0;
    b[0]=(uint8_t)m; b[1]=(uint8_t)(m>>8); b[2]=(uint8_t)(m>>16);
}
static void hist_log(uint8_t type, uint8_t data, uint8_t flags=0) {
    if(!hist_file_rw || millis()<hist_ready_at) return;
    time_t now; time(&now);
    if(now<(time_t)HIST_EPOCH_BASE) return;
    HistEvent ev={type,data,{},flags};
    hist_pack_ts(ev.ts,now);
    hist_file_rw.seek(8+hist_head*(uint32_t)sizeof(HistEvent));
    hist_file_rw.write((const uint8_t*)&ev,sizeof(ev));
    hist_head=(hist_head+1)%HIST_CAPACITY;
    if(hist_count<HIST_CAPACITY) hist_count++;
    hist_file_rw.seek(0);
    hist_file_rw.write((const uint8_t*)&hist_head,4);
    hist_file_rw.write((const uint8_t*)&hist_count,4);
    hist_file_rw.flush();
}
static void temp_log_now() {
    if(!temp_file_rw||isnan(tuya_temp_int)) return;
    time_t now; time(&now);
    if(now<(time_t)HIST_EPOCH_BASE) return;
    TempSample s;
    s.t_int=(int8_t)constrain((int)(tuya_temp_int*2.0f),-128,127);
    s.t_ext=(int8_t)constrain((int)(wx.temp*2.0f),-128,127);
    s.hum  =(uint8_t)constrain((int)tuya_humidity,0,255);
    hist_pack_ts(s.ts,now);
    temp_file_rw.seek(8+temp_head*(uint32_t)sizeof(TempSample));
    temp_file_rw.write((const uint8_t*)&s,sizeof(s));
    temp_head=(temp_head+1)%TEMP_CAPACITY;
    if(temp_count<TEMP_CAPACITY) temp_count++;
    temp_file_rw.seek(0);
    temp_file_rw.write((const uint8_t*)&temp_head,4);
    temp_file_rw.write((const uint8_t*)&temp_count,4);
    temp_file_rw.flush();
    temp_log_last=millis();
}
static void hist_open_file(const char *path, File &fh, uint32_t &head, uint32_t &count, uint32_t cap) {
    if(LittleFS.exists(path)){
        File f=LittleFS.open(path,"r");
        f.read((uint8_t*)&head,4); f.read((uint8_t*)&count,4); f.close();
        if(head>=cap||count>cap){ head=0; count=0; }
        Serial.printf("%s: %u registros\n",path,count);
    } else {
        uint32_t z=0;
        File f=LittleFS.open(path,"w");
        f.write((const uint8_t*)&z,4); f.write((const uint8_t*)&z,4); f.close();
        Serial.printf("%s: archivo nuevo\n",path);
    }
    fh=LittleFS.open(path,"r+");
    if(!fh) Serial.printf("%s: error r+\n",path);
}
static void hist_init() {
    if(!LittleFS.begin(true)){ Serial.println("LittleFS: error"); return; }
    Serial.printf("LittleFS: %u KB / %u KB\n",
        (unsigned)(LittleFS.usedBytes()/1024),(unsigned)(LittleFS.totalBytes()/1024));
    hist_open_file("/hist.bin",hist_file_rw,hist_head,hist_count,HIST_CAPACITY);
    hist_open_file("/temp.bin",temp_file_rw,temp_head,temp_count,TEMP_CAPACITY);
}

// ── MQTT ─────────────────────────────────────────────────────
#define MQTT_STATE "A6v3/30EDA03B1378/STATE"
#define MQTT_SET   "A6v3/30EDA03B1378/SET"
static unsigned long mqtt_last_activity  = 0;
static time_t        mqtt_last_conn_epoch = 0;

class DomusBroker : public sMQTTBroker {
public:
    bool onEvent(sMQTTEvent *ev) override {
        mqtt_last_activity = millis();
        time_t now; time(&now); if(now > 1000000000L) mqtt_last_conn_epoch = now;
        if (ev->Type() == NewClient_sMQTTEventType)
            Serial.println("Broker: cliente conectado");
        else if (ev->Type() == Public_sMQTTEventType) {
            auto *p = (sMQTTPublicClientEvent*)ev;
            if (p->Topic() != MQTT_STATE) return true;
            JsonDocument doc;
            if (deserializeJson(doc, p->Payload().c_str(), p->Payload().size())) return true;
            for (int i=1;i<=6;i++) {
                char ki[10], ko[10];
                snprintf(ki,10,"input%d",i);
                snprintf(ko,10,"output%d",i);
                if (!doc[ki]["value"].isNull()) {
                    bool nv=doc[ki]["value"].as<bool>();
                    if(nv!=a6v3.input[i])
                        hist_log(nv?HT_SENSOR_ON:HT_SENSOR_OFF,(uint8_t)i);
                    a6v3.input[i]=nv;
                }
                if (!doc[ko]["value"].isNull()) {
                    bool nv=doc[ko]["value"].as<bool>();
                    if(nv!=a6v3.output[i]){
                        uint8_t src=relay_manual_pending[i]?HSRC_MANUAL:HSRC_AUTO;
                        relay_manual_pending[i]=false;
                        if(i==2) hist_log(nv?HT_HEAT_RELAY_ON:HT_HEAT_RELAY_OFF,(uint8_t)heat_mode,src);
                        else     hist_log(nv?HT_RELAY_ON:HT_RELAY_OFF,(uint8_t)i,src);
                    }
                    a6v3.output[i]=nv;
                }
            }
            ui_needs_update = true;
        }
        return true;
    }
} broker;

static void relay_set(int n, bool v) {
    if(n>=1&&n<=6) relay_manual_pending[n]=true;
    char buf[64];
    snprintf(buf,64,"{\"output%d\":{\"value\":%s}}",n,v?"true":"false");
    broker.publish(std::string(MQTT_SET), std::string(buf));
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
static void set_brightness(int val) {
    ledcWrite(0, (uint32_t)constrain(val, 10, 255));
}

// ── LVGL flush ───────────────────────────────────────────────
static void lvgl_flush_cb(lv_display_t *d, const lv_area_t *a, uint8_t *px) {
    display->draw16bitRGBBitmap(a->x1, a->y1, (uint16_t*)px,
        a->x2-a->x1+1, a->y2-a->y1+1);
    lv_display_flush_ready(d);
}

// ── Touch ────────────────────────────────────────────────────
static uint8_t touch_i2c_addr = 0;
static void touch_init() {
    uint8_t cands[] = {0x15, 0x38, 0x5D};
    for (uint8_t a : cands) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) { touch_i2c_addr = a; break; }
    }
}
static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    if (!touch_i2c_addr) { data->state = LV_INDEV_STATE_RELEASED; return; }
    Wire.beginTransmission(touch_i2c_addr); Wire.write(0x01);
    Wire.endTransmission(false);
    Wire.requestFrom(touch_i2c_addr, (uint8_t)6);
    if (Wire.available() >= 6) {
        Wire.read();
        uint8_t fingers = Wire.read();
        uint8_t xh=Wire.read(), xl=Wire.read(), yh=Wire.read(), yl=Wire.read();
        if (fingers > 0) {
            data->point.x = ((xh&0x0F)<<8)|xl;
            data->point.y = ((yh&0x0F)<<8)|yl;
            data->state   = LV_INDEV_STATE_PRESSED;
            last_touch_ms = millis();
            if (screen_dimmed) { set_brightness(cfg_brightness); screen_dimmed = false; }
        } else { data->state = LV_INDEV_STATE_RELEASED; }
    } else { while(Wire.available()) Wire.read(); data->state=LV_INDEV_STATE_RELEASED; }
}

// ── Widgets globales ─────────────────────────────────────────
static lv_obj_t *tv;
static lv_obj_t *tile_home, *tile_sensors, *tile_relays, *tile_alarm, *tile_settings;

// Home
static lv_obj_t *harc[5], *rarc[4], *arc_alarm_zone, *arc_settings_zone, *lbl_alarm_badge;
static lv_obj_t *lbl_sistema = nullptr, *lbl_temp_ext, *lbl_weather, *lbl_temp_int, *lbl_humidity;
static lv_obj_t *wx_chart = nullptr;
static lv_chart_series_t *chart_temp_ser = nullptr, *chart_rain_ser = nullptr;
static lv_obj_t *lbl_tmax=nullptr, *lbl_tmin=nullptr, *lbl_rmax=nullptr, *lbl_rmin=nullptr;

// Sensores
static lv_obj_t *sdot[5];

// Relés (nuevo layout)
static lv_obj_t *btn_agua_r=nullptr,   *lbl_agua_r=nullptr;
static lv_obj_t *btn_sirena_r=nullptr, *lbl_sirena_r=nullptr;
static lv_obj_t *btn_hm[3]={};         // MANUAL, CONSIGNA, PROGRAMA
static lv_obj_t *btn_setpoint=nullptr, *lbl_setpoint=nullptr;

// Alarma tile
static lv_obj_t *lbl_arm_state, *btn_arm;
// Settings
static lv_obj_t *lbl_settings_ip=nullptr, *lbl_cfg_brightness=nullptr, *lbl_cfg_dim=nullptr;
static lv_obj_t *lbl_broker_status=nullptr;
// Overlays
static lv_obj_t *scr_alarm=nullptr, *scr_pin=nullptr, *scr_arming=nullptr, *scr_dial=nullptr, *scr_change_pin=nullptr;
static lv_obj_t *lbl_chpin_title=nullptr, *lbl_chpin_dots=nullptr, *lbl_chpin_msg=nullptr;
static char chpin_buf[5]={};
static int  chpin_len=0, chpin_phase=0;
static char chpin_new[5]={};
static lv_obj_t *lbl_alarm_type, *lbl_alarm_detail, *btn_deactivate;
static lv_obj_t *lbl_pin_dots, *lbl_pin_msg;
static lv_obj_t *lbl_countdown;
static lv_obj_t *dial_arc=nullptr, *lbl_dial_title_w=nullptr;
static lv_obj_t *lbl_dial_val=nullptr, *lbl_dial_unit=nullptr;

enum DialMode { DIAL_NONE, DIAL_BRIGHTNESS, DIAL_DIM, DIAL_SETPOINT };
static DialMode dial_mode = DIAL_NONE;
static int      dial_val  = 0;

// ── Navegación ───────────────────────────────────────────────
enum Screen { SCR_TV, SCR_ALARM_CRIT, SCR_PIN, SCR_ARMING_SCR, SCR_DIAL, SCR_CHANGE_PIN };
static Screen cur_scr=SCR_TV, pend_scr=SCR_TV;
static bool   scr_change=false;
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

static lv_obj_t *make_sector_arc(lv_obj_t *parent, int16_t start, int16_t end,
                                  uint32_t col, bool clickable=false) {
    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_set_size(arc, 480, 480);
    lv_obj_center(arc);
    lv_obj_set_style_pad_all(arc, 0, 0);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(arc, 0, 0);
    lv_obj_set_style_arc_width(arc, ARC_WIDTH, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(col), LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 0, LV_PART_INDICATOR);
    lv_obj_set_style_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_pad_all(arc, 0, LV_PART_KNOB);
    lv_arc_set_bg_angles(arc, start, end);
    lv_arc_set_value(arc, 0);
    if (!clickable) lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(arc, LV_OBJ_FLAG_EVENT_BUBBLE);
    return arc;
}

static void sector_label(lv_obj_t *parent, const char *txt, int cx, int cy) {
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_width(l, 50);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
    lv_obj_set_pos(l, cx-25, cy-7);
}

static void add_home_hint(lv_obj_t *tile, lv_align_t align, const char *txt) {
    lv_obj_t *l = lv_label_create(tile);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0x444444), 0);
    lv_obj_align(l, align, 0, 0);
}

// ── DIAL ─────────────────────────────────────────────────────
static void update_dial_display() {
    if (!lbl_dial_val) return;
    char buf[16];
    if (dial_mode == DIAL_BRIGHTNESS)
        snprintf(buf, sizeof(buf), "%d", dial_val);
    else if (dial_mode == DIAL_DIM)
        snprintf(buf, sizeof(buf), dial_val==0 ? "OFF" : "%d", dial_val);
    else // SETPOINT
        snprintf(buf, sizeof(buf), "%d\xc2\xb0""C", dial_val);
    lv_label_set_text(lbl_dial_val, buf);
    if (dial_mode == DIAL_BRIGHTNESS) set_brightness(dial_val);
}

static void update_setpoint_btn() {
    if (!lbl_setpoint) return;
    char buf[10]; snprintf(buf, sizeof(buf), "%d\xc2\xb0""C", heat_setpoint);
    lv_label_set_text(lbl_setpoint, buf);
}

static void apply_dial() {
    prefs.begin("domus", false);
    if (dial_mode == DIAL_BRIGHTNESS) {
        cfg_brightness = dial_val;
        set_brightness(cfg_brightness);
        prefs.putInt("brightness", cfg_brightness);
        if (lbl_cfg_brightness) {
            char b[24]; snprintf(b,sizeof(b),"Brillo: %d", cfg_brightness);
            lv_label_set_text(lbl_cfg_brightness, b);
        }
    } else if (dial_mode == DIAL_DIM) {
        cfg_dim_delay = dial_val;
        prefs.putInt("dim_delay", cfg_dim_delay);
        if (lbl_cfg_dim) {
            char b[24];
            snprintf(b,sizeof(b), cfg_dim_delay==0 ? "Dim: OFF" : "Dim: %d min", cfg_dim_delay);
            lv_label_set_text(lbl_cfg_dim, b);
        }
    } else if (dial_mode == DIAL_SETPOINT) {
        heat_setpoint = dial_val;
        prefs.putInt("heat_sp", heat_setpoint);
        update_setpoint_btn();
    }
    prefs.end();
}

static void cb_dial_arc_changed(lv_event_t *e) {
    if (!dial_arc) return;
    dial_val = lv_arc_get_value(dial_arc);
    update_dial_display();
}
static void cb_dial_ok(lv_event_t *e)     { apply_dial(); dial_mode=DIAL_NONE; go_to(SCR_TV); }
static void cb_dial_cancel(lv_event_t *e) {
    if (dial_mode==DIAL_BRIGHTNESS) set_brightness(cfg_brightness);
    dial_mode=DIAL_NONE; go_to(SCR_TV);
}

static void open_dial(DialMode mode) {
    dial_mode = mode;
    if (mode == DIAL_BRIGHTNESS) {
        dial_val=cfg_brightness;
        lv_label_set_text(lbl_dial_title_w, "BRILLO");
        lv_label_set_text(lbl_dial_unit, "intensidad");
        lv_arc_set_range(dial_arc, 10, 255);
    } else if (mode == DIAL_DIM) {
        dial_val=cfg_dim_delay;
        lv_label_set_text(lbl_dial_title_w, "APAGADO AUTO");
        lv_label_set_text(lbl_dial_unit, "minutos (0=nunca)");
        lv_arc_set_range(dial_arc, 0, 30);
    } else {
        dial_val=heat_setpoint;
        lv_label_set_text(lbl_dial_title_w, "CONSIGNA");
        lv_label_set_text(lbl_dial_unit, "grados C");
        lv_arc_set_range(dial_arc, 15, 30);
    }
    lv_arc_set_value(dial_arc, dial_val);
    update_dial_display();
    go_to(SCR_DIAL);
}

// ── CALEFACCIÓN: lógica ───────────────────────────────────────
static void refresh_heat_ui();  // forward decl

static void heat_set_mode(HeatMode m) {
    if (heat_mode == m) {
        heat_mode = HM_OFF;
        relay_set(2, false);
    } else {
        heat_mode = m;
        if (m == HM_MANUAL) relay_set(2, true);
        // CONSIGNA: relay lo gestiona check_heating_auto()
    }
    hist_log(HT_HEAT_MODE,(uint8_t)heat_mode);
    prefs.begin("domus",false); prefs.putInt("heat_mode",(int)heat_mode); prefs.end();
    refresh_heat_ui();
}

static void check_heating_auto() {
    if (heat_mode != HM_CONSIGNA) return;
    if (isnan(tuya_temp_int)) return;
    const float hyst = 0.5f;
    if (tuya_temp_int < heat_setpoint - hyst && !a6v3.output[2]) relay_set(2, true);
    else if (tuya_temp_int > heat_setpoint + hyst && a6v3.output[2])  relay_set(2, false);
}

// ── BUILD TILE HOME ──────────────────────────────────────────
static void cb_alarm_badge(lv_event_t *e)   { lv_tileview_set_tile(tv, tile_alarm,    LV_ANIM_ON); }
static void cb_settings_icon(lv_event_t *e) { lv_tileview_set_tile(tv, tile_settings, LV_ANIM_ON); }

static void build_tile_home() {
    bg(tile_home);
    harc[1]=make_sector_arc(tile_home,116,139,COL_OK_DIM);
    harc[2]=make_sector_arc(tile_home,151,174,COL_OK_DIM);
    harc[3]=make_sector_arc(tile_home,186,209,COL_OK_DIM);
    harc[4]=make_sector_arc(tile_home,221,244,COL_OK_DIM);
    rarc[1]=make_sector_arc(tile_home,296,331,COL_RELAY_DIM);
    rarc[2]=make_sector_arc(tile_home,343, 18,COL_RELAY_DIM);
    rarc[3]=make_sector_arc(tile_home, 30, 65,COL_RELAY_DIM);
    arc_alarm_zone  =make_sector_arc(tile_home,253,287,COL_ZONE_ALARM);
    arc_settings_zone=make_sector_arc(tile_home,73,107,COL_ZONE_SET);

    sector_label(tile_home,"INUND",121,395);
    sector_label(tile_home,"PIR",   54,299);
    sector_label(tile_home,"HUMO",  54,181);
    sector_label(tile_home,"220V", 121, 85);
    sector_label(tile_home,"AGUA", 374, 99);
    sector_label(tile_home,"CALEF",435,242);
    sector_label(tile_home,"SIR",  372,384);

    lbl_alarm_badge=lv_label_create(tile_home);
    lv_label_set_text(lbl_alarm_badge,"ALARMA\nOFF");
    lv_label_set_long_mode(lbl_alarm_badge,LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(lbl_alarm_badge,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_alarm_badge,lv_color_hex(COL_MUTED),0);
    lv_obj_set_width(lbl_alarm_badge,62);
    lv_obj_set_style_text_align(lbl_alarm_badge,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_pos(lbl_alarm_badge,209,22);
    lv_obj_add_flag(lbl_alarm_badge,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lbl_alarm_badge,cb_alarm_badge,LV_EVENT_CLICKED,nullptr);

    lbl_sistema=lv_label_create(tile_home);
    lv_label_set_text(lbl_sistema,"Sistema\nOFF");
    lv_label_set_long_mode(lbl_sistema,LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(lbl_sistema,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_sistema,lv_color_hex(COL_ALERT),0);
    lv_obj_set_width(lbl_sistema,70);
    lv_obj_set_style_text_align(lbl_sistema,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_pos(lbl_sistema,205,426);
    lv_obj_add_flag(lbl_sistema,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lbl_sistema,cb_settings_icon,LV_EVENT_CLICKED,nullptr);

    lv_obj_t *circle=lv_obj_create(tile_home);
    lv_obj_set_size(circle,320,320); lv_obj_center(circle);
    lv_obj_set_style_radius(circle,160,0);
    lv_obj_set_style_bg_color(circle,lv_color_hex(COL_BG),0);
    lv_obj_set_style_border_width(circle,1,0);
    lv_obj_set_style_border_color(circle,lv_color_hex(0x222222),0);
    lv_obj_set_style_pad_all(circle,0,0);
    lv_obj_clear_flag(circle,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(circle,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(circle,LV_OBJ_FLAG_EVENT_BUBBLE);

    lbl_temp_ext=centered_label(tile_home,"--.-°C",&lv_font_montserrat_40,COL_TEXT,-120);
    lbl_weather =centered_label(tile_home,"-- km/h  0.0 mm/h",&lv_font_montserrat_14,COL_MUTED,-78);

    wx_chart=lv_chart_create(tile_home);
    lv_obj_set_size(wx_chart,200,50);
    lv_obj_align(wx_chart,LV_ALIGN_CENTER,0,-22);
    lv_chart_set_type(wx_chart,LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(wx_chart,12);
    lv_chart_set_range(wx_chart,LV_CHART_AXIS_PRIMARY_Y,0,30);
    lv_chart_set_range(wx_chart,LV_CHART_AXIS_SECONDARY_Y,0,10);
    lv_obj_set_style_bg_opa(wx_chart,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_opa(wx_chart,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(wx_chart,0,0);
    lv_obj_set_style_line_opa(wx_chart,LV_OPA_20,LV_PART_MAIN);
    lv_obj_set_style_size(wx_chart,0,0,LV_PART_INDICATOR);
    lv_obj_set_style_line_width(wx_chart,2,LV_PART_ITEMS);
    lv_obj_clear_flag(wx_chart,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(wx_chart,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(wx_chart,LV_OBJ_FLAG_EVENT_BUBBLE);
    chart_temp_ser=lv_chart_add_series(wx_chart,lv_color_hex(COL_CHART_TEMP),LV_CHART_AXIS_PRIMARY_Y);
    chart_rain_ser=lv_chart_add_series(wx_chart,lv_color_hex(COL_CHART_RAIN),LV_CHART_AXIS_SECONDARY_Y);
    lv_chart_set_all_value(wx_chart,chart_temp_ser,15);
    lv_chart_set_all_value(wx_chart,chart_rain_ser,0);

    // Etiquetas min/max
    lbl_tmax=lv_label_create(tile_home); lv_label_set_text(lbl_tmax,"--");
    lv_obj_set_style_text_font(lbl_tmax,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_tmax,lv_color_hex(COL_CHART_TEMP),0);
    lv_obj_set_style_text_align(lbl_tmax,LV_TEXT_ALIGN_RIGHT,0);
    lv_obj_set_width(lbl_tmax,28); lv_obj_set_pos(lbl_tmax,108,204);

    lbl_tmin=lv_label_create(tile_home); lv_label_set_text(lbl_tmin,"--");
    lv_obj_set_style_text_font(lbl_tmin,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_tmin,lv_color_hex(COL_CHART_TEMP),0);
    lv_obj_set_style_text_align(lbl_tmin,LV_TEXT_ALIGN_RIGHT,0);
    lv_obj_set_width(lbl_tmin,28); lv_obj_set_pos(lbl_tmin,108,228);

    lbl_rmax=lv_label_create(tile_home); lv_label_set_text(lbl_rmax,"--");
    lv_obj_set_style_text_font(lbl_rmax,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_rmax,lv_color_hex(COL_CHART_RAIN),0);
    lv_obj_set_width(lbl_rmax,28); lv_obj_set_pos(lbl_rmax,344,204);

    lbl_rmin=lv_label_create(tile_home); lv_label_set_text(lbl_rmin,"--");
    lv_obj_set_style_text_font(lbl_rmin,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_rmin,lv_color_hex(COL_CHART_RAIN),0);
    lv_obj_set_width(lbl_rmin,28); lv_obj_set_pos(lbl_rmin,344,228);

    // Marcadores hora X
    const int cx0=140,step=18,ly=262;
    const char *hlbl[4]={"0h","3h","6h","9h"};
    const int hidx[4]={0,3,6,9};
    for(int i=0;i<4;i++){
        lv_obj_t *l=lv_label_create(tile_home);
        lv_label_set_text(l,hlbl[i]);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(l,lv_color_hex(0x555555),0);
        lv_obj_set_pos(l,cx0+hidx[i]*step-8,ly);
    }

    lbl_temp_int=centered_label(tile_home,"--.-°C",&lv_font_montserrat_48,COL_TEXT,88);
    lbl_humidity=centered_label(tile_home,"--% HR",&lv_font_montserrat_14,COL_MUTED,128);
}

// ── BUILD TILE SENSORES ──────────────────────────────────────
static const char *S_NAME[5]={"","Inundacion","Movimiento","Humo","Red 220V"};
static void build_tile_sensors() {
    bg(tile_sensors);
    centered_label(tile_sensors,"SENSORES",&lv_font_montserrat_20,COL_TEXT,-175);
    for(int i=1;i<=4;i++){
        int y=-100+(i-1)*60;
        lv_obj_t *card=lv_obj_create(tile_sensors);
        lv_obj_set_size(card,300,44);
        lv_obj_align(card,LV_ALIGN_CENTER,0,y);
        lv_obj_set_style_bg_color(card,lv_color_hex(COL_CARD),0);
        lv_obj_set_style_border_width(card,0,0);
        lv_obj_set_style_radius(card,10,0);
        lv_obj_set_style_pad_all(card,0,0);
        lv_obj_clear_flag(card,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *lbl=lv_label_create(card);
        lv_label_set_text(lbl,S_NAME[i]);
        lv_obj_set_style_text_font(lbl,&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(lbl,lv_color_hex(COL_MUTED),0);
        lv_obj_align(lbl,LV_ALIGN_LEFT_MID,14,0);
        sdot[i]=make_dot_at(card,272,15);
    }
    add_home_hint(tile_sensors,LV_ALIGN_RIGHT_MID,LV_SYMBOL_RIGHT " HOME");
}

// ── BUILD TILE RELÉS (nuevo layout) ──────────────────────────
static void cb_agua(lv_event_t *e)   { relay_set(1,!a6v3.output[1]); }
static void cb_sirena(lv_event_t *e) { relay_set(3,!a6v3.output[3]); }
static void cb_hm_manual(lv_event_t *e)   { heat_set_mode(HM_MANUAL); }
static void cb_hm_consigna(lv_event_t *e) { heat_set_mode(HM_CONSIGNA); }
static void cb_hm_prog(lv_event_t *e)     { /* Fase B */ }
static void cb_setpoint_btn(lv_event_t *e) {
    if(heat_mode != HM_CONSIGNA) {
        heat_mode = HM_CONSIGNA;
        hist_log(HT_HEAT_MODE,(uint8_t)HM_CONSIGNA);
        prefs.begin("domus",false); prefs.putInt("heat_mode",(int)heat_mode); prefs.end();
        refresh_heat_ui();
    }
    open_dial(DIAL_SETPOINT);
}

static lv_obj_t *make_mode_btn(lv_obj_t *parent, const char *txt,
                                int x, int y, lv_event_cb_t cb) {
    lv_obj_t *b=lv_obj_create(parent);
    lv_obj_set_size(b,82,42);
    lv_obj_align(b,LV_ALIGN_CENTER,x,y);
    lv_obj_set_style_bg_color(b,lv_color_hex(COL_OFF),0);
    lv_obj_set_style_radius(b,8,0);
    lv_obj_set_style_border_width(b,0,0);
    lv_obj_set_style_pad_all(b,0,0);
    lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(b,LV_OBJ_FLAG_CLICKABLE);
    if(cb) lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,nullptr);
    lv_obj_t *l=lv_label_create(b);
    lv_label_set_text(l,txt);
    lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
    lv_obj_center(l);
    return b;
}

static void refresh_heat_ui() {
    if(!btn_hm[0]) return;
    lv_obj_set_style_bg_color(btn_hm[0],lv_color_hex(heat_mode==HM_MANUAL?COL_OK:COL_OFF),0);
    lv_obj_set_style_bg_color(btn_hm[1],lv_color_hex(heat_mode==HM_CONSIGNA?COL_OK:COL_OFF),0);
    lv_obj_set_style_bg_color(btn_hm[2],lv_color_hex(COL_OFF),0);
}

static void build_tile_relays() {
    bg(tile_relays);

    // AGUA — arriba
    btn_agua_r=lv_obj_create(tile_relays);
    lv_obj_set_size(btn_agua_r,260,65);
    lv_obj_align(btn_agua_r,LV_ALIGN_CENTER,0,-140);
    lv_obj_set_style_bg_color(btn_agua_r,lv_color_hex(COL_OFF),0);
    lv_obj_set_style_radius(btn_agua_r,32,0);
    lv_obj_set_style_border_width(btn_agua_r,0,0);
    lv_obj_set_style_pad_all(btn_agua_r,0,0);
    lv_obj_clear_flag(btn_agua_r,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_agua_r,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_agua_r,cb_agua,LV_EVENT_CLICKED,nullptr);
    lbl_agua_r=lv_label_create(btn_agua_r);
    lv_label_set_text(lbl_agua_r,"AGUA");
    lv_obj_set_style_text_font(lbl_agua_r,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_color(lbl_agua_r,lv_color_hex(COL_TEXT),0);
    lv_obj_center(lbl_agua_r);

    // CALEFACCIÓN — centro (recuadro con 3 filas)
    lv_obj_t *heat_box=lv_obj_create(tile_relays);
    lv_obj_set_size(heat_box,280,200);
    lv_obj_align(heat_box,LV_ALIGN_CENTER,0,0);
    lv_obj_set_style_bg_color(heat_box,lv_color_hex(COL_BG),0);
    lv_obj_set_style_bg_opa(heat_box,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(heat_box,lv_color_hex(0x303030),0);
    lv_obj_set_style_border_width(heat_box,1,0);
    lv_obj_set_style_radius(heat_box,16,0);
    lv_obj_set_style_pad_all(heat_box,0,0);
    lv_obj_clear_flag(heat_box,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(heat_box,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(heat_box,LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *lbl_htitle=lv_label_create(heat_box);
    lv_label_set_text(lbl_htitle,"CALEFACCION");
    lv_obj_set_style_text_font(lbl_htitle,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_htitle,lv_color_hex(COL_MUTED),0);
    lv_obj_align(lbl_htitle,LV_ALIGN_TOP_MID,0,7);

    // Fila 1: MANUAL
    btn_hm[0]=lv_obj_create(heat_box);
    lv_obj_set_size(btn_hm[0],260,44);
    lv_obj_align(btn_hm[0],LV_ALIGN_CENTER,0,-50);
    lv_obj_set_style_bg_color(btn_hm[0],lv_color_hex(COL_OFF),0);
    lv_obj_set_style_radius(btn_hm[0],8,0);
    lv_obj_set_style_border_width(btn_hm[0],0,0);
    lv_obj_set_style_pad_all(btn_hm[0],0,0);
    lv_obj_clear_flag(btn_hm[0],LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_hm[0],LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_hm[0],cb_hm_manual,LV_EVENT_CLICKED,nullptr);
    { lv_obj_t *l=lv_label_create(btn_hm[0]);
      lv_label_set_text(l,"MANUAL");
      lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
      lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
      lv_obj_center(l); }

    // Fila 2: CONSIGNA + botón temperatura (siempre visible)
    btn_hm[1]=lv_obj_create(heat_box);
    lv_obj_set_size(btn_hm[1],170,44);
    lv_obj_align(btn_hm[1],LV_ALIGN_CENTER,-44,0);
    lv_obj_set_style_bg_color(btn_hm[1],lv_color_hex(COL_OFF),0);
    lv_obj_set_style_radius(btn_hm[1],8,0);
    lv_obj_set_style_border_width(btn_hm[1],0,0);
    lv_obj_set_style_pad_all(btn_hm[1],0,0);
    lv_obj_clear_flag(btn_hm[1],LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_hm[1],LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_hm[1],cb_hm_consigna,LV_EVENT_CLICKED,nullptr);
    { lv_obj_t *l=lv_label_create(btn_hm[1]);
      lv_label_set_text(l,"CONSIGNA");
      lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
      lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
      lv_obj_center(l); }

    btn_setpoint=lv_obj_create(heat_box);
    lv_obj_set_size(btn_setpoint,82,44);
    lv_obj_align(btn_setpoint,LV_ALIGN_CENTER,89,0);
    lv_obj_set_style_bg_color(btn_setpoint,lv_color_hex(COL_CARD),0);
    lv_obj_set_style_radius(btn_setpoint,8,0);
    lv_obj_set_style_border_width(btn_setpoint,1,0);
    lv_obj_set_style_border_color(btn_setpoint,lv_color_hex(0x444444),0);
    lv_obj_set_style_pad_all(btn_setpoint,0,0);
    lv_obj_clear_flag(btn_setpoint,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_setpoint,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_setpoint,cb_setpoint_btn,LV_EVENT_CLICKED,nullptr);
    lbl_setpoint=lv_label_create(btn_setpoint);
    lv_obj_set_style_text_font(lbl_setpoint,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_color(lbl_setpoint,lv_color_hex(COL_TEXT),0);
    lv_obj_center(lbl_setpoint);
    update_setpoint_btn();

    // Fila 3: PROGRAMA
    btn_hm[2]=lv_obj_create(heat_box);
    lv_obj_set_size(btn_hm[2],260,44);
    lv_obj_align(btn_hm[2],LV_ALIGN_CENTER,0,50);
    lv_obj_set_style_bg_color(btn_hm[2],lv_color_hex(COL_OFF),0);
    lv_obj_set_style_radius(btn_hm[2],8,0);
    lv_obj_set_style_border_width(btn_hm[2],0,0);
    lv_obj_set_style_pad_all(btn_hm[2],0,0);
    lv_obj_clear_flag(btn_hm[2],LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_hm[2],LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_hm[2],cb_hm_prog,LV_EVENT_CLICKED,nullptr);
    { lv_obj_t *l=lv_label_create(btn_hm[2]);
      lv_label_set_text(l,"PROGRAMA");
      lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
      lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
      lv_obj_center(l); }

    // SIRENA — abajo
    btn_sirena_r=lv_obj_create(tile_relays);
    lv_obj_set_size(btn_sirena_r,260,65);
    lv_obj_align(btn_sirena_r,LV_ALIGN_CENTER,0,140);
    lv_obj_set_style_bg_color(btn_sirena_r,lv_color_hex(COL_OFF),0);
    lv_obj_set_style_radius(btn_sirena_r,32,0);
    lv_obj_set_style_border_width(btn_sirena_r,0,0);
    lv_obj_set_style_pad_all(btn_sirena_r,0,0);
    lv_obj_clear_flag(btn_sirena_r,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_sirena_r,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_sirena_r,cb_sirena,LV_EVENT_CLICKED,nullptr);
    lbl_sirena_r=lv_label_create(btn_sirena_r);
    lv_label_set_text(lbl_sirena_r,"SIRENA");
    lv_obj_set_style_text_font(lbl_sirena_r,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_color(lbl_sirena_r,lv_color_hex(COL_TEXT),0);
    lv_obj_center(lbl_sirena_r);

    refresh_heat_ui();
    add_home_hint(tile_relays,LV_ALIGN_LEFT_MID,"HOME " LV_SYMBOL_LEFT);
}

// ── BUILD TILE ALARMA ────────────────────────────────────────
static void cb_go_pin(lv_event_t *e) {
    pin_for_arm=(alarm_state==AS_OFF); memset(pin_buf,0,5); pin_len=0; go_to(SCR_PIN);
}
static void build_tile_alarm() {
    bg(tile_alarm);
    centered_label(tile_alarm,"ALARMA INTRUSION",&lv_font_montserrat_20,COL_TEXT,-160);
    lbl_arm_state=centered_label(tile_alarm,"DESARMADA",&lv_font_montserrat_40,COL_MUTED,-60);
    btn_arm=make_big_btn(tile_alarm,"ARMAR",COL_OFF,60,240,70,cb_go_pin,nullptr);
    add_home_hint(tile_alarm,LV_ALIGN_BOTTOM_MID,"HOME " LV_SYMBOL_DOWN);
}

// ── BUILD OVERLAY CAMBIO PIN ─────────────────────────────────
static void update_chpin_dots() {
    if(!lbl_chpin_dots) return;
    char b[9]="_ _ _ _";
    for(int i=0;i<chpin_len&&i<4;i++) b[i*2]='*';
    lv_label_set_text(lbl_chpin_dots,b);
}
static void chpin_check() {
    if(chpin_phase==0){
        if(strncmp(chpin_buf,cfg_pin,4)==0){
            chpin_phase=1; memset(chpin_buf,0,5); chpin_len=0;
            lv_label_set_text(lbl_chpin_title,"PIN nuevo:");
            lv_label_set_text(lbl_chpin_msg,""); update_chpin_dots();
        } else { lv_label_set_text(lbl_chpin_msg,"PIN incorrecto"); memset(chpin_buf,0,5); chpin_len=0; update_chpin_dots(); }
    } else if(chpin_phase==1){
        memcpy(chpin_new,chpin_buf,5); chpin_phase=2; memset(chpin_buf,0,5); chpin_len=0;
        lv_label_set_text(lbl_chpin_title,"Confirmar:"); lv_label_set_text(lbl_chpin_msg,""); update_chpin_dots();
    } else {
        if(strncmp(chpin_buf,chpin_new,4)==0){
            memcpy(cfg_pin,chpin_new,5);
            prefs.begin("domus",false); prefs.putString("pin",cfg_pin); prefs.end();
            lv_label_set_text(lbl_chpin_msg,"PIN cambiado!");
            go_to(SCR_TV);
        } else { lv_label_set_text(lbl_chpin_msg,"No coincide"); memset(chpin_buf,0,5); chpin_len=0; update_chpin_dots(); }
    }
}
static void cb_chpin_key(lv_event_t *e) {
    int d=(int)(intptr_t)lv_event_get_user_data(e);
    if(d==-2){ memset(chpin_buf,0,5); chpin_len=0; chpin_phase=0; go_to(SCR_TV); }
    else if(d==-1){ if(chpin_len>0){chpin_len--;chpin_buf[chpin_len]=0;} update_chpin_dots(); }
    else if(chpin_len<4){ chpin_buf[chpin_len++]='0'+d; update_chpin_dots(); if(chpin_len==4) chpin_check(); }
}
static void cb_open_change_pin(lv_event_t *e){ go_to(SCR_CHANGE_PIN); }
static void build_scr_change_pin() {
    scr_change_pin=lv_obj_create(nullptr); bg(scr_change_pin);
    lbl_chpin_title=centered_label(scr_change_pin,"PIN actual:",&lv_font_montserrat_20,COL_TEXT,-185);
    lbl_chpin_dots =centered_label(scr_change_pin,"_ _ _ _",&lv_font_montserrat_40,COL_TEXT,-150);
    lbl_chpin_msg  =centered_label(scr_change_pin,"",&lv_font_montserrat_14,COL_ALERT,-110);
    const int W=128,H=56,G=6;
    const int sx=-(3*W+2*G)/2+W/2, sy=-78;
    const int    digits[12]={1,2,3,4,5,6,7,8,9,-1,0,-2};
    const char  *lbs[12]   ={"1","2","3","4","5","6","7","8","9","DEL","0","CANCEL"};
    const uint32_t cols[12]={COL_CARD,COL_CARD,COL_CARD,COL_CARD,COL_CARD,COL_CARD,
                             COL_CARD,COL_CARD,COL_CARD,COL_ALERT,COL_CARD,COL_OFF};
    for(int i=0;i<12;i++){
        lv_obj_t *b=lv_obj_create(scr_change_pin);
        lv_obj_set_size(b,W,H);
        lv_obj_align(b,LV_ALIGN_CENTER,sx+(i%3)*(W+G),sy+(i/3)*(H+G));
        lv_obj_set_style_bg_color(b,lv_color_hex(cols[i]),0);
        lv_obj_set_style_radius(b,12,0); lv_obj_set_style_border_width(b,0,0);
        lv_obj_set_style_pad_all(b,0,0); lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(b,LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(b,cb_chpin_key,LV_EVENT_CLICKED,(void*)(intptr_t)digits[i]);
        lv_obj_t *l=lv_label_create(b);
        lv_label_set_text(l,lbs[i]);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
        lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
        lv_obj_center(l);
    }
}

// ── BUILD TILE SETTINGS ──────────────────────────────────────
static void cb_open_brightness(lv_event_t *e){ open_dial(DIAL_BRIGHTNESS); }
static void cb_open_dim(lv_event_t *e)       { open_dial(DIAL_DIM); }
static void build_tile_settings() {
    bg(tile_settings);
    centered_label(tile_settings,"AJUSTES",&lv_font_montserrat_20,COL_TEXT,-170);
    lbl_settings_ip=centered_label(tile_settings,"IP: ---",&lv_font_montserrat_14,COL_MUTED,-130);
    centered_label(tile_settings,"DOMUS v0.5",&lv_font_montserrat_14,COL_MUTED,-108);
    lbl_broker_status=centered_label(tile_settings,"Broker: sin conexion",&lv_font_montserrat_14,COL_ALERT,-84);

    lv_obj_t *btn_br=make_big_btn(tile_settings,"",COL_OFF,-30,260,56,cb_open_brightness,nullptr);
    lbl_cfg_brightness=lv_label_create(btn_br);
    char b1[24]; snprintf(b1,sizeof(b1),"Brillo: %d",cfg_brightness);
    lv_label_set_text(lbl_cfg_brightness,b1);
    lv_obj_set_style_text_font(lbl_cfg_brightness,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_color(lbl_cfg_brightness,lv_color_hex(COL_TEXT),0);
    lv_obj_center(lbl_cfg_brightness);

    lv_obj_t *btn_dim=make_big_btn(tile_settings,"",COL_OFF,50,260,56,cb_open_dim,nullptr);
    lbl_cfg_dim=lv_label_create(btn_dim);
    char b2[24];
    snprintf(b2,sizeof(b2),cfg_dim_delay==0?"Dim: OFF":"Dim: %d min",cfg_dim_delay);
    lv_label_set_text(lbl_cfg_dim,b2);
    lv_obj_set_style_text_font(lbl_cfg_dim,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_color(lbl_cfg_dim,lv_color_hex(COL_TEXT),0);
    lv_obj_center(lbl_cfg_dim);

    make_big_btn(tile_settings,"Cambiar PIN",COL_OFF,130,260,44,cb_open_change_pin,nullptr);

    add_home_hint(tile_settings,LV_ALIGN_TOP_MID,LV_SYMBOL_UP " HOME");
}

// ── BUILD OVERLAY ALARMA CRÍTICA ─────────────────────────────
static void cb_deactivate(lv_event_t *e) {
    if(intruder_active){ pin_for_arm=false; memset(pin_buf,0,5); pin_len=0; go_to(SCR_PIN); }
    else { relay_set(3,false); critical_alert=false; alarm_state=alarm_armed?AS_ARMED:AS_OFF; go_to(SCR_TV); }
}
static void build_scr_alarm() {
    scr_alarm=lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_alarm,lv_color_hex(0x1A0000),0);
    lv_obj_set_style_bg_opa(scr_alarm,LV_OPA_COVER,0);
    lbl_alarm_type  =centered_label(scr_alarm,"! ALARMA !",&lv_font_montserrat_40,COL_ALERT,-80);
    lbl_alarm_detail=centered_label(scr_alarm,"",&lv_font_montserrat_20,COL_TEXT,-20);
    btn_deactivate  =make_big_btn(scr_alarm,"DESACTIVAR",COL_ALERT,70,260,80,cb_deactivate,nullptr);
}

// ── BUILD OVERLAY PIN ─────────────────────────────────────────
static void update_pin_dots() {
    char b[9]="_ _ _ _";
    for(int i=0;i<pin_len&&i<4;i++) b[i*2]='*';
    lv_label_set_text(lbl_pin_dots,b);
}
static void pin_check() {
    if(strncmp(pin_buf,cfg_pin,4)==0){
        if(pin_for_arm){ hist_log(HT_ALARM_ARM,0); alarm_state=AS_ARMING; alarm_armed=true; alarm_ts=millis(); arming_end_ms=alarm_ts+120000UL; start_beep_seq(1,100); lv_label_set_text(lbl_pin_msg,""); go_to(SCR_ARMING_SCR); }
        else { hist_log(HT_ALARM_DISARM,0); alarm_state=AS_OFF; alarm_armed=false; intruder_active=false; critical_alert=false; beep_seq={}; siren_off_at=0; relay_set(3,false); lv_label_set_text(lbl_pin_msg,""); go_to(SCR_TV); }
    } else { lv_label_set_text(lbl_pin_msg,"PIN incorrecto"); memset(pin_buf,0,5); pin_len=0; update_pin_dots(); }
}
static void cb_pin_key(lv_event_t *e) {
    int d=(int)(intptr_t)lv_event_get_user_data(e);
    if(d==-2){ memset(pin_buf,0,5); pin_len=0; go_to(SCR_TV); }
    else if(d==-1){ if(pin_len>0){pin_len--;pin_buf[pin_len]=0;} update_pin_dots(); }
    else if(pin_len<4){ pin_buf[pin_len++]='0'+d; update_pin_dots(); if(pin_len==4) pin_check(); }
}
static void build_scr_pin() {
    scr_pin=lv_obj_create(nullptr); bg(scr_pin);
    lbl_pin_dots=centered_label(scr_pin,"_ _ _ _",&lv_font_montserrat_40,COL_TEXT,-170);
    lbl_pin_msg =centered_label(scr_pin,"",&lv_font_montserrat_14,COL_ALERT,-125);
    const int W=128,H=56,G=6;
    const int sx=-(3*W+2*G)/2+W/2, sy=-88;
    const int    digits[12]={1,2,3,4,5,6,7,8,9,-1,0,-2};
    const char  *lbs[12]   ={"1","2","3","4","5","6","7","8","9","DEL","0","CANCEL"};
    const uint32_t cols[12] ={COL_CARD,COL_CARD,COL_CARD,COL_CARD,COL_CARD,COL_CARD,
                              COL_CARD,COL_CARD,COL_CARD,COL_ALERT,COL_CARD,COL_OFF};
    for(int i=0;i<12;i++){
        lv_obj_t *b=lv_obj_create(scr_pin);
        lv_obj_set_size(b,W,H);
        lv_obj_align(b,LV_ALIGN_CENTER,sx+(i%3)*(W+G),sy+(i/3)*(H+G));
        lv_obj_set_style_bg_color(b,lv_color_hex(cols[i]),0);
        lv_obj_set_style_radius(b,12,0); lv_obj_set_style_border_width(b,0,0);
        lv_obj_set_style_pad_all(b,0,0); lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(b,LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(b,cb_pin_key,LV_EVENT_CLICKED,(void*)(intptr_t)digits[i]);
        lv_obj_t *l=lv_label_create(b);
        lv_label_set_text(l,lbs[i]);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
        lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
        lv_obj_center(l);
    }
}

// ── BUILD OVERLAY CUENTA ATRÁS ────────────────────────────────
static void cb_arming_extend(lv_event_t *e) { arming_end_ms += 120000UL; }
static void cb_arming_cancel(lv_event_t *e) {
    hist_log(HT_ALARM_CANCEL,0);
    alarm_state=AS_OFF; alarm_armed=false; beep_seq={}; siren_off_at=0; go_to(SCR_TV);
}
static void build_scr_arming() {
    scr_arming=lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_arming,lv_color_hex(0x1010CC),0); // rojo inicial (BGR)
    lv_obj_set_style_bg_opa(scr_arming,LV_OPA_COVER,0);
    centered_label(scr_arming,"SALIR DE LA CASA",&lv_font_montserrat_20,0x111111,-155);
    centered_label(scr_arming,"Armando en:",&lv_font_montserrat_14,0x222222,-112);
    lbl_countdown=centered_label(scr_arming,"2:00",&lv_font_montserrat_48,0x111111,-48);
    make_big_btn(scr_arming,"+2 minutos",COL_RELAY_DIM,38,260,64,cb_arming_extend,nullptr);
    make_big_btn(scr_arming,"CANCELAR ARMADO",COL_OFF,116,260,54,cb_arming_cancel,nullptr);
}

// ── BUILD OVERLAY DIAL ────────────────────────────────────────
static void build_scr_dial() {
    scr_dial=lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_dial,lv_color_hex(0x0A0A12),0);
    lv_obj_set_style_bg_opa(scr_dial,LV_OPA_COVER,0);
    lbl_dial_title_w=centered_label(scr_dial,"BRILLO",&lv_font_montserrat_20,COL_TEXT,-190);
    dial_arc=lv_arc_create(scr_dial);
    lv_obj_set_size(dial_arc,300,300);
    lv_obj_align(dial_arc,LV_ALIGN_CENTER,0,-20);
    lv_arc_set_rotation(dial_arc,135);
    lv_arc_set_bg_angles(dial_arc,0,270);
    lv_arc_set_range(dial_arc,10,255);
    lv_arc_set_value(dial_arc,200);
    lv_obj_set_style_arc_color(dial_arc,lv_color_hex(0x252530),LV_PART_MAIN);
    lv_obj_set_style_arc_width(dial_arc,20,LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(dial_arc,true,LV_PART_MAIN);
    lv_obj_set_style_arc_color(dial_arc,lv_color_hex(COL_RELAY_ON),LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(dial_arc,20,LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(dial_arc,true,LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(dial_arc,lv_color_hex(COL_RELAY_ON),LV_PART_KNOB);
    lv_obj_set_style_pad_all(dial_arc,6,LV_PART_KNOB);
    lv_obj_set_style_bg_opa(dial_arc,LV_OPA_COVER,LV_PART_KNOB);
    lv_obj_add_event_cb(dial_arc,cb_dial_arc_changed,LV_EVENT_VALUE_CHANGED,nullptr);
    lbl_dial_val=lv_label_create(scr_dial);
    lv_label_set_text(lbl_dial_val,"200");
    lv_obj_set_style_text_font(lbl_dial_val,&lv_font_montserrat_40,0);
    lv_obj_set_style_text_color(lbl_dial_val,lv_color_hex(COL_TEXT),0);
    lv_obj_align(lbl_dial_val,LV_ALIGN_CENTER,0,-20);
    lbl_dial_unit=centered_label(scr_dial,"intensidad",&lv_font_montserrat_14,COL_MUTED,30);
    make_big_btn(scr_dial,"OK",      COL_OK, 120,200,56,cb_dial_ok,    nullptr);
    make_big_btn(scr_dial,"CANCELAR",COL_OFF,185,200,44,cb_dial_cancel,nullptr);
}

// ── UPDATE UI ─────────────────────────────────────────────────
static void update_home() {
    alarm_armed=(alarm_state!=AS_OFF);
    lv_obj_set_style_arc_color(harc[1],lv_color_hex(a6v3.input[1]?COL_ALERT:COL_OK_DIM),LV_PART_MAIN);
    lv_obj_set_style_arc_color(harc[2],lv_color_hex(!a6v3.input[4]?COL_ALERT:COL_OK_DIM),LV_PART_MAIN);
    lv_obj_set_style_arc_color(harc[3],lv_color_hex(!a6v3.input[5]?COL_ALERT:COL_OK_DIM),LV_PART_MAIN);
    lv_obj_set_style_arc_color(harc[4],lv_color_hex(a6v3.input[6]?COL_OK:COL_ALERT),LV_PART_MAIN);
    lv_obj_set_style_arc_color(rarc[1],lv_color_hex(a6v3.output[1]?COL_RELAY_ON:COL_RELAY_DIM),LV_PART_MAIN);
    lv_obj_set_style_arc_color(rarc[2],lv_color_hex(a6v3.output[2]?COL_RELAY_ON:COL_RELAY_DIM),LV_PART_MAIN);
    lv_obj_set_style_arc_color(rarc[3],lv_color_hex(a6v3.output[3]?COL_ALERT:COL_RELAY_DIM),LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc_alarm_zone,lv_color_hex(alarm_armed?COL_ALERT:COL_ZONE_ALARM),LV_PART_MAIN);
    lv_label_set_text(lbl_alarm_badge,alarm_armed?"ALARMA\nON":"ALARMA\nOFF");
    lv_obj_set_style_text_color(lbl_alarm_badge,lv_color_hex(alarm_armed?COL_ALERT:COL_MUTED),0);
}
static void update_sensors_tile() {
    lv_obj_set_style_bg_color(sdot[1],lv_color_hex(a6v3.input[1]?COL_ALERT:COL_OFF),0);
    lv_obj_set_style_bg_color(sdot[2],lv_color_hex(!a6v3.input[4]?COL_ALERT:COL_OK),0);
    lv_obj_set_style_bg_color(sdot[3],lv_color_hex(!a6v3.input[5]?COL_ALERT:COL_OK),0);
    lv_obj_set_style_bg_color(sdot[4],lv_color_hex(a6v3.input[6]?COL_OK:COL_ALERT),0);
}
static void update_relays_tile() {
    if(btn_agua_r)
        lv_obj_set_style_bg_color(btn_agua_r,lv_color_hex(a6v3.output[1]?COL_OK:COL_OFF),0);
    if(btn_sirena_r)
        lv_obj_set_style_bg_color(btn_sirena_r,lv_color_hex(a6v3.output[3]?COL_ALERT:COL_OFF),0);
    refresh_heat_ui();
}
static void update_alarm_tile() {
    alarm_armed=(alarm_state!=AS_OFF);
    if(alarm_armed){
        lv_label_set_text(lbl_arm_state,"ARMADA");
        lv_obj_set_style_text_color(lbl_arm_state,lv_color_hex(COL_ALERT),0);
        lv_label_set_text((lv_obj_t*)lv_obj_get_child(btn_arm,0),"DESARMAR");
        lv_obj_set_style_bg_color(btn_arm,lv_color_hex(COL_ALERT),0);
    } else {
        lv_label_set_text(lbl_arm_state,"DESARMADA");
        lv_obj_set_style_text_color(lbl_arm_state,lv_color_hex(COL_MUTED),0);
        lv_label_set_text((lv_obj_t*)lv_obj_get_child(btn_arm,0),"ARMAR");
        lv_obj_set_style_bg_color(btn_arm,lv_color_hex(COL_OFF),0);
    }
}
static void update_chart() {
    if(!wx.ok||!wx_chart) return;
    float tmin=wx.h_temp[0],tmax=wx.h_temp[0],rmin=wx.h_rain[0],rmax=0.0f;
    for(int i=0;i<12;i++){
        if(wx.h_temp[i]<tmin) tmin=wx.h_temp[i];
        if(wx.h_temp[i]>tmax) tmax=wx.h_temp[i];
        if(wx.h_rain[i]<rmin) rmin=wx.h_rain[i];
        if(wx.h_rain[i]>rmax) rmax=wx.h_rain[i];
    }
    float t_lo=tmin-1.0f, t_hi=tmax+1.0f;
    if(t_hi-t_lo<2.0f) t_hi=t_lo+2.0f;
    float r_hi=rmax<1.0f?1.0f:rmax+0.5f;
    lv_chart_set_range(wx_chart,LV_CHART_AXIS_PRIMARY_Y,  (int32_t)t_lo,(int32_t)(t_hi+1));
    lv_chart_set_range(wx_chart,LV_CHART_AXIS_SECONDARY_Y,0,(int32_t)(r_hi+1));
    lv_value_precise_t *t=lv_chart_get_y_array(wx_chart,chart_temp_ser);
    lv_value_precise_t *r=lv_chart_get_y_array(wx_chart,chart_rain_ser);
    for(int i=0;i<12;i++){ t[i]=(lv_value_precise_t)wx.h_temp[i]; r[i]=(lv_value_precise_t)wx.h_rain[i]; }
    lv_chart_refresh(wx_chart);
    if(lbl_tmax){
        char buf[8];
        snprintf(buf,sizeof(buf),"%.0f°",tmax); lv_label_set_text(lbl_tmax,buf);
        snprintf(buf,sizeof(buf),"%.0f°",tmin); lv_label_set_text(lbl_tmin,buf);
        snprintf(buf,sizeof(buf),"%.1f", rmax); lv_label_set_text(lbl_rmax,buf);
        snprintf(buf,sizeof(buf),"%.1f", rmin); lv_label_set_text(lbl_rmin,buf);
    }
}
static void update_weather_display() {
    if(!wx.ok) return;
    char buf[40];
    snprintf(buf,sizeof(buf),"%.1f\xc2\xb0""C",wx.temp);
    lv_label_set_text(lbl_temp_ext,buf);
    snprintf(buf,sizeof(buf),"%d km/h  %.1f mm/h",(int)wx.wind,wx.rain_now);
    lv_label_set_text(lbl_weather,buf);
    update_chart();
}

// ── ESTADO BROKER ────────────────────────────────────────────
static void update_broker_status() {
    bool online = (mqtt_last_activity > 0 &&
                   millis() - mqtt_last_activity < 5UL*60000UL);
    if(arc_settings_zone)
        lv_obj_set_style_arc_color(arc_settings_zone,
            lv_color_hex(online ? COL_OK_DIM : COL_ZONE_SET), LV_PART_MAIN);
    if(lbl_sistema) {
        lv_label_set_text(lbl_sistema, online ? "Sistema\nOK" : "Sistema\nOFF");
        lv_obj_set_style_text_color(lbl_sistema,
            lv_color_hex(online ? COL_OK : COL_TEXT), 0);
    }
    if(lbl_broker_status) {
        if(online) {
            lv_label_set_text(lbl_broker_status,"Broker: Conectado");
            lv_obj_set_style_text_color(lbl_broker_status,lv_color_hex(COL_OK),0);
        } else {
            char buf[48];
            if(mqtt_last_conn_epoch > 0) {
                struct tm *t=localtime(&mqtt_last_conn_epoch);
                snprintf(buf,sizeof(buf),"Broker: OFF  %02d/%02d %02d:%02d",
                    t->tm_mday,t->tm_mon+1,t->tm_hour,t->tm_min);
            } else {
                snprintf(buf,sizeof(buf),"Broker: sin conexion");
            }
            lv_label_set_text(lbl_broker_status,buf);
            lv_obj_set_style_text_color(lbl_broker_status,lv_color_hex(COL_MUTED),0);
        }
    }
}

// ── ALARMAS ───────────────────────────────────────────────────
static void check_alarms() {
    bool flood=a6v3.input[1], smoke=!a6v3.input[5];
    if((flood||smoke)&&!critical_alert){
        hist_log(HT_ALARM_TRIGGER,(uint8_t)(flood?1:5));
        critical_alert=true; alarm_state=AS_SOUNDING; beep_seq={}; siren_off_at=0;
        relay_set(3,true);
        lv_label_set_text(lbl_alarm_type, flood?"INUNDACION":"HUMO");
        lv_label_set_text(lbl_alarm_detail,flood?"Valvulas cerradas auto.":"Sirena activada");
        lv_label_set_text((lv_obj_t*)lv_obj_get_child(btn_deactivate,0),"DESACTIVAR");
        go_to(SCR_ALARM_CRIT);
    }
    if(alarm_state==AS_ARMED&&!a6v3.input[4]&&!intruder_active){
        hist_log(HT_ALARM_TRIGGER,4);
        intruder_active=true; alarm_state=AS_GRACE; alarm_ts=millis();
        memset(grace_beeps,0,sizeof(grace_beeps));
        lv_label_set_text(lbl_alarm_type,"INTRUSION");
        lv_label_set_text(lbl_alarm_detail,"Introduce PIN para desarmar");
        lv_label_set_text((lv_obj_t*)lv_obj_get_child(btn_deactivate,0),"INTRODUCIR PIN");
        go_to(SCR_ALARM_CRIT);
    }
}

// ── SCREEN SWITCH ─────────────────────────────────────────────
static void do_switch(Screen s) {
    switch(s){
        case SCR_TV:
            update_home(); update_sensors_tile(); update_relays_tile(); update_alarm_tile();
            lv_scr_load_anim(tv,LV_SCR_LOAD_ANIM_FADE_IN,200,0,false);
            cur_scr=SCR_TV; return;
        case SCR_ALARM_CRIT:
            set_brightness(cfg_brightness); screen_dimmed=false; last_touch_ms=millis();
            lv_scr_load_anim(scr_alarm,LV_SCR_LOAD_ANIM_MOVE_TOP,250,0,false);
            cur_scr=SCR_ALARM_CRIT; return;
        case SCR_PIN:
            update_pin_dots(); lv_label_set_text(lbl_pin_msg,"");
            lv_scr_load_anim(scr_pin,LV_SCR_LOAD_ANIM_MOVE_TOP,250,0,false);
            cur_scr=SCR_PIN; return;
        case SCR_ARMING_SCR:
            if(lbl_countdown) lv_label_set_text(lbl_countdown,"2:00");
            lv_scr_load_anim(scr_arming,LV_SCR_LOAD_ANIM_MOVE_TOP,250,0,false);
            cur_scr=SCR_ARMING_SCR; return;
        case SCR_DIAL:
            lv_scr_load_anim(scr_dial,LV_SCR_LOAD_ANIM_MOVE_BOTTOM,250,0,false);
            cur_scr=SCR_DIAL; return;
        case SCR_CHANGE_PIN:
            chpin_phase=0; memset(chpin_buf,0,5); chpin_len=0;
            if(lbl_chpin_title) lv_label_set_text(lbl_chpin_title,"PIN actual:");
            if(lbl_chpin_dots){ char b[9]="_ _ _ _"; lv_label_set_text(lbl_chpin_dots,b); }
            if(lbl_chpin_msg)  lv_label_set_text(lbl_chpin_msg,"");
            lv_scr_load_anim(scr_change_pin,LV_SCR_LOAD_ANIM_MOVE_TOP,250,0,false);
            cur_scr=SCR_CHANGE_PIN; return;
    }
}

// ── TUYA: funciones ───────────────────────────────────────────
static String tuya_hmac(const String& key, const String& msg) {
    uint8_t hmac[32];
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx,mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),1);
    mbedtls_md_hmac_starts(&ctx,(const uint8_t*)key.c_str(),key.length());
    mbedtls_md_hmac_update(&ctx,(const uint8_t*)msg.c_str(),msg.length());
    mbedtls_md_hmac_finish(&ctx,hmac);
    mbedtls_md_free(&ctx);
    String out="";
    for(int i=0;i<32;i++){ char h[3]; sprintf(h,"%02X",hmac[i]); out+=h; }
    return out;
}
static String tuya_ts() {
    char buf[20]; time_t now; time(&now);
    snprintf(buf,sizeof(buf),"%llu",(unsigned long long)now*1000ULL);
    return String(buf);
}
static String tuya_sign(const String& prefix, const String& path) {
    String sts="GET\n" TUYA_SHA256_EMPTY "\n\n"+path;
    return tuya_hmac(TUYA_CLIENT_SECRET, prefix+sts);
}
static bool tuya_get_token() {
    String ts=tuya_ts(), nonce="";
    String sign=tuya_sign(String(TUYA_CLIENT_ID)+ts+nonce,"/v1.0/token?grant_type=1");
    WiFiClientSecure cl; cl.setInsecure();
    HTTPClient http;
    http.begin(cl,TUYA_BASE_URL "/v1.0/token?grant_type=1");
    http.addHeader("client_id",TUYA_CLIENT_ID);
    http.addHeader("t",ts); http.addHeader("nonce",nonce);
    http.addHeader("sign_method","HMAC-SHA256"); http.addHeader("sign",sign);
    http.setTimeout(15000);
    bool ok=false;
    int code=http.GET(); String body=http.getString();
    Serial.printf("Tuya token HTTP %d: %s\n",code,body.c_str());
    if(code==200){
        JsonDocument doc;
        if(!deserializeJson(doc,body)&&doc["success"].as<bool>()){
            tuya_token=doc["result"]["access_token"].as<String>();
            int exp=doc["result"]["expire_time"].as<int>();
            tuya_token_exp=millis()+(unsigned long)(exp-60)*1000UL;
            ok=true; Serial.println("Tuya: token OK");
        }
    }
    http.end(); return ok;
}
static void fetch_tuya_temp() {
    if(!WiFi.isConnected()) return;
    tuya_last=millis();
    if(tuya_token.isEmpty()||millis()>tuya_token_exp)
        if(!tuya_get_token()) return;
    String ts=tuya_ts(), nonce="";
    String path="/v1.0/devices/" TUYA_DEVICE_ID "/status";
    String sign=tuya_sign(String(TUYA_CLIENT_ID)+tuya_token+ts+nonce, path);
    WiFiClientSecure cl; cl.setInsecure();
    HTTPClient http;
    http.begin(cl, TUYA_BASE_URL+path);
    http.addHeader("client_id",TUYA_CLIENT_ID);
    http.addHeader("access_token",tuya_token);
    http.addHeader("t",ts); http.addHeader("nonce",nonce);
    http.addHeader("sign_method","HMAC-SHA256"); http.addHeader("sign",sign);
    http.setTimeout(15000);
    if(http.GET()==200){
        JsonDocument doc;
        if(!deserializeJson(doc,http.getString())&&doc["success"].as<bool>()){
            for(JsonVariant item : doc["result"].as<JsonArray>()){
                String c=item["code"].as<String>();
                if(c=="va_temperature"||c=="temp_current"){
                    tuya_temp_int=item["value"].as<float>()/10.0f;
                    char buf[12]; snprintf(buf,sizeof(buf),"%.1f\xc2\xb0""C",tuya_temp_int);
                    if(lbl_temp_int) lv_label_set_text(lbl_temp_int,buf);
                    Serial.printf("Tuya: %.1f C\n",tuya_temp_int);
                }
                if(c=="va_humidity"||c=="humidity_value"){
                    tuya_humidity=item["value"].as<float>();
                    // algunos sensores dan humedad en décimas, otros en enteros
                    if(tuya_humidity>100.0f) tuya_humidity/=10.0f;
                    char buf[12]; snprintf(buf,sizeof(buf),"%.0f%% HR",tuya_humidity);
                    if(lbl_humidity) lv_label_set_text(lbl_humidity,buf);
                }
            }
            check_heating_auto();
        }
    }
    http.end();
}

// ── WEATHER FETCH ─────────────────────────────────────────────
static void fetch_weather() {
    if(!WiFi.isConnected()) return;
    WiFiClientSecure client; client.setInsecure();
    HTTPClient http; http.begin(client,WX_URL); http.setTimeout(15000);
    if(http.GET()==200){
        String body=http.getString();
        JsonDocument doc;
        if(!deserializeJson(doc,body)){
            wx.temp    =doc["current"]["temperature_2m"].as<float>();
            wx.rain_now=doc["current"]["precipitation"].as<float>();
            float wmax=0;
            for(int i=0;i<12;i++){
                float w=doc["hourly"]["wind_speed_10m"][i].as<float>();
                if(w>wmax) wmax=w;
                wx.h_temp[i]=doc["hourly"]["temperature_2m"][i].as<float>();
                wx.h_rain[i]=doc["hourly"]["precipitation"][i].as<float>();
            }
            wx.wind=wmax; wx.ok=true;
            update_weather_display();
        }
    }
    http.end(); wx_last=millis();
}

// Aumenta el stack del loop task de 8 KB (defecto) a 32 KB para LVGL
size_t getArduinoLoopTaskStackSize(void) { return 32768; }

// ── SETUP ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    unsigned long t0=millis();
    while(!Serial&&(millis()-t0)<5000) delay(10);
    Serial.println("DOMUS v0.5 arranque...");
    hist_init();
    hist_ready_at=millis()+30000UL;

    prefs.begin("domus",true);
    cfg_brightness=(int)prefs.getInt("brightness",200);
    cfg_dim_delay =(int)prefs.getInt("dim_delay", 5);
    heat_setpoint =(int)prefs.getInt("heat_sp",   20);
    heat_mode     =(HeatMode)prefs.getInt("heat_mode",0);
    prefs.getString("pin","1234").toCharArray(cfg_pin,sizeof(cfg_pin));
    prefs.end();

    Wire.begin(I2C_SDA,I2C_SCL);
    lcd_power_on();
    touch_init();

    ledcSetup(0,5000,8);
    ledcAttachPin(TFT_BL_PIN,0);
    set_brightness(cfg_brightness);

    pinMode(ENC_A,INPUT_PULLUP); pinMode(ENC_B,INPUT_PULLUP); pinMode(ENC_SW,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A),enc_isr,CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B),enc_isr,CHANGE);

    display->begin();
    lv_init();

    lv_display_t *disp=lv_display_create(TFT_WIDTH,TFT_HEIGHT);
    lv_display_set_flush_cb(disp,lvgl_flush_cb);

    // Buffer parcial: 40 filas. Sin doble buffer para evitar deadlock en LVGL 9.x.
    size_t buf_sz=(size_t)TFT_WIDTH*40*sizeof(lv_color_t);
    lv_color_t *buf1=(lv_color_t*)heap_caps_malloc(buf_sz,MALLOC_CAP_SPIRAM);
    if(!buf1) buf1=(lv_color_t*)malloc(buf_sz);
    lv_display_set_buffers(disp,buf1,nullptr,buf_sz,LV_DISPLAY_RENDER_MODE_PARTIAL);

    esp_timer_handle_t lt;
    esp_timer_create_args_t la={.callback=[](void*){lv_tick_inc(2);},.arg=nullptr,.name="lv"};
    esp_timer_create(&la,&lt); esp_timer_start_periodic(lt,2000);

    lv_indev_t *indev=lv_indev_create();
    lv_indev_set_type(indev,LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev,touch_read_cb);
    lv_indev_set_scroll_limit(indev,2);   // swipe más fácil (default=10)

    tv=lv_tileview_create(nullptr);
    lv_obj_set_style_bg_color(tv,lv_color_hex(COL_BG),0);
    lv_obj_set_scrollbar_mode(tv,LV_SCROLLBAR_MODE_OFF);
    tile_alarm   =lv_tileview_add_tile(tv,1,0,LV_DIR_BOTTOM);
    tile_sensors =lv_tileview_add_tile(tv,0,1,LV_DIR_RIGHT);
    tile_home    =lv_tileview_add_tile(tv,1,1,LV_DIR_ALL);
    tile_relays  =lv_tileview_add_tile(tv,2,1,LV_DIR_LEFT);
    tile_settings=lv_tileview_add_tile(tv,1,2,LV_DIR_TOP);

    build_tile_home();
    build_tile_sensors();
    build_tile_relays();
    build_tile_alarm();
    build_tile_settings();
    build_scr_alarm();
    build_scr_pin();
    build_scr_change_pin();
    build_scr_arming();
    build_scr_dial();

    lv_tileview_set_tile(tv,tile_home,LV_ANIM_OFF);
    lv_scr_load(tv);
    for(int i=0;i<20;i++){lv_timer_handler();delay(5);}
    Serial.println("Pantalla lista.");

    WiFiManager wm;
    if(wm.autoConnect("DOMUS-Setup")){
        Serial.printf("WiFi: %s\n",WiFi.localIP().toString().c_str());
        if(lbl_settings_ip){
            char buf[40]; snprintf(buf,sizeof(buf),"IP: %s",WiFi.localIP().toString().c_str());
            lv_label_set_text(lbl_settings_ip,buf);
        }
    }
    broker.init(1883);

    configTime(0,0,"pool.ntp.org","time.cloudflare.com");
    Serial.print("NTP sync");
    time_t now=0;
    for(int i=0;i<40&&now<1000000000L;i++){delay(500);time(&now);Serial.print(".");}
    Serial.printf("%s\n",now>1000000000L?" OK":" timeout");

    last_touch_ms=millis();
    fetch_weather();
    fetch_tuya_temp();
}

// ── LOOP ──────────────────────────────────────────────────────
static unsigned long last_status=0, flash_ts=0, countdown_ts=0;
static bool flash_state=false;

void loop() {
    broker.update();

    // Beeps
    if(siren_off_at&&millis()>=siren_off_at&&alarm_state!=AS_SOUNDING){ relay_set(3,false); siren_off_at=0; }
    if(beep_seq.total>0&&beep_seq.done<beep_seq.total&&!siren_off_at&&millis()>=beep_seq.next){
        relay_set(3,true); siren_off_at=millis()+beep_seq.dur_ms;
        beep_seq.done++; beep_seq.next=siren_off_at+beep_seq.pause_ms;
        if(beep_seq.done>=beep_seq.total) beep_seq.total=0;
    }

    // Encoder rotación
    if(enc_delta!=0){
        noInterrupts(); int d=enc_delta; enc_delta=0; interrupts();
        if(dial_mode!=DIAL_NONE&&dial_arc){
            enc_accum+=d;
            int steps=enc_accum/2; enc_accum%=2;
            if(steps!=0){
                int lo=(dial_mode==DIAL_BRIGHTNESS)?10:(dial_mode==DIAL_DIM)?0:15;
                int hi=(dial_mode==DIAL_BRIGHTNESS)?255:(dial_mode==DIAL_DIM)?30:30;
                int mult=(dial_mode==DIAL_BRIGHTNESS)?5:1;
                dial_val=constrain(dial_val+steps*mult,lo,hi);
                lv_arc_set_value(dial_arc,dial_val);
                update_dial_display();
            }
        }
    }

    // Encoder botón
    bool btn_now=(digitalRead(ENC_SW)==LOW);
    if(btn_now&&!enc_btn_prev&&millis()-enc_btn_last_ms>150){
        enc_btn_last_ms=millis();
        if(dial_mode!=DIAL_NONE){ apply_dial(); dial_mode=DIAL_NONE; go_to(SCR_TV); }
    }
    enc_btn_prev=btn_now;

    // Auto-dim
    if(cfg_dim_delay>0&&!screen_dimmed)
        if(millis()-last_touch_ms>(unsigned long)cfg_dim_delay*60000UL){ set_brightness(40); screen_dimmed=true; }

    // Cuenta atrás armado
    if(alarm_state==AS_ARMING){
        unsigned long now=millis();
        if(now>=arming_end_ms){
            alarm_state=AS_ARMED; alarm_armed=true;
            pin_for_arm=false; memset(pin_buf,0,5); pin_len=0;
            go_to(SCR_PIN);
        } else if(now-countdown_ts>=500){
            countdown_ts=now;
            unsigned long rem=(arming_end_ms-now)/1000UL;
            if(lbl_countdown){ char buf[8]; snprintf(buf,sizeof(buf),"%lu:%02lu",rem/60,rem%60); lv_label_set_text(lbl_countdown,buf); }
        }
    }

    // Gracia intrusión
    if(alarm_state==AS_GRACE){
        unsigned long e=millis()-alarm_ts;
        if(e>=5000 &&!grace_beeps[0]){grace_beeps[0]=true;start_beep_seq(1,100,200);}
        if(e>=30000&&!grace_beeps[1]){grace_beeps[1]=true;start_beep_seq(2,100,300);}
        if(e>=60000&&!grace_beeps[2]){grace_beeps[2]=true;alarm_state=AS_SOUNDING;beep_seq={};siren_off_at=0;relay_set(3,true);}
    }

    // Parpadeo alarma crítica
    if(cur_scr==SCR_ALARM_CRIT&&millis()-flash_ts>=400){
        flash_state=!flash_state; flash_ts=millis();
        lv_obj_set_style_bg_color(scr_alarm,lv_color_hex(flash_state?0x3A0000:0x100000),0);
    }
    // Parpadeo cuenta atrás (blanco / rojo)
    if(cur_scr==SCR_ARMING_SCR&&millis()-flash_ts>=500){
        flash_state=!flash_state; flash_ts=millis();
        lv_obj_set_style_bg_color(scr_arming,lv_color_hex(flash_state?0xFFFFFF:0x1010CC),0);
    }

    // UI MQTT
    if(ui_needs_update){
        check_alarms(); update_home(); update_sensors_tile(); update_relays_tile(); update_alarm_tile();
        check_heating_auto();
        ui_needs_update=false;
    }

    if(scr_change){ scr_change=false; do_switch(pend_scr); }
    if(millis()-wx_last  >30UL*60000UL) fetch_weather();
    if(millis()-tuya_last> 5UL*60000UL) fetch_tuya_temp();
    if(millis()-temp_log_last>TEMP_LOG_MS) temp_log_now();

    if(millis()-last_status>=5000){
        last_status=millis();
        update_broker_status();
        Serial.printf("[S] flood=%d pir=%d smoke=%d pwr=%d | agua=%d calef=%d sir=%d | heat=%d sp=%d int=%.1f\n",
            a6v3.input[1],a6v3.input[4],a6v3.input[5],a6v3.input[6],
            a6v3.output[1],a6v3.output[2],a6v3.output[3],
            (int)heat_mode,heat_setpoint,tuya_temp_int);
    }

    lv_timer_handler();
    delay(2);
}
