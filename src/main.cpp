#include <Arduino.h>
#define FW_VERSION "v2.29"
#include <Wire.h>
#include <esp_task_wdt.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <sMQTTBroker.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <mbedtls/md.h>
#include <time.h>
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
#define COL_HEAT_BOX    0x1820B0  // BGR→ pantalla: rojo medio vívido (R=176,G=32,B=24)
#define ARC_WIDTH      72
#define COL_CHART_TEMP 0x4040FF
#define COL_CHART_RAIN 0xFF4000

// ── NVS ──────────────────────────────────────────────────────
static Preferences prefs;
static int cfg_brightness     = 200;
static int cfg_dim_delay      = 2;   // min hasta atenuar (0=nunca)
static int cfg_dim_brightness = 5;   // brillo al hacer dim (1-100 %)
static int cfg_off_delay      = 10;  // min hasta apagar/salvapantallas (0=nunca, >=dim)
static uint8_t cfg_saver_mode = 2;   // salvapantallas: 0=Off(apagar) 1=On(siempre) 2=Solo si calefaccion
static uint8_t cfg_anim       = 0;   // 0=ninguna 1=deslizar

// ── Calefacción ───────────────────────────────────────────────
enum HeatMode { HM_OFF, HM_MANUAL, HM_CONSIGNA, HM_PROGRAMA };
static HeatMode heat_mode     = HM_OFF;
static int      heat_setpoint = 20;
static int      cfg_eco_sp    = 16;
static uint64_t cfg_prog[7]   = {};  // bit r = slot r activo (30min), 48 slots/día
static float    tuya_temp_int = NAN;
static float    tuya_humidity = NAN;

// ── Auto-dim / apagado / salvapantallas ──────────────────────
static unsigned long last_touch_ms = 0;
static bool          screen_dimmed = false;  // true en DIM o en APAGADO/salvapantallas
static bool          screen_off    = false;  // true en fase de apagado (negro o salvapantallas)
static bool          saver_active  = false;  // salvapantallas cargado en pantalla
static bool          wake_reload_pending = false; // recargar pantalla previa fuera de callbacks
static lv_obj_t     *scr_before_saver = nullptr;

// ── Encoder ──────────────────────────────────────────────────
static volatile int  enc_delta       = 0;
static int           enc_accum       = 0;
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
static unsigned long arming_end_ms = 0;
static bool grace_beeps[3]      = {};

// ── Estado A6v3 ──────────────────────────────────────────────
struct { bool input[7]={}; bool output[7]={}; } a6v3;
static bool ota_started = false;   // OTA arrancado (solo tras conectar al WiFi)
static bool ui_needs_update = false;
static bool alarm_armed     = false;
static bool intruder_active = false;
static bool critical_alert  = false;

// ── PIN ──────────────────────────────────────────────────────
static char          cfg_pin[5]    = "1234";
static char          pin_buf[5]    = {};
static int           pin_len       = 0;
static bool          pin_for_arm   = true;
static unsigned long pin_open_ts   = 0; // debounce toque fantasma del encoder

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
    float temp=NAN; float wind=0; float rain_now=0; bool ok=false; int code=-1;
    float h_temp[12]={}; float h_rain[12]={};
} wx;
static unsigned long wx_last = 0;
#define WX_URL "https://api.open-meteo.com/v1/forecast" \
    "?latitude=42.80632457618044&longitude=-1.6290100870212767" \
    "&current=temperature_2m,wind_speed_10m,precipitation" \
    "&hourly=precipitation,wind_speed_10m,temperature_2m" \
    "&daily=weather_code&forecast_days=1&timezone=Europe%2FMadrid"

// ── Tuya API ─────────────────────────────────────────────────
#define TUYA_CLIENT_ID     "df33cuper4k8ce7qstn8"
#define TUYA_CLIENT_SECRET "c86757d118de4dcda2f1c143936dc7bd"
#define TUYA_DEVICE_ID     "bf080169b9cf3e36f2khp2"
#define TUYA_BASE_URL      "https://openapi.tuyaeu.com"
#define TUYA_SHA256_EMPTY  "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"

static String        tuya_token     = "";
static unsigned long tuya_token_exp = 0;
static unsigned long tuya_last      = 0;

// ── MQTT ─────────────────────────────────────────────────────
#define MQTT_STATE "A6v3/30EDA03B1378/STATE"
#define MQTT_SET   "A6v3/30EDA03B1378/SET"
static unsigned long mqtt_last_activity  = 0;
static time_t        mqtt_last_conn_epoch = 0;
static volatile bool mqtt_cmd_disarm     = false;
static volatile bool mqtt_cmd_arm        = false;
static time_t        domus_pub_epoch      = 0;

class DomusBroker : public sMQTTBroker {
public:
    bool onEvent(sMQTTEvent *ev) override {
        mqtt_last_activity = millis();
        time_t now; time(&now); if(now > 1000000000L) mqtt_last_conn_epoch = now;
        if (ev->Type() == NewClient_sMQTTEventType)
            Serial.println("Broker: cliente conectado");
        else if (ev->Type() == Public_sMQTTEventType) {
            auto *p = (sMQTTPublicClientEvent*)ev;
            if (p->Topic() == "DOMUS/CMD") {
                JsonDocument doc;
                if (!deserializeJson(doc, p->Payload().c_str(), p->Payload().size())) {
                    String cmd = doc["cmd"] | "";
                    if (cmd == "disarm") {
                        mqtt_cmd_disarm = true;
                        Serial.println("[CMD] disarm via MQTT");
                    } else if (cmd == "arm") {
                        String pin = doc["pin"] | "";
                        if (pin.length()==4 && strncmp(pin.c_str(),cfg_pin,4)==0)
                            mqtt_cmd_arm = true;
                        else
                            Serial.println("[CMD] arm via MQTT: PIN incorrecto");
                    }
                }
                return true;
            }
            if (p->Topic() == "DOMUS/ENV") {   // Tª/H interior desde HA (Xiaomi BLE), JSON {"t":21.5,"h":53}
                JsonDocument doc;
                if (!deserializeJson(doc, p->Payload().c_str(), p->Payload().size())) {
                    if (!doc["t"].isNull()) tuya_temp_int = doc["t"].as<float>();
                    if (!doc["h"].isNull()) tuya_humidity = doc["h"].as<float>();
                    ui_needs_update = true;
                }
                return true;
            }
            if (p->Topic() != MQTT_STATE) return true;
            JsonDocument doc;
            if (deserializeJson(doc, p->Payload().c_str(), p->Payload().size())) return true;
            for (int i=1;i<=6;i++) {
                char ki[10], ko[10];
                snprintf(ki,10,"input%d",i);
                snprintf(ko,10,"output%d",i);
                if (!doc[ki]["value"].isNull()) a6v3.input[i]=doc[ki]["value"].as<bool>();
                if (!doc[ko]["value"].isNull()) a6v3.output[i]=doc[ko]["value"].as<bool>();
            }
            ui_needs_update = true;
        }
        return true;
    }
} broker;

static void mqtt_publish_status();  // forward declaration

static void relay_set(int n, bool v) {
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
    ledcWrite(0, (uint32_t)constrain(val, 0, 255));  // 0 = retro apagada del todo
}
// Brillo del DIM: cfg_dim_brightness es 1-100 % → PWM (suelo 2 para "casi apagado")
static void apply_dim_brightness() {
    int pwm = cfg_dim_brightness * 255 / 100;
    if (pwm < 2) pwm = 2;
    set_brightness(pwm);
}
// Despierta la pantalla a brillo pleno. Si 'reload', recarga la pantalla previa
// al salvapantallas (diferido al loop; nunca llamar lv_scr_load desde callbacks
// de input). Para casos donde sigue un go_to(), usar reload=false.
static void request_wake(bool reload) {
    last_touch_ms = millis();
    set_brightness(cfg_brightness);
    screen_dimmed = false; screen_off = false;
    if (saver_active) { saver_active = false; if (reload) wake_reload_pending = true; }
}

// ── LVGL flush ───────────────────────────────────────────────
static void lvgl_flush_cb(lv_display_t *d, const lv_area_t *a, uint8_t *px) {
    esp_task_wdt_reset();
    display->draw16bitRGBBitmap(a->x1, a->y1, (uint16_t*)px,
        a->x2-a->x1+1, a->y2-a->y1+1);
    lv_display_flush_ready(d);
}

// ── Navegación (enum y estado antes de touch_read_cb) ────────
enum Screen { SCR_TV, SCR_ALARM_CRIT, SCR_PIN, SCR_ARMING_SCR, SCR_DIAL, SCR_CHANGE_PIN, SCR_GRAPH, SCR_HEATMAP, SCR_HIST_MENU, SCR_LOG, SCR_PROG };
static Screen cur_scr=SCR_TV, pend_scr=SCR_TV;
static bool   scr_change=false;

// ── Swipe manual ─────────────────────────────────────────────
static int16_t sw_sx = -1, sw_sy = -1, sw_lx = -1, sw_ly = -1;
static bool    sw_down = false;
static bool    sw_consumed = false;   // swipe ya detectado → ignorar release
static int8_t  sw_dc = 0, sw_dr = 0;
// Bloqueo post-swipe: impide clicks en tiles 350ms tras cualquier swipe
static unsigned long swipe_block_until = 0;
static inline bool swipe_blocked() { return millis() < swipe_block_until; }

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
            int16_t tx = ((xh&0x0F)<<8)|xl;
            int16_t ty = ((yh&0x0F)<<8)|yl;
            last_touch_ms = millis();
            if (screen_dimmed) request_wake(true);
            if (!sw_down) { sw_sx=tx; sw_sy=ty; sw_down=true; sw_consumed=false; }
            sw_lx = tx; sw_ly = ty;
            // Detectar swipe durante el movimiento (threshold 30px)
            if (!sw_consumed && cur_scr == SCR_TV) {
                int dx = tx - sw_sx, dy = ty - sw_sy;
                if (abs(dx) > abs(dy) && abs(dx) > 30) {
                    sw_dc = (dx < 0) ? 1 : -1;
                    sw_consumed = true;
                    swipe_block_until = millis() + 350;
                    lv_indev_reset(indev, NULL);
                } else if (abs(dy) > abs(dx) && abs(dy) > 30) {
                    sw_dr = (dy < 0) ? 1 : -1;
                    sw_consumed = true;
                    swipe_block_until = millis() + 350;
                    lv_indev_reset(indev, NULL);
                }
            }
            if (sw_consumed) {
                // Swipe activo: reportar RELEASED para que LVGL no registre
                // nuevas pulsaciones mientras el dedo sigue en la pantalla.
                // lv_indev_reset cancelará act_obj en el próximo frame.
                data->point.x = sw_sx; data->point.y = sw_sy;
                data->state   = LV_INDEV_STATE_RELEASED;
            } else {
                data->point.x = tx; data->point.y = ty;
                data->state   = LV_INDEV_STATE_PRESSED;
            }
        } else {
            sw_down = false; sw_consumed = false;
            data->state = LV_INDEV_STATE_RELEASED;
        }
    } else { while(Wire.available()) Wire.read(); data->state=LV_INDEV_STATE_RELEASED; }
}

// ── Widgets globales ─────────────────────────────────────────
static lv_obj_t *tv;
static lv_obj_t *tile_home, *tile_sensors, *tile_relays, *tile_alarm, *tile_settings;
static uint8_t tile_col = 1, tile_row = 1; // posición actual (home=1,1)

// Home
static lv_obj_t *harc[5], *rarc[4], *arc_alarm_zone, *arc_settings_zone, *lbl_alarm_badge;
static lv_obj_t *lbl_sistema = nullptr, *lbl_temp_ext, *lbl_weather, *lbl_temp_int, *lbl_humidity;
static lv_obj_t *lbl_wx_forecast = nullptr;
static lv_obj_t *lbl_wx_icon     = nullptr;
extern "C" const lv_font_t weather_font;  // src/weather_font.c — iconos MDI del tiempo
extern "C" const lv_font_t icon_font;     // src/icon_font.c    — icono home-thermometer (calefaccion)

// Historial tile
static lv_obj_t *btn_hist[3] = {}; // Temperatura, Presencia, Registro
static lv_obj_t *lbl_hist_tile_mem = nullptr;

// Relés
static lv_obj_t *btn_agua_r=nullptr;
static lv_obj_t *btn_sirena_r=nullptr;
static lv_obj_t *btn_hm[3]={};
static lv_obj_t *btn_setpoint=nullptr, *lbl_setpoint=nullptr;

// Alarma tile
static lv_obj_t *lbl_arm_state, *btn_arm;
// Settings
static lv_obj_t *lbl_settings_ip=nullptr, *lbl_cfg_brightness=nullptr, *lbl_cfg_dim=nullptr;
static lv_obj_t *lbl_broker_status=nullptr, *lbl_cfg_anim=nullptr, *lbl_cfg_saver=nullptr;
// Overlays
static lv_obj_t *scr_alarm=nullptr, *scr_pin=nullptr, *scr_arming=nullptr, *scr_dial=nullptr, *scr_change_pin=nullptr;
static lv_obj_t *scr_saver=nullptr, *saver_box=nullptr;
static lv_obj_t *lbl_saver_time=nullptr, *lbl_saver_ext=nullptr, *lbl_saver_int=nullptr, *lbl_saver_heat=nullptr;

// Programa calefacción
static lv_obj_t           *scr_prog      = nullptr;
static lv_obj_t           *prog_cells[7][48] = {};
static lv_obj_t           *lbl_eco_sp    = nullptr;

// Graph
static lv_obj_t           *scr_graph     = nullptr;
static lv_obj_t           *scr_heatmap   = nullptr;
static lv_obj_t           *hm_view       = nullptr;
static lv_obj_t           *scr_hist_menu = nullptr;
static lv_obj_t           *hist_menu_btns[4] = {};
static int                 hist_menu_sel = 0;
static lv_obj_t           *scr_log       = nullptr;
static lv_obj_t           *log_label     = nullptr;
static lv_obj_t           *log_cont      = nullptr;
static uint32_t            log_day_off   = 0;
static bool                log_hide_pres = false;
static lv_obj_t           *btn_log_pres  = nullptr;
static lv_obj_t           *lbl_log_pres_btn = nullptr;
static lv_obj_t           *lbl_hist_mem  = nullptr;
static lv_obj_t           *chart_temp    = nullptr;
static lv_chart_series_t  *ser_int_g     = nullptr;
static lv_chart_series_t  *ser_ext_g     = nullptr;
static lv_obj_t           *lbl_graph_t0  = nullptr; // tiempo inicio visible
static lv_obj_t           *lbl_graph_t1  = nullptr; // tiempo medio
static lv_obj_t           *lbl_graph_t2  = nullptr; // tiempo fin visible
static lv_obj_t           *lbl_graph_zoom= nullptr; // indicador zoom
static lv_obj_t           *lbl_y[6]      = {};      // etiquetas eje Y (dinámicas)
static int32_t             graph_ymin    = -10;     // escala Y actual
static int32_t             graph_ymax    = 40;
static int32_t             graph_offset_samp = 0;   // muestras desplazadas hacia atrás
static uint32_t            graph_zoom_h   = 24;     // horas visibles
static uint32_t            graph_t0m=0, graph_t1m=0, graph_t2m=0; // epoch_min de los 3 puntos
static int32_t             chart_touch_prev_x = INT32_MIN;
static lv_obj_t *lbl_chpin_title=nullptr, *lbl_chpin_dots=nullptr, *lbl_chpin_msg=nullptr;
static char chpin_buf[5]={};
static int  chpin_len=0, chpin_phase=0;
static char chpin_new[5]={};
static lv_obj_t *lbl_alarm_type, *lbl_alarm_detail, *btn_deactivate;
static lv_obj_t *lbl_pin_dots, *lbl_pin_msg;
static lv_obj_t *lbl_countdown;
static lv_obj_t *dial_arc=nullptr, *lbl_dial_title_w=nullptr;
static lv_obj_t *lbl_dial_val=nullptr, *lbl_dial_unit=nullptr;
static lv_obj_t *dial_box_min=nullptr, *dial_box_off=nullptr, *dial_box_bright=nullptr;

enum DialMode { DIAL_NONE, DIAL_BRIGHTNESS, DIAL_DIM, DIAL_OFF, DIAL_DIM_BRIGHT, DIAL_SETPOINT };
static DialMode dial_mode = DIAL_NONE;
static int      dial_val  = 0;
static int      dial_dim_min_tmp    = 0;
static int      dial_dim_off_tmp    = 10;
static int      dial_dim_bright_tmp = 5;

// Heat active indicator (home tile, shown/blinking when relay 2 ON)
static lv_obj_t *lbl_heat_active = nullptr;

// Settings tile buttons (saved for encoder focus navigation)
static lv_obj_t *btn_settings_br=nullptr, *btn_settings_dim=nullptr;
static lv_obj_t *btn_settings_pin=nullptr, *btn_settings_anim=nullptr, *btn_settings_saver=nullptr;
static lv_obj_t *lbl_settings_mem=nullptr;

// Home-hint labels per tile (saved for encoder focus HOME-nav)
static lv_obj_t *hint_sensors=nullptr, *hint_relays=nullptr;
static lv_obj_t *hint_alarm=nullptr,   *hint_settings=nullptr;

// ── Encoder focus / tile-cycle system ────────────────────────
#define MAX_FOCUS 8
struct FocusList { lv_obj_t *items[MAX_FOCUS]; bool home_nav[MAX_FOCUS]; int count; };
static FocusList tile_focus[5]; // 0=home 1=relays 2=settings 3=hist 4=alarm

// Tile cycle order: home→relays→settings→sensors→alarm
static const uint8_t TC_COL[]={1,2,1,0,1};
static const uint8_t TC_ROW[]={1,1,2,1,0};
static int tile_cycle_idx = 0;

enum EncoderMode { ENC_IDLE, ENC_FOCUS };
static EncoderMode enc_mode      = ENC_IDLE;
static int         focus_idx     = 0;
static unsigned long focus_idle_ms = 0;
static unsigned long focus_blink_ts = 0;
static bool          focus_blink_st = false;

// Sensor/relay alert blink state
static bool     harc_alert[5]    = {}; // sensor bands: [1-4]
static bool     rarc_alert[4]    = {}; // relay bands: [1]=agua,[3]=sirena (ON=alert)
static bool     sensor_blink_st  = false;
static unsigned long sensor_blink_ts = 0;

static void go_to(Screen s) { pend_scr=s; scr_change=true; }

// ── HISTORIAL ─────────────────────────────────────────────────
enum EvtType : uint8_t {
    EVT_SYSTEM_BOOT   = 1,
    EVT_RELAY_AGUA    = 2,
    EVT_RELAY_CALEF   = 3,
    EVT_RELAY_SIRENA  = 4,
    EVT_HEAT_MODE     = 5,
    EVT_ALARM_ARM     = 6,
    EVT_ALARM_DISARM  = 7,
    EVT_ALARM_TRIGGER = 8,
    EVT_PRESENCE      = 9,
    EVT_FLOOD         = 10,
    EVT_SMOKE         = 11,
};

struct __attribute__((packed)) EvtRec {
    uint32_t ts;    // epoch seconds
    uint8_t  type;
    uint8_t  value; // 1=ON/activo, 0=OFF, o valor de modo
    uint8_t  aux;   // contexto: heat_mode para RELAY_CALEF; zona para ALARM_TRIGGER
    uint8_t  pad;
};  // 8 bytes

struct __attribute__((packed)) TmpRec {
    uint32_t ts_min; // epoch en minutos
    int16_t  t_int;  // °C × 10  (INT16_MIN = sin dato)
    int16_t  t_ext;  // °C × 10
};  // 8 bytes

static const uint32_t MAX_TEMP = 105120UL; // ~1 año a 5 min  → ~822 KB
static const uint32_t MAX_EVT  = 131072UL; // ~1.8 años a 200 evt/día → ~1 MB

static uint32_t hist_th = 0; // write head temp
static uint32_t hist_eh = 0; // write head events

static uint8_t       heatmap[7][96] = {}; // buffer de pintado: presencia (15min × día) de la semana mostrada; se rellena desde /evt.bin en heatmap_load_week()
static int8_t        hm_week_off    = 0;  // 0=semana actual, -1=anterior, etc.
static lv_obj_t     *lbl_hm_week    = nullptr;

// Estado previo para detección de cambios y logging
static bool hp_input[7]  = {};
static bool hp_output[7] = {};

// ── Buffer de eventos en RAM ──────────────────────────────────
// Escribir en flash bloquea el bus PSRAM/flash compartido y produce un glitch
// en el panel RGB. Para que el display sea fluido, los eventos se encolan en
// RAM y se vuelcan a flash AGRUPADOS cada pocos segundos (o de inmediato si son
// críticos). El estado en pantalla NO depende de esto: se refresca al instante
// desde a6v3.input/output en update_*(), así que el PIR se ve sin retardo.
static const uint16_t EVT_BUF_N   = 250;
static EvtRec         evt_buf[EVT_BUF_N];
static uint16_t       evt_buf_n   = 0;
static uint32_t       g_nflush    = 0;  // telemetria: nº grabaciones a flash
static uint32_t       g_nevt      = 0;  // telemetria: nº eventos encolados
static uint32_t       g_dropped   = 0;  // telemetria: nº descartados (buffer lleno despierto)

static void flush_events() {
    if (evt_buf_n == 0) return;
    File f = LittleFS.open("/evt.bin", "r+");
    if (!f) f = LittleFS.open("/evt.bin", "w");
    if (!f) { Serial.println("[HIST] evt open FAIL"); return; }
    for (uint8_t i = 0; i < evt_buf_n; i++) {
        f.seek(hist_eh * sizeof(EvtRec));
        f.write((uint8_t*)&evt_buf[i], sizeof(EvtRec));
        hist_eh = (hist_eh + 1) % MAX_EVT;
    }
    f.close();
    Preferences p; p.begin("hist", false);
    p.putUInt("eh", hist_eh); p.end();          // un único commit NVS por lote
    g_nflush++;
    Serial.printf("[HIST] flush %u evt  eh=%lu  nflush=%lu\n", (unsigned)evt_buf_n, hist_eh, (unsigned long)g_nflush);
    evt_buf_n = 0;
}

static void log_event(EvtType type, uint8_t value, uint8_t aux = 0) {
    time_t now = time(NULL);
    if (now < 1000000000L) return; // NTP no sincronizado
    EvtRec r;
    r.ts = (uint32_t)now; r.type = (uint8_t)type;
    r.value = value; r.aux = aux; r.pad = 0;
    g_nevt++;
    // Encolar en RAM (sin tocar flash → la pantalla no parpadea).
    // Tope (EVT_BUF_N): graba todo aunque esté despierta — glitch rarísimo (con dim a
    // 1 min nunca se llega), pero así no se pierde ningún evento.
    if (evt_buf_n >= EVT_BUF_N) flush_events();
    evt_buf[evt_buf_n++] = r;
    // Eventos críticos (raros): persistir ya, para no perderlos ante un corte
    switch (type) {
        case EVT_FLOOD: case EVT_SMOKE: case EVT_RELAY_SIRENA:
        case EVT_ALARM_ARM: case EVT_ALARM_DISARM: case EVT_ALARM_TRIGGER:
        case EVT_SYSTEM_BOOT:
            flush_events();
            break;
        default: break;   // PRESENCE, relés agua/calef, heat_mode → diferido
    }
    Serial.printf("[HIST] evt type=%u val=%u aux=%u ts=%lu (buf=%u)\n",
        (unsigned)type, value, aux, r.ts, evt_buf_n);
}

static void log_temp(float ti, float te) {
    time_t now = time(NULL);
    if (now < 1000000000L) return;
    TmpRec r;
    r.ts_min = (uint32_t)(now / 60);
    r.t_int  = isnan(ti) ? INT16_MIN : (int16_t)(ti * 10.0f);
    r.t_ext  = isnan(te) ? INT16_MIN : (int16_t)(te * 10.0f);
    File f = LittleFS.open("/temp.bin", "r+");
    if (!f) f = LittleFS.open("/temp.bin", "w");
    if (!f) { Serial.println("[HIST] tmp open FAIL"); return; }
    f.seek(hist_th * sizeof(TmpRec));
    f.write((uint8_t*)&r, sizeof(r));
    f.close();
    hist_th = (hist_th + 1) % MAX_TEMP;
    Preferences p; p.begin("hist", false);
    p.putUInt("th", hist_th); p.end();
}

// Reconstruye el mapa de presencia de la semana indicada (0=actual, -1=anterior…)
// a partir del registro real /evt.bin: cada EVT_PRESENCE con value==1 (alguien
// entra) suma 1 a su franja de 15 min. Así "< Sem"/"Sem >" muestran la semana
// real, hacia atrás mientras haya datos (no una suma acumulada por día-semana).
static void heatmap_load_week(int week_off) {
    memset(heatmap, 0, sizeof(heatmap));
    flush_events();                 // volcar lo pendiente en RAM para incluir lo reciente
    time_t now = time(NULL);
    if (now < 1000000000L) return;  // sin hora NTP no podemos ubicar la semana

    // Lunes 00:00 de la semana seleccionada y fin exclusivo (lunes siguiente)
    struct tm t; localtime_r(&now, &t);
    t.tm_hour = 0; t.tm_min = 0; t.tm_sec = 0;
    int wd = t.tm_wday == 0 ? 6 : t.tm_wday - 1;   // Lun=0..Dom=6
    time_t week_start = mktime(&t) - (time_t)wd*86400L + (time_t)week_off*7L*86400L;
    time_t week_end   = week_start + 7L*86400L;

    File f = LittleFS.open("/evt.bin", "r");
    if (!f) return;
    uint32_t total = f.size() / sizeof(EvtRec);
    if (total == 0) { f.close(); return; }
    uint32_t count = total < MAX_EVT ? total : MAX_EVT;
    uint32_t head  = hist_eh;
    for (uint32_t i = 0; i < count; i++) {
        if ((i & 0x3F) == 0) esp_task_wdt_reset();
        uint32_t idx = (head + MAX_EVT - 1 - i) % MAX_EVT;
        EvtRec r;
        f.seek(idx * sizeof(EvtRec));
        if (f.read((uint8_t*)&r, sizeof(r)) != sizeof(r)) continue;
        if (r.ts < 1000000000UL) continue;
        time_t ts = (time_t)r.ts;
        if (ts >= week_end) continue;   // más reciente que la ventana → sigue retrocediendo
        if (ts <  week_start) break;    // ya pasamos la ventana (más antiguo) → fin
        if ((EvtType)r.type != EVT_PRESENCE || r.value != 1) continue;
        struct tm tt; localtime_r(&ts, &tt);
        int d = tt.tm_wday;                       // 0=Dom..6=Sab (índice de heatmap)
        int s = (tt.tm_hour*60 + tt.tm_min) / 15; // 0-95
        if (d >= 0 && d < 7 && s >= 0 && s < 96 && heatmap[d][s] < 255) heatmap[d][s]++;
    }
    f.close();
}

static const char *evt_name(uint8_t t, uint8_t v, uint8_t aux);

static void hist_init() {
    if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) {
        Serial.println("[HIST] LittleFS mount FAIL"); return;
    }
    Preferences p; p.begin("hist", true);
    hist_th = p.getUInt("th", 0);
    hist_eh = p.getUInt("eh", 0);
    p.end();
    // El mapa de presencia se reconstruye desde /evt.bin al abrir la pantalla
    // (heatmap_load_week), no se persiste por separado.
    Serial.printf("[HIST] init  th=%lu  eh=%lu\n", hist_th, hist_eh);
}

static void serial_dump_events() {
    File f = LittleFS.open("/evt.bin", "r");
    if (!f) { Serial.println("[EVT] archivo no encontrado"); return; }
    uint32_t total = f.size() / sizeof(EvtRec);
    Serial.printf("[EVT] total registros en disco: %lu  head(eh)=%lu\n", total, hist_eh);
    if (total == 0) { f.close(); return; }
    uint32_t count = total < MAX_EVT ? total : MAX_EVT;
    Serial.println("[EVT] idx  | timestamp           | tipo  val  aux | descripcion");
    Serial.println("[EVT] -----+---------------------+----------------+------------");
    uint32_t printed = 0;
    for (uint32_t i = 0; i < count; i++) {
        if ((i & 0x3F) == 0) esp_task_wdt_reset();
        uint32_t idx = (hist_eh + MAX_EVT - 1 - i) % MAX_EVT;
        EvtRec r;
        f.seek(idx * sizeof(EvtRec));
        if (f.read((uint8_t*)&r, sizeof(r)) != sizeof(r)) continue;
        if (r.ts < 1000000000UL) continue;
        time_t ts = (time_t)r.ts;
        struct tm t; localtime_r(&ts, &t);
        Serial.printf("[EVT] %5lu | %02d/%02d/%04d %02d:%02d:%02d | t=%-3u v=%-3u a=%-3u | %s\n",
            idx,
            t.tm_mday, t.tm_mon+1, t.tm_year+1900,
            t.tm_hour, t.tm_min, t.tm_sec,
            r.type, r.value, r.aux,
            evt_name(r.type, r.value, r.aux));
        printed++;
        if (printed >= 500) { Serial.println("[EVT] ... (limitado a 500)"); break; }
    }
    f.close();
    Serial.printf("[EVT] mostrados %lu de %lu\n", printed, count);
}

// ─────────────────────────────────────────────────────────────

// Forward declarations for focus system (defined after build_* functions)
static void focus_exit();
static void sync_tile_cycle_idx();
static void cb_open_graph(lv_event_t *e);
static void cb_open_hist_menu(lv_event_t *e);
static void cb_hist_temp(lv_event_t *e);
static void cb_hist_pres(lv_event_t *e);
static void cb_hist_log(lv_event_t *e);

// ── Navegación de tiles ──────────────────────────────────────
// Layout: alarm(1,0) sensors(0,1) home(1,1) relays(2,1) settings(1,2)
static const struct { uint8_t col, row; lv_obj_t **tile; } TILE_MAP[] = {
    {1,0,&tile_alarm}, {0,1,&tile_sensors}, {1,1,&tile_home},
    {2,1,&tile_relays}, {1,2,&tile_settings}
};
static void nav_tile(int8_t dc, int8_t dr) {
    int nc = (int)tile_col + dc, nr = (int)tile_row + dr;
    for (auto &t : TILE_MAP) {
        if (t.col == (uint8_t)nc && t.row == (uint8_t)nr) {
            if(enc_mode==ENC_FOCUS) focus_exit();
            tile_col = (uint8_t)nc; tile_row = (uint8_t)nr;
            sync_tile_cycle_idx();
            lv_tileview_set_tile(tv, *t.tile, cfg_anim ? LV_ANIM_ON : LV_ANIM_OFF);
            return;
        }
    }
}

// ── Helpers UI ───────────────────────────────────────────────
static lv_obj_t *make_band(lv_obj_t *parent, int x, int y, int w, int h, uint32_t col) {
    lv_obj_t *b = lv_obj_create(parent);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_radius(b, 0, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(col), 0);
    lv_obj_set_style_border_width(b, 0, 0);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(b, LV_OBJ_FLAG_EVENT_BUBBLE);
    return b;
}

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


static lv_obj_t *add_home_hint(lv_obj_t *tile, lv_align_t align, const char *txt) {
    lv_obj_t *l = lv_label_create(tile);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(COL_TEXT), 0);
    lv_obj_align(l, align, 0, 0);
    return l;
}

// ── DIAL ─────────────────────────────────────────────────────
static void dim_box_set(lv_obj_t *b, bool active) {
    if (!b) return;
    lv_obj_set_style_bg_color(b, lv_color_hex(active ? COL_CARD : COL_OFF), 0);
    lv_obj_set_style_border_width(b, active ? 2 : 0, 0);
}
static void update_dim_box_highlight() {
    dim_box_set(dial_box_min,    dial_mode == DIAL_DIM);
    dim_box_set(dial_box_off,    dial_mode == DIAL_OFF);
    dim_box_set(dial_box_bright, dial_mode == DIAL_DIM_BRIGHT);
}

static void update_dial_display() {
    if (!lbl_dial_val) return;
    char buf[16];
    if (dial_mode == DIAL_BRIGHTNESS) {
        snprintf(buf, sizeof(buf), "%d", dial_val);
    } else if (dial_mode == DIAL_DIM) {
        dial_dim_min_tmp = dial_val;
        snprintf(buf, sizeof(buf), dial_val==0 ? "OFF" : "%d", dial_val);
    } else if (dial_mode == DIAL_OFF) {
        dial_dim_off_tmp = dial_val;
        snprintf(buf, sizeof(buf), dial_val==0 ? "OFF" : "%d", dial_val);
    } else if (dial_mode == DIAL_DIM_BRIGHT) {
        dial_dim_bright_tmp = dial_val;
        snprintf(buf, sizeof(buf), "%d%%", dial_val);
    } else {
        snprintf(buf, sizeof(buf), "%d\xc2\xb0""C", dial_val);
    }
    lv_label_set_text(lbl_dial_val, buf);
    if (dial_mode == DIAL_BRIGHTNESS) set_brightness(dial_val);
    // Mantener las etiquetas de las 3 cajas de PANTALLA sincronizadas
    if (dial_box_min) {
        char b[16];
        snprintf(b, sizeof(b), dial_dim_min_tmp==0 ? "DIM\nOFF" : "DIM\n%d min", dial_dim_min_tmp);
        lv_label_set_text((lv_obj_t*)lv_obj_get_child(dial_box_min, 0), b);
    }
    if (dial_box_off) {
        char b[16];
        snprintf(b, sizeof(b), dial_dim_off_tmp==0 ? "Apagar\nOFF" : "Apagar\n%d min", dial_dim_off_tmp);
        lv_label_set_text((lv_obj_t*)lv_obj_get_child(dial_box_off, 0), b);
    }
    if (dial_box_bright) {
        char b[16]; snprintf(b, sizeof(b), "Brillo\n%d%%", dial_dim_bright_tmp);
        lv_label_set_text((lv_obj_t*)lv_obj_get_child(dial_box_bright, 0), b);
    }
}

static void update_setpoint_btn() {
    if (!lbl_setpoint) return;
    char buf[10]; snprintf(buf, sizeof(buf), "%d\xc2\xb0""C", heat_setpoint);
    lv_label_set_text(lbl_setpoint, buf);
}

static void update_screen_pill_label() {
    if (!lbl_cfg_dim) return;
    char d[12], o[12], b[48];
    if (cfg_dim_delay==0) snprintf(d,sizeof(d),"DIM -");
    else                  snprintf(d,sizeof(d),"DIM %dm", cfg_dim_delay);
    if (cfg_off_delay==0) snprintf(o,sizeof(o),"Apg -");
    else                  snprintf(o,sizeof(o),"Apg %dm", cfg_off_delay);
    snprintf(b,sizeof(b),"%s  %s  %d%%", d, o, cfg_dim_brightness);
    lv_label_set_text(lbl_cfg_dim, b);
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
    } else if (dial_mode == DIAL_DIM || dial_mode == DIAL_OFF || dial_mode == DIAL_DIM_BRIGHT) {
        // Sincronizar el valor actual del arco a su tmp y guardar los 3
        if      (dial_mode == DIAL_DIM)        dial_dim_min_tmp    = dial_val;
        else if (dial_mode == DIAL_OFF)        dial_dim_off_tmp    = dial_val;
        else                                   dial_dim_bright_tmp = dial_val;
        // Apagar debe ser >= DIM (si no, lo igualamos al DIM)
        if (dial_dim_off_tmp != 0 && dial_dim_min_tmp != 0 && dial_dim_off_tmp < dial_dim_min_tmp)
            dial_dim_off_tmp = dial_dim_min_tmp;
        cfg_dim_delay      = dial_dim_min_tmp;
        cfg_off_delay      = dial_dim_off_tmp;
        cfg_dim_brightness = dial_dim_bright_tmp;
        prefs.putInt("dim_delay",  cfg_dim_delay);
        prefs.putInt("off_delay",  cfg_off_delay);
        prefs.putInt("dim_bright", cfg_dim_brightness);
        update_screen_pill_label();
        // Ocultar cajas selectoras al salir
        if (dial_box_min)    lv_obj_add_flag(dial_box_min,    LV_OBJ_FLAG_HIDDEN);
        if (dial_box_off)    lv_obj_add_flag(dial_box_off,    LV_OBJ_FLAG_HIDDEN);
        if (dial_box_bright) lv_obj_add_flag(dial_box_bright, LV_OBJ_FLAG_HIDDEN);
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
static void dim_boxes_hide() {
    if (dial_box_min)    lv_obj_add_flag(dial_box_min,    LV_OBJ_FLAG_HIDDEN);
    if (dial_box_off)    lv_obj_add_flag(dial_box_off,    LV_OBJ_FLAG_HIDDEN);
    if (dial_box_bright) lv_obj_add_flag(dial_box_bright, LV_OBJ_FLAG_HIDDEN);
}
static void cb_dial_cancel(lv_event_t *e) {
    if (dial_mode==DIAL_BRIGHTNESS) set_brightness(cfg_brightness);
    if (dial_mode==DIAL_DIM || dial_mode==DIAL_OFF || dial_mode==DIAL_DIM_BRIGHT) dim_boxes_hide();
    dial_mode=DIAL_NONE; go_to(SCR_TV);
}

// Antes de cambiar de caja, guarda el valor del arco en su tmp correspondiente
static void dim_sync_current() {
    if      (dial_mode == DIAL_DIM)        dial_dim_min_tmp    = dial_val;
    else if (dial_mode == DIAL_OFF)        dial_dim_off_tmp    = dial_val;
    else if (dial_mode == DIAL_DIM_BRIGHT) dial_dim_bright_tmp = dial_val;
}
static void cb_dim_box_min(lv_event_t *e) {
    dim_sync_current();
    dial_mode = DIAL_DIM;
    dial_val  = dial_dim_min_tmp;
    lv_arc_set_range(dial_arc, 0, 30);
    lv_arc_set_value(dial_arc, dial_val);
    lv_label_set_text(lbl_dial_unit, "DIM: minutos (0=nunca)");
    update_dial_display();
    update_dim_box_highlight();
}
static void cb_dim_box_off(lv_event_t *e) {
    dim_sync_current();
    dial_mode = DIAL_OFF;
    dial_val  = dial_dim_off_tmp;
    lv_arc_set_range(dial_arc, 0, 60);
    lv_arc_set_value(dial_arc, dial_val);
    lv_label_set_text(lbl_dial_unit, "Apagar: minutos (0=nunca)");
    update_dial_display();
    update_dim_box_highlight();
}
static void cb_dim_box_bright(lv_event_t *e) {
    dim_sync_current();
    dial_mode = DIAL_DIM_BRIGHT;
    dial_val  = dial_dim_bright_tmp;
    lv_arc_set_range(dial_arc, 1, 100);
    lv_arc_set_value(dial_arc, dial_val);
    lv_label_set_text(lbl_dial_unit, "brillo % (dim)");
    update_dial_display();
    update_dim_box_highlight();
}

static void open_dial(DialMode mode) {
    dial_mode = mode;
    dim_boxes_hide();  // ocultas salvo en modos PANTALLA
    if (mode == DIAL_BRIGHTNESS) {
        dial_val=cfg_brightness;
        lv_label_set_text(lbl_dial_title_w, "BRILLO");
        lv_label_set_text(lbl_dial_unit, "intensidad");
        lv_arc_set_range(dial_arc, 10, 255);
    } else if (mode == DIAL_DIM) {
        dial_dim_min_tmp    = cfg_dim_delay;
        dial_dim_off_tmp    = cfg_off_delay;
        dial_dim_bright_tmp = cfg_dim_brightness;
        dial_val = cfg_dim_delay;
        lv_label_set_text(lbl_dial_title_w, "PANTALLA");
        lv_label_set_text(lbl_dial_unit, "DIM: minutos (0=nunca)");
        lv_arc_set_range(dial_arc, 0, 30);
        if (dial_box_min)    lv_obj_clear_flag(dial_box_min,    LV_OBJ_FLAG_HIDDEN);
        if (dial_box_off)    lv_obj_clear_flag(dial_box_off,    LV_OBJ_FLAG_HIDDEN);
        if (dial_box_bright) lv_obj_clear_flag(dial_box_bright, LV_OBJ_FLAG_HIDDEN);
        update_dim_box_highlight();
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

// ── CALEFACCIÓN ───────────────────────────────────────────────
static void refresh_heat_ui();
static void check_heating_auto();

static void heat_set_mode(HeatMode m) {
    if (heat_mode == m) {
        heat_mode = HM_OFF;
        relay_set(2, false);
    } else {
        heat_mode = m;
        if (m == HM_MANUAL) relay_set(2, true);
        else relay_set(2, false); // modo auto: apagar relé y dejar que check_heating_auto evalúe
    }
    log_event(EVT_HEAT_MODE, (uint8_t)heat_mode);
    prefs.begin("domus",false); prefs.putInt("heat_mode",(int)heat_mode); prefs.end();
    refresh_heat_ui();
    check_heating_auto();
}

static void check_heating_auto() {
    if(heat_mode == HM_CONSIGNA) {
        if(isnan(tuya_temp_int)) return;
        const float hyst = 0.5f;
        if(tuya_temp_int < heat_setpoint - hyst && !a6v3.output[2]) relay_set(2, true);
        else if(tuya_temp_int > heat_setpoint + hyst && a6v3.output[2]) relay_set(2, false);
    } else if(heat_mode == HM_PROGRAMA) {
        if(isnan(tuya_temp_int)) return;
        time_t now = time(NULL);
        if(now < 1000000000L) return;
        struct tm t; localtime_r(&now, &t);
        int block = t.tm_hour * 2 + t.tm_min / 30; // 48 slots de 30min (0..47)
        bool confort = (cfg_prog[t.tm_wday] >> block) & 1ULL;
        float target = confort ? (float)heat_setpoint : (float)cfg_eco_sp;
        const float hyst = 0.5f;
        if(tuya_temp_int < target - hyst && !a6v3.output[2]) relay_set(2, true);
        else if(tuya_temp_int > target + hyst && a6v3.output[2]) relay_set(2, false);
    }
}

// ── BUILD TILE HOME ──────────────────────────────────────────
static void cb_alarm_badge(lv_event_t *e)   { if(swipe_blocked()) return; tile_col=1; tile_row=0; lv_tileview_set_tile(tv, tile_alarm,    cfg_anim?LV_ANIM_ON:LV_ANIM_OFF); }
static void cb_settings_icon(lv_event_t *e) { if(swipe_blocked()) return; tile_col=1; tile_row=2; lv_tileview_set_tile(tv, tile_settings, cfg_anim?LV_ANIM_ON:LV_ANIM_OFF); }

static void build_tile_home() {
    bg(tile_home);
    // Franja superior alarma — top band, h=82 (gap 6 hasta y=88)
    arc_alarm_zone = make_band(tile_home, 0, 0, 480, 82, COL_ZONE_ALARM);

    // 4 franjas sensores (izquierda, x=0..240)
    // Sección media y=88..392 (304px), gap=6 uniforme: 4×71+1×2+3×6=304
    // harc[1]: y=88 h=71 | harc[2]: y=165 h=71 | harc[3]: y=242 h=71 | harc[4]: y=319 h=73
    harc[1]=make_band(tile_home, 0,  88, 240, 71, COL_OK_DIM);
    harc[2]=make_band(tile_home, 0, 165, 240, 71, COL_OK_DIM);
    harc[3]=make_band(tile_home, 0, 242, 240, 71, COL_OK_DIM);
    harc[4]=make_band(tile_home, 0, 319, 240, 73, COL_OK_DIM);
    // Labels centrados en el anillo (dx calculado por centro de anillo en cada y)
    { struct { lv_obj_t **b; const char *n; int dx; } sl[]={
        {&harc[1],"Inund",-40},{&harc[2],"Hall",-76},
        {&harc[3],"Humo", -77},{&harc[4],"220V",-41}};
      for(auto &s:sl){
        lv_obj_t *l=lv_label_create(*s.b);
        lv_label_set_text(l,s.n);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
        lv_obj_align(l,LV_ALIGN_CENTER,s.dx,0);
        lv_obj_add_flag(l,LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_clear_flag(l,LV_OBJ_FLAG_CLICKABLE);
      }
    }

    // 3 franjas relés (derecha, x=240..480)
    // gap=6 uniforme: 3×97+1×1+2×6=304
    // rarc[1]: y=88 h=97 | rarc[2]: y=191 h=97 | rarc[3]: y=294 h=98
    rarc[1]=make_band(tile_home, 240,  88, 240, 97, COL_RELAY_DIM);
    rarc[2]=make_band(tile_home, 240, 191, 240, 97, COL_RELAY_DIM);
    rarc[3]=make_band(tile_home, 240, 294, 240, 98, COL_RELAY_DIM);
    { struct { lv_obj_t **b; const char *n; int dx; } rl[]={
        {&rarc[1],"Agua",  +48},{&rarc[2],"Calef",+78},
        {&rarc[3],"Sirena",+49}};
      for(auto &r:rl){
        lv_obj_t *l=lv_label_create(*r.b);
        lv_label_set_text(l,r.n);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
        lv_obj_align(l,LV_ALIGN_CENTER,r.dx,0);
        lv_obj_add_flag(l,LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_clear_flag(l,LV_OBJ_FLAG_CLICKABLE);
      }
    }

    // Franja inferior broker/ajustes — gap 6 desde y=392, h=82
    arc_settings_zone = make_band(tile_home, 0, 398, 480, 82, COL_ZONE_SET);

    lbl_alarm_badge=lv_label_create(tile_home);
    lv_label_set_text(lbl_alarm_badge,"ALARMA\nOFF");
    lv_label_set_long_mode(lbl_alarm_badge,LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(lbl_alarm_badge,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_alarm_badge,lv_color_hex(COL_MUTED),0);
    lv_obj_set_width(lbl_alarm_badge,62);
    lv_obj_set_style_text_align(lbl_alarm_badge,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_pos(lbl_alarm_badge,209,25);
    lv_obj_add_flag(lbl_alarm_badge,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lbl_alarm_badge,cb_alarm_badge,LV_EVENT_CLICKED,nullptr);

    lbl_sistema=lv_label_create(tile_home);
    lv_label_set_text(lbl_sistema,"Sistema\nOFF");
    lv_label_set_long_mode(lbl_sistema,LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(lbl_sistema,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_sistema,lv_color_hex(COL_ALERT),0);
    lv_obj_set_width(lbl_sistema,70);
    lv_obj_set_style_text_align(lbl_sistema,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_pos(lbl_sistema,205,420);
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

    lbl_temp_ext   =centered_label(tile_home,"--.-\xc2\xb0""C",    &lv_font_montserrat_40,COL_TEXT, -100);
    // Meteo más grande a la izquierda + icono del tiempo grande a la derecha
    lbl_weather    =centered_label(tile_home,"-- km/h  0.0 mm/h", &lv_font_montserrat_20,COL_MUTED, -50);
    lv_obj_align(lbl_weather, LV_ALIGN_CENTER, -34, -50);
    lbl_wx_forecast=centered_label(tile_home,"--\xc2\xb0 / --\xc2\xb0",&lv_font_montserrat_20,COL_MUTED,-22);
    lv_obj_align(lbl_wx_forecast, LV_ALIGN_CENTER, -34, -22);
    lbl_wx_icon    =lv_label_create(tile_home);
    lv_obj_set_style_text_font(lbl_wx_icon,&weather_font,0);
    lv_obj_set_style_text_color(lbl_wx_icon,lv_color_hex(COL_TEXT),0);
    lv_label_set_text(lbl_wx_icon,"");
    lv_obj_align(lbl_wx_icon, LV_ALIGN_CENTER, 92, -34);

    // Heating active indicator — hidden by default, blinks in loop() when heat_mode != HM_OFF
    lbl_heat_active=lv_obj_create(tile_home);
    lv_obj_set_size(lbl_heat_active,280,46);
    lv_obj_align(lbl_heat_active,LV_ALIGN_CENTER,0,+56);
    lv_obj_set_style_bg_color(lbl_heat_active,lv_color_hex(COL_HEAT_BOX),0);
    lv_obj_set_style_radius(lbl_heat_active,10,0);
    lv_obj_set_style_border_width(lbl_heat_active,0,0);
    lv_obj_set_style_pad_all(lbl_heat_active,0,0);
    lv_obj_clear_flag(lbl_heat_active,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(lbl_heat_active,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(lbl_heat_active,LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(lbl_heat_active,LV_OBJ_FLAG_HIDDEN);
    { lv_obj_t *hl=lv_label_create(lbl_heat_active);
      lv_label_set_text(hl,"Calefaccion activada");
      lv_obj_set_style_text_font(hl,&lv_font_montserrat_20,0);
      lv_obj_set_style_text_color(hl,lv_color_hex(COL_TEXT),0);
      lv_obj_center(hl); }

    lbl_temp_int   =centered_label(tile_home,"--.-\xc2\xb0""C",    &lv_font_montserrat_48,COL_TEXT,  100);
    lbl_humidity   =centered_label(tile_home,"--% HR",             &lv_font_montserrat_20,COL_MUTED, 148);
}

// ── BUILD TILE HISTORIAL ─────────────────────────────────────
static void build_tile_hist() {
    bg(tile_sensors);
    centered_label(tile_sensors,"HISTORIAL",&lv_font_montserrat_20,COL_TEXT,-175);
    btn_hist[0]=make_big_btn(tile_sensors,"Temperatura",COL_RELAY_DIM, -72,280,65,cb_hist_temp,nullptr);
    btn_hist[1]=make_big_btn(tile_sensors,"Presencia",  COL_RELAY_DIM,   8,280,65,cb_hist_pres,nullptr);
    btn_hist[2]=make_big_btn(tile_sensors,"Registro",   COL_RELAY_DIM,  88,280,65,cb_hist_log, nullptr);
    lbl_hist_tile_mem=centered_label(tile_sensors,"Flash: ...", &lv_font_montserrat_14, COL_MUTED, +148);
    hint_sensors=add_home_hint(tile_sensors,LV_ALIGN_RIGHT_MID,LV_SYMBOL_RIGHT " HOME");
}

// ── BUILD TILE RELÉS ─────────────────────────────────────────
static void cb_agua(lv_event_t *e)        { if(swipe_blocked()) return; relay_set(1,!a6v3.output[1]); }

static lv_obj_t *siren_confirm_panel = nullptr;
static void siren_confirm_close() {
    if(!siren_confirm_panel) return;
    lv_obj_delete(siren_confirm_panel); siren_confirm_panel = nullptr;
    lv_obj_clear_flag(lv_layer_top(), LV_OBJ_FLAG_CLICKABLE);
}
static void cb_siren_yes(lv_event_t *e) { relay_set(3, true);  siren_confirm_close(); }
static void cb_siren_no (lv_event_t *e) {                       siren_confirm_close(); }

static void cb_sirena(lv_event_t *e) {
    if(swipe_blocked()) return;
    if(a6v3.output[3]) { relay_set(3, false); return; } // apagar: sin confirmación
    if(siren_confirm_panel) return;
    lv_obj_add_flag(lv_layer_top(), LV_OBJ_FLAG_CLICKABLE);
    siren_confirm_panel = lv_obj_create(lv_layer_top());
    lv_obj_set_size(siren_confirm_panel, 300, 160);
    lv_obj_center(siren_confirm_panel);
    lv_obj_set_style_bg_color(siren_confirm_panel, lv_color_hex(COL_CARD), 0);
    lv_obj_set_style_bg_opa(siren_confirm_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(siren_confirm_panel, 16, 0);
    lv_obj_set_style_border_color(siren_confirm_panel, lv_color_hex(0x444444), 0);
    lv_obj_set_style_border_width(siren_confirm_panel, 1, 0);
    lv_obj_set_style_pad_all(siren_confirm_panel, 0, 0);
    lv_obj_clear_flag(siren_confirm_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *lbl=lv_label_create(siren_confirm_panel);
    lv_label_set_text(lbl,"Activar sirena?");
    lv_obj_set_style_text_font(lbl,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_color(lbl,lv_color_hex(COL_TEXT),0);
    lv_obj_align(lbl,LV_ALIGN_TOP_MID,0,22);
    auto mk=[&](const char *txt, int x, uint32_t col, lv_event_cb_t cb){
        lv_obj_t *b=lv_obj_create(siren_confirm_panel);
        lv_obj_set_size(b,120,52);
        lv_obj_align(b,x<0?LV_ALIGN_BOTTOM_LEFT:LV_ALIGN_BOTTOM_RIGHT,x<0?16:-16,-16);
        lv_obj_set_style_bg_color(b,lv_color_hex(col),0);
        lv_obj_set_style_radius(b,10,0); lv_obj_set_style_border_width(b,0,0);
        lv_obj_set_style_pad_all(b,0,0);
        lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE); lv_obj_add_flag(b,LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,nullptr);
        lv_obj_t *l=lv_label_create(b); lv_label_set_text(l,txt);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
        lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0); lv_obj_center(l);
    };
    mk("ACTIVAR", -1, COL_ALERT, cb_siren_yes);
    mk("CANCELAR",+1, COL_OFF,   cb_siren_no);
}
static void cb_hm_manual(lv_event_t *e)   { if(swipe_blocked()) return; heat_set_mode(HM_MANUAL); }
static void cb_hm_consigna(lv_event_t *e) { if(swipe_blocked()) return; heat_set_mode(HM_CONSIGNA); }
static void cb_hm_prog(lv_event_t *e) {
    if(swipe_blocked()) return;
    if(heat_mode == HM_PROGRAMA) {
        heat_mode = HM_OFF; relay_set(2, false);
    } else {
        heat_mode = HM_PROGRAMA;
        relay_set(2, false); // reset; check_heating_auto decidirá
    }
    log_event(EVT_HEAT_MODE, (uint8_t)heat_mode);
    prefs.begin("domus",false); prefs.putInt("heat_mode",(int)heat_mode); prefs.end();
    refresh_heat_ui();
    check_heating_auto();
    if(heat_mode == HM_PROGRAMA) go_to(SCR_PROG);
}
static void cb_setpoint_btn(lv_event_t *e) {
    if(swipe_blocked()) return;
    if(heat_mode != HM_CONSIGNA) {
        heat_mode = HM_CONSIGNA;
        prefs.begin("domus",false); prefs.putInt("heat_mode",(int)heat_mode); prefs.end();
        refresh_heat_ui();
    }
    open_dial(DIAL_SETPOINT);
}

static void refresh_heat_ui() {
    if(!btn_hm[0]) return;
    lv_obj_set_style_bg_color(btn_hm[0],lv_color_hex(heat_mode==HM_MANUAL?COL_OK:COL_OFF),0);
    lv_obj_set_style_bg_color(btn_hm[1],lv_color_hex(heat_mode==HM_CONSIGNA?COL_OK:COL_OFF),0);
    lv_obj_set_style_bg_color(btn_hm[2],lv_color_hex(heat_mode==HM_PROGRAMA?COL_OK:COL_OFF),0);
}

static void build_tile_relays() {
    bg(tile_relays);

    // SIRENA — cajetín superior, ancho completo, clickable
    btn_sirena_r=lv_obj_create(tile_relays);
    lv_obj_set_pos(btn_sirena_r,0,0);
    lv_obj_set_size(btn_sirena_r,480,90);
    lv_obj_set_style_radius(btn_sirena_r,0,0);
    lv_obj_set_style_bg_color(btn_sirena_r,lv_color_hex(COL_ZONE_ALARM),0);
    lv_obj_set_style_border_width(btn_sirena_r,0,0);
    lv_obj_set_style_pad_all(btn_sirena_r,0,0);
    lv_obj_clear_flag(btn_sirena_r,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_sirena_r,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_sirena_r,cb_sirena,LV_EVENT_CLICKED,nullptr);
    { lv_obj_t *l=lv_label_create(btn_sirena_r);
      lv_label_set_text(l,"SIRENA");
      lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
      lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
      lv_obj_center(l); }

    { lv_obj_t *l=lv_label_create(tile_relays);
      lv_label_set_text(l,"CALEFACCION");
      lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
      lv_obj_set_style_text_color(l,lv_color_hex(COL_MUTED),0);
      lv_obj_align(l,LV_ALIGN_CENTER,0,-128); }

    auto mk_hm=[](lv_obj_t *par, int y, int x, int w, lv_event_cb_t cb, const char *txt) -> lv_obj_t* {
        lv_obj_t *b=lv_obj_create(par);
        lv_obj_set_size(b,w,65);
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
        lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
        lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
        lv_obj_center(l);
        return b;
    };

    btn_hm[0]=mk_hm(tile_relays,-73,  0,260,cb_hm_manual,  "MANUAL");
    btn_hm[1]=mk_hm(tile_relays,  0,-44,170,cb_hm_consigna,"CONSIGNA");
    btn_hm[2]=mk_hm(tile_relays,+73,  0,260,cb_hm_prog,    "PROGRAMA");

    btn_setpoint=lv_obj_create(tile_relays);
    lv_obj_set_size(btn_setpoint,82,65);
    lv_obj_align(btn_setpoint,LV_ALIGN_CENTER,+89,0);
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

    // AGUA — cajetín inferior, ancho completo, clickable
    btn_agua_r=lv_obj_create(tile_relays);
    lv_obj_set_pos(btn_agua_r,0,390);
    lv_obj_set_size(btn_agua_r,480,90);
    lv_obj_set_style_radius(btn_agua_r,0,0);
    lv_obj_set_style_bg_color(btn_agua_r,lv_color_hex(COL_ZONE_SET),0);
    lv_obj_set_style_border_width(btn_agua_r,0,0);
    lv_obj_set_style_pad_all(btn_agua_r,0,0);
    lv_obj_clear_flag(btn_agua_r,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_agua_r,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_agua_r,cb_agua,LV_EVENT_CLICKED,nullptr);
    { lv_obj_t *l=lv_label_create(btn_agua_r);
      lv_label_set_text(l,"AGUA");
      lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
      lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
      lv_obj_center(l); }

    refresh_heat_ui();
    hint_relays=add_home_hint(tile_relays,LV_ALIGN_LEFT_MID,"HOME " LV_SYMBOL_LEFT);
}

// ── BUILD TILE ALARMA ────────────────────────────────────────
static void cb_go_pin(lv_event_t *e) {
    if(swipe_blocked()) return;
    pin_for_arm=(alarm_state==AS_OFF); memset(pin_buf,0,5); pin_len=0; go_to(SCR_PIN);
}
static void build_tile_alarm() {
    bg(tile_alarm);
    centered_label(tile_alarm,"ALARMA INTRUSION",&lv_font_montserrat_20,COL_TEXT,-160);
    lbl_arm_state=centered_label(tile_alarm,"DESARMADA",&lv_font_montserrat_40,COL_MUTED,-60);
    btn_arm=make_big_btn(tile_alarm,"ARMAR",COL_RELAY_DIM,60,240,70,cb_go_pin,nullptr);
    hint_alarm=add_home_hint(tile_alarm,LV_ALIGN_BOTTOM_MID,"HOME " LV_SYMBOL_DOWN);
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
static void cb_open_change_pin(lv_event_t *e){ if(swipe_blocked()) return; go_to(SCR_CHANGE_PIN); }
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

// ── GRAPH TEMPERATURA ────────────────────────────────────────
// Colores series (valores BGR para display RGB-invertido):
// 0xFF4000 → azul en pantalla (Ti interior)
// 0x0040FF → rojo en pantalla (Te exterior)

static void fmt_hm(uint32_t epoch_min, char *buf, int n) {
    if(!epoch_min){ snprintf(buf,n,"--:--"); return; }
    time_t t=(time_t)(epoch_min*60); struct tm tm; localtime_r(&t,&tm);
    snprintf(buf,n,"%02d:%02d",tm.tm_hour,tm.tm_min);
}
static void fmt_dm(uint32_t epoch_min, char *buf, int n) {  // DD/MM para zoom alejado
    if(!epoch_min){ snprintf(buf,n,"--/--"); return; }
    time_t t=(time_t)(epoch_min*60); struct tm tm; localtime_r(&t,&tm);
    snprintf(buf,n,"%02d/%02d",tm.tm_mday,tm.tm_mon+1);
}

static void graph_update_ylabels() {
    if(!chart_temp) return;
    lv_chart_set_range(chart_temp, LV_CHART_AXIS_PRIMARY_Y, graph_ymin, graph_ymax);
    int32_t span = graph_ymax - graph_ymin;
    for(int i=0;i<6;i++) {
        if(!lbl_y[i]) continue;
        int32_t v = graph_ymax - (int32_t)(i * span / 5);
        char buf[8]; snprintf(buf, sizeof(buf), "%ld", (long)v);
        lv_label_set_text(lbl_y[i], buf);
        int yoff = -131 + (int)(232L * (graph_ymax - v) / span);
        lv_obj_align(lbl_y[i], LV_ALIGN_CENTER, -190, yoff);
    }
}

static void graph_update_xlabels() {
    char b[8];
    bool days = graph_zoom_h >= 48;   // ventana >=2 dias → eje X por dia (DD/MM)
    if(lbl_graph_t0){ days?fmt_dm(graph_t0m,b,sizeof(b)):fmt_hm(graph_t0m,b,sizeof(b)); lv_label_set_text(lbl_graph_t0,b); }
    if(lbl_graph_t1){ days?fmt_dm(graph_t1m,b,sizeof(b)):fmt_hm(graph_t1m,b,sizeof(b)); lv_label_set_text(lbl_graph_t1,b); }
    if(lbl_graph_t2){ days?fmt_dm(graph_t2m,b,sizeof(b)):fmt_hm(graph_t2m,b,sizeof(b)); lv_label_set_text(lbl_graph_t2,b); }
    if(lbl_graph_zoom){
        char zb[8];
        if(graph_zoom_h>=24) snprintf(zb,sizeof(zb),"%lud",(unsigned long)(graph_zoom_h/24));
        else                 snprintf(zb,sizeof(zb),"%luh",(unsigned long)graph_zoom_h);
        lv_label_set_text(lbl_graph_zoom,zb);
    }
}

static void graph_load() {
    if(!chart_temp||!ser_int_g||!ser_ext_g) return;
    uint32_t avail=(hist_th<MAX_TEMP)?hist_th:MAX_TEMP;
    uint32_t window_samp=graph_zoom_h*12; // 12 muestras/hora a 5min
    if(window_samp<2) window_samp=2;
    // Clamp offset
    if(avail>window_samp){ uint32_t mo=avail-window_samp; if((uint32_t)graph_offset_samp>mo) graph_offset_samp=(int32_t)mo; }
    else graph_offset_samp=0;
    if(graph_offset_samp<0) graph_offset_samp=0;
    uint32_t off=(uint32_t)graph_offset_samp;
    graph_t0m=graph_t1m=graph_t2m=0;
    if(avail==0||hist_th==0||off>=avail){ graph_update_xlabels(); lv_chart_refresh(chart_temp); return; }
    uint32_t end_idx=hist_th-1-off;
    uint32_t n=(window_samp<=(end_idx+1))?window_samp:(end_idx+1);
    if(n<2){ graph_update_xlabels(); lv_chart_refresh(chart_temp); return; }
    uint32_t start_idx=end_idx-n+1;
    uint16_t pts=(uint16_t)(n<120?n:120); if(pts<2) pts=2;
    lv_chart_set_point_count(chart_temp,pts);
    int32_t *yi=lv_chart_get_y_array(chart_temp,ser_int_g);
    int32_t *ye=lv_chart_get_y_array(chart_temp,ser_ext_g);
    if(!yi||!ye){ lv_chart_refresh(chart_temp); return; }
    for(uint16_t i=0;i<pts;i++){ yi[i]=LV_CHART_POINT_NONE; ye[i]=LV_CHART_POINT_NONE; }
    File f=LittleFS.open("/temp.bin","r");
    if(!f){ graph_update_xlabels(); lv_chart_refresh(chart_temp); return; }
    uint32_t step=n/pts; if(step<1) step=1;
    TmpRec rec;
    for(uint16_t i=0;i<pts;i++){
        uint32_t idx=start_idx+(uint32_t)i*step;
        f.seek(idx*sizeof(TmpRec));
        if(f.read((uint8_t*)&rec,sizeof(rec))==sizeof(rec)){
            if(rec.t_int!=INT16_MIN) yi[i]=(int32_t)(rec.t_int/10);
            if(rec.t_ext!=INT16_MIN) ye[i]=(int32_t)(rec.t_ext/10);
            if(i==0)             graph_t0m=rec.ts_min;
            if(i==(uint16_t)(pts/2)) graph_t1m=rec.ts_min;
            if(i==(uint16_t)(pts-1)) graph_t2m=rec.ts_min;
        }
    }
    f.close();
    // Autoescalado: calcular min/max de los puntos cargados
    {
        int32_t ylo=INT32_MAX, yhi=INT32_MIN;
        for(uint16_t i=0;i<pts;i++){
            if(yi[i]!=LV_CHART_POINT_NONE){ ylo=min(ylo,yi[i]); yhi=max(yhi,yi[i]); }
            if(ye[i]!=LV_CHART_POINT_NONE){ ylo=min(ylo,ye[i]); yhi=max(yhi,ye[i]); }
        }
        if(ylo!=INT32_MAX){
            // redondear a múltiplos de 5 con 5° de margen
            graph_ymin = ((ylo-5)/5)*5;
            graph_ymax = ((yhi+9)/5)*5;
            if(graph_ymax<=graph_ymin) graph_ymax=graph_ymin+5;
        } else {
            graph_ymin=-10; graph_ymax=40;
        }
    }
    graph_update_ylabels();
    graph_update_xlabels();
    lv_chart_refresh(chart_temp);
}

static void cb_graph_zoomin (lv_event_t *e){ if(graph_zoom_h>1){ graph_zoom_h=(graph_zoom_h<=2)?1:(graph_zoom_h/2); graph_load(); } }
static void cb_graph_zoomout(lv_event_t *e){ if(graph_zoom_h<720){ graph_zoom_h=min(720u,graph_zoom_h*2); graph_load(); } }
static void cb_graph_exit   (lv_event_t *e){ go_to(SCR_TV); }

// Guías verticales dibujadas sobre el chart en cada render strip (sin objetos extra)
static void chart_draw_guides(lv_event_t *e) {
    lv_layer_t *layer = lv_event_get_layer(e);
    if(!layer || graph_t0m==0 || graph_t2m==0 || graph_t2m<=graph_t0m) return;
    lv_obj_t *obj = (lv_obj_t*)lv_event_get_target(e);
    lv_area_t ca; lv_obj_get_coords(obj, &ca);
    int32_t w = ca.x2 - ca.x1 - 8; // restar padding 4px a cada lado
    int32_t x0 = ca.x1 + 4;
    int32_t span = (int32_t)(graph_t2m - graph_t0m);
    uint32_t interval = (graph_zoom_h < 48) ? 120u : 1440u;
    uint32_t first = ((graph_t0m / interval) + 1) * interval;
    lv_draw_line_dsc_t ldsc; lv_draw_line_dsc_init(&ldsc);
    ldsc.color = lv_color_hex(COL_MUTED);
    ldsc.width = 1; ldsc.opa = LV_OPA_40;
    for(uint32_t t = first; t < graph_t2m; t += interval) {
        int32_t x = x0 + (int32_t)((int64_t)(t - graph_t0m) * w / span);
        if(x <= ca.x1 || x >= ca.x2) continue;
        ldsc.p1 = {(lv_value_precise_t)x, (lv_value_precise_t)(ca.y1+4)};
        ldsc.p2 = {(lv_value_precise_t)x, (lv_value_precise_t)(ca.y2-4)};
        lv_draw_line(layer, &ldsc);
    }
}

// Touch scroll horizontal en la gráfica
static void chart_touch_scroll(lv_event_t *e) {
    lv_indev_t *indev = lv_indev_get_act();
    if(!indev) return;
    lv_point_t pos; lv_indev_get_point(indev, &pos);
    if(chart_touch_prev_x != INT32_MIN) {
        int32_t dx = pos.x - chart_touch_prev_x;
        if(dx != 0) {
            uint32_t win=graph_zoom_h*12;
            uint32_t avail=(hist_th<MAX_TEMP)?hist_th:MAX_TEMP;
            int32_t max_off=(avail>win)?(int32_t)(avail-win):0;
            int32_t dsamp = (int32_t)((int64_t)dx * win / 360);
            graph_offset_samp=constrain(graph_offset_samp+dsamp, 0, max_off);
            graph_load();
        }
    }
    chart_touch_prev_x = pos.x;
}
static void chart_touch_reset(lv_event_t *e) { chart_touch_prev_x = INT32_MIN; }


static void cb_open_graph(lv_event_t *e) {
    graph_offset_samp=0; graph_zoom_h=24;
    graph_ymin=-10; graph_ymax=40;
    graph_load(); go_to(SCR_GRAPH);
}

static void build_scr_graph() {
    scr_graph=lv_obj_create(nullptr); bg(scr_graph);
    centered_label(scr_graph,"TEMPERATURA",&lv_font_montserrat_20,COL_TEXT,-195);

    // Etiquetas eje Y (dinámicas — actualizadas en graph_update_ylabels)
    for(int i=0;i<6;i++){
        lbl_y[i]=lv_label_create(scr_graph);
        lv_label_set_text(lbl_y[i],"--");
        lv_obj_set_style_text_font(lbl_y[i],&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(lbl_y[i],lv_color_hex(COL_MUTED),0);
        lv_obj_align(lbl_y[i],LV_ALIGN_CENTER,-190,-131+(int)(232*i/5));
    }

    // Gráfica (ligeramente más pequeña para dar más espacio a botones)
    chart_temp=lv_chart_create(scr_graph);
    lv_obj_set_size(chart_temp,360,230);
    lv_obj_align(chart_temp,LV_ALIGN_CENTER,0,-18);
    lv_chart_set_type(chart_temp,LV_CHART_TYPE_LINE);
    lv_obj_set_style_bg_color(chart_temp,lv_color_hex(0x101018),0);
    lv_obj_set_style_bg_opa(chart_temp,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(chart_temp,lv_color_hex(COL_MUTED),0);
    lv_obj_set_style_border_width(chart_temp,1,0);
    lv_obj_set_style_pad_all(chart_temp,4,LV_PART_MAIN);
    lv_obj_clear_flag(chart_temp,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(chart_temp,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(chart_temp, chart_touch_scroll, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(chart_temp, chart_touch_reset,  LV_EVENT_RELEASED,  nullptr);
    lv_obj_add_event_cb(chart_temp, chart_touch_reset,  LV_EVENT_PRESS_LOST,nullptr);
    lv_obj_set_style_line_color(chart_temp,lv_color_hex(0x202030),LV_PART_MAIN);
    lv_chart_set_range(chart_temp,LV_CHART_AXIS_PRIMARY_Y,graph_ymin,graph_ymax);
    lv_chart_set_div_line_count(chart_temp,5,5);
    lv_obj_set_style_size(chart_temp,0,0,LV_PART_INDICATOR); // sin puntos individuales
    ser_int_g=lv_chart_add_series(chart_temp,lv_color_hex(0xFF4000),LV_CHART_AXIS_PRIMARY_Y); // Ti=azul
    ser_ext_g=lv_chart_add_series(chart_temp,lv_color_hex(0x0040FF),LV_CHART_AXIS_PRIMARY_Y); // Te=rojo
    lv_chart_set_point_count(chart_temp,120);
    lv_chart_set_all_value(chart_temp,ser_int_g,LV_CHART_POINT_NONE);
    lv_chart_set_all_value(chart_temp,ser_ext_g,LV_CHART_POINT_NONE);

    // Leyenda Ti/Te dentro de la gráfica (esquina superior derecha)
    { lv_obj_t *l=lv_label_create(scr_graph);
      lv_label_set_text(l,"Ti"); lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
      lv_obj_set_style_text_color(l,lv_color_hex(0xFF4000),0);
      lv_obj_align(l,LV_ALIGN_CENTER,+143,-118); }
    { lv_obj_t *l=lv_label_create(scr_graph);
      lv_label_set_text(l,"Te"); lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
      lv_obj_set_style_text_color(l,lv_color_hex(0x0040FF),0);
      lv_obj_align(l,LV_ALIGN_CENTER,+143,-103); }

    // Etiquetas eje X (tiempo): inicio, medio, fin del rango visible
    lbl_graph_t0=lv_label_create(scr_graph);
    lv_label_set_text(lbl_graph_t0,"--:--");
    lv_obj_set_style_text_font(lbl_graph_t0,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_graph_t0,lv_color_hex(COL_MUTED),0);
    lv_obj_align(lbl_graph_t0,LV_ALIGN_CENTER,-130,+115);

    lbl_graph_t1=lv_label_create(scr_graph);
    lv_label_set_text(lbl_graph_t1,"--:--");
    lv_obj_set_style_text_font(lbl_graph_t1,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_graph_t1,lv_color_hex(COL_MUTED),0);
    lv_obj_align(lbl_graph_t1,LV_ALIGN_CENTER,0,+115);

    lbl_graph_t2=lv_label_create(scr_graph);
    lv_label_set_text(lbl_graph_t2,"--:--");
    lv_obj_set_style_text_font(lbl_graph_t2,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_graph_t2,lv_color_hex(COL_MUTED),0);
    lv_obj_align(lbl_graph_t2,LV_ALIGN_CENTER,+130,+115);

    // Indicador zoom (centro, sobre botones)
    lbl_graph_zoom=lv_label_create(scr_graph);
    lv_label_set_text(lbl_graph_zoom,"24h");
    lv_obj_set_style_text_font(lbl_graph_zoom,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(lbl_graph_zoom,lv_color_hex(COL_MUTED),0);
    lv_obj_align(lbl_graph_zoom,LV_ALIGN_CENTER,0,+140);

    // Botones: [−] izq  [SALIR] centro  [+] der — más grandes y separados
    struct BtnDef { const char *txt; int x; int w; lv_event_cb_t cb; uint32_t col; };
    BtnDef bdefs[]={
        {"-",    -122, 86, cb_graph_zoomout, COL_CARD},
        {"SALIR",   0,110, cb_graph_exit,    COL_OFF },
        {"+",    +122, 86, cb_graph_zoomin,  COL_CARD},
    };
    for(auto &d : bdefs){
        lv_obj_t *b=lv_obj_create(scr_graph);
        lv_obj_set_size(b,d.w,54);
        lv_obj_align(b,LV_ALIGN_CENTER,d.x,+172);
        lv_obj_set_style_bg_color(b,lv_color_hex(d.col),0);
        lv_obj_set_style_radius(b,10,0);
        lv_obj_set_style_border_width(b,0,0);
        lv_obj_set_style_pad_all(b,0,0);
        lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(b,LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(b,d.cb,LV_EVENT_CLICKED,nullptr);
        lv_obj_t *l=lv_label_create(b);
        lv_label_set_text(l,d.txt);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
        lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
        lv_obj_center(l);
    }

    lv_obj_add_event_cb(chart_temp, chart_draw_guides, LV_EVENT_DRAW_MAIN_END, nullptr);
}

// ── BUILD SCREEN HEATMAP ─────────────────────────────────────
static void hm_update_week_label() {
    if(!lbl_hm_week) return;
    time_t now = time(NULL);
    if(now < 1000000000L){ lv_label_set_text(lbl_hm_week,"Sin hora NTP"); return; }
    struct tm t; localtime_r(&now, &t);
    t.tm_hour=0; t.tm_min=0; t.tm_sec=0;
    int wday = t.tm_wday==0 ? 6 : t.tm_wday-1; // Mon=0...Sun=6
    time_t monday = mktime(&t) - (time_t)wday*86400L + (time_t)hm_week_off*7L*86400L;
    time_t sunday = monday + 6L*86400L;
    struct tm m, s; localtime_r(&monday,&m); localtime_r(&sunday,&s);
    char buf[22];
    snprintf(buf,sizeof(buf),"%02d/%02d - %02d/%02d/%04d",
        m.tm_mday,m.tm_mon+1, s.tm_mday,s.tm_mon+1,s.tm_year+1900);
    lv_label_set_text(lbl_hm_week, buf);
}
static void heatmap_goto_week(int off) {
    if(off > 0) off = 0;                 // no hay futuro
    hm_week_off = (int8_t)off;
    heatmap_load_week(hm_week_off);      // reconstruye desde /evt.bin
    hm_update_week_label();
    if(hm_view) lv_obj_invalidate(hm_view);
}
static void cb_hm_week_prev(lv_event_t *e) { heatmap_goto_week(hm_week_off - 1); }
static void cb_hm_week_next(lv_event_t *e) { heatmap_goto_week(hm_week_off + 1); }
static void cb_heatmap_exit(lv_event_t *e) { go_to(SCR_TV); }

// Col c → wday (c+1)%7   [c=0→Mon=1 … c=6→Sun=0]
// Fila r → hora (r+6)%24 [r=0→6am … r=17→23h … r=23→5am]
// Draw callback: LVGL lo llama una vez por render strip — sin buffer PSRAM
static void hm_draw_event(lv_event_t *e) {
    lv_layer_t *layer = lv_event_get_layer(e);
    if(!layer) return;
    lv_obj_t *obj = (lv_obj_t*)lv_event_get_target(e);
    lv_area_t area; lv_obj_get_coords(obj, &area);
    int32_t cw = (area.x2 - area.x1) / 7;
    int32_t ch = (area.y2 - area.y1) / 24;
    uint8_t maxv = 1;
    for(int d=0;d<7;d++) for(int s=0;s<96;s++) if(heatmap[d][s]>maxv) maxv=heatmap[d][s];
    lv_color_t c_lo = lv_color_hex(0x151515);
    lv_color_t c_hi = lv_color_hex(0x0050FF);  // swap→0xFF5000 naranja-rojo (mas contraste)
    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.border_width=0; dsc.radius=0; dsc.bg_opa=LV_OPA_COVER;
    for(int r=0; r<24; r++) {
        int hour = (r+6)%24;
        for(int c=0; c<7; c++) {
            int wday=(c+1)%7;
            uint16_t sum=0;
            for(int q=0;q<4;q++) sum+=heatmap[wday][hour*4+q];
            uint8_t avg=(uint8_t)(sum/4);
            uint8_t t=maxv>0?(uint8_t)((uint16_t)avg*255/maxv):0;
            dsc.bg_color=lv_color_mix(c_hi,c_lo,t);
            lv_area_t cell = {area.x1+c*cw, area.y1+r*ch, area.x1+c*cw+cw-1, area.y1+r*ch+ch-1};
            lv_draw_rect(layer, &dsc, &cell);
        }
    }
}

static void build_scr_heatmap() {
    scr_heatmap = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_heatmap, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(scr_heatmap, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr_heatmap, LV_OBJ_FLAG_SCROLLABLE);

    centered_label(scr_heatmap, "PRESENCIA", &lv_font_montserrat_20, COL_TEXT, -220);

    static const char *DAYS[7]={"L","M","X","J","V","S","D"};
    for(int c=0;c<7;c++){
        lv_obj_t *l=lv_label_create(scr_heatmap);
        lv_label_set_text(l, DAYS[c]);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(COL_MUTED), 0);
        lv_obj_set_pos(l, 90 + c*42 + 14, 44);
    }

    static const char *HLBLS[]={"06","09","12","15","18","21","00","03"};
    for(int i=0;i<8;i++){
        lv_obj_t *l=lv_label_create(scr_heatmap);
        lv_label_set_text(l, HLBLS[i]);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(COL_MUTED), 0);
        lv_obj_set_pos(l, 54, 62 + i*36);
    }

    hm_view = lv_obj_create(scr_heatmap);
    lv_obj_remove_style_all(hm_view);
    lv_obj_set_pos(hm_view, 90, 62);
    lv_obj_set_size(hm_view, 294, 288);
    lv_obj_clear_flag(hm_view, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(hm_view, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(hm_view, hm_draw_event, LV_EVENT_DRAW_MAIN, nullptr);

    // Navegación semanas — derecha, alineados con grid (y=135 y y=215 absolutos)
    { lv_obj_t *b=lv_obj_create(scr_heatmap);
      lv_obj_set_size(b,70,58); lv_obj_set_pos(b,397,120);
      lv_obj_set_style_bg_color(b,lv_color_hex(COL_OFF),0);
      lv_obj_set_style_radius(b,8,0); lv_obj_set_style_border_width(b,0,0);
      lv_obj_set_style_pad_all(b,0,0);
      lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE); lv_obj_add_flag(b,LV_OBJ_FLAG_CLICKABLE);
      lv_obj_add_event_cb(b,cb_hm_week_prev,LV_EVENT_CLICKED,nullptr);
      lv_obj_t *l=lv_label_create(b); lv_label_set_text(l,"< Sem");
      lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
      lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0); lv_obj_center(l); }
    { lv_obj_t *b=lv_obj_create(scr_heatmap);
      lv_obj_set_size(b,70,58); lv_obj_set_pos(b,397,210);
      lv_obj_set_style_bg_color(b,lv_color_hex(COL_OFF),0);
      lv_obj_set_style_radius(b,8,0); lv_obj_set_style_border_width(b,0,0);
      lv_obj_set_style_pad_all(b,0,0);
      lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE); lv_obj_add_flag(b,LV_OBJ_FLAG_CLICKABLE);
      lv_obj_add_event_cb(b,cb_hm_week_next,LV_EVENT_CLICKED,nullptr);
      lv_obj_t *l=lv_label_create(b); lv_label_set_text(l,"Sem >");
      lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
      lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0); lv_obj_center(l); }

    // Etiqueta semana — entre grid y SALIR
    lbl_hm_week = centered_label(scr_heatmap,"...", &lv_font_montserrat_14, COL_MUTED, 128);

    make_big_btn(scr_heatmap, "SALIR", COL_OFF, 190, 280, 60, cb_heatmap_exit, nullptr);
}

// ── VISOR LOG ────────────────────────────────────────────────

static const char *evt_name(uint8_t t, uint8_t v, uint8_t aux) {
    switch((EvtType)t){
        case EVT_SYSTEM_BOOT:   return "Arranque";
        case EVT_RELAY_AGUA:    return v ? "Agua ON"     : "Agua OFF";
        case EVT_RELAY_CALEF:   return v ? "Calef ON"    : "Calef OFF";
        case EVT_RELAY_SIRENA:  return v ? "Sirena ON"   : "Sirena OFF";
        case EVT_HEAT_MODE:     return v==HM_OFF?"Calef OFF":v==HM_MANUAL?"Calef Manual":v==HM_CONSIGNA?"Calef Consigna":"Calef Programa";
        case EVT_ALARM_ARM:     return "Alarma ARMADA";
        case EVT_ALARM_DISARM:  return "Alarma Desarmada";
        case EVT_ALARM_TRIGGER: return "INTRUSION";
        case EVT_PRESENCE:      return "Presencia";
        case EVT_FLOOD:         return v ? "Inund. ACTIVA" : "Inund. OK";
        case EVT_SMOKE:         return v ? "Humo ACTIVO"   : "Humo OK";
        default:                return "?";
    }
}

static void log_load() {
    if(!log_label) return;
    flush_events();   // volcar pendientes en RAM para mostrar lo más reciente
    static char buf[20000];
    buf[0] = '\0';
    int pos = 0;

    File f = LittleFS.open("/evt.bin","r");
    if(!f){ lv_label_set_text(log_label,"Sin registros"); return; }
    uint32_t total = f.size() / sizeof(EvtRec);
    if(total==0){ f.close(); lv_label_set_text(log_label,"Sin registros"); return; }
    uint32_t count = total < MAX_EVT ? total : MAX_EVT;

    time_t now = time(NULL);
    time_t day_start, day_end;
    {
        struct tm t; localtime_r(&now, &t);
        t.tm_hour=0; t.tm_min=0; t.tm_sec=0;
        time_t today = mktime(&t);
        day_start = today - (time_t)log_day_off * 86400L;
        day_end   = day_start + 86400L;
    }

    uint32_t added = 0;
    uint32_t head = hist_eh;
    for(uint32_t i=0; i<count; i++){
        if((i & 0x0F)==0) esp_task_wdt_reset();
        uint32_t idx = (head + MAX_EVT - 1 - i) % MAX_EVT;
        EvtRec r;
        f.seek(idx * sizeof(EvtRec));
        if(f.read((uint8_t*)&r, sizeof(r)) != sizeof(r)) continue;
        if(r.ts < 1000000000UL) continue;
        if(log_hide_pres && (EvtType)r.type == EVT_PRESENCE) continue;
        time_t ts = (time_t)r.ts;
        if(ts < day_start || ts >= day_end) {
            if(ts < day_start) break;
            continue;
        }
        struct tm t; localtime_r(&ts, &t);
        pos += snprintf(buf+pos, sizeof(buf)-pos-1, "%02d/%02d %02d:%02d  %s\n",
            t.tm_mday, t.tm_mon+1, t.tm_hour, t.tm_min,
            evt_name(r.type, r.value, r.aux));
        if(pos >= (int)sizeof(buf)-80) break;
        added++;
    }
    f.close();
    if(added==0) lv_label_set_text(log_label,"Sin eventos ese día");
    else         lv_label_set_text(log_label, buf);
}

static void log_update_pres_btn() {
    if(btn_log_pres) lv_obj_set_style_bg_color(btn_log_pres,
        lv_color_hex(log_hide_pres ? COL_OFF : COL_RELAY_DIM), 0);
    if(lbl_log_pres_btn) lv_label_set_text(lbl_log_pres_btn,
        log_hide_pres ? "Mov: oculto" : "Mov: visible");
}
static void cb_log_toggle_pres(lv_event_t *e) {
    log_hide_pres = !log_hide_pres;
    log_update_pres_btn();
    log_load();
}

static lv_obj_t *lbl_log_date = nullptr;
static void log_update_date_label() {
    if(!lbl_log_date) return;
    if(log_day_off==0){ lv_label_set_text(lbl_log_date,"Hoy"); return; }
    time_t now = time(NULL);
    struct tm t; localtime_r(&now, &t);
    t.tm_hour=0; t.tm_min=0; t.tm_sec=0;
    time_t d = mktime(&t) - (time_t)log_day_off * 86400L;
    struct tm td; localtime_r(&d, &td);
    char buf[12]; snprintf(buf,sizeof(buf),"%02d/%02d/%04d",td.tm_mday,td.tm_mon+1,td.tm_year+1900);
    lv_label_set_text(lbl_log_date, buf);
}

static void cb_log_nav(lv_event_t *e) {
    int d = (int)(intptr_t)lv_event_get_user_data(e);
    int32_t new_off = (int32_t)log_day_off + d;
    if(new_off < 0) return;
    log_day_off = (uint32_t)new_off;
    log_update_date_label();
    log_load();
}
static void cb_log_exit(lv_event_t *e) { go_to(SCR_TV); }

static lv_obj_t* make_nav_btn(lv_obj_t *parent, const char *lbl, int delta, int x, int y) {
    lv_obj_t *b = lv_obj_create(parent);
    lv_obj_set_size(b, 104, 72);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_style_bg_color(b, lv_color_hex(COL_OFF), 0);
    lv_obj_set_style_radius(b, 8, 0);
    lv_obj_set_style_border_width(b, 0, 0);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(b, cb_log_nav, LV_EVENT_CLICKED, (void*)(intptr_t)delta);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, lbl);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(COL_TEXT), 0);
    lv_obj_center(l);
    return b;
}

static void build_scr_log() {
    scr_log = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_log, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(scr_log, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr_log, LV_OBJ_FLAG_SCROLLABLE);

    // Toggle visibilidad presencia — barra superior
    btn_log_pres = lv_obj_create(scr_log);
    lv_obj_set_size(btn_log_pres, 160, 36);
    lv_obj_set_pos(btn_log_pres, 160, 6);
    lv_obj_set_style_bg_color(btn_log_pres, lv_color_hex(COL_RELAY_DIM), 0);
    lv_obj_set_style_radius(btn_log_pres, 18, 0);
    lv_obj_set_style_border_width(btn_log_pres, 0, 0);
    lv_obj_set_style_pad_all(btn_log_pres, 0, 0);
    lv_obj_clear_flag(btn_log_pres, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_log_pres, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_log_pres, cb_log_toggle_pres, LV_EVENT_CLICKED, nullptr);
    lbl_log_pres_btn = lv_label_create(btn_log_pres);
    lv_label_set_text(lbl_log_pres_btn, "Mov: visible");
    lv_obj_set_style_text_font(lbl_log_pres_btn, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_log_pres_btn, lv_color_hex(COL_TEXT), 0);
    lv_obj_center(lbl_log_pres_btn);

    // Botones izquierda — 2 botones centrados verticalmente
    make_nav_btn(scr_log, LV_SYMBOL_UP "7d",  7,  6, 154);
    make_nav_btn(scr_log, LV_SYMBOL_UP "1d",  1,  6, 248);

    // Botones derecha — 2 botones centrados verticalmente
    make_nav_btn(scr_log, "1d" LV_SYMBOL_DOWN, -1, 370, 154);
    make_nav_btn(scr_log, "7d" LV_SYMBOL_DOWN, -7, 370, 248);

    // Botón SALIR abajo centrado — estilo estándar
    make_big_btn(scr_log, "SALIR", COL_OFF, 180, 280, 68, cb_log_exit, nullptr);

    // Contenedor scrollable centro — más estrecho para dar espacio a los botones
    log_cont = lv_obj_create(scr_log);
    lv_obj_set_pos(log_cont, 113, 48);
    lv_obj_set_size(log_cont, 254, 362);
    lv_obj_set_style_bg_color(log_cont, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(log_cont, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(log_cont, 0, 0);
    lv_obj_set_style_pad_all(log_cont, 2, 0);

    log_label = lv_label_create(log_cont);
    lv_obj_set_width(log_label, 246);
    lv_label_set_long_mode(log_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(log_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(log_label, lv_color_hex(COL_TEXT), 0);
    lv_label_set_text(log_label, "");

    // Fecha — creada al final para renderizar encima del contenedor
    lbl_log_date = centered_label(scr_log,"Hoy",&lv_font_montserrat_20,COL_MUTED,220-14);
}

// ── BUILD SCREEN HIST MENU ───────────────────────────────────

static void cb_open_hist_menu(lv_event_t *e) { if(swipe_blocked()) return; go_to(SCR_HIST_MENU); }
static void cb_hist_temp(lv_event_t *e)  { cb_open_graph(nullptr); }
static void cb_hist_pres(lv_event_t *e)  { go_to(SCR_HEATMAP); }
static void cb_hist_log(lv_event_t *e)   { log_day_off=0; go_to(SCR_LOG); }
static void cb_hist_exit(lv_event_t *e)  { go_to(SCR_TV); }

static void hist_menu_set_sel(int idx) {
    hist_menu_sel = idx;
    for(int i=0;i<4;i++){
        if(!hist_menu_btns[i]) continue;
        lv_obj_set_style_border_width(hist_menu_btns[i], i==idx ? 2 : 0, 0);
        if(i==idx) lv_obj_set_style_border_color(hist_menu_btns[i], lv_color_hex(COL_TEXT), 0);
    }
}

static void build_scr_hist_menu() {
    scr_hist_menu = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_hist_menu, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(scr_hist_menu, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr_hist_menu, LV_OBJ_FLAG_SCROLLABLE);
    centered_label(scr_hist_menu,"HISTORIAL",&lv_font_montserrat_20,COL_TEXT,-180);
    lbl_hist_mem = centered_label(scr_hist_menu,"...", &lv_font_montserrat_14, COL_MUTED, -130);
    hist_menu_btns[0]=make_big_btn(scr_hist_menu,"Temperatura",COL_RELAY_DIM,  -72,300,68,cb_hist_temp,nullptr);
    hist_menu_btns[1]=make_big_btn(scr_hist_menu,"Presencia",  COL_RELAY_DIM,    8,300,68,cb_hist_pres,nullptr);
    hist_menu_btns[2]=make_big_btn(scr_hist_menu,"Registro",   COL_RELAY_DIM,   88,300,68,cb_hist_log, nullptr);
    hist_menu_btns[3]=make_big_btn(scr_hist_menu,"SALIR",      COL_OFF,        170,280,52,cb_hist_exit,nullptr);
}

// ── BUILD SCREEN PROGRAMA ────────────────────────────────────
static void cb_prog_cell(lv_event_t *e) {
    uint16_t data = (uint16_t)(uintptr_t)lv_event_get_user_data(e);
    uint8_t col = data >> 6;
    uint8_t row = data & 0x3F;
    int wday = (col + 1) % 7;
    cfg_prog[wday] ^= (1ULL << row);
    bool on = (cfg_prog[wday] >> row) & 1ULL;
    if(prog_cells[col][row])
        lv_obj_set_style_bg_color(prog_cells[col][row], lv_color_hex(on ? COL_RELAY_DIM : COL_OFF), 0);
    Preferences p; p.begin("heat", false);
    char k[8]; snprintf(k, sizeof(k), "prog%d", wday);
    p.putULong64(k, cfg_prog[wday]);
    p.end();
}
static void cb_eco_minus(lv_event_t *e) {
    if(cfg_eco_sp > 10) cfg_eco_sp--;
    if(lbl_eco_sp){ char b[8]; snprintf(b,sizeof(b),"%d\xc2\xb0""C",cfg_eco_sp); lv_label_set_text(lbl_eco_sp,b); }
    Preferences p; p.begin("heat",false); p.putInt("eco_sp",cfg_eco_sp); p.end();
}
static void cb_eco_plus(lv_event_t *e) {
    if(cfg_eco_sp < 35) cfg_eco_sp++;
    if(lbl_eco_sp){ char b[8]; snprintf(b,sizeof(b),"%d\xc2\xb0""C",cfg_eco_sp); lv_label_set_text(lbl_eco_sp,b); }
    Preferences p; p.begin("heat",false); p.putInt("eco_sp",cfg_eco_sp); p.end();
}
static void cb_prog_exit(lv_event_t *e) { go_to(SCR_TV); }

static void build_scr_prog() {
    scr_prog = lv_obj_create(nullptr); bg(scr_prog);

    // Layout (px absolutos, pantalla 480×480):
    //  y=  8..36  título
    //  y= 40..60  cabeceras días (fijas)
    //  y= 62..344 grid scrollable (282px)
    //  y=352..406 controles temperatura
    //  y=414..464 SALIR

    static const char *DAYS[7] = {"L","M","X","J","V","S","D"};
    const int LABEL_W=46;  // columna hora
    const int CELL_W =61;  // ancho columna día
    const int ROW_H  =26;  // alto fila 30min
    const int NROWS  =48;  // slots por día
    const int SCROLL_Y=62, SCROLL_H=282;

    // Título
    { lv_obj_t *l=lv_label_create(scr_prog);
      lv_label_set_text(l,"PROGRAMA");
      lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
      lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
      lv_obj_align(l,LV_ALIGN_TOP_MID,0,10); }

    // Cabeceras días (fijas, alineadas con columnas del grid)
    for(int c=0;c<7;c++){
        lv_obj_t *l=lv_label_create(scr_prog);
        lv_label_set_text(l, DAYS[c]);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(l,lv_color_hex(COL_MUTED),0);
        lv_obj_set_pos(l, LABEL_W + c*CELL_W + 24, 42);
    }

    // Contenedor scrollable (solo Y)
    lv_obj_t *sc = lv_obj_create(scr_prog);
    lv_obj_set_pos(sc, 0, SCROLL_Y);
    lv_obj_set_size(sc, 480, SCROLL_H);
    lv_obj_set_style_bg_color(sc, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_border_width(sc, 0, 0);
    lv_obj_set_style_pad_all(sc, 0, 0);
    lv_obj_set_style_radius(sc, 0, 0);
    lv_obj_set_scroll_dir(sc, LV_DIR_VER);
    lv_obj_clear_flag(sc, LV_OBJ_FLAG_SCROLL_ELASTIC);

    // Filas grid: 48 × ROW_H
    for(int r=0;r<NROWS;r++){
        int ry = r * ROW_H;
        // Etiqueta hora (HH:MM)
        { lv_obj_t *l=lv_label_create(sc);
          char hbuf[6]; snprintf(hbuf,sizeof(hbuf),"%02d:%02d",(r/2),(r%2)*30);
          lv_label_set_text(l,hbuf);
          lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
          lv_obj_set_style_text_color(l,lv_color_hex(COL_MUTED),0);
          lv_obj_set_pos(l, 2, ry+5); }
        // 7 celdas
        for(int c=0;c<7;c++){
            int wday=(c+1)%7;
            bool on=(cfg_prog[wday]>>r)&1ULL;
            lv_obj_t *cell=lv_obj_create(sc);
            lv_obj_set_size(cell, CELL_W-2, ROW_H-2);
            lv_obj_set_pos(cell, LABEL_W + c*CELL_W + 1, ry+1);
            lv_obj_set_style_bg_color(cell, lv_color_hex(on?COL_RELAY_DIM:COL_OFF), 0);
            lv_obj_set_style_radius(cell, 3, 0);
            lv_obj_set_style_border_width(cell, 0, 0);
            lv_obj_set_style_pad_all(cell, 0, 0);
            lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(cell, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(cell, cb_prog_cell, LV_EVENT_CLICKED,
                (void*)(uintptr_t)((uint16_t)((c<<6)|r)));
            prog_cells[c][r]=cell;
        }
    }

    // Controles temperatura: [−]  20°C  [+]
    auto mk_pm=[](lv_obj_t *par, int x, int y, lv_event_cb_t cb, const char *txt) {
        lv_obj_t *b=lv_obj_create(par);
        lv_obj_set_size(b,72,52); lv_obj_set_pos(b,x,y);
        lv_obj_set_style_bg_color(b,lv_color_hex(COL_CARD),0);
        lv_obj_set_style_radius(b,10,0); lv_obj_set_style_border_width(b,0,0);
        lv_obj_set_style_pad_all(b,0,0);
        lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE); lv_obj_add_flag(b,LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,nullptr);
        lv_obj_t *l=lv_label_create(b); lv_label_set_text(l,txt);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_40,0);
        lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0); lv_obj_center(l);
    };
    mk_pm(scr_prog,  60, 354, cb_eco_minus, "-");
    mk_pm(scr_prog, 348, 354, cb_eco_plus,  "+");

    { char ebuf[8]; snprintf(ebuf,sizeof(ebuf),"%d\xc2\xb0""C",cfg_eco_sp);
      lbl_eco_sp=lv_label_create(scr_prog);
      lv_label_set_text(lbl_eco_sp,ebuf);
      lv_obj_set_style_text_font(lbl_eco_sp,&lv_font_montserrat_40,0);
      lv_obj_set_style_text_color(lbl_eco_sp,lv_color_hex(COL_TEXT),0);
      lv_obj_align(lbl_eco_sp,LV_ALIGN_TOP_MID,0,357); }

    make_big_btn(scr_prog, "SALIR", COL_OFF, +192, 220, 52, cb_prog_exit, nullptr);
}

// ── BUILD TILE SETTINGS ──────────────────────────────────────
static void cb_open_brightness(lv_event_t *e){ if(swipe_blocked()) return; open_dial(DIAL_BRIGHTNESS); }
static void cb_open_dim(lv_event_t *e)       { if(swipe_blocked()) return; open_dial(DIAL_DIM); }
static void cb_cycle_anim(lv_event_t *e) {
    if(swipe_blocked()) return;
    cfg_anim = (cfg_anim + 1) % 2;
    if(lbl_cfg_anim)
        lv_label_set_text(lbl_cfg_anim, cfg_anim==0 ? "Anim: Ninguna" : "Anim: Deslizar");
}
static const char* saver_mode_text() {
    return cfg_saver_mode==0 ? "Salvapant: Off"
         : cfg_saver_mode==1 ? "Salvapant: On"
                             : "Salvapant: Calef";
}
static void cb_cycle_saver(lv_event_t *e) {
    if(swipe_blocked()) return;
    cfg_saver_mode = (cfg_saver_mode + 1) % 3;
    prefs.begin("domus",false); prefs.putInt("saver_mode", cfg_saver_mode); prefs.end();
    if(lbl_cfg_saver) lv_label_set_text(lbl_cfg_saver, saver_mode_text());
}
static void build_tile_settings() {
    bg(tile_settings);
    centered_label(tile_settings,"AJUSTES",&lv_font_montserrat_20,COL_TEXT,-170);
    lbl_settings_ip=centered_label(tile_settings,"IP: ---",&lv_font_montserrat_14,COL_MUTED,-148);
    centered_label(tile_settings,FW_VERSION,&lv_font_montserrat_14,COL_MUTED,-128);
    lbl_broker_status=centered_label(tile_settings,"Broker: sin conexion",&lv_font_montserrat_14,COL_ALERT,-108);

    // 5 píldoras iguales: 260×46px, gap=8px
    btn_settings_br=make_big_btn(tile_settings,"",COL_RELAY_DIM,-78,260,46,cb_open_brightness,nullptr);
    lv_obj_t *btn_br=btn_settings_br;
    lbl_cfg_brightness=lv_label_create(btn_br);
    char b1[24]; snprintf(b1,sizeof(b1),"Brillo: %d",cfg_brightness);
    lv_label_set_text(lbl_cfg_brightness,b1);
    lv_obj_set_style_text_font(lbl_cfg_brightness,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_color(lbl_cfg_brightness,lv_color_hex(COL_TEXT),0);
    lv_obj_center(lbl_cfg_brightness);

    btn_settings_dim=make_big_btn(tile_settings,"",COL_RELAY_DIM,-24,260,46,cb_open_dim,nullptr);
    lv_obj_t *btn_dim=btn_settings_dim;
    lbl_cfg_dim=lv_label_create(btn_dim);
    lv_obj_set_style_text_font(lbl_cfg_dim,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_color(lbl_cfg_dim,lv_color_hex(COL_TEXT),0);
    update_screen_pill_label();
    lv_obj_center(lbl_cfg_dim);

    btn_settings_saver=make_big_btn(tile_settings,"",COL_RELAY_DIM,+30,260,46,cb_cycle_saver,nullptr);
    lbl_cfg_saver=lv_label_create(btn_settings_saver);
    lv_label_set_text(lbl_cfg_saver, saver_mode_text());
    lv_obj_set_style_text_font(lbl_cfg_saver,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_color(lbl_cfg_saver,lv_color_hex(COL_TEXT),0);
    lv_obj_center(lbl_cfg_saver);

    btn_settings_pin=make_big_btn(tile_settings,"Cambiar PIN",COL_RELAY_DIM,+84,260,46,cb_open_change_pin,nullptr);

    btn_settings_anim=make_big_btn(tile_settings,"",COL_RELAY_DIM,+138,260,46,cb_cycle_anim,nullptr);
    lv_obj_t *btn_an=btn_settings_anim;
    lbl_cfg_anim=lv_label_create(btn_an);
    lv_label_set_text(lbl_cfg_anim,"Anim: Ninguna");
    lv_obj_set_style_text_font(lbl_cfg_anim,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_color(lbl_cfg_anim,lv_color_hex(COL_TEXT),0);
    lv_obj_center(lbl_cfg_anim);

    hint_settings=add_home_hint(tile_settings,LV_ALIGN_TOP_MID,LV_SYMBOL_UP " HOME");
}

// ── BUILD OVERLAY ALARMA CRÍTICA ─────────────────────────────
static void cb_deactivate(lv_event_t *e) {
    if(intruder_active){ pin_for_arm=false; memset(pin_buf,0,5); pin_len=0; go_to(SCR_PIN); }
    else { relay_set(3,false); critical_alert=false; alarm_state=alarm_armed?AS_ARMED:AS_OFF; mqtt_publish_status(); go_to(SCR_TV); }
}
static void build_scr_alarm() {
    scr_alarm=lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_alarm,lv_color_hex(0x1A0000),0);
    lv_obj_set_style_bg_opa(scr_alarm,LV_OPA_COVER,0);
    lbl_alarm_type  =centered_label(scr_alarm,"! ALARMA !",&lv_font_montserrat_40,COL_ALERT,-80);
    lbl_alarm_detail=centered_label(scr_alarm,"",&lv_font_montserrat_20,COL_TEXT,-20);
    btn_deactivate  =make_big_btn(scr_alarm,"DESACTIVAR",COL_ALERT,70,260,80,cb_deactivate,nullptr);
}

// ── SALVAPANTALLAS (anti-ghost) ───────────────────────────────
// Pantalla negra con hora + temp ext/int + icono de calefaccion (rojo, solo si
// hay un modo de calefaccion activo). El bloque salta a posiciones aleatorias
// para que ningun pixel quede fijo. Brillo bajo (el del DIM).
#define SAVER_W 200
#define SAVER_H 170
static void build_scr_saver() {
    scr_saver = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_saver, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(scr_saver, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr_saver, LV_OBJ_FLAG_SCROLLABLE);

    saver_box = lv_obj_create(scr_saver);
    lv_obj_set_size(saver_box, SAVER_W, SAVER_H);
    lv_obj_set_style_bg_opa(saver_box, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(saver_box, 0, 0);
    lv_obj_set_style_pad_all(saver_box, 0, 0);
    lv_obj_clear_flag(saver_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(saver_box, LV_OBJ_FLAG_CLICKABLE);

    lbl_saver_time = lv_label_create(saver_box);
    lv_label_set_text(lbl_saver_time, "--:--");
    lv_obj_set_style_text_font(lbl_saver_time, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(lbl_saver_time, lv_color_hex(COL_TEXT), 0);
    lv_obj_align(lbl_saver_time, LV_ALIGN_TOP_MID, 0, 0);

    lbl_saver_ext = lv_label_create(saver_box);
    lv_label_set_text(lbl_saver_ext, "Ext --");
    lv_obj_set_style_text_font(lbl_saver_ext, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl_saver_ext, lv_color_hex(COL_MUTED), 0);
    lv_obj_align(lbl_saver_ext, LV_ALIGN_TOP_MID, 0, 62);

    lbl_saver_int = lv_label_create(saver_box);
    lv_label_set_text(lbl_saver_int, "Int --");
    lv_obj_set_style_text_font(lbl_saver_int, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl_saver_int, lv_color_hex(COL_MUTED), 0);
    lv_obj_align(lbl_saver_int, LV_ALIGN_TOP_MID, 0, 92);

    lbl_saver_heat = lv_label_create(saver_box);
    lv_label_set_text(lbl_saver_heat, "\xF3\xB0\xBD\x94");  // mdi home-thermometer (U+F0F54)
    lv_obj_set_style_text_font(lbl_saver_heat, &icon_font, 0);
    lv_obj_set_style_text_color(lbl_saver_heat, lv_color_hex(COL_ALERT), 0);  // rojo
    lv_obj_align(lbl_saver_heat, LV_ALIGN_TOP_MID, 0, 122);
    lv_obj_add_flag(lbl_saver_heat, LV_OBJ_FLAG_HIDDEN);
}

static void saver_reposition() {
    if (!saver_box) return;
    int cx, cy;  // centro del bloque en circulo r<=104 → cabe entero (pantalla redonda)
    do { cx = random(136, 345); cy = random(136, 345); }
    while ((cx-240)*(cx-240) + (cy-240)*(cy-240) > 104*104);
    lv_obj_align(saver_box, LV_ALIGN_TOP_LEFT, cx - SAVER_W/2, cy - SAVER_H/2);
}
static void saver_update_content() {
    time_t now = time(NULL);
    if (now > 1000000000L) {
        struct tm *t = localtime(&now);
        char b[8]; snprintf(b, sizeof(b), "%02d:%02d", t->tm_hour, t->tm_min);
        lv_label_set_text(lbl_saver_time, b);
    }
    char e[20];
    if (wx.ok && !isnan(wx.temp)) snprintf(e, sizeof(e), "Ext %.1f\xc2\xb0", wx.temp);
    else                          snprintf(e, sizeof(e), "Ext --");
    lv_label_set_text(lbl_saver_ext, e);
    char in[20];
    if (!isnan(tuya_temp_int)) snprintf(in, sizeof(in), "Int %.1f\xc2\xb0", tuya_temp_int);
    else                       snprintf(in, sizeof(in), "Int --");
    lv_label_set_text(lbl_saver_int, in);
    if (lbl_saver_heat) {
        if (heat_mode != HM_OFF) lv_obj_clear_flag(lbl_saver_heat, LV_OBJ_FLAG_HIDDEN);
        else                     lv_obj_add_flag(lbl_saver_heat, LV_OBJ_FLAG_HIDDEN);
    }
}
static void enter_saver() {
    scr_before_saver = lv_scr_act();
    saver_update_content();
    saver_reposition();
    apply_dim_brightness();
    lv_scr_load(scr_saver);
    saver_active = true;
}
static void enter_dim() {
    apply_dim_brightness();
    screen_dimmed = true; screen_off = false;
}
static void enter_off() {
    screen_dimmed = true; screen_off = true;
    bool heating = (heat_mode != HM_OFF);
    bool show_saver = (cfg_saver_mode == 1) || (cfg_saver_mode == 2 && heating);
    if (show_saver) enter_saver();
    else { saver_active = false; set_brightness(0); }
}
// Despertar por un evento de sensor/actuador: solo si la pantalla esta
// apagada/atenuada y NO estamos atendiendo una alarma (no pisar esa pantalla).
static void wake_on_event() {
    if (critical_alert || intruder_active) return;
    if (cur_scr == SCR_ALARM_CRIT || cur_scr == SCR_PIN) return;
    if (screen_dimmed) request_wake(true);
}

// ── BUILD OVERLAY PIN ─────────────────────────────────────────
static void update_pin_dots() {
    char b[9]="_ _ _ _";
    for(int i=0;i<pin_len&&i<4;i++) b[i*2]='*';
    lv_label_set_text(lbl_pin_dots,b);
}
static void pin_check() {
    if(strncmp(pin_buf,cfg_pin,4)==0){
        if(pin_for_arm){ alarm_state=AS_ARMING; alarm_armed=true; alarm_ts=millis(); arming_end_ms=alarm_ts+120000UL; start_beep_seq(1,100); lv_label_set_text(lbl_pin_msg,""); mqtt_publish_status(); go_to(SCR_ARMING_SCR); }
        else { alarm_state=AS_OFF; alarm_armed=false; intruder_active=false; critical_alert=false; beep_seq={}; siren_off_at=0; relay_set(3,false); log_event(EVT_ALARM_DISARM,1); lv_label_set_text(lbl_pin_msg,""); mqtt_publish_status(); go_to(SCR_TV); }
    } else { lv_label_set_text(lbl_pin_msg,"PIN incorrecto"); memset(pin_buf,0,5); pin_len=0; update_pin_dots(); }
}
static void cb_pin_key(lv_event_t *e) {
    if(millis()-pin_open_ts < 350) return; // ignora toques fantasma del encoder
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
    const uint32_t cols[12]={COL_CARD,COL_CARD,COL_CARD,COL_CARD,COL_CARD,COL_CARD,
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
    alarm_state=AS_OFF; alarm_armed=false; beep_seq={}; siren_off_at=0; mqtt_publish_status(); go_to(SCR_TV);
}
static void build_scr_arming() {
    scr_arming=lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_arming,lv_color_hex(0x1010CC),0);
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

    // Cajas selectoras de PANTALLA — visibles solo en modo DIM/Apagar/Brillo
    auto make_dim_box=[&](int x_ofs, const char *txt, lv_event_cb_t cb) -> lv_obj_t* {
        lv_obj_t *b=lv_obj_create(scr_dial);
        lv_obj_set_size(b,112,48);
        lv_obj_align(b,LV_ALIGN_CENTER,x_ofs,+72);
        lv_obj_set_style_bg_color(b,lv_color_hex(COL_OFF),0);
        lv_obj_set_style_radius(b,10,0);
        lv_obj_set_style_border_color(b,lv_color_hex(COL_RELAY_ON),0);
        lv_obj_set_style_border_width(b,0,0);
        lv_obj_set_style_pad_all(b,0,0);
        lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(b,LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(b,LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,nullptr);
        lv_obj_t *l=lv_label_create(b);
        lv_label_set_text(l,txt);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(l,lv_color_hex(COL_TEXT),0);
        lv_obj_set_style_text_align(l,LV_TEXT_ALIGN_CENTER,0);
        lv_obj_center(l);
        return b;
    };
    dial_box_min   = make_dim_box(-122, "DIM\n5 min",  cb_dim_box_min);
    dial_box_off   = make_dim_box(   0, "Apagar\n10 min", cb_dim_box_off);
    dial_box_bright= make_dim_box(+122, "Brillo\n5%",     cb_dim_box_bright);

    make_big_btn(scr_dial,"OK",      COL_OK, 120,200,56,cb_dial_ok,    nullptr);
    make_big_btn(scr_dial,"CANCELAR",COL_OFF,185,200,44,cb_dial_cancel,nullptr);
}

// ── ENCODER FOCUS SYSTEM ──────────────────────────────────────
static lv_obj_t *tile_by_cycle(int idx){
    lv_obj_t *t[]={tile_home,tile_relays,tile_settings,tile_sensors,tile_alarm};
    return t[idx%5];
}
static void sync_tile_cycle_idx(){
    for(int i=0;i<5;i++) if(TC_COL[i]==tile_col&&TC_ROW[i]==tile_row){tile_cycle_idx=i;return;}
}

static void focus_set_outline(lv_obj_t *w, bool on, bool blink_type){
    if(!w) return;
    lv_obj_set_style_outline_width(w, on?3:0, LV_PART_MAIN);
    lv_obj_set_style_outline_color(w, lv_color_hex(COL_RELAY_ON), LV_PART_MAIN);
    lv_obj_set_style_outline_pad(w, 3, LV_PART_MAIN);
    // For HOME-nav items, opa is toggled by the blink loop; for others, solid
    if(!blink_type) lv_obj_set_style_outline_opa(w, on?LV_OPA_COVER:LV_OPA_TRANSP, LV_PART_MAIN);
    else if(!on)    lv_obj_set_style_outline_opa(w, LV_OPA_TRANSP, LV_PART_MAIN);
}
static void focus_clear_current(){
    FocusList &fl=tile_focus[tile_cycle_idx];
    if(fl.count>0&&focus_idx>=0&&focus_idx<fl.count)
        focus_set_outline(fl.items[focus_idx], false, fl.home_nav[focus_idx]);
}
static void focus_exit(){
    focus_clear_current();
    focus_idx=0; enc_mode=ENC_IDLE;
    enc_accum=0; focus_blink_st=false;
}
static void focus_enter(){
    enc_mode=ENC_FOCUS; focus_idx=0;
    focus_idle_ms=millis(); enc_accum=0;
    FocusList &fl=tile_focus[tile_cycle_idx];
    if(fl.count>0) focus_set_outline(fl.items[0], true, fl.home_nav[0]);
}

static void build_focus_lists(){
    // 0 = home tile
    { FocusList &f=tile_focus[0]; f.count=2;
      f.items[0]=lbl_alarm_badge; f.home_nav[0]=false;
      f.items[1]=lbl_sistema;     f.home_nav[1]=false; }
    // 1 = relays tile
    { FocusList &f=tile_focus[1]; f.count=7;
      f.items[0]=btn_agua_r;   f.items[1]=btn_hm[0];  f.items[2]=btn_hm[1];
      f.items[3]=btn_setpoint; f.items[4]=btn_hm[2];  f.items[5]=btn_sirena_r;
      f.items[6]=hint_relays;  f.home_nav[6]=true; }
    // 2 = settings tile
    { FocusList &f=tile_focus[2]; f.count=6;
      f.items[0]=btn_settings_br;    f.items[1]=btn_settings_dim;
      f.items[2]=btn_settings_saver; f.items[3]=btn_settings_pin;
      f.items[4]=btn_settings_anim;  f.items[5]=hint_settings; f.home_nav[5]=true; }
    // 3 = hist tile
    { FocusList &f=tile_focus[3]; f.count=4;
      f.items[0]=btn_hist[0];  f.home_nav[0]=false;
      f.items[1]=btn_hist[1];  f.home_nav[1]=false;
      f.items[2]=btn_hist[2];  f.home_nav[2]=false;
      f.items[3]=hint_sensors; f.home_nav[3]=true; }
    // 4 = alarm tile
    { FocusList &f=tile_focus[4]; f.count=2;
      f.items[0]=btn_arm;
      f.items[1]=hint_alarm; f.home_nav[1]=true; }
}

// ── UPDATE UI ─────────────────────────────────────────────────
static void update_home() {
    alarm_armed=(alarm_state!=AS_OFF);
    // Track alert state; blink loop in loop() handles color for alerting bands
    harc_alert[1]=a6v3.input[1];
    harc_alert[2]=!a6v3.input[4];
    harc_alert[3]=!a6v3.input[5];
    harc_alert[4]=!a6v3.input[6];
    if(!harc_alert[1]) lv_obj_set_style_bg_color(harc[1],lv_color_hex(COL_OK_DIM),0);
    if(!harc_alert[2]) lv_obj_set_style_bg_color(harc[2],lv_color_hex(COL_OK_DIM),0);
    if(!harc_alert[3]) lv_obj_set_style_bg_color(harc[3],lv_color_hex(COL_OK_DIM),0);
    if(!harc_alert[4]) lv_obj_set_style_bg_color(harc[4],lv_color_hex(COL_OK),0);
    // blink loop maneja el color de alerta; aqui solo el color estático normal
    rarc_alert[1]=!a6v3.output[1]; // agua: normal=ON(naranja), alerta=OFF(parpadea rojo)
    rarc_alert[2]= a6v3.output[2]; // calef: normal=OFF(apagado), alerta=ON(parpadea rojo)
    rarc_alert[3]= a6v3.output[3]; // sirena: normal=OFF(apagado), alerta=ON(parpadea rojo)
    if(!rarc_alert[1]) lv_obj_set_style_bg_color(rarc[1],lv_color_hex(COL_RELAY_ON),0);
    if(!rarc_alert[2]) lv_obj_set_style_bg_color(rarc[2],lv_color_hex(COL_RELAY_DIM),0);
    if(!rarc_alert[3]) lv_obj_set_style_bg_color(rarc[3],lv_color_hex(COL_RELAY_DIM),0);
    if(arc_alarm_zone)
        lv_obj_set_style_bg_color(arc_alarm_zone,lv_color_hex(alarm_armed?COL_ALERT:COL_ZONE_ALARM),0);
    lv_label_set_text(lbl_alarm_badge,alarm_armed?"ALARMA\nON":"ALARMA\nOFF");
    lv_obj_set_style_text_color(lbl_alarm_badge,lv_color_hex(alarm_armed?COL_ALERT:COL_MUTED),0);
    // Temp/humedad interior — fuente: MQTT DOMUS/ENV (HA publica el Xiaomi BLE)
    if(lbl_temp_int){
        char b[12];
        if(isnan(tuya_temp_int)) snprintf(b,sizeof(b),"--.-\xc2\xb0""C");
        else                     snprintf(b,sizeof(b),"%.1f\xc2\xb0""C",tuya_temp_int);
        lv_label_set_text(lbl_temp_int,b);
    }
    if(lbl_humidity){
        char b[12];
        if(isnan(tuya_humidity)) snprintf(b,sizeof(b),"--%% HR");
        else                     snprintf(b,sizeof(b),"%.0f%% HR",tuya_humidity);
        lv_label_set_text(lbl_humidity,b);
    }
}
static void update_sensors_tile() { /* sensors tile replaced by hist tile */ }
static void update_relays_tile() {
    if(btn_agua_r)   lv_obj_set_style_bg_color(btn_agua_r,  lv_color_hex(a6v3.output[1]?COL_OK:COL_ZONE_SET),0);
    if(btn_sirena_r) lv_obj_set_style_bg_color(btn_sirena_r,lv_color_hex(a6v3.output[3]?COL_ALERT:COL_ZONE_ALARM),0);
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
        lv_obj_set_style_bg_color(btn_arm,lv_color_hex(COL_RELAY_DIM),0);
    }
}
// Mapa código WMO (open-meteo) → glyph MDI de weather_font (UTF-8)
static const char* wmo_icon(int c) {
    if(c<0)          return "";
    if(c==0)         return "\xF3\xB0\x96\x99"; // despejado → sunny
    if(c<=2)         return "\xF3\xB0\x96\x95"; // poco nuboso → partly-cloudy
    if(c==3)         return "\xF3\xB0\x96\x90"; // nublado → cloudy
    if(c==45||c==48) return "\xF3\xB0\x96\x91"; // niebla → fog
    if(c>=51&&c<=57) return "\xF3\xB0\x96\x97"; // llovizna → rainy
    if(c>=61&&c<=65) return "\xF3\xB0\x96\x96"; // lluvia → pouring
    if(c==66||c==67) return "\xF3\xB0\x99\xBF"; // aguanieve → snowy-rainy
    if(c>=71&&c<=77) return "\xF3\xB0\x96\x98"; // nieve → snowy
    if(c>=80&&c<=82) return "\xF3\xB0\x96\x96"; // chubascos → pouring
    if(c==85||c==86) return "\xF3\xB0\x96\x98"; // chubascos nieve → snowy
    if(c>=95)        return "\xF3\xB0\x96\x93"; // tormenta → lightning
    return "\xF3\xB0\x96\x90";                  // fallback → cloudy
}
static void update_weather_display() {
    if(!wx.ok) return;
    char buf[40];
    snprintf(buf,sizeof(buf),"%.1f\xc2\xb0""C",wx.temp);
    lv_label_set_text(lbl_temp_ext,buf);
    snprintf(buf,sizeof(buf),"%d km/h  %.1f mm/h",(int)wx.wind,wx.rain_now);
    lv_label_set_text(lbl_weather,buf);
    if(lbl_wx_forecast) {
        float tmax=wx.h_temp[0], tmin=wx.h_temp[0], rsum=0;
        for(int i=0;i<12;i++){
            if(wx.h_temp[i]>tmax) tmax=wx.h_temp[i];
            if(wx.h_temp[i]<tmin) tmin=wx.h_temp[i];
            rsum+=wx.h_rain[i];
        }
        snprintf(buf,sizeof(buf),"%.0f\xc2\xb0 / %.0f\xc2\xb0  %.1fmm",tmax,tmin,rsum);
        lv_label_set_text(lbl_wx_forecast,buf);
    }
    if(lbl_wx_icon) lv_label_set_text(lbl_wx_icon, wmo_icon(wx.code));
}

// ── ESTADO BROKER ────────────────────────────────────────────
static void update_broker_status() {
    bool online = (mqtt_last_activity > 0 &&
                   millis() - mqtt_last_activity < 5UL*60000UL);
    if(arc_settings_zone)
        lv_obj_set_style_bg_color(arc_settings_zone, lv_color_hex(online?COL_OK_DIM:COL_ZONE_SET), 0);
    if(lbl_sistema) {
        lv_label_set_text(lbl_sistema, online ? "Sistema\nOK" : "Sistema\nOFF");
        lv_obj_set_style_text_color(lbl_sistema, lv_color_hex(online ? COL_OK : COL_TEXT), 0);
    }
    if(lbl_broker_status) {
        if(online) {
            char buf[48];
            if(domus_pub_epoch > 0) {
                struct tm *t = localtime(&domus_pub_epoch);
                snprintf(buf,sizeof(buf),"Broker OK  |  pub %02d:%02d:%02d",
                    t->tm_hour, t->tm_min, t->tm_sec);
            } else {
                snprintf(buf,sizeof(buf),"Broker: Conectado");
            }
            lv_label_set_text(lbl_broker_status, buf);
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
        critical_alert=true; alarm_state=AS_SOUNDING; beep_seq={}; siren_off_at=0;
        relay_set(3,true);
        log_event(EVT_ALARM_TRIGGER, 1, flood ? EVT_FLOOD : EVT_SMOKE);
        lv_label_set_text(lbl_alarm_type, flood?"INUNDACION":"HUMO");
        lv_label_set_text(lbl_alarm_detail,flood?"Valvulas cerradas auto.":"Sirena activada");
        lv_label_set_text((lv_obj_t*)lv_obj_get_child(btn_deactivate,0),"DESACTIVAR");
        mqtt_publish_status();
        request_wake(false);
        go_to(SCR_ALARM_CRIT);
    }
    if(alarm_state==AS_ARMED&&!a6v3.input[4]&&!intruder_active){
        intruder_active=true; alarm_state=AS_GRACE; alarm_ts=millis();
        memset(grace_beeps,0,sizeof(grace_beeps));
        lv_label_set_text(lbl_alarm_type,"INTRUSION");
        lv_label_set_text(lbl_alarm_detail,"Introduce PIN para desarmar");
        lv_label_set_text((lv_obj_t*)lv_obj_get_child(btn_deactivate,0),"INTRODUCIR PIN");
        mqtt_publish_status();
        request_wake(false);
        go_to(SCR_ALARM_CRIT);
    }
}

// ── SCREEN SWITCH ─────────────────────────────────────────────
static void do_switch(Screen s) {
    wake_reload_pending = false;  // un cambio explicito de pantalla anula la recarga del salvapantallas
    switch(s){
        case SCR_TV:
            update_home();
            update_sensors_tile();
            if(lbl_hist_tile_mem){
                size_t fu=LittleFS.usedBytes(), ft=LittleFS.totalBytes();
                char mb[36]; snprintf(mb,sizeof(mb),"Flash: %d/%d KB (%d%%)",
                    (int)(fu/1024),(int)(ft/1024),ft>0?(int)(fu*100/ft):0);
                lv_label_set_text(lbl_hist_tile_mem,mb);
            }
            update_relays_tile();
            update_alarm_tile();
            lv_scr_load_anim(tv,LV_SCR_LOAD_ANIM_NONE,0,0,false);
            if(enc_mode==ENC_FOCUS) focus_idle_ms=millis();
            cur_scr=SCR_TV;
            return;
        case SCR_ALARM_CRIT:
            request_wake(false);
            lv_scr_load_anim(scr_alarm,LV_SCR_LOAD_ANIM_MOVE_TOP,250,0,false);
            cur_scr=SCR_ALARM_CRIT; return;
        case SCR_PIN:
            update_pin_dots(); lv_label_set_text(lbl_pin_msg,"");
            pin_open_ts=millis();
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
        case SCR_GRAPH:
            lv_scr_load_anim(scr_graph,LV_SCR_LOAD_ANIM_MOVE_TOP,250,0,false);
            cur_scr=SCR_GRAPH; return;
        case SCR_HEATMAP:
            if(!scr_heatmap){ esp_task_wdt_reset(); build_scr_heatmap(); }
            heatmap_goto_week(0);
            lv_scr_load_anim(scr_heatmap,LV_SCR_LOAD_ANIM_NONE,0,0,false);
            cur_scr=SCR_HEATMAP; return;
        case SCR_HIST_MENU:
            if(!scr_hist_menu){ build_scr_hist_menu(); }
            if(lbl_hist_mem){
                size_t fs_used = LittleFS.usedBytes(), fs_tot = LittleFS.totalBytes();
                char mbuf[40];
                snprintf(mbuf,sizeof(mbuf),"Historial: %d KB / %d KB  (%d%%)",
                    (int)(fs_used/1024),(int)(fs_tot/1024),
                    fs_tot>0?(int)(fs_used*100/fs_tot):0);
                lv_label_set_text(lbl_hist_mem, mbuf);
            }
            hist_menu_set_sel(0);
            enc_accum = 0;
            lv_scr_load_anim(scr_hist_menu,LV_SCR_LOAD_ANIM_NONE,0,0,false);
            cur_scr=SCR_HIST_MENU; return;
        case SCR_LOG:
            if(!scr_log){ build_scr_log(); }
            log_update_date_label();
            log_update_pres_btn();
            log_load();
            lv_scr_load_anim(scr_log,LV_SCR_LOAD_ANIM_NONE,0,0,false);
            cur_scr=SCR_LOG; return;
        case SCR_PROG:
            if(!scr_prog){ esp_task_wdt_reset(); build_scr_prog(); }
            if(lbl_eco_sp){ char b[14]; snprintf(b,sizeof(b),"ECO: %d\xc2\xb0""C",cfg_eco_sp); lv_label_set_text(lbl_eco_sp,b); }
            for(int c=0;c<7;c++){ int wd=(c+1)%7;
                for(int r=0;r<6;r++) if(prog_cells[c][r])
                    lv_obj_set_style_bg_color(prog_cells[c][r], lv_color_hex((cfg_prog[wd]>>r)&1?COL_RELAY_DIM:COL_OFF), 0); }
            lv_scr_load_anim(scr_prog,LV_SCR_LOAD_ANIM_NONE,0,0,false);
            cur_scr=SCR_PROG; return;
    }
}

// ── TUYA ─────────────────────────────────────────────────────
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
static void mqtt_publish_debug() {  // telemetria del buffer de eventos → DOMUS/debug
    char dbg[140];
    snprintf(dbg,sizeof(dbg),
        "{\"buf\":%u,\"max\":%u,\"eh\":%lu,\"nflush\":%lu,\"nevt\":%lu,\"dropped\":%lu,\"dim\":%s}",
        (unsigned)evt_buf_n,(unsigned)EVT_BUF_N,(unsigned long)hist_eh,
        (unsigned long)g_nflush,(unsigned long)g_nevt,(unsigned long)g_dropped,
        screen_dimmed?"true":"false");
    broker.publish("DOMUS/debug", std::string(dbg));
}
static void mqtt_publish_status() {
    const char *as =
        alarm_state==AS_OFF     ?"OFF"     :
        alarm_state==AS_ARMING  ?"ARMING"  :
        alarm_state==AS_ARMED   ?"ARMED"   :
        alarm_state==AS_GRACE   ?"GRACE"   :"SOUNDING";
    char buf[160];
    if(isnan(tuya_temp_int))
        snprintf(buf,sizeof(buf),
            "{\"alarm\":\"%s\",\"heat_mode\":%d,\"heat_relay\":%s,"
            "\"flood\":%s,\"smoke\":%s,\"power\":%s}",
            as,(int)heat_mode,a6v3.output[2]?"true":"false",
            a6v3.input[1]?"true":"false",
            a6v3.input[5]?"false":"true",
            a6v3.input[6]?"true":"false");
    else
        snprintf(buf,sizeof(buf),
            "{\"alarm\":\"%s\",\"temp_int\":%.1f,\"humidity\":%.0f,"
            "\"heat_mode\":%d,\"heat_relay\":%s,"
            "\"flood\":%s,\"smoke\":%s,\"power\":%s}",
            as,tuya_temp_int,isnan(tuya_humidity)?0.0f:tuya_humidity,
            (int)heat_mode,a6v3.output[2]?"true":"false",
            a6v3.input[1]?"true":"false",
            a6v3.input[5]?"false":"true",
            a6v3.input[6]?"true":"false");
    broker.publish("DOMUS/status", std::string(buf));
    time(&domus_pub_epoch);
    Serial.printf("[DOMUS/status] %s\n", buf);
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
                    if(tuya_humidity>100.0f) tuya_humidity/=10.0f;
                    char buf[12]; snprintf(buf,sizeof(buf),"%.0f%% HR",tuya_humidity);
                    if(lbl_humidity) lv_label_set_text(lbl_humidity,buf);
                }
            }
            check_heating_auto();
        }
    }
    http.end();
    mqtt_publish_status();
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
            wx.wind=wmax; wx.code=doc["daily"]["weather_code"][0] | -1; wx.ok=true;
            update_weather_display();
        }
    }
    http.end(); wx_last=millis();
}

// Ejecuta la acción del ítem en foco llamando directamente al callback
static void exec_focus_item() {
    FocusList &fl=tile_focus[tile_cycle_idx];
    if(fl.count==0||focus_idx<0||focus_idx>=fl.count||!fl.items[focus_idx]) return;
    if(fl.home_nav[focus_idx]) {
        focus_exit();
        tile_cycle_idx=0; tile_col=1; tile_row=1;
        lv_tileview_set_tile(tv,tile_home,cfg_anim?LV_ANIM_ON:LV_ANIM_OFF);
        return;
    }
    focus_idle_ms=millis();
    lv_obj_t *w=fl.items[focus_idx];
    if     (w==btn_agua_r)        cb_agua(nullptr);
    else if(w==btn_hm[0])         cb_hm_manual(nullptr);
    else if(w==btn_hm[1])         cb_hm_consigna(nullptr);
    else if(w==btn_hm[2])         cb_hm_prog(nullptr);
    else if(w==btn_setpoint)      cb_setpoint_btn(nullptr);
    else if(w==btn_sirena_r)      cb_sirena(nullptr);
    else if(w==btn_settings_br)   cb_open_brightness(nullptr);
    else if(w==btn_settings_dim)  cb_open_dim(nullptr);
    else if(w==btn_settings_saver)cb_cycle_saver(nullptr);
    else if(w==btn_settings_pin)  cb_open_change_pin(nullptr);
    else if(w==btn_settings_anim) cb_cycle_anim(nullptr);
    else if(w==btn_hist[0])       cb_hist_temp(nullptr);
    else if(w==btn_hist[1])       cb_hist_pres(nullptr);
    else if(w==btn_hist[2])       cb_hist_log(nullptr);
    else if(w==btn_arm)           cb_go_pin(nullptr);
    else if(w==lbl_alarm_badge)   cb_alarm_badge(nullptr);
    else if(w==lbl_sistema)       cb_settings_icon(nullptr);
}

size_t getArduinoLoopTaskStackSize(void) { return 32768; }

// ── SETUP ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    unsigned long t0=millis();
    while(!Serial&&(millis()-t0)<5000) delay(10);
    Serial.println("DOMUS " FW_VERSION);

    prefs.begin("domus",true);
    cfg_brightness    =(int)prefs.getInt("brightness", 200);
    cfg_dim_delay     =(int)prefs.getInt("dim_delay",  2);
    cfg_off_delay     =(int)prefs.getInt("off_delay",  10);
    cfg_dim_brightness=(int)prefs.getInt("dim_bright", 5);
    cfg_saver_mode    =(uint8_t)prefs.getInt("saver_mode", 2);
    heat_setpoint     =(int)prefs.getInt("heat_sp",    20);
    heat_mode         =(HeatMode)prefs.getInt("heat_mode",0);
    prefs.getString("pin","1234").toCharArray(cfg_pin,sizeof(cfg_pin));
    prefs.end();
    { Preferences ph; ph.begin("heat",true);
      cfg_eco_sp=(int)ph.getInt("eco_sp",16);
      for(int d=0;d<7;d++){ char k[8]; snprintf(k,sizeof(k),"prog%d",d); cfg_prog[d]=ph.getULong64(k,0); }
      ph.end(); }

    Wire.begin(I2C_SDA,I2C_SCL);
    lcd_power_on();
    delay(200);   // extra init time for CST8XX after LCD reset
    touch_init();
    if(touch_i2c_addr) Serial.printf("[TOUCH] Found at 0x%02X\n", touch_i2c_addr);
    else               Serial.println("[TOUCH] NOT found — will retry in loop");

    ledcSetup(0,5000,8);
    ledcAttachPin(TFT_BL_PIN,0);
    set_brightness(cfg_brightness);

    pinMode(ENC_A,INPUT_PULLUP); pinMode(ENC_B,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A),enc_isr,CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B),enc_isr,CHANGE);
    // ENC_SW (GPIO0) se adjunta al FINAL de setup(), después de que
    // el pin de strapping haya dejado de rebotar.

    display->begin();
    lv_init();

    lv_display_t *disp=lv_display_create(TFT_WIDTH,TFT_HEIGHT);
    // Forzar RGB565 — LVGL 9.x usa RGB888 por defecto lo que causa sizeof(lv_color_t)=3
    // y corrupción de heap en operaciones de color. Hay que setear antes del buffer.
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp,lvgl_flush_cb);

    // Buffer pantalla completa PSRAM — FULL mode evita crashes LVGL 9.x con ST7701 PARTIAL
    // 2 bytes/px fijo (RGB565) — no depender de sizeof(lv_color_t)
    size_t buf_sz=(size_t)TFT_WIDTH*TFT_HEIGHT*2;
    uint8_t *buf1=(uint8_t*)heap_caps_malloc(buf_sz,MALLOC_CAP_SPIRAM);
    if(!buf1){ Serial.println("[MEM] ERROR: malloc buf1 failed!"); while(1) delay(1000); }
    lv_display_set_buffers(disp,buf1,nullptr,buf_sz,LV_DISPLAY_RENDER_MODE_FULL);

    esp_timer_handle_t lt;
    esp_timer_create_args_t la={.callback=[](void*){lv_tick_inc(2);},.arg=nullptr,.name="lv"};
    esp_timer_create(&la,&lt); esp_timer_start_periodic(lt,2000);

    lv_indev_t *indev=lv_indev_create();
    lv_indev_set_type(indev,LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev,touch_read_cb);
    lv_indev_set_scroll_limit(indev,10);

    tv=lv_tileview_create(nullptr);
    lv_obj_set_style_bg_color(tv,lv_color_hex(COL_BG),0);
    lv_obj_set_scrollbar_mode(tv,LV_SCROLLBAR_MODE_OFF);
    // Deshabilitar scroll táctil: el swipe se detecta manualmente en touch_read_cb
    // y se aplica con LV_ANIM_OFF → cero frames intermedios → sin freeze
    lv_obj_remove_flag(tv, LV_OBJ_FLAG_SCROLLABLE);

    tile_alarm   =lv_tileview_add_tile(tv,1,0,LV_DIR_BOTTOM);
    tile_sensors =lv_tileview_add_tile(tv,0,1,LV_DIR_RIGHT);
    tile_home    =lv_tileview_add_tile(tv,1,1,LV_DIR_ALL);
    tile_relays  =lv_tileview_add_tile(tv,2,1,LV_DIR_LEFT);
    tile_settings=lv_tileview_add_tile(tv,1,2,LV_DIR_TOP);

    build_tile_home();
    build_tile_hist();
    build_tile_relays();
    build_tile_alarm();
    build_tile_settings();
    build_focus_lists();
    build_scr_alarm();
    build_scr_saver();
    build_scr_pin();
    build_scr_change_pin();
    build_scr_arming();
    build_scr_dial();
    build_scr_graph();
    // scr_heatmap: lazy — se construye la primera vez que se abre

    lv_tileview_set_tile(tv,tile_home,LV_ANIM_OFF);
    lv_scr_load(tv);
    for(int i=0;i<20;i++){lv_timer_handler();delay(5);}
    Serial.println("Pantalla lista.");

    WiFiManager wm;
    // Arranque NO bloqueante: si el router aun no esta listo (p.ej. tras un corte
    // de luz), acotar el intento y el portal para no colgar el setup — asi el
    // broker y la pantalla arrancan igual. El WiFi se reconecta en segundo plano
    // (vigia en loop) cuando el router vuelve, sin tener que reiniciar a mano.
    wm.setConnectTimeout(20);         // intento de conexion acotado (s)
    wm.setConfigPortalTimeout(120);   // el portal AP se cierra tras 2 min y sigue
    bool wifi_ok = wm.autoConnect("DOMUS-Setup");
    WiFi.setAutoReconnect(true);
    if(wifi_ok){
        Serial.printf("WiFi: %s\n",WiFi.localIP().toString().c_str());
        if(lbl_settings_ip){
            char buf[40]; snprintf(buf,sizeof(buf),"IP: %s",WiFi.localIP().toString().c_str());
            lv_label_set_text(lbl_settings_ip,buf);
        }
        // OTA por WiFi — actualizar firmware sin desmontar la pantalla
        ArduinoOTA.setHostname("domus-crowpanel");
        ArduinoOTA.setPassword("domusota");
        ArduinoOTA.begin();
        ota_started = true;
        Serial.println("OTA listo (domus-crowpanel)");
    } else {
        WiFi.mode(WIFI_STA);   // asegurar modo estacion para reconectar en segundo plano
        Serial.println("WiFi no conectado al arrancar — se reintentara en segundo plano");
    }
    broker.init(1883);

    configTime(0,0,"pool.ntp.org","time.cloudflare.com");
    setenv("TZ","CET-1CEST,M3.5.0,M10.5.0/3",1); // España: UTC+1 invierno, UTC+2 verano
    tzset();
    Serial.print("NTP sync");
    time_t now=0;
    for(int i=0;i<40&&now<1000000000L;i++){delay(500);time(&now);Serial.print(".");}
    Serial.printf("%s\n",now>1000000000L?" OK":" timeout");

    hist_init();
    if(lbl_hist_tile_mem){
        size_t fu=LittleFS.usedBytes(), ft=LittleFS.totalBytes();
        char mb[36]; snprintf(mb,sizeof(mb),"Flash: %d/%d KB (%d%%)",
            (int)(fu/1024),(int)(ft/1024),ft>0?(int)(fu*100/ft):0);
        lv_label_set_text(lbl_hist_tile_mem,mb);
    }
    serial_dump_events();
    log_event(EVT_SYSTEM_BOOT, 1);

    last_touch_ms=millis();
    fetch_weather();
    // Tª/H interior ya no se baja de la nube Tuya: llega por MQTT (DOMUS/ENV, HA→Xiaomi)

    // ENC_SW detectado en BIT5 del PCF8574 (I2C 0x21), activo LOW.
    // Se lee por polling en loop() cada 25ms — no se usa interrupt de GPIO.
}

// ── LOOP ──────────────────────────────────────────────────────
static unsigned long last_status=0, flash_ts=0, countdown_ts=0;
static bool flash_state=false;
static unsigned long heat_blink_ts=0;
static bool heat_blink_state=false;

void loop() {
    ArduinoOTA.handle();
    broker.update();

    // Salir del salvapantallas: recargar la pantalla previa (fuera de callbacks de input)
    if (wake_reload_pending) {
        wake_reload_pending = false;
        lv_scr_load(scr_before_saver ? scr_before_saver : tv);
    }

    // Procesar swipe detectado en touch_read_cb (fuera de ISR/callback)
    if (sw_dc != 0 || sw_dr != 0) {
        int8_t dc = sw_dc, dr = sw_dr;
        sw_dc = 0; sw_dr = 0;
        nav_tile(dc, dr);
        // Cancelar click pendiente en LVGL para que el botón sobre el que
        // empezó el swipe no dispare su callback
        lv_indev_t *tid = lv_indev_get_next(NULL);
        if(tid) lv_indev_reset(tid, NULL);
    }

    if(siren_off_at&&millis()>=siren_off_at&&alarm_state!=AS_SOUNDING){ relay_set(3,false); siren_off_at=0; }
    if(beep_seq.total>0&&beep_seq.done<beep_seq.total&&!siren_off_at&&millis()>=beep_seq.next){
        relay_set(3,true); siren_off_at=millis()+beep_seq.dur_ms;
        beep_seq.done++; beep_seq.next=siren_off_at+beep_seq.pause_ms;
        if(beep_seq.done>=beep_seq.total) beep_seq.total=0;
    }

    if(enc_delta!=0){
        noInterrupts(); int d=enc_delta; enc_delta=0; interrupts();
        if(dial_mode!=DIAL_NONE&&dial_arc){
            // ── Dial adjustment ───────────────────────────────
            enc_accum+=d;
            int steps=enc_accum/2; enc_accum%=2;
            if(steps!=0){
                int lo  =(dial_mode==DIAL_BRIGHTNESS)?10:(dial_mode==DIAL_DIM)?0:(dial_mode==DIAL_DIM_BRIGHT)?1:15;
                int hi  =(dial_mode==DIAL_BRIGHTNESS)?255:(dial_mode==DIAL_DIM)?30:(dial_mode==DIAL_DIM_BRIGHT)?100:30;
                int mult=(dial_mode==DIAL_BRIGHTNESS)?5:1;
                dial_val=constrain(dial_val+steps*mult,lo,hi);
                lv_arc_set_value(dial_arc,dial_val);
                update_dial_display();
            }
        } else if(cur_scr==SCR_GRAPH){
            enc_accum+=d; int steps=enc_accum/2; enc_accum%=2;
            if(steps!=0){
                uint32_t avail=(hist_th<MAX_TEMP)?hist_th:MAX_TEMP;
                uint32_t win=graph_zoom_h*12;
                int32_t max_off=(avail>win)?(int32_t)(avail-win):0;
                // Velocidad escala con zoom: mín 6 muestras (30min), máx win/20 (5% ventana)
                int32_t mult=(int32_t)max(6u, win/20u);
                graph_offset_samp=constrain(graph_offset_samp+steps*mult,0,max_off);
                graph_load();
            }
        } else if(cur_scr==SCR_LOG && log_cont){
            enc_accum+=d; int steps=enc_accum/2; enc_accum%=2;
            if(steps!=0) lv_obj_scroll_by(log_cont, 0, steps*30, LV_ANIM_OFF);
        } else if(cur_scr==SCR_HIST_MENU){
            enc_accum+=d; int steps=enc_accum/2; enc_accum%=2;
            if(steps!=0){
                Serial.printf("[ENC] hist d=%d steps=%d sel:%d->%d\n",d,steps,hist_menu_sel,((hist_menu_sel+steps)%4+4)%4);
                hist_menu_set_sel(((hist_menu_sel+steps)%4+4)%4);
            }
        } else if(cur_scr==SCR_HEATMAP){
            enc_accum+=d; int steps=enc_accum/2; enc_accum%=2;
            if(steps!=0){
                heatmap_goto_week(hm_week_off + steps); // CCW→semanas pasadas, CW→volver al presente
            }
        } else if(cur_scr==SCR_TV){
            // Encoder activity: wake screen if dimmed
            last_touch_ms=millis();
            if(screen_dimmed) request_wake(true);
            // ── Focus mode: move between items ────────────────
            if(enc_mode==ENC_FOCUS){
                enc_accum+=d; int steps=enc_accum/2; enc_accum%=2;
                if(steps!=0){
                    FocusList &fl=tile_focus[tile_cycle_idx];
                    if(fl.count>0){
                        focus_set_outline(fl.items[focus_idx],false,fl.home_nav[focus_idx]);
                        focus_idx=((focus_idx+steps)%fl.count+fl.count)%fl.count;
                        focus_set_outline(fl.items[focus_idx],true,fl.home_nav[focus_idx]);
                        focus_blink_st=true; focus_blink_ts=millis();
                    }
                    focus_idle_ms=millis();
                }
            } else {
                // ── Tile cycle ────────────────────────────────
                enc_accum+=d; int steps=enc_accum/2; enc_accum%=2;
                if(steps!=0){
                    tile_cycle_idx=((tile_cycle_idx+steps)%5+5)%5;
                    tile_col=TC_COL[tile_cycle_idx]; tile_row=TC_ROW[tile_cycle_idx];
                    lv_tileview_set_tile(tv,tile_by_cycle(tile_cycle_idx),cfg_anim?LV_ANIM_ON:LV_ANIM_OFF);
                }
            }
        }
    }

    // ── Botón encoder: PCF8574 BIT5, activo LOW ──────────────────
    {
        static uint8_t       pcf_btn_prev = 1;
        static unsigned long pcf_btn_ts   = 0;
        if(millis()-pcf_btn_ts >= 25){
            pcf_btn_ts=millis();
            Wire.requestFrom((uint8_t)PCF8574_ADDR, (uint8_t)1);
            if(Wire.available()){
                uint8_t v = Wire.read();
                uint8_t bit5 = (v >> 5) & 1;
                if(bit5==0 && pcf_btn_prev==1 && millis()-enc_btn_last_ms>200){
                    enc_btn_last_ms=millis();
                    last_touch_ms=millis();
                    if(screen_dimmed) request_wake(true);
                    if(dial_mode!=DIAL_NONE){
                        apply_dial(); dial_mode=DIAL_NONE; go_to(SCR_TV);
                    } else if(cur_scr==SCR_PIN||cur_scr==SCR_CHANGE_PIN){
                        memset(pin_buf,0,5); pin_len=0; go_to(SCR_TV);
                    } else if(cur_scr==SCR_GRAPH||cur_scr==SCR_HEATMAP||cur_scr==SCR_LOG||cur_scr==SCR_PROG){
                        go_to(SCR_TV);
                    } else if(cur_scr==SCR_HIST_MENU){
                        switch(hist_menu_sel){
                            case 0: cb_hist_temp(nullptr); break;
                            case 1: go_to(SCR_HEATMAP);   break;
                            case 2: log_day_off=0; go_to(SCR_LOG); break;
                            default: go_to(SCR_TV);        break;
                        }
                    } else if(cur_scr==SCR_TV){
                        if(enc_mode==ENC_IDLE) focus_enter();
                        else exec_focus_item();
                    }
                }
                pcf_btn_prev=bit5;
            }
        }
    }
    // ─────────────────────────────────────────────────────────────

    // ── Reposo de pantalla: ACTIVA → DIM → APAGADO/salvapantallas ─
    // (no atenuar/apagar con alarma activa: GRACE/SOUNDING)
    if(alarm_state<AS_GRACE){
        unsigned long idle = millis()-last_touch_ms;
        // 1) DIM
        if(!screen_dimmed && cfg_dim_delay>0 && idle>(unsigned long)cfg_dim_delay*60000UL)
            enter_dim();
        // 2) APAGADO / salvapantallas
        if(!screen_off && cfg_off_delay>0 && idle>(unsigned long)cfg_off_delay*60000UL)
            enter_off();
        // 3) Tick del salvapantallas: recolocar + refrescar cada 10s; si modo
        //    "solo calefaccion" y ya no hay calefaccion, apagar del todo.
        if(saver_active){
            static unsigned long saver_tick=0;
            if(millis()-saver_tick>=10000UL){
                saver_tick=millis();
                if(cfg_saver_mode==2 && heat_mode==HM_OFF){
                    saver_active=false; lv_scr_load(scr_before_saver?scr_before_saver:tv);
                    set_brightness(0);
                } else {
                    saver_update_content(); saver_reposition();
                }
            }
        }
    }

    if(alarm_state==AS_ARMING){
        unsigned long now=millis();
        if(now>=arming_end_ms){
            alarm_state=AS_ARMED; alarm_armed=true;
            log_event(EVT_ALARM_ARM, 1);
            pin_for_arm=false; memset(pin_buf,0,5); pin_len=0;
            go_to(SCR_PIN);
        } else if(now-countdown_ts>=500){
            countdown_ts=now;
            unsigned long rem=(arming_end_ms-now)/1000UL;
            if(lbl_countdown){ char buf[8]; snprintf(buf,sizeof(buf),"%lu:%02lu",rem/60,rem%60); lv_label_set_text(lbl_countdown,buf); }
        }
    }

    if(alarm_state==AS_GRACE){
        unsigned long e=millis()-alarm_ts;
        if(e>=5000 &&!grace_beeps[0]){grace_beeps[0]=true;start_beep_seq(1,100,200);}
        if(e>=30000&&!grace_beeps[1]){grace_beeps[1]=true;start_beep_seq(2,100,300);}
        if(e>=60000&&!grace_beeps[2]){grace_beeps[2]=true;alarm_state=AS_SOUNDING;beep_seq={};siren_off_at=0;relay_set(3,true);}
    }

    if(cur_scr==SCR_ALARM_CRIT&&millis()-flash_ts>=400){
        flash_state=!flash_state; flash_ts=millis();
        lv_obj_set_style_bg_color(scr_alarm,lv_color_hex(flash_state?0x3A0000:0x100000),0);
    }
    if(cur_scr==SCR_ARMING_SCR&&millis()-flash_ts>=500){
        flash_state=!flash_state; flash_ts=millis();
        lv_obj_set_style_bg_color(scr_arming,lv_color_hex(flash_state?0xFFFFFF:0x1010CC),0);
    }

    // Sensor + relay alert blink (shared 400ms timer, synchronized)
    if(harc[1]&&millis()-sensor_blink_ts>=400){
        sensor_blink_ts=millis(); sensor_blink_st=!sensor_blink_st;
        // Sensores
        if(harc_alert[1]) lv_obj_set_style_bg_color(harc[1],lv_color_hex(sensor_blink_st?COL_ALERT:COL_OK_DIM),0);
        if(harc_alert[2]) lv_obj_set_style_bg_color(harc[2],lv_color_hex(sensor_blink_st?COL_ALERT:COL_OK_DIM),0);
        if(harc_alert[3]) lv_obj_set_style_bg_color(harc[3],lv_color_hex(sensor_blink_st?COL_ALERT:COL_OK_DIM),0);
        if(harc_alert[4]) lv_obj_set_style_bg_color(harc[4],lv_color_hex(sensor_blink_st?COL_ALERT:COL_OK),0);
        // Relés: los tres parpadean rojo en alerta
        if(rarc_alert[1]) lv_obj_set_style_bg_color(rarc[1],lv_color_hex(sensor_blink_st?COL_ALERT:COL_RELAY_DIM),0);
        if(rarc_alert[2]) lv_obj_set_style_bg_color(rarc[2],lv_color_hex(sensor_blink_st?COL_ALERT:COL_RELAY_DIM),0);
        if(rarc_alert[3]) lv_obj_set_style_bg_color(rarc[3],lv_color_hex(sensor_blink_st?COL_ALERT:COL_RELAY_DIM),0);
    }

    // Focus HOME-nav item blink (outline opa toggle)
    if(enc_mode==ENC_FOCUS&&millis()-focus_blink_ts>=350){
        focus_blink_ts=millis(); focus_blink_st=!focus_blink_st;
        FocusList &fl=tile_focus[tile_cycle_idx];
        if(fl.count>0&&fl.home_nav[focus_idx]&&fl.items[focus_idx])
            lv_obj_set_style_outline_opa(fl.items[focus_idx],
                focus_blink_st?LV_OPA_COVER:LV_OPA_TRANSP, LV_PART_MAIN);
    }

    // Focus timeout — 30s idle → exit focus and return to home
    if(enc_mode==ENC_FOCUS&&cur_scr==SCR_TV&&millis()-focus_idle_ms>30000UL){
        focus_exit();
        tile_cycle_idx=0; tile_col=1; tile_row=1;
        lv_tileview_set_tile(tv,tile_home,cfg_anim?LV_ANIM_ON:LV_ANIM_OFF);
    }

    // Heating indicator blink — muestra cuando modo consigna (o futuro programa) activo
    if(lbl_heat_active){
        if(heat_mode!=HM_OFF){
            if(millis()-heat_blink_ts>=500){
                heat_blink_ts=millis();
                heat_blink_state=!heat_blink_state;
                if(heat_blink_state) lv_obj_clear_flag(lbl_heat_active,LV_OBJ_FLAG_HIDDEN);
                else                 lv_obj_add_flag(lbl_heat_active,LV_OBJ_FLAG_HIDDEN);
            }
        } else {
            lv_obj_add_flag(lbl_heat_active,LV_OBJ_FLAG_HIDDEN);
            heat_blink_state=false;
        }
    }

    if(ui_needs_update){
        check_alarms(); update_home(); update_sensors_tile(); update_relays_tile(); update_alarm_tile();
        check_heating_auto();

        // ── Log cambios de estado ─────────────────────────────
        // Relés
        if(a6v3.output[1]!=hp_output[1]){
            log_event(EVT_RELAY_AGUA,  a6v3.output[1]?1:0);
            hp_output[1]=a6v3.output[1];
        }
        if(a6v3.output[2]!=hp_output[2]){
            log_event(EVT_RELAY_CALEF, a6v3.output[2]?1:0, (uint8_t)heat_mode);
            if(a6v3.output[2]) wake_on_event();   // calefaccion al arrancar enciende pantalla
            hp_output[2]=a6v3.output[2];
        }
        if(a6v3.output[3]!=hp_output[3]){
            log_event(EVT_RELAY_SIRENA,a6v3.output[3]?1:0);
            hp_output[3]=a6v3.output[3];
        }
        // Presencia (input[4] invertido: false = hay alguien)
        if(a6v3.input[4]!=hp_input[4]){
            bool pres=!a6v3.input[4];
            log_event(EVT_PRESENCE, pres?1:0);  // el mapa de presencia se reconstruye de aquí
            hp_input[4]=a6v3.input[4];
        }
        // Inundación (input[1]: true = activo)
        if(a6v3.input[1]!=hp_input[1]){
            log_event(EVT_FLOOD, a6v3.input[1]?1:0);
            wake_on_event();
            hp_input[1]=a6v3.input[1];
        }
        // Humo (input[5] invertido: false = humo)
        if(a6v3.input[5]!=hp_input[5]){
            log_event(EVT_SMOKE, !a6v3.input[5]?1:0);
            wake_on_event();
            hp_input[5]=a6v3.input[5];
        }
        // 220V (input[6]): cambio enciende pantalla (sin log; el debounce esta en HA)
        if(a6v3.input[6]!=hp_input[6]){
            wake_on_event();
            hp_input[6]=a6v3.input[6];
        }
        // Nota: presencia/PIR (input[4], "el hall") NO enciende la pantalla por
        // si misma; solo lo hace via check_alarms cuando dispara la intrusion.
        // ─────────────────────────────────────────────────────

        ui_needs_update=false;
    }

    // ── Timers de historial ───────────────────────────────────
    static unsigned long last_temp_log = 0;
    if(millis()-last_temp_log >= 300000UL){ // cada 5 min
        last_temp_log=millis();
        log_temp(tuya_temp_int, wx.ok ? wx.temp : NAN);
    }
    // Volcado a flash con la pantalla en dim (atenuada, el glitch no se ve), pero solo
    // si hay >=50 acumulados → agrupa escrituras y reduce desgaste de flash.
    if(screen_dimmed && evt_buf_n>=50) flush_events();
    // Chivato del buffer por MQTT cada 8s (telemetria; quitar tras diagnostico)
    { static unsigned long dbg_last=0;
      if(millis()-dbg_last>=8000UL){ dbg_last=millis(); mqtt_publish_debug(); } }
    // ─────────────────────────────────────────────────────────

    // ── Comandos MQTT remotos (DOMUS/CMD) ────────────────────────
    if(mqtt_cmd_disarm){
        mqtt_cmd_disarm=false;
        alarm_state=AS_OFF; alarm_armed=false; intruder_active=false;
        critical_alert=false; beep_seq={}; siren_off_at=0;
        relay_set(3,false); log_event(EVT_ALARM_DISARM,1);
        go_to(SCR_TV); ui_needs_update=true;
    }
    if(mqtt_cmd_arm){
        mqtt_cmd_arm=false;
        if(alarm_state==AS_OFF && !alarm_armed){
            alarm_state=AS_ARMING; alarm_armed=true;
            alarm_ts=millis(); arming_end_ms=alarm_ts+120000UL;
            start_beep_seq(1,100); log_event(EVT_ALARM_ARM,0);
            go_to(SCR_ARMING_SCR);
        }
    }
    // ─────────────────────────────────────────────────────────────

    if(scr_change){ scr_change=false; do_switch(pend_scr); }
    if(millis()-wx_last  >30UL*60000UL) fetch_weather();
    // (Tª interior llega por MQTT DOMUS/ENV; ya no se consulta la nube Tuya)
    { static unsigned long domus_pub_last=0;
      if(millis()-domus_pub_last>=60000UL){ domus_pub_last=millis(); mqtt_publish_status(); } }

    // Vigía WiFi: si el router no estaba listo al arrancar (corte de luz) o se cae,
    // reconectar en segundo plano sin bloquear. Al recuperar WiFi, arrancar el OTA
    // (que necesita red) si aún no estaba activo, y refrescar la IP en Ajustes.
    { static unsigned long wifi_chk=0;
      if(millis()-wifi_chk >= 10000UL){
          wifi_chk=millis();
          if(WiFi.status()!=WL_CONNECTED){
              WiFi.reconnect();
          } else if(!ota_started){
              ArduinoOTA.setHostname("domus-crowpanel");
              ArduinoOTA.setPassword("domusota");
              ArduinoOTA.begin();
              ota_started=true;
              if(lbl_settings_ip){
                  char b[40]; snprintf(b,sizeof(b),"IP: %s",WiFi.localIP().toString().c_str());
                  lv_label_set_text(lbl_settings_ip,b);
              }
              Serial.println("OTA listo (reconexion WiFi en segundo plano)");
          }
      } }

    if(millis()-last_status>=5000){
        last_status=millis();
        update_broker_status();
        Serial.printf("[S] flood=%d pir=%d smoke=%d pwr=%d | agua=%d calef=%d sir=%d | heat=%d sp=%d int=%.1f\n",
            a6v3.input[1],a6v3.input[4],a6v3.input[5],a6v3.input[6],
            a6v3.output[1],a6v3.output[2],a6v3.output[3],
            (int)heat_mode,heat_setpoint,tuya_temp_int);
    }

    // Reintento touch si no se encontró en setup (IC puede necesitar >100ms tras reset)
    if(!touch_i2c_addr){
        static unsigned long touch_retry_ts = 0;
        if(millis()-touch_retry_ts > 2000){ touch_retry_ts=millis(); touch_init();
            if(touch_i2c_addr) Serial.printf("[TOUCH] Found on retry: 0x%02X\n", touch_i2c_addr); }
    }

    lv_timer_handler();
    delay(2);
}
