#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "Arduino_GFX_Library.h"
#include <Adafruit_XCA9554.h>
#include <Adafruit_GFX.h>
#include "pin_config.h"
#include <Wire.h>
#include <time.h>
#include "secrets.h"  //rename and fill secrets_template.h to secrets.h

// --- FONTY ---
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

// ---------------------------------------------------------
// NASTAVENÍ SÍTĚ A TISKÁRNY
// ---------------------------------------------------------
const char* ssid          = MY_SSID;
const char* password      = MY_PASSWORD;
const char* mqtt_server   = MY_MQTT_SERVER;
const int   mqtt_port     = 8883;
const char* mqtt_user     = "bblp";
const char* mqtt_password = MY_MQTT_PASSWORD;
const char* serial_number = SERIAL_NUMBER;

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ---------------------------------------------------------
// MAPOVÁNÍ TRYSEK / ČASOVAČE
// ---------------------------------------------------------
#define LEFT_NOZZLE_ID     1        // id LEVÉ trysky (přehoď na 0, kdyby se prohodily)
#define ACTIVE_TARGET_MIN  40.0     // cíl nad touto hodnotou = aktivní tryska
#define FINISH_NOTIFY_MS   300000UL // notifikace "Finished" zobrazená 5 min, pak hodiny

// ---------------------------------------------------------
// HARDWARE
// ---------------------------------------------------------
Adafruit_XCA9554 expander;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_SH8601 *gfx = new Arduino_SH8601(bus, GFX_NOT_DEFINED, 0, LCD_WIDTH, LCD_HEIGHT);

// Jeden off-screen buffer na celou šířku – kreslíme po pásech a posíláme najednou
// (žádné blikání). 368 x 130 px.
#define FB_W 368
#define FB_H 130
GFXcanvas16 *fb = nullptr;

// ---------------------------------------------------------
// BAREVNÁ PALETA
// ---------------------------------------------------------
#define COLOR_BG            0x0000
#define COLOR_CARD          0x10A2
#define COLOR_CARD_ACTIVE   0x2965
#define COLOR_TEXT          0xFFFF
#define COLOR_SUBTEXT       0xD6BA
#define COLOR_BAMBU         0x05E8
#define COLOR_NOZZLE        0xF800   // červená (horké / topení)
#define COLOR_BED           0x041F   // modrá (chlazení)
#define COLOR_GRAY          0x7BEF
#define COLOR_GREEN         0x07E0   // notifikace dokončeno
#define COLOR_ORANGE        0xFD20   // notifikace pauza
#define COLOR_RED           0xF800   // notifikace chyba

// ---------------------------------------------------------
// PROMĚNNÉ
// ---------------------------------------------------------
float left_nozzle_temp = 0, left_nozzle_target = 0;
float right_nozzle_temp = 0, right_nozzle_target = 0;
int   left_tray = -1, right_tray = -1;
int   g_active_side = -1;

float bed_temp = 0, bed_target = 0;
float chamber_temp = 0, chamber_target = 0;
int   air_mode = -1;                 // 0 = cooling, 1 = heating

int mc_percent = 0, remaining_time_min = 0;
int current_layer = 0, total_layers = 0;
int print_error = 0;
int stg_cur = -99;                   // aktuální fáze tisku (status message)
String gcode_state = "IDLE";

// --- FILAMENT ---
struct Filament { String type = ""; uint16_t color = 0x0000; bool valid = false; };
Filament ams_filaments[16];
Filament ext_filament;

// --- REŽIM DISPLEJE ---
enum DispMode { M_DASH, M_GREEN, M_ORANGE, M_RED, M_CLOCK };
DispMode last_mode = (DispMode)-1;
int last_clock_min = -1;

// --- CACHE PRO PŘEKRESLENÍ ---
float last_l_t = -1, last_l_tgt = -1, last_r_t = -1, last_r_tgt = -1;
int   last_l_tray = -99, last_r_tray = -99, last_l_active = -1, last_r_active = -1;
float last_bed_t = -1, last_bed_tgt = -1;
float last_cham_t = -1, last_cham_tgt = -1;
int   last_air_mode = -99, last_perc = -1, last_rem_min = -1;
int   last_cur_layer = -1, last_tot_layer = -1;
int   last_stg = -100;
bool  force_redraw = true;

// ---------------------------------------------------------
// POMOCNÉ FUNKCE
// ---------------------------------------------------------
uint16_t hexToRGB565(String hex) {
  if (hex.length() < 6) return COLOR_GRAY;
  long rgb = strtol(hex.substring(0, 6).c_str(), NULL, 16);
  uint8_t r = (rgb >> 16) & 0xFF, g = (rgb >> 8) & 0xFF, b = rgb & 0xFF;
  if (r < 40 && g < 40 && b < 40) return COLOR_GRAY;
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static void unpackTemp(uint32_t packed, float &cur, float &tgt) {
  cur = (float)(packed & 0xFFFF);
  tgt = (float)((packed >> 16) & 0xFFFF);
  if (cur > 500) cur = 0;
  if (tgt > 500) tgt = 0;
}

static int snowToTray(int snow) {
  if (snow < 0 || snow >= 65534) return -1;
  if (snow == 254 || snow == 255) return snow;
  return (snow / 256) * 4 + (snow % 256);
}

Filament filamentForTray(int tray) {
  if (tray >= 0 && tray < 16) return ams_filaments[tray];
  if (tray == 254 || tray == 255) return ext_filament;
  Filament empty; return empty;
}

String formatTimeStr(int mins) {
  if (mins <= 0) return "0h 0m";
  return String(mins / 60) + "h " + String(mins % 60) + "m";
}

String getETAStr(int rem_min) {
  if (rem_min <= 0) return "--:--";
  time_t now; time(&now);
  if (now < 1000000000) return "Sync...";
  time_t finish = now + (rem_min * 60);
  struct tm *ti = localtime(&finish);
  char buf[10]; sprintf(buf, "%02d:%02d", ti->tm_hour, ti->tm_min);
  return String(buf);
}

// Vycentrovaný text přímo na displej (hodiny, notifikace).
void printCentered(String text, int cx, int by, const GFXfont *font, uint16_t color) {
  gfx->setFont(font); gfx->setTextSize(1); gfx->setTextColor(color);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(text, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(cx - (tw / 2), by); gfx->print(text);
}

// Vycentrovaný text do off-screen bufferu.
void fbCentered(String text, int cx, int by, const GFXfont *font, uint16_t color) {
  fb->setFont(font); fb->setTextSize(1); fb->setTextColor(color);
  int16_t x1, y1; uint16_t tw, th;
  fb->getTextBounds(text, 0, 0, &x1, &y1, &tw, &th);
  fb->setCursor(cx - (tw / 2), by); fb->print(text);
}

// Vykreslí jednu kartu do bufferu na lokální pozici (cx, cy).
void drawCardInto(int cx, int cy, int w, int h, String title, String val1, String val2,
                  uint16_t valColor, bool isActive, Filament f = Filament()) {
  uint16_t bg = isActive ? COLOR_CARD_ACTIVE : COLOR_CARD;
  fb->fillRoundRect(cx, cy, w, h, 12, bg);
  fbCentered(title, cx + w / 2, cy + 30, &FreeSans9pt7b, isActive ? COLOR_TEXT : COLOR_GRAY);
  fbCentered(val1,  cx + w / 2, cy + 70, &FreeSansBold18pt7b, valColor);
  if (val2 != "") fbCentered(val2, cx + w / 2, cy + 100, &FreeSans9pt7b, isActive ? COLOR_SUBTEXT : COLOR_GRAY);

  if (f.valid && f.type != "") {
    String s = f.type;
    if (s.length() > 10) s = s.substring(0, 8) + "..";
    fb->setFont(&FreeSans9pt7b); fb->setTextSize(1);
    int16_t bx, by; uint16_t tw, th;
    fb->getTextBounds(s, 0, 0, &bx, &by, &tw, &th);
    int total = 14 + tw, sx = cx + (w - total) / 2;
    fb->fillCircle(sx + 4, cy + h - 15, 5, f.color);
    fb->setTextColor(COLOR_TEXT);
    fb->setCursor(sx + 14, cy + h - 10); fb->print(s);
  }
}

// ---------------------------------------------------------
// PÁSY DASHBOARDU (každý se pošle na displej jedním blokem)
// ---------------------------------------------------------
void renderTop(bool la, bool ra) {
  if (!fb) return;
  fb->fillScreen(COLOR_BG);
  drawCardInto(10, 0, 169, 130, "Left Nozzle",
               String(left_nozzle_temp, 0) + " C", "Tgt: " + String(left_nozzle_target, 0),
               COLOR_NOZZLE, la, filamentForTray(left_tray));
  drawCardInto(189, 0, 169, 130, "Right Nozzle",
               String(right_nozzle_temp, 0) + " C", "Tgt: " + String(right_nozzle_target, 0),
               COLOR_NOZZLE, ra, filamentForTray(right_tray));
  gfx->draw16bitRGBBitmap(0, 15, fb->getBuffer(), 368, 130);
}

void renderMid() {
  if (!fb) return;
  uint16_t cc = COLOR_TEXT;
  if (air_mode == 1)      cc = COLOR_NOZZLE;  // heating -> červená
  else if (air_mode == 0) cc = COLOR_BED;     // cooling -> modrá
  fb->fillScreen(COLOR_BG);
  drawCardInto(10, 0, 169, 110, "Heatbed",
               String(bed_temp, 0) + " C", "Tgt: " + String(bed_target, 0), COLOR_BED, false);
  drawCardInto(189, 0, 169, 110, "Chamber",
               String(chamber_temp, 0) + " C", "Tgt: " + String(chamber_target, 0), cc, false);
  gfx->draw16bitRGBBitmap(0, 155, fb->getBuffer(), 368, 110);
}

// Aktuální fáze tisku (stg_cur) -> text. Pro neznámé/H2D kódy vrací "".
String stageText(int s) {
  switch (s) {
    case 0:  return "Printing";
    case 1:  return "Auto bed leveling";
    case 2:  return "Heatbed preheating";
    case 3:  return "Vibration compensation";
    case 4:  return "Changing filament";
    case 5:  return "M400 pause";
    case 6:  return "Filament runout";
    case 7:  return "Heating hotend";
    case 8:  return "Calibrating extrusion";
    case 9:  return "Scanning bed surface";
    case 10: return "Inspecting first layer";
    case 11: return "Identifying build plate";
    case 12: return "Calibrating Micro Lidar";
    case 13: return "Homing toolhead";
    case 14: return "Cleaning nozzle";
    case 15: return "Checking extruder temp";
    case 16: return "Paused by user";
    case 17: return "Front cover fell off";
    case 18: return "Calibrating Micro Lidar";
    case 19: return "Calibrating flow";
    case 20: return "Nozzle temp error";
    case 21: return "Bed temp error";
    case 22: return "Filament unloading";
    case 23: return "Skip step pause";
    case 24: return "Filament loading";
    case 25: return "Motor noise calibration";
    case 26: return "AMS lost";
    case 27: return "Heat-break fan slow";
    case 28: return "Chamber temp error";
    case 29: return "Cooling chamber";
    case 30: return "Paused (Gcode)";
    case 31: return "Calibrating motor";
    default: return "";
  }
}

void renderBottom() {
  if (!fb) return;
  fb->fillScreen(COLOR_BG);
  fbCentered("End: " + getETAStr(remaining_time_min), 70, 14, &FreeSans9pt7b, COLOR_GRAY);
  fbCentered("Rem: " + formatTimeStr(remaining_time_min), 298, 14, &FreeSans9pt7b, COLOR_GRAY);
  fb->fillRoundRect(27, 28, 314, 22, 6, COLOR_CARD);
  int barWidth = map(mc_percent, 0, 100, 0, 314);
  if (barWidth > 0) fb->fillRoundRect(27, 28, barWidth, 22, 6, COLOR_BAMBU);
  fbCentered(String(mc_percent) + " %", 100, 90, &FreeSansBold24pt7b, COLOR_TEXT);
  fbCentered("Layer " + String(current_layer) + " / " + String(total_layers), 260, 82, &FreeSans12pt7b, COLOR_GRAY);

  // Status fáze tisku pod progress barem
  String st = stageText(stg_cur);
  if (st == "" && gcode_state == "RUNNING") st = "Printing";
  if (st != "") fbCentered(st, 184, 120, &FreeSans12pt7b, COLOR_SUBTEXT);

  gfx->draw16bitRGBBitmap(0, 285, fb->getBuffer(), 368, 130);
}

void drawDashboard() {
  if (force_redraw) {
    gfx->fillScreen(COLOR_BG); force_redraw = false;
    last_l_t = last_l_tgt = last_r_t = last_r_tgt = -1;
    last_l_tray = last_r_tray = -99; last_l_active = last_r_active = -1;
    last_bed_t = last_bed_tgt = -1; last_cham_t = last_cham_tgt = -1; last_air_mode = -99;
    last_perc = last_rem_min = -1; last_cur_layer = last_tot_layer = -1; last_stg = -100;
  }

  bool la = (g_active_side == 0), ra = (g_active_side == 1);
  if (last_l_t != left_nozzle_temp || last_l_tgt != left_nozzle_target || last_l_tray != left_tray || last_l_active != (int)la ||
      last_r_t != right_nozzle_temp || last_r_tgt != right_nozzle_target || last_r_tray != right_tray || last_r_active != (int)ra) {
    renderTop(la, ra);
    last_l_t = left_nozzle_temp; last_l_tgt = left_nozzle_target; last_l_tray = left_tray; last_l_active = (int)la;
    last_r_t = right_nozzle_temp; last_r_tgt = right_nozzle_target; last_r_tray = right_tray; last_r_active = (int)ra;
  }

  if (last_bed_t != bed_temp || last_bed_tgt != bed_target ||
      last_cham_t != chamber_temp || last_cham_tgt != chamber_target || last_air_mode != air_mode) {
    renderMid();
    last_bed_t = bed_temp; last_bed_tgt = bed_target;
    last_cham_t = chamber_temp; last_cham_tgt = chamber_target; last_air_mode = air_mode;
  }

  if (last_perc != mc_percent || last_rem_min != remaining_time_min ||
      last_cur_layer != current_layer || last_tot_layer != total_layers || last_stg != stg_cur) {
    renderBottom();
    last_perc = mc_percent; last_rem_min = remaining_time_min;
    last_cur_layer = current_layer; last_tot_layer = total_layers; last_stg = stg_cur;
  }
}

// ---------------------------------------------------------
// NOTIFIKACE A HODINY (přes celý displej)
// ---------------------------------------------------------
void drawNotification(uint16_t bg, uint16_t fg, const char* title, String sub) {
  gfx->fillScreen(bg);
  gfx->setFont(&FreeSansBold24pt7b); gfx->setTextSize(1); gfx->setTextColor(fg);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(184 - tw / 2, 215); gfx->print(title);
  if (sub != "") printCentered(sub, 184, 260, &FreeSans12pt7b, fg);
}

void drawClockUI() {
  time_t now; time(&now); struct tm *ti = localtime(&now);
  if (ti->tm_year < 100) return;
  if (last_clock_min != ti->tm_min) {
    gfx->fillRect(0, 150, 368, 180, COLOR_BG);
    char buf[10]; sprintf(buf, "%02d:%02d", ti->tm_hour, ti->tm_min);
    gfx->setFont(&FreeSansBold24pt7b); gfx->setTextSize(2); gfx->setTextColor(COLOR_TEXT);
    int16_t x1, y1; uint16_t tw, th; gfx->getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
    gfx->setCursor(184 - (tw / 2), 250); gfx->print(buf);
    char dateBuf[20]; sprintf(dateBuf, "%02d.%02d.%04d", ti->tm_mday, ti->tm_mon + 1, ti->tm_year + 1900);
    printCentered(dateBuf, 184, 290, &FreeSans12pt7b, COLOR_GRAY);
    last_clock_min = ti->tm_min;
  }
}

// ---------------------------------------------------------
// VÝBĚR REŽIMU
// ---------------------------------------------------------
DispMode computeMode() {
  bool err    = (gcode_state == "FAILED") || (print_error != 0);
  bool paused = (gcode_state == "PAUSE");
  bool active = (gcode_state == "RUNNING" || gcode_state == "PREPARE" || gcode_state == "SLICING");

  static bool finishLatch = false;
  static unsigned long finishAt = 0;

  if (active || err || paused) finishLatch = false;
  if (gcode_state == "FINISH" && !finishLatch && !active && !err && !paused) {
    finishLatch = true; finishAt = millis();
  }

  if (err)    return M_RED;
  if (paused) return M_ORANGE;
  if (active) return M_DASH;
  if (finishLatch && (millis() - finishAt < FINISH_NOTIFY_MS)) return M_GREEN;
  return M_CLOCK;   // nečinnost / standby (i po 5 min od dokončení)
}

void updateUI() {
  DispMode mode = computeMode();

  if (mode != last_mode) {                 // vstup do nového režimu -> jednorázové překreslení
    last_mode = mode;
    last_clock_min = -1;
    if      (mode == M_DASH)   force_redraw = true;
    else if (mode == M_GREEN)  drawNotification(COLOR_GREEN,  0x0000, "FINISHED", "Print complete");
    else if (mode == M_ORANGE) drawNotification(COLOR_ORANGE, 0x0000, "PAUSED",   "Print paused");
    else if (mode == M_RED)    drawNotification(COLOR_RED,    0xFFFF, "ERROR",
                                  print_error != 0 ? ("Code " + String(print_error)) : "Check printer");
    else if (mode == M_CLOCK)  gfx->fillScreen(COLOR_BG);
  }

  if (mode == M_DASH)  drawDashboard();
  else if (mode == M_CLOCK) drawClockUI();
  // notifikační režimy: overlay zůstává až do vyřešení, nic dalšího se nekreslí
}

// ---------------------------------------------------------
// MQTT CALLBACK
// ---------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, length)) return;
  if (!doc.containsKey("print")) return;
  JsonObject p = doc["print"];

  // --- TRYSKY: print.device.extruder.info[] (temp = (target<<16)|current) ---
  bool dual_parsed = false;
  if (p["device"].is<JsonObject>() && p["device"]["extruder"]["info"].is<JsonArray>()) {
    for (JsonObject e : p["device"]["extruder"]["info"].as<JsonArray>()) {
      int id = e.containsKey("id") ? e["id"].as<int>() : -1;
      bool isLeft  = (id == LEFT_NOZZLE_ID);
      bool isRight = (id == (1 - LEFT_NOZZLE_ID));
      if (!isLeft && !isRight) continue;

      float cur = 0, tgt = 0;
      if (e.containsKey("temp")) unpackTemp(e["temp"].as<uint32_t>(), cur, tgt);
      int tray = e.containsKey("snow") ? snowToTray(e["snow"].as<int>()) : -1;

      if (isLeft) { left_nozzle_temp = cur; left_nozzle_target = tgt; left_tray = tray; }
      else        { right_nozzle_temp = cur; right_nozzle_target = tgt; right_tray = tray; }
      dual_parsed = true;
    }
    if (left_nozzle_target >= ACTIVE_TARGET_MIN || right_nozzle_target >= ACTIVE_TARGET_MIN) {
      if (left_nozzle_target > right_nozzle_target)      g_active_side = 0;
      else if (right_nozzle_target > left_nozzle_target) g_active_side = 1;
    } else {
      g_active_side = -1;
    }
  }

  if (!dual_parsed) {  // záloha pro jednotryskový formát
    if (p.containsKey("nozzle_temper")) {
      float v = p["nozzle_temper"].as<float>();
      if (g_active_side == 1) right_nozzle_temp = v; else left_nozzle_temp = v;
    }
    if (p.containsKey("nozzle_target_temper")) {
      float v = p["nozzle_target_temper"].as<float>();
      if (g_active_side == 1) right_nozzle_target = v; else left_nozzle_target = v;
    }
  }

  // --- PODLOŽKA / KOMORA ---
  if (p.containsKey("bed_temper"))        bed_temp = p["bed_temper"].as<float>();
  if (p.containsKey("bed_target_temper")) bed_target = p["bed_target_temper"].as<float>();

  if (p["device"]["ctc"]["info"]["temp"].is<float>() || p["device"]["ctc"]["info"]["temp"].is<int>())
    chamber_temp = p["device"]["ctc"]["info"]["temp"].as<float>();
  else if (p.containsKey("chamber_temper")) chamber_temp = p["chamber_temper"].as<float>();

  if (p.containsKey("chamber_target_temper")) chamber_target = p["chamber_target_temper"].as<float>();
  if (p["device"]["airduct"]["modeCur"].is<int>()) air_mode = p["device"]["airduct"]["modeCur"].as<int>();

  // --- PROGRES / STAV ---
  if (p.containsKey("mc_percent"))        mc_percent = p["mc_percent"].as<int>();
  if (p.containsKey("mc_remaining_time")) remaining_time_min = p["mc_remaining_time"].as<int>();
  if (p.containsKey("layer_num"))         current_layer = p["layer_num"].as<int>();
  if (p.containsKey("total_layer_num"))   total_layers = p["total_layer_num"].as<int>();
  if (p.containsKey("gcode_state"))       gcode_state = p["gcode_state"].as<String>();
  if (p.containsKey("print_error"))       print_error = p["print_error"].as<int>();
  if (p.containsKey("stg_cur"))           stg_cur = p["stg_cur"].as<int>();

  // --- AMS: tabulka filamentů ---
  if (p["ams"]["ams"].is<JsonArray>()) {
    for (JsonObject a : p["ams"]["ams"].as<JsonArray>()) {
      int ams_id = a["id"].as<int>();
      for (JsonObject t : a["tray"].as<JsonArray>()) {
        int abs_id = ams_id * 4 + t["id"].as<int>();
        if (abs_id >= 0 && abs_id < 16) {
          ams_filaments[abs_id].valid = true;
          if (t.containsKey("tray_sub_brands") && t["tray_sub_brands"].as<String>() != "")
            ams_filaments[abs_id].type = t["tray_sub_brands"].as<String>();
          else if (t.containsKey("tray_type"))
            ams_filaments[abs_id].type = t["tray_type"].as<String>();
          if (t.containsKey("tray_color"))
            ams_filaments[abs_id].color = hexToRGB565(t["tray_color"].as<String>());
        }
      }
    }
  }
}

// ---------------------------------------------------------
// SETUP / LOOP
// ---------------------------------------------------------
void setup() {
  Wire.begin(IIC_SDA, IIC_SCL);
  if (!expander.begin(0x20)) while (1);
  expander.pinMode(4, OUTPUT); expander.pinMode(5, OUTPUT);
  expander.digitalWrite(4, 1); expander.digitalWrite(5, 1);
  delay(200);

  if (!gfx->begin()) while (1);
  gfx->setBrightness(255);
  gfx->fillScreen(COLOR_BG);

  fb = new GFXcanvas16(FB_W, FB_H);   // off-screen buffer (alokace předem)

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");

  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(32768);
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32_Dashboard", mqtt_user, mqtt_password)) {
      client.subscribe((String("device/") + serial_number + "/report").c_str());
      client.publish((String("device/") + serial_number + "/request").c_str(),
                     "{\"pushing\": {\"sequence_id\": \"1\", \"command\": \"pushall\"}}");
    } else delay(5000);
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) reconnect();
    client.loop();
  }
  updateUI();
  static unsigned long last_time_update = 0;
  if (millis() - last_time_update > 30000) { last_rem_min = -1; last_time_update = millis(); }
}
