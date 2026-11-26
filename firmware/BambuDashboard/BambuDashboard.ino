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
#include <esp_system.h>

// Výpisy vždy přes USB (HW CDC). Na ESP32-C6 je výchozí "USB CDC On Boot: Disabled"
// a Serial by pak šel na piny UART0 – v Serial monitoru přes USB by nebylo nic vidět.
#if ARDUINO_USB_CDC_ON_BOOT
  #define LOG Serial
#else
  HWCDC usbLog;
  #define LOG usbLog
#endif

// Typy tiskáren (pro P1_TYPE / P2_TYPE v secrets.h)
#define PRINTER_DUAL    0   // H2D – dvě trysky, vyhřívaná komora
#define PRINTER_SINGLE  1   // P2S, P1S, X1C… – jedna tryska, bez vyhřívané komory

#include "secrets.h"  //rename and fill secrets_template.h to secrets.h

// --- FONTY ---
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

// ---------------------------------------------------------
// NASTAVENÍ SÍTĚ A TISKÁREN
// ---------------------------------------------------------
// Zpětná kompatibilita se starým secrets.h (jedna tiskárna)
#ifndef P1_IP
  #define P1_IP          MY_MQTT_SERVER
  #define P1_ACCESS_CODE MY_MQTT_PASSWORD
  #define P1_SERIAL      SERIAL_NUMBER
#endif
#ifndef P1_NAME
  #define P1_NAME "H2D"
#endif
#ifndef P1_TYPE
  #define P1_TYPE PRINTER_DUAL
#endif
#if defined(P2_IP) && !defined(P2_NAME)
  #define P2_NAME "P2S"
#endif
#if defined(P2_IP) && !defined(P2_TYPE)
  #define P2_TYPE PRINTER_SINGLE
#endif

const char* ssid      = MY_SSID;
const char* password  = MY_PASSWORD;
const int   mqtt_port = 8883;
const char* mqtt_user = "bblp";

// ---------------------------------------------------------
// MAPOVÁNÍ TRYSEK / ČASOVAČE
// ---------------------------------------------------------
#define LEFT_NOZZLE_ID     1        // id LEVÉ trysky H2D (přehoď na 0, kdyby se prohodily)
#define ACTIVE_TARGET_MIN  40.0     // cíl nad touto hodnotou = aktivní tryska
#define FINISH_NOTIFY_MS   300000UL // obrazovka "Finished" / "Cancelled" 5 min, pak jen ✓ ve stavovém řádku
#define ROTATE_MS          10000UL  // přepínání mezi tiskárnami, když jich má víc co ukázat
#define ROTATE_ALERT_MS    20000UL  // chyba / pauza zůstává při střídání déle

// ---------------------------------------------------------
// HARDWARE
// ---------------------------------------------------------
Adafruit_XCA9554 expander;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_SH8601 *gfx = new Arduino_SH8601(bus, GFX_NOT_DEFINED, 0, LCD_WIDTH, LCD_HEIGHT);

// Off-screen buffer na celou šířku – pás dashboardu (až 130 px) se kreslí v lokálních
// souřadnicích a posílá na displej po částech výšky FB_H. ESP32-C6 nemá PSRAM, celý pás
// (368x130x2 = 95 KB) by s Wi-Fi + 2x TLS + 2x MQTT bufferem nevešel do RAM -> 368 x 65 (47 KB).
#define FB_W 368
#define FB_H 65
GFXcanvas16 *fb = nullptr;
int fb_oy = 0;   // svislý posun právě kreslené části pásu (0, FB_H, ...)
// Pozn.: žádné funkce před definicí struktur níže – Arduino IDE vkládá automatické
// prototypy před první funkci a typy Printer/Filament by tam ještě neexistovaly.

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
#define COLOR_DIM           0x39E7   // neaktivní tečka přepínače
#define COLOR_GREEN         0x07E0   // notifikace dokončeno
#define COLOR_ORANGE        0xFD20   // notifikace pauza
#define COLOR_RED           0xF800   // notifikace chyba

// ---------------------------------------------------------
// STAV TISKÁRNY
// ---------------------------------------------------------
struct Filament { String type = ""; uint16_t color = 0x0000; bool valid = false; };

struct Printer {
  // konfigurace
  const char* name = "";
  int type = PRINTER_SINGLE;
  const char* host = "";
  const char* code = "";
  const char* serial = "";

  // spojení
  WiFiClientSecure net;
  PubSubClient mqtt;
  unsigned long last_try = 0, last_ok = 0;
  int fails = 0;
  unsigned long last_msg = 0;          // millis() poslední přijaté zprávy (0 = zatím nic)
  String log_state = ""; int log_stg = -100, log_perc = -1;   // co bylo naposledy vypsáno

  // trysky: [0] = levá (u jednotryskové jediná), [1] = pravá
  float noz_t[2] = {0, 0}, noz_tgt[2] = {0, 0};
  int   tray[2] = {-1, -1};
  int   active_side = -1;

  float bed_temp = 0, bed_target = 0;
  float chamber_temp = 0, chamber_target = 0;
  bool  chamber_packed = false;
  int   air_mode = -1;                 // 0 = cooling, 1 = heating

  int mc_percent = 0, remaining_time_min = 0;
  int current_layer = 0, total_layers = 0;
  int print_error = 0;
  int stg_cur = -99;                   // aktuální fáze tisku (status message)
  String gcode_state = "IDLE";
  String job = "";                     // název tisku (subtask_name)

  Filament ams[16];
  Filament ext;

  // Přechody stavu – notifikace jen při skutečném konci tisku (ne po zapnutí displeje,
  // když tiskárna už hodiny hlásí FINISH / FAILED).
  unsigned long finish_at = 0;         // millis() přechodu tisk -> FINISH (0 = nebylo)
  unsigned long cancel_at = 0;         // millis() přechodu tisk -> FAILED bez chyby
  bool done = false;                   // dotištěno, čeká na odebrání (✓ ve stavovém řádku)

  Printer() : mqtt(net) {}
};

#ifdef P2_IP
  #define NUM_PRINTERS 2
#else
  #define NUM_PRINTERS 1
#endif
Printer printers[NUM_PRINTERS];

// --- REŽIM DISPLEJE ---
enum DispMode { M_DASH, M_GREEN, M_ORANGE, M_RED, M_GRAY, M_CLOCK };
DispMode last_mode = (DispMode)-1;
int last_shown = -2;
bool last_rotating = false;
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
// Stav tisku
bool isActiveState(const String &s) { return s == "RUNNING" || s == "PREPARE" || s == "SLICING"; }
bool isPrinting(const Printer &P)   { return isActiveState(P.gcode_state) || P.gcode_state == "PAUSE"; }
bool hasError(const Printer &P)     { return P.print_error != 0 && P.gcode_state != "OFFLINE"; }
bool isPaused(const Printer &P)     { return !hasError(P) && P.gcode_state == "PAUSE"; }

uint16_t hexToRGB565(String hex) {
  if (hex.length() < 6) return COLOR_GRAY;
  long rgb = strtol(hex.substring(0, 6).c_str(), NULL, 16);
  uint8_t r = (rgb >> 16) & 0xFF, g = (rgb >> 8) & 0xFF, b = rgb & 0xFF;
  if (r < 40 && g < 40 && b < 40) return COLOR_GRAY;
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Číslo z JSONu (číslo nebo řetězec); false, pokud klíč chybí / není číselný
static bool readNumber(JsonVariant v, double &out) {
  if (v.isNull()) return false;
  if (v.is<double>() || v.is<long>() || v.is<unsigned long>()) { out = v.as<double>(); return true; }
  if (v.is<const char*>()) {
    const char* s = v.as<const char*>();
    char* end;
    out = strtod(s, &end);
    return end != s;
  }
  return false;
}

static void unpackTemp(uint32_t packed, float &cur, float &tgt) {
  cur = (float)(packed & 0xFFFF);
  tgt = (float)((packed >> 16) & 0xFFFF);
  if (cur > 500) cur = 0;
  if (tgt > 500) tgt = 0;
}

// Prostá teplota; kdyby přišla zabalená ((target<<16)|current), vezme se jen current.
static bool readPlainTemp(JsonVariant v, float &out) {
  double d;
  if (!readNumber(v, d)) return false;
  if (d >= 65536.0) { float tgt; unpackTemp((uint32_t)d, out, tgt); }
  else out = (float)d;
  return true;
}

static int snowToTray(int snow) {
  if (snow < 0 || snow >= 65534) return -1;
  if (snow == 254 || snow == 255) return snow;
  return (snow / 256) * 4 + (snow % 256);
}

Filament filamentForTray(const Printer &P, int tray) {
  if (tray >= 0 && tray < 16) return P.ams[tray];
  if (tray == 254 || tray == 255) return P.ext;
  Filament empty; return empty;
}

static void readFilament(JsonObject t, Filament &f) {
  f.valid = true;
  if (t.containsKey("tray_sub_brands") && t["tray_sub_brands"].as<String>() != "")
    f.type = t["tray_sub_brands"].as<String>();
  else if (t.containsKey("tray_type"))
    f.type = t["tray_type"].as<String>();
  if (t.containsKey("tray_color"))
    f.color = hexToRGB565(t["tray_color"].as<String>());
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

// Vycentrovaný text do off-screen bufferu (by = souřadnice v pásu, posun fb_oy se odečte).
void fbCentered(String text, int cx, int by, const GFXfont *font, uint16_t color) {
  fb->setFont(font); fb->setTextSize(1); fb->setTextColor(color);
  int16_t x1, y1; uint16_t tw, th;
  fb->getTextBounds(text, 0, 0, &x1, &y1, &tw, &th);
  fb->setCursor(cx - (tw / 2), by - fb_oy); fb->print(text);
}

// Vykreslí jednu kartu do bufferu na pozici (cx, cy) v pásu.
void drawCardInto(int cx, int cy, int w, int h, String title, String val1, String val2,
                  uint16_t valColor, bool isActive, Filament f = Filament()) {
  uint16_t bg = isActive ? COLOR_CARD_ACTIVE : COLOR_CARD;
  int ly = cy - fb_oy;   // pro přímé kreslení do bufferu (fbCentered posun řeší sám)
  fb->fillRoundRect(cx, ly, w, h, 12, bg);
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
    fb->fillCircle(sx + 4, ly + h - 15, 5, f.color);
    fb->setTextColor(COLOR_TEXT);
    fb->setCursor(sx + 14, ly + h - 10); fb->print(s);
  }
}

// ---------------------------------------------------------
// IDENTIFIKACE TISKÁRNY (malý štítek dole, pod dashboardem)
// ---------------------------------------------------------
#define LABEL_Y  418   // pruh 418..448, pod spodním pásem dashboardu

// Stav spojení tiskárny: 0 = nepřipojeno, 1 = připojeno bez dat, 2 = chodí data.
// Filtrované časem, aby krátký výpadek / reconnect tečkou neblikal:
//  - červená až po STATUS_OFFLINE_MS bez spojení,
//  - zelená, dokud poslední zpráva není starší než STATUS_STALE_MS.
#define STATUS_OFFLINE_MS  30000UL
#define STATUS_STALE_MS    30000UL
int printerStatus(Printer &P) {
  unsigned long now = millis();
  if (!P.mqtt.connected() && (P.last_ok == 0 || now - P.last_ok > STATUS_OFFLINE_MS)) return 0;
  return (P.last_msg != 0 && now - P.last_msg < STATUS_STALE_MS) ? 2 : 1;
}

// Stav tisku pro stavový řádek: 0 = nic, 1 = chyba, 2 = pauza, 3 = dotištěno (čeká na odebrání)
int printerFlag(const Printer &P) {
  if (hasError(P)) return 1;
  if (isPaused(P)) return 2;
  if (P.done && P.gcode_state == "FINISH") return 3;
  return 0;
}

// Malá zelená "fajfka" (font nemá ✓), levý horní roh boxu 10x10 na (x, y).
void drawCheck(int x, int y, uint16_t c) {
  for (int t = 0; t < 2; t++) {               // tloušťka 2 px
    gfx->drawLine(x, y + 5 + t, x + 3, y + 8 + t, c);
    gfx->drawLine(x + 3, y + 8 + t, x + 9, y + 1 + t, c);
  }
}

// Stavový řádek: názvy všech tiskáren + tečka spojení (zelená = chodí data, oranžová =
// připojeno bez dat, červená = nepřipojeno). Barva názvu: červená = chyba, oranžová = pauza,
// jinak bílá pro zobrazenou tiskárnu (highlight) a šedá pro ostatní. ✓ = dotištěno.
// Překresluje se jen při změně; *last = -1 vynutí překreslení.
void drawStatusStrip(int y, int highlight, int *last) {
  int code = highlight + 1;
  for (int i = 0; i < NUM_PRINTERS; i++) code = code * 12 + printerStatus(printers[i]) * 4 + printerFlag(printers[i]);
  if (code == *last) return;
  *last = code;

  gfx->fillRect(0, y, 368, 30, COLOR_BG);
  gfx->setFont(&FreeSans9pt7b); gfx->setTextSize(1);
  int16_t x1, y1; uint16_t tw, th;
  int widths[NUM_PRINTERS], textw[NUM_PRINTERS], total = 0;
  for (int i = 0; i < NUM_PRINTERS; i++) {
    gfx->getTextBounds(printers[i].name, 0, 0, &x1, &y1, &tw, &th);
    textw[i] = tw;
    widths[i] = tw + 16 + (printerFlag(printers[i]) == 3 ? 14 : 0);   // text (+ ✓) + tečka
    total += widths[i] + (i ? 24 : 0);
  }
  int x = 184 - total / 2;
  for (int i = 0; i < NUM_PRINTERS; i++) {
    int s = printerStatus(printers[i]), fl = printerFlag(printers[i]);
    uint16_t dot = s == 0 ? COLOR_RED : (s == 1 ? COLOR_ORANGE : COLOR_GREEN);
    uint16_t tc  = fl == 1 ? COLOR_RED : fl == 2 ? COLOR_ORANGE : (i == highlight ? COLOR_TEXT : COLOR_GRAY);
    gfx->setTextColor(tc);
    gfx->setCursor(x, y + 18); gfx->print(printers[i].name);
    if (fl == 3) drawCheck(x + textw[i] + 5, y + 6, COLOR_GREEN);
    gfx->fillCircle(x + widths[i] - 5, y + 12, 4, dot);
    x += widths[i] + 24;
  }
}
int last_label_code = -1;

// ---------------------------------------------------------
// PÁSY DASHBOARDU (každý se pošle na displej jedním blokem)
// ---------------------------------------------------------
// Stejné rozložení pro obě: trysky (y 15) / podložka+komora (y 155) / progres (y 285).
// Jednotrysková má nahoře jednu kartu přes celou šířku.
#define DUAL_TOP_Y     15
#define DUAL_MID_Y     155
#define DUAL_BOT_Y     285

// Pošle aktuální část pásu (výška h, umístěný na displeji na y) na displej.
void fbFlush(int y, int h) {
  gfx->draw16bitRGBBitmap(0, y + fb_oy, fb->getBuffer(), FB_W, min(FB_H, h - fb_oy));
}

// Každý pás se kreslí v cyklu po částech výšky FB_H: for (fb_oy...) { kreslit; fbFlush(); }
void renderTopDual(const Printer &P, bool la, bool ra) {
  if (!fb) return;
  for (fb_oy = 0; fb_oy < 130; fb_oy += FB_H) {
    fb->fillScreen(COLOR_BG);
    drawCardInto(10, 0, 169, 130, "Left Nozzle",
                 String(P.noz_t[0], 0) + " C", "Tgt: " + String(P.noz_tgt[0], 0),
                 COLOR_NOZZLE, la, filamentForTray(P, P.tray[0]));
    drawCardInto(189, 0, 169, 130, "Right Nozzle",
                 String(P.noz_t[1], 0) + " C", "Tgt: " + String(P.noz_tgt[1], 0),
                 COLOR_NOZZLE, ra, filamentForTray(P, P.tray[1]));
    fbFlush(DUAL_TOP_Y, 130);
  }
  fb_oy = 0;
}

// Podložka + komora. H2D: barva komory podle režimu (topí = červená, chladí = modrá) a target.
// Jednotrysková (P2S…) komoru nevyhřívá: vždy modře a bez targetu.
void renderMid(const Printer &P) {
  if (!fb) return;
  bool dual = (P.type == PRINTER_DUAL);
  uint16_t cc = COLOR_BED;
  String ctgt = "";
  if (dual) {
    cc = COLOR_TEXT;
    if (P.air_mode == 1)      cc = COLOR_NOZZLE;  // heating -> červená
    else if (P.air_mode == 0) cc = COLOR_BED;     // cooling -> modrá
    ctgt = "Tgt: " + String(P.chamber_target, 0);
  }
  for (fb_oy = 0; fb_oy < 110; fb_oy += FB_H) {
    fb->fillScreen(COLOR_BG);
    drawCardInto(10, 0, 169, 110, "Heatbed",
                 String(P.bed_temp, 0) + " C", "Tgt: " + String(P.bed_target, 0), COLOR_BED, false);
    drawCardInto(189, 0, 169, 110, "Chamber",
                 String(P.chamber_temp, 0) + " C", ctgt, cc, false);
    fbFlush(DUAL_MID_Y, 110);
  }
  fb_oy = 0;
}

// Jedna tryska: karta přes celou šířku (tryska, target, filament).
void renderTopSingle(const Printer &P) {
  if (!fb) return;
  for (fb_oy = 0; fb_oy < 130; fb_oy += FB_H) {
    fb->fillScreen(COLOR_BG);
    drawCardInto(10, 0, 348, 130, "Nozzle",
                 String(P.noz_t[0], 0) + " C", "Tgt: " + String(P.noz_tgt[0], 0),
                 COLOR_NOZZLE, P.noz_tgt[0] >= ACTIVE_TARGET_MIN, filamentForTray(P, P.tray[0]));
    fbFlush(DUAL_TOP_Y, 130);
  }
  fb_oy = 0;
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

void renderBottom(const Printer &P, int y) {
  if (!fb) return;
  String eta = "End: " + getETAStr(P.remaining_time_min);
  String rem = "Rem: " + formatTimeStr(P.remaining_time_min);
  int barWidth = map(P.mc_percent, 0, 100, 0, 314);
  // Status fáze tisku pod progress barem
  String st = stageText(P.stg_cur);
  if (st == "" && P.gcode_state == "RUNNING") st = "Printing";

  for (fb_oy = 0; fb_oy < 130; fb_oy += FB_H) {
    fb->fillScreen(COLOR_BG);
    fbCentered(eta, 70, 14, &FreeSans9pt7b, COLOR_GRAY);
    fbCentered(rem, 298, 14, &FreeSans9pt7b, COLOR_GRAY);
    fb->fillRoundRect(27, 28 - fb_oy, 314, 22, 6, COLOR_CARD);
    if (barWidth > 0) fb->fillRoundRect(27, 28 - fb_oy, barWidth, 22, 6, COLOR_BAMBU);
    fbCentered(String(P.mc_percent) + " %", 100, 90, &FreeSansBold24pt7b, COLOR_TEXT);
    fbCentered("Layer " + String(P.current_layer) + " / " + String(P.total_layers), 260, 82, &FreeSans12pt7b, COLOR_GRAY);
    if (st != "") fbCentered(st, 184, 120, &FreeSans12pt7b, COLOR_SUBTEXT);
    fbFlush(y, 130);
  }
  fb_oy = 0;
}

void drawDashboard(int idx, bool rotating) {
  const Printer &P = printers[idx];
  bool dual = (P.type == PRINTER_DUAL);

  if (force_redraw) {
    gfx->fillScreen(COLOR_BG); force_redraw = false;
    last_l_t = last_l_tgt = last_r_t = last_r_tgt = -1;
    last_l_tray = last_r_tray = -99; last_l_active = last_r_active = -1;
    last_bed_t = last_bed_tgt = -1; last_cham_t = last_cham_tgt = -1; last_air_mode = -99;
    last_perc = last_rem_min = -1; last_cur_layer = last_tot_layer = -1; last_stg = -100;
    last_label_code = -1;
  }
  if (NUM_PRINTERS > 1) drawStatusStrip(LABEL_Y, idx, &last_label_code);

  if (dual) {
    bool la = (P.active_side == 0), ra = (P.active_side == 1);
    if (last_l_t != P.noz_t[0] || last_l_tgt != P.noz_tgt[0] || last_l_tray != P.tray[0] || last_l_active != (int)la ||
        last_r_t != P.noz_t[1] || last_r_tgt != P.noz_tgt[1] || last_r_tray != P.tray[1] || last_r_active != (int)ra) {
      renderTopDual(P, la, ra);
      last_l_t = P.noz_t[0]; last_l_tgt = P.noz_tgt[0]; last_l_tray = P.tray[0]; last_l_active = (int)la;
      last_r_t = P.noz_t[1]; last_r_tgt = P.noz_tgt[1]; last_r_tray = P.tray[1]; last_r_active = (int)ra;
    }
  } else {
    if (last_l_t != P.noz_t[0] || last_l_tgt != P.noz_tgt[0] || last_l_tray != P.tray[0]) {
      renderTopSingle(P);
      last_l_t = P.noz_t[0]; last_l_tgt = P.noz_tgt[0]; last_l_tray = P.tray[0];
    }
  }

  if (last_bed_t != P.bed_temp || last_bed_tgt != P.bed_target ||
      last_cham_t != P.chamber_temp || last_cham_tgt != P.chamber_target || last_air_mode != P.air_mode) {
    renderMid(P);
    last_bed_t = P.bed_temp; last_bed_tgt = P.bed_target;
    last_cham_t = P.chamber_temp; last_cham_tgt = P.chamber_target; last_air_mode = P.air_mode;
  }

  if (last_perc != P.mc_percent || last_rem_min != P.remaining_time_min ||
      last_cur_layer != P.current_layer || last_tot_layer != P.total_layers || last_stg != P.stg_cur) {
    renderBottom(P, DUAL_BOT_Y);
    last_perc = P.mc_percent; last_rem_min = P.remaining_time_min;
    last_cur_layer = P.current_layer; last_tot_layer = P.total_layers; last_stg = P.stg_cur;
  }
}

// ---------------------------------------------------------
// NOTIFIKACE A HODINY (přes celý displej)
// ---------------------------------------------------------
void drawNotification(int idx, bool rotating, uint16_t bg, uint16_t fg, const char* title, String sub) {
  gfx->fillScreen(bg);
  printCentered(printers[idx].name, 184, 150, &FreeSansBold18pt7b, fg);   // na které tiskárně
  gfx->setFont(&FreeSansBold24pt7b); gfx->setTextSize(1); gfx->setTextColor(fg);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(184 - tw / 2, 225); gfx->print(title);
  if (sub != "") printCentered(sub, 184, 275, &FreeSans12pt7b, fg);
  if (rotating) {   // které z střídaných obrazovek: plná tečka = tato tiskárna
    for (int i = 0; i < NUM_PRINTERS; i++) {
      gfx->fillCircle(184 - (NUM_PRINTERS - 1) * 7 + i * 14, 320, 4, fg);
      if (i != idx) gfx->fillCircle(184 - (NUM_PRINTERS - 1) * 7 + i * 14, 320, 2, bg);
    }
  }
}

// Ochrana OLED proti vypálení: blok hodin se každých CLOCK_SHIFT_MS přesune
// na náhodnou svislou pozici. Blok = čas (baseline CLOCK_TIME_BASE + offset)
// a datum (baseline CLOCK_DATE_BASE + offset), přibližně y 180..300 při offsetu 0.
#define CLOCK_SHIFT_MS   (5UL * 60UL * 1000UL)
#define CLOCK_TIME_BASE  250
#define CLOCK_DATE_BASE  290
#define CLOCK_BLOCK_TOP  175   // horní okraj velkých číslic při offsetu 0 (s rezervou)
#define CLOCK_BLOCK_BOT  300   // spodní okraj data při offsetu 0 (s rezervou)
#define CLOCK_MARGIN     10
int clock_offset = 0;
unsigned long clock_shift_at = 0;
int last_status_code = -1;   // -1 = vynutit překreslení stavového řádku

// Stav tiskáren u hodin: řádek je vždy na opačné straně než blok hodin
// (hodiny dole -> řádek nahoře a naopak), aby se nepřekrývaly.
void drawClockStatus() {
  drawStatusStrip((clock_offset > 0) ? 4 : LABEL_Y, -1, &last_status_code);
}

void drawClockUI() {
  time_t now; time(&now); struct tm *ti = localtime(&now);
  if (ti->tm_year < 100) {               // NTP ještě nedoběhlo
    if (last_clock_min != -2) {
      printCentered("Syncing time...", 184, CLOCK_TIME_BASE + clock_offset, &FreeSans12pt7b, COLOR_GRAY);
      last_clock_min = -2;
    }
    drawClockStatus();
    return;
  }

  if (millis() - clock_shift_at >= CLOCK_SHIFT_MS) {
    const int minOff = CLOCK_MARGIN - CLOCK_BLOCK_TOP;
    const int maxOff = (LCD_HEIGHT - CLOCK_MARGIN) - CLOCK_BLOCK_BOT;
    int n = clock_offset;
    for (int i = 0; i < 10 && abs(n - clock_offset) < 40; i++) n = random(minOff, maxOff + 1);
    clock_offset = n;
    clock_shift_at = millis();
    gfx->fillScreen(COLOR_BG);
    last_clock_min = -1;
    last_status_code = -1;
  }

  if (last_clock_min != ti->tm_min) {
    gfx->fillRect(0, CLOCK_BLOCK_TOP + clock_offset, 368, CLOCK_BLOCK_BOT - CLOCK_BLOCK_TOP, COLOR_BG);
    char buf[10]; sprintf(buf, "%02d:%02d", ti->tm_hour, ti->tm_min);
    gfx->setFont(&FreeSansBold24pt7b); gfx->setTextSize(2); gfx->setTextColor(COLOR_TEXT);
    int16_t x1, y1; uint16_t tw, th; gfx->getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
    gfx->setCursor(184 - (tw / 2), CLOCK_TIME_BASE + clock_offset); gfx->print(buf);
    char dateBuf[20]; sprintf(dateBuf, "%02d.%02d.%04d", ti->tm_mday, ti->tm_mon + 1, ti->tm_year + 1900);
    printCentered(dateBuf, 184, CLOCK_DATE_BASE + clock_offset, &FreeSans12pt7b, COLOR_GRAY);
    last_clock_min = ti->tm_min;
  }
  drawClockStatus();
}

// ---------------------------------------------------------
// VÝBĚR REŽIMU
// ---------------------------------------------------------
// Priorita: chyba > pauza > tisk > dokončeno (5 min) > zrušeno (5 min) > nic (hodiny).
// Offline tiskárna nikdy obrazovku nezabere (jen červená tečka ve stavovém řádku).
DispMode computeMode(Printer &P) {
  unsigned long now = millis();
  if (hasError(P))                    return M_RED;
  if (isPaused(P))                    return M_ORANGE;
  if (isActiveState(P.gcode_state))   return M_DASH;
  if (P.gcode_state == "FINISH" && P.finish_at && now - P.finish_at < FINISH_NOTIFY_MS) return M_GREEN;
  if (P.gcode_state == "FAILED" && P.cancel_at && now - P.cancel_at < FINISH_NOTIFY_MS) return M_GRAY;
  return M_CLOCK;
}

// Kód chyby Bambu se zobrazuje jako XXXX-XXXX (hex), stejně jako na tiskárně / v aplikaci.
String errorCodeStr(int e) {
  char buf[16];
  sprintf(buf, "%04X-%04X", ((uint32_t)e >> 16) & 0xFFFF, (uint32_t)e & 0xFFFF);
  return String(buf);
}

void updateUI() {
  // Které tiskárny mají co ukázat (tisk / notifikace)?
  DispMode modes[NUM_PRINTERS];
  int activeCount = 0, firstActive = -1;
  for (int i = 0; i < NUM_PRINTERS; i++) {
    modes[i] = computeMode(printers[i]);
    if (modes[i] != M_CLOCK) { activeCount++; if (firstActive < 0) firstActive = i; }
  }

  static int shown = -1;
  static unsigned long rotate_at = 0;
  if (activeCount == 0) {
    shown = -1;
  } else if (activeCount == 1) {
    shown = firstActive;
  } else if (shown < 0 || modes[shown] == M_CLOCK) {
    shown = firstActive; rotate_at = millis();
  } else {
    // víc tiskáren má co ukázat -> střídat; chyba / pauza zůstává déle
    unsigned long dwell = (modes[shown] == M_RED || modes[shown] == M_ORANGE) ? ROTATE_ALERT_MS : ROTATE_MS;
    if (millis() - rotate_at >= dwell) {
      do { shown = (shown + 1) % NUM_PRINTERS; } while (modes[shown] == M_CLOCK);
      rotate_at = millis();
    }
  }
  bool rotating = (activeCount > 1);
  DispMode mode = (shown < 0) ? M_CLOCK : modes[shown];

  // vstup do nového režimu / jiné tiskárny / změna kódu chyby -> jednorázové překreslení
  static int last_err_shown = 0;
  int err_now = (shown >= 0) ? printers[shown].print_error : 0;
  if (mode != last_mode || shown != last_shown || rotating != last_rotating ||
      (mode == M_RED && err_now != last_err_shown)) {
    last_mode = mode; last_shown = shown; last_rotating = rotating; last_err_shown = err_now;
    last_clock_min = -1;
    if (mode == M_DASH) {
      force_redraw = true;
    } else if (mode == M_GREEN) {
      String job = printers[shown].job;
      if (job.length() > 24) job = job.substring(0, 22) + "..";
      drawNotification(shown, rotating, COLOR_GREEN, 0x0000, "FINISHED", job != "" ? job : String("Print complete"));
    } else if (mode == M_ORANGE) {
      drawNotification(shown, rotating, COLOR_ORANGE, 0x0000, "PAUSED", "Print paused");
    } else if (mode == M_RED) {
      drawNotification(shown, rotating, COLOR_RED, 0xFFFF, "ERROR", "Code " + errorCodeStr(err_now));
    } else if (mode == M_GRAY) {
      drawNotification(shown, rotating, COLOR_CARD_ACTIVE, COLOR_TEXT, "CANCELLED", "Print stopped");
    } else if (mode == M_CLOCK) {
      gfx->fillScreen(COLOR_BG); last_status_code = -1;
    }
  }

  if (mode == M_DASH)  drawDashboard(shown, rotating);
  else if (mode == M_CLOCK) drawClockUI();
  // notifikační režimy: obrazovka zůstává až do změny stavu, nic dalšího se nekreslí
}

// ---------------------------------------------------------
// MQTT – ZPRACOVÁNÍ REPORTU
// ---------------------------------------------------------
void parseNozzlesDual(Printer &P, JsonObject p) {
  // print.device.extruder.info[] (temp = (target<<16)|current)
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

      int s = isLeft ? 0 : 1;
      P.noz_t[s] = cur; P.noz_tgt[s] = tgt; P.tray[s] = tray;
      dual_parsed = true;
    }
    if (P.noz_tgt[0] >= ACTIVE_TARGET_MIN || P.noz_tgt[1] >= ACTIVE_TARGET_MIN) {
      if (P.noz_tgt[0] > P.noz_tgt[1])      P.active_side = 0;
      else if (P.noz_tgt[1] > P.noz_tgt[0]) P.active_side = 1;
    } else {
      P.active_side = -1;
    }
  }

  if (!dual_parsed) {  // záloha pro jednotryskový formát
    int s = (P.active_side == 1) ? 1 : 0;
    readPlainTemp(p["nozzle_temper"], P.noz_t[s]);
    readPlainTemp(p["nozzle_target_temper"], P.noz_tgt[s]);
  }
}

void parseNozzleSingle(Printer &P, JsonObject p) {
  // Novější firmware posílá i u jednotryskových print.device.extruder.info[] (jedna položka)
  bool parsed = false;
  if (p["device"].is<JsonObject>() && p["device"]["extruder"]["info"].is<JsonArray>()) {
    for (JsonObject e : p["device"]["extruder"]["info"].as<JsonArray>()) {
      if (e.containsKey("id") && e["id"].as<int>() != 0) continue;
      if (e.containsKey("temp")) unpackTemp(e["temp"].as<uint32_t>(), P.noz_t[0], P.noz_tgt[0]);
      if (e.containsKey("snow")) P.tray[0] = snowToTray(e["snow"].as<int>());
      parsed = true;
      break;
    }
  }
  if (!parsed) {
    readPlainTemp(p["nozzle_temper"], P.noz_t[0]);
    readPlainTemp(p["nozzle_target_temper"], P.noz_tgt[0]);
  }
  // aktuálně zavedený filament: ams.tray_now ("0".."15", 254 = externí cívka, 255 = nic)
  double tn;
  if (readNumber(p["ams"]["tray_now"], tn)) {
    int t = (int)tn;
    P.tray[0] = (t == 255) ? -1 : t;
  }
}

void parseReport(Printer &P, JsonObject p) {
  if (P.type == PRINTER_DUAL) parseNozzlesDual(P, p);
  else                        parseNozzleSingle(P, p);

  // --- PODLOŽKA ---
  readPlainTemp(p["bed_temper"], P.bed_temp);
  readPlainTemp(p["bed_target_temper"], P.bed_target);

  // --- KOMORA (H2D i jednotryskové – P2S má čidlo, jen nevyhřívá) ---
  // print.device.ctc.info.temp i chamber_temper mohou být zabalené jako u trysek
  // ((target<<16)|current). Bez vyhřívání je target 0 a hodnota vypadá jako prostá teplota,
  // proto se rozbaluje vše, co je >= 65536. Jakmile přijde zabalená hodnota, target se bere
  // jen z ní (chamber_target_temper pak tiskárna posílá jako 0).
  {
    double v;
    if (readNumber(p["chamber_temper"], v)) {
      if (v >= 65536.0) { unpackTemp((uint32_t)v, P.chamber_temp, P.chamber_target); P.chamber_packed = true; }
      else P.chamber_temp = (float)v;
    }
    if (readNumber(p["device"]["ctc"]["info"]["temp"], v)) {
      if (v >= 65536.0) { unpackTemp((uint32_t)v, P.chamber_temp, P.chamber_target); P.chamber_packed = true; }
      else { P.chamber_temp = (float)v; if (P.chamber_packed) P.chamber_target = 0; }
    }
    if (!P.chamber_packed && readNumber(p["chamber_target_temper"], v)) {
      if (v >= 65536.0) { float dummy; unpackTemp((uint32_t)v, dummy, P.chamber_target); }
      else P.chamber_target = (float)v;
    }
    if (P.type == PRINTER_DUAL && p["device"]["airduct"]["modeCur"].is<int>()) P.air_mode = p["device"]["airduct"]["modeCur"].as<int>();
  }

  // --- PROGRES / STAV ---
  if (p.containsKey("mc_percent"))        P.mc_percent = p["mc_percent"].as<int>();
  if (p.containsKey("mc_remaining_time")) P.remaining_time_min = p["mc_remaining_time"].as<int>();
  if (p.containsKey("layer_num"))         P.current_layer = p["layer_num"].as<int>();
  if (p.containsKey("total_layer_num"))   P.total_layers = p["total_layer_num"].as<int>();
  if (p.containsKey("gcode_state")) {
    String ns = p["gcode_state"].as<String>();
    bool wasPrinting = isActiveState(P.gcode_state) || P.gcode_state == "PAUSE";
    P.gcode_state = ns;
    if (isPrinting(P)) {                        // nový / pokračující tisk ruší dokončení
      P.done = false; P.finish_at = 0; P.cancel_at = 0;
    } else if (wasPrinting && ns == "FINISH") {
      P.finish_at = millis(); P.done = true;
    } else if (wasPrinting && ns == "FAILED") {
      P.cancel_at = millis();                   // zrušeno / selhalo (chybu řeší print_error)
    }
  }
  if (p["subtask_name"].is<const char*>())  P.job = p["subtask_name"].as<String>();
  if (p.containsKey("print_error"))       P.print_error = p["print_error"].as<int>();
  if (p.containsKey("stg_cur"))           P.stg_cur = p["stg_cur"].as<int>();

  // --- AMS: tabulka filamentů ---
  if (p["ams"]["ams"].is<JsonArray>()) {
    for (JsonObject a : p["ams"]["ams"].as<JsonArray>()) {
      int ams_id = a["id"].as<int>();
      for (JsonObject t : a["tray"].as<JsonArray>()) {
        int abs_id = ams_id * 4 + t["id"].as<int>();
        if (abs_id >= 0 && abs_id < 16) readFilament(t, P.ams[abs_id]);
      }
    }
  }
  // --- externí cívka ---
  if (p["vt_tray"].is<JsonObject>()) readFilament(p["vt_tray"].as<JsonObject>(), P.ext);
}

// Velké JSON dokumenty (H2D pushall má desítky kB) dávat do PSRAM, je-li k dispozici,
// ať nedojde vnitřní RAM (framebuffer + 2x TLS + MQTT buffery).
struct SpiRamAllocator : ArduinoJson::Allocator {
  void* allocate(size_t n) override { return psramFound() ? ps_malloc(n) : malloc(n); }
  void deallocate(void* p) override { free(p); }
  void* reallocate(void* p, size_t n) override { return psramFound() ? ps_realloc(p, n) : realloc(p, n); }
};
SpiRamAllocator spiAlloc;

// Filtr: z reportu se do paměti načtou jen klíče, které parseReport() používá.
// Pushall H2D má desítky kB (hms, net, ipcam, upgrade_state…) – bez filtru by se JSON
// na ESP32-C6 (bez PSRAM) vedle 2x TLS nemusel vejít.
JsonDocument json_filter;
void buildJsonFilter() {
  JsonObject p = json_filter["print"].to<JsonObject>();
  const char* keys[] = { "nozzle_temper", "nozzle_target_temper", "bed_temper", "bed_target_temper",
                         "chamber_temper", "chamber_target_temper", "mc_percent", "mc_remaining_time",
                         "layer_num", "total_layer_num", "gcode_state", "print_error", "stg_cur", "subtask_name" };
  for (const char* k : keys) p[k] = true;

  JsonObject ext = p["device"]["extruder"]["info"][0].to<JsonObject>();
  ext["id"] = true; ext["temp"] = true; ext["snow"] = true;
  p["device"]["ctc"]["info"]["temp"] = true;
  p["device"]["airduct"]["modeCur"] = true;

  p["ams"]["tray_now"] = true;
  JsonObject ams = p["ams"]["ams"][0].to<JsonObject>();
  ams["id"] = true;
  const char* trayKeys[] = { "id", "tray_sub_brands", "tray_type", "tray_color" };
  JsonObject tray = ams["tray"][0].to<JsonObject>();
  JsonObject vt = p["vt_tray"].to<JsonObject>();
  for (const char* k : trayKeys) { tray[k] = true; vt[k] = true; }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // topic = device/<serial>/report -> podle sériového čísla najít tiskárnu
  Printer *P = nullptr;
  for (int i = 0; i < NUM_PRINTERS; i++)
    if (strstr(topic, printers[i].serial)) { P = &printers[i]; break; }
  if (!P) return;

  JsonDocument doc(&spiAlloc);
  DeserializationError err = deserializeJson(doc, payload, length, DeserializationOption::Filter(json_filter));
  if (err) {
    LOG.printf("[%s] JSON error: %s (%u B, heap %u, max blok %u)\n", P->name, err.c_str(), length,
                  ESP.getFreeHeap(), ESP.getMaxAllocHeap());
    return;
  }
  P->last_msg = millis();
  if (!doc.containsKey("print")) return;
  parseReport(*P, doc["print"].as<JsonObject>());
  // vypisovat jen změny (stav / fáze / procenta), ne každý report
  if (P->gcode_state != P->log_state || P->stg_cur != P->log_stg || P->mc_percent != P->log_perc) {
    P->log_state = P->gcode_state; P->log_stg = P->stg_cur; P->log_perc = P->mc_percent;
    LOG.printf("[%s] report %u B: state %s, stg %d, %d %%, noz %.0f/%.0f, bed %.0f/%.0f, cham %.0f/%.0f, tray %d\n",
               P->name, length, P->gcode_state.c_str(), P->stg_cur, P->mc_percent,
               P->noz_t[0], P->noz_tgt[0], P->bed_temp, P->bed_target, P->chamber_temp, P->chamber_target, P->tray[0]);
  }
}

// ---------------------------------------------------------
// SETUP / LOOP
// ---------------------------------------------------------
void setupPrinter(Printer &P, const char* name, int type, const char* host, const char* code, const char* serial) {
  P.name = name; P.type = type; P.host = host; P.code = code; P.serial = serial;
  P.net.setInsecure();
  P.net.setHandshakeTimeout(5);   // s
  P.mqtt.setSocketTimeout(5);     // s
  P.mqtt.setServer(host, mqtt_port);
  P.mqtt.setCallback(callback);
  // Zprávu větší než buffer PubSubClient POTICHU zahodí. Pushall H2D i P2S má desítky kB,
  // proto 32 KB pro obě (RAM ušetřená menším framebufferem na to stačí).
  int buf = 32768;
  if (!P.mqtt.setBufferSize(buf)) LOG.printf("[%s] MQTT buffer %d B: malloc FAILED\n", name, buf);
}

// Stavové řádky při startu (na displej i do Serialu), ať černá obrazovka něco říká.
int boot_line = 0;
void bootMsg(String msg, uint16_t color = COLOR_GRAY) {
  LOG.println(msg);
  printCentered(msg, 184, 60 + boot_line * 26, &FreeSans9pt7b, color);
  boot_line++;
}

const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "CRASH (panic)";
    case ESP_RST_INT_WDT:  return "CRASH (int WDT)";
    case ESP_RST_TASK_WDT: return "CRASH (task WDT)";
    case ESP_RST_WDT:      return "CRASH (WDT)";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_DEEPSLEEP:return "deep sleep";
    default:               return "other";
  }
}

void setup() {
  LOG.begin(115200);
  // Bez čtoucího programu na PC by každý zápis čekal až 20x timeout (výchozí 100 ms) a
  // brzdil smyčku tak, že padalo MQTT. S 0 se výpis prostě zahodí, když ho nikdo nečte.
  LOG.setTxTimeoutMs(0);
  delay(300);
  LOG.println("\n=== Bambu dashboard boot ===");
  Wire.begin(IIC_SDA, IIC_SCL);
  if (!expander.begin(0x20)) while (1);
  expander.pinMode(4, OUTPUT); expander.pinMode(5, OUTPUT);
  expander.digitalWrite(4, 1); expander.digitalWrite(5, 1);
  delay(200);

  if (!gfx->begin()) while (1);
  gfx->setBrightness(255);
  gfx->fillScreen(COLOR_BG);

  esp_reset_reason_t rr = esp_reset_reason();
  bootMsg(String("Reset: ") + resetReasonStr(rr),
          (rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT ||
           rr == ESP_RST_WDT || rr == ESP_RST_BROWNOUT) ? COLOR_RED : COLOR_GRAY);
  bootMsg(String("PSRAM: ") + (psramFound() ? String((int)(ESP.getPsramSize() / 1024)) + " kB" : String("NONE")),
          psramFound() ? COLOR_GRAY : COLOR_ORANGE);

  fb = new GFXcanvas16(FB_W, FB_H);   // off-screen buffer (alokace předem)
  if (!fb || !fb->getBuffer()) { bootMsg("Framebuffer alloc FAILED", COLOR_RED); delete fb; fb = nullptr; }
  buildJsonFilter();
  bootMsg(String("Free heap: ") + String((int)(ESP.getFreeHeap() / 1024)) + " kB");

  bootMsg(String("WiFi: ") + ssid + " ...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long wifi_start = millis();
  bool wifi_warned = false;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (!wifi_warned && millis() - wifi_start > 20000) {
      bootMsg("WiFi not connecting - check SSID/password", COLOR_ORANGE);
      wifi_warned = true;
    }
  }
  bootMsg(String("WiFi OK, IP ") + WiFi.localIP().toString(), COLOR_GREEN);

  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");

  setupPrinter(printers[0], P1_NAME, P1_TYPE, P1_IP, P1_ACCESS_CODE, P1_SERIAL);
#ifdef P2_IP
  setupPrinter(printers[1], P2_NAME, P2_TYPE, P2_IP, P2_ACCESS_CODE, P2_SERIAL);
#endif
  for (int i = 0; i < NUM_PRINTERS; i++)
    bootMsg(String(printers[i].name) + ": " + printers[i].host);
  bootMsg(String("Free heap: ") + String((int)(ESP.getFreeHeap() / 1024)) + " kB, max blok " +
          String((int)(ESP.getMaxAllocHeap() / 1024)) + " kB");
  delay(1500);   // ať jde start přečíst; pak hodiny / dashboard
}

// Neblokující pokus o připojení. Interval se při opakovaných neúspěších prodlužuje
// (10 s, 20 s, 40 s, max 60 s), aby vypnutá tiskárna co nejméně brzdila tu druhou.
#define MQTT_RETRY_MS     10000UL
#define MQTT_RETRY_MAX_MS 60000UL
#define OFFLINE_AFTER_MS  60000UL  // tak dlouho bez spojení -> data považujeme za neplatná
#define TCP_CONNECT_MS    5000     // výchozí timeout NetworkClientSecure je 30 s
#define TLS_MIN_HEAP      45000UL  // méně volné RAM -> TLS handshake vůbec nezkoušet (hrozí pád)

void reconnect(Printer &P, int idx) {
  unsigned long wait = min(MQTT_RETRY_MS << min(P.fails, 3), MQTT_RETRY_MAX_MS);
  if (P.last_try != 0 && millis() - P.last_try < wait) return;
  P.last_try = millis();
  String clientId = String("ESP32_Dashboard_") + idx;
  LOG.printf("[%s] MQTT connect %s ... (heap %u, max blok %u)\n", P.name, P.host,
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  if (ESP.getFreeHeap() < TLS_MIN_HEAP) {
    P.fails++;
    LOG.printf("[%s] malo pameti pro TLS, pokus vynechan\n", P.name);
    return;
  }
  // TCP+TLS spojení napřed s krátkým timeoutem; PubSubClient pak existující spojení použije.
  if (!P.net.connected() && !P.net.connect(P.host, mqtt_port, TCP_CONNECT_MS)) {
    P.fails++;
    P.net.stop();
    LOG.printf("[%s] TCP/TLS connect failed\n", P.name);
    P.last_try = millis();
    return;
  }
  if (P.mqtt.connect(clientId.c_str(), mqtt_user, P.code)) {
    LOG.printf("[%s] MQTT connected\n", P.name);
    P.fails = 0;
    P.mqtt.subscribe((String("device/") + P.serial + "/report").c_str());
    P.mqtt.publish((String("device/") + P.serial + "/request").c_str(),
                   "{\"pushing\": {\"sequence_id\": \"1\", \"command\": \"pushall\"}}");
  } else {
    P.fails++;
    LOG.printf("[%s] MQTT failed, state %d\n", P.name, P.mqtt.state());
  }
  P.last_try = millis();   // interval počítat od konce pokusu (connect může chvíli blokovat)
}

void loop() {
  for (int i = 0; i < NUM_PRINTERS; i++) {
    Printer &P = printers[i];
    if (WiFi.status() == WL_CONNECTED) {
      if (!P.mqtt.connected()) reconnect(P, i);
      if (P.mqtt.connected()) P.mqtt.loop();
    }

    if (P.mqtt.connected()) {
      P.last_ok = millis();
    } else if (millis() - P.last_ok > OFFLINE_AFTER_MS && P.gcode_state != "OFFLINE") {
      // tiskárna nedostupná: zahodit starý stav, aby nezůstal viset dashboard/notifikace
      P.gcode_state = "OFFLINE"; P.print_error = 0;
      P.done = false; P.finish_at = 0; P.cancel_at = 0;
    }
    updateUI();   // i mezi tiskárnami, ať blokující connect jedné nezdržuje displej
  }

  static unsigned long last_time_update = 0;
  if (millis() - last_time_update > 30000) { last_rem_min = -1; last_time_update = millis(); }
}
