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
#define FINISH_NOTIFY_MS   300000UL // notifikace "Finished" zobrazená 5 min, pak hodiny
#define ROTATE_MS          10000UL  // přepínání mezi tiskárnami, když tisknou obě

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

  Filament ams[16];
  Filament ext;

  bool finish_latch = false;
  unsigned long finish_at = 0;

  Printer() : mqtt(net) {}
};

#ifdef P2_IP
  #define NUM_PRINTERS 2
#else
  #define NUM_PRINTERS 1
#endif
Printer printers[NUM_PRINTERS];

// --- REŽIM DISPLEJE ---
enum DispMode { M_DASH, M_GREEN, M_ORANGE, M_RED, M_CLOCK };
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
// IDENTIFIKACE TISKÁRNY (malý štítek dole, pod dashboardem)
// ---------------------------------------------------------
#define LABEL_Y  418   // pruh 418..448, pod spodním pásem dashboardu
void drawPrinterLabel(int idx, bool rotating) {
  gfx->fillRect(0, LABEL_Y, 368, LCD_HEIGHT - LABEL_Y, COLOR_BG);
  if (NUM_PRINTERS < 2) return;   // s jednou tiskárnou štítek nemá smysl
  const char* name = printers[idx].name;
  gfx->setFont(&FreeSans9pt7b); gfx->setTextSize(1);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(name, 0, 0, &x1, &y1, &tw, &th);
  int dotsW = rotating ? 24 : 0;              // dvě tečky: která z obou tiskáren je vidět
  int sx = 184 - (int)(tw + dotsW) / 2;
  gfx->setTextColor(COLOR_GRAY);
  gfx->setCursor(sx, LABEL_Y + 18); gfx->print(name);
  if (rotating) {
    int dx = sx + tw + 10;
    for (int i = 0; i < NUM_PRINTERS; i++)
      gfx->fillCircle(dx + i * 10, LABEL_Y + 12, 3, i == idx ? COLOR_TEXT : COLOR_DIM);
  }
}

// ---------------------------------------------------------
// PÁSY DASHBOARDU (každý se pošle na displej jedním blokem)
// ---------------------------------------------------------
// H2D: trysky (y 15) / podložka+komora (y 155) / progres (y 285)
// jednotrysková: tryska+podložka (y 60) / progres (y 220)
#define DUAL_TOP_Y     15
#define DUAL_MID_Y     155
#define DUAL_BOT_Y     285
#define SINGLE_TOP_Y   60
#define SINGLE_BOT_Y   220

void renderTopDual(const Printer &P, bool la, bool ra) {
  if (!fb) return;
  fb->fillScreen(COLOR_BG);
  drawCardInto(10, 0, 169, 130, "Left Nozzle",
               String(P.noz_t[0], 0) + " C", "Tgt: " + String(P.noz_tgt[0], 0),
               COLOR_NOZZLE, la, filamentForTray(P, P.tray[0]));
  drawCardInto(189, 0, 169, 130, "Right Nozzle",
               String(P.noz_t[1], 0) + " C", "Tgt: " + String(P.noz_tgt[1], 0),
               COLOR_NOZZLE, ra, filamentForTray(P, P.tray[1]));
  gfx->draw16bitRGBBitmap(0, DUAL_TOP_Y, fb->getBuffer(), 368, 130);
}

void renderMidDual(const Printer &P) {
  if (!fb) return;
  uint16_t cc = COLOR_TEXT;
  if (P.air_mode == 1)      cc = COLOR_NOZZLE;  // heating -> červená
  else if (P.air_mode == 0) cc = COLOR_BED;     // cooling -> modrá
  fb->fillScreen(COLOR_BG);
  drawCardInto(10, 0, 169, 110, "Heatbed",
               String(P.bed_temp, 0) + " C", "Tgt: " + String(P.bed_target, 0), COLOR_BED, false);
  drawCardInto(189, 0, 169, 110, "Chamber",
               String(P.chamber_temp, 0) + " C", "Tgt: " + String(P.chamber_target, 0), cc, false);
  gfx->draw16bitRGBBitmap(0, DUAL_MID_Y, fb->getBuffer(), 368, 110);
}

void renderTopSingle(const Printer &P) {
  if (!fb) return;
  fb->fillScreen(COLOR_BG);
  drawCardInto(10, 0, 169, 130, "Nozzle",
               String(P.noz_t[0], 0) + " C", "Tgt: " + String(P.noz_tgt[0], 0),
               COLOR_NOZZLE, P.noz_tgt[0] >= ACTIVE_TARGET_MIN, filamentForTray(P, P.tray[0]));
  drawCardInto(189, 0, 169, 130, "Heatbed",
               String(P.bed_temp, 0) + " C", "Tgt: " + String(P.bed_target, 0), COLOR_BED, false);
  gfx->draw16bitRGBBitmap(0, SINGLE_TOP_Y, fb->getBuffer(), 368, 130);
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
  fb->fillScreen(COLOR_BG);
  fbCentered("End: " + getETAStr(P.remaining_time_min), 70, 14, &FreeSans9pt7b, COLOR_GRAY);
  fbCentered("Rem: " + formatTimeStr(P.remaining_time_min), 298, 14, &FreeSans9pt7b, COLOR_GRAY);
  fb->fillRoundRect(27, 28, 314, 22, 6, COLOR_CARD);
  int barWidth = map(P.mc_percent, 0, 100, 0, 314);
  if (barWidth > 0) fb->fillRoundRect(27, 28, barWidth, 22, 6, COLOR_BAMBU);
  fbCentered(String(P.mc_percent) + " %", 100, 90, &FreeSansBold24pt7b, COLOR_TEXT);
  fbCentered("Layer " + String(P.current_layer) + " / " + String(P.total_layers), 260, 82, &FreeSans12pt7b, COLOR_GRAY);

  // Status fáze tisku pod progress barem
  String st = stageText(P.stg_cur);
  if (st == "" && P.gcode_state == "RUNNING") st = "Printing";
  if (st != "") fbCentered(st, 184, 120, &FreeSans12pt7b, COLOR_SUBTEXT);

  gfx->draw16bitRGBBitmap(0, y, fb->getBuffer(), 368, 130);
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
    drawPrinterLabel(idx, rotating);
  }

  if (dual) {
    bool la = (P.active_side == 0), ra = (P.active_side == 1);
    if (last_l_t != P.noz_t[0] || last_l_tgt != P.noz_tgt[0] || last_l_tray != P.tray[0] || last_l_active != (int)la ||
        last_r_t != P.noz_t[1] || last_r_tgt != P.noz_tgt[1] || last_r_tray != P.tray[1] || last_r_active != (int)ra) {
      renderTopDual(P, la, ra);
      last_l_t = P.noz_t[0]; last_l_tgt = P.noz_tgt[0]; last_l_tray = P.tray[0]; last_l_active = (int)la;
      last_r_t = P.noz_t[1]; last_r_tgt = P.noz_tgt[1]; last_r_tray = P.tray[1]; last_r_active = (int)ra;
    }
    if (last_bed_t != P.bed_temp || last_bed_tgt != P.bed_target ||
        last_cham_t != P.chamber_temp || last_cham_tgt != P.chamber_target || last_air_mode != P.air_mode) {
      renderMidDual(P);
      last_bed_t = P.bed_temp; last_bed_tgt = P.bed_target;
      last_cham_t = P.chamber_temp; last_cham_tgt = P.chamber_target; last_air_mode = P.air_mode;
    }
  } else {
    if (last_l_t != P.noz_t[0] || last_l_tgt != P.noz_tgt[0] || last_l_tray != P.tray[0] ||
        last_bed_t != P.bed_temp || last_bed_tgt != P.bed_target) {
      renderTopSingle(P);
      last_l_t = P.noz_t[0]; last_l_tgt = P.noz_tgt[0]; last_l_tray = P.tray[0];
      last_bed_t = P.bed_temp; last_bed_tgt = P.bed_target;
    }
  }

  if (last_perc != P.mc_percent || last_rem_min != P.remaining_time_min ||
      last_cur_layer != P.current_layer || last_tot_layer != P.total_layers || last_stg != P.stg_cur) {
    renderBottom(P, dual ? DUAL_BOT_Y : SINGLE_BOT_Y);
    last_perc = P.mc_percent; last_rem_min = P.remaining_time_min;
    last_cur_layer = P.current_layer; last_tot_layer = P.total_layers; last_stg = P.stg_cur;
  }
}

// ---------------------------------------------------------
// NOTIFIKACE A HODINY (přes celý displej)
// ---------------------------------------------------------
void drawNotification(int idx, bool rotating, uint16_t bg, uint16_t fg, const char* title, String sub) {
  gfx->fillScreen(bg);
  if (NUM_PRINTERS > 1) printCentered(printers[idx].name, 184, 160, &FreeSans12pt7b, fg);
  gfx->setFont(&FreeSansBold24pt7b); gfx->setTextSize(1); gfx->setTextColor(fg);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(184 - tw / 2, 215); gfx->print(title);
  if (sub != "") printCentered(sub, 184, 260, &FreeSans12pt7b, fg);
  if (rotating) {
    for (int i = 0; i < NUM_PRINTERS; i++)
      gfx->fillCircle(179 + i * 10, 300, 3, fg);
    gfx->fillCircle(179 + (idx == 0 ? 10 : 0), 300, 2, bg);   // neaktivní = prázdná tečka
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

void drawClockUI() {
  time_t now; time(&now); struct tm *ti = localtime(&now);
  if (ti->tm_year < 100) return;

  if (millis() - clock_shift_at >= CLOCK_SHIFT_MS) {
    const int minOff = CLOCK_MARGIN - CLOCK_BLOCK_TOP;
    const int maxOff = (LCD_HEIGHT - CLOCK_MARGIN) - CLOCK_BLOCK_BOT;
    int n = clock_offset;
    for (int i = 0; i < 10 && abs(n - clock_offset) < 40; i++) n = random(minOff, maxOff + 1);
    clock_offset = n;
    clock_shift_at = millis();
    gfx->fillScreen(COLOR_BG);
    last_clock_min = -1;
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
}

// ---------------------------------------------------------
// VÝBĚR REŽIMU
// ---------------------------------------------------------
DispMode computeMode(Printer &P) {
  bool err    = (P.gcode_state == "FAILED") || (P.print_error != 0);
  bool paused = (P.gcode_state == "PAUSE");
  bool active = (P.gcode_state == "RUNNING" || P.gcode_state == "PREPARE" || P.gcode_state == "SLICING");

  if (active || err || paused) P.finish_latch = false;
  if (P.gcode_state == "FINISH" && !P.finish_latch && !active && !err && !paused) {
    P.finish_latch = true; P.finish_at = millis();
  }

  if (err)    return M_RED;
  if (paused) return M_ORANGE;
  if (active) return M_DASH;
  if (P.finish_latch && (millis() - P.finish_at < FINISH_NOTIFY_MS)) return M_GREEN;
  return M_CLOCK;   // nečinnost / standby (i po 5 min od dokončení)
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
  } else if (millis() - rotate_at >= ROTATE_MS) {
    // aktivní jsou obě -> střídat
    do { shown = (shown + 1) % NUM_PRINTERS; } while (modes[shown] == M_CLOCK);
    rotate_at = millis();
  }
  bool rotating = (activeCount > 1);
  DispMode mode = (shown < 0) ? M_CLOCK : modes[shown];

  // vstup do nového režimu / jiné tiskárny -> jednorázové překreslení
  if (mode != last_mode || shown != last_shown || rotating != last_rotating) {
    last_mode = mode; last_shown = shown; last_rotating = rotating;
    last_clock_min = -1;
    if      (mode == M_DASH)   force_redraw = true;
    else if (mode == M_GREEN)  drawNotification(shown, rotating, COLOR_GREEN,  0x0000, "FINISHED", "Print complete");
    else if (mode == M_ORANGE) drawNotification(shown, rotating, COLOR_ORANGE, 0x0000, "PAUSED",   "Print paused");
    else if (mode == M_RED)    drawNotification(shown, rotating, COLOR_RED,    0xFFFF, "ERROR",
                                  printers[shown].print_error != 0 ? ("Code " + String(printers[shown].print_error))
                                                                   : "Check printer");
    else if (mode == M_CLOCK)  gfx->fillScreen(COLOR_BG);
  }

  if (mode == M_DASH)  drawDashboard(shown, rotating);
  else if (mode == M_CLOCK) drawClockUI();
  // notifikační režimy: overlay zůstává až do vyřešení, nic dalšího se nekreslí
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

  // --- KOMORA (jen H2D) ---
  // print.device.ctc.info.temp i chamber_temper mohou být zabalené jako u trysek
  // ((target<<16)|current). Bez vyhřívání je target 0 a hodnota vypadá jako prostá teplota,
  // proto se rozbaluje vše, co je >= 65536. Jakmile přijde zabalená hodnota, target se bere
  // jen z ní (chamber_target_temper pak tiskárna posílá jako 0).
  if (P.type == PRINTER_DUAL) {
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
    if (p["device"]["airduct"]["modeCur"].is<int>()) P.air_mode = p["device"]["airduct"]["modeCur"].as<int>();
  }

  // --- PROGRES / STAV ---
  if (p.containsKey("mc_percent"))        P.mc_percent = p["mc_percent"].as<int>();
  if (p.containsKey("mc_remaining_time")) P.remaining_time_min = p["mc_remaining_time"].as<int>();
  if (p.containsKey("layer_num"))         P.current_layer = p["layer_num"].as<int>();
  if (p.containsKey("total_layer_num"))   P.total_layers = p["total_layer_num"].as<int>();
  if (p.containsKey("gcode_state"))       P.gcode_state = p["gcode_state"].as<String>();
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

void callback(char* topic, byte* payload, unsigned int length) {
  // topic = device/<serial>/report -> podle sériového čísla najít tiskárnu
  Printer *P = nullptr;
  for (int i = 0; i < NUM_PRINTERS; i++)
    if (strstr(topic, printers[i].serial)) { P = &printers[i]; break; }
  if (!P) return;

  JsonDocument doc;
  if (deserializeJson(doc, payload, length)) return;
  if (!doc.containsKey("print")) return;
  parseReport(*P, doc["print"].as<JsonObject>());
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
  P.mqtt.setBufferSize(32768);
}

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

  setupPrinter(printers[0], P1_NAME, P1_TYPE, P1_IP, P1_ACCESS_CODE, P1_SERIAL);
#ifdef P2_IP
  setupPrinter(printers[1], P2_NAME, P2_TYPE, P2_IP, P2_ACCESS_CODE, P2_SERIAL);
#endif
}

// Neblokující pokus o připojení. Interval se při opakovaných neúspěších prodlužuje
// (10 s, 20 s, 40 s, max 60 s), aby vypnutá tiskárna co nejméně brzdila tu druhou.
#define MQTT_RETRY_MS     10000UL
#define MQTT_RETRY_MAX_MS 60000UL
#define OFFLINE_AFTER_MS  60000UL  // tak dlouho bez spojení -> data považujeme za neplatná

void reconnect(Printer &P, int idx) {
  unsigned long wait = min(MQTT_RETRY_MS << min(P.fails, 3), MQTT_RETRY_MAX_MS);
  if (P.last_try != 0 && millis() - P.last_try < wait) return;
  String clientId = String("ESP32_Dashboard_") + idx;
  if (P.mqtt.connect(clientId.c_str(), mqtt_user, P.code)) {
    P.fails = 0;
    P.mqtt.subscribe((String("device/") + P.serial + "/report").c_str());
    P.mqtt.publish((String("device/") + P.serial + "/request").c_str(),
                   "{\"pushing\": {\"sequence_id\": \"1\", \"command\": \"pushall\"}}");
  } else {
    P.fails++;
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
    }
    updateUI();   // i mezi tiskárnami, ať blokující connect jedné nezdržuje displej
  }

  static unsigned long last_time_update = 0;
  if (millis() - last_time_update > 30000) { last_rem_min = -1; last_time_update = millis(); }
}
