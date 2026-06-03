#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPIFFS.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <HTTPUpdate.h>
#include <esp_system.h>
#include <esp_sleep.h>
#include <vector>
#include <map>
#include <algorithm>
#include <time.h>

// ── In-memory log ring buffer (accessible via /logs) ─────────────────
#define LOG_BUF_SIZE 8192
#define LOG_REQ() logf("[req] %s  heap=%u\n", server.uri().c_str(), ESP.getFreeHeap())
static char logBuf[LOG_BUF_SIZE];
static int  logHead = 0;
static bool logWrapped = false;

void logWrite(const char* msg) {
    Serial.print(msg);
    for (int i = 0; msg[i]; i++) {
        logBuf[logHead] = msg[i];
        logHead = (logHead + 1) % LOG_BUF_SIZE;
        if (logHead == 0) logWrapped = true;
    }
}

void logf(const char* fmt, ...) {
    char tmp[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    logWrite(tmp);
}

String getLogContents() {
    String out;
    out.reserve(LOG_BUF_SIZE);
    if (logWrapped) for (int i = logHead; i < LOG_BUF_SIZE; i++) if (logBuf[i]) out += logBuf[i];
    for (int i = 0; i < logHead; i++) if (logBuf[i]) out += logBuf[i];
    return out;
}

// ── Crash tracking ──────────────────────────────────────────────────
// RTC memory survives any reset (panic, WDT, brownout) but not power cycle.
// We store a breadcrumb (last known location) and heap so we know what the
// device was doing when it crashed.
RTC_DATA_ATTR static uint32_t rtcBootCount = 0;
RTC_DATA_ATTR static char     rtcCrumb[48] = "boot";
RTC_DATA_ATTR static uint32_t rtcHeap      = 0;

static const char* CRASH_LOG_PATH = "/crash.log";

void setCrumb(const char* s) {
    strncpy(rtcCrumb, s, sizeof(rtcCrumb) - 1);
    rtcCrumb[sizeof(rtcCrumb) - 1] = '\0';
    rtcHeap = ESP.getFreeHeap();
}

void saveCrashInfo() {
    // Must be called after SPIFFS.begin()
    esp_reset_reason_t r = esp_reset_reason();
    rtcBootCount++;

    const char* rs = nullptr;
    switch (r) {
        case ESP_RST_PANIC:    rs = "panic";    break;
        case ESP_RST_INT_WDT:  rs = "int_wdt";  break;
        case ESP_RST_TASK_WDT: rs = "task_wdt"; break;
        case ESP_RST_WDT:      rs = "wdt";      break;
        case ESP_RST_BROWNOUT: rs = "brownout"; break;
        default: break;
    }
    if (!rs) return; // normal power-on or software restart — don't log

    // Keep log under 4 KB
    if (SPIFFS.exists(CRASH_LOG_PATH)) {
        File f = SPIFFS.open(CRASH_LOG_PATH, "r");
        if (f && f.size() > 3800) { f.close(); SPIFFS.remove(CRASH_LOG_PATH); }
        else if (f) f.close();
    }

    File f = SPIFFS.open(CRASH_LOG_PATH, "a");
    if (!f) return;
    f.printf("boot#%u  reason=%-10s  at=%-24s  heap=%u\n",
             (unsigned)rtcBootCount, rs, rtcCrumb, (unsigned)rtcHeap);
    f.close();
    logf("[crash] logged: reason=%s at=%s heap=%u\n", rs, rtcCrumb, (unsigned)rtcHeap);
}

// ── Firmware version ────────────────────────────────────────────────
#define FW_VERSION "1.7.0"
#define OTA_VERSION_URL      "https://raw.githubusercontent.com/blumleo2004/linetracker/master/version.json"
#define OTA_VERSION_URL_BETA "https://raw.githubusercontent.com/blumleo2004/linetracker/master/version-beta.json"
static const unsigned long OTA_CHECK_INTERVAL_MS = 6UL * 60 * 60 * 1000; // 6h

// ── Display ──────────────────────────────────────────────────────────
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);

// Fahrgastinformationssystem: amber LEDs on dark matte display
static uint16_t BG_COLOR;    // dark warm matte grey (display cover)
static uint16_t AMBER;       // bright amber LED color (#FFBF00)
static uint16_t AMBER_DIM;   // dim amber for separators/glow

// ── Config ───────────────────────────────────────────────────────────
static const char* CONFIG_PATH = "/config.json";
static const char* WIFI_RESET_FLAG      = "/wifi_reset";
static const char* WIFI_CONFIGURED_FLAG = "/wifi_ok";

// CSV cache on SPIFFS
static const char* CACHE_HALT_PATH   = "/halt.csv";
static const char* CACHE_STEIGE_PATH = "/steige.csv";
static const char* CACHE_LINIEN_PATH = "/linien.csv";
static const char* CACHE_TS_PATH     = "/cache_ts";
static const unsigned long CACHE_MAX_AGE_MS = 24UL * 60 * 60 * 1000; // 24h

// ── Struct for discovered lines at a station ─────────────────────────
struct FoundLine {
    String rbl;
    String lineName;
    String towards;
    String type;
    String stopName;
};

// Direction cache: RBL → line name, towards, type
static const char* DIR_CACHE_PATH = "/dir_cache.json";
static std::map<String, std::vector<FoundLine>> dirCache;  // rbl → all lines (multi-line RBLs)

// Static line directions: linienId → {name, type, terminusH, terminusR}
static const char* LINE_DIRS_PATH = "/line_dirs.json";
struct LineDirInfo {
    String name;      // line designation (e.g. "10A", "U6")
    String type;      // transport type (e.g. "ptTram", "ptMetro")
    String terminusH; // end station direction H
    String terminusR; // end station direction R
};
static std::map<String, LineDirInfo> lineDirMap;  // linienId → info

// Steige CSV entry (defined here so search index types are available globally)
struct SteigeInfo {
    String rbl;
    String linienId;
    String richtung;  // "H" or "R"
};

// PSRAM-backed search indexes — char arrays keep small allocations out of internal heap
struct HaltRecord   { char haltId[12]; char name[64]; };
struct SteigeRecord { char haltId[12]; char rbl[8]; char linienId[12]; char richtung; char _pad[3]; };
static HaltRecord*   haltRecords        = nullptr;
static int           haltRecordCount    = 0;
static SteigeRecord* steigeRecords      = nullptr;
static int           steigeRecordCount  = 0;

// WiFi portal state
static WiFiManager   wm;
static DNSServer     apDns;
static unsigned long wifiDownSince  = 0;
static volatile bool portalOpen     = false;
static volatile bool portalShouldOpen = false;

// ── Pong (2-Player Easter Egg) ───────────────────────────────────────
enum AppMode { MODE_DEPARTURES = 0, MODE_PONG = 1, MODE_SNAKE = 2 };
static volatile AppMode appMode = MODE_DEPARTURES;

struct PongPlayer {
    bool          active       = false;
    char          token[9]     = {0};      // 8-char hex session token
    int8_t        inputDir     = 0;         // -1 up, 0 idle, +1 down
    unsigned long lastInputMs  = 0;
    unsigned long lastSeenMs   = 0;
};

struct PongState {
    PongPlayer    left, right;
    int           leftPaddleY  = 64;
    int           rightPaddleY = 64;
    int           ballX        = 156;
    int           ballY        = 81;
    int           ballVX       = 4;
    int           ballVY       = 1;
    int           leftScore    = 0;
    int           rightScore   = 0;
    bool          gameRunning  = false;
    bool          gameOver     = false;
    int           winner       = 0;        // 0 = none, 1 = left, 2 = right
    unsigned long gameOverMs   = 0;
    // Visual feedback
    int           trailX[4]    = {0,0,0,0};
    int           trailY[4]    = {0,0,0,0};
    uint8_t       trailIdx     = 0;
    unsigned long leftHitMs    = 0;
    unsigned long rightHitMs   = 0;
    unsigned long lastGoalMs   = 0;        // 0 = no recent goal
    int           lastGoalSide = 0;         // 1=left scored, 2=right scored
    unsigned long serveAtMs    = 0;        // ball is frozen until this time
    int           rallyHits    = 0;        // for speed scaling
};
static PongState pong;
static SemaphoreHandle_t pongMutex = nullptr;

static const int  PONG_PADDLE_W       = 12;
static const int  PONG_PADDLE_H       = 42;
static const int  PONG_BALL_SIZE      = 9;
static const int  PONG_PADDLE_SPEED   = 5;
static const int  PONG_BALL_SPEED_START = 4;
static const int  PONG_BALL_SPEED_MAX   = 7;
static const int  PONG_SCORE_TO_WIN   = 5;
static const unsigned long PONG_PLAYER_TIMEOUT_MS  = 15000;
static const unsigned long PONG_ABANDON_TIMEOUT_MS = 30000;
static const unsigned long PONG_GAMEOVER_HOLD_MS   = 6500;
static const unsigned long PONG_GOAL_ANIM_MS      = 1500;   // total freeze after goal
static const unsigned long PONG_HIT_FLASH_MS       = 120;

// Score (0..5) → Stationsname (U4-Style Endstation-Progression)
static const char* PONG_STATIONS[] = {
    "Karlsplatz", "Stephansplatz", "Schwedenplatz",
    "Praterstern", "Schottenring", "Heiligenstadt"
};

// ── Snake (1-Player Easter Egg, U-Bahn-themed) ──────────────────────
static const int SNAKE_CELL    = 10;
static const int SNAKE_HEADER  = 18;       // top strip for score
static const int SNAKE_COLS    = 32;       // 320 / 10
static const int SNAKE_ROWS    = 15;       // (170 - 18) / 10 ≈ 15
static const int SNAKE_MAX_LEN = 80;
static const unsigned long SNAKE_PLAYER_TIMEOUT_MS  = 15000;
static const unsigned long SNAKE_ABANDON_TIMEOUT_MS = 30000;
static const unsigned long SNAKE_GAMEOVER_HOLD_MS   = 5000;
static const int SNAKE_TICK_START_MS = 230;
static const int SNAKE_TICK_MIN_MS   = 90;

struct SnakeCell { int8_t x, y; };

struct SnakeState {
    bool          active        = false;
    char          token[9]      = {0};
    int8_t        dir           = 0;        // 0=right 1=down 2=left 3=up
    int8_t        pendingDir    = 0;        // queued input for next tick
    unsigned long lastSeenMs    = 0;
    unsigned long lastInputMs   = 0;
    unsigned long lastTickMs    = 0;
    int           tickIntervalMs = SNAKE_TICK_START_MS;
    SnakeCell     body[SNAKE_MAX_LEN];      // body[0] = tail, body[len-1] = head
    int           length        = 0;
    SnakeCell     food          = {0, 0};
    int           score         = 0;
    int           highScore     = 0;
    bool          gameRunning   = false;
    bool          gameOver      = false;
    unsigned long gameOverMs    = 0;
    unsigned long deathFlashMs  = 0;
};
static SnakeState snake;
static SemaphoreHandle_t snakeMutex = nullptr;

void loadDirCache() {
    if (!SPIFFS.exists(DIR_CACHE_PATH)) return;
    File f = SPIFFS.open(DIR_CACHE_PATH, "r");
    if (!f) return;
    JsonDocument doc;
    if (deserializeJson(doc, f)) { f.close(); return; }
    f.close();
    for (JsonPair kv : doc.as<JsonObject>()) {
        String rbl = kv.key().c_str();
        for (JsonObject obj : kv.value().as<JsonArray>()) {
            FoundLine fl;
            fl.rbl = rbl; fl.lineName = obj["n"] | ""; fl.towards = obj["t"] | "";
            fl.type = obj["y"] | ""; fl.stopName = obj["s"] | "";
            if (fl.lineName.length() > 0) dirCache[rbl].push_back(fl);
        }
    }
    int total = 0; for (auto& kv2 : dirCache) total += kv2.second.size();
    Serial.printf("Direction cache loaded: %d RBLs, %d lines\n", dirCache.size(), total);
}

void saveDirCache() {
    JsonDocument doc;
    for (auto& pair : dirCache) {
        JsonArray arr = doc[pair.first].to<JsonArray>();
        for (auto& fl : pair.second) {
            JsonObject obj = arr.add<JsonObject>();
            obj["n"] = fl.lineName; obj["t"] = fl.towards;
            obj["y"] = fl.type;    obj["s"] = fl.stopName;
        }
    }
    File f = SPIFFS.open(DIR_CACHE_PATH, "w");
    if (!f) return;
    serializeJson(doc, f);
    f.close();
}

void cacheDirEntry(const String& rbl, const String& name, const String& towards, const String& type, const String& stop) {
    auto& vec = dirCache[rbl];
    for (auto& fl : vec) { if (fl.lineName == name && fl.towards == towards) return; }
    FoundLine fl;
    fl.rbl = rbl; fl.lineName = name; fl.towards = towards; fl.type = type; fl.stopName = stop;
    vec.push_back(fl);
}

void loadLineDirections() {
    lineDirMap.clear();
    if (!SPIFFS.exists(LINE_DIRS_PATH)) return;
    File f = SPIFFS.open(LINE_DIRS_PATH, "r");
    if (!f) return;
    JsonDocument doc;
    if (deserializeJson(doc, f)) { f.close(); return; }
    f.close();
    for (JsonPair kv : doc.as<JsonObject>()) {
        LineDirInfo info;
        info.name      = kv.value()["n"] | "";
        info.type      = kv.value()["y"] | "";
        info.terminusH = kv.value()["h"] | "";
        info.terminusR = kv.value()["r"] | "";
        if (info.name.length() > 0) {
            lineDirMap[String(kv.key().c_str())] = info;
        }
    }
    logf("Line directions loaded: %d entries\n", lineDirMap.size());
}

static int  cfgRotateSec      = 5;     // page switch interval in seconds (default 5)
static int  cfgBrightness     = 255;   // backlight 0-255 (default max)
static int  cfgNightFrom      = -1;    // night mode start hour (0-23), -1 = disabled
static int  cfgNightTo        = -1;    // night mode end hour (0-23)
static int  cfgNightBright    = 20;    // backlight during night mode
// Standby / sleep: display fully off (backlight + panel sleep) during a window.
static int  cfgStandbyFrom     = -1;    // standby start hour (0-23), -1 = disabled
static int  cfgStandbyTo       = -1;    // standby end hour (0-23)
static bool cfgStandbyDeepSleep = false; // EXPERIMENTAL: ESP32 deep sleep (timer wakeup) instead of panel sleep
// Weekend schedule: separate night/standby windows for Sat/Sun when enabled.
static bool cfgWeekendSchedule = false;
static int  cfgNightFromWe     = -1;
static int  cfgNightToWe       = -1;
static int  cfgStandbyFromWe   = -1;
static int  cfgStandbyToWe     = -1;
static bool   cfgShowNext        = false; // show next departure below main countdown
static bool   cfgShowDisruptions = false; // show WL disruption ticker at bottom
static bool   cfgShowClock        = false; // show a clock as its own page in the rotation
static bool   cfgShowWeather      = false; // show a weather page (Vienna, Open-Meteo) in the rotation
static bool   cfgLineColors       = false; // tint U-Bahn lines in official colors (default off = amber FIS)
static bool   cfgSortByTime      = true;  // sort display slots by countdown
static bool   cfgBetaChannel     = false; // pull OTA updates from the beta channel (version-beta.json)
static String cfgHostname        = "";    // mDNS hostname, generated from MAC on first boot

struct ConfigLine {
    String rbl;
    String name;
    String towards;
    String type;
    int    walkMin = 0;  // minutes to walk to stop (0 = off; when countdown<=walkMin the slot blinks "leave now")
};
static std::vector<ConfigLine> cfgLines;

struct OebbStation {
    String stationName;  // canonical ÖBB station name (e.g. "Wien Rennweg")
    String line;         // e.g. "S 3", "REX 7"
    String towards;      // destination direction
    int    walkMin = 0;
};
static std::vector<OebbStation> cfgOebb;

// Watch group: N selected lines merged into N display slots
struct WatchGroupEntry {
    String source;       // "wl" or "oebb"
    String rbl;          // WL: platform ID
    String oebbStation;  // ÖBB: station name
    String lineName;
    String towards;
    String type;
    int    walkMin = 0;
};
struct WatchGroup {
    String label;
    int    maxDepartures; // 1-5
    std::vector<WatchGroupEntry> entries;
};
static std::vector<WatchGroup> cfgWatchGroups;

bool loadConfig() {
    if (!SPIFFS.exists(CONFIG_PATH)) return false;
    File f = SPIFFS.open(CONFIG_PATH, "r");
    if (!f) return false;
    JsonDocument doc;
    if (deserializeJson(doc, f)) { f.close(); return false; }
    f.close();

    cfgLines.clear();
    cfgOebb.clear();
    cfgWatchGroups.clear();

    if (doc["lines"].is<JsonArray>()) {
        for (JsonObject line : doc["lines"].as<JsonArray>()) {
            ConfigLine cl;
            cl.rbl     = line["rbl"].as<String>();
            cl.name    = line["name"] | "";
            cl.towards = line["towards"] | "";
            cl.type    = line["type"] | "";
            cl.walkMin = line["walk"]  | 0;
            if (cl.walkMin < 0)  cl.walkMin = 0;
            if (cl.walkMin > 30) cl.walkMin = 30;
            if (cl.rbl.length() > 0) cfgLines.push_back(cl);
        }
    } else if (doc["rbls"].is<String>()) {
        String rbls = doc["rbls"].as<String>();
        while (rbls.length() > 0) {
            int comma = rbls.indexOf(',');
            String rbl;
            if (comma == -1) { rbl = rbls; rbls = ""; }
            else { rbl = rbls.substring(0, comma); rbls = rbls.substring(comma + 1); }
            rbl.trim();
            if (rbl.length() > 0) { ConfigLine cl; cl.rbl = rbl; cfgLines.push_back(cl); }
        }
    }
    if (doc["oebb"].is<JsonArray>()) {
        for (JsonObject stn : doc["oebb"].as<JsonArray>()) {
            OebbStation os;
            os.stationName = stn["station"].as<String>();
            os.line        = stn["line"] | "";
            os.towards     = stn["towards"] | "";
            os.walkMin     = stn["walk"]  | 0;
            if (os.walkMin < 0)  os.walkMin = 0;
            if (os.walkMin > 30) os.walkMin = 30;
            if (os.stationName.length() > 0) cfgOebb.push_back(os);
        }
    }
    if (doc["watch_groups"].is<JsonArray>()) {
        for (JsonObject wg : doc["watch_groups"].as<JsonArray>()) {
            WatchGroup g;
            g.label          = wg["label"] | "";
            g.maxDepartures  = wg["max"] | 2;
            if (wg["entries"].is<JsonArray>()) {
                for (JsonObject e : wg["entries"].as<JsonArray>()) {
                    WatchGroupEntry en;
                    en.source      = e["src"] | "wl";
                    en.rbl         = e["rbl"] | "";
                    en.oebbStation = e["station"] | "";
                    en.lineName    = e["line"] | "";
                    en.towards     = e["towards"] | "";
                    en.type        = e["type"] | "";
                    en.walkMin     = e["walk"] | 0;
                    if (en.walkMin < 0)  en.walkMin = 0;
                    if (en.walkMin > 30) en.walkMin = 30;
                    if (en.lineName.length() > 0) g.entries.push_back(en);
                }
            }
            if (!g.entries.empty()) cfgWatchGroups.push_back(g);
        }
    }

    cfgRotateSec       = doc["rotate_sec"]        | 5;
    cfgBrightness      = doc["brightness"]        | 255;
    cfgNightFrom       = doc["night_from"]        | -1;
    cfgNightTo         = doc["night_to"]          | -1;
    cfgNightBright     = doc["night_bright"]      | 20;
    cfgStandbyFrom     = doc["standby_from"]      | -1;
    cfgStandbyTo       = doc["standby_to"]        | -1;
    cfgStandbyDeepSleep = doc["standby_deep"]     | false;
    cfgWeekendSchedule = doc["weekend_sched"]     | false;
    cfgNightFromWe     = doc["night_from_we"]     | -1;
    cfgNightToWe       = doc["night_to_we"]       | -1;
    cfgStandbyFromWe   = doc["standby_from_we"]   | -1;
    cfgStandbyToWe     = doc["standby_to_we"]     | -1;
    cfgShowNext        = doc["show_next"]         | false;
    cfgShowDisruptions = doc["show_disruptions"]  | false;
    cfgShowClock       = doc["show_clock"]        | false;
    cfgShowWeather     = doc["show_weather"]      | false;
    cfgLineColors      = doc["line_colors"]       | false;
    cfgSortByTime      = doc["sort_by_time"]      | true;
    cfgBetaChannel     = doc["beta_channel"]      | false;
    cfgHostname        = doc["hostname"]          | "";
    if (cfgRotateSec   < 2)   cfgRotateSec   = 2;
    if (cfgRotateSec   > 60)  cfgRotateSec   = 60;
    if (cfgBrightness  < 10)  cfgBrightness  = 10;
    if (cfgBrightness  > 255) cfgBrightness  = 255;
    if (cfgNightBright < 0)   cfgNightBright = 0;
    if (cfgNightBright > 255) cfgNightBright = 255;

    return cfgLines.size() > 0 || cfgOebb.size() > 0 || !cfgWatchGroups.empty();
}

void saveConfig() {
    File f = SPIFFS.open(CONFIG_PATH, "w");
    if (!f) return;
    JsonDocument doc;
    JsonArray arr = doc["lines"].to<JsonArray>();
    for (auto& cl : cfgLines) {
        JsonObject obj = arr.add<JsonObject>();
        obj["rbl"]     = cl.rbl;
        obj["name"]    = cl.name;
        obj["towards"] = cl.towards;
        obj["type"]    = cl.type;
        if (cl.walkMin > 0) obj["walk"] = cl.walkMin;
    }
    JsonArray oArr = doc["oebb"].to<JsonArray>();
    for (auto& os : cfgOebb) {
        JsonObject obj = oArr.add<JsonObject>();
        obj["station"] = os.stationName;
        obj["line"]    = os.line;
        obj["towards"] = os.towards;
        if (os.walkMin > 0) obj["walk"] = os.walkMin;
    }
    JsonArray wgArr = doc["watch_groups"].to<JsonArray>();
    for (auto& g : cfgWatchGroups) {
        JsonObject obj = wgArr.add<JsonObject>();
        obj["label"] = g.label;
        obj["max"]   = g.maxDepartures;
        JsonArray eArr = obj["entries"].to<JsonArray>();
        for (auto& e : g.entries) {
            JsonObject eo = eArr.add<JsonObject>();
            eo["src"]     = e.source;
            eo["line"]    = e.lineName;
            eo["towards"] = e.towards;
            eo["type"]    = e.type;
            if (e.source == "wl")   eo["rbl"]     = e.rbl;
            else                    eo["station"]  = e.oebbStation;
            if (e.walkMin > 0) eo["walk"] = e.walkMin;
        }
    }
    doc["rotate_sec"]       = cfgRotateSec;
    doc["brightness"]       = cfgBrightness;
    doc["night_from"]       = cfgNightFrom;
    doc["night_to"]         = cfgNightTo;
    doc["night_bright"]     = cfgNightBright;
    doc["standby_from"]     = cfgStandbyFrom;
    doc["standby_to"]       = cfgStandbyTo;
    doc["standby_deep"]     = cfgStandbyDeepSleep;
    doc["weekend_sched"]    = cfgWeekendSchedule;
    doc["night_from_we"]    = cfgNightFromWe;
    doc["night_to_we"]      = cfgNightToWe;
    doc["standby_from_we"]  = cfgStandbyFromWe;
    doc["standby_to_we"]    = cfgStandbyToWe;
    doc["show_next"]        = cfgShowNext;
    doc["show_disruptions"] = cfgShowDisruptions;
    doc["show_clock"]       = cfgShowClock;
    doc["show_weather"]     = cfgShowWeather;
    doc["line_colors"]      = cfgLineColors;
    doc["sort_by_time"]     = cfgSortByTime;
    doc["beta_channel"]     = cfgBetaChannel;
    doc["hostname"]         = cfgHostname;
    serializeJson(doc, f);
    f.close();
}

// ── Data model ───────────────────────────────────────────────────────
struct Departure {
    String lineName;
    String towards;
    String type;
    int    countdown;
    bool   realtime;
    int    watchIdx = -1; // -1 = normal, >=0 = station watch entry index
    int    walkMin  = 0;  // resolved walk-to-stop minutes (0 = off)
};

static const int MAX_ROWS = 3;                // departure rows per display page
static std::vector<Departure> departures;     // all departures (raw)
static std::vector<Departure> displaySlots;   // one per line+direction (smart grouped)
static std::vector<String>    disruptions;    // active WL disruption titles
static SemaphoreHandle_t dataMutex;
static const unsigned long FETCH_INTERVAL_MS = 20000;
static bool fetchError = false;
static volatile bool configChanged = false;   // triggers immediate refetch
static volatile bool otaInProgress = false;
static volatile int  otaPercent    = 0;
static String        otaNewVersion = "";

// ── Config Web Server ────────────────────────────────────────────────
WebServer server(80);
static bool configMode = false;

// ── CSV Cache ────────────────────────────────────────────────────────
bool downloadCsvToCache(const char* url, const char* path) {
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.begin(client, url);
    http.setTimeout(20000);
    int code = http.GET();
    if (code != 200) { http.end(); return false; }

    File f = SPIFFS.open(path, "w");
    if (!f) { http.end(); return false; }

    WiFiClient* stream = http.getStreamPtr();
    uint8_t buf[512];
    int total = http.getSize(); // -1 if chunked/unknown
    int received = 0;
    while (http.connected() && (total < 0 || received < total)) {
        int avail = stream->available();
        if (avail > 0) {
            int len = stream->readBytes(buf, min(avail, (int)sizeof(buf)));
            f.write(buf, len);
            received += len;
        } else {
            delay(1);
        }
    }
    f.close();
    http.end();
    // If content-length was known and we got less, the download was truncated
    if (total > 0 && received < total) {
        SPIFFS.remove(path);
        logf("Download truncated %s: %d/%d bytes\n", path, received, total);
        return false;
    }
    logf("Downloaded %s: %d bytes\n", path, received);
    return received > 0;
}

bool isCacheValid() {
    if (!SPIFFS.exists(CACHE_TS_PATH)) return false;
    File f = SPIFFS.open(CACHE_TS_PATH, "r");
    if (!f) return false;
    String ts = f.readString();
    f.close();
    time_t cached = (time_t)strtoull(ts.c_str(), NULL, 10);
    if (cached < 1700000000) return false;  // invalid or legacy millis() timestamp
    time_t now;
    time(&now);
    if (now < 1700000000) return true;  // NTP not synced yet, trust existing cache files
    return difftime(now, cached) < (CACHE_MAX_AGE_MS / 1000);
}

void saveCacheTimestamp() {
    time_t now;
    time(&now);
    File f = SPIFFS.open(CACHE_TS_PATH, "w");
    if (f) { f.print((unsigned long long)now); f.close(); }
}

void buildLineDirections() {
    setCrumb("buildLineDirections");
    logf("Building line directions... heap=%u psram=%u\n",
         ESP.getFreeHeap(), ESP.getFreePsram());

    // Step 1: Load all line info from linien.csv
    std::map<String, std::pair<String,String>> lineInfoMap; // id → {name, type}
    {
        File f = SPIFFS.open(CACHE_LINIEN_PATH, "r");
        if (!f) { logf("linien.csv not found\n"); return; }
        while (f.available()) {
            String line = f.readStringUntil('\n');
            int s1 = line.indexOf(';'); if (s1 < 0) continue;
            String id = line.substring(0, s1); id.replace("\"", "");
            int s2 = line.indexOf(';', s1 + 1); if (s2 < 0) continue;
            String name = line.substring(s1 + 1, s2); name.replace("\"", "");
            int s3 = line.indexOf(';', s2 + 1); if (s3 < 0) continue;
            int s4 = line.indexOf(';', s3 + 1); if (s4 < 0) continue;
            int s5 = line.indexOf(';', s4 + 1);
            String type = (s5 > 0) ? line.substring(s4 + 1, s5) : line.substring(s4 + 1);
            type.replace("\"", ""); type.trim();
            if (name.length() > 0) lineInfoMap[id] = {name, type};
        }
        f.close();
    }

    // Step 2: Scan steige.csv for terminus stations (highest REIHENFOLGE per line+direction)
    struct TermInfo { int maxH = -1; String haltH; int maxR = -1; String haltR; };
    std::map<String, TermInfo> termMap;
    {
        const int MAX_STEIGE = 10000;
        if (steigeRecords) { free(steigeRecords); steigeRecords = nullptr; steigeRecordCount = 0; }
        steigeRecords = (SteigeRecord*)ps_malloc(MAX_STEIGE * sizeof(SteigeRecord));
        if (!steigeRecords) logf("[buildLD] WARN: ps_malloc steigeRecords failed\n");

        File f = SPIFFS.open(CACHE_STEIGE_PATH, "r");
        if (!f) { logf("steige.csv not found\n"); return; }
        int yieldCounter = 0;
        while (f.available()) {
            if (++yieldCounter % 200 == 0) vTaskDelay(pdMS_TO_TICKS(1));
            String line = f.readStringUntil('\n');
            int s1 = line.indexOf(';'); if (s1 < 0) continue;
            int s2 = line.indexOf(';', s1 + 1); if (s2 < 0) continue;
            int s3 = line.indexOf(';', s2 + 1); if (s3 < 0) continue;
            int s4 = line.indexOf(';', s3 + 1); if (s4 < 0) continue;
            int s5 = line.indexOf(';', s4 + 1); if (s5 < 0) continue;
            String linienId = line.substring(s1 + 1, s2); linienId.replace("\"", "");
            String haltId   = line.substring(s2 + 1, s3); haltId.replace("\"", "");
            String richtung = line.substring(s3 + 1, s4); richtung.replace("\"", "");
            String seqStr   = line.substring(s4 + 1, s5); seqStr.replace("\"", "");
            int seq = seqStr.toInt();
            auto& ti = termMap[linienId];
            if (richtung == "H" && seq > ti.maxH) { ti.maxH = seq; ti.haltH = haltId; }
            else if (richtung == "R" && seq > ti.maxR) { ti.maxR = seq; ti.haltR = haltId; }
            // Fill steigeRecords (deduplicated via sort+unique after scan)
            int s6 = line.indexOf(';', s5 + 1);
            if (s6 >= 0 && steigeRecords && steigeRecordCount < MAX_STEIGE) {
                String rbl = line.substring(s5 + 1, s6); rbl.replace("\"", ""); rbl.trim();
                if (rbl.length() > 0) {
                    auto& r = steigeRecords[steigeRecordCount++];
                    strncpy(r.haltId,   haltId.c_str(),   11); r.haltId[11]   = '\0';
                    strncpy(r.rbl,      rbl.c_str(),        7); r.rbl[7]       = '\0';
                    strncpy(r.linienId, linienId.c_str(),  11); r.linienId[11] = '\0';
                    r.richtung = (richtung == "H") ? 'H' : 'R';
                }
            }
        }
        f.close();
        if (steigeRecords && steigeRecordCount > 0) {
            std::sort(steigeRecords, steigeRecords + steigeRecordCount, [](const SteigeRecord& a, const SteigeRecord& b) {
                int c = strcmp(a.haltId, b.haltId); if (c != 0) return c < 0;
                c = strcmp(a.rbl, b.rbl); if (c != 0) return c < 0;
                return strcmp(a.linienId, b.linienId) < 0;
            });
            auto endIt = std::unique(steigeRecords, steigeRecords + steigeRecordCount, [](const SteigeRecord& a, const SteigeRecord& b) {
                return strcmp(a.haltId, b.haltId) == 0 && strcmp(a.rbl, b.rbl) == 0 && strcmp(a.linienId, b.linienId) == 0;
            });
            steigeRecordCount = endIt - steigeRecords;
        }
        logf("[buildLD] steige scan done: %d records, heap=%u psram=%u\n",
             steigeRecordCount, ESP.getFreeHeap(), ESP.getFreePsram());
    }

    // Step 3: Scan full halt.csv — resolve terminus names AND build search index
    std::map<String, String> haltNames;
    for (auto& p : termMap) {
        if (p.second.haltH.length() > 0) haltNames[p.second.haltH] = "";
        if (p.second.haltR.length() > 0) haltNames[p.second.haltR] = "";
    }
    const int MAX_HALT = 2200;
    if (haltRecords) { free(haltRecords); haltRecords = nullptr; haltRecordCount = 0; }
    haltRecords = (HaltRecord*)ps_malloc(MAX_HALT * sizeof(HaltRecord));
    if (!haltRecords) logf("[buildLD] WARN: ps_malloc haltRecords failed\n");
    {
        File f = SPIFFS.open(CACHE_HALT_PATH, "r");
        if (!f) { logf("halt.csv not found\n"); return; }
        while (f.available()) {
            String line = f.readStringUntil('\n');
            int s1 = line.indexOf(';'); if (s1 < 0) continue;
            String haltId = line.substring(0, s1); haltId.replace("\"", "");
            int s2 = line.indexOf(';', s1 + 1); if (s2 < 0) continue;
            int s3 = line.indexOf(';', s2 + 1); if (s3 < 0) continue;
            int s4 = line.indexOf(';', s3 + 1); if (s4 < 0) continue;
            String name = line.substring(s3 + 1, s4); name.replace("\"", "");
            if (name.length() == 0) continue;
            auto it = haltNames.find(haltId);
            if (it != haltNames.end()) it->second = name;
            if (haltRecords && haltRecordCount < MAX_HALT) {
                auto& r = haltRecords[haltRecordCount++];
                strncpy(r.haltId, haltId.c_str(), 11); r.haltId[11] = '\0';
                strncpy(r.name,   name.c_str(),   63); r.name[63]   = '\0';
            }
        }
        f.close();
    }

    // Step 4: Build and save JSON
    JsonDocument doc;
    int count = 0;
    for (auto& p : termMap) {
        auto li = lineInfoMap.find(p.first);
        if (li == lineInfoMap.end()) continue;
        JsonObject obj = doc[p.first].to<JsonObject>();
        obj["n"] = li->second.first;
        obj["y"] = li->second.second;
        obj["h"] = haltNames[p.second.haltH];
        obj["r"] = haltNames[p.second.haltR];
        count++;
    }
    {
        File f = SPIFFS.open(LINE_DIRS_PATH, "w");
        if (f) { serializeJson(doc, f); f.close(); }
    }
    logf("Line directions built: %d lines, halt index: %d, steige index: %d, heap=%u psram=%u\n",
         count, haltRecordCount, steigeRecordCount, ESP.getFreeHeap(), ESP.getFreePsram());

    // Reload into memory
    loadLineDirections();
    logf("lineDirMap loaded: %d entries\n", lineDirMap.size());
}

bool refreshCsvCache(bool force = false) {
    if (!force && isCacheValid()
        && SPIFFS.exists(CACHE_HALT_PATH)
        && SPIFFS.exists(CACHE_STEIGE_PATH)
        && SPIFFS.exists(CACHE_LINIEN_PATH)) {
        logf("CSV cache valid, skipping download\n");
        return true;
    }
    logf("Downloading CSV cache...\n");
    bool ok = true;
    ok &= downloadCsvToCache("https://data.wien.gv.at/csv/wienerlinien-ogd-haltestellen.csv", CACHE_HALT_PATH);
    ok &= downloadCsvToCache("https://data.wien.gv.at/csv/wienerlinien-ogd-steige.csv", CACHE_STEIGE_PATH);
    ok &= downloadCsvToCache("https://data.wien.gv.at/csv/wienerlinien-ogd-linien.csv", CACHE_LINIEN_PATH);
    if (ok) {
        saveCacheTimestamp();
        logf("CSV cache updated\n");
    } else {
        logf("CSV cache download failed (partial)\n");
    }
    return ok;
}

// Look up line name and type from linien cache by LINIEN_ID
// Linien CSV: "LINIEN_ID";"BEZEICHNUNG";"REIHENFOLGE";"ECHTZEIT";"VERKEHRSMITTEL";"STAND"
bool lookupLineInfo(const String& linienId, String& outName, String& outType) {
    File f = SPIFFS.open(CACHE_LINIEN_PATH, "r");
    if (!f) return false;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        int s1 = line.indexOf(';');
        if (s1 < 0) continue;
        String id = line.substring(0, s1);
        id.replace("\"", "");
        if (id != linienId) continue;
        // BEZEICHNUNG (field 2)
        int s2 = line.indexOf(';', s1 + 1);
        if (s2 < 0) { f.close(); return false; }
        outName = line.substring(s1 + 1, s2);
        outName.replace("\"", "");
        // skip REIHENFOLGE (field 3)
        int s3 = line.indexOf(';', s2 + 1);
        if (s3 < 0) { f.close(); return false; }
        // skip ECHTZEIT (field 4)
        int s4 = line.indexOf(';', s3 + 1);
        if (s4 < 0) { f.close(); return false; }
        // VERKEHRSMITTEL (field 5)
        int s5 = line.indexOf(';', s4 + 1);
        String vm = (s5 > 0) ? line.substring(s4 + 1, s5) : line.substring(s4 + 1);
        vm.replace("\"", "");
        vm.trim();
        outType = vm;
        f.close();
        return true;
    }
    f.close();
    return false;
}

// Normalize string for search: Umlauts to ASCII, lowercase
String normalizeForSearch(const String& s) {
    String r = s;
    r.toLowerCase();
    r.replace("ä", "ae"); r.replace("ö", "oe");
    r.replace("ü", "ue"); r.replace("ß", "ss");
    return r;
}

// Transport type priority for sorting search results
int transportPriority(const String& type) {
    if (type == "ptMetro") return 0;
    if (type == "ptTram" || type == "ptTramWLB") return 1;
    if (type.startsWith("ptBus")) return 2;
    if (type == "ptTrainS") return 3;
    return 4;
}

// Search stations from cached haltestellen CSV (Umlaut-tolerant)
std::vector<std::pair<String,String>> searchStations(const String& query) {
    std::vector<std::pair<String,String>> results;

    String lowerQuery = query;
    lowerQuery.toLowerCase();
    String normQuery = normalizeForSearch(query);

    // Fast path: use pre-built PSRAM index (built during buildLineDirections)
    if (haltRecords && haltRecordCount > 0) {
        for (int i = 0; i < haltRecordCount; i++) {
            String name = String(haltRecords[i].name);
            String lowerName = name; lowerName.toLowerCase();
            String normName = normalizeForSearch(name);
            String normNameNS = normName; normNameNS.replace(" ", "");
            String normQueryNS = normQuery; normQueryNS.replace(" ", "");
            if (lowerName.indexOf(lowerQuery) >= 0 || normName.indexOf(normQuery) >= 0 || normNameNS.indexOf(normQueryNS) >= 0) {
                results.push_back({String(haltRecords[i].haltId), name});
                if (results.size() >= 15) break;
            }
        }
        return results;
    }

    // Slow fallback: scan halt.csv (used before buildLineDirections completes)
    File f = SPIFFS.open(CACHE_HALT_PATH, "r");
    if (!f) return results;

    while (f.available()) {
        String line = f.readStringUntil('\n');
        // CSV: "HALTESTELLEN_ID";"TYP";"DIVA";"NAME";"GEMEINDE";...
        int s1 = line.indexOf(';');
        if (s1 < 0) continue;
        int s2 = line.indexOf(';', s1 + 1);
        if (s2 < 0) continue;
        int s3 = line.indexOf(';', s2 + 1);
        if (s3 < 0) continue;
        int s4 = line.indexOf(';', s3 + 1);
        if (s4 < 0) continue;
        String name = line.substring(s3 + 1, s4);
        name.replace("\"", "");
        if (name.length() == 0) continue;

        // Match against both lowercase UTF-8 and normalized ASCII (incl. space-stripped)
        String lowerName = name;
        lowerName.toLowerCase();
        String normName = normalizeForSearch(name);
        String normNameNS = normName; normNameNS.replace(" ", "");
        String normQueryNS = normQuery; normQueryNS.replace(" ", "");

        if (lowerName.indexOf(lowerQuery) >= 0 || normName.indexOf(normQuery) >= 0 || normNameNS.indexOf(normQueryNS) >= 0) {
            String haltId = line.substring(0, s1);
            haltId.replace("\"", "");
            if (haltId.length() > 0) {
                results.push_back({haltId, name});
            }
        }
        if (results.size() >= 15) break;
    }
    f.close();
    return results;
}

// Find all RBLs (with line IDs) for a given station ID from cached steige CSV
std::vector<SteigeInfo> findSteigeForStation(const String& haltId) {
    std::vector<SteigeInfo> results;

    File f = SPIFFS.open(CACHE_STEIGE_PATH, "r");
    if (!f) return results;

    while (f.available()) {
        String line = f.readStringUntil('\n');
        if (line.indexOf(haltId) < 0) continue;
        int s1 = line.indexOf(';');
        if (s1 < 0) continue;
        int s2 = line.indexOf(';', s1 + 1);
        if (s2 < 0) continue;
        int s3 = line.indexOf(';', s2 + 1);
        if (s3 < 0) continue;
        String foundHalt = line.substring(s2 + 1, s3);
        foundHalt.replace("\"", "");
        if (foundHalt != haltId) continue;

        // RICHTUNG
        int s4 = line.indexOf(';', s3 + 1);
        if (s4 < 0) continue;
        String richtung = line.substring(s3 + 1, s4);
        richtung.replace("\"", "");

        // skip REIHENFOLGE
        int s5 = line.indexOf(';', s4 + 1);
        if (s5 < 0) continue;
        // RBL_NUMMER
        int s6 = line.indexOf(';', s5 + 1);
        if (s6 < 0) continue;
        String rbl = line.substring(s5 + 1, s6);
        rbl.replace("\"", "");
        rbl.trim();

        // FK_LINIEN_ID
        String linienId = line.substring(s1 + 1, s2);
        linienId.replace("\"", "");

        if (rbl.length() > 0) {
            bool dup = false;
            for (auto& r : results) { if (r.rbl == rbl && r.linienId == linienId) { dup = true; break; } }
            if (!dup) results.push_back({rbl, linienId, richtung});
        }
    }
    f.close();
    return results;
}

// Read steige.csv once for multiple station IDs simultaneously
std::map<String, std::vector<SteigeInfo>> findSteigeForStations(const std::vector<String>& haltIds) {
    std::map<String, std::vector<SteigeInfo>> results;
    // Fast path: PSRAM sorted array + binary search
    if (steigeRecords && steigeRecordCount > 0) {
        for (auto& id : haltIds) {
            SteigeRecord key; strncpy(key.haltId, id.c_str(), 11); key.haltId[11] = '\0';
            auto lo = std::lower_bound(steigeRecords, steigeRecords + steigeRecordCount, key,
                [](const SteigeRecord& a, const SteigeRecord& b) { return strcmp(a.haltId, b.haltId) < 0; });
            auto& vec = results[id];
            while (lo < steigeRecords + steigeRecordCount && strcmp(lo->haltId, key.haltId) == 0) {
                vec.push_back({String(lo->rbl), String(lo->linienId), lo->richtung == 'H' ? "H" : "R"});
                ++lo;
            }
        }
        return results;
    }

    // Slow fallback: scan steige.csv
    unsigned long t0 = millis();
    File f = SPIFFS.open(CACHE_STEIGE_PATH, "r");
    if (!f) return results;
    int linesRead = 0, linesMatched = 0;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        linesRead++;
        // Quick pre-filter: skip lines that don't contain any of the halt IDs
        bool preMatch = false;
        for (auto& id : haltIds) { if (line.indexOf(id) >= 0) { preMatch = true; break; } }
        if (!preMatch) continue;
        int s1 = line.indexOf(';'); if (s1 < 0) continue;
        int s2 = line.indexOf(';', s1 + 1); if (s2 < 0) continue;
        int s3 = line.indexOf(';', s2 + 1); if (s3 < 0) continue;
        String foundHalt = line.substring(s2 + 1, s3);
        foundHalt.replace("\"", "");
        bool matched = false;
        for (auto& id : haltIds) { if (foundHalt == id) { matched = true; break; } }
        if (!matched) continue;
        linesMatched++;
        int s4 = line.indexOf(';', s3 + 1); if (s4 < 0) continue;
        String richtung = line.substring(s3 + 1, s4); richtung.replace("\"", "");
        int s5 = line.indexOf(';', s4 + 1); if (s5 < 0) continue;
        int s6 = line.indexOf(';', s5 + 1); if (s6 < 0) continue;
        String rbl = line.substring(s5 + 1, s6); rbl.replace("\"", ""); rbl.trim();
        String linienId = line.substring(s1 + 1, s2); linienId.replace("\"", "");
        if (rbl.length() > 0) {
            auto& vec = results[foundHalt];
            bool dup = false;
            for (auto& r : vec) { if (r.rbl == rbl && r.linienId == linienId) { dup = true; break; } }
            if (!dup) vec.push_back({rbl, linienId, richtung});
        }
    }
    f.close();
    logf("[steige] %lums, %d lines read, %d matched\n", millis()-t0, linesRead, linesMatched);
    return results;
}

// Query the monitor API to see what lines/directions are at given RBLs
std::vector<FoundLine> probeRbls(const std::vector<String>& rbls) {
    std::vector<FoundLine> results;
    if (rbls.empty()) return results;

    for (size_t start = 0; start < rbls.size(); start += 10) {
        String url = "https://www.wienerlinien.at/ogd_realtime/monitor?activateTrafficInfo=stoerunglang";
        size_t end = min(start + 10, rbls.size());
        for (size_t i = start; i < end; i++) {
            url += "&rbl=" + rbls[i];
        }

        WiFiClientSecure client;
        client.setInsecure();
        HTTPClient http;
        http.begin(client, url);
        http.setTimeout(12000);
        int code = http.GET();
        if (code != 200) { http.end(); continue; }

        String payload = http.getString();
        http.end();

        JsonDocument filter;
        filter["data"]["monitors"][0]["locationStop"]["properties"]["title"] = true;
        filter["data"]["monitors"][0]["locationStop"]["properties"]["attributes"]["rbl"] = true;
        filter["data"]["monitors"][0]["lines"][0]["name"] = true;
        filter["data"]["monitors"][0]["lines"][0]["towards"] = true;
        filter["data"]["monitors"][0]["lines"][0]["type"] = true;

        JsonDocument doc;
        if (deserializeJson(doc, payload, DeserializationOption::Filter(filter),
                            DeserializationOption::NestingLimit(20))) continue;

        JsonArray monitors = doc["data"]["monitors"].as<JsonArray>();
        for (JsonObject mon : monitors) {
            String stopName = mon["locationStop"]["properties"]["title"].as<String>();
            String rbl = String((int)mon["locationStop"]["properties"]["attributes"]["rbl"]);
            JsonArray lines = mon["lines"].as<JsonArray>();
            for (JsonObject line : lines) {
                FoundLine fl;
                fl.rbl = rbl;
                fl.lineName = line["name"].as<String>();
                fl.towards = line["towards"].as<String>();
                fl.type = line["type"].as<String>();
                fl.stopName = stopName;
                bool dup = false;
                for (auto& r : results) {
                    if (r.rbl == fl.rbl && r.lineName == fl.lineName && r.towards == fl.towards) {
                        dup = true; break;
                    }
                }
                if (!dup) {
                    results.push_back(fl);
                    cacheDirEntry(fl.rbl, fl.lineName, fl.towards, fl.type, fl.stopName);
                }
            }
        }
    }
    if (!results.empty()) saveDirCache();
    return results;
}

// ── HTML Templates ───────────────────────────────────────────────────
const char HTML_HEAD[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1'>
<title>LineTracker</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Helvetica Neue',Helvetica,Arial,sans-serif;background:#0d0d0d;color:#f0f0f0;padding:0 14px 32px;max-width:480px;margin:0 auto;-webkit-font-smoothing:antialiased;line-height:1.5}
.hdr{display:flex;align-items:center;justify-content:space-between;padding:20px 0 14px}
.hdr h1{font-size:1.5em;font-weight:700;letter-spacing:2px;color:#f5c400;font-family:'Courier New',Courier,monospace}
.hdr a{color:#686868;font-size:22px;text-decoration:none;padding:6px;-webkit-tap-highlight-color:transparent}
.hdr a:hover{color:#f5c400}
.card{background:#171717;border:1px solid rgba(255,255,255,.1);border-radius:10px;padding:16px;margin-bottom:12px;box-shadow:0 4px 24px rgba(0,0,0,.6),inset 0 1px 0 rgba(255,255,255,.04)}
details>summary{cursor:pointer;list-style:none;padding:2px 0;display:flex;align-items:center;justify-content:space-between;user-select:none;-webkit-tap-highlight-color:transparent}
details>summary::-webkit-details-marker{display:none}
details>summary::after{content:'›';color:#686868;font-size:20px;line-height:1;transition:transform .2s;flex-shrink:0;margin-left:8px}
details[open]>summary::after{transform:rotate(90deg)}
details>div{margin-top:10px}
h2{color:#f5c400;font-size:.75em;letter-spacing:1.5px;text-transform:uppercase;font-weight:700;flex:1}
.count{font-family:'Courier New',Courier,monospace;font-size:.7em;color:#686868;margin-left:6px;font-weight:400}
.line-item{display:flex;align-items:center;gap:10px;padding:11px 0;border-bottom:1px solid rgba(255,255,255,.05)}
.line-item:last-child{border-bottom:none}
.badge{font-family:'Courier New',Courier,monospace;display:inline-flex;align-items:center;justify-content:center;padding:5px 10px;border-radius:4px;color:#0d0d0d;font-weight:700;min-width:44px;text-align:center;font-size:13px;letter-spacing:.5px;flex-shrink:0}
.badge.metro{background:#f5c400}
.badge.tram{background:#e53935;color:#fff}
.badge.bus{background:#4a90e2;color:#fff}
.badge.train{background:#4a90e2;color:#fff}
.badge.unknown{background:#444;color:#fff}
.dir{flex:1;font-size:14px;line-height:1.4;min-width:0}
.stop{font-size:11px;color:#686868;margin-top:2px;font-family:'Courier New',Courier,monospace}
input[type=text],input[type=number]{width:100%;padding:12px 14px;border-radius:6px;border:1px solid rgba(255,255,255,.1);background:#1f1f1f;color:#f0f0f0;font-size:16px;font-family:inherit;transition:border .15s,box-shadow .15s}
input[type=text]:focus,input[type=number]:focus{outline:none;border-color:#f5c400;box-shadow:0 0 0 2px rgba(245,196,0,.2)}
input[type=number]{width:70px;padding:10px;text-align:center}
input[type=range]{width:100%;margin:8px 0;accent-color:#f5c400;cursor:pointer}
input[type=checkbox]{width:20px;height:20px;accent-color:#f5c400;cursor:pointer;flex-shrink:0}
select{padding:10px 12px;border-radius:6px;border:1px solid rgba(255,255,255,.1);background:#1f1f1f;color:#f0f0f0;font-size:14px;font-family:inherit}
button{background:#f5c400;color:#0d0d0d;border:none;padding:13px 20px;border-radius:6px;font-size:14px;font-weight:700;font-family:inherit;cursor:pointer;width:100%;margin-top:10px;letter-spacing:.5px;transition:opacity .15s,transform .1s;-webkit-tap-highlight-color:transparent}
button:hover{opacity:.88;transform:translateY(-1px)}
button:active{transform:scale(.98);opacity:.8}
button.add{background:#2e7d32;color:#fff}
.walk{display:inline-flex;align-items:center;gap:4px;font-size:11px;color:#686868;flex-shrink:0;margin-right:2px}
.walk input[type=number]{width:38px;padding:6px 4px;text-align:center;font-size:12px;background:#181818;border:1px solid rgba(245,196,0,.18);color:#f5c400;font-family:'Courier New',Courier,monospace}
.walk input[type=number].on{border-color:rgba(245,196,0,.6);background:#221b00}
.walk .lbl{font-size:10px;line-height:1;letter-spacing:.5px;text-transform:uppercase}
.rm{background:transparent;border:1px solid rgba(229,57,53,.5);color:#e53935;width:34px;min-width:34px;height:34px;padding:0;font-size:14px;margin:0;border-radius:4px;display:inline-flex;align-items:center;justify-content:center;flex-shrink:0;transition:background .15s,color .15s,border-color .15s}
.spinner{display:inline-block;width:14px;height:14px;border:2px solid rgba(245,196,0,.25);border-top-color:#f5c400;border-radius:50%;animation:spin .7s linear infinite;vertical-align:middle;margin-right:6px}
@keyframes spin{to{transform:rotate(360deg)}}
.live-page{margin-bottom:12px}
.live-page:last-child{margin-bottom:0}
.live-page-label{font-size:10px;letter-spacing:1px;text-transform:uppercase;color:#686868;margin:2px 0 4px;border-bottom:1px solid #2a2a2a;padding-bottom:3px}
.live-clock{font-family:'Courier New',Courier,monospace;font-weight:700;color:#f5c400;font-size:26px;text-align:center;padding:10px 0}
.live-row{display:flex;align-items:center;gap:10px;padding:7px 0;border-bottom:1px solid #1c1c1c}
.live-row:last-child{border-bottom:0}
.live-line{font-family:'Courier New',Courier,monospace;font-weight:700;color:#f5c400;min-width:44px;flex-shrink:0}
.live-to{flex:1;color:#cfcfcf;font-size:13px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.live-cd{font-family:'Courier New',Courier,monospace;font-weight:700;color:#f5c400;min-width:38px;text-align:right;flex-shrink:0}
.live-cd.now{color:#ff5252}
.reorder{display:flex;flex-direction:column;gap:2px;flex-shrink:0;margin-right:2px}
.ord{width:38px;height:21px;min-width:38px;padding:0;margin:0;line-height:1;font-size:12px;background:#181818;border:1px solid rgba(245,196,0,.35);color:#f5c400;border-radius:4px;cursor:pointer;display:flex;align-items:center;justify-content:center;-webkit-tap-highlight-color:transparent;touch-action:manipulation}
.ord:active:not(:disabled){background:#332900}
.ord:disabled{opacity:.3;cursor:default}
.rm:hover{background:#e53935;color:#fff;border-color:#e53935}
.btn-secondary{background:#252525;color:#ccc;font-weight:500;border:1px solid rgba(255,255,255,.1)}
.btn-secondary:hover{background:#2e2e2e;opacity:1;transform:none}
.btn-warn{background:#e65100;color:#fff}
.btn-danger{background:#e53935;color:#fff}
.btn-info{background:#1a3a5c;color:#4a90e2;border:1px solid rgba(74,144,226,.3)}
.btn-back{background:none;color:#686868;border:none;width:auto;padding:4px 0;font-size:13px;margin:0 0 16px;font-weight:400;letter-spacing:0}
.btn-back:hover{color:#f5c400;transform:none;opacity:1}
.status{text-align:center;color:#686868;padding:20px 8px;font-size:13px;line-height:1.6}
.msg{background:#1b5e20;border-radius:6px;padding:12px;margin:8px 0;text-align:center;font-weight:600;font-size:14px}
.setting{margin-top:12px}
.setting-label{font-size:12px;color:#686868;display:flex;align-items:center;justify-content:space-between;margin-bottom:4px}
.setting-label b{color:#f0f0f0;font-family:'Courier New',Courier,monospace}
.info-text{font-size:13px;color:#686868;margin-bottom:8px}
.info-text b{color:#f0f0f0}
.hint{font-size:11px;color:#444;margin-bottom:8px}
.check-label{font-size:13px;color:#888;display:flex;align-items:center;gap:8px}
.check-label input{width:auto;flex-shrink:0}
.footer{text-align:center;margin:20px 0 8px;font-size:11px;color:#444;line-height:1.8}
.footer b{color:#f5c400;font-family:'Courier New',Courier,monospace}
.footer a{color:#444}
.toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);background:#2e7d32;color:#fff;padding:10px 22px;border-radius:6px;font-weight:600;font-size:13px;z-index:999;white-space:nowrap;animation:tin .2s,tout .35s 2.1s forwards}
@keyframes tin{from{opacity:0;transform:translateX(-50%) translateY(6px)}to{opacity:1;transform:translateX(-50%)}}
@keyframes tout{to{opacity:0}}
.time-row{display:flex;gap:8px;margin:8px 0;font-size:13px;color:#686868;align-items:center}
.options-col{margin-top:12px;display:flex;flex-direction:column;gap:8px}
.stn-count{color:#686868;font-size:11px;margin:3px 0}
.letters{display:flex;flex-wrap:wrap;gap:5px;justify-content:center;margin:10px 0}
.letters a{display:inline-flex;align-items:center;justify-content:center;width:34px;height:34px;background:#1f1f1f;color:#f5c400;border-radius:4px;text-decoration:none;font-weight:700;font-size:13px;font-family:'Courier New',Courier,monospace;border:1px solid rgba(255,255,255,.08);transition:all .15s;-webkit-tap-highlight-color:transparent}
.letters a:hover,.letters a.act{background:#f5c400;color:#0d0d0d;border-color:#f5c400}
.stn{display:block;padding:12px 8px;border-bottom:1px solid rgba(255,255,255,.04);color:#f0f0f0;text-decoration:none;font-size:14px;transition:all .15s;border-radius:4px}
.stn:hover{background:rgba(245,196,0,.06);padding-left:14px}
.search-row{display:flex;gap:8px}
.search-row input{flex:1}
.search-row button{width:auto;margin:0;padding:0 18px}
.sep{border:none;border-top:1px solid rgba(255,255,255,.06);margin:4px 0}
@media(max-width:380px){
body{padding:0 10px 24px}
.card{padding:12px}
button{padding:11px 14px;font-size:13px}
.badge{min-width:36px;font-size:12px}
.hdr h1{font-size:1.2em}
}
</style></head><body>
)rawliteral";

String badgeClassForType(const String& type) {
    if (type == "ptMetro") return "badge metro";
    if (type == "ptTram")  return "badge tram";
    if (type.startsWith("ptBus")) return "badge bus";
    if (type == "ptTrainS") return "badge train";
    return "badge unknown";
}

// Official Vienna U-Bahn colors as an inline-style override for a badge.
// Only active when cfgLineColors is on; returns "" otherwise (badge keeps its
// default class color). Trams/buses already have their own badge colors.
String badgeColorStyle(const String& name, const String& type) {
    if (!cfgLineColors || type != "ptMetro") return "";
    if (name == "U1") return " style='background:#E20613;color:#fff'";
    if (name == "U2") return " style='background:#9C4F9F;color:#fff'";
    if (name == "U3") return " style='background:#EF7C00;color:#0d0d0d'";
    if (name == "U4") return " style='background:#00A64F;color:#fff'";
    if (name == "U6") return " style='background:#9C6B30;color:#fff'";
    return "";
}

void sendHtml(const String& html) {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send(200, "text/html; charset=utf-8", html);
}

void handleSettingsPage() {
    LOG_REQ();
    String html = FPSTR(HTML_HEAD);
    html += "<div class='hdr'><a href='/' class='btn-back' style='font-size:20px;color:#686868;text-decoration:none'>&#8592;</a><h1 style='font-size:1.1em;letter-spacing:1px'>EINSTELLUNGEN</h1><span style='width:34px'></span></div>";

    // ── Display settings ──────────────────────────────────────────────────
    html += "<div class='card'>";
    html += "<form action='/settings' method='POST'>";

    html += "<div class='setting'><div class='setting-label'>Seitenwechsel <b id='v_rot'>" + String(cfgRotateSec) + " Sek</b></div>";
    html += "<input type='range' name='rotate_sec' min='2' max='30' value='" + String(cfgRotateSec) + "' oninput=\"document.getElementById('v_rot').textContent=this.value+' Sek'\"></div>";

    html += "<div class='setting'><div class='setting-label'>Helligkeit <b id='v_bri'>" + String(cfgBrightness * 100 / 255) + "%</b></div>";
    html += "<input type='range' name='brightness' min='10' max='255' value='" + String(cfgBrightness) + "' oninput=\"document.getElementById('v_bri').textContent=Math.round(this.value*100/255)+'%'\"></div>";

    {
        String nightFromVal = (cfgNightFrom >= 0) ? String(cfgNightFrom) : "22";
        String nightToVal   = (cfgNightTo   >= 0) ? String(cfgNightTo)   : "7";
        String nightBrVal   = String(cfgNightBright * 100 / 255);
        bool nightOn = (cfgNightFrom >= 0);
        html += "<div class='setting'>";
        html += "<label class='check-label'><input type='checkbox' name='night_on' value='1' id='cb_night'";
        if (nightOn) html += " checked";
        html += " onchange=\"document.getElementById('night_opts').style.display=this.checked?'block':'none'\">Nachtmodus</label></div>";
        html += "<div id='night_opts' style='display:" + String(nightOn ? "block" : "none") + "'>";
        html += "<div class='time-row'><span>Von</span><input type='number' name='night_from' min='0' max='23' value='" + nightFromVal + "'>";
        html += "<span>bis</span><input type='number' name='night_to' min='0' max='23' value='" + nightToVal + "'><span>Uhr</span></div>";
        html += "<div class='setting-label'>Nacht-Helligkeit <b id='v_nbri'>" + nightBrVal + "%</b></div>";
        html += "<input type='range' name='night_bright' min='0' max='100' value='" + nightBrVal + "' oninput=\"document.getElementById('v_nbri').textContent=this.value+'%'\">";
        html += "</div>";
    }

    // ── Standby (display fully off) ──
    {
        String sbFromVal = (cfgStandbyFrom >= 0) ? String(cfgStandbyFrom) : "0";
        String sbToVal   = (cfgStandbyTo   >= 0) ? String(cfgStandbyTo)   : "6";
        bool sbOn = (cfgStandbyFrom >= 0);
        html += "<div class='setting'>";
        html += "<label class='check-label'><input type='checkbox' name='standby_on' value='1' id='cb_sb'";
        if (sbOn) html += " checked";
        html += " onchange=\"document.getElementById('sb_opts').style.display=this.checked?'block':'none'\">Standby &ndash; Display ganz aus</label></div>";
        html += "<div id='sb_opts' style='display:" + String(sbOn ? "block" : "none") + "'>";
        html += "<div class='time-row'><span>Von</span><input type='number' name='standby_from' min='0' max='23' value='" + sbFromVal + "'>";
        html += "<span>bis</span><input type='number' name='standby_to' min='0' max='23' value='" + sbToVal + "'><span>Uhr</span></div>";
        // Deep sleep (experimental) — requires explicit risk confirmation
        html += "<label class='check-label' style='margin-top:8px'><input type='checkbox' name='standby_deep' value='1' id='cb_deep'";
        if (cfgStandbyDeepSleep) html += " checked";
        html += " onchange=\"document.getElementById('deep_warn').style.display=this.checked?'block':'none'\">Deep Sleep (experimentell)</label>";
        html += "<div id='deep_warn' style='display:" + String(cfgStandbyDeepSleep ? "block" : "none") + ";background:#3a1212;border:1px solid #a33;border-radius:8px;padding:10px;margin-top:6px'>";
        html += "<p style='color:#f99;font-size:12px;margin:0 0 8px'><b>Achtung:</b> Im Deep Sleep ist das Ger&auml;t NICHT per WLAN/Web erreichbar, bis das Zeitfenster endet. Bei falscher Zeit oder Konfiguration kann das Display unerreichbar bleiben &ndash; im Notfall ist ein Laptop zum Neu-Flashen n&ouml;tig. Nur f&uuml;r erfahrene Nutzer.</p>";
        html += "<label class='check-label' style='color:#f99'><input type='checkbox' name='standby_deep_ack' value='1'";
        if (cfgStandbyDeepSleep) html += " checked";
        html += ">Ich verstehe das Risiko</label></div>";
        html += "</div>";
    }

    // ── Weekend schedule (separate night/standby for Sat/Sun) ──
    {
        bool weOn = cfgWeekendSchedule;
        String wnFrom = (cfgNightFromWe   >= 0) ? String(cfgNightFromWe)   : "23";
        String wnTo   = (cfgNightToWe     >= 0) ? String(cfgNightToWe)     : "9";
        String wsFrom = (cfgStandbyFromWe >= 0) ? String(cfgStandbyFromWe) : "0";
        String wsTo   = (cfgStandbyToWe   >= 0) ? String(cfgStandbyToWe)   : "9";
        html += "<div class='setting'>";
        html += "<label class='check-label'><input type='checkbox' name='weekend_sched' value='1' id='cb_we'";
        if (weOn) html += " checked";
        html += " onchange=\"document.getElementById('we_opts').style.display=this.checked?'block':'none'\">Eigener Wochenend-Zeitplan (Sa/So)</label></div>";
        html += "<div id='we_opts' style='display:" + String(weOn ? "block" : "none") + "'>";
        html += "<p class='hint' style='margin:4px 0'>Nachtmodus Sa/So</p>";
        html += "<div class='time-row'><span>Von</span><input type='number' name='night_from_we' min='0' max='23' value='" + wnFrom + "'>";
        html += "<span>bis</span><input type='number' name='night_to_we' min='0' max='23' value='" + wnTo + "'><span>Uhr</span></div>";
        html += "<p class='hint' style='margin:4px 0'>Standby Sa/So</p>";
        html += "<div class='time-row'><span>Von</span><input type='number' name='standby_from_we' min='0' max='23' value='" + wsFrom + "'>";
        html += "<span>bis</span><input type='number' name='standby_to_we' min='0' max='23' value='" + wsTo + "'><span>Uhr</span></div>";
        html += "</div>";
    }

    html += "<div class='options-col'>";
    html += "<label class='check-label'><input type='checkbox' name='show_next' value='1'";
    if (cfgShowNext) html += " checked";
    html += ">N&auml;chste Abfahrt anzeigen</label>";
    html += "<label class='check-label'><input type='checkbox' name='show_disruptions' value='1'";
    if (cfgShowDisruptions) html += " checked";
    html += ">St&ouml;rungsticker anzeigen</label>";
    html += "<label class='check-label'><input type='checkbox' name='show_clock' value='1'";
    if (cfgShowClock) html += " checked";
    html += ">Uhr als eigene Seite anzeigen</label>";
    html += "<label class='check-label'><input type='checkbox' name='show_weather' value='1'";
    if (cfgShowWeather) html += " checked";
    html += ">Wetter als eigene Seite anzeigen</label>";
    html += "<label class='check-label'><input type='checkbox' name='line_colors' value='1'";
    if (cfgLineColors) html += " checked";
    html += ">U-Bahn-Linienfarben</label>";
    html += "<label class='check-label'><input type='checkbox' name='sort_by_time' value='1'";
    if (cfgSortByTime) html += " checked";
    html += ">Display nach Zeit sortieren</label>";
    html += "<label class='check-label'><input type='checkbox' name='beta_channel' value='1'";
    if (cfgBetaChannel) html += " checked";
    html += ">Beta-Updates (Testkanal)</label>";
    html += "</div>";

    html += "<button type='submit' style='margin-top:16px'>Speichern</button>";
    html += "</form></div>";

    // ── Easter Eggs ───────────────────────────────────────────────────────
    html += "<div class='card'>";
    html += "<h2 style='margin-bottom:12px'>Spiele</h2>";
    html += "<p class='hint' style='margin-bottom:10px'>Auf einem zweiten Gerät öffnen — das Display übernimmt das Spiel.</p>";
    html += "<a href='/pong'><button class='btn-secondary' style='margin-bottom:8px;text-align:left;display:flex;align-items:center;gap:10px;justify-content:flex-start'>"
            "<span style='font-size:18px'>&#127918;</span><span>Pong <span style='color:#686868;font-size:11px;font-weight:400'>&middot; 2 Spieler</span></span></button></a>";
    html += "<a href='/snake'><button class='btn-secondary' style='text-align:left;display:flex;align-items:center;gap:10px;justify-content:flex-start'>"
            "<span style='font-size:18px'>&#128012;</span><span>Snake <span style='color:#686868;font-size:11px;font-weight:400'>&middot; 1 Spieler &middot; U-Bahn</span></span></button></a>";
    html += "</div>";

    // ── System ────────────────────────────────────────────────────────────
    html += "<div class='card'>";
    html += "<h2 style='margin-bottom:12px'>System</h2>";

    html += "<p class='info-text'>WLAN: <b>" + WiFi.SSID() + "</b></p>";
    html += "<p class='hint'>Nur 2,4 GHz Netzwerke werden unterst&uuml;tzt.</p>";
    html += "<form action='/wifi-reset' method='POST' onsubmit=\"return confirm('WLAN wirklich \\u00e4ndern?')\">";
    html += "<button class='btn-warn'>WLAN &auml;ndern</button></form>";

    html += "<hr class='sep' style='margin:14px 0'>";
    html += "<p class='info-text'>Version: <b>v" + String(FW_VERSION) + "</b></p>";
    html += "<a href='/update'><button class='btn-info'>Nach Update suchen</button></a>";

    html += "<hr class='sep' style='margin:14px 0'>";
    html += "<form action='/factory-reset' method='POST' onsubmit=\"return confirm('Wirklich alles l\\u00f6schen?')\">";
    html += "<button class='btn-danger'>Werksreset</button></form>";

    html += "</div>";
    html += "</body></html>";
    sendHtml(html);
}

void handleRoot() {
    LOG_REQ();
    unsigned long t0 = millis();
    String html = FPSTR(HTML_HEAD);
    logf("[root] HTML_HEAD: %lums\n", millis()-t0);

    // Header with gear icon
    html += "<div class='hdr'><h1>LINETRACKER</h1><a href='/settings'>&#9881;</a></div>";

    if (server.hasArg("saved")) {
        html += "<div class='toast'>Gespeichert</div>"
                "<script>setTimeout(function(){var t=document.querySelector('.toast');if(t)t.remove()},2500);"
                "history.replaceState(null,'','/');</script>";
    }

    // ── Live preview card (mirrors the display, read-only) ────────────────
    html += "<div class='card'><h2 style='margin-bottom:10px'>Aktuell am Display</h2>";
    html += "<div id='live'><p class='status'>Lade&hellip;</p></div></div>";

    // ── Search card ──────────────────────────────────────────────────────
    html += "<div class='card'>";
    html += "<form id='searchForm' action='/search' method='GET' class='search-row'>";
    html += "<input type='text' name='q' placeholder='Station suchen...' autofocus>";
    html += "<button type='submit' style='width:auto;margin:0;padding:0 16px'>&#8594;</button>";
    html += "</form>";
    html += "<a href='/browse'><button class='btn-secondary' style='margin-top:8px;font-size:12px;padding:10px'>Alle Stationen</button></a>";
    html += "</div>";
    html += "<div id='searchResults'></div>";

    // ── WL Active Lines ──────────────────────────────────────────────────
    {
        int n = cfgLines.size();
        html += "<div class='card'><details" + String(n > 0 ? " open" : "") + "><summary>";
        html += "<h2>Wiener Linien<span class='count'>(" + String(n) + ")</span></h2></summary><div>";
        if (n == 0) {
            html += "<p class='status'>Noch keine Linien. Station oben suchen.</p>";
        } else {
            for (size_t i = 0; i < cfgLines.size(); i++) {
                auto& cl = cfgLines[i];
                html += "<div class='line-item'>";
                html += "<span class='" + badgeClassForType(cl.type) + "'" + badgeColorStyle(cl.name, cl.type) + ">" + cl.name + "</span>";
                html += "<div class='dir'>" + cl.towards + "</div>";
                html += "<label class='walk' title='Gehweg zum Stop in Minuten \\u2014 wenn Countdown \\u2264 dieser Wert, blinkt die Abfahrt (Zeit zum Losgehen)'><span class='lbl'>Gehw</span>";
                html += "<input type='number' min='0' max='30' value='" + String(cl.walkMin) +
                        "' class='" + String(cl.walkMin > 0 ? "on" : "") +
                        "' data-kind='wl' data-idx='" + String(i) + "' onchange='setWalk(this)'></label>";
                {
                    html += "<div class='reorder'>";
                    html += "<button type='button' class='ord' onclick='moveLine(" + String(i) + ",-1)'" +
                            (i == 0 ? " disabled" : "") + ">&#9650;</button>";
                    html += "<button type='button' class='ord' onclick='moveLine(" + String(i) + ",1)'" +
                            (i == (size_t)(n - 1) ? " disabled" : "") + ">&#9660;</button>";
                    html += "</div>";
                }
                html += "<form action='/remove' method='POST' style='margin:0'>";
                html += "<input type='hidden' name='idx' value='" + String(i) + "'>";
                html += "<button class='rm' type='button' onclick='confirmRm(this)'>&#x2715;</button></form>";
                html += "</div>";
            }
        }
        html += "</div></details></div>";
    }
    logf("[root] wl lines: %lums\n", millis()-t0);

    // ── ÖBB ─────────────────────────────────────────────────────────────
    {
        int n = cfgOebb.size();
        html += "<div class='card'><details" + String(n > 0 ? " open" : "") + "><summary>";
        html += "<h2>S-Bahn &amp; Z&uuml;ge<span class='count'>(" + String(n) + ")</span></h2></summary><div>";
        for (size_t i = 0; i < cfgOebb.size(); i++) {
            auto& os = cfgOebb[i];
            html += "<div class='line-item'>";
            html += "<span class='badge train'>" + os.line + "</span>";
            html += "<div style='flex:1;min-width:0'><div class='dir'>" + os.towards + "</div>";
            html += "<div class='stop'>" + os.stationName + "</div></div>";
            html += "<label class='walk' title='Gehweg zur Station in Minuten'><span class='lbl'>Gehw</span>";
            html += "<input type='number' min='0' max='30' value='" + String(os.walkMin) +
                    "' class='" + String(os.walkMin > 0 ? "on" : "") +
                    "' data-kind='oebb' data-idx='" + String(i) + "' onchange='setWalk(this)'></label>";
            html += "<form action='/oebb-remove' method='POST' style='margin:0'>";
            html += "<input type='hidden' name='idx' value='" + String(i) + "'>";
            html += "<button class='rm' type='button' onclick='confirmRm(this)'>&#x2715;</button></form>";
            html += "</div>";
        }
        html += "<form action='/oebb-search' method='GET' class='search-row' style='margin-top:" + String(n > 0 ? "10" : "0") + "px'>";
        html += "<input type='text' name='q' placeholder='ÖBB-Station suchen...'>";
        html += "<button type='submit' class='add' style='width:auto;margin:0;padding:0 14px'>&#8594;</button>";
        html += "</form>";
        html += "</div></details></div>";
    }
    logf("[root] oebb card: %lums\n", millis()-t0);

    // ── Watch Groups ─────────────────────────────────────────────────────
    {
        int n = cfgWatchGroups.size();
        html += "<div class='card'><details" + String(n > 0 ? " open" : "") + "><summary>";
        html += "<h2>Watch-Gruppen<span class='count'>(" + String(n) + ")</span></h2></summary><div>";
        if (n == 0) {
            html += "<p class='status'>Keine Gruppen.<br>In den Suchergebnissen 'Als Gruppe speichern' w&auml;hlen.</p>";
        } else {
            for (size_t i = 0; i < cfgWatchGroups.size(); i++) {
                auto& g = cfgWatchGroups[i];
                bool hasOebb = false;
                for (auto& e : g.entries) if (e.source == "oebb") { hasOebb = true; break; }
                html += "<div class='line-item'>";
                html += "<span class='" + String(hasOebb ? "badge train" : "badge metro") + "'>" + String(g.entries.size()) + "</span>";
                html += "<div><div class='dir'><a href='/watch-group-edit?idx=" + String(i) + "' style='color:#f5c400;text-decoration:none'>" + g.label + "</a></div>";
                html += "<div class='stop'>n&auml;chste " + String(g.maxDepartures) + " Abfahrten</div></div>";
                html += "<form action='/watch-group-remove' method='POST' style='margin:0'>";
                html += "<input type='hidden' name='idx' value='" + String(i) + "'>";
                html += "<button class='rm' type='button' onclick='confirmRm(this)'>&#x2715;</button></form>";
                html += "</div>";
            }
        }
        html += "</div></details></div>";
    }
    logf("[root] watch card: %lums\n", millis()-t0);

    html += "<div class='footer'><b>LineTracker</b> &middot; Leo Blum<br>"
            "<a href='https://data.wien.gv.at' style='color:#444'>data.wien.gv.at</a>"
            " &middot; <a href='https://fahrplan.oebb.at' style='color:#444'>&Ouml;BB/SCOTTY</a>"
            "<br><a href='/pong' style='color:#666'>&#127918; Pong</a> &middot; "
            "<a href='/snake' style='color:#666'>&#128012; Snake</a></div>";

    html += "<script>function confirmRm(b){if(b._c){clearTimeout(b._t);b.closest('form').submit();return}"
            "b._c=1;b.innerHTML='?';b.style.background='#e53935';b.style.borderColor='#e53935';b.style.color='#fff';"
            "b._t=setTimeout(function(){b._c=0;b.innerHTML='&#x2715;';b.style.background='';b.style.borderColor='';b.style.color=''},3000)}\n"
            "function setWalk(el){var m=parseInt(el.value,10);if(isNaN(m)||m<0)m=0;if(m>30)m=30;el.value=m;"
            "el.classList.toggle('on',m>0);"
            "var b=new URLSearchParams({kind:el.dataset.kind,idx:el.dataset.idx,sub:(el.dataset.sub||''),min:m});"
            "fetch('/set-walk',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b.toString()})"
            ".then(function(r){if(!r.ok)el.style.borderColor='#e53935';else{el.style.borderColor='';"
            "var t=document.createElement('div');t.className='toast';t.textContent=m>0?('Gehweg '+m+' min'):'Gehweg aus';"
            "document.body.appendChild(t);setTimeout(function(){t.remove()},2500);}}).catch(function(){el.style.borderColor='#e53935'})}"
            "function moveLine(i,d){var b=new URLSearchParams({idx:i,dir:d});"
            "fetch('/move-line',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b.toString()})"
            ".then(function(r){if(r.ok)location.reload()})}"
            "function liveRow(s){var row=document.createElement('div');row.className='live-row';"
            "var ln=document.createElement('span');ln.className='live-line';ln.textContent=s.line;"
            "var to=document.createElement('span');to.className='live-to';to.textContent=s.to;"
            "var cd=document.createElement('span');cd.className='live-cd';"
            "if(s.cd===0){cd.textContent='\\u25A0';}else if(s.walk>0&&s.cd<=s.walk){cd.textContent='jetzt';cd.classList.add('now');}else{cd.textContent=s.cd+\"'\";}"
            "row.appendChild(ln);row.appendChild(to);row.appendChild(cd);return row;}"
            "function liveLabel(t){var l=document.createElement('div');l.className='live-page-label';l.textContent=t;return l;}"
            "function renderLive(d){var e=document.getElementById('live');if(!e)return;"
            "var slots=d.slots||[],rows=d.rows||3;"
            "if(!slots.length&&!d.clock){e.innerHTML=\"<p class='status'>Keine Abfahrten</p>\";return;}"
            "e.innerHTML='';"
            "var nPages=Math.ceil(slots.length/rows),total=nPages+(d.clock?1:0),labeled=total>1;"
            "for(var p=0;p<nPages;p++){var pg=document.createElement('div');pg.className='live-page';"
            "if(labeled)pg.appendChild(liveLabel('Seite '+(p+1)));"
            "for(var k=p*rows;k<Math.min((p+1)*rows,slots.length);k++)pg.appendChild(liveRow(slots[k]));"
            "e.appendChild(pg);}"
            "if(d.clock){var cp=document.createElement('div');cp.className='live-page';"
            "if(labeled)cp.appendChild(liveLabel('Uhr'));"
            "var cr=document.createElement('div');cr.className='live-clock';var nw=new Date();"
            "cr.textContent=('0'+nw.getHours()).slice(-2)+':'+('0'+nw.getMinutes()).slice(-2);"
            "cp.appendChild(cr);e.appendChild(cp);}}"
            "function pollLive(){fetch('/api/now').then(function(r){return r.json()}).then(renderLive).catch(function(){})}"
            "if(document.getElementById('live')){pollLive();setInterval(pollLive,5000);}"
            "var sf=document.getElementById('searchForm');"
            "if(sf){sf.addEventListener('submit',function(ev){ev.preventDefault();"
            "var q=sf.q.value.trim();var rc=document.getElementById('searchResults');"
            "if(q.length<2){rc.innerHTML=\"<div class='card'><p class='status'>Mindestens 2 Zeichen eingeben.</p></div>\";return;}"
            "rc.innerHTML=\"<div class='card'><p class='status'><span class='spinner'></span>Suche&hellip;</p></div>\";"
            "fetch('/search?frag=1&q='+encodeURIComponent(q)).then(function(r){return r.text()}).then(function(t){rc.innerHTML=t;})"
            ".catch(function(){rc.innerHTML=\"<div class='card'><p class='status'>Fehler bei der Suche.</p></div>\";});});}"
            "</script>";
    html += "</body></html>";
    logf("[root] html built: %lums  size: %u bytes\n", millis()-t0, html.length());
    sendHtml(html);
    logf("[root] sendHtml done: %lums\n", millis()-t0);
}

void handleSearch() {
    LOG_REQ();
    String query = server.arg("q");
    bool frag = server.hasArg("frag");  // AJAX: return only the result card, no page chrome
    auto finish = [&](const String& inner) {
        if (frag) { sendHtml(inner); return; }
        String h = FPSTR(HTML_HEAD);
        h += inner;
        h += "</body></html>";
        sendHtml(h);
    };
    if (query.length() < 2) {
        if (frag) { sendHtml(String("<div class='card'><p class='status'>Mindestens 2 Zeichen eingeben.</p></div>")); return; }
        server.sendHeader("Location", "/");
        server.send(302);
        return;
    }

    unsigned long t0 = millis();
    auto stations = searchStations(query);
    logf("[search] q='%s' csv scan: %lums, stations: %d\n", query.c_str(), millis()-t0, stations.size());

    if (stations.empty()) {
        String inner = "<div class='card'><p class='status'>Keine Station gefunden für:<br><b style='color:#e8e8f0'>" + query + "</b></p>";
        inner += "<p class='hint' style='text-align:center'>Tipp: Versuche einen kürzeren Suchbegriff oder <a href='/browse' style='color:#ffbf00'>blättere durch alle Stationen</a>.</p>";
        if (!frag) inner += "<a href='/'><button>Zurück</button></a>";
        inner += "</div>";
        finish(inner);
        return;
    }

    // Hybrid search: static CSV data + dirCache + live API probe
    // steige.csv only lists ONE line per platform, but multiple lines share platforms
    // dirCache (from live departures) and API probe discover the real line set

    struct SearchResult {
        String rbl;
        String lineName;
        String towards;
        String type;
    };
    std::vector<SearchResult> results;
    std::map<String, bool> seenLineDir;  // dedupe by lineName|towards

    // Collect all RBLs — read steige.csv once for all matched stations
    unsigned long t1 = millis();
    std::vector<String> haltIds;
    for (auto& st : stations) haltIds.push_back(st.first);
    auto steigeByHalt = findSteigeForStations(haltIds);
    std::vector<SteigeInfo> allSteige;
    for (auto& kv : steigeByHalt)
        for (auto& s : kv.second) {
            bool dup = false;
            for (auto& a : allSteige) { if (a.rbl == s.rbl && a.linienId == s.linienId) { dup = true; break; } }
            if (!dup) allSteige.push_back(s);
        }
    logf("[search] steige scan: %lums, rbls: %d\n", millis()-t1, allSteige.size());

    // Layer 1: Add lines from dirCache (most accurate — from live departures)
    // uncachedRbls: RBLs not in dirCache AND not resolvable by lineDirMap (truly unknown)
    std::vector<String> uncachedRbls;
    for (auto& si : allSteige) {
        auto it = dirCache.find(si.rbl);
        if (it != dirCache.end()) {
            for (auto& fl : it->second) {
                String key = fl.lineName + "|" + si.richtung;
                if (seenLineDir.find(key) == seenLineDir.end()) {
                    seenLineDir[key] = true;
                    results.push_back({si.rbl, fl.lineName, fl.towards, fl.type});
                }
            }
        } else if (lineDirMap.find(si.linienId) == lineDirMap.end()) {
            // Not in dirCache and lineDirMap can't resolve it either → probe live API
            uncachedRbls.push_back(si.rbl);
        }
    }

    // Layer 2: Add lines from static lineDirMap (for lines not yet in dirCache)
    for (auto& si : allSteige) {
        auto it = lineDirMap.find(si.linienId);
        if (it == lineDirMap.end()) continue;
        String towards = (si.richtung == "H") ? it->second.terminusH : it->second.terminusR;
        if (towards.length() == 0) towards = (si.richtung == "H") ? "Richtung H" : "Richtung R";
        String key = it->second.name + "|" + si.richtung;
        if (seenLineDir.find(key) != seenLineDir.end()) continue;
        seenLineDir[key] = true;
        results.push_back({si.rbl, it->second.name, towards, it->second.type});
    }

    logf("[search] L1+L2: %d results, uncached: %d, lineDirMap: %d\n", results.size(), uncachedRbls.size(), lineDirMap.size());

    // Layer 3: Probe live API for RBLs that neither dirCache nor lineDirMap could resolve.
    // This covers: new lines not yet in the CSV, and first-boot before lineDirMap is built.
    if (!uncachedRbls.empty() && WiFi.status() == WL_CONNECTED) {
        unsigned long t3 = millis();
        auto probed = probeRbls(uncachedRbls);
        logf("[search] L3 probe: %lums, found: %d\n", millis()-t3, probed.size());
        for (auto& fl : probed) {
            String key = fl.lineName + "|" + fl.towards;
            if (seenLineDir.find(key) != seenLineDir.end()) continue;
            seenLineDir[key] = true;
            results.push_back({fl.rbl, fl.lineName, fl.towards, fl.type});
        }
    }

    logf("[search] total: %lums, %d results\n", millis()-t0, results.size());

    // Sort by transport type: U-Bahn → Tram → Bus → Train → other
    std::sort(results.begin(), results.end(), [](const SearchResult& a, const SearchResult& b) {
        int pa = transportPriority(a.type), pb = transportPriority(b.type);
        if (pa != pb) return pa < pb;
        return a.lineName < b.lineName;
    });

    String html = "";
    html += "<div class='card'><h2>Ergebnisse: " + query + "</h2>";

    if (lineDirMap.empty()) {
        html += "<p class='hint' style='color:#ffbf00'>&#9888; Liniendatenbank wird noch geladen &mdash; Ergebnisse m&ouml;glicherweise unvollst&auml;ndig. Seite in ca. 1 Minute neu laden.</p>";
    }

    if (results.empty()) {
        html += "<p class='status'>Keine Linien gefunden</p>";
    } else {
        html += "<form action='/save' method='POST'>";
        for (size_t i = 0; i < results.size(); i++) {
            auto& sr = results[i];
            bool alreadyActive = false;
            for (auto& cl : cfgLines) {
                if (cl.rbl == sr.rbl && cl.name == sr.lineName) { alreadyActive = true; break; }
            }

            String val = sr.rbl + "|" + sr.lineName + "|" + sr.towards + "|" + sr.type;
            html += "<div class='line-item'>";
            html += "<input type='checkbox' name='line' value='" + val + "'";
            if (alreadyActive) html += " checked disabled";
            html += ">";
            html += "<span class='" + badgeClassForType(sr.type) + "'" + badgeColorStyle(sr.lineName, sr.type) + ">" + sr.lineName + "</span>";
            html += "<div class='dir'>" + sr.towards + "</div>";
            html += "</div>";
        }
        html += "<button class='add' type='submit'>Einzeln hinzuf&uuml;gen</button>";
        html += "</form>";

        // ── Watch Group option ────────────────────────────────────────
        html += "<div style='border-top:1px solid rgba(255,255,255,.06);margin-top:16px;padding-top:12px'>";
        html += "<p class='hint' style='margin-bottom:8px'>Oder: Auswahl als Gruppe &ndash; zeigt nur die n&auml;chsten N Abfahrten aller ausgew&auml;hlten Linien zusammen.</p>";

        html += "<form action='/watch-group-save' method='POST'>";
        html += "<input type='hidden' name='source' value='wl'>";
        html += "<input type='hidden' name='label' value='" + query + "'>";
        for (size_t i = 0; i < results.size(); i++) {
            auto& sr = results[i];
            String val = sr.rbl + "|" + sr.lineName + "|" + sr.towards + "|" + sr.type;
            html += "<div class='line-item'>";
            html += "<input type='checkbox' name='line' value='" + val + "'>";
            html += "<span class='" + badgeClassForType(sr.type) + "'" + badgeColorStyle(sr.lineName, sr.type) + ">" + sr.lineName + "</span>";
            html += "<div class='dir'>" + sr.towards + "</div>";
            html += "</div>";
        }
        html += "<div style='display:flex;gap:8px;align-items:center;margin-top:10px'>";
        html += "<label style='font-size:13px;color:#888;white-space:nowrap'>N&auml;chste</label>";
        html += "<input type='number' name='max' min='1' max='5' value='2' style='width:55px;padding:8px;text-align:center'>";
        html += "<label style='font-size:13px;color:#888;white-space:nowrap'>Abfahrten anzeigen</label>";
        html += "<button class='add' type='submit' style='flex:1;margin:0'>Als Gruppe speichern</button>";
        html += "</div></form>";
        html += "</div>";
    }

    if (!frag) html += "<br><a href='/'><button>Zur&uuml;ck</button></a>";
    html += "</div>";
    finish(html);
}

void handleSave() {
    LOG_REQ();
    for (int i = 0; i < server.args(); i++) {
        if (server.argName(i) == "line") {
            String val = server.arg(i);
            // Parse: rbl|name|towards|type
            int p1 = val.indexOf('|');
            int p2 = val.indexOf('|', p1 + 1);
            int p3 = val.indexOf('|', p2 + 1);
            if (p1 < 0 || p2 < 0 || p3 < 0) continue;

            ConfigLine cl;
            cl.rbl     = val.substring(0, p1);
            cl.name    = val.substring(p1 + 1, p2);
            cl.towards = val.substring(p2 + 1, p3);
            cl.type    = val.substring(p3 + 1);

            // Skip duplicates
            bool dup = false;
            for (auto& existing : cfgLines) {
                if (existing.rbl == cl.rbl && existing.name == cl.name) { dup = true; break; }
            }
            if (!dup && cl.rbl.length() > 0) cfgLines.push_back(cl);
        }
    }
    saveConfig();
    configChanged = true;

    server.sendHeader("Location", "/?saved=1");
    server.send(302);
}

void handleRemove() {
    if (server.hasArg("idx")) {
        int idx = server.arg("idx").toInt();
        if (idx >= 0 && idx < (int)cfgLines.size()) {
            cfgLines.erase(cfgLines.begin() + idx);
        }
    }
    saveConfig();
    // Clear stale display data immediately
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    departures.clear();
    displaySlots.clear();
    xSemaphoreGive(dataMutex);
    configChanged = true;
    server.sendHeader("Location", "/");
    server.send(302);
}

// POST /set-walk — body: kind=wl|oebb|wg, idx=N, sub=M (for wg entries), min=0..30
void handleSetWalk() {
    String kind = server.arg("kind");
    int idx     = server.arg("idx").toInt();
    int sub     = server.hasArg("sub") && server.arg("sub").length() > 0 ? server.arg("sub").toInt() : -1;
    int m       = server.arg("min").toInt();
    if (m < 0)  m = 0;
    if (m > 30) m = 30;

    bool ok = false;
    if (kind == "wl") {
        if (idx >= 0 && idx < (int)cfgLines.size()) { cfgLines[idx].walkMin = m; ok = true; }
    } else if (kind == "oebb") {
        if (idx >= 0 && idx < (int)cfgOebb.size())  { cfgOebb[idx].walkMin  = m; ok = true; }
    } else if (kind == "wg") {
        if (idx >= 0 && idx < (int)cfgWatchGroups.size()) {
            auto& g = cfgWatchGroups[idx];
            if (sub >= 0 && sub < (int)g.entries.size()) { g.entries[sub].walkMin = m; ok = true; }
        }
    }

    if (!ok) { server.send(400, "text/plain", "bad target"); return; }
    saveConfig();
    configChanged = true;
    server.send(204);
}

// POST /move-line — body: idx=N, dir=-1|1 — swap a WL line up/down in cfgLines
void handleMoveLine() {
    int idx = server.arg("idx").toInt();
    int dir = server.arg("dir").toInt();
    int j   = idx + (dir < 0 ? -1 : 1);
    if (idx < 0 || idx >= (int)cfgLines.size() || j < 0 || j >= (int)cfgLines.size()) {
        server.send(400, "text/plain", "bad move");
        return;
    }
    std::swap(cfgLines[idx], cfgLines[j]);
    // Manual reordering implies manual order mode — otherwise time-sort would
    // override it and the arrows would appear to do nothing.
    cfgSortByTime = false;
    saveConfig();
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    departures.clear();
    displaySlots.clear();
    xSemaphoreGive(dataMutex);
    configChanged = true;
    server.send(204);
}

// GET /api/now — read-only snapshot of what is currently shown on the display
void handleApiNow() {
    JsonDocument doc;
    JsonArray arr = doc["slots"].to<JsonArray>();
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    for (const Departure& d : displaySlots) {
        JsonObject o = arr.add<JsonObject>();
        o["line"] = d.lineName;
        o["to"]   = d.towards;
        o["cd"]   = d.countdown;
        o["walk"] = d.walkMin;
    }
    doc["err"]   = fetchError;
    doc["rows"]  = MAX_ROWS;        // departures per display page
    doc["clock"] = cfgShowClock;    // whether a clock page is in the rotation
    xSemaphoreGive(dataMutex);
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
}

// Forward declarations
std::vector<Departure> fetchOebbStation(const String& stationName, int nowMin);
void applyPowerSchedule();
void maybeEnterDeepSleep();
String sanitize(String s);
extern bool lastNightState;
extern volatile bool standbyActive;  // true while the display panel is in sleep (standby window)
extern volatile bool powerDirty;     // set when config changes → displayTask re-evaluates power state

// Search ÖBB for station name, returns canonical name or empty string
String searchOebbStation(const String& query) {
    String encoded = query;
    encoded.replace(" ", "+");
    String url = "https://fahrplan.oebb.at/bin/ajax-getstop.exe/dn"
                 "?start=1&tpl=stop2json&REQ0JourneyStopsS0A=1"
                 "&REQ0JourneyStopsS0G=" + encoded + "&js=true&";

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.begin(client, url);
    http.setTimeout(15000);
    int code = http.GET();
    if (code != 200) { http.end(); return ""; }
    String payload = http.getString();
    http.end();

    // Response: SLs.sls={"suggestions":[{"value":"Wien Rennweg",...}]};SLs.showSuggestion();
    int eqIdx = payload.indexOf('=');
    if (eqIdx < 0) return "";
    int scIdx = payload.indexOf(';', eqIdx);
    String jsonStr = (scIdx > 0) ? payload.substring(eqIdx + 1, scIdx) : payload.substring(eqIdx + 1);
    jsonStr.trim();

    JsonDocument doc;
    if (deserializeJson(doc, jsonStr, DeserializationOption::NestingLimit(10))) return "";

    JsonArray suggestions = doc["suggestions"].as<JsonArray>();
    if (suggestions.isNull() || suggestions.size() == 0) return "";

    String name = suggestions[0]["value"].as<String>();
    name.trim();
    return name;
}

void handleOebbSearch() {
    LOG_REQ();
    String query = server.arg("q");
    query.trim();
    if (query.length() < 2) {
        server.sendHeader("Location", "/");
        server.send(302);
        return;
    }

    tft.fillScreen(BG_COLOR);
    tft.setTextColor(AMBER);
    tft.setTextFont(1);
    tft.setTextSize(2);
    tft.setCursor(10, 70);
    tft.print("OeBB Suche...");

    // Find canonical station name
    String stationName = searchOebbStation(query);
    if (stationName.length() == 0) {
        String html = FPSTR(HTML_HEAD);
    
        html += "<div class='card'><p class='status'>ÖBB-Station nicht gefunden: " + query + "</p>";
        html += "<a href='/'><button>Zurück</button></a></div></body></html>";
        sendHtml(html);
        return;
    }

    tft.setCursor(10, 95);
    tft.print("Lade Abfahrten...");

    // Fetch departures to discover lines/directions
    struct tm timeinfo;
    int nowMin = 0;
    if (getLocalTime(&timeinfo, 1000)) nowMin = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    auto allDeps = fetchOebbStation(stationName, nowMin);

    // Deduplicate line+direction combos
    std::vector<std::pair<String,String>> combos;  // line, towards
    for (auto& d : allDeps) {
        bool dup = false;
        for (auto& c : combos) {
            if (c.first == d.lineName && c.second == d.towards) { dup = true; break; }
        }
        if (!dup) combos.push_back(std::make_pair(d.lineName, d.towards));
    }

    String html = FPSTR(HTML_HEAD);

    html += "<div class='card'><h2>" + stationName + "</h2>";

    if (combos.empty()) {
        html += "<p class='status'>Keine Abfahrten gefunden</p>";
    } else {
        html += "<form action='/oebb-save' method='POST'>";
        html += "<input type='hidden' name='station' value='" + stationName + "'>";
        for (auto& c : combos) {
            bool active = false;
            for (auto& os : cfgOebb) {
                if (os.stationName == stationName && os.line == c.first && os.towards == c.second) {
                    active = true; break;
                }
            }
            String val = c.first + "|" + c.second;
            html += "<div class='line-item'>";
            html += "<input type='checkbox' name='entry' value='" + val + "'";
            if (active) html += " checked disabled";
            html += ">";
            html += "<span class='badge train'>" + c.first + "</span>";
            html += "<div class='dir'>" + c.second + "</div>";
            html += "</div>";
        }
        html += "<button class='add' type='submit'>Einzeln hinzuf&uuml;gen</button>";
        html += "</form>";

        // ── Watch Group option ────────────────────────────────────────
        html += "<div style='border-top:1px solid rgba(255,255,255,.06);margin-top:16px;padding-top:12px'>";
        html += "<p class='hint' style='margin-bottom:8px'>Oder: Auswahl als Gruppe &ndash; zeigt nur die n&auml;chsten N Z&uuml;ge aller ausgew&auml;hlten Linien zusammen.</p>";
        html += "<form action='/watch-group-save' method='POST'>";
        html += "<input type='hidden' name='source' value='oebb'>";
        html += "<input type='hidden' name='station' value='" + stationName + "'>";
        html += "<input type='hidden' name='label' value='" + stationName + "'>";
        for (auto& c : combos) {
            String val = c.first + "|" + c.second;
            html += "<div class='line-item'>";
            html += "<input type='checkbox' name='entry' value='" + val + "'>";
            html += "<span class='badge train'>" + c.first + "</span>";
            html += "<div class='dir'>" + c.second + "</div>";
            html += "</div>";
        }
        html += "<div style='display:flex;gap:8px;align-items:center;margin-top:10px'>";
        html += "<label style='font-size:13px;color:#888;white-space:nowrap'>N&auml;chste</label>";
        html += "<input type='number' name='max' min='1' max='5' value='2' style='width:55px;padding:8px;text-align:center'>";
        html += "<label style='font-size:13px;color:#888;white-space:nowrap'>Z&uuml;ge anzeigen</label>";
        html += "<button class='add' type='submit' style='flex:1;margin:0'>Als Gruppe speichern</button>";
        html += "</div></form>";
        html += "</div>";
    }

    html += "<br><a href='/'><button>Zur&uuml;ck</button></a></div></body></html>";
    sendHtml(html);
}

void handleOebbSave() {
    String stationName = server.arg("station");
    for (int i = 0; i < server.args(); i++) {
        if (server.argName(i) != "entry") continue;
        String val = server.arg(i);
        int sep = val.indexOf('|');
        if (sep < 0) continue;
        String line    = val.substring(0, sep);
        String towards = val.substring(sep + 1);

        bool dup = false;
        for (auto& os : cfgOebb) {
            if (os.stationName == stationName && os.line == line && os.towards == towards) {
                dup = true; break;
            }
        }
        if (!dup) {
            OebbStation os;
            os.stationName = stationName;
            os.line        = line;
            os.towards     = towards;
            cfgOebb.push_back(os);
        }
    }
    saveConfig();
    configChanged = true;
    server.sendHeader("Location", "/?saved=1");
    server.send(302);
}

void handleOebbRemove() {
    if (server.hasArg("idx")) {
        int idx = server.arg("idx").toInt();
        if (idx >= 0 && idx < (int)cfgOebb.size()) {
            cfgOebb.erase(cfgOebb.begin() + idx);
        }
    }
    saveConfig();
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    departures.clear();
    displaySlots.clear();
    xSemaphoreGive(dataMutex);
    configChanged = true;
    server.sendHeader("Location", "/");
    server.send(302);
}

void handleWatchGroupSave() {
    // Builds a WatchGroup from POSTed checkboxes (same format as /save and /oebb-save)
    String source  = server.arg("source");  // "wl" or "oebb"
    String label   = server.arg("label");
    int    maxN    = server.arg("max").toInt();
    if (maxN < 1 || maxN > 5) maxN = 2;
    if (source != "wl" && source != "oebb") { server.sendHeader("Location", "/"); server.send(302); return; }

    WatchGroup g;
    g.label          = label;
    g.maxDepartures  = maxN;

    if (source == "wl") {
        // entries: "rbl|name|towards|type" (same as /save)
        for (int i = 0; i < server.args(); i++) {
            if (server.argName(i) != "line") continue;
            String val = server.arg(i);
            int p1 = val.indexOf('|'), p2 = val.indexOf('|', p1+1), p3 = val.indexOf('|', p2+1);
            if (p1 < 0 || p2 < 0 || p3 < 0) continue;
            WatchGroupEntry e;
            e.source   = "wl";
            e.rbl      = val.substring(0, p1);
            e.lineName = val.substring(p1+1, p2);
            e.towards  = val.substring(p2+1, p3);
            e.type     = val.substring(p3+1);
            if (e.rbl.length() > 0) g.entries.push_back(e);
        }
    } else {
        // ÖBB entries: station + "line|towards" (same format as /oebb-save)
        String stationName = server.arg("station");
        for (int i = 0; i < server.args(); i++) {
            if (server.argName(i) != "entry") continue;
            String val = server.arg(i);
            int sep = val.indexOf('|');
            if (sep < 0) continue;
            WatchGroupEntry e;
            e.source      = "oebb";
            e.oebbStation = stationName;
            e.lineName    = val.substring(0, sep);
            e.towards     = val.substring(sep+1);
            e.type        = "ptTrainS";
            if (e.lineName.length() > 0) g.entries.push_back(e);
        }
    }

    if (g.entries.empty()) { server.sendHeader("Location", "/"); server.send(302); return; }
    cfgWatchGroups.push_back(g);
    saveConfig();
    configChanged = true;
    server.sendHeader("Location", "/?saved=1");
    server.send(302);
}

void handleWatchGroupRemove() {
    if (server.hasArg("idx")) {
        int idx = server.arg("idx").toInt();
        if (idx >= 0 && idx < (int)cfgWatchGroups.size())
            cfgWatchGroups.erase(cfgWatchGroups.begin() + idx);
    }
    saveConfig();
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    departures.clear();
    displaySlots.clear();
    xSemaphoreGive(dataMutex);
    configChanged = true;
    server.sendHeader("Location", "/");
    server.send(302);
}

void handleWatchGroupEdit() {
    LOG_REQ();
    int idx = server.arg("idx").toInt();
    if (idx < 0 || idx >= (int)cfgWatchGroups.size()) {
        server.sendHeader("Location", "/"); server.send(302); return;
    }
    WatchGroup& g = cfgWatchGroups[idx];
    String html = FPSTR(HTML_HEAD);
    html += "<div class='card'><h2>Gruppe bearbeiten</h2>";
    html += "<form action='/watch-group-update' method='POST'>";
    html += "<input type='hidden' name='idx' value='" + String(idx) + "'>";
    html += "<div class='setting'><div class='setting-label'>Label</div>";
    html += "<input type='text' name='label' value='" + g.label + "'></div>";
    html += "<div class='setting'><div class='setting-label'>N&auml;chste</div>";
    html += "<select name='max'>";
    for (int n = 1; n <= 5; n++) {
        html += "<option value='" + String(n) + "'";
        if (n == g.maxDepartures) html += " selected";
        html += ">" + String(n) + "</option>";
    }
    html += "</select></div>";
    html += "<h3>Eintr&auml;ge</h3>";
    for (size_t ei = 0; ei < g.entries.size(); ei++) {
        auto& e = g.entries[ei];
        String entryLabel = e.lineName + " &rarr; " + e.towards;
        if (e.source == "oebb") entryLabel += " (" + e.oebbStation + ")";
        html += "<div style='display:flex;align-items:center;gap:8px;margin:6px 0'>";
        html += "<label class='check-label' style='flex:1'><input type='checkbox' name='keep' value='" + String(ei) + "' checked> " + entryLabel + "</label>";
        html += "<label class='walk' title='Gehweg in Minuten \\u2014 0 = aus'><span class='lbl'>Gehw</span>";
        html += "<input type='number' min='0' max='30' name='walk_" + String(ei) + "' value='" + String(e.walkMin) +
                "' class='" + String(e.walkMin > 0 ? "on" : "") + "'></label>";
        html += "</div>";
    }
    html += "<button type='submit'>Speichern</button>";
    html += "</form>";
    html += "<a href='/'><button class='btn-info' style='margin-top:8px'>Abbrechen</button></a></div>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleWatchGroupUpdate() {
    int idx = server.arg("idx").toInt();
    if (idx < 0 || idx >= (int)cfgWatchGroups.size()) {
        server.sendHeader("Location", "/"); server.send(302); return;
    }
    WatchGroup& g = cfgWatchGroups[idx];
    String newLabel = server.arg("label");
    if (newLabel.length() > 0) g.label = newLabel;
    int newMax = server.arg("max").toInt();
    if (newMax >= 1 && newMax <= 5) g.maxDepartures = newMax;
    std::vector<bool> keep(g.entries.size(), false);
    for (int i = 0; i < server.args(); i++) {
        if (server.argName(i) == "keep") {
            int ei = server.arg(i).toInt();
            if (ei >= 0 && ei < (int)g.entries.size()) keep[ei] = true;
        }
    }
    for (size_t ei = 0; ei < g.entries.size(); ei++) {
        String wn = "walk_" + String(ei);
        if (server.hasArg(wn)) {
            int wmv = server.arg(wn).toInt();
            if (wmv < 0)  wmv = 0;
            if (wmv > 30) wmv = 30;
            g.entries[ei].walkMin = wmv;
        }
    }
    std::vector<WatchGroupEntry> newEntries;
    for (size_t ei = 0; ei < g.entries.size(); ei++)
        if (keep[ei]) newEntries.push_back(g.entries[ei]);
    if (newEntries.empty()) {
        cfgWatchGroups.erase(cfgWatchGroups.begin() + idx);
    } else {
        g.entries = newEntries;
    }
    saveConfig();
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    departures.clear();
    displaySlots.clear();
    xSemaphoreGive(dataMutex);
    configChanged = true;
    server.sendHeader("Location", "/?saved=1");
    server.send(302);
}

void handleBrowse() {
    LOG_REQ();
    unsigned long t0 = millis();
    String html = FPSTR(HTML_HEAD);

    html += "<div class='card'><h2>Stationen A-Z</h2>";

    // Letter navigation
    String letter = server.arg("l");
    letter.toUpperCase();
    html += "<div class='letters'>";
    for (char c = 'A'; c <= 'Z'; c++) {
        String l(c);
        String cls = (letter == l) ? " class='act'" : "";
        html += "<a href='/browse?l=" + l + "'" + cls + ">" + l + "</a>";
    }
    html += "</div>";

    if (letter.length() >= 1) {
        File f = SPIFFS.open(CACHE_HALT_PATH, "r");

        if (f) {
            std::vector<String> names;
            std::map<String, bool> seen;

            while (f.available()) {
                String line = f.readStringUntil('\n');
                int s1 = line.indexOf(';');
                if (s1 < 0) continue;
                int s2 = line.indexOf(';', s1 + 1);
                if (s2 < 0) continue;
                int s3 = line.indexOf(';', s2 + 1);
                if (s3 < 0) continue;
                int s4 = line.indexOf(';', s3 + 1);
                if (s4 < 0) continue;
                String name = line.substring(s3 + 1, s4);
                name.replace("\"", "");
                if (name.length() == 0) continue;

                // Map Umlauts to base letter for browse matching
                String normalized = normalizeForSearch(name.substring(0, 2));  // first char (UTF-8 may be 2 bytes)
                String firstNorm = normalized.substring(0, 1);
                firstNorm.toUpperCase();
                if (firstNorm != letter) continue;

                if (seen.find(name) != seen.end()) continue;
                seen[name] = true;
                names.push_back(name);
            }
            f.close();

            std::sort(names.begin(), names.end());

            if (names.empty()) {
                html += "<p class='status'>Keine Stationen mit " + letter + "</p>";
            } else {
                html += "<p class='stn-count'>" + String(names.size()) + " Stationen</p>";
                for (auto& n : names) {
                    String encoded = n;
                    encoded.replace(" ", "+");
                    html += "<a class='stn' href='/search?q=" + encoded + "'>" + n + "</a>";
                }
            }
        } else {
            html += "<p class='status'>Stationsdaten nicht verfügbar. Bitte neu starten.</p>";
        }
    } else {
        html += "<p class='status'>Wähle einen Buchstaben</p>";
    }

    html += "<br><a href='/'><button>Zurück</button></a></div></body></html>";
    logf("[browse] html built: %lums  size: %u bytes\n", millis()-t0, html.length());
    sendHtml(html);
    logf("[browse] sendHtml done: %lums\n", millis()-t0);
}

void handleWifiReset() {
    LOG_REQ();
    // Set flag so setup() opens config portal on restart
    File f = SPIFFS.open(WIFI_RESET_FLAG, "w");
    if (f) { f.print("1"); f.close(); }

    String html = FPSTR(HTML_HEAD);
    html += "<div class='card'><h2>WiFi Reset</h2><p class='status'>Monitor startet neu...<br><br>";
    html += "Verbinde dich mit:<br><b style='color:#ffbf00;font-size:1.3em'>LineTracker</b><br><br>";
    html += "Wähle dein WLAN, dann öffne:<br><b style='color:#ffbf00'>linetracker.local</b></p></div></body></html>";
    sendHtml(html);
    delay(1500);
    ESP.restart();
}

void handleFactoryReset() {
    LOG_REQ();
    SPIFFS.remove(CONFIG_PATH);
    SPIFFS.remove(DIR_CACHE_PATH);
    SPIFFS.remove(LINE_DIRS_PATH);
    SPIFFS.remove(CACHE_HALT_PATH);
    SPIFFS.remove(CACHE_STEIGE_PATH);
    SPIFFS.remove(CACHE_LINIEN_PATH);
    SPIFFS.remove(CACHE_TS_PATH);
    SPIFFS.remove(WIFI_CONFIGURED_FLAG);

    WiFiManager wm;
    wm.resetSettings();

    String html = FPSTR(HTML_HEAD);
    html += "<div class='card'><h2>Werksreset</h2><p class='status'>Alles gelöscht!<br><br>";
    html += "Monitor startet neu...<br><br>";
    html += "Verbinde dich mit:<br><b style='color:#ffbf00;font-size:1.3em'>LineTracker</b><br><br>";
    html += "Wähle dein WLAN, dann öffne:<br><b style='color:#ffbf00'>linetracker.local</b></p></div></body></html>";
    sendHtml(html);
    delay(1500);
    ESP.restart();
}

void handleSettings() {
    LOG_REQ();
    if (server.hasArg("rotate_sec")) {
        cfgRotateSec = server.arg("rotate_sec").toInt();
        if (cfgRotateSec < 2)  cfgRotateSec = 2;
        if (cfgRotateSec > 60) cfgRotateSec = 60;
    }
    if (server.hasArg("brightness")) {
        cfgBrightness = server.arg("brightness").toInt();
        if (cfgBrightness < 10)  cfgBrightness = 10;
        if (cfgBrightness > 255) cfgBrightness = 255;
    }
    // Night mode
    bool nightOn = server.hasArg("night_on");
    if (nightOn) {
        cfgNightFrom = server.arg("night_from").toInt();
        cfgNightTo   = server.arg("night_to").toInt();
        // night_bright slider is 0-100 percent
        int pct      = server.arg("night_bright").toInt();
        cfgNightBright = pct * 255 / 100;
    } else {
        cfgNightFrom = -1;
        cfgNightTo   = -1;
    }
    // Standby / sleep (display fully off)
    bool standbyOn = server.hasArg("standby_on");
    if (standbyOn) {
        cfgStandbyFrom = server.arg("standby_from").toInt();
        cfgStandbyTo   = server.arg("standby_to").toInt();
        // Deep sleep is experimental and only accepted with explicit risk confirmation.
        cfgStandbyDeepSleep = server.hasArg("standby_deep") && server.hasArg("standby_deep_ack");
    } else {
        cfgStandbyFrom = -1;
        cfgStandbyTo   = -1;
        cfgStandbyDeepSleep = false;
    }
    // Weekend schedule (separate night/standby windows for Sat/Sun)
    cfgWeekendSchedule = server.hasArg("weekend_sched");
    if (cfgWeekendSchedule) {
        cfgNightFromWe   = server.hasArg("night_from_we")   ? server.arg("night_from_we").toInt()   : -1;
        cfgNightToWe     = server.hasArg("night_to_we")     ? server.arg("night_to_we").toInt()     : -1;
        cfgStandbyFromWe = server.hasArg("standby_from_we") ? server.arg("standby_from_we").toInt() : -1;
        cfgStandbyToWe   = server.hasArg("standby_to_we")   ? server.arg("standby_to_we").toInt()   : -1;
    }
    cfgShowNext        = server.hasArg("show_next");
    cfgShowDisruptions = server.hasArg("show_disruptions");
    cfgShowClock       = server.hasArg("show_clock");
    cfgShowWeather     = server.hasArg("show_weather");
    cfgLineColors      = server.hasArg("line_colors");
    cfgSortByTime      = server.hasArg("sort_by_time");
    cfgBetaChannel     = server.hasArg("beta_channel");

    // Let displayTask re-evaluate backlight/panel state on its next tick
    // (panel SPI commands must not run from the web-server task).
    powerDirty = true;

    saveConfig();
    server.sendHeader("Location", "/?saved=1");
    server.send(302);
}

// ── OTA Update ──────────────────────────────────────────────────────
// Compare semantic versions: returns true if remote > local
bool isNewerVersion(const String& remote, const String& local) {
    int rMaj = 0, rMin = 0, rPat = 0;
    int lMaj = 0, lMin = 0, lPat = 0;
    sscanf(remote.c_str(), "%d.%d.%d", &rMaj, &rMin, &rPat);
    sscanf(local.c_str(),  "%d.%d.%d", &lMaj, &lMin, &lPat);
    if (rMaj != lMaj) return rMaj > lMaj;
    if (rMin != lMin) return rMin > lMin;
    return rPat > lPat;
}

// Check for OTA update, returns true if update was started
bool checkOtaUpdate() {
    if (WiFi.status() != WL_CONNECTED) return false;

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.begin(client, cfgBetaChannel ? OTA_VERSION_URL_BETA : OTA_VERSION_URL);
    http.setTimeout(10000);
    int code = http.GET();
    if (code != 200) { http.end(); return false; }

    String payload = http.getString();
    http.end();

    JsonDocument doc;
    if (deserializeJson(doc, payload)) return false;

    String remoteVer = doc["version"] | "";
    String binUrl    = doc["url"] | "";
    if (remoteVer.length() == 0 || binUrl.length() == 0) return false;

    if (!isNewerVersion(remoteVer, FW_VERSION)) {
        Serial.println("OTA: up to date (v" + String(FW_VERSION) + ")");
        return false;
    }

    Serial.println("OTA: new version " + remoteVer + " available, updating...");

    // Signal display task to show OTA progress
    otaNewVersion = remoteVer;
    otaPercent = 0;
    otaInProgress = true;

    WiFiClientSecure updClient;
    updClient.setInsecure();

    httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    httpUpdate.onProgress([](int cur, int total) {
        if (total > 0) otaPercent = cur * 100 / total;
    });
    t_httpUpdate_return ret = httpUpdate.update(updClient, binUrl);

    otaInProgress = false;
    switch (ret) {
        case HTTP_UPDATE_OK:
            Serial.println("OTA: success, rebooting");
            ESP.restart();
            break;
        case HTTP_UPDATE_FAILED:
            Serial.printf("OTA: failed (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
            break;
        case HTTP_UPDATE_NO_UPDATES:
            Serial.println("OTA: no update needed");
            break;
    }
    return false;
}

// Check version only (no flash), returns remote version or empty
String checkRemoteVersion() {
    if (WiFi.status() != WL_CONNECTED) return "";
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.begin(client, cfgBetaChannel ? OTA_VERSION_URL_BETA : OTA_VERSION_URL);
    http.setTimeout(10000);
    int code = http.GET();
    if (code != 200) {
        Serial.printf("OTA check: HTTP %d\n", code);
        http.end();
        return "";
    }
    String payload = http.getString();
    http.end();
    JsonDocument doc;
    if (deserializeJson(doc, payload)) return "";
    return doc["version"] | "";
}

void handleOtaCheck() {
    LOG_REQ();
    String html = FPSTR(HTML_HEAD);
    html += "<div class='card'><h2>Firmware Update</h2>";
    html += "<p style='font-size:13px;color:#aaa;margin-bottom:12px'>Installiert: <b style='color:#eee'>v" + String(FW_VERSION) + "</b></p>";
    html += "<div id='res'><p class='status'>Prüfe auf Updates...</p></div>";
    html += "<script>"
            "fetch('/update-status').then(r=>r.json()).then(function(d){"
            "var el=document.getElementById('res');"
            "if(d.err){el.innerHTML=\"<p class='status'>Updateserver nicht erreichbar.</p>\";}"
            "else if(d.newer){el.innerHTML=\"<p style='text-align:center;color:#1a8f3c;font-size:16px;font-weight:600;margin:12px 0'>v\"+d.ver+\" verf\\u00fcgbar!</p>"
            "<a href='/update-now'><button class='add'>Jetzt updaten</button></a>\";}"
            "else{el.innerHTML=\"<p class='status'>Firmware ist aktuell.</p>\";}"
            "}).catch(function(){document.getElementById('res').innerHTML=\"<p class='status'>Updateserver nicht erreichbar.</p>\";});"
            "</script>";
    html += "<br><a href='/'><button class='btn-secondary'>Zurück</button></a>";
    html += "</div></body></html>";
    sendHtml(html);
}

void handleOtaStatus() {
    unsigned long t0 = millis();
    logf("[ota] status check started\n");
    String remoteVer = checkRemoteVersion();
    logf("[ota] checkRemoteVersion: %lums\n", millis()-t0);
    if (remoteVer.length() == 0) {
        server.send(200, "application/json", "{\"err\":1}");
        return;
    }
    bool newer = isNewerVersion(remoteVer, FW_VERSION);
    String json = "{\"ver\":\"" + remoteVer + "\",\"newer\":" + (newer ? "1" : "0") + "}";
    server.send(200, "application/json", json);
    logf("[ota] status done: %lums\n", millis()-t0);
}

void handleOtaDoUpdate() {
    String html = FPSTR(HTML_HEAD);
    html += "<div class='card' style='text-align:center'>";
    html += "<h2>Update wird installiert...</h2>";
    html += "<p style='color:#aaa;font-size:14px;margin:16px 0'>Bitte nicht ausschalten!<br>Der Monitor startet automatisch neu.</p>";
    html += "<div style='margin:20px auto;width:80%;height:10px;background:#222;border-radius:5px'>";
    html += "<div id='bar' style='width:0%;height:100%;background:#ffbf00;border-radius:5px;transition:width .5s'></div></div>";
    html += "<p id='pct' style='color:#ffbf00;font-size:20px;font-weight:700'>0%</p>";
    html += "</div>";
    html += "<script>"
            "var iv=setInterval(function(){"
            "fetch('/update-progress').then(r=>r.json()).then(d=>{"
            "document.getElementById('bar').style.width=d.p+'%';"
            "document.getElementById('pct').textContent=d.p+'%';"
            "if(d.p>=100){clearInterval(iv);document.getElementById('pct').textContent='Neustart...';}"
            "}).catch(function(){clearInterval(iv);"
            "document.getElementById('pct').textContent='Neustart...';"
            "setTimeout(function(){location.href='/';},8000);"
            "})},800);"
            "</script></body></html>";
    sendHtml(html);
    delay(200);
    checkOtaUpdate();
}

void handleOtaProgress() {
    String json = "{\"p\":" + String(otaPercent) + "}";
    server.send(200, "application/json", json);
}

// ── Pong handlers ───────────────────────────────────────────────────
static const char PONG_PAGE[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="de"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>LineTracker Pong</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent;touch-action:manipulation}
  html,body{height:100%;background:#0a0805;color:#ffbf00;font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',monospace;overflow:hidden}
  body{display:flex;flex-direction:column;height:100vh;height:100svh}
  header{padding:10px 14px;background:#1a1408;border-bottom:1px solid #322600;font-size:12px;display:flex;justify-content:space-between;align-items:center;gap:8px}
  header .me{font-weight:700;letter-spacing:.5px}
  header .score{font-variant-numeric:tabular-nums;font-size:14px}
  .pad{flex:1;display:flex;align-items:center;justify-content:center;font-size:48px;font-weight:700;letter-spacing:2px;background:#0a0805;border:none;color:#ffbf00;transition:background .08s}
  .pad.up{border-bottom:1px dashed #322600}
  .pad.down{border-top:1px dashed #322600}
  .pad.held{background:#322600;color:#ffd24a}
  .pad:disabled{opacity:.3}
  footer{padding:8px 14px;background:#1a1408;border-top:1px solid #322600;font-size:11px;text-align:center;color:#a07a00}
  .banner{position:fixed;inset:0;display:none;align-items:center;justify-content:center;background:rgba(10,8,5,.92);z-index:10;flex-direction:column;gap:14px;padding:24px;text-align:center}
  .banner.show{display:flex}
  .banner h1{font-size:28px;letter-spacing:1px}
  .banner p{font-size:14px;color:#c89200}
</style></head><body>
<header>
  <div class="me" id="me">verbinde…</div>
  <div class="score" id="score">— : —</div>
</header>
<button class="pad up" id="up">&#9650; HOCH</button>
<button class="pad down" id="down">&#9660; RUNTER</button>
<footer>linetracker.local/pong &middot; halt gedrückt um zu fahren</footer>
<div class="banner" id="banner"><h1 id="bTitle">—</h1><p id="bSub"></p></div>
<script>
(function(){
  let token = localStorage.getItem('pongToken') || '';
  let side = 'spectator';
  let curDir = 0;
  let lastSentDir = 0;

  async function postForm(url, params){
    const body = new URLSearchParams(params).toString();
    const r = await fetch(url, {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});
    return r;
  }

  async function join(){
    try{
      const r = await postForm('/pong/join', token ? {token} : {});
      if(r.status === 409){
        side = 'spectator';
        document.getElementById('me').textContent = 'Zuschauer';
        document.getElementById('up').disabled = true;
        document.getElementById('down').disabled = true;
        return;
      }
      const d = await r.json();
      token = d.token; side = d.side;
      localStorage.setItem('pongToken', token);
      document.getElementById('me').textContent = side === 'left' ? 'Du: LINKS' : 'Du: RECHTS';
    }catch(e){
      document.getElementById('me').textContent = 'Verbindung fehlgeschlagen';
    }
  }

  function setDir(d){
    if(d !== 0 && d !== curDir) navigator.vibrate && navigator.vibrate(18);
    curDir = d;
    document.getElementById('up').classList.toggle('held', d === -1);
    document.getElementById('down').classList.toggle('held', d === 1);
  }

  function bindPad(id, dir){
    const el = document.getElementById(id);
    const press = e => { e.preventDefault(); if(side==='spectator') return; setDir(dir); };
    const release = e => { e.preventDefault(); setDir(0); };
    el.addEventListener('touchstart', press, {passive:false});
    el.addEventListener('touchend',   release);
    el.addEventListener('touchcancel',release);
    el.addEventListener('mousedown',  press);
    el.addEventListener('mouseup',    release);
    el.addEventListener('mouseleave', release);
  }
  bindPad('up', -1); bindPad('down', 1);

  // Send move commands while a button is held; also a heartbeat at idle
  setInterval(()=>{
    if(!token || side==='spectator') return;
    if(curDir !== lastSentDir || curDir !== 0){
      postForm('/pong/move', {token, dir: curDir}).catch(()=>{});
      lastSentDir = curDir;
    }
  }, 100);

  // Idle heartbeat so server keeps slot alive
  setInterval(()=>{
    if(!token || side==='spectator') return;
    postForm('/pong/move', {token, dir: 0}).catch(()=>{});
  }, 4000);

  // Poll state for live score + vibration feedback
  let prevL = 0, prevR = 0, prevOver = false;
  async function pollState(){
    try{
      const r = await fetch('/pong/state');
      const d = await r.json();
      document.getElementById('score').textContent = d.l + ' : ' + d.r;

      // Goal vibration: detect score increase
      if(side !== 'spectator'){
        const myScore = side === 'left' ? d.l : d.r;
        const myPrev  = side === 'left' ? prevL : prevR;
        const oppScore = side === 'left' ? d.r : d.l;
        const oppPrev  = side === 'left' ? prevR : prevL;
        if(myScore > myPrev)  navigator.vibrate && navigator.vibrate([40, 30, 40]);  // scored!
        if(oppScore > oppPrev) navigator.vibrate && navigator.vibrate(80);            // conceded
      }
      prevL = d.l; prevR = d.r;

      const banner = document.getElementById('banner');
      if(d.over){
        if(!prevOver) navigator.vibrate && navigator.vibrate([100, 80, 100, 80, 200]);
        document.getElementById('bTitle').textContent = 'Endstation!';
        document.getElementById('bSub').textContent = (d.win===1?'LINKS':'RECHTS') + ' gewinnt';
        banner.classList.add('show');
      } else {
        banner.classList.remove('show');
      }
      prevOver = d.over;
    }catch(e){}
  }
  setInterval(pollState, 500);

  window.addEventListener('beforeunload', ()=>{
    if(token) navigator.sendBeacon('/pong/leave', new URLSearchParams({token}).toString());
  });

  join();
})();
</script>
</body></html>)rawliteral";

static void pongFillToken(char out[9]) {
    static const char* hex = "0123456789abcdef";
    uint32_t a = esp_random();
    uint32_t b = esp_random();
    for (int i = 0; i < 4; i++) { out[i]     = hex[(a >> (i * 4)) & 0xF]; }
    for (int i = 0; i < 4; i++) { out[i + 4] = hex[(b >> (i * 4)) & 0xF]; }
    out[8] = 0;
}

static PongPlayer* pongPlayerByToken(const String& tk) {
    if (tk.length() != 8) return nullptr;
    if (pong.left.active  && tk == pong.left.token)  return &pong.left;
    if (pong.right.active && tk == pong.right.token) return &pong.right;
    return nullptr;
}

void handlePongPage() {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html; charset=utf-8", PONG_PAGE);
}

void handlePongJoin() {
    if (!pongMutex) { server.send(503, "text/plain", "not ready"); return; }
    if (xSemaphoreTake(pongMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        server.send(503, "text/plain", "busy"); return;
    }

    unsigned long now = millis();

    // If client sent an existing token, refresh that slot instead of taking a new one
    String tk = server.arg("token");
    PongPlayer* existing = pongPlayerByToken(tk);
    if (existing) {
        existing->lastSeenMs  = now;
        existing->lastInputMs = now;
        const char* side = (existing == &pong.left) ? "left" : "right";
        String json = String("{\"side\":\"") + side + "\",\"token\":\"" + existing->token + "\"}";
        xSemaphoreGive(pongMutex);
        server.send(200, "application/json", json);
        return;
    }

    // Reclaim stale slots
    if (pong.left.active  && now - pong.left.lastSeenMs  > PONG_PLAYER_TIMEOUT_MS) {
        pong.left.active = false; pong.left.token[0] = 0;
    }
    if (pong.right.active && now - pong.right.lastSeenMs > PONG_PLAYER_TIMEOUT_MS) {
        pong.right.active = false; pong.right.token[0] = 0;
    }

    PongPlayer* slot = nullptr;
    const char* sideStr = nullptr;
    if (!pong.left.active)        { slot = &pong.left;  sideStr = "left"; }
    else if (!pong.right.active)  { slot = &pong.right; sideStr = "right"; }

    if (!slot) {
        xSemaphoreGive(pongMutex);
        server.send(409, "application/json", "{\"error\":\"full\"}");
        return;
    }

    pongFillToken(slot->token);
    slot->active      = true;
    slot->inputDir    = 0;
    slot->lastInputMs = now;
    slot->lastSeenMs  = now;

    appMode = MODE_PONG;

    String json = String("{\"side\":\"") + sideStr + "\",\"token\":\"" + slot->token + "\"}";
    xSemaphoreGive(pongMutex);
    server.send(200, "application/json", json);
}

void handlePongMove() {
    if (!pongMutex) { server.send(503, "text/plain", "not ready"); return; }
    if (xSemaphoreTake(pongMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        server.send(503, "text/plain", "busy"); return;
    }

    String tk = server.arg("token");
    int dir = server.arg("dir").toInt();
    if (dir < -1) dir = -1;
    if (dir > 1)  dir = 1;

    PongPlayer* p = pongPlayerByToken(tk);
    if (!p) {
        xSemaphoreGive(pongMutex);
        server.send(401, "text/plain", "bad token");
        return;
    }

    unsigned long now = millis();
    p->inputDir    = (int8_t)dir;
    p->lastInputMs = now;
    p->lastSeenMs  = now;

    xSemaphoreGive(pongMutex);
    server.send(204, "text/plain", "");
}

void handlePongState() {
    if (!pongMutex) { server.send(503, "text/plain", "not ready"); return; }
    if (xSemaphoreTake(pongMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        server.send(503, "text/plain", "busy"); return;
    }

    String tk = server.arg("token");
    const char* side = "spectator";
    PongPlayer* p = pongPlayerByToken(tk);
    if (p) side = (p == &pong.left) ? "left" : "right";

    String json = "{";
    json += "\"l\":" + String(pong.leftScore) + ",";
    json += "\"r\":" + String(pong.rightScore) + ",";
    json += "\"running\":" + String(pong.gameRunning ? "true" : "false") + ",";
    json += "\"over\":" + String(pong.gameOver ? "true" : "false") + ",";
    json += "\"win\":" + String(pong.winner) + ",";
    json += "\"side\":\"" + String(side) + "\"";
    json += "}";

    xSemaphoreGive(pongMutex);
    server.send(200, "application/json", json);
}

void handlePongLeave() {
    if (!pongMutex) { server.send(204, "text/plain", ""); return; }
    if (xSemaphoreTake(pongMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        server.send(204, "text/plain", ""); return;
    }

    String tk = server.arg("token");
    PongPlayer* p = pongPlayerByToken(tk);
    if (p) {
        p->active = false;
        p->token[0] = 0;
        p->inputDir = 0;
    }
    if (!pong.left.active && !pong.right.active) {
        pong.gameRunning = false;
        pong.gameOver    = false;
        appMode          = MODE_DEPARTURES;
    }

    xSemaphoreGive(pongMutex);
    server.send(204, "text/plain", "");
}

// ── Snake handlers ──────────────────────────────────────────────────
static const char SNAKE_PAGE[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="de"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>LineTracker Snake</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent;touch-action:manipulation}
  html,body{height:100%;background:#0a0805;color:#ffbf00;font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',monospace;overflow:hidden}
  body{display:flex;flex-direction:column;height:100vh;height:100svh}
  header{padding:10px 14px;background:#1a1408;border-bottom:1px solid #322600;font-size:12px;display:flex;justify-content:space-between;align-items:center;gap:8px}
  header .me{font-weight:700;letter-spacing:.5px}
  header .score{font-variant-numeric:tabular-nums;font-size:14px}
  .field{flex:1;display:flex;align-items:center;justify-content:center;background:#0a0805}
  .pad{display:grid;grid-template-columns:1fr 1fr 1fr;grid-template-rows:1fr 1fr 1fr;width:min(86vw,360px);aspect-ratio:1;gap:8px}
  .pad button{background:#1a1408;border:1px solid #322600;color:#ffbf00;font-size:32px;font-weight:700;border-radius:10px;transition:background .08s,transform .08s}
  .pad button.held{background:#322600;color:#ffd24a;transform:scale(.96)}
  .pad button.spacer{visibility:hidden}
  .up{grid-column:2;grid-row:1}
  .left{grid-column:1;grid-row:2}
  .right{grid-column:3;grid-row:2}
  .down{grid-column:2;grid-row:3}
  footer{padding:8px 14px;background:#1a1408;border-top:1px solid #322600;font-size:11px;text-align:center;color:#a07a00}
  .banner{position:fixed;inset:0;display:none;align-items:center;justify-content:center;background:rgba(10,8,5,.92);z-index:10;flex-direction:column;gap:14px;padding:24px;text-align:center}
  .banner.show{display:flex}
  .banner h1{font-size:28px;letter-spacing:1px}
  .banner p{font-size:14px;color:#c89200}
  .hint{font-size:11px;color:#a07a00;margin-top:6px}
</style></head><body>
<header>
  <div class="me" id="me">verbinde&hellip;</div>
  <div class="score" id="score">&mdash;</div>
</header>
<div class="field">
  <div class="pad">
    <button class="up"    id="bUp">&#9650;</button>
    <button class="left"  id="bLeft">&#9664;</button>
    <button class="right" id="bRight">&#9654;</button>
    <button class="down"  id="bDown">&#9660;</button>
  </div>
</div>
<footer>linetracker.local/snake &middot; wische oder D-Pad</footer>
<div class="banner" id="banner"><h1 id="bTitle">&mdash;</h1><p id="bSub"></p></div>
<script>
(function(){
  let token = localStorage.getItem('snakeToken') || '';
  let mine = false;

  async function postForm(url, params){
    const body = new URLSearchParams(params).toString();
    return fetch(url, {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});
  }
  async function join(){
    try{
      const r = await postForm('/snake/join', token ? {token} : {});
      if(r.status === 409){
        mine = false;
        document.getElementById('me').textContent = 'Belegt';
        ['bUp','bDown','bLeft','bRight'].forEach(id=>document.getElementById(id).disabled = true);
        return;
      }
      const d = await r.json();
      token = d.token; mine = true;
      localStorage.setItem('snakeToken', token);
      document.getElementById('me').textContent = 'Du fährst';
    }catch(e){
      document.getElementById('me').textContent = 'Verbindung fehlgeschlagen';
    }
  }
  function send(dir){
    if(!mine || !token) return;
    if(navigator.vibrate) navigator.vibrate(15);
    postForm('/snake/move', {token, dir}).catch(()=>{});
  }
  function bind(id, dir){
    const el = document.getElementById(id);
    const press = e => { e.preventDefault(); el.classList.add('held'); send(dir); };
    const release = e => { e.preventDefault(); el.classList.remove('held'); };
    el.addEventListener('touchstart', press, {passive:false});
    el.addEventListener('touchend',   release);
    el.addEventListener('touchcancel',release);
    el.addEventListener('mousedown',  press);
    el.addEventListener('mouseup',    release);
    el.addEventListener('mouseleave', release);
  }
  bind('bUp', 3); bind('bDown', 1); bind('bLeft', 2); bind('bRight', 0);

  // Swipe on the field
  let sx=0, sy=0, st=0;
  const field = document.querySelector('.field');
  field.addEventListener('touchstart', e => { const t=e.touches[0]; sx=t.clientX; sy=t.clientY; st=Date.now(); }, {passive:true});
  field.addEventListener('touchend', e => {
    const t=e.changedTouches[0]; const dx=t.clientX-sx, dy=t.clientY-sy;
    if(Date.now()-st > 600) return;
    if(Math.abs(dx) < 24 && Math.abs(dy) < 24) return;
    if(Math.abs(dx) > Math.abs(dy))  send(dx > 0 ? 0 : 2);
    else                              send(dy > 0 ? 1 : 3);
  });

  // Keyboard for desktop testing
  document.addEventListener('keydown', e => {
    const m = {ArrowUp:3, ArrowDown:1, ArrowLeft:2, ArrowRight:0, w:3, s:1, a:2, d:0};
    if(m[e.key] !== undefined) { e.preventDefault(); send(m[e.key]); }
  });

  // Heartbeat
  setInterval(()=>{ if(!token||!mine) return; postForm('/snake/move', {token, dir:-1}).catch(()=>{}); }, 4000);

  let prevScore = 0, prevOver = false;
  async function poll(){
    try{
      const r = await fetch('/snake/state');
      const d = await r.json();
      document.getElementById('score').textContent = String(d.score) + ' / best ' + d.best;
      if(d.score > prevScore && navigator.vibrate) navigator.vibrate([20,10,20]);
      prevScore = d.score;
      const banner = document.getElementById('banner');
      if(d.over){
        if(!prevOver && navigator.vibrate) navigator.vibrate([100,80,200]);
        document.getElementById('bTitle').textContent = 'Endstation';
        document.getElementById('bSub').textContent = 'Score ' + d.score + ' — startet neu';
        banner.classList.add('show');
      } else {
        banner.classList.remove('show');
      }
      prevOver = d.over;
    }catch(e){}
  }
  setInterval(poll, 500);

  window.addEventListener('beforeunload', ()=>{
    if(token) navigator.sendBeacon('/snake/leave', new URLSearchParams({token}).toString());
  });
  join();
})();
</script>
</body></html>)rawliteral";

static void snakeFillToken(char out[9]) {
    static const char* hex = "0123456789abcdef";
    uint32_t a = esp_random();
    uint32_t b = esp_random();
    for (int i = 0; i < 4; i++) { out[i]     = hex[(a >> (i * 4)) & 0xF]; }
    for (int i = 0; i < 4; i++) { out[i + 4] = hex[(b >> (i * 4)) & 0xF]; }
    out[8] = 0;
}

// Caller must hold snakeMutex.
static bool snakeBodyContains(int x, int y) {
    for (int i = 0; i < snake.length; i++) {
        if (snake.body[i].x == x && snake.body[i].y == y) return true;
    }
    return false;
}

// Caller must hold snakeMutex.
static void snakePlaceFood() {
    if (snake.length >= SNAKE_COLS * SNAKE_ROWS) return;
    for (int tries = 0; tries < 200; tries++) {
        int x = esp_random() % SNAKE_COLS;
        int y = esp_random() % SNAKE_ROWS;
        if (!snakeBodyContains(x, y)) { snake.food.x = x; snake.food.y = y; return; }
    }
}

// Caller must hold snakeMutex.
static void snakeResetMatch() {
    int cx = SNAKE_COLS / 2;
    int cy = SNAKE_ROWS / 2;
    snake.length = 3;
    snake.body[0] = {(int8_t)(cx - 2), (int8_t)cy};  // tail
    snake.body[1] = {(int8_t)(cx - 1), (int8_t)cy};
    snake.body[2] = {(int8_t)cx,       (int8_t)cy};  // head
    snake.dir = 0;          // moving right
    snake.pendingDir = 0;
    snake.score = 0;
    snake.gameOver = false;
    snake.gameOverMs = 0;
    snake.deathFlashMs = 0;
    snake.tickIntervalMs = SNAKE_TICK_START_MS;
    snake.lastTickMs = millis();
    snakePlaceFood();
}

void handleSnakePage() {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html; charset=utf-8", SNAKE_PAGE);
}

void handleSnakeJoin() {
    if (!snakeMutex) { server.send(503, "text/plain", "not ready"); return; }
    if (xSemaphoreTake(snakeMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        server.send(503, "text/plain", "busy"); return;
    }

    unsigned long now = millis();
    String tk = server.arg("token");

    // Refresh existing slot
    if (snake.active && tk.length() == 8 && tk == snake.token) {
        snake.lastSeenMs  = now;
        snake.lastInputMs = now;
        String json = String("{\"token\":\"") + snake.token + "\"}";
        xSemaphoreGive(snakeMutex);
        server.send(200, "application/json", json);
        return;
    }

    // Reclaim stale slot
    if (snake.active && now - snake.lastSeenMs > SNAKE_PLAYER_TIMEOUT_MS) {
        snake.active = false; snake.token[0] = 0;
    }

    if (snake.active) {
        xSemaphoreGive(snakeMutex);
        server.send(409, "application/json", "{\"error\":\"busy\"}");
        return;
    }

    snakeFillToken(snake.token);
    snake.active      = true;
    snake.lastInputMs = now;
    snake.lastSeenMs  = now;
    snakeResetMatch();
    snake.gameRunning = true;
    appMode = MODE_SNAKE;

    String json = String("{\"token\":\"") + snake.token + "\"}";
    xSemaphoreGive(snakeMutex);
    server.send(200, "application/json", json);
}

void handleSnakeMove() {
    if (!snakeMutex) { server.send(503, "text/plain", "not ready"); return; }
    if (xSemaphoreTake(snakeMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        server.send(503, "text/plain", "busy"); return;
    }

    String tk = server.arg("token");
    if (!snake.active || tk.length() != 8 || tk != snake.token) {
        xSemaphoreGive(snakeMutex);
        server.send(401, "text/plain", "bad token");
        return;
    }

    int dir = server.arg("dir").toInt();
    unsigned long now = millis();
    snake.lastInputMs = now;
    snake.lastSeenMs  = now;

    // dir == -1 is just heartbeat
    if (dir >= 0 && dir <= 3) {
        // Reject reversal (dir is opposite of current dir → would suicide)
        int opp = (snake.dir + 2) & 3;
        if (dir != opp || snake.length <= 1) {
            snake.pendingDir = (int8_t)dir;
        }
    }

    xSemaphoreGive(snakeMutex);
    server.send(204, "text/plain", "");
}

void handleSnakeState() {
    if (!snakeMutex) { server.send(503, "text/plain", "not ready"); return; }
    if (xSemaphoreTake(snakeMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        server.send(503, "text/plain", "busy"); return;
    }
    String json = "{";
    json += "\"score\":"   + String(snake.score) + ",";
    json += "\"best\":"    + String(snake.highScore) + ",";
    json += "\"running\":" + String(snake.gameRunning ? "true" : "false") + ",";
    json += "\"over\":"    + String(snake.gameOver ? "true" : "false");
    json += "}";
    xSemaphoreGive(snakeMutex);
    server.send(200, "application/json", json);
}

void handleSnakeLeave() {
    if (!snakeMutex) { server.send(204, "text/plain", ""); return; }
    if (xSemaphoreTake(snakeMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        server.send(204, "text/plain", ""); return;
    }
    String tk = server.arg("token");
    if (snake.active && tk.length() == 8 && tk == snake.token) {
        snake.active      = false;
        snake.token[0]    = 0;
        snake.gameRunning = false;
        snake.gameOver    = false;
        appMode = MODE_DEPARTURES;
    }
    xSemaphoreGive(snakeMutex);
    server.send(204, "text/plain", "");
}

void startConfigServer() {
    server.on("/", handleRoot);
    server.on("/search", handleSearch);
    server.on("/browse", handleBrowse);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/remove", HTTP_POST, handleRemove);
    server.on("/set-walk", HTTP_POST, handleSetWalk);
    server.on("/move-line", HTTP_POST, handleMoveLine);
    server.on("/api/now", HTTP_GET, handleApiNow);
    server.on("/settings", HTTP_GET, handleSettingsPage);
    server.on("/settings", HTTP_POST, handleSettings);
    server.on("/oebb-search", handleOebbSearch);
    server.on("/oebb-save", HTTP_POST, handleOebbSave);
    server.on("/oebb-remove", HTTP_POST, handleOebbRemove);
    server.on("/watch-group-save", HTTP_POST, handleWatchGroupSave);
    server.on("/watch-group-remove", HTTP_POST, handleWatchGroupRemove);
    server.on("/watch-group-edit", handleWatchGroupEdit);
    server.on("/watch-group-update", HTTP_POST, handleWatchGroupUpdate);
    server.on("/wifi-reset", HTTP_POST, handleWifiReset);
    server.on("/factory-reset", HTTP_POST, handleFactoryReset);
    server.on("/update", handleOtaCheck);
    server.on("/update-status", handleOtaStatus);
    server.on("/update-now", handleOtaDoUpdate);
    server.on("/update-progress", handleOtaProgress);
    server.on("/pong", HTTP_GET, handlePongPage);
    server.on("/pong/join",  HTTP_POST, handlePongJoin);
    server.on("/pong/move",  HTTP_POST, handlePongMove);
    server.on("/pong/state", HTTP_GET,  handlePongState);
    server.on("/pong/leave", HTTP_POST, handlePongLeave);
    server.on("/snake",        HTTP_GET,  handleSnakePage);
    server.on("/snake/join",   HTTP_POST, handleSnakeJoin);
    server.on("/snake/move",   HTTP_POST, handleSnakeMove);
    server.on("/snake/state",  HTTP_GET,  handleSnakeState);
    server.on("/snake/leave",  HTTP_POST, handleSnakeLeave);
    server.on("/crash", []() {
        String html = FPSTR(HTML_HEAD);
        html += "<div class='card'><h2>Crash Log</h2>";
        html += "<p class='hint'>boot#" + String((unsigned)rtcBootCount) +
                " &nbsp;|&nbsp; at: " + String(rtcCrumb) +
                " &nbsp;|&nbsp; heap: " + String((unsigned)rtcHeap) + "</p>";
        if (!SPIFFS.exists(CRASH_LOG_PATH)) {
            html += "<p class='status'>Kein Crash gespeichert.</p>";
        } else {
            File f = SPIFFS.open(CRASH_LOG_PATH, "r");
            html += "<pre style='color:#ffbf00;font-size:12px;white-space:pre-wrap;word-break:break-all'>";
            html += f.readString();
            html += "</pre>";
            f.close();
            html += "<form method='post' action='/crash-clear' style='margin-top:8px'>"
                    "<button class='btn-danger'>Log löschen</button></form>";
        }
        html += "</div></body></html>";
        sendHtml(html);
    });
    server.on("/crash-clear", HTTP_POST, []() {
        SPIFFS.remove(CRASH_LOG_PATH);
        server.sendHeader("Location", "/crash");
        server.send(302);
    });
    // Captive portal detection — iOS/Android send these to check for portal
    auto captiveRedirect = []() {
        server.sendHeader("Location", "http://192.168.4.1/", true);
        server.send(302, "text/plain", "");
    };
    server.on("/hotspot-detect.html", captiveRedirect);   // iOS
    server.on("/generate_204", captiveRedirect);           // Android
    server.on("/connecttest.txt", captiveRedirect);        // Windows
    server.onNotFound([captiveRedirect]() {
        if (portalOpen) captiveRedirect();
        else server.send(404, "text/plain", "Not found");
    });
    server.on("/logs", []() {
        String html = "<html><head><meta charset='utf-8'>"
                      "<meta http-equiv='refresh' content='2'>"
                      "<style>body{background:#0a0805;color:#ffbf00;font-family:monospace;"
                      "font-size:13px;padding:12px;white-space:pre-wrap;word-break:break-all}"
                      "a{color:#ffbf00}</style></head><body>"
                      "<a href='/'>← Home</a>  <a href='/logs'>⟳ Refresh</a>\n\n";
        html += getLogContents();
        html += "</body></html>";
        server.send(200, "text/html; charset=utf-8", html);
    });
    server.begin();
    configMode = true;
}

// ── WiFiManager ──────────────────────────────────────────────────────
void showWifiSetupScreen(const char* subtitle) {
    tft.fillScreen(BG_COLOR);
    tft.setTextFont(1);

    // "LineTracker" branding
    tft.setTextColor(AMBER, BG_COLOR);
    tft.setTextSize(3);
    const char* brand = "LineTracker";
    tft.setCursor((320 - tft.textWidth(brand)) / 2, 8);
    tft.print(brand);

    tft.drawFastHLine(60, 35, 200, AMBER_DIM);

    // Subtitle
    tft.setTextColor(AMBER_DIM, BG_COLOR);
    tft.setTextSize(1);
    int sw = tft.textWidth(subtitle);
    tft.setCursor((320 - sw) / 2, 42);
    tft.print(subtitle);

    // Instructions
    tft.setTextColor(AMBER, BG_COLOR);
    tft.setTextSize(2);
    const char* s1 = "1) WLAN verbinden:";
    tft.setCursor((320 - tft.textWidth(s1)) / 2, 56);
    tft.print(s1);

    tft.setTextColor(tft.color565(255, 220, 60), BG_COLOR);
    const char* ap = "\"LineTracker\"";
    tft.setCursor((320 - tft.textWidth(ap)) / 2, 76);
    tft.print(ap);

    tft.setTextColor(AMBER, BG_COLOR);
    const char* s2 = "2) Netzwerk waehlen";
    tft.setCursor((320 - tft.textWidth(s2)) / 2, 100);
    tft.print(s2);

    tft.setTextColor(AMBER_DIM, BG_COLOR);
    tft.setTextSize(1);
    const char* hint = "Nur 2.4 GHz WLAN unterstuetzt";
    tft.setCursor((320 - tft.textWidth(hint)) / 2, 128);
    tft.print(hint);

    // Attribution
    tft.setTextColor(tft.color565(30, 24, 5), BG_COLOR);
    const char* attr = "by Leo Blum";
    tft.setCursor((320 - tft.textWidth(attr)) / 2, 158);
    tft.print(attr);
}

void setupWiFi() {
    // Force 2.4 GHz and maximize compatibility
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

    wm.setConnectRetries(3);              // retry connection up to 3 times
    wm.setMinimumSignalQuality(10);       // accept weak signals

    bool wifiResetRequested = SPIFFS.exists(WIFI_RESET_FLAG);
    if (wifiResetRequested) {
        SPIFFS.remove(WIFI_RESET_FLAG);
    }

    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);

    // Write wifi_ok after any successful portal save (first setup or WiFi change)
    wm.setSaveConfigCallback([]() {
        File f = SPIFFS.open(WIFI_CONFIGURED_FLAG, "w");
        if (f) { f.print("1"); f.close(); }
    });

    if (wifiResetRequested) {
        // Web UI "WLAN ändern" button: change WiFi via blocking portal
        showWifiSetupScreen("WLAN ändern");
        wm.setSaveConnectTimeout(30);
        wm.setConfigPortalTimeout(180);
        wm.startConfigPortal("LineTracker");
        // After portal, continue — dataTask handles reconnect
    } else if (!SPIFFS.exists(WIFI_CONFIGURED_FLAG)) {
        // /wifi_ok missing. Could be a genuine first setup, OR the SPIFFS flag was
        // lost (e.g. filesystem wiped on reflash) while WiFi credentials still live
        // in NVS. If credentials exist, connect directly instead of forcing the
        // setup portal — and recreate the flag so we don't show setup every boot.
        if (wm.getWiFiIsSaved()) {
            WiFi.begin();
            unsigned long start = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
                delay(200);
            }
            if (WiFi.status() == WL_CONNECTED) {
                File f = SPIFFS.open(WIFI_CONFIGURED_FLAG, "w");
                if (f) { f.print("1"); f.close(); }
            } else {
                // Saved credentials no longer work → fall back to setup portal
                showWifiSetupScreen("Ersteinrichtung");
                wm.setSaveConnectTimeout(30);
                wm.setConfigPortalTimeout(0);
                wm.startConfigPortal("LineTracker");
            }
        } else {
            // Truly never configured → first-time setup
            showWifiSetupScreen("Ersteinrichtung");
            wm.setSaveConnectTimeout(30);
            wm.setConfigPortalTimeout(0); // no timeout for first setup
            wm.startConfigPortal("LineTracker");
        }
    } else {
        // Already configured — try to connect, dataTask handles if WiFi is down
        WiFi.begin();
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
            delay(200);
        }
    }
}

// ── Display slot builder (shared by WL and ÖBB fetch) ───────────────
// deps must be sorted by countdown ascending.
// Normal entries: 1 slot per line+direction (soonest).
// Watch entries (watchIdx >= 0): top N per watch, sorted by countdown.
static std::vector<Departure> buildSlots(const std::vector<Departure>& deps) {
    std::vector<Departure> slots;
    std::map<String, bool> seen;
    std::map<int, std::vector<size_t>> watchIdx2pos; // watchIdx → positions in deps
    for (size_t i = 0; i < deps.size(); i++) {
        const auto& d = deps[i];
        if (d.watchIdx >= 0) {
            watchIdx2pos[d.watchIdx].push_back(i);
        } else {
            String key = d.lineName + "|" + d.towards;
            if (seen.find(key) == seen.end()) {
                slots.push_back(d);
                seen[key] = true;
            }
        }
    }
    for (auto& kv : watchIdx2pos) {
        int wi = kv.first;
        if (wi >= (int)cfgWatchGroups.size()) continue;
        int maxN = cfgWatchGroups[wi].maxDepartures;
        for (int n = 0; n < maxN && n < (int)kv.second.size(); n++) {
            slots.push_back(deps[kv.second[n]]);
        }
    }
    if (cfgSortByTime) {
        std::sort(slots.begin(), slots.end(),
            [](const Departure& a, const Departure& b){ return a.countdown < b.countdown; });
    } else {
        // Manual order: follow the configured order (cfgLines, then cfgOebb),
        // watch groups grouped by their index. deps is already countdown-sorted,
        // so stable_sort keeps the soonest departure first within equal ranks.
        auto rankOf = [](const Departure& d) -> int {
            if (d.watchIdx >= 0) return 1000000 + d.watchIdx * 1000;
            for (size_t i = 0; i < cfgLines.size(); i++)
                if (cfgLines[i].name == d.lineName && cfgLines[i].towards == d.towards)
                    return (int)i;
            for (size_t j = 0; j < cfgOebb.size(); j++)
                if (cfgOebb[j].line == d.lineName && cfgOebb[j].towards == d.towards)
                    return 100000 + (int)j;
            return 900000; // unknown → before watch groups, after configured lines
        };
        std::stable_sort(slots.begin(), slots.end(),
            [&](const Departure& a, const Departure& b){ return rankOf(a) < rankOf(b); });
    }
    return slots;
}

// ── API fetch ────────────────────────────────────────────────────────
String buildUrl() {
    String url = "https://www.wienerlinien.at/ogd_realtime/monitor?activateTrafficInfo=stoerunglang";
    std::map<String, bool> seen;
    for (auto& cl : cfgLines) {
        if (!seen[cl.rbl]) { url += "&rbl=" + cl.rbl; seen[cl.rbl] = true; }
    }
    for (auto& g : cfgWatchGroups) {
        for (auto& e : g.entries) {
            if (e.source == "wl" && e.rbl.length() > 0 && !seen[e.rbl]) {
                url += "&rbl=" + e.rbl;
                seen[e.rbl] = true;
            }
        }
    }
    return url;
}

void fetchDepartures() {
    setCrumb("fetchDepartures");
    if (WiFi.status() != WL_CONNECTED) return;
    bool hasWlWatch = false;
    for (auto& g : cfgWatchGroups)
        for (auto& e : g.entries) if (e.source == "wl") { hasWlWatch = true; break; }
    if (cfgLines.empty() && !hasWlWatch) return;

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.begin(client, buildUrl());
    http.setTimeout(15000);
    int code = http.GET();

    if (code != 200) { http.end(); fetchError = true; return; }

    String payload = http.getString();
    http.end();

    JsonDocument filter;
    filter["data"]["monitors"][0]["locationStop"]["properties"]["title"] = true;
    filter["data"]["monitors"][0]["locationStop"]["properties"]["attributes"]["rbl"] = true;
    filter["data"]["monitors"][0]["lines"][0]["name"] = true;
    filter["data"]["monitors"][0]["lines"][0]["towards"] = true;
    filter["data"]["monitors"][0]["lines"][0]["type"] = true;
    filter["data"]["monitors"][0]["lines"][0]["realtimeSupported"] = true;
    filter["data"]["monitors"][0]["lines"][0]["departures"]["departure"][0]["departureTime"]["countdown"] = true;
    filter["data"]["trafficInfos"][0]["title"] = true;

    JsonDocument doc;
    if (deserializeJson(doc, payload, DeserializationOption::Filter(filter),
                        DeserializationOption::NestingLimit(20))) {
        fetchError = true;
        return;
    }

    // Build set of configured (rbl|lineName) pairs for filtering
    std::map<String, bool> cfgLineSet;
    for (auto& cl : cfgLines) cfgLineSet[cl.rbl + "|" + cl.name] = true;

    // Build map: "rbl|lineName|towards" → groupIdx for WL watch groups
    std::map<String, int> watchKeyMap;
    for (size_t gi = 0; gi < cfgWatchGroups.size(); gi++) {
        for (auto& e : cfgWatchGroups[gi].entries) {
            if (e.source == "wl") {
                String key = e.rbl + "|" + e.lineName + "|" + e.towards;
                if (watchKeyMap.find(key) == watchKeyMap.end())
                    watchKeyMap[key] = (int)gi;
            }
        }
    }

    std::vector<Departure> newDeps;
    bool dirCacheUpdated = false;
    JsonArray monitors = doc["data"]["monitors"].as<JsonArray>();
    for (JsonObject monitor : monitors) {
        String stopName = monitor["locationStop"]["properties"]["title"] | "";
        String rbl = String((int)(monitor["locationStop"]["properties"]["attributes"]["rbl"] | 0));
        JsonArray lines = monitor["lines"].as<JsonArray>();
        for (JsonObject line : lines) {
            String name    = line["name"].as<String>();
            String towards = line["towards"].as<String>();
            String type    = line["type"].as<String>();
            bool   rt      = line["realtimeSupported"] | false;

            // Cache direction info from live data
            if (rbl.length() > 0 && name.length() > 0 && towards.length() > 0) {
                size_t szBefore = dirCache[rbl].size();
                cacheDirEntry(rbl, name, towards, type, stopName);
                if (dirCache[rbl].size() != szBefore) dirCacheUpdated = true;
            }

            // Check if configured line or WL watch group entry
            bool isLine = cfgLineSet[rbl + "|" + name];
            int gi = -1;
            {
                auto wit = watchKeyMap.find(rbl + "|" + name + "|" + towards);
                if (wit != watchKeyMap.end()) gi = wit->second;
            }
            if (!isLine && gi < 0) continue;

            // Resolve walkMin: cfgLines for normal, watch entry for watch group
            int wm = 0;
            if (isLine) {
                for (auto& cl : cfgLines) {
                    if (cl.rbl == rbl && cl.name == name) { wm = cl.walkMin; break; }
                }
            } else if (gi >= 0) {
                for (auto& e : cfgWatchGroups[gi].entries) {
                    if (e.source == "wl" && e.rbl == rbl && e.lineName == name && e.towards == towards) {
                        wm = e.walkMin; break;
                    }
                }
            }

            JsonArray deps = line["departures"]["departure"].as<JsonArray>();
            for (JsonObject dep : deps) {
                Departure d;
                d.lineName  = name;
                d.towards   = towards;
                d.type      = type;
                d.countdown = dep["departureTime"]["countdown"] | -1;
                d.realtime  = rt;
                d.watchIdx  = isLine ? -1 : gi;
                d.walkMin   = wm;
                if (d.countdown >= 0) newDeps.push_back(d);
            }
        }
    }

    // Sort all by countdown
    std::sort(newDeps.begin(), newDeps.end(),
        [](const Departure& a, const Departure& b) { return a.countdown < b.countdown; });

    // ── Smart grouping: 1 slot per line+direction + watch top-N ──
    auto newSlots = buildSlots(newDeps);

    // Parse disruptions
    std::vector<String> newDisruptions;
    if (doc["data"]["trafficInfos"].is<JsonArray>()) {
        for (JsonObject info : doc["data"]["trafficInfos"].as<JsonArray>()) {
            String title = info["title"].as<String>();
            title.trim();
            if (title.length() > 0) newDisruptions.push_back(sanitize(title));
        }
    }

    if (dirCacheUpdated) saveDirCache();

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    departures   = newDeps;
    displaySlots = newSlots;
    disruptions  = newDisruptions;
    fetchError   = false;
    xSemaphoreGive(dataMutex);
}

// ── ÖBB S-Bahn / train fetch ─────────────────────────────────────────
String decodeHtmlEntities(String s) {
    s.replace("&#228;", "ä"); s.replace("&#246;", "ö");
    s.replace("&#252;", "ü"); s.replace("&#196;", "Ä");
    s.replace("&#214;", "Ö"); s.replace("&#220;", "Ü");
    s.replace("&#223;", "ß"); s.replace("&#38;", "&");
    s.replace("&amp;", "&"); s.replace("&lt;", "<");
    s.replace("&gt;", ">"); s.replace("&quot;", "\"");
    return s;
}

// Fetch ÖBB departures for a single station, return raw list
std::vector<Departure> fetchOebbStation(const String& stationName, int nowMin) {
    std::vector<Departure> result;
    String encoded = stationName;
    encoded.replace(" ", "+");

    String url = "https://fahrplan.oebb.at/bin/stboard.exe/dn"
                 "?L=vs_scotty.vs_liveticker"
                 "&input=" + encoded +
                 "&boardType=dep"
                 "&productsFilter=0000110000"
                 "&maxJourneys=20"
                 "&outputMode=tickerDataOnly"
                 "&start=yes&time=now&selectDate=today";

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.begin(client, url);
    http.setTimeout(15000);
    int code = http.GET();
    if (code != 200) { http.end(); return result; }

    String payload = http.getString();
    http.end();

    int eqIdx = payload.indexOf('=');
    if (eqIdx < 0) return result;
    String jsonStr = payload.substring(eqIdx + 1);
    jsonStr.trim();
    if (jsonStr.endsWith(";")) jsonStr = jsonStr.substring(0, jsonStr.length() - 1);

    JsonDocument doc;
    if (deserializeJson(doc, jsonStr, DeserializationOption::NestingLimit(15))) return result;

    JsonArray journeys = doc["journey"].as<JsonArray>();
    if (journeys.isNull()) return result;

    for (JsonObject j : journeys) {
        String line = j["pr"].as<String>();
        String dest = j["st"].as<String>();
        String ti   = j["ti"].as<String>();
        dest = decodeHtmlEntities(dest);
        line = decodeHtmlEntities(line);
        line.trim(); dest.trim();

        String depTime = ti;
        if (j["rt"].is<JsonObject>()) {
            String rtTime = j["rt"]["dlt"].as<String>();
            if (rtTime.length() >= 5) depTime = rtTime;
        }

        int col = depTime.indexOf(':');
        if (col < 0) continue;
        int depMin = depTime.substring(0, col).toInt() * 60 + depTime.substring(col + 1).toInt();
        int diff = depMin - nowMin;
        if (diff < -60) diff += 1440;
        if (diff < 0) continue;

        Departure d;
        d.lineName  = line;
        d.towards   = dest;
        d.type      = "ptTrainS";
        d.countdown = diff;
        d.realtime  = j["rt"].is<JsonObject>();
        result.push_back(d);
    }
    return result;
}

void fetchOebbDepartures() {
    setCrumb("fetchOebbDepartures");
    if (WiFi.status() != WL_CONNECTED) return;
    bool hasOebbWatch = false;
    for (auto& g : cfgWatchGroups)
        for (auto& e : g.entries) if (e.source == "oebb") { hasOebbWatch = true; break; }
    if (cfgOebb.empty() && !hasOebbWatch) return;

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 5000)) {
        logf("OeBB: NTP not available\n");
        return;
    }
    int nowMin = timeinfo.tm_hour * 60 + timeinfo.tm_min;

    // Collect unique station names to avoid duplicate API calls
    std::map<String, std::vector<Departure>> stationCache;
    for (auto& os : cfgOebb) {
        if (stationCache.find(os.stationName) == stationCache.end()) {
            stationCache[os.stationName] = fetchOebbStation(os.stationName, nowMin);
        }
    }

    // Filter cached results by configured line+direction
    std::vector<Departure> oebbDeps;
    for (auto& os : cfgOebb) {
        auto& allDeps = stationCache[os.stationName];
        for (auto& d : allDeps) {
            if (d.lineName == os.line && d.towards == os.towards) {
                Departure copy = d;
                copy.walkMin = os.walkMin;
                oebbDeps.push_back(copy);
            }
        }
    }

    // Add ÖBB watch group departures (selected lines, tagged with group index)
    for (size_t gi = 0; gi < cfgWatchGroups.size(); gi++) {
        for (auto& e : cfgWatchGroups[gi].entries) {
            if (e.source != "oebb") continue;
            if (stationCache.find(e.oebbStation) == stationCache.end())
                stationCache[e.oebbStation] = fetchOebbStation(e.oebbStation, nowMin);
            for (auto& d : stationCache[e.oebbStation]) {
                if (d.lineName == e.lineName && d.towards == e.towards) {
                    Departure wd = d;
                    wd.watchIdx = (int)gi;
                    wd.walkMin  = e.walkMin;
                    oebbDeps.push_back(wd);
                    break; // take only soonest matching departure per entry
                }
            }
        }
    }

    if (oebbDeps.empty()) return;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    for (auto& d : oebbDeps) departures.push_back(d);
    std::sort(departures.begin(), departures.end(),
        [](const Departure& a, const Departure& b) { return a.countdown < b.countdown; });
    displaySlots = buildSlots(departures);
    xSemaphoreGive(dataMutex);
}

// ── German chars to ASCII ────────────────────────────────────────────
String sanitize(String s) {
    s.replace("ä", "ae"); s.replace("Ä", "Ae");
    s.replace("ö", "oe"); s.replace("Ö", "Oe");
    s.replace("ü", "ue"); s.replace("Ü", "Ue");
    s.replace("ß", "ss");
    return s;
}

// ── Drawing: Fahrgastinformationssystem (dot-matrix amber LED style) ──
static const int SCREEN_W  = 320;
static const int SCREEN_H  = 170;
static const int PX_MARGIN  = 6;
static const int SEP_H      = 3;
static const int SCROLL_PX  = 2;
static const int SCROLL_GAP = 50;

// Font 1 (5x7 dot-matrix) at different textSize scales
static const int NAME_SZ = 5;  // line name: 5×7=35px visible
static const int DIR_SZ  = 2;  // direction: 2×7=14px visible
static const int CD_SZ   = 4;  // countdown: 4×7=28px visible

// Scroll state per row
static int scrollOffset[MAX_ROWS] = {0, 0, 0};
static String lastScrollText[MAX_ROWS];
static int tickerOffset = 0;  // disruption ticker horizontal scroll

// Page rotation for cycling through groups
static unsigned long lastRotateMs = 0;
static int pageOffset = 0;
static int dispPage   = 0;   // current page in the rotation cycle (departure pages + optional clock page)
// ROTATE_INTERVAL is now dynamic via cfgRotateSec

// Helper: draw glow behind text (1px amber halo for LED bleed)
void drawGlowText(TFT_eSprite& spr, int x, int y, const String& text, uint16_t core, uint16_t glow) {
    spr.setTextColor(glow, BG_COLOR);
    spr.setCursor(x - 1, y); spr.print(text);
    spr.setCursor(x + 1, y); spr.print(text);
    spr.setCursor(x, y - 1); spr.print(text);
    spr.setCursor(x, y + 1); spr.print(text);
    spr.setTextColor(core, BG_COLOR);
    spr.setCursor(x, y); spr.print(text);
}

void drawGlowText(TFT_eSprite& spr, int x, int y, const String& text) {
    drawGlowText(spr, x, y, text, AMBER, AMBER_DIM);
}

// Display tint for a U-Bahn line name when cfgLineColors is on; AMBER otherwise.
// `glowOut` receives a dimmed halo matching the core. Trams/buses stay amber.
uint16_t lineColor565(const String& name, uint16_t& glowOut) {
    glowOut = AMBER_DIM;
    if (!cfgLineColors || name.length() < 2 || name[0] != 'U') return AMBER;
    if (name == "U1") { glowOut = tft.color565(90, 5, 5);  return tft.color565(226, 6, 19);  }
    if (name == "U2") { glowOut = tft.color565(45, 20, 45); return tft.color565(156, 79, 159); }
    if (name == "U3") { glowOut = tft.color565(70, 35, 0);  return tft.color565(239, 124, 0); }
    if (name == "U4") { glowOut = tft.color565(0, 50, 22);  return tft.color565(0, 166, 79);  }
    if (name == "U6") { glowOut = tft.color565(45, 30, 12); return tft.color565(156, 107, 48); }
    return AMBER;
}

// ── Pong: physics tick ───────────────────────────────────────────────
static void resetPongBall(int direction) {
    pong.ballX  = SCREEN_W / 2 - PONG_BALL_SIZE / 2;
    pong.ballY  = SCREEN_H / 2 - PONG_BALL_SIZE / 2;
    pong.ballVX = direction >= 0 ? PONG_BALL_SPEED_START : -PONG_BALL_SPEED_START;
    pong.ballVY = (esp_random() & 1) ? 1 : -1;
    pong.rallyHits = 0;
    for (int i = 0; i < 4; i++) { pong.trailX[i] = pong.ballX; pong.trailY[i] = pong.ballY; }
}

static void resetPongMatch() {
    pong.leftScore  = 0;
    pong.rightScore = 0;
    pong.gameOver   = false;
    pong.winner     = 0;
    pong.gameOverMs = 0;
    pong.lastGoalMs = 0;
    pong.lastGoalSide = 0;
    pong.leftPaddleY  = (SCREEN_H - PONG_PADDLE_H) / 2;
    pong.rightPaddleY = (SCREEN_H - PONG_PADDLE_H) / 2;
    resetPongBall((esp_random() & 1) ? 1 : -1);
    pong.serveAtMs = millis() + 1500;  // initial 1.5s countdown before first serve
}

void pongTick() {
    if (!pongMutex) return;
    if (xSemaphoreTake(pongMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

    unsigned long now = millis();

    // Idle inputs that haven't been refreshed recently — stops paddle drift
    if (pong.left.active && now - pong.left.lastInputMs > 500)  pong.left.inputDir  = 0;
    if (pong.right.active && now - pong.right.lastInputMs > 500) pong.right.inputDir = 0;

    // Slot timeout: a player who hasn't pinged at all for PLAYER_TIMEOUT_MS becomes free
    if (pong.left.active  && now - pong.left.lastSeenMs  > PONG_PLAYER_TIMEOUT_MS) {
        pong.left.active = false; pong.left.token[0] = 0; pong.left.inputDir = 0;
    }
    if (pong.right.active && now - pong.right.lastSeenMs > PONG_PLAYER_TIMEOUT_MS) {
        pong.right.active = false; pong.right.token[0] = 0; pong.right.inputDir = 0;
    }

    // Both gone for ABANDON_TIMEOUT_MS → back to departures, clean slate
    if (!pong.left.active && !pong.right.active) {
        unsigned long lastSeen = max(pong.left.lastSeenMs, pong.right.lastSeenMs);
        if (lastSeen > 0 && now - lastSeen > PONG_ABANDON_TIMEOUT_MS) {
            resetPongMatch();
            pong.gameRunning = false;
            appMode = MODE_DEPARTURES;
        }
    }

    // Game starts only when both slots are taken
    bool bothActive = pong.left.active && pong.right.active;
    if (bothActive && !pong.gameRunning && !pong.gameOver) {
        resetPongMatch();
        pong.gameRunning = true;
    }
    if (!bothActive) {
        pong.gameRunning = false;
    }

    // Game-over hold: show banner for PONG_GAMEOVER_HOLD_MS, then start a new match
    if (pong.gameOver && now - pong.gameOverMs > PONG_GAMEOVER_HOLD_MS) {
        if (bothActive) {
            resetPongMatch();
            pong.gameRunning = true;
        } else {
            pong.gameOver = false;
            pong.winner = 0;
        }
    }

    if (pong.gameRunning && !pong.gameOver) {
        // Paddel-Update — works even during serve freeze so players can position
        pong.leftPaddleY  += pong.left.inputDir  * PONG_PADDLE_SPEED;
        pong.rightPaddleY += pong.right.inputDir * PONG_PADDLE_SPEED;
        if (pong.leftPaddleY  < 0) pong.leftPaddleY  = 0;
        if (pong.rightPaddleY < 0) pong.rightPaddleY = 0;
        if (pong.leftPaddleY  > SCREEN_H - PONG_PADDLE_H) pong.leftPaddleY  = SCREEN_H - PONG_PADDLE_H;
        if (pong.rightPaddleY > SCREEN_H - PONG_PADDLE_H) pong.rightPaddleY = SCREEN_H - PONG_PADDLE_H;

        // Serve freeze: ball stays still while goal animation / countdown plays
        bool ballFrozen = (pong.serveAtMs > now);

        if (!ballFrozen) {
            // Trail (record before move so trail trails the new position)
            pong.trailX[pong.trailIdx] = pong.ballX;
            pong.trailY[pong.trailIdx] = pong.ballY;
            pong.trailIdx = (pong.trailIdx + 1) & 0x3;

            // Ball-Update
            pong.ballX += pong.ballVX;
            pong.ballY += pong.ballVY;

            // Wall bounce (top/bottom)
            if (pong.ballY <= 0) { pong.ballY = 0; pong.ballVY = -pong.ballVY; }
            if (pong.ballY >= SCREEN_H - PONG_BALL_SIZE) {
                pong.ballY = SCREEN_H - PONG_BALL_SIZE;
                pong.ballVY = -pong.ballVY;
            }

            // Paddle collision
            const int leftPaddleX  = PX_MARGIN;
            const int rightPaddleX = SCREEN_W - PX_MARGIN - PONG_PADDLE_W;

            if (pong.ballVX < 0 &&
                pong.ballX <= leftPaddleX + PONG_PADDLE_W &&
                pong.ballX + PONG_BALL_SIZE >= leftPaddleX &&
                pong.ballY + PONG_BALL_SIZE >= pong.leftPaddleY &&
                pong.ballY <= pong.leftPaddleY + PONG_PADDLE_H) {
                pong.ballX = leftPaddleX + PONG_PADDLE_W;
                pong.ballVX = -pong.ballVX;
                if (pong.ballVX < PONG_BALL_SPEED_MAX) pong.ballVX += 1;   // accelerate
                int hitOffset = (pong.ballY + PONG_BALL_SIZE / 2) - (pong.leftPaddleY + PONG_PADDLE_H / 2);
                pong.ballVY += hitOffset / 8;
                if (pong.ballVY > 4)  pong.ballVY = 4;
                if (pong.ballVY < -4) pong.ballVY = -4;
                pong.leftHitMs = now;
                pong.rallyHits++;
            }

            if (pong.ballVX > 0 &&
                pong.ballX + PONG_BALL_SIZE >= rightPaddleX &&
                pong.ballX <= rightPaddleX + PONG_PADDLE_W &&
                pong.ballY + PONG_BALL_SIZE >= pong.rightPaddleY &&
                pong.ballY <= pong.rightPaddleY + PONG_PADDLE_H) {
                pong.ballX = rightPaddleX - PONG_BALL_SIZE;
                pong.ballVX = -pong.ballVX;
                if (pong.ballVX > -PONG_BALL_SPEED_MAX) pong.ballVX -= 1;  // accelerate (negative)
                int hitOffset = (pong.ballY + PONG_BALL_SIZE / 2) - (pong.rightPaddleY + PONG_PADDLE_H / 2);
                pong.ballVY += hitOffset / 8;
                if (pong.ballVY > 4)  pong.ballVY = 4;
                if (pong.ballVY < -4) pong.ballVY = -4;
                pong.rightHitMs = now;
                pong.rallyHits++;
            }

            // Goal
            if (pong.ballX + PONG_BALL_SIZE < 0) {
                pong.rightScore++;
                pong.lastGoalMs = now;
                pong.lastGoalSide = 2;
                if (pong.rightScore >= PONG_SCORE_TO_WIN) {
                    pong.gameOver = true; pong.winner = 2; pong.gameOverMs = now;
                    pong.gameRunning = false;
                } else {
                    resetPongBall(-1);
                    pong.serveAtMs = now + PONG_GOAL_ANIM_MS;
                }
            } else if (pong.ballX > SCREEN_W) {
                pong.leftScore++;
                pong.lastGoalMs = now;
                pong.lastGoalSide = 1;
                if (pong.leftScore >= PONG_SCORE_TO_WIN) {
                    pong.gameOver = true; pong.winner = 1; pong.gameOverMs = now;
                    pong.gameRunning = false;
                } else {
                    resetPongBall(1);
                    pong.serveAtMs = now + PONG_GOAL_ANIM_MS;
                }
            }
        }
    }

    xSemaphoreGive(pongMutex);
}

// ── Pong: render helpers ────────────────────────────────────────────
// Draws a paddle styled as a Wiener-U-Bahn-Waggon (12x42 px).
//   facingRight = true → "front" / headlight on right side (left paddle)
//   facingRight = false → headlight on left side (right paddle)
//   bright = true → recently hit, draw with glow halo
static void drawUBahnWaggon(int x, int y, bool facingRight, bool dim, bool bright) {
    uint16_t main   = dim ? AMBER_DIM : AMBER;
    uint16_t accent = dim ? AMBER_DIM : AMBER;

    // Bright glow halo when recently hit
    if (bright) {
        sprite.drawRect(x - 1, y - 1, PONG_PADDLE_W + 2, PONG_PADDLE_H + 2, AMBER);
        sprite.drawRect(x - 2, y - 2, PONG_PADDLE_W + 4, PONG_PADDLE_H + 4, AMBER_DIM);
    }

    // Body outline (rounded corners simulated by skipping corner pixels)
    sprite.fillRect(x, y, PONG_PADDLE_W, PONG_PADDLE_H, BG_COLOR);
    sprite.drawRect(x,     y,     PONG_PADDLE_W,     PONG_PADDLE_H,     main);
    sprite.drawRect(x + 1, y + 1, PONG_PADDLE_W - 2, PONG_PADDLE_H - 2, main);
    // Corner trims
    sprite.drawPixel(x,                   y,                   BG_COLOR);
    sprite.drawPixel(x + PONG_PADDLE_W-1, y,                   BG_COLOR);
    sprite.drawPixel(x,                   y + PONG_PADDLE_H-1, BG_COLOR);
    sprite.drawPixel(x + PONG_PADDLE_W-1, y + PONG_PADDLE_H-1, BG_COLOR);

    // 4 windows down the side, each 6 wide x 5 tall
    int winX = x + 3;
    int winYStart = y + 5;
    for (int i = 0; i < 4; i++) {
        int wy = winYStart + i * 9;
        if (wy + 5 > y + PONG_PADDLE_H - 5) break;
        sprite.fillRect(winX, wy, 6, 5, accent);
    }

    // Headlight strip on the front side
    int headX = facingRight ? (x + PONG_PADDLE_W - 2) : x;
    sprite.fillRect(headX, y + PONG_PADDLE_H / 2 - 6, 2, 12, AMBER);
}

// Draw the ball as a "Fahrgast" — body + head, recognizable as a person.
static void drawFahrgast(int cx, int cy, uint16_t col) {
    // Body (6x5 oval)
    sprite.fillRect(cx - 3, cy - 1, 6, 5, col);
    sprite.drawPixel(cx - 3, cy - 1, BG_COLOR);
    sprite.drawPixel(cx + 2, cy - 1, BG_COLOR);
    sprite.drawPixel(cx - 3, cy + 3, BG_COLOR);
    sprite.drawPixel(cx + 2, cy + 3, BG_COLOR);
    // Shoulders (slightly wider top of body)
    sprite.drawFastHLine(cx - 3, cy, 6, col);
    // Head (small block above)
    sprite.fillRect(cx - 1, cy - 4, 3, 3, col);
}

// ── Pong: render scene ───────────────────────────────────────────────
void drawPongScene() {
    PongState s;
    bool bothActive = false;
    if (pongMutex && xSemaphoreTake(pongMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s = pong;
        bothActive = pong.left.active && pong.right.active;
        xSemaphoreGive(pongMutex);
    } else {
        s = pong;
    }

    unsigned long now = millis();
    bool inGoalAnim = s.lastGoalMs > 0 && now - s.lastGoalMs < PONG_GOAL_ANIM_MS;
    unsigned long sinceGoal = inGoalAnim ? (now - s.lastGoalMs) : 0;

    // Goal flash: full-screen amber pulse (twice, ~70ms each, in first 200ms)
    bool goalFlash = inGoalAnim && (sinceGoal < 70 || (sinceGoal > 90 && sinceGoal < 160));
    sprite.fillSprite(goalFlash ? AMBER_DIM : BG_COLOR);

    // U-Bahn track rails (chunky dashed top/bottom)
    for (int x = 0; x < SCREEN_W; x += 8) {
        sprite.fillRect(x,     2,             5, 2, AMBER_DIM);
        sprite.fillRect(x,     SCREEN_H - 4,  5, 2, AMBER_DIM);
    }

    // Center dashed line
    for (int y = 22; y < SCREEN_H - 8; y += 10) {
        sprite.fillRect(SCREEN_W / 2 - 1, y, 2, 5, AMBER_DIM);
    }

    // Score header — Stationsname + number
    sprite.setTextFont(1);
    sprite.setTextSize(1);
    int stationCount = (int)(sizeof(PONG_STATIONS) / sizeof(PONG_STATIONS[0]));
    int leftIdx  = s.leftScore  < stationCount ? s.leftScore  : stationCount - 1;
    int rightIdx = s.rightScore < stationCount ? s.rightScore : stationCount - 1;

    bool pulseLeft  = inGoalAnim && s.lastGoalSide == 1 && (sinceGoal / 100) % 2 == 0;
    bool pulseRight = inGoalAnim && s.lastGoalSide == 2 && (sinceGoal / 100) % 2 == 0;

    String leftName  = String(s.leftScore)  + ":" + String(PONG_STATIONS[leftIdx]);
    String rightName = String(PONG_STATIONS[rightIdx]) + ":" + String(s.rightScore);
    sprite.setTextColor(pulseLeft ? AMBER_DIM : AMBER, BG_COLOR);
    sprite.setCursor(8, 8); sprite.print(leftName);
    int rw = sprite.textWidth(rightName);
    sprite.setTextColor(pulseRight ? AMBER_DIM : AMBER, BG_COLOR);
    sprite.setCursor(SCREEN_W - rw - 8, 8); sprite.print(rightName);

    // Win sequence (≥ 6.5s hold): U-Bahn slides into Heiligenstadt
    if (s.gameOver) {
        sprite.setTextColor(AMBER, BG_COLOR);

        unsigned long elapsed = now - s.gameOverMs;

        // Phase 1: train slides in from the loser's side toward winner's side
        // (winner=1: train enters from right, slides left → "arrives" at left=Heiligenstadt sign)
        // (winner=2: train enters from left, slides right)
        const int trainW = 110;
        const int trainH = 48;
        int trainY = SCREEN_H / 2 - trainH / 2;
        int slideMs = 1800;
        int progress = elapsed > (unsigned long)slideMs ? slideMs : (int)elapsed;
        int trainX;
        if (s.winner == 1) {
            // start off-screen right, end at left margin
            int startX = SCREEN_W;
            int endX   = 12;
            trainX = startX + (endX - startX) * progress / slideMs;
        } else {
            int startX = -trainW;
            int endX   = SCREEN_W - trainW - 12;
            trainX = startX + (endX - startX) * progress / slideMs;
        }

        // Draw a 3-waggon train
        for (int i = 0; i < 3; i++) {
            int wx = trainX + i * 38;
            // body
            sprite.fillRect(wx, trainY, 34, trainH, BG_COLOR);
            sprite.drawRect(wx, trainY, 34, trainH, AMBER);
            sprite.drawRect(wx + 1, trainY + 1, 32, trainH - 2, AMBER);
            // windows
            for (int j = 0; j < 3; j++) {
                sprite.fillRect(wx + 4 + j * 9, trainY + 6, 7, 12, AMBER);
            }
            // bottom wheels
            sprite.fillRect(wx + 4,  trainY + trainH - 4, 6, 3, AMBER_DIM);
            sprite.fillRect(wx + 24, trainY + trainH - 4, 6, 3, AMBER_DIM);
            // doors
            sprite.fillRect(wx + 4, trainY + 22, 26, 18, BG_COLOR);
            sprite.drawRect(wx + 4, trainY + 22, 26, 18, AMBER);
            sprite.drawFastVLine(wx + 17, trainY + 22, 18, AMBER);
        }
        // Headlight on the leading waggon
        if (s.winner == 1) {
            sprite.fillRect(trainX - 2, trainY + trainH/2 - 4, 3, 8, AMBER);
        } else {
            sprite.fillRect(trainX + 3 * 38 + 32, trainY + trainH/2 - 4, 3, 8, AMBER);
        }

        // Phase 2: Endstation banner overlay (after slide finishes)
        if (elapsed > 1800) {
            sprite.setTextSize(2);
            const char* line1 = "ENDSTATION";
            int tw1 = sprite.textWidth(line1);
            drawGlowText(sprite, (SCREEN_W - tw1) / 2, 18, line1);

            sprite.setTextSize(2);
            const char* line2 = "HEILIGENSTADT";
            int tw2 = sprite.textWidth(line2);
            drawGlowText(sprite, (SCREEN_W - tw2) / 2, 38, line2);

            sprite.setTextSize(1);
            String line3 = String(s.winner == 1 ? "LINKS" : "RECHTS") + " gewinnt!";
            int tw3 = sprite.textWidth(line3);
            drawGlowText(sprite, (SCREEN_W - tw3) / 2, SCREEN_H - 22, line3);
        }
        return;
    }

    // Trail (3 most recent past positions, fading)
    if (bothActive && s.gameRunning && !inGoalAnim) {
        for (int i = 1; i <= 3; i++) {
            uint8_t idx = (uint8_t)((s.trailIdx - i) & 0x3);
            int tx = s.trailX[idx] + PONG_BALL_SIZE / 2;
            int ty = s.trailY[idx] + PONG_BALL_SIZE / 2;
            int r  = 4 - i;
            if (r > 0) sprite.fillRect(tx - r, ty - r, r * 2, r * 2, AMBER_DIM);
        }
    }

    // Paddles (U-Bahn-Waggons)
    bool leftBright  = (now - s.leftHitMs)  < PONG_HIT_FLASH_MS;
    bool rightBright = (now - s.rightHitMs) < PONG_HIT_FLASH_MS;
    drawUBahnWaggon(PX_MARGIN, s.leftPaddleY, true,  !s.left.active,  leftBright);
    drawUBahnWaggon(SCREEN_W - PX_MARGIN - PONG_PADDLE_W, s.rightPaddleY, false, !s.right.active, rightBright);

    // Ball = Fahrgast — only show if not in goal-flash window (so flash reads as goal)
    bool ballHidden = inGoalAnim && sinceGoal < 200;
    if (!ballHidden) {
        int bcx = s.ballX + PONG_BALL_SIZE / 2;
        int bcy = s.ballY + PONG_BALL_SIZE / 2;
        // Glow halo
        sprite.fillRect(bcx - 5, bcy - 6, 10, 12, AMBER_DIM);
        drawFahrgast(bcx, bcy, AMBER);
    }

    // Overlays
    sprite.setTextFont(1);

    if (!bothActive) {
        sprite.setTextSize(2);
        const char* msg = "Warte auf Mitspieler...";
        int tw = sprite.textWidth(msg);
        drawGlowText(sprite, (SCREEN_W - tw) / 2, SCREEN_H / 2 - 12, msg);
        sprite.setTextSize(1);
        const char* hint = "linetracker.local/pong";
        int hw = sprite.textWidth(hint);
        sprite.setTextColor(AMBER_DIM, BG_COLOR);
        sprite.setCursor((SCREEN_W - hw) / 2, SCREEN_H / 2 + 12);
        sprite.print(hint);
        return;
    }

    // Goal animation: big EINFAHRT! text, then 3-2-1 countdown before serve
    if (inGoalAnim) {
        sprite.setTextSize(3);
        const char* msg = "EINFAHRT!";
        int tw = sprite.textWidth(msg);
        // grow effect: shift up slightly during first 400ms
        int yOff = sinceGoal < 400 ? (int)(sinceGoal / 100) : 4;
        drawGlowText(sprite, (SCREEN_W - tw) / 2, SCREEN_H / 2 - 12 - (4 - yOff), msg);
    } else if (s.serveAtMs > now) {
        unsigned long remain = s.serveAtMs - now;
        int n = (int)((remain + 333) / 333);
        if (n < 1) n = 1; if (n > 3) n = 3;
        sprite.setTextSize(5);
        String num = String(n);
        int tw = sprite.textWidth(num);
        drawGlowText(sprite, (SCREEN_W - tw) / 2, SCREEN_H / 2 - 18, num);
    }
}

// ── Snake: physics tick ─────────────────────────────────────────────
void snakeTick() {
    if (!snakeMutex) return;
    if (xSemaphoreTake(snakeMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

    unsigned long now = millis();

    // Player timeout — slot becomes free
    if (snake.active && now - snake.lastSeenMs > SNAKE_PLAYER_TIMEOUT_MS) {
        snake.active = false; snake.token[0] = 0;
    }

    // No player and abandoned long enough → back to departures
    if (!snake.active) {
        if (snake.lastSeenMs > 0 && now - snake.lastSeenMs > SNAKE_ABANDON_TIMEOUT_MS) {
            snake.gameRunning = false;
            snake.gameOver    = false;
            appMode = MODE_DEPARTURES;
        }
        xSemaphoreGive(snakeMutex);
        return;
    }

    // Game-over hold; auto-restart while player is still here
    if (snake.gameOver) {
        if (now - snake.gameOverMs > SNAKE_GAMEOVER_HOLD_MS) {
            snakeResetMatch();
            snake.gameRunning = true;
        }
        xSemaphoreGive(snakeMutex);
        return;
    }

    if (!snake.gameRunning) {
        xSemaphoreGive(snakeMutex);
        return;
    }

    if (now - snake.lastTickMs < (unsigned long)snake.tickIntervalMs) {
        xSemaphoreGive(snakeMutex);
        return;
    }
    snake.lastTickMs = now;

    // Apply pending direction (already validated against reversal)
    snake.dir = snake.pendingDir;

    // Compute new head
    int hx = snake.body[snake.length - 1].x;
    int hy = snake.body[snake.length - 1].y;
    switch (snake.dir) {
        case 0: hx += 1; break;
        case 1: hy += 1; break;
        case 2: hx -= 1; break;
        case 3: hy -= 1; break;
    }

    // Wall collision
    if (hx < 0 || hx >= SNAKE_COLS || hy < 0 || hy >= SNAKE_ROWS) {
        snake.gameOver     = true;
        snake.gameRunning  = false;
        snake.gameOverMs   = now;
        snake.deathFlashMs = now;
        if (snake.score > snake.highScore) snake.highScore = snake.score;
        xSemaphoreGive(snakeMutex);
        return;
    }

    // Self collision (skip the tail because it will move out this tick UNLESS we're eating)
    bool eating = (hx == snake.food.x && hy == snake.food.y);
    int selfStart = eating ? 0 : 1;
    for (int i = selfStart; i < snake.length; i++) {
        if (snake.body[i].x == hx && snake.body[i].y == hy) {
            snake.gameOver     = true;
            snake.gameRunning  = false;
            snake.gameOverMs   = now;
            snake.deathFlashMs = now;
            if (snake.score > snake.highScore) snake.highScore = snake.score;
            xSemaphoreGive(snakeMutex);
            return;
        }
    }

    if (eating) {
        // Grow: append head
        if (snake.length < SNAKE_MAX_LEN) {
            snake.body[snake.length].x = (int8_t)hx;
            snake.body[snake.length].y = (int8_t)hy;
            snake.length++;
        }
        snake.score++;
        // Speed up every 3 fahrgäste
        if (snake.score % 3 == 0 && snake.tickIntervalMs > SNAKE_TICK_MIN_MS) {
            snake.tickIntervalMs -= 15;
            if (snake.tickIntervalMs < SNAKE_TICK_MIN_MS) snake.tickIntervalMs = SNAKE_TICK_MIN_MS;
        }
        snakePlaceFood();
    } else {
        // Shift: drop tail, push new head
        for (int i = 0; i < snake.length - 1; i++) snake.body[i] = snake.body[i + 1];
        snake.body[snake.length - 1].x = (int8_t)hx;
        snake.body[snake.length - 1].y = (int8_t)hy;
    }

    xSemaphoreGive(snakeMutex);
}

// ── Snake: render scene ─────────────────────────────────────────────
void drawSnakeScene() {
    SnakeState s;
    if (snakeMutex && xSemaphoreTake(snakeMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s = snake;
        xSemaphoreGive(snakeMutex);
    } else {
        s = snake;
    }

    unsigned long now = millis();
    bool deathFlash = s.deathFlashMs > 0 && now - s.deathFlashMs < 500
                      && ((now - s.deathFlashMs) / 80) % 2 == 0;
    sprite.fillSprite(deathFlash ? AMBER_DIM : BG_COLOR);

    // Header bar — score + best
    sprite.setTextFont(1);
    sprite.setTextSize(1);
    sprite.setTextColor(AMBER, BG_COLOR);
    sprite.setCursor(8, 5);
    sprite.printf("SCORE %d", s.score);
    String best = "BEST " + String(s.highScore);
    int bw = sprite.textWidth(best);
    sprite.setCursor(SCREEN_W - bw - 8, 5);
    sprite.print(best);
    sprite.drawFastHLine(0, SNAKE_HEADER - 2, SCREEN_W, AMBER_DIM);

    // Track rails — top and bottom of grid
    int gridY0 = SNAKE_HEADER;
    int gridY1 = gridY0 + SNAKE_ROWS * SNAKE_CELL;
    for (int x = 0; x < SCREEN_W; x += 8) {
        sprite.fillRect(x, gridY1 + 1, 5, 2, AMBER_DIM);
    }

    // Faint grid dots so the play area reads as a track
    for (int gy = 0; gy < SNAKE_ROWS; gy++) {
        for (int gx = 0; gx < SNAKE_COLS; gx += 2) {
            int px = gx * SNAKE_CELL + SNAKE_CELL / 2;
            int py = gridY0 + gy * SNAKE_CELL + SNAKE_CELL / 2;
            if (((gx + gy) & 1) == 0) sprite.drawPixel(px, py, AMBER_DIM);
        }
    }

    // Food = Fahrgast (small body+head)
    {
        int cx = s.food.x * SNAKE_CELL + SNAKE_CELL / 2;
        int cy = gridY0 + s.food.y * SNAKE_CELL + SNAKE_CELL / 2;
        // Pulsing halo
        bool pulse = (now / 300) % 2 == 0;
        if (pulse) sprite.fillRect(cx - 4, cy - 4, 8, 8, AMBER_DIM);
        // Body
        sprite.fillRect(cx - 2, cy - 1, 5, 4, AMBER);
        // Head
        sprite.fillRect(cx - 1, cy - 4, 3, 3, AMBER);
    }

    // Snake body — first segment (tail) is small/dim, head is bright with headlight
    for (int i = 0; i < s.length; i++) {
        int x = s.body[i].x * SNAKE_CELL;
        int y = gridY0 + s.body[i].y * SNAKE_CELL;
        bool isHead = (i == s.length - 1);
        bool isTail = (i == 0);
        uint16_t col = (isHead || s.length - i <= 3) ? AMBER : AMBER_DIM;

        // Waggon body
        int pad = isTail ? 2 : 1;
        sprite.fillRect(x + pad, y + pad, SNAKE_CELL - 2 * pad, SNAKE_CELL - 2 * pad, BG_COLOR);
        sprite.drawRect(x + pad, y + pad, SNAKE_CELL - 2 * pad, SNAKE_CELL - 2 * pad, col);

        // Window in the middle
        if (!isTail) sprite.fillRect(x + 4, y + 4, 2, 2, col);

        // Headlight on the head
        if (isHead) {
            switch (s.dir) {
                case 0: sprite.fillRect(x + SNAKE_CELL - 2, y + SNAKE_CELL/2 - 1, 2, 3, AMBER); break;
                case 1: sprite.fillRect(x + SNAKE_CELL/2 - 1, y + SNAKE_CELL - 2, 3, 2, AMBER); break;
                case 2: sprite.fillRect(x,                   y + SNAKE_CELL/2 - 1, 2, 3, AMBER); break;
                case 3: sprite.fillRect(x + SNAKE_CELL/2 - 1, y,                   3, 2, AMBER); break;
            }
        }
    }

    // Overlays
    if (!s.active) {
        sprite.setTextSize(2);
        const char* msg = "Warte auf Spieler...";
        int tw = sprite.textWidth(msg);
        drawGlowText(sprite, (SCREEN_W - tw) / 2, SCREEN_H / 2 - 12, msg);
        sprite.setTextSize(1);
        const char* hint = "linetracker.local/snake";
        int hw = sprite.textWidth(hint);
        sprite.setTextColor(AMBER_DIM, BG_COLOR);
        sprite.setCursor((SCREEN_W - hw) / 2, SCREEN_H / 2 + 12);
        sprite.print(hint);
        return;
    }

    if (s.gameOver) {
        // Dim overlay + Endstation banner
        sprite.fillRect(0, gridY0 + 20, SCREEN_W, 70, BG_COLOR);
        sprite.drawRect(0, gridY0 + 20, SCREEN_W, 70, AMBER);
        sprite.drawRect(1, gridY0 + 21, SCREEN_W - 2, 68, AMBER_DIM);
        sprite.setTextSize(3);
        const char* line1 = "ENDSTATION";
        int tw = sprite.textWidth(line1);
        drawGlowText(sprite, (SCREEN_W - tw) / 2, gridY0 + 30, line1);
        sprite.setTextSize(1);
        String sub = "Score " + String(s.score) + " - startet neu";
        int sw = sprite.textWidth(sub);
        sprite.setTextColor(AMBER_DIM, BG_COLOR);
        sprite.setCursor((SCREEN_W - sw) / 2, gridY0 + 70);
        sprite.print(sub);
    }
}

// Big clock page (drawn into the global sprite, which the caller pushes).
void drawClockScreen(const struct tm& ti) {
    char hhmm[6];
    snprintf(hhmm, sizeof(hhmm), "%02d:%02d", ti.tm_hour, ti.tm_min);
    sprite.setTextFont(1);
    sprite.setTextColor(AMBER, BG_COLOR);
    sprite.setTextSize(7);
    int w = sprite.textWidth(hhmm);
    drawGlowText(sprite, (SCREEN_W - w) / 2, 35, hhmm);

    static const char* wd[7] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"};
    char datestr[24];
    snprintf(datestr, sizeof(datestr), "%s %02d.%02d.%04d",
             wd[ti.tm_wday % 7], ti.tm_mday, ti.tm_mon + 1, ti.tm_year + 1900);
    sprite.setTextSize(2);
    sprite.setTextColor(AMBER_DIM, BG_COLOR);
    int dw = sprite.textWidth(datestr);
    sprite.setCursor((SCREEN_W - dw) / 2, 120);
    sprite.print(datestr);
}

// ── Weather (Vienna, Open-Meteo, no API key) ─────────────────────────
static float weatherTemp  = 0;
static int   weatherCode  = -1;
static bool  weatherValid = false;
static unsigned long lastWeatherFetch = 0;
static const unsigned long WEATHER_INTERVAL_MS = 15UL * 60 * 1000; // 15 min

// WMO weather code → short German label.
static const char* weatherText(int code) {
    if (code == 0)                   return "Klar";
    if (code >= 1 && code <= 3)      return "Bewoelkt";
    if (code == 45 || code == 48)    return "Nebel";
    if (code >= 51 && code <= 57)    return "Nieselregen";
    if (code >= 61 && code <= 67)    return "Regen";
    if (code >= 71 && code <= 77)    return "Schnee";
    if (code >= 80 && code <= 82)    return "Schauer";
    if (code == 85 || code == 86)    return "Schneeschauer";
    if (code >= 95)                  return "Gewitter";
    return "";
}

void fetchWeather() {
    if (WiFi.status() != WL_CONNECTED) return;
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.begin(client,
        "https://api.open-meteo.com/v1/forecast?latitude=48.21&longitude=16.37"
        "&current=temperature_2m,weather_code&timezone=Europe%2FVienna");
    http.setTimeout(15000);
    int code = http.GET();
    if (code != 200) { http.end(); return; }
    String payload = http.getString();
    http.end();

    JsonDocument doc;
    if (deserializeJson(doc, payload)) return;
    if (!doc["current"]["temperature_2m"].is<float>() &&
        !doc["current"]["temperature_2m"].is<int>()) return;
    weatherTemp  = doc["current"]["temperature_2m"] | 0.0f;
    weatherCode  = doc["current"]["weather_code"]   | -1;
    weatherValid = true;
    logf("Weather: %.1f C, code %d\n", weatherTemp, weatherCode);
}

// Weather page (drawn into the global sprite, which the caller pushes).
// Layout: big temperature number + a hand-drawn degree ring + "C", since the
// GLCD font 1 doesn't render '°' reliably.
void drawWeatherScreen() {
    char tstr[8];
    snprintf(tstr, sizeof(tstr), "%.0f", weatherTemp);
    sprite.setTextFont(1);

    sprite.setTextSize(7);
    int numW = sprite.textWidth(tstr);
    sprite.setTextSize(4);
    int cW   = sprite.textWidth("C");
    const int ringR = 6, gap1 = 10, gap2 = 6;
    int totalW = numW + gap1 + ringR * 2 + gap2 + cW;
    int x = (SCREEN_W - totalW) / 2;

    sprite.setTextSize(7);
    drawGlowText(sprite, x, 35, String(tstr));

    int sx = x + numW + gap1;
    sprite.drawCircle(sx + ringR, 45, ringR, AMBER);
    sprite.drawCircle(sx + ringR, 45, ringR - 1, AMBER);
    sprite.setTextSize(4);
    sprite.setTextColor(AMBER, BG_COLOR);
    sprite.setCursor(sx + ringR * 2 + gap2, 45);
    sprite.print("C");

    const char* cond = weatherText(weatherCode);
    sprite.setTextSize(2);
    sprite.setTextColor(AMBER_DIM, BG_COLOR);
    int cw = sprite.textWidth(cond);
    sprite.setCursor((SCREEN_W - cw) / 2, 120);
    sprite.print(cond);

    sprite.setTextSize(1);
    const char* loc = "Wien";
    sprite.setCursor((SCREEN_W - sprite.textWidth(loc)) / 2, 150);
    sprite.print(loc);
}

void drawDisplay() {
    // Standby: panel is asleep, skip all rendering (games keep it awake).
    if (standbyActive && appMode == MODE_DEPARTURES) return;

    sprite.createSprite(SCREEN_W, SCREEN_H);
    sprite.fillSprite(BG_COLOR);
    sprite.setTextFont(1);

    // ── Pong takeover ──
    if (appMode == MODE_PONG) {
        drawPongScene();
        sprite.pushSprite(0, 0);
        sprite.deleteSprite();
        return;
    }

    // ── Snake takeover ──
    if (appMode == MODE_SNAKE) {
        drawSnakeScene();
        sprite.pushSprite(0, 0);
        sprite.deleteSprite();
        return;
    }

    // ── OTA update progress screen ──
    if (otaInProgress) {
        sprite.setTextColor(AMBER, BG_COLOR);
        sprite.setTextSize(3);
        const char* brand = "LineTracker";
        int tw = sprite.textWidth(brand);
        sprite.setCursor((SCREEN_W - tw) / 2, 6);
        sprite.print(brand);

        sprite.drawFastHLine(40, 38, 240, AMBER_DIM);

        sprite.setTextSize(2);
        const char* msg = "Firmware Update";
        tw = sprite.textWidth(msg);
        sprite.setCursor((SCREEN_W - tw) / 2, 48);
        sprite.print(msg);

        sprite.setTextColor(AMBER_DIM, BG_COLOR);
        sprite.setTextSize(1);
        String verStr = "v" + String(FW_VERSION) + " -> v" + otaNewVersion;
        tw = sprite.textWidth(verStr);
        sprite.setCursor((SCREEN_W - tw) / 2, 72);
        sprite.print(verStr);

        // Progress bar
        int barX = 30, barY = 92, barW = 260, barH = 16;
        sprite.drawRect(barX, barY, barW, barH, AMBER_DIM);
        int fillW = (barW - 4) * otaPercent / 100;
        if (fillW > 0) sprite.fillRect(barX + 2, barY + 2, fillW, barH - 4, AMBER);

        // Percentage
        sprite.setTextColor(AMBER, BG_COLOR);
        sprite.setTextSize(2);
        String pctStr = String(otaPercent) + "%";
        tw = sprite.textWidth(pctStr);
        sprite.setCursor((SCREEN_W - tw) / 2, 118);
        sprite.print(pctStr);

        sprite.setTextColor(AMBER_DIM, BG_COLOR);
        sprite.setTextSize(1);
        const char* warn = "Nicht ausschalten!";
        tw = sprite.textWidth(warn);
        sprite.setCursor((SCREEN_W - tw) / 2, 150);
        sprite.print(warn);

        sprite.pushSprite(0, 0);
        sprite.deleteSprite();
        return;
    }

    // ── WiFi down screen ──
    if (WiFi.status() != WL_CONNECTED) {
        sprite.setTextColor(AMBER, BG_COLOR);
        sprite.setTextSize(3);
        const char* title = "Kein WLAN";
        int tw = sprite.textWidth(title);
        sprite.setCursor((SCREEN_W - tw) / 2, 18);
        sprite.print(title);

        sprite.drawFastHLine(40, 52, 240, AMBER_DIM);

        sprite.setTextColor(AMBER_DIM, BG_COLOR);
        sprite.setTextSize(1);
        const char* sub1 = "Verbindet automatisch";
        tw = sprite.textWidth(sub1);
        sprite.setCursor((SCREEN_W - tw) / 2, 68);
        sprite.print(sub1);
        const char* sub2 = "wenn WLAN wieder verfuegbar ist.";
        tw = sprite.textWidth(sub2);
        sprite.setCursor((SCREEN_W - tw) / 2, 84);
        sprite.print(sub2);

        sprite.drawFastHLine(40, 110, 240, AMBER_DIM);

        sprite.setTextColor(AMBER_DIM, BG_COLOR);
        const char* h1 = "Mit \"LineTracker\" verbinden,";
        tw = sprite.textWidth(h1);
        sprite.setCursor((SCREEN_W - tw) / 2, 118);
        sprite.print(h1);
        const char* h2 = "dann linetracker.local oeffnen.";
        tw = sprite.textWidth(h2);
        sprite.setCursor((SCREEN_W - tw) / 2, 134);
        sprite.print(h2);

        sprite.pushSprite(0, 0);
        sprite.deleteSprite();
        return;
    }

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    int totalSlots = displaySlots.size();

    if (cfgLines.empty() && cfgOebb.empty()) {
        // ── Setup screen (320×170) ──
        // LineTracker branding
        sprite.setTextColor(AMBER, BG_COLOR);
        sprite.setTextSize(4);
        const char* brand = "LineTracker";
        int tw = sprite.textWidth(brand);
        sprite.setCursor((SCREEN_W - tw) / 2, 6);
        sprite.print(brand);

        sprite.drawFastHLine(40, 42, 240, AMBER_DIM);

        // URL prominent
        sprite.setTextColor(AMBER, BG_COLOR);
        sprite.setTextSize(2);
        String hostnameStr = cfgHostname + ".local";
        tw = sprite.textWidth(hostnameStr);
        sprite.setCursor((SCREEN_W - tw) / 2, 54);
        sprite.print(hostnameStr);

        // IP fallback
        sprite.setTextColor(AMBER_DIM, BG_COLOR);
        sprite.setTextSize(1);
        String ip = WiFi.localIP().toString();
        tw = sprite.textWidth(ip);
        sprite.setCursor((SCREEN_W - tw) / 2, 80);
        sprite.print(ip);

        // Instruction
        sprite.drawFastHLine(80, 98, 160, AMBER_DIM);
        sprite.setTextColor(AMBER, BG_COLOR);
        sprite.setTextSize(2);
        const char* s3 = "Linien auswaehlen";
        tw = sprite.textWidth(s3);
        sprite.setCursor((SCREEN_W - tw) / 2, 108);
        sprite.print(s3);

        // "by Leo Blum" subtle
        sprite.setTextColor(tft.color565(30, 24, 5), BG_COLOR);
        sprite.setTextSize(1);
        const char* attr = "by Leo Blum";
        tw = sprite.textWidth(attr);
        sprite.setCursor((SCREEN_W - tw) / 2, 158);
        sprite.print(attr);
    } else if (totalSlots == 0) {
        // ── Loading screen ──
        sprite.setTextColor(AMBER, BG_COLOR);
        sprite.setTextSize(2);
        const char* msg = fetchError ? "Daten werden geladen..." : "Lade Abfahrten...";
        int tw = sprite.textWidth(msg);
        sprite.setCursor((SCREEN_W - tw) / 2, (SCREEN_H - 14) / 2);
        sprite.print(msg);
        // Show IP as fallback for web access
        String ip = WiFi.localIP().toString();
        if (ip != "0.0.0.0") {
            sprite.setTextColor(AMBER_DIM, BG_COLOR);
            sprite.setTextSize(1);
            int iw = sprite.textWidth(ip);
            sprite.setCursor((SCREEN_W - iw) / 2, (SCREEN_H - 14) / 2 + 22);
            sprite.print(ip);
        }
    } else {
        // ── Smart display with page rotation (departure pages + optional clock page) ──
        int numDepPages = (totalSlots + MAX_ROWS - 1) / MAX_ROWS;
        if (numDepPages < 1) numDepPages = 1;

        struct tm clockTi;
        bool clockReady   = cfgShowClock && getLocalTime(&clockTi, 0);
        bool weatherReady = cfgShowWeather && weatherValid;
        int clockPage   = numDepPages;                     // index of clock page (if any)
        int weatherPage = numDepPages + (clockReady ? 1 : 0); // index of weather page (if any)
        int totalPages  = numDepPages + (clockReady ? 1 : 0) + (weatherReady ? 1 : 0);

        if (totalPages > 1) {
            if (millis() - lastRotateMs > (unsigned long)cfgRotateSec * 1000) {
                dispPage++;
                if (dispPage >= totalPages) dispPage = 0;
                lastRotateMs = millis();
                // Reset scroll for new page
                for (int j = 0; j < MAX_ROWS; j++) {
                    scrollOffset[j] = 0;
                    lastScrollText[j] = "";
                }
            }
        } else {
            dispPage = 0;
        }
        if (dispPage >= totalPages) dispPage = 0;  // config may have changed since last frame

        // Clock page: draw and bail (no departure slots involved)
        if (clockReady && dispPage == clockPage) {
            drawClockScreen(clockTi);
            xSemaphoreGive(dataMutex);
            sprite.pushSprite(0, 0);
            sprite.deleteSprite();
            return;
        }
        // Weather page: draw and bail
        if (weatherReady && dispPage == weatherPage) {
            drawWeatherScreen();
            xSemaphoreGive(dataMutex);
            sprite.pushSprite(0, 0);
            sprite.deleteSprite();
            return;
        }

        pageOffset = dispPage * MAX_ROWS;
        if (pageOffset >= totalSlots) pageOffset = 0;

        // Reserve bottom strip for disruption ticker if active
        bool showTicker = cfgShowDisruptions && !disruptions.empty();
        int tickerH = showTicker ? 13 : 0;
        int usableH = SCREEN_H - tickerH;

        int rows = min(totalSlots - pageOffset, MAX_ROWS);
        int totalSep = (rows - 1) * SEP_H;
        int rowH = (usableH - totalSep) / rows;

        // Visible height for centering (Font 1 char = 7px, not 8px)
        int nameH = 7 * NAME_SZ;
        int cdH   = 7 * CD_SZ;
        int dirLineH = 7 * DIR_SZ;
        int dirCharW = 6 * DIR_SZ;

        // ── Dynamic column widths ──
        sprite.setTextSize(NAME_SZ);
        int maxLeftW = 0;
        for (int i = 0; i < rows; i++) {
            int lw = sprite.textWidth(displaySlots[pageOffset + i].lineName);
            if (lw > maxLeftW) maxLeftW = lw;
        }

        // Time-to-leave: a slot is in "leave now" mode when walkMin > 0 and the
        // remaining countdown has fallen to/below the walk time. It uses the same
        // blinking-squares animation as an arriving (countdown==0) slot, so it
        // needs no extra width beyond the arriving-square floor below.
        sprite.setTextSize(CD_SZ);
        int maxRightW = 0;
        for (int i = 0; i < rows; i++) {
            const Departure& sd = displaySlots[pageOffset + i];
            bool jetzt = sd.walkMin > 0 && sd.countdown > 0 && sd.countdown <= sd.walkMin;
            if (sd.countdown > 0 && !jetzt) {
                int rw = sprite.textWidth(String(sd.countdown));
                if (rw > maxRightW) maxRightW = rw;
            }
        }
        int arrSz = 10;
        if (maxRightW < arrSz * 2 + 2) maxRightW = arrSz * 2 + 2;

        int centerX = PX_MARGIN + maxLeftW + PX_MARGIN * 2;
        int centerW = SCREEN_W - centerX - maxRightW - PX_MARGIN * 3;
        if (centerW < 20) centerW = 20;

        for (int i = 0; i < rows; i++) {
            const Departure& d = displaySlots[pageOffset + i];
            int y = i * (rowH + SEP_H);
            int centerY = y + rowH / 2;

            // Separator
            if (i > 0) {
                sprite.fillRect(0, y - SEP_H, SCREEN_W, SEP_H, TFT_BLACK);
            }

            // ── Line name (huge, left, with glow) ──
            sprite.setTextSize(NAME_SZ);
            {
                uint16_t nameGlow;
                uint16_t nameCore = lineColor565(d.lineName, nameGlow);
                drawGlowText(sprite, PX_MARGIN, centerY - nameH / 2, d.lineName, nameCore, nameGlow);
            }

            // ── Direction (always clipped to center column) ──
            sprite.setTextSize(DIR_SZ);
            String dir = sanitize(d.towards);

            if (lastScrollText[i] != dir) {
                scrollOffset[i] = 0;
                lastScrollText[i] = dir;
            }

            {
                // Render direction in a sub-sprite that clips to centerW
                TFT_eSprite clip = TFT_eSprite(&tft);
                clip.createSprite(centerW, rowH);
                clip.fillSprite(BG_COLOR);
                clip.setTextFont(1);
                clip.setTextSize(DIR_SZ);
                clip.setTextColor(AMBER, BG_COLOR);

                int dirW = clip.textWidth(dir);
                int maxCharsPerLine = centerW / max(dirCharW, 1);

                if ((int)dir.length() <= maxCharsPerLine) {
                    // Single line — vertically centered in clip
                    clip.setCursor(0, rowH / 2 - dirLineH / 2);
                    clip.print(dir);
                } else {
                    // Try two-line split at space near middle
                    int splitIdx = -1;
                    int mid = dir.length() / 2;
                    for (int off = 0; off <= mid; off++) {
                        if (mid + off < (int)dir.length() && dir.charAt(mid + off) == ' ') {
                            splitIdx = mid + off; break;
                        }
                        if (mid - off >= 0 && dir.charAt(mid - off) == ' ') {
                            splitIdx = mid - off; break;
                        }
                    }

                    if (splitIdx > 0) {
                        String line1 = dir.substring(0, splitIdx);
                        String line2 = dir.substring(splitIdx + 1);
                        int l1W = clip.textWidth(line1);
                        int l2W = clip.textWidth(line2);
                        bool needsScroll = (l1W > centerW || l2W > centerW);

                        if (!needsScroll) {
                            // Two lines fit — render stacked
                            int twoLineH = dirLineH * 2 + 2;
                            int startY = rowH / 2 - twoLineH / 2;
                            clip.setCursor(0, startY);
                            clip.print(line1);
                            clip.setCursor(0, startY + dirLineH + 2);
                            clip.print(line2);
                        } else {
                            // Too wide even split — scroll single line
                            int sy = rowH / 2 - dirLineH / 2;
                            int sx = -scrollOffset[i];
                            clip.setCursor(sx, sy);
                            clip.print(dir);
                            clip.setCursor(sx + dirW + SCROLL_GAP, sy);
                            clip.print(dir);
                            scrollOffset[i] += SCROLL_PX;
                            if (scrollOffset[i] >= dirW + SCROLL_GAP)
                                scrollOffset[i] = 0;
                        }
                    } else {
                        // No space to split — scroll
                        int sy = rowH / 2 - dirLineH / 2;
                        int sx = -scrollOffset[i];
                        clip.setCursor(sx, sy);
                        clip.print(dir);
                        clip.setCursor(sx + dirW + SCROLL_GAP, sy);
                        clip.print(dir);
                        scrollOffset[i] += SCROLL_PX;
                        if (scrollOffset[i] >= dirW + SCROLL_GAP)
                            scrollOffset[i] = 0;
                    }
                }

                clip.pushToSprite(&sprite, centerX, y);
                clip.deleteSprite();
            }

            // ── Countdown / arriving-or-leave-now blink (right) ──
            // jetztMode (walkMin reached) shows the same blinking squares as an
            // arriving slot — "leave now" and "arriving" share one indicator.
            bool jetztMode = d.walkMin > 0 && d.countdown > 0 && d.countdown <= d.walkMin;
            sprite.setTextSize(CD_SZ);
            if (d.countdown == 0 || jetztMode) {
                bool phase = (millis() / 1000) % 2 == 0;
                int sz  = arrSz;
                int gap = 2;
                int tw  = sz * 2 + gap;
                int th  = sz * 2 + gap;
                int ax  = SCREEN_W - PX_MARGIN - tw;
                int ay  = centerY - th / 2;
                if (phase) {
                    sprite.fillRect(ax + sz + gap, ay,            sz, sz, AMBER);
                    sprite.fillRect(ax,            ay + sz + gap, sz, sz, AMBER);
                } else {
                    sprite.fillRect(ax,            ay,            sz, sz, AMBER);
                    sprite.fillRect(ax + sz + gap, ay + sz + gap, sz, sz, AMBER);
                }
            } else {
                // Main countdown
                String cdStr = String(d.countdown);
                int cdW = sprite.textWidth(cdStr);

                if (cfgShowNext) {
                    // Find next departure for same line+direction
                    int nextCd = -1;
                    bool found1 = false;
                    for (auto& dep : departures) {
                        if (dep.lineName == d.lineName && dep.towards == d.towards) {
                            if (!found1) { found1 = true; continue; }  // skip first (= current)
                            nextCd = dep.countdown; break;
                        }
                    }

                    if (nextCd >= 0) {
                        // Stack: main countdown top, "→ Xmin" below
                        int nextSz = 2;
                        int nextH  = 7 * nextSz;
                        int totalCdH = cdH + 3 + nextH;
                        int topY = centerY - totalCdH / 2;
                        drawGlowText(sprite, SCREEN_W - PX_MARGIN - cdW, topY, cdStr);
                        sprite.setTextSize(nextSz);
                        sprite.setTextColor(AMBER_DIM, BG_COLOR);
                        String nextStr = ">" + String(nextCd);
                        int nextW = sprite.textWidth(nextStr);
                        sprite.setCursor(SCREEN_W - PX_MARGIN - nextW, topY + cdH + 3);
                        sprite.print(nextStr);
                        sprite.setTextSize(CD_SZ);
                    } else {
                        drawGlowText(sprite, SCREEN_W - PX_MARGIN - cdW, centerY - cdH / 2, cdStr);
                    }
                } else {
                    drawGlowText(sprite, SCREEN_W - PX_MARGIN - cdW, centerY - cdH / 2, cdStr);
                }
            }
        }

        // ── Page indicator dots (if multiple pages, no ticker) ──
        if (totalSlots > MAX_ROWS && !showTicker) {
            int pages = (totalSlots + MAX_ROWS - 1) / MAX_ROWS;
            int currentPage = pageOffset / MAX_ROWS;
            int dotR = 2;
            int dotGap = 8;
            int dotsW = pages * (dotR * 2) + (pages - 1) * dotGap;
            int dotX = (SCREEN_W - dotsW) / 2;
            int dotY = usableH - 5;
            for (int p = 0; p < pages; p++) {
                uint16_t col = (p == currentPage) ? AMBER : AMBER_DIM;
                sprite.fillCircle(dotX + p * (dotR * 2 + dotGap) + dotR, dotY, dotR, col);
            }
        }

        // ── Disruption ticker ──
        if (showTicker) {
            int ty = usableH;
            sprite.fillRect(0, ty, SCREEN_W, tickerH, TFT_BLACK);
            sprite.drawFastHLine(0, ty, SCREEN_W, AMBER_DIM);

            // Build full ticker string from all disruptions
            String tickerText = "";
            for (size_t ti2 = 0; ti2 < disruptions.size(); ti2++) {
                if (ti2 > 0) tickerText += "  |  ";
                tickerText += disruptions[ti2];
            }
            tickerText += "     ";

            TFT_eSprite tickerClip = TFT_eSprite(&tft);
            tickerClip.createSprite(SCREEN_W, tickerH - 1);
            tickerClip.fillSprite(TFT_BLACK);
            tickerClip.setTextFont(1);
            tickerClip.setTextSize(1);
            tickerClip.setTextColor(AMBER_DIM, TFT_BLACK);
            int textW = tickerClip.textWidth(tickerText);
            tickerClip.setCursor(-tickerOffset, (tickerH - 1 - 7) / 2);
            tickerClip.print(tickerText);
            // Wrap-around copy
            tickerClip.setCursor(-tickerOffset + textW, (tickerH - 1 - 7) / 2);
            tickerClip.print(tickerText);
            tickerClip.pushToSprite(&sprite, 0, ty + 1);
            tickerClip.deleteSprite();

            tickerOffset += 1;
            if (tickerOffset >= textW) tickerOffset = 0;
        }

        // ── IP address — drawn last so it's always on top ──
        {
            String ip = WiFi.localIP().toString();
            sprite.setTextFont(1);
            sprite.setTextSize(1);
            sprite.setTextColor(tft.color565(25, 20, 5), BG_COLOR);
            int iw = sprite.textWidth(ip);
            sprite.setCursor(SCREEN_W - iw - 2, 1);
            sprite.print(ip);
        }
    }
    xSemaphoreGive(dataMutex);

    sprite.pushSprite(0, 0);
    sprite.deleteSprite();
}

// ── Power schedule: night dim + standby (panel off) + deep sleep ─────
bool lastNightState = false;
volatile bool standbyActive = false;
volatile bool powerDirty = false;

// ST7789 command opcodes
static const uint8_t ST_SLPIN  = 0x10;
static const uint8_t ST_SLPOUT = 0x11;
static const uint8_t ST_DISPOFF = 0x28;
static const uint8_t ST_DISPON  = 0x29;

// Deep sleep safety bounds
static const unsigned long MIN_AWAKE_MS = 90000;     // stay awake ≥90s after boot before deep sleep
static const long          MIN_SLEEP_S  = 120;       // don't deep-sleep for < 2 min (anti-thrash)
static const long          MAX_SLEEP_S  = 14L * 3600;// cap at 14h (sanity bound)

// True if hour h falls inside [from, to). from==to or any negative → empty/disabled.
static bool inWindow(int h, int from, int to) {
    if (from < 0 || to < 0 || from == to) return false;
    if (from < to) return (h >= from && h < to);
    return (h >= from || h < to);  // wraps midnight (e.g. 22–7)
}

void panelSleep() {
    ledcWrite(0, 0);
    tft.writecommand(ST_DISPOFF);
    tft.writecommand(ST_SLPIN);
}

void panelWake() {
    tft.writecommand(ST_SLPOUT);
    delay(120);
    tft.writecommand(ST_DISPON);
}

// Runs in displayTask only (panel SPI commands must not run from another task).
void applyPowerSchedule() {
    struct tm ti;
    if (!getLocalTime(&ti, 0)) {
        // No valid time yet — make sure the display is on at day brightness.
        if (standbyActive) { panelWake(); standbyActive = false; }
        if (lastNightState) { ledcWrite(0, cfgBrightness); lastNightState = false; }
        return;
    }
    bool we = cfgWeekendSchedule && (ti.tm_wday == 0 || ti.tm_wday == 6);
    int sFrom = we ? cfgStandbyFromWe : cfgStandbyFrom;
    int sTo   = we ? cfgStandbyToWe   : cfgStandbyTo;
    int nFrom = we ? cfgNightFromWe   : cfgNightFrom;
    int nTo   = we ? cfgNightToWe     : cfgNightTo;
    int h = ti.tm_hour;

    // Standby only applies to the departures view; games keep the screen on.
    bool standby = (appMode == MODE_DEPARTURES) && inWindow(h, sFrom, sTo);

    if (standby) {
        if (!standbyActive) { panelSleep(); standbyActive = true; }
        return;  // deep sleep (if enabled) handled by maybeEnterDeepSleep()
    }

    // Not in standby → ensure panel awake, then apply night/day brightness.
    bool justWoke = false;
    if (standbyActive) { panelWake(); standbyActive = false; justWoke = true; }

    bool isNight = inWindow(h, nFrom, nTo);
    if (isNight != lastNightState || justWoke) {
        ledcWrite(0, isNight ? cfgNightBright : cfgBrightness);
        lastNightState = isNight;
    }
}

// EXPERIMENTAL deep sleep. Runs in displayTask. Multiple safety gates guard
// against boot-loops / making the device unreachable. Never returns if it sleeps.
void maybeEnterDeepSleep() {
    if (!cfgStandbyDeepSleep) return;
    if (appMode != MODE_DEPARTURES) return;
    if (otaInProgress) return;
    if (millis() < MIN_AWAKE_MS) return;     // give web server time to come up first
    struct tm ti;
    if (!getLocalTime(&ti, 0)) return;       // never deep-sleep without a valid clock

    bool we = cfgWeekendSchedule && (ti.tm_wday == 0 || ti.tm_wday == 6);
    int sFrom = we ? cfgStandbyFromWe : cfgStandbyFrom;
    int sTo   = we ? cfgStandbyToWe   : cfgStandbyTo;
    if (!inWindow(ti.tm_hour, sFrom, sTo)) return;

    // Seconds until the next occurrence of sTo:00:00, + small margin.
    long nowSec  = ti.tm_hour * 3600L + ti.tm_min * 60L + ti.tm_sec;
    long delta   = sTo * 3600L - nowSec;
    if (delta <= 0) delta += 24 * 3600;
    delta += 30;
    if (delta < MIN_SLEEP_S) return;         // too close to window end → skip
    if (delta > MAX_SLEEP_S) delta = MAX_SLEEP_S;

    logf("Deep sleep for %ld s (until %02d:00)\n", delta, sTo);

    // Brief on-device notice so it's clear what happened. Wake the panel first —
    // applyPowerSchedule() may have already put it to sleep on entering standby.
    panelWake();
    standbyActive = false;
    sprite.createSprite(SCREEN_W, SCREEN_H);
    sprite.fillSprite(BG_COLOR);
    sprite.setTextFont(1);
    sprite.setTextColor(AMBER, BG_COLOR);
    sprite.setTextSize(2);
    char msg[32];
    snprintf(msg, sizeof(msg), "Deep Sleep bis %02d:00", sTo);
    sprite.setCursor((SCREEN_W - sprite.textWidth(msg)) / 2, 70);
    sprite.print(msg);
    sprite.pushSprite(0, 0);
    sprite.deleteSprite();
    delay(1500);

    // Clean shutdown: panel sleep + backlight forced low so it stays dark.
    panelSleep();
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, LOW);

    esp_sleep_enable_timer_wakeup((uint64_t)delta * 1000000ULL);
    esp_deep_sleep_start();  // never returns; wakes via full reboot
}

// ── FreeRTOS tasks ───────────────────────────────────────────────────
void dataTask(void* param) {
    // Wait for NTP sync (non-blocking, max 4s)
    {
        struct tm ti;
        int ntpWaits = 0;
        while (!getLocalTime(&ti, 1000) && ntpWaits < 4) ntpWaits++;
        logf("%s\n", getLocalTime(&ti, 0) ? "NTP synced" : "NTP timeout, will retry");
    }

    unsigned long lastOtaCheck = millis();
    for (;;) {
        setCrumb("dataTask:idle");
        // WiFi state tracking — ESP32 auto-reconnects, we just manage the portal
        if (WiFi.status() != WL_CONNECTED) {
            if (wifiDownSince == 0) {
                wifiDownSince = millis();
                logf("WiFi lost, requesting portal\n");
                portalShouldOpen = true;
            }
        } else if (wifiDownSince != 0) {
            logf("WiFi reconnected: %s\n", WiFi.localIP().toString().c_str());
            wifiDownSince = 0;
            if (portalOpen) {
                portalOpen = false;
                portalShouldOpen = false;
                logf("Closing AP after reconnect\n");
            }
            MDNS.begin(cfgHostname.c_str());
            MDNS.addService("http", "tcp", 80);
        }
        fetchDepartures();
        fetchOebbDepartures();
        // Weather: only when the page is enabled, throttled (changes slowly)
        if (cfgShowWeather && (lastWeatherFetch == 0 ||
                               millis() - lastWeatherFetch > WEATHER_INTERVAL_MS)) {
            fetchWeather();
            lastWeatherFetch = millis();
        }
        // Refresh CSV cache if stale, then rebuild line directions
        if (!isCacheValid()) {
            if (refreshCsvCache(true)) buildLineDirections();
        } else if ((lineDirMap.empty() || haltRecordCount == 0) && SPIFFS.exists(CACHE_STEIGE_PATH)) {
            // Cache valid but line directions or search indexes not built yet
            buildLineDirections();
        }
        // Check for OTA updates every 6h
        if (millis() - lastOtaCheck > OTA_CHECK_INTERVAL_MS) {
            checkOtaUpdate();
            lastOtaCheck = millis();
        }
        // Wait 20s, but wake early if config changed or WiFi drops
        for (int t = 0; t < 40; t++) {  // 40 × 500ms = 20s
            if (configChanged) {
                configChanged = false;
                break;
            }
            if (WiFi.status() != WL_CONNECTED && wifiDownSince == 0) {
                wifiDownSince = millis();
                logf("WiFi lost (sleep), requesting portal\n");
                portalShouldOpen = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

void displayTask(void* param) {
    unsigned long lastPowerCheck = 0;
    AppMode lastMode = appMode;
    for (;;) {
        if (appMode == MODE_PONG)  pongTick();
        if (appMode == MODE_SNAKE) snakeTick();
        drawDisplay();
        // Re-evaluate power state on a timer, when config changed, or when the
        // app mode switched (e.g. a game starts during standby → wake the panel).
        if (appMode != lastMode) { lastMode = appMode; powerDirty = true; }
        if (powerDirty || millis() - lastPowerCheck > 10000) {
            powerDirty = false;
            applyPowerSchedule();
            maybeEnterDeepSleep();  // may not return
            lastPowerCheck = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // ~20fps for smooth scrolling
    }
}


// ── Setup & Loop ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    logf("LineTracker v%s starting...\n", FW_VERSION);

    tft.init();
    delay(150);  // give ST7789 time to wake up before first draw
    tft.setRotation(3);

    // Fahrgastinformationssystem colors
    BG_COLOR  = tft.color565(10, 8, 5);
    AMBER     = tft.color565(255, 191, 0);
    AMBER_DIM = tft.color565(50, 38, 0);

    tft.fillScreen(BG_COLOR);
    tft.setTextColor(AMBER);

    // Backlight via LEDC PWM — brightness set after loadConfig()
    ledcSetup(0, 2000, 8);
    ledcAttachPin(TFT_BL, 0);
    ledcWrite(0, 255);  // start at full, updated after config loads

    // ── Splash screen ───────────────────────────────────────────────
    tft.setTextFont(1);
    tft.setTextColor(AMBER, BG_COLOR);

    // "LineTracker" centered, large
    tft.setTextSize(4);
    const char* brand = "LineTracker";
    int bw = tft.textWidth(brand);
    tft.setCursor((320 - bw) / 2, 30);
    tft.print(brand);

    // Decorative line
    int lineY = 75;
    tft.drawFastHLine(40, lineY, 240, AMBER_DIM);

    // "by Leo Blum"
    tft.setTextSize(2);
    tft.setTextColor(AMBER_DIM, BG_COLOR);
    const char* author = "by Leo Blum";
    int aw = tft.textWidth(author);
    tft.setCursor((320 - aw) / 2, 85);
    tft.print(author);

    // Version
    tft.setTextSize(1);
    String verStr = "v" + String(FW_VERSION);
    int vw = tft.textWidth(verStr);
    tft.setCursor((320 - vw) / 2, 112);
    tft.print(verStr);

    // Data attribution
    tft.setTextColor(tft.color565(30, 24, 5), BG_COLOR);
    const char* attr = "Data: Stadt Wien (CC BY 4.0) / OeBB";
    int atw = tft.textWidth(attr);
    tft.setCursor((320 - atw) / 2, 155);
    tft.print(attr);

    delay(2000);  // show splash for 2 seconds
    // ── End splash ──────────────────────────────────────────────────

    if (!SPIFFS.begin(true)) logf("SPIFFS mount failed\n");

    saveCrashInfo();
    loadConfig();
    if (cfgHostname.length() == 0) {
        cfgHostname = "linetracker";
        saveConfig();
    }
    loadDirCache();
    loadLineDirections();
    ledcWrite(0, cfgBrightness);

    // Show loading status below splash (skip if not yet configured — goes straight to setup screen)
    if (SPIFFS.exists(WIFI_CONFIGURED_FLAG)) {
        tft.setTextColor(AMBER_DIM, BG_COLOR);
        tft.setTextSize(1);
        tft.setCursor((320 - tft.textWidth("Verbinde WiFi...")) / 2, 140);
        tft.print("Verbinde WiFi...");
    }

    setupWiFi();

    // Give the WiFi stack a moment to fully stabilize before mDNS check
    delay(500);

    // Detect hostname conflict: check if linetracker.local is already in use by
    // another device. If so, assign a unique MAC-based name. If no conflict and
    // we previously had a MAC-based name, revert to "linetracker".
    {
        IPAddress resolved;
        bool conflict = WiFi.hostByName("linetracker.local", resolved)
                        && resolved != WiFi.localIP()
                        && resolved != IPAddress(0, 0, 0, 0);
        if (conflict) {
            uint8_t mac[6];
            WiFi.macAddress(mac);
            char suffix[5];
            snprintf(suffix, sizeof(suffix), "%02x%02x", mac[4], mac[5]);
            String uniqueName = "linetracker-" + String(suffix);
            if (cfgHostname != uniqueName) {
                cfgHostname = uniqueName;
                saveConfig();
                logf("Hostname conflict — using: %s\n", cfgHostname.c_str());
            }
        } else if (cfgHostname != "linetracker") {
            // No conflict — revert to default if we had a MAC-based name
            cfgHostname = "linetracker";
            saveConfig();
        }
    }

    // Start mDNS + web server immediately so user can access UI
    if (MDNS.begin(cfgHostname.c_str())) {
        MDNS.addService("http", "tcp", 80);
        logf("mDNS: %s.local\n", cfgHostname.c_str());
    }
    logf("Connected! IP: %s heap=%u psram=%u\n",
         WiFi.localIP().toString().c_str(), ESP.getFreeHeap(), ESP.getFreePsram());
    startConfigServer();

    // Briefly show assigned hostname on splash so user always knows the URL
    {
        String urlStr = "http://" + cfgHostname + ".local";
        tft.fillRect(0, 130, 320, 40, BG_COLOR);
        tft.setTextFont(1);
        tft.setTextColor(AMBER, BG_COLOR);
        tft.setTextSize(2);
        int uw = tft.textWidth(urlStr);
        tft.setCursor((320 - uw) / 2, 138);
        tft.print(urlStr);
        delay(3000);
    }

    // Start display task immediately so setup screen with IP shows
    dataMutex  = xSemaphoreCreateMutex();
    pongMutex  = xSemaphoreCreateMutex();
    snakeMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(displayTask, "display", 8192,  NULL, 1, NULL, 1);

    // NTP config (sync happens in background, no blocking wait)
    configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");

    logf("WL Lines: %d\n", cfgLines.size());
    logf("OeBB Stations: %d\n", cfgOebb.size());

    // Start data fetch task immediately (handles NTP wait, CSV cache, line directions)
    xTaskCreatePinnedToCore(dataTask,    "data",    16384, NULL, 1, NULL, 0);
}

void loop() {
    server.handleClient();
    if (portalShouldOpen && !portalOpen) {
        portalShouldOpen = false;
        // Open raw soft AP without touching STA reconnect.
        // DNS server redirects all domains → 192.168.4.1 so phones auto-open captive portal.
        // WiFiManager is NOT involved here — it would call WiFi.disconnect() and break reconnect.
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP("LineTracker");
        WiFi.begin(); // restart STA with saved NVS credentials
        apDns.start(53, "*", WiFi.softAPIP());
        portalOpen = true;
        logf("AP opened at %s, STA reconnecting\n", WiFi.softAPIP().toString().c_str());
    }
    if (portalOpen) {
        apDns.processNextRequest();
        if (WiFi.status() == WL_CONNECTED) {
            apDns.stop();
            WiFi.softAPdisconnect(true);
            portalOpen = false;
            logf("AP closed after reconnect\n");
        }
    }
    delay(5);
}
