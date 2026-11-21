/*
 * bridge_sd_log.cpp — SD logging for the RCX1 NTRIP base station
 * =============================================================
 * See bridge_sd_log.h for the file layout and the non-blocking architecture.
 *
 * PROTECTIVE NOTES (read before editing):
 *  - NEVER write to the card from loop() or the RTCM parser. Producers only
 *    enqueue; the Core-0 task does all I/O. Breaking this re-introduces the
 *    GNSS-UART starvation we worked hard to eliminate.
 *  - SD_MMC shares no pins with the TFT. Display is on SPI 40/41/42/45/46/39;
 *    SD_MMC (1-bit) is on CLK=14, CMD=15, D0=16, with D3/CS=21. VERIFIED against
 *    the board's hardware map and the working rover wiring — keep it that way.
 *  - DO NOT "fix" these to 18/19/20: GPIO19/20 are the ESP32-S3's native USB D−/D+.
 *    Putting SD there bricks USB flashing and forces BOOT-button/UART recovery.
 *    A static_assert below hard-blocks 19/20/26–37 so this cannot ship again.
 *  - D3/CS (GPIO21) is NOT driven by firmware: the board's 10K pull-up holds it
 *    HIGH, which keeps the card in SD-bus (not SPI) mode. 1-bit mode uses only
 *    CLK/CMD/D0, so setPins() is the 3-arg form and D3 is never touched.
 */

#include "bridge_sd_log.h"

// ── Log file names (shared with the sketch's download and delete handlers) ───
// EXACT NAMES ONLY (keep). The session files are exactly the names openTagFile() and
// openRawFile() create, and nothing else is treated as one. A prefix test is not enough:
// a FAT 8.3 short alias such as "event_~1.csv" begins with "event_", and FatFs resolves
// the alias to the same file as its long name, so a delete matched by prefix would unlink
// the current session's open file while the SD task writes it (cross-linked clusters),
// with the session number read as 0.
bool bridge_sdlog_is_log_name(const char* nm, long* sessionOut) {
    if (!nm) return false;
    static const char* const kStems[] = { "gps", "sat", "status", "rtcm", "event" };
    const char* p = nullptr;
    bool rtcm = false;
    for (const char* st : kStems) {
        const size_t n = strlen(st);
        if (strncmp(nm, st, n) == 0 && nm[n] == '_') { p = nm + n + 1; rtcm = (st == kStems[3]); break; }
    }
    if (!p) return false;
    long sn = 0;
    int digits = 0;
    for (; *p >= '0' && *p <= '9'; ++p) {
        if (++digits > 5) return false;
        sn = sn * 10 + (*p - '0');
    }
    if (digits == 0 || sn > 65535) return false;
    if (p[0] == '_') {                                   // rotation suffix: one letter a-z
        if (p[1] < 'a' || p[1] > 'z') return false;
        p += 2;
    }
    if (strcmp(p, ".csv") != 0 && !(rtcm && strcmp(p, ".bin") == 0)) return false;
    if (sessionOut) *sessionOut = sn;
    return true;
}

#if BRIDGE_LOG_SD_ENABLE

// ── HARD GUARD: never let an SD pin land on USB / flash / PSRAM GPIOs ─────────
// GPIO19/20 = native USB D−/D+ (assigning them breaks USB flashing); 26–32 =
// SPI flash; 33–37 = octal PSRAM on the -S3R8. This caught a real bug once;
// keep it. If this fires, the offending pin is in bridge_sd_log.h.
#define BRIDGE_PIN_FORBIDDEN(p) ((p) == 19 || (p) == 20 || \
                                 ((p) >= 26 && (p) <= 37))
static_assert(!BRIDGE_PIN_FORBIDDEN(BRIDGE_SD_CLK_PIN), "SD CLK pin hits USB/flash/PSRAM GPIO");
static_assert(!BRIDGE_PIN_FORBIDDEN(BRIDGE_SD_CMD_PIN), "SD CMD pin hits USB/flash/PSRAM GPIO");
static_assert(!BRIDGE_PIN_FORBIDDEN(BRIDGE_SD_D0_PIN),  "SD D0 pin hits USB/flash/PSRAM GPIO");
static_assert(!BRIDGE_PIN_FORBIDDEN(BRIDGE_SD_D3_PIN),  "SD D3 pin hits USB/flash/PSRAM GPIO");

#include "FS.h"
#include "SD_MMC.h"
#include "esp_heap_caps.h"   // heap_caps_* for the pre-mount internal-RAM diagnostic
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"

// ── Record + tags ────────────────────────────────────────────────────────────
// TAG_EVENT carries the diagnostics that previously existed only on the serial port.
// The base is normally run where nobody can attach a laptop — on a roof, in a paddock,
// under open sky — and every hard-won answer in this project has come from a log file
// rather than from someone watching a console. A finding that only prints to serial is a
// finding that is unavailable exactly when the base is doing its real job.
enum LogTag : uint8_t { TAG_GPS = 0, TAG_SAT = 1, TAG_STATUS = 2, TAG_RTCM = 3,
                        TAG_EVENT = 4, TAG_COUNT = 5 };

struct LogLine {
    uint8_t tag;
    // MUST exceed the longest row any channel can produce, or enqueueText() silently
    // truncates it and the trailing columns vanish from the file with nothing to show it
    // happened. The status row is the widest and has grown repeatedly — caster mount and
    // error strings, then the heap columns — and at 352 it had ALREADY overrun: rows in a
    // captured session measured 353 characters, so the position-check columns at the end
    // were being cut off. The lint's csv check compares header columns against format
    // specifiers; it cannot see this, which is why the number is stated here with the
    // measurement behind it. Current worst case is ~385 characters.
    char    text[560];   // >= the widest builder; enforced by the lint's rowfit check
};

static QueueHandle_t     g_queue     = nullptr;
static StreamBufferHandle_t g_rawStream = nullptr;   // raw RTCM bytes
static TaskHandle_t g_sdTaskHandle = nullptr;         // for bridge_sdlog_stack_free(); cleared if the task ends
static volatile bool     g_sdReady   = false;
// The card could not be mounted and the SD task has ended. Read by the boot screen and
// the dashboard, which show the card as FAILED instead of waiting for it.
static volatile bool     g_sdMountFailed = false;
static volatile uint32_t g_dropped   = 0;
static uint32_t          g_statusTrunc = 0;   // status rows rebuilt to fit (see ROW FIT)
// Rows or raw chunks that could not be written to the card: a file that would not open, or
// a write that returned fewer bytes than were handed to it. Distinct from g_dropped, which
// counts rows lost to a full queue. Read by the sketch to report a full or failing card.
static volatile uint32_t g_writeFail = 0;
static uint16_t          g_session   = 0;
// Set by the SD task when the session collision guard moved to another number; loop()
// persists it (bridge_sdlog_service()). The task never writes NVS itself: a flash commit
// disables the instruction cache on both cores.
static volatile bool     g_sessionPersistPending = false;

// ── Runtime channel enables (default to the compile-time flags; toggled live
// from the web UI and persisted in NVS). volatile: written from the web/loop
// context, read by producers and the SD task. A torn read is benign (worst case
// one row logged or skipped) so no lock is needed.
static volatile bool g_enGps    = BRIDGE_LOG_GPS_ENABLE;
static volatile bool g_enSat    = BRIDGE_LOG_SAT_ENABLE;
static volatile bool g_enStatus = BRIDGE_LOG_STATUS_ENABLE;
static volatile bool g_enRtcm   = BRIDGE_LOG_RTCM_DETAIL_ENABLE;
static volatile bool g_enRaw    = BRIDGE_LOG_RTCM_RAW_ENABLE;
// Events default ON and should stay on: the channel is a handful of lines per session,
// and it is the only record of the things that used to be visible solely on serial.
static volatile bool g_enEvent  = true;

// DOWNLOAD COORDINATION: while a web download is streaming from the card, the
// task stops writing so there's exactly one reader and no head-thrash/contention.
// The queue keeps accepting (and dropping when full) — logging is best-effort, so
// losing a few rows during a download is fine and keeps producers non-blocking.
static volatile bool g_downloadActive = false;

// Producer-side cadence timers (run in loop context)
static uint32_t g_lastGpsMs    = 0;
static uint32_t g_lastStatusMs = 0;

// GSV capture window state (producer/NMEA context)
static bool     g_satCapturing   = false;
static uint32_t g_lastSatMs      = 0;
static uint32_t g_satWindowStart = 0;
// One snapshot = the GSV sentences of one burst, which arrive within ~60 ms at 1 Hz.
#ifndef BRIDGE_LOG_SAT_WINDOW_MS
#define BRIDGE_LOG_SAT_WINDOW_MS     300
#endif

// Latest GGA time-of-day, stashed by tick() so sat rows (which run in NMEA
// context without the snapshot) can reference wall-clock time.
static char     g_lastUtc[16]    = "";
// RMC calendar date "ddmmyy", paired with g_lastUtc to form a full datetime.
static char     g_lastDate[12]   = "";

// Format a full ISO-8601 UTC timestamp from raw NMEA fields:
//   date "ddmmyy"  +  utc "hhmmss.sss"  →  "2026-06-28T15:43:58.000"
// Returns false (and leaves dst[0]=='\0') if either field is missing or malformed.
// This is the ONLY source of the utc_time column — never approximated or inferred.
static bool fmtDatetime(char* dst, size_t dstSize,
                        const char* date6, const char* utc8) {
    dst[0] = '\0';
    if (!date6 || strlen(date6) < 6) return false;
    if (!utc8  || strlen(utc8)  < 6) return false;
    // ddmmyy
    const int dd = (date6[0]-'0')*10 + (date6[1]-'0');
    const int mm = (date6[2]-'0')*10 + (date6[3]-'0');
    const int yy = (date6[4]-'0')*10 + (date6[5]-'0');
    // Sanity: reject obvious bad values (month 0 or >12, day 0 or >31)
    if (mm < 1 || mm > 12 || dd < 1 || dd > 31) return false;
    const int yyyy = 2000 + yy;
    // hhmmss — always 6 chars minimum
    const int hh = (utc8[0]-'0')*10 + (utc8[1]-'0');
    const int mi = (utc8[2]-'0')*10 + (utc8[3]-'0');
    const int ss = (utc8[4]-'0')*10 + (utc8[5]-'0');
    // Sub-second: keep up to 3 digits (PQTMPVT gives .sss; GGA gives .ss → pad to .ss0)
    const char* dot = strchr(utc8, '.');
    char sub[8] = ".000";
    if (dot) {
        sub[0] = '.';
        sub[1] = (dot[1] ? dot[1] : '0');
        sub[2] = (dot[2] ? dot[2] : '0');
        sub[3] = (dot[3] ? dot[3] : '0');
        sub[4] = '\0';
    }
    // ISO 8601 with T separator and Z suffix (all timestamps are UTC)
    snprintf(dst, dstSize, "%04d-%02d-%02dT%02d:%02d:%02d%sZ",
             yyyy, mm, dd, hh, mi, ss, sub);
    return true;
}

// ── Helpers ──────────────────────────────────────────────────────────────────
// Rows refused because the queue was nearly full and the row was low priority (sat and
// RTCM-detail rows). Counted apart from g_dropped, which is rows LOST from a full queue.
static volatile uint32_t g_shed = 0;
// LOW-PRIORITY SHEDDING (keep). Satellite rows arrive in bursts (a factory 10 Hz receiver
// produced ~240 rows/s before PQTMCFGFIXRATE took effect; a download pauses the task
// while they keep coming) and filled the shared queue, so event, gps, status and logbook
// rows were dropped behind them. Sat and RTCM-detail rows now stop being queued once fewer
// than BRIDGE_LOG_LOWPRIO_RESERVE slots remain, keeping that headroom for the rows that
// carry the diagnosis.
#ifndef BRIDGE_LOG_LOWPRIO_RESERVE
#define BRIDGE_LOG_LOWPRIO_RESERVE  24
#endif
// EVENT RESERVE (keep). The last BRIDGE_LOG_EVENT_RESERVE slots take event rows only.
// While a download pauses the SD task every other row keeps arriving, and the event that
// records the download's end (bytes, duration, ok or aborted) is the row that explains
// the gap in all the others; it must find a slot. A non-event row refused here is counted
// as dropped, exactly as if the queue had been full.
#ifndef BRIDGE_LOG_EVENT_RESERVE
#define BRIDGE_LOG_EVENT_RESERVE    4
#endif
static void enqueue(const LogLine& ln) {
    // No card (mount failed, task ended): nothing will ever drain the queue, so rows are not
    // queued at all. Counting them as dropped would report a logging fault that is really
    // the card failure the dashboard already shows (log.sdfail).
    if (!g_queue || g_sdMountFailed) return;
    const UBaseType_t space = uxQueueSpacesAvailable(g_queue);
    if ((ln.tag == TAG_SAT || ln.tag == TAG_RTCM) && space < BRIDGE_LOG_LOWPRIO_RESERVE) {
        g_shed = g_shed + 1;
        // The first shed of the session is recorded as an event (the running count is the
        // status log's log_shed column), so a hole in sat_/rtcm_ rows is explained on the
        // card itself. Event rows are not low priority, so this cannot recurse.
        if (g_shed == 1) {
            Serial.println("⚠️  SD: queue nearly full - sat/RTCM-detail rows are being skipped");
            bridge_sdlog_event("warn", "SD log: queue nearly full - sat and RTCM-detail rows are being "
                                       "skipped (first this session; running count in status log_shed)");
        }
        return;
    }
    if (ln.tag != TAG_EVENT && space <= BRIDGE_LOG_EVENT_RESERVE) { g_dropped = g_dropped + 1; return; }
    if (xQueueSend(g_queue, &ln, 0) != pdTRUE) g_dropped = g_dropped + 1;   // non-blocking; drop if full (g_dropped is volatile — avoid deprecated volatile ++)
}

static void enqueueText(uint8_t tag, const char* text) {
    LogLine ln;
    ln.tag = tag;
    strncpy(ln.text, text, sizeof(ln.text) - 1);
    ln.text[sizeof(ln.text) - 1] = '\0';
    enqueue(ln);
}

// ── CSV COLUMN METADATA (units documented here; headers carry them inline) ───
// Unit convention: a trailing _<unit> suffix on a column name gives its unit.
//   _deg = degrees      _m = metres         _s = seconds        _f = °F
//   _dbhz = dB-Hz       _fps = frames/s     _bps = bytes/s
// Columns with no suffix are unitless identifiers, flags (0/1), counts, or strings:
//   ms          = milliseconds since boot (monotonic anchor, always present)
//   utc_time    = ISO 8601 UTC "YYYY-MM-DDThh:mm:ss.sssZ" from PQTMPVT date+time.
//                 Available from first satellite epoch, before GGA fix quality > 0.
//                 Blank rows = module had not yet received satellite time.
//   fix         = GGA fix-quality code (0/1/2/4/5/7); sats = satellite count
//   svin_valid  = PQTMSVINSTATUS valid (0/1/2); base_ready/wifi/crc_ok = 0/1 flags
//   svin_obs_s  = survey elapsed seconds; svin_target_s = survey target seconds
//   svin_mode   = probed receiver mode: 0=unconfigured, 1=survey-in, 2=fixed-base
//   hot_start   = 1 if firmware skipped reconfigure (warm/battery state preserved)
//   epe_3d_m / epe_2d_m = position error estimate, 3D and horizontal 2D
//   lat_wgs84_deg / lon_wgs84_deg = WGS84 latitude / longitude
//   alt_msl_m   = height above mean sea level as the receiver reports it (GGA)
//   ellh_wgs84_m = height above the WGS84 ellipsoid (MSL + geoid separation); blank until
//                 the receiver has reported a separation
//   ecef_wgs84_x_m / _y_m / _z_m = WGS84 ECEF position components (metres), from the
//                 ellipsoidal height; blank on the same condition as ellh_wgs84_m
//   cN_*        = caster N fields; type/len = RTCM message type / byte length
//   esp_temp_f  = ESP32-S3 die temperature (°F)
//   imu_temp_f  = QMI8658 temperature (°F) — reads heat-soak in trapped pocket
//                 under SD card, NOT case ambient. Blank when IMU absent or read fails.
// English name for the GGA <Quality> field (see LG290P protocol spec). Note 7 =
// "Manual input mode or Survey-in" → the base is reporting a FIXED/entered position,
// not navigating. All names are comma-free so they're safe as a bare CSV field.
static const char* fixQualityName(int q) {
    switch (q) {
        case 0:  return "Invalid";
        case 1:  return "GPS SPS";
        case 2:  return "DGPS/SBAS";
        case 3:  return "GPS PPS";
        case 4:  return "RTK Fixed";
        case 5:  return "RTK Float";
        case 6:  return "Dead Reckoning";
        case 7:  return "Fixed/Survey-in";
        case 8:  return "Simulation";
        default: return "Unknown";
    }
}

static const char* headerForTag(uint8_t tag) {
    switch (tag) {
        // Coordinate columns name their reference: see COORDINATE REFERENCE in the header.
        case TAG_GPS:    return "ms,utc_time,fix,fix_name,sats,hdop,lat_wgs84_deg,lon_wgs84_deg,alt_msl_m,"
                                "ellh_wgs84_m,epe_3d_m,epe_2d_m,"
                                "ecef_wgs84_x_m,ecef_wgs84_y_m,ecef_wgs84_z_m,svin_valid,svin_obs_s,"
                                "svin_target_s,svin_meanacc_m,base_ready";
        // signal_id: NMEA 4.11 GSV signal ID (the field after the last satellite group), which
        // tells the bands of one PRN apart (GPS 1 = L1 C/A, 8 = L5Q; Galileo 7 = E1, 5 = E6 ...;
        // see the receiver's protocol spec). Blank on a sentence without one.
        case TAG_SAT:    return "ms,utc_time,constellation,prn,elev_deg,azim_deg,snr_dbhz,signal_id";
        case TAG_STATUS: return "ms,utc_time,uptime_s,wifi,ssid,ip,fix,sats,base_ready,"
                                "svin_valid,svin_mode,svin_obs_s,svin_meanacc_m,hot_start,"
                                "rtcm_fps,rtcm_bps,valid_frames,"
                                "c0_host,c0_mount,c0_en,c0_state,c0_hand,c0_frames,c0_bytes,c0_drop,c0_age_s,c0_err,"
                                "c1_host,c1_mount,c1_en,c1_state,c1_hand,c1_frames,c1_bytes,c1_drop,c1_age_s,c1_err,"
                                "lat_wgs84_deg,lon_wgs84_deg,acc_m,pos_src,pos_acc_m,"
                                "loop_max_ms,uart_hw,rtcm_crc_fail,rtcm_frame_fail,nmea_cs_fail,nmea_desync,"
                                "esp_temp_f,imu_temp_f,heap_int,heap_largest,heap_min,lc_clients,lc_served,log_drop,log_shed,"
                                "pc_state,pc_fixes,pc_lat_wgs84_deg,pc_lon_wgs84_deg,pc_epe_m,pc_dist_m,pc_thresh_m";
        case TAG_RTCM:   return "ms,utc_time,type,len,crc_ok,caster0,caster1,detail";
        // level is one of ok / warn / fail / info — enough to filter a long session down
        // to the moments that mattered without reading every line.
        case TAG_EVENT:  return "ms,utc_time,level,event";
        default:         return "";
    }
}

static const char* baseNameForTag(uint8_t tag) {
    switch (tag) {
        case TAG_GPS:    return "gps";
        case TAG_SAT:    return "sat";
        case TAG_STATUS: return "status";
        case TAG_RTCM:   return "rtcm";
        case TAG_EVENT:  return "event";
        default:         return "log";
    }
}

// Constellation name from the GSV talker id (first two chars after '$').
static const char* constellationName(char a, char b) {
    if (a == 'G' && b == 'P') return "GPS";
    if (a == 'G' && b == 'L') return "GLONASS";
    if (a == 'G' && b == 'A') return "Galileo";
    if (a == 'G' && b == 'B') return "BeiDou";
    if (a == 'B' && b == 'D') return "BeiDou";
    if (a == 'G' && b == 'Q') return "QZSS";
    if (a == 'Q' && b == 'Z') return "QZSS";
    if (a == 'G' && b == 'N') return "GNSS";
    return "OTHER";
}

// ── SD task (Core 0) ─────────────────────────────────────────────────────────
static File     g_files[TAG_COUNT];
static bool     g_headerWritten[TAG_COUNT] = {false, false, false, false, false};
static uint32_t g_writeCount[TAG_COUNT]    = {0, 0, 0, 0, 0};
static uint32_t g_fileBytes[TAG_COUNT]     = {0, 0, 0, 0, 0};
static uint8_t  g_rotateSeq[TAG_COUNT]     = {0, 0, 0, 0, 0};
static uint32_t g_lastFlushMs[TAG_COUNT]   = {0, 0, 0, 0, 0};
// COMMIT POLICY (keep). FAT only records a file's new size when it is synced or closed,
// and this base is switched off by cutting power, so rows written but not yet synced are
// lost. A 50-row bound alone would leave a typical session's event file (fewer than 50
// rows) as a bare header on the card, and minutes of status and gps rows uncommitted. So
// event rows are committed at once (they are few and are what a post-mortem needs),
// every other tag at least every BRIDGE_LOG_FLUSH_MS while it is being written, plus the
// 50-row bound. About one sync a
// second in total, not the per-pass syncing the raw-stream note below warns against.
// The time bound is also checked on every pass of the task (commitIdleTags()), so the
// last rows before a tag stops producing (a receiver that goes silent stops gps and sat
// rows) are committed within BRIDGE_LOG_FLUSH_MS instead of waiting for a next row that
// may never come.
#ifndef BRIDGE_LOG_FLUSH_MS
#define BRIDGE_LOG_FLUSH_MS  5000
#endif
// Set by the task once it has flushed everything and is idle for a download; see
// bridge_sdlog_pause_acked().
static volatile bool g_pauseAck = false;
static File     g_rawFile;
static uint32_t g_rawBytes = 0;
static uint8_t  g_rawSeq   = 0;

// NEVER TRUNCATE (keep). FILE_WRITE truncates a file that already exists, and logs are
// never overwritten or deleted automatically. A name already on the card (a reused session
// number's rotated file, or a file another unit left) is skipped: the rotation letter
// (seq) advances to the next free name. Past 'z' the existing file is appended to instead.
// appended = true when the returned file already existed (it has its header).
static File openNewLogFile(const char* stem, const char* ext, uint8_t& seq, bool& appended,
                           char* name, size_t nameSize) {
    for (;;) {
        if (seq == 0) {
            snprintf(name, nameSize, "/%s_%04u.%s", stem, g_session, ext);
        } else {
            // Letter suffix (_a, _b, ...), not a number — see BRIDGE_LOG_ROTATE_MAX_SEQ.
            const char suffix = 'a' + ((seq - 1) % BRIDGE_LOG_ROTATE_MAX_SEQ);
            snprintf(name, nameSize, "/%s_%04u_%c.%s", stem, g_session, suffix, ext);
        }
        if (!SD_MMC.exists(name)) { appended = false; return SD_MMC.open(name, FILE_WRITE); }
        if (seq >= BRIDGE_LOG_ROTATE_MAX_SEQ) { appended = true; return SD_MMC.open(name, FILE_APPEND); }
        Serial.printf("⚠️  SD: %s already exists - not overwritten, using the next name\n", name);
        ++seq;
    }
}

static void openTagFile(uint8_t tag) {
    char name[40];
    bool appended = false;
    g_files[tag] = openNewLogFile(baseNameForTag(tag), "csv", g_rotateSeq[tag], appended, name, sizeof(name));
    if (g_files[tag]) {
        if (!appended) g_files[tag].println(headerForTag(tag));
        g_files[tag].flush();
        g_headerWritten[tag] = true;
        g_fileBytes[tag] = 0;
        Serial.printf("📝 SD: logging → %s\n", name);
    } else {
        Serial.printf("❌ SD: cannot open %s\n", name);
    }
}

static void eventRow(const char* level, const char* msg, bool withUtc);   // event row; see its definition

// ROTATION ROW (keep). One event row per rotation: which file filled, at what size, and the
// rotation letter of the file that continues it (openNewLogFile() may have skipped names
// already on the card), so a reader can follow the session across files. Queued from this
// task onto its own queue like any producer; never written directly.
static void logRotation(const char* stem, const char* ext, uint32_t closedBytes, uint8_t seq, bool opened) {
    char msg[160];
    snprintf(msg, sizeof msg, "SD log rotation: %s_%04u %s file closed at %lu B (limit %lu B); continued in %s_%04u_%c.%s%s",
             stem, (unsigned)g_session, ext, (unsigned long)closedBytes, (unsigned long)BRIDGE_LOG_ROTATE_BYTES,
             stem, (unsigned)g_session,
             (char)('a' + ((seq - 1) % BRIDGE_LOG_ROTATE_MAX_SEQ)), ext, opened ? "" : " (could not be opened)");
    eventRow(opened ? "info" : "fail", msg, false);   // Core 0: no utc_time (see eventRow())
}

static void writeLine(const LogLine& ln) {
    const uint8_t tag = ln.tag;
    if (tag >= TAG_COUNT) return;
    if (!g_files[tag] || !g_headerWritten[tag]) openTagFile(tag);
    if (!g_files[tag]) { g_writeFail = g_writeFail + 1; return; }

    const size_t n = g_files[tag].println(ln.text);
    if (n < strlen(ln.text)) g_writeFail = g_writeFail + 1;
    g_fileBytes[tag] += n;

    const uint32_t nowMs = millis();
    if (tag == TAG_EVENT || ++g_writeCount[tag] >= 50 ||       // see COMMIT POLICY
        (uint32_t)(nowMs - g_lastFlushMs[tag]) >= BRIDGE_LOG_FLUSH_MS) {
        g_files[tag].flush();
        g_writeCount[tag] = 0;
        g_lastFlushMs[tag] = nowMs;
    }
    // Rotate every tag by size so a long session stays manageable and gps/sat/status
    // never approach FAT32's single-file cap on a multi-week deployment. Clamped at the
    // 'z' suffix: past BRIDGE_LOG_ROTATE_MAX_SEQ rotations we stop advancing the letter
    // and just keep appending to the 'z' file, so an extreme session never wraps into a
    // colliding filename or silently overwrites an earlier rotation.
    if (BRIDGE_LOG_ROTATE_BYTES > 0 && g_fileBytes[tag] >= BRIDGE_LOG_ROTATE_BYTES &&
        g_rotateSeq[tag] < BRIDGE_LOG_ROTATE_MAX_SEQ) {
        const uint32_t closedBytes = g_fileBytes[tag];
        g_files[tag].flush();
        g_files[tag].close();
        g_rotateSeq[tag]++;
        g_headerWritten[tag] = false;
        openTagFile(tag);
        logRotation(baseNameForTag(tag), "csv", closedBytes, g_rotateSeq[tag], (bool)g_files[tag]);
    }
}

// Time bound of the COMMIT POLICY for tags that have stopped producing rows. A tag with
// uncommitted rows (g_writeCount > 0) is synced once BRIDGE_LOG_FLUSH_MS has passed since
// its last commit; a tag with nothing pending costs one comparison.
static void commitIdleTags() {
    const uint32_t nowMs = millis();
    for (uint8_t t = 0; t < TAG_COUNT; ++t) {
        if (g_files[t] && g_writeCount[t] &&
            (uint32_t)(nowMs - g_lastFlushMs[t]) >= BRIDGE_LOG_FLUSH_MS) {
            g_files[t].flush();
            g_writeCount[t]  = 0;
            g_lastFlushMs[t] = nowMs;
        }
    }
}

// ── Logbook ──────────────────────────────────────────────────────────────────
// One running file for the whole card, never rotated and never part of a session's
// files: a line when a session's UTC time and approximate position are first known, and
// a line for every saved base position, each tagged with the session's file number. It is
// opened, appended and closed for each line, so nothing is held open or resident between the
// few lines a session writes.
// Rows reach it through the same queue as every other row, so this task stays the only
// SD writer. The header is written when the file is empty (first use of a card).
//
// HEADER VARIANTS (keep). The header is written only when the file is created, and an
// existing logbook is never rewritten or deleted (old logs are never altered), so a card
// whose logbook was started under the lat_deg/lon_deg/ellh_m header keeps that header:
//     utc_time,file_no,uptime_ms,kind,src,lat_deg,lon_deg,ellh_m,hacc_m,survey_s,fix,note
// Both headers have the same twelve columns in the same order, and the values are the same
// in both: WGS84 latitude and longitude in degrees, height above the WGS84 ellipsoid. Only
// the names differ, so the datum and height type are also stated in the rows themselves:
// every session's boot row says so in its note (serviceLogbook()), and every session's
// event file carries the "Coordinate reference" row (setup()), which names both height
// column spellings. The page's logbook reader (parseLogbook() in bridge_web_page.h) skips
// any line starting "utc_time" and reads the columns by position, so it reads either file.
static constexpr uint8_t TAG_LOGBOOK = 0xB0;   // outside TAG_COUNT: not a session file
static const char* const LOGBOOK_PATH   = "/" BRIDGE_LOGBOOK_FILE;
static const char* const LOGBOOK_HEADER =
    "utc_time,file_no,uptime_ms,kind,src,lat_wgs84_deg,lon_wgs84_deg,ellh_wgs84_m,hacc_m,survey_s,fix,note";

static void appendLogbook(const LogLine& ln) {
    // Existence is checked before the open: on a file that open() has just created,
    // File::size() can return an unset value in released arduino-esp32 3.x cores, which
    // would skip the header.
    const bool fresh = !SD_MMC.exists(LOGBOOK_PATH);
    File f = SD_MMC.open(LOGBOOK_PATH, FILE_APPEND);
    if (!f) { g_writeFail = g_writeFail + 1; return; }
    if (fresh) f.println(LOGBOOK_HEADER);
    const size_t n = f.println(ln.text);
    if (n < strlen(ln.text)) g_writeFail = g_writeFail + 1;
    f.close();   // commits the data and the directory entry; no handle is kept
}

static void openRawFile() {
    char name[40];
    bool appended = false;   // a raw capture has no header either way
    g_rawFile = openNewLogFile("rtcm", "bin", g_rawSeq, appended, name, sizeof(name));
    if (g_rawFile) { g_rawBytes = 0; Serial.printf("📝 SD: raw RTCM → %s\n", name); }
    else           Serial.printf("❌ SD: cannot open %s\n", name);
}

static uint32_t g_rawWritesSinceFlush = 0;
static uint32_t g_rawLastFlushMs      = 0;
// Time-bound the raw .bin flush. See drainRawStream() for why per-pass is wrong.
#ifndef BRIDGE_LOG_RAW_FLUSH_MS
#define BRIDGE_LOG_RAW_FLUSH_MS  1000
#endif

static void drainRawStream() {
    if (!g_rawStream) return;
    uint8_t buf[512];
    size_t got;
    while ((got = xStreamBufferReceive(g_rawStream, buf, sizeof(buf), 0)) > 0) {
        if (!g_rawFile) openRawFile();
        if (!g_rawFile) { g_writeFail = g_writeFail + 1; return; }
        if (g_rawFile.write(buf, got) != got) g_writeFail = g_writeFail + 1;
        g_rawBytes += got;
        // PERIODIC FLUSH (keep): commits the .bin's data and directory entry every
        // ~16 chunks (~8 KB). Without it the size on the card is only committed at the
        // 64 MB rotation, so a card pulled mid-session, or simply inspected, reads 0
        // bytes despite data being buffered. Bounds the loss on a power cut, mirroring
        // the CSV tags' 50-write flush cadence.
        if (++g_rawWritesSinceFlush >= 16) {
            g_rawFile.flush();
            g_rawWritesSinceFlush = 0;
            g_rawLastFlushMs = millis();
        }
        if (BRIDGE_LOG_ROTATE_BYTES > 0 && g_rawBytes >= BRIDGE_LOG_ROTATE_BYTES &&
            g_rawSeq < BRIDGE_LOG_ROTATE_MAX_SEQ) {
            const uint32_t closedBytes = g_rawBytes;
            g_rawFile.flush();
            g_rawFile.close();
            g_rawSeq++;
            openRawFile();
            logRotation("rtcm", "bin", closedBytes, g_rawSeq, (bool)g_rawFile);
        }
    }
    // Commit a low-rate stream (the normal ~400 B/s base) without waiting for 16
    // chunks — but on a TIMER, not on every pass.
    // DO NOT flush unconditionally here. This task drains every ~20 ms, so a per-pass
    // f_sync issues up to 50 FAT + directory writes PER SECOND. That keeps the SDMMC
    // bus and its Core-0 DMA busy essentially continuously (Core 0 is also where the
    // WiFi driver and lwIP tasks live), and it throttles this task's own queue drain
    // badly enough that status rows get dropped on a full queue. A 1 s bound gives the
    // same worst-case data loss at ~1/50th the write amplification. Checked on every pass,
    // not only after a write, so the last chunk before the stream stops is committed too;
    // g_rawWritesSinceFlush > 0 keeps an idle, already-committed file from being synced.
    const uint32_t nowMs = millis();
    if (g_rawFile && g_rawWritesSinceFlush > 0 &&
        (uint32_t)(nowMs - g_rawLastFlushMs) >= BRIDGE_LOG_RAW_FLUSH_MS) {
        g_rawFile.flush();
        g_rawWritesSinceFlush = 0;
        g_rawLastFlushMs = nowMs;
    }
}

static void sdTask(void*) {
    delay(800);   // let the SD power rail settle after boot

    // D3/CS is held HIGH by the board's 10K pull-up to keep the card in SD-bus
    // (not SPI) mode — this matches the working rover wiring, so firmware does
    // NOT drive it. 1-bit mode uses CLK/CMD/D0 only.
    SD_MMC.setPins(BRIDGE_SD_CLK_PIN, BRIDGE_SD_CMD_PIN, BRIDGE_SD_D0_PIN);

    // DIAGNOSTIC (do not remove): print DMA-capable internal heap right before mount.
    // esp_vfs_fat_sdmmc_mount() allocates its FATFS work area + `maxOpenFiles` file
    // objects from INTERNAL RAM (not PSRAM). On this board the webserver/WiFi already
    // pressure internal DRAM, and a low figure here turning into a mount failure is the
    // signature of RAM exhaustion vs. a card/bus problem. If this number is healthy
    // (tens of KB) and mount still fails, the cause is the card/FAT or the bus, not RAM.
    Serial.printf("ℹ️  SD: pre-mount internal heap=%u B (largest block=%u B)\n",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    // mountpoint, mode1bit=true, format_if_mount_failed, freq, maxOpenFiles.
    // maxOpenFiles=12: with every channel on, the task holds up to 5 files open
    // (4 CSV + 1 .bin); a concurrent web download/listing needs more handles, and the
    // default of 5 would make downloads fail exactly when all logging is enabled.
    //
    // ROBUST MOUNT (replaces the old single-shot begin at 20 MHz that just died silently):
    // a card/bus that worked before but now fails to mount is most often (a) a marginal
    // bus that no longer clocks clean at SDMMC_FREQ_DEFAULT (20 MHz) — it will usually
    // still mount at the 400 kHz probing clock — or (b) a FAT corrupted by an unclean
    // power-off mid-write (this device power-cycles with no flush guarantee). We try the
    // normal clock first, then fall back to the slow probing clock, and ONLY if both fail
    // do we consider a one-shot reformat — guarded behind BRIDGE_SD_FORMAT_ON_FAIL so we
    // never silently wipe a card that merely has flaky contacts. Default keeps the card
    // read-only-safe (no reformat) so data is recoverable on a PC.
    const struct { int freqKhz; const char* label; } kAttempts[] = {
        { SDMMC_FREQ_DEFAULT, "20MHz" },
        { SDMMC_FREQ_PROBING, "400kHz" },   // slow, signal-robust; mounts marginal buses
    };
    bool mounted = false;
    const char* mountedAt = "";   // the clock that mounted it, for the MOUNT ROW
    for (uint8_t i = 0; i < (sizeof(kAttempts)/sizeof(kAttempts[0])) && !mounted; ++i) {
        if (SD_MMC.begin("/sdcard", true, false, kAttempts[i].freqKhz, 12)) {
            mounted = true;
            mountedAt = kAttempts[i].label;
            Serial.printf("✅ SD_MMC: mounted at %s (1-bit)\n", kAttempts[i].label);
            break;
        }
        Serial.printf("⚠️  SD_MMC: mount failed at %s — retrying slower\n", kAttempts[i].label);
        SD_MMC.end();
        delay(150);
    }

#if BRIDGE_SD_FORMAT_ON_FAIL
    // OPT-IN LAST RESORT: both clocks failed. If the card is present but its FAT is
    // corrupt, this recovers it by reformatting (DESTROYS existing logs). Only compiled
    // in when you deliberately enable the flag, so it can't surprise-wipe a good card.
    if (!mounted) {
        Serial.println("🛠️  SD_MMC: both clocks failed — attempting one-shot reformat (FORMAT_ON_FAIL=1)");
        if (SD_MMC.begin("/sdcard", true, true /*format_if_mount_failed*/, SDMMC_FREQ_PROBING, 12)) {
            mounted = true;
            mountedAt = "400kHz after a reformat";
            Serial.println("✅ SD_MMC: reformatted and mounted — prior logs were lost");
        }
    }
#endif

    if (!mounted) {
        // Clear, actionable differential instead of the old generic "check pins" line.
        // Pins are compile-time-verified by the static_assert above, so do NOT chase pins.
        Serial.println("❌ SD_MMC: mount failed at both 20MHz and 400kHz.");
        Serial.println("   → If the pre-mount heap above is low (<~20KB largest block): internal-RAM exhaustion.");
        Serial.println("   → Else most likely a corrupted FAT (unclean power-off) or a card not seated/dead.");
        Serial.println("   → Try: read the card on a PC; reformat FAT32; or build with BRIDGE_SD_FORMAT_ON_FAIL=1 once.");
        // ENDING THE TASK (keep this order). The failure flag is raised and the handle
        // cleared BEFORE vTaskDelete(): loop() reads the handle once a second for the stack
        // headroom figure, and a handle left pointing at a deleted task's freed control
        // block is a use-after-free (a bogus reading, or a LoadProhibited reboot loop on a
        // dead card). Nothing else holds the handle.
        g_sdMountFailed = true;
        g_sdTaskHandle  = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    // SESSION COLLISION GUARD (keep). The session number lives in NVS, but the files live
    // on the card. Erased NVS (an "erase all flash" upload, the core's NVS reformat) or a
    // card moved between units reuses a number, and the earlier session's logs would be
    // mixed with or overwritten by this one's. The newest session always carries the highest
    // number on the card, which the listing and the "recent" set rely on: if any session file
    // carries this number or a higher one (rotated files such as gps_NNNN_a.csv included),
    // the session moves past the highest number on the card. "Higher" covers a card moved
    // from another base with higher numbers, and erased NVS on a card whose low-numbered
    // sessions were deleted, where the requested number is free but not the newest. One
    // pass over the root directory by name only (getNextFileName() reads entries without
    // opening or stat-ing each one, so the pass stays cheap on a card with hundreds of
    // files). Done before any row is written and
    // before g_sdReady releases the logbook. The new number is persisted by loop()
    // (bridge_sdlog_service()), never from this Core-0 task. openNewLogFile() is the
    // second line of defence: it never opens an existing file for writing.
    bool sessionMoved = false;   // for the MOUNT ROW: decided here, not read back from the shared flag
    {
        const uint16_t requested = g_session;
        bool used = false;
        long highest = -1;
        // Which of the numbers just above the requested one, and of 0001-0256, the card
        // already uses (bit i = requested + 1 + i, and 1 + i): enough to choose a free number
        // when the card's highest is 65535 (see below) without a second pass.
        static constexpr uint32_t kFreeScan = 256;
        uint8_t usedAbove[kFreeScan / 8] = {0}, usedLow[kFreeScan / 8] = {0};
        File root = SD_MMC.open("/");
        if (root) {
            bool isDir = false;
            for (String path = root.getNextFileName(&isDir); path.length() > 0;
                 path = root.getNextFileName(&isDir)) {
                if (isDir) continue;
                const char* nm = strrchr(path.c_str(), '/');
                nm = nm ? nm + 1 : path.c_str();
                long sn = -1;
                if (!bridge_sdlog_is_log_name(nm, &sn)) continue;
                if (sn == (long)requested) used = true;
                if (sn > highest) highest = sn;
                const long a = sn - (long)requested - 1, l = sn - 1;
                if (a >= 0 && a < (long)kFreeScan) usedAbove[a / 8] |= (uint8_t)(1u << (a % 8));
                if (l >= 0 && l < (long)kFreeScan) usedLow[l / 8]   |= (uint8_t)(1u << (l % 8));
            }
            root.close();
        }
        // A card holding 65535, the last number there is, cannot be moved past: its
        // highest + 1 would wrap to 0001, which the card may already hold, and every later
        // boot would then land on it. So on such a card a requested number that is free keeps
        // itself, and a requested number that collides moves to the first free number above
        // it (0001-0256 if none of the next 256 is free). With all of those taken it stays,
        // and openNewLogFile() still never writes into an existing file.
        if (highest < 65535) {
            if (used || highest >= (long)requested) g_session = (uint16_t)(highest + 1);
        } else if (used) {
            bool found = false;
            for (uint32_t i = 0; i < kFreeScan && !found; ++i) {
                const uint32_t n = (uint32_t)requested + 1u + i;
                if (n > 65535u) break;
                if (!(usedAbove[i / 8] & (1u << (i % 8)))) { g_session = (uint16_t)n; found = true; }
            }
            for (uint32_t i = 0; i < kFreeScan && !found; ++i) {
                if (!(usedLow[i / 8] & (1u << (i % 8)))) { g_session = (uint16_t)(1u + i); found = true; }
            }
        }
        sessionMoved = (g_session != requested);
        if (sessionMoved) {
            g_sessionPersistPending = true;
            Serial.printf("⚠️  SD: session %04u %s on this card (highest %04ld) - using %04u\n",
                          (unsigned)requested, used ? "already has files" : "is below the newest session",
                          highest, (unsigned)g_session);
        }
    }
    const uint64_t freeMB = (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / 1048576ULL;
    // Cast explicitly rather than relying on uint64_t's underlying type. It is
    // "unsigned long long" on the ESP32 and "unsigned long" on a 64-bit host, so an
    // uncast %llu is correct in one place and a mismatch in the other — which is enough
    // to make the off-target check either miss real format bugs or reject this one.
    Serial.printf("✅ SD_MMC: ready, %llu MB free (session %04u)\n",
                  (unsigned long long)freeMB, g_session);
    // MOUNT ROW (keep): the card's free space and the session number this boot writes under,
    // and whether the collision guard above moved it, as the first row from this task.
    {
        char msg[160];
        snprintf(msg, sizeof msg, "SD card mounted at %s: %llu MB free; session %04u%s",
                 mountedAt, (unsigned long long)freeMB, (unsigned)g_session,
                 sessionMoved ? " (moved past the numbers already on this card)" : "");
        eventRow("info", msg, false);   // Core 0: no utc_time (see eventRow())
    }
    g_sdReady = true;
    // Pre-expire the status timer so the first status row is written immediately
    // when the SD task starts draining the queue — not after a 5 s wait. This
    // captures the boot state (wifi, svin_mode, hot_start, temps) in the very
    // first rows of every session.
    g_lastStatusMs = 0;

    LogLine ln;
    for (;;) {
        // DOWNLOAD COORDINATION: hold off all card I/O while the web server streams
        // a file, so there's a single reader and no contention. Queue/stream keep
        // accepting and drop on overflow — bounded, best-effort.
        // On entering the pause, commit every open file first, so a download of the
        // current session's files reads everything written so far, then acknowledge.
        if (g_downloadActive) {
            if (!g_pauseAck) {
                for (uint8_t t = 0; t < TAG_COUNT; ++t) {
                    if (g_files[t]) { g_files[t].flush(); g_writeCount[t] = 0; g_lastFlushMs[t] = millis(); }
                }
                if (g_rawFile) { g_rawFile.flush(); g_rawWritesSinceFlush = 0; g_rawLastFlushMs = millis(); }
                g_pauseAck = true;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        g_pauseAck = false;

        if (xQueueReceive(g_queue, &ln, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (ln.tag == TAG_LOGBOOK) appendLogbook(ln);
            else                       writeLine(ln);
        }
        drainRawStream();
        commitIdleTags();
    }
}

// ── Public API ───────────────────────────────────────────────────────────────
void bridge_sdlog_init() {
    // Monotonic session id + persisted channel toggles in NVS.
    Preferences p;
    p.begin("bridgelog", false);
    g_session = p.getUShort("session", 0) + 1;
    // Session numbers run 1..65535; the increment past 65535 wraps to 0, which is not a
    // valid session number (the log-name grammar starts at 1), so it continues at 1.
    if (g_session == 0) g_session = 1;
    p.putUShort("session", g_session);
    g_enGps    = p.getBool("enGps",    BRIDGE_LOG_GPS_ENABLE);
    g_enSat    = p.getBool("enSat",    BRIDGE_LOG_SAT_ENABLE);
    // STATUS LOG: always enabled regardless of NVS. It is the primary diagnostic
    // channel and must start from SD-ready on every boot. Allowing it to be
    // persisted-off (from accidentally toggling it in the web UI) silently breaks
    // the only log that records survey-in and caster state during boot. It can still
    // be toggled off at runtime for the current session if needed; we just never
    // persist that off-state as the boot default.
    g_enStatus = true;
    g_enRtcm   = p.getBool("enRtcm",   BRIDGE_LOG_RTCM_DETAIL_ENABLE);
    g_enRaw    = p.getBool("enRaw",    BRIDGE_LOG_RTCM_RAW_ENABLE);
    g_enEvent  = p.getBool("enEvent",  true);
    p.end();

    // INTERNAL RAM, NOT PSRAM (keep). The queue and the raw stream carry every log row and
    // raw RTCM byte from the GNSS path, and their control blocks hold the spinlock and wait
    // lists both cores take on every send and receive. On the ESP32-S3 the atomic
    // compare-and-set a spinlock depends on cannot run on PSRAM: ESP-IDF emulates it under
    // one global lock with interrupts masked (esp_cpu_compare_and_set), and the
    // xQueueCreateWithCaps / xStreamBufferCreateWithCaps family would put the control
    // blocks in PSRAM along with the storage. Relocation was evaluated and declined; the
    // internal heap these use is watched instead (status-row heap columns, low-heap warning).
    g_queue = xQueueCreate(BRIDGE_LOG_QUEUE_DEPTH, sizeof(LogLine));
    if (!g_queue) { Serial.println("❌ SD: log queue alloc failed"); return; }
    // Always allocate the raw stream so raw capture can be toggled on at runtime.
    g_rawStream = xStreamBufferCreate(16384, 1);
    if (!g_rawStream) Serial.println("⚠️ SD: raw stream alloc failed (raw RTCM unavailable)");

    // Pin to Core 0 so SD latency never lands on the Core-1 GNSS/caster loop.
    // Stack = 10240: SD_MMC.begin()/FATFS plus the 561-byte LogLine that lives on this
    // task's stack make 6144 marginal. 10240 is the value proven stable on the RCX
    // RaceCapture rover's equivalent SD task — match it rather than run close to the edge.
    xTaskCreatePinnedToCore(sdTask, "bridgeSdLog", 10240, nullptr, 1, &g_sdTaskHandle, 0);
}

void bridge_sdlog_tick(const BridgeLogSnapshot& s) {
    const uint32_t now = s.nowMs;
    // Stash the latest time-of-day and date so NMEA/RTCM context paths can use them.
    strncpy(g_lastUtc,  s.utc,  sizeof(g_lastUtc)  - 1); g_lastUtc[sizeof(g_lastUtc)-1]   = '\0';
    strncpy(g_lastDate, s.date, sizeof(g_lastDate)  - 1); g_lastDate[sizeof(g_lastDate)-1] = '\0';

    // Build the shared datetime string once per tick — all rows this tick get the same stamp.
    char dt[32]; fmtDatetime(dt, sizeof(dt), g_lastDate, g_lastUtc);

    // svin_meanacc_m is written BLANK when no accuracy figure exists yet — see
    // BridgeLogSnapshot::svinMeanAccKnown. A numeric 0.0000 in this column is a claim of
    // sub-millimetre accuracy, and it was being written on rows where nothing had been
    // measured, indistinguishable there from a genuine reading. Built once per tick and
    // shared by both rows that carry the column, so the two can never disagree about how
    // "unknown" is written.
    char accField[16];
    if (s.svinMeanAccKnown) snprintf(accField, sizeof(accField), "%.4f", s.svinMeanAccM);
    else                    accField[0] = '\0';

    if (g_enGps && s.haveFix && now - g_lastGpsMs >= BRIDGE_LOG_GPS_PERIOD_MS) {
        g_lastGpsMs = now;
        char line[256];
        // Ellipsoidal height and ECEF are blank until the receiver has reported a geoid
        // separation: without it the sketch's ECEF uses the MSL height, tens of metres off.
        char ellh[20], ex[20], ey[20], ez[20];
        if (s.ellHKnown) {
            snprintf(ellh, sizeof(ellh), "%.3f", s.ellHM);
            snprintf(ex, sizeof(ex), "%.3f", s.ecefX);
            snprintf(ey, sizeof(ey), "%.3f", s.ecefY);
            snprintf(ez, sizeof(ez), "%.3f", s.ecefZ);
        } else {
            ellh[0] = ex[0] = ey[0] = ez[0] = '\0';
        }
        // ms,utc_time,fix,fix_name,sats,hdop,lat_wgs84_deg,lon_wgs84_deg,alt_msl_m,ellh_wgs84_m,
        // epe_3d_m,epe_2d_m,ecef_wgs84_x_m,ecef_wgs84_y_m,ecef_wgs84_z_m,svin_valid,svin_obs_s,
        // svin_target_s,svin_meanacc_m,base_ready
        snprintf(line, sizeof(line),
                 "%lu,%s,%d,%s,%d,%.1f,%.9f,%.9f,%.3f,%s,%.3f,%.3f,%s,%s,%s,%u,%lu,%lu,%s,%d",
                 (unsigned long)now, dt, s.fixQuality, fixQualityName(s.fixQuality), s.satellites, s.hdop,
                 s.lat, s.lon, s.altM, ellh, s.epeM, s.epe2dM, ex, ey, ez,
                 (unsigned)s.svinValid, (unsigned long)s.svinObs, (unsigned long)s.svinTarget,
                 accField, s.baseReady ? 1 : 0);
        enqueueText(TAG_GPS, line);
    }

    // PEAK SINCE LAST ROW (keep). The sketch hands over loop_max_ms and uart_hw as peaks
    // over its ~200 ms snapshot interval; a status row is written every 5 s. Using the
    // snapshot's value as-is recorded only the last 200 ms of each row's interval, so a
    // multi-second stall or a ring overflow earlier in the 5 s was never seen. The peaks
    // are accumulated here across snapshots and reset when the row is written.
    static uint32_t peakLoopMs = 0, peakUartHw = 0;
    if (s.loopMaxMs > peakLoopMs)     peakLoopMs = s.loopMaxMs;
    if (s.uartHighWater > peakUartHw) peakUartHw = s.uartHighWater;
    if (!g_enStatus) { peakLoopMs = 0; peakUartHw = 0; }   // no row to report them in: start fresh when re-enabled
    if (g_enStatus && now - g_lastStatusMs >= BRIDGE_LOG_STATUS_PERIOD_MS) {
        g_lastStatusMs = now;
        // Sized to the measured worst case, not to a round number. A captured session's
        // status rows ran 353 characters with empty caster fields; the three heap columns
        // add ~30, and the two caster mount (23) and error (55) strings add ~148 more when
        // populated, giving ~535. The figure matters in both directions: too small
        // truncates the row, and too large forces LogLine.text wider, which multiplies by
        // the queue depth straight out of the internal RAM the SD driver needs.
        char line[560];
        // Temp fields: chip always present; board blank when the IMU sensor is
        // absent/unhealthy (matches the blank-until-valid convention of datetime).
        char tChip[12];  char tBoard[12];
        if (isnan(s.chipTempF)) tChip[0] = '\0';
        else snprintf(tChip, sizeof(tChip), "%.1f", s.chipTempF);
        // pos_acc_m: blank rather than a number when no saved position exists, so an
        // unsurveyed session can never be mistaken for a highly accurate one.
        char posAcc[16];
        // Position-check columns: blank rather than 0 when there is nothing measured, so
        // "no check ran" is distinguishable from "measured exactly zero".
        char pcLat[20], pcLon[20], pcEpe[16], pcDist[16], pcThresh[16];
        if (s.pcLat == 0.0 && s.pcLon == 0.0) { pcLat[0] = '\0'; pcLon[0] = '\0'; }
        else { snprintf(pcLat, sizeof(pcLat), "%.9f", s.pcLat);
               snprintf(pcLon, sizeof(pcLon), "%.9f", s.pcLon); }
        if (s.pcEpeM    <= 0.0) pcEpe[0] = '\0';    else snprintf(pcEpe, sizeof(pcEpe), "%.3f", s.pcEpeM);
        if (s.pcDistM   <  0.0) pcDist[0] = '\0';   else snprintf(pcDist, sizeof(pcDist), "%.3f", s.pcDistM);
        if (s.pcThreshM <= 0.0) pcThresh[0] = '\0'; else snprintf(pcThresh, sizeof(pcThresh), "%.3f", s.pcThreshM);
        if (s.posAccM < 0.0) posAcc[0] = '\0';
        else snprintf(posAcc, sizeof(posAcc), "%.3f", s.posAccM);
        if (!s.boardTempOk || isnan(s.boardTempF)) tBoard[0] = '\0';
        else snprintf(tBoard, sizeof(tBoard), "%.1f", s.boardTempF);
        // ms,utc_time,uptime_s,wifi,ssid,ip,fix,sats,base_ready,
        // svin_valid,svin_mode,svin_obs_s,svin_meanacc_m,hot_start,
        // rtcm_fps,rtcm_bps,valid_frames,
        // c0_host..c0_err, c1_host..c1_err, lat_wgs84_deg,lon_wgs84_deg,acc_m,esp_temp_f,imu_temp_f
        // ROW FIT (keep). The row must fit line[] and LogLine::text (equal sizes), or its
        // tail is cut: the trailing venue-check columns vanish and the row has the wrong
        // column count. Realistic values reach ~610 characters (long SSID, two caster hosts
        // and error strings), so a row that does not fit is rebuilt with the caster error
        // text blanked (the event file carries every caster error), then with the caster
        // host and mount blanked as well. Every attempt writes every column. Rebuilt rows
        // are counted (bridge_sdlog_status_trunc(), dashboard log.trunc) and the first is
        // recorded as an event.
        auto build = [&](const char* h0, const char* m0, const char* e0,
                         const char* h1, const char* m1, const char* e1) -> int {
            return snprintf(line, sizeof(line),
                     "%lu,%s,%lu,%d,%s,%s,%d,%d,%d,%u,%u,%lu,%s,%d,%lu,%lu,%llu,"
                     "%s,%s,%d,%s,%d,%llu,%llu,%lu,%ld,%s,"
                     "%s,%s,%d,%s,%d,%llu,%llu,%lu,%ld,%s,"
                     "%.9f,%.9f,%.3f,%s,%s,%lu,%lu,%lu,%lu,%lu,%lu,%s,%s,%lu,%lu,%lu,%u,%lu,%lu,%lu,"
                     "%s,%lu,%s,%s,%s,%s,%s",
                     (unsigned long)now, dt,
                     (unsigned long)(now / 1000UL),
                     s.wifiConnected ? 1 : 0, s.ssid, s.ip,
                     s.fixQuality, s.satellites, s.baseReady ? 1 : 0,
                     (unsigned)s.svinValid, (unsigned)s.svinMode,
                     (unsigned long)s.svinObs, accField, s.hotStart ? 1 : 0,
                     (unsigned long)s.rtcmFps, (unsigned long)s.rtcmBps,
                     (unsigned long long)s.rtcmValidFrames,
                     h0, m0, s.caster[0].enabled ? 1 : 0,
                     s.caster[0].state, s.caster[0].handshake ? 1 : 0,
                     (unsigned long long)s.caster[0].framesWritten,
                     (unsigned long long)s.caster[0].bytesAccepted,
                     (unsigned long)s.caster[0].dropped, (long)s.caster[0].lastWriteAgeS,
                     e0,
                     h1, m1, s.caster[1].enabled ? 1 : 0,
                     s.caster[1].state, s.caster[1].handshake ? 1 : 0,
                     (unsigned long long)s.caster[1].framesWritten,
                     (unsigned long long)s.caster[1].bytesAccepted,
                     (unsigned long)s.caster[1].dropped, (long)s.caster[1].lastWriteAgeS,
                     e1,
                     s.lat, s.lon, s.epeM, s.posSource, posAcc,
                     (unsigned long)peakLoopMs, (unsigned long)peakUartHw,
                     (unsigned long)s.rtcmCrcFailures, (unsigned long)s.rtcmFramingFailures,
                     (unsigned long)s.nmeaChecksumFailures,
                     (unsigned long)s.nmeaFramerDesyncs, tChip, tBoard,
                     // See BridgeLogSnapshot::heapFreeInt — the trend is the whole point.
                     (unsigned long)s.heapFreeInt, (unsigned long)s.heapLargestInt,
                     (unsigned long)s.heapMinInt,
                     (unsigned)s.localClients, (unsigned long)s.localServed,
                     // CUMULATIVE rows discarded because the queue was full. Any non-zero
                     // value means the Core-0 SD task stalled long enough to overflow
                     // BRIDGE_LOG_QUEUE_DEPTH; the sat channel, having by far the highest
                     // row rate, loses the most. The counter already existed but was never
                     // logged, so overflow could only be spotted by eye afterwards.
                     (unsigned long)g_dropped,
                     // CUMULATIVE sat and RTCM-detail rows skipped because the queue was nearly
                     // full (LOW-PRIORITY SHEDDING). Not losses of the diagnostic rows, but a
                     // hole in sat_/rtcm_ files that this column dates.
                     (unsigned long)g_shed,
                     s.pcState, (unsigned long)s.pcFixes,
                     pcLat, pcLon, pcEpe, pcDist, pcThresh);
        };
        int w = build(s.caster[0].host, s.caster[0].mount, s.caster[0].err,
                      s.caster[1].host, s.caster[1].mount, s.caster[1].err);
        if (w < 0 || w >= (int)sizeof(line)) {
            w = build(s.caster[0].host, s.caster[0].mount, "", s.caster[1].host, s.caster[1].mount, "");
            if (w < 0 || w >= (int)sizeof(line)) w = build("", "", "", "", "", "");
            if (g_statusTrunc++ == 0)
                bridge_sdlog_event("warn", "Status row too long for the log line - caster text "
                                           "blanked in rows that do not fit");
        }
        enqueueText(TAG_STATUS, line);
        peakLoopMs = 0;                       // next row reports its own interval
        peakUartHw = 0;
    }
}

void bridge_sdlog_feed_nmea(const char* s) {
    if (!g_enSat || !s) return;
    const char* p = s;
    if (*p == '$') ++p;
    if (strlen(p) < 6) return;
    if (strncmp(p + 2, "GSV", 3) != 0) return;     // only GSV carries per-sat data

    const uint32_t now = millis();
    // Capture one BRIDGE_LOG_SAT_WINDOW_MS window (a full multi-constellation GSV burst)
    // per period. FIRST SENTENCE KEPT (keep): the sentence that finds the previous window
    // expired is the FIRST of the next burst, so it closes the old window and is then
    // considered for a new one, never discarded: it carries the first four satellites of
    // the first constellation, which every snapshot needs. A new window opens once
    // the period less one window has passed, so a burst that arrives a few ms early is not
    // skipped or entered part-way through.
    if (g_satCapturing && now - g_satWindowStart > BRIDGE_LOG_SAT_WINDOW_MS) g_satCapturing = false;
    if (!g_satCapturing) {
        if (now - g_lastSatMs < BRIDGE_LOG_SAT_PERIOD_MS - BRIDGE_LOG_SAT_WINDOW_MS) return;
        g_satCapturing = true;
        g_satWindowStart = now;
        g_lastSatMs = now;
    }

    const char* cons = constellationName(p[0], p[1]);

    // Local split (don't touch the sketch's parser state).
    char work[200];
    strncpy(work, p, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';
    char* f[24] = {0};
    size_t n = 0;
    char* q = work;
    f[n++] = q;
    while (*q && n < 24) {
        if (*q == ',' || *q == '*') { *q = '\0'; f[n++] = q + 1; }
        ++q;
    }
    // f[0]=xxGSV f[1]=numMsg f[2]=msgNum f[3]=inView, then groups of 4, then (NMEA 4.10+)
    // the signal ID, then the checksum (split on '*' above, so f[n-1] is the checksum when
    // the sentence carried one). The data fields after the header are 4 per satellite plus
    // one when a signal ID is present.
    const bool   hasCs  = strchr(p, '*') != nullptr;
    const size_t data   = (n > 4) ? n - 4 - (hasCs ? 1 : 0) : 0;
    const bool   hasSig = (data % 4) == 1;
    const char*  sigId  = hasSig ? f[4 + data - 1] : "";
    const int    groups = (int)(data / 4);
    for (int g = 0; g < groups && g < 4; ++g) {
        const int base = 4 + g * 4;
        if (f[base][0] == '\0') continue;            // empty group = end of list
        char dt[32]; fmtDatetime(dt, sizeof(dt), g_lastDate, g_lastUtc);
        char line[128];
        snprintf(line, sizeof(line), "%lu,%s,%s,%s,%s,%s,%s,%s",
                 (unsigned long)now,
                 dt,
                 cons, f[base], f[base + 1], f[base + 2], f[base + 3], sigId);
        enqueueText(TAG_SAT, line);
    }
}

void bridge_sdlog_rtcm_frame(uint16_t type, uint16_t len, bool crcOk,
                             const char* c0, const char* c1,
                             const char* detail, const uint8_t* frame) {
    if (g_enRtcm) {
        char dt[32]; fmtDatetime(dt, sizeof(dt), g_lastDate, g_lastUtc);
        char line[200];
        snprintf(line, sizeof(line), "%lu,%s,%u,%u,%d,%s,%s,%s",
                 (unsigned long)millis(), dt, type, len, crcOk ? 1 : 0,
                 c0 ? c0 : "", c1 ? c1 : "", detail ? detail : "");
        enqueueText(TAG_RTCM, line);
    }
    // CRC-VALID FRAMES ONLY (keep). The .bin is the exact byte stream the casters were
    // offered, and a frame that failed its CRC is never offered to them; it is recorded in
    // the detailed CSV above (crc_ok=0) and counted in the status log instead.
    if (g_enRaw && crcOk && g_rawStream && frame && len && !g_sdMountFailed) {
        // ALL-OR-NOTHING (do not simplify back to a bare xStreamBufferSend). Stream
        // buffers do PARTIAL writes: with a 0-tick timeout xStreamBufferSend() copies
        // however many bytes happen to fit and returns that count. The old call
        // ignored the return value, so a nearly-full buffer spliced a TRUNCATED RTCM
        // frame into the .bin and desynced every parser downstream — the "isolated
        // single-epoch corruption that recovered one second later" seen in the field.
        // Only commit when the WHOLE frame fits; otherwise drop it intact and count it.
        if (xStreamBufferSpacesAvailable(g_rawStream) >= len) {
            xStreamBufferSend(g_rawStream, frame, len, 0);
        } else {
            g_dropped = g_dropped + 1;   // surfaces via bridge_sdlog_dropped()
        }
    }
}

// One event row. withUtc false leaves utc_time blank: rows built on the Core-0 SD task (mount,
// rotation) do not read g_lastDate / g_lastUtc, which loop() rewrites on Core 1 without a lock,
// so such a row can never carry a torn time; its ms column dates it.
static void eventRow(const char* level, const char* msg, bool withUtc) {
    if (!g_enEvent || msg == nullptr) return;
    char dt[32] = "";
    if (withUtc) fmtDatetime(dt, sizeof(dt), g_lastDate, g_lastUtc);
    // The message is free text from a printf at the call site, so commas and newlines
    // have to be neutralised or one event would shift every field after it — the same
    // failure the CSV column check exists to catch, arriving through the data instead of
    // the format string.
    char safe[224];   // must match logEvent()'s msg buffer or the file loses the tail
    size_t o = 0;
    for (const char* q = msg; *q && o < sizeof(safe) - 1; ++q) {
        const char c = *q;
        if (c == ',')                     safe[o++] = ';';
        else if (c == '\r' || c == '\n') safe[o++] = ' ';
        else if ((uint8_t)c < 0x20)       safe[o++] = ' ';
        else                              safe[o++] = c;
    }
    safe[o] = '\0';
    char line[288];
    snprintf(line, sizeof(line), "%lu,%s,%s,%s",
             (unsigned long)millis(), dt, level ? level : "info", safe);
    enqueueText(TAG_EVENT, line);
}

void bridge_sdlog_event(const char* level, const char* msg) { eventRow(level, msg, true); }

bool bridge_sdlog_logbook(const char* kind, const char* fields) {
    if (!g_queue || !g_sdReady || !kind || !fields) return false;
    char dt[32];
    if (!fmtDatetime(dt, sizeof(dt), g_lastDate, g_lastUtc)) return false;   // no UTC date yet
    LogLine ln;
    ln.tag = TAG_LOGBOOK;
    // file_no is the session number exactly as it appears in that session's file names
    // (gps_NNNN.csv ...), so each log file can be dated from the logbook alone.
    snprintf(ln.text, sizeof(ln.text), "%s,%04u,%lu,%s,%s",
             dt, (unsigned)g_session, (unsigned long)millis(), kind, fields);
    // The caller keeps a refused line and retries, so it waits rather than take a slot kept
    // for event rows (EVENT RESERVE).
    if (uxQueueSpacesAvailable(g_queue) <= BRIDGE_LOG_EVENT_RESERVE) return false;
    return xQueueSend(g_queue, &ln, 0) == pdTRUE;   // false: caller keeps it and retries
}

uint32_t bridge_sdlog_dropped() { return g_dropped; }
uint32_t bridge_sdlog_shed()    { return g_shed; }
// Lowest free stack the SD task has had, in bytes (ESP-IDF counts stack in bytes).
// 0 once the task has ended (card not mounted): the handle is read once and never used
// after the task cleared it (see ENDING THE TASK in sdTask()).
uint32_t bridge_sdlog_stack_free() {
    const TaskHandle_t h = g_sdTaskHandle;
    return (h && !g_sdMountFailed) ? (uint32_t)uxTaskGetStackHighWaterMark(h) : 0;
}
bool     bridge_sdlog_failed()  { return g_sdMountFailed; }
// loop() side: persists a session number the SD task moved past a collision. Runs on the
// loop's core so the NVS commit never comes from the SD task (see g_sessionPersistPending).
void     bridge_sdlog_service() {
    if (!g_sessionPersistPending) return;
    g_sessionPersistPending = false;
    Preferences p;
    if (p.begin("bridgelog", false)) { p.putUShort("session", g_session); p.end(); }
}
// True once the task has committed its files and gone idle for a download (or when there
// is no card task to wait for).
bool     bridge_sdlog_pause_acked() { return g_pauseAck || !g_sdReady; }
uint32_t bridge_sdlog_write_failures() { return g_writeFail; }
uint32_t bridge_sdlog_status_trunc() { return g_statusTrunc; }
bool     bridge_sdlog_ready()   { return g_sdReady; }
uint16_t bridge_sdlog_session() { return g_session; }
void     bridge_sdlog_set_download_active(bool a) {
    // Clear the acknowledgement BEFORE raising the pause, so a stale "idle" from the last
    // pause can never be read as an acknowledgement of this one.
    if (a) g_pauseAck = false;
    g_downloadActive = a;
}

bool bridge_sdlog_get_channel(int ch) {
    switch (ch) {
        case BLOG_GPS:    return g_enGps;
        case BLOG_SAT:    return g_enSat;
        case BLOG_STATUS: return g_enStatus;
        case BLOG_RTCM:   return g_enRtcm;
        case BLOG_RAW:    return g_enRaw;
        case BLOG_EVENT:  return g_enEvent;
        default:          return false;
    }
}

void bridge_sdlog_set_channel(int ch, bool on) {
    const char* key = nullptr;
    switch (ch) {
        case BLOG_GPS:    g_enGps = on;    key = "enGps";    break;
        case BLOG_SAT:    g_enSat = on;    key = "enSat";    break;
        case BLOG_STATUS: g_enStatus = on; key = "enStatus"; break;
        case BLOG_RTCM:   g_enRtcm = on;   key = "enRtcm";   break;
        case BLOG_RAW:    g_enRaw = on;    key = "enRaw";    break;
        case BLOG_EVENT:  g_enEvent = on;  key = "enEvent";  break;
        default: return;
    }
    Preferences p;                       // persist so the choice survives reboot
    p.begin("bridgelog", false);
    p.putBool(key, on);
    p.end();
}

#else  // BRIDGE_LOG_SD_ENABLE

void bridge_sdlog_init() {}
void bridge_sdlog_tick(const BridgeLogSnapshot&) {}
void bridge_sdlog_feed_nmea(const char*) {}
void bridge_sdlog_rtcm_frame(uint16_t, uint16_t, bool, const char*, const char*, const char*, const uint8_t*) {}
uint32_t bridge_sdlog_dropped() { return 0; }
uint32_t bridge_sdlog_shed()    { return 0; }
uint32_t bridge_sdlog_stack_free() { return 0; }
bool     bridge_sdlog_failed()  { return false; }
void     bridge_sdlog_service() {}
bool     bridge_sdlog_pause_acked() { return true; }
uint32_t bridge_sdlog_write_failures() { return 0; }
uint32_t bridge_sdlog_status_trunc() { return 0; }
bool     bridge_sdlog_logbook(const char*, const char*) { return false; }
void     bridge_sdlog_event(const char*, const char*) {}
bool     bridge_sdlog_ready()   { return false; }
uint16_t bridge_sdlog_session() { return 0; }
void     bridge_sdlog_set_download_active(bool) {}
bool     bridge_sdlog_get_channel(int) { return false; }
void     bridge_sdlog_set_channel(int, bool) {}

#endif // BRIDGE_LOG_SD_ENABLE
