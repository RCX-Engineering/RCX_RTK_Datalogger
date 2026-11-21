/*
 * sd_log.cpp — four-channel SD logger (GPS · IMU · CAN · SAT)
 * ============================================================
 * Each channel is independently toggleable and writes its own file.
 * All files share the same timestamp base (first valid GPS fix after boot).
 */

#include "sd_log.h"
#include "config.h"
#include "types.h"
#include "gnss.h"
#include "ntrip.h"
#include "can_bus.h"          // can_getSniffer() — gates the raw sniffer log file
#include "thermal.h"
#include "ble_racecapture.h"   // ble_linkState/ble_txDelivered/ble_txSuperseded — BLE health columns          // thermal_*() — LCD/SD throttling gates, see thermal.h
#include "can_porsche718_extra.h"
#include "display.h"            // display_coneModeActive() — full-rate IMU during cone surveys
#include <SD_MMC.h>
#ifndef BOARD_MAX_SDMMC_FREQ          // normally from the board variant; fall back to
#define BOARD_MAX_SDMMC_FREQ SDMMC_FREQ_DEFAULT   // the SD default clock (20 MHz)
#endif
#include <Preferences.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>        // settimeofday() — sets the system clock from GPS
#include <esp_heap_caps.h>   // heap_caps_malloc(MALLOC_CAP_SPIRAM) for queue storage
#include "breadcrumb.h"
#include "debug_log.h"      // LAST include: routes this file's Serial output through the SD mirror and
                            // declares DebugLog::drainToFile()/closeFile(), called from sdLogTask

#if defined(SD_LOG_ENABLE) && SD_LOG_ENABLE

// ── Log record: the GPS epoch row (loop, 20 Hz) and the decoded CAN snapshot (50 Hz active) ──
struct LogRecord {
    uint32_t ms;
    bool     valid;
    double   lat, lon, altMSL, geoidSepM;
    float    speedMph, headingDeg, hAccM, vAccM;
    int8_t   rtkType; uint8_t numSV; bool epeValid;
    uint16_t staId;   // GGA field 14: differential station ID. 9001/9002 = the module's own
                      // PPP solution (B2b/E6 HAS), NOT our base — rtk_type reads 1 (float)
                      // in that case, so this column is the ONLY way to tell PPP from a real
                      // RTK float in post-processing. 0 = no corrections in use.
    uint16_t year; uint8_t mon, day, hr, min, sec; uint16_t msec;
    bool     ntripConn; char mountpoint[20]; float ntripKm;
    int8_t   carrier; bool vrs; uint32_t rtcmTotal;
    float    rpm, tpsActual, pedalReq, targetTorque, actualTorque;
    float    vehicleSpeedKph, coolantC, oilC, oilBar, brakeBar;
    float    brakeSwitch, brakeSwitch2;
    float    wsFL, wsFR, wsRL, wsRR;   // kph
    float    steerDeg, steerRate;
    float    fuelLevel, kickdown, atmosphericKpa, fuelTempC, engineTempC;
    uint8_t  gear;
    float    pdkGearRaw, pdkSelectorRaw;

    // 718-only supplemental channels. These are appended to the CSV schema so
    // all legacy/shared columns retain their positions. Values are NAN on 987.2.
    bool     is718, gearValid718;
    float    intakeAirTempC, manifoldAbsPressureBar, massAirFlowGps;
    float    canLateralAccelG, canLongitudinalAccelG, canYawRateDegPerSec;
    float    clutchPositionPercent, outsideTempC, odometerKm;
    float    driveMode, psmMode, pdkState, pdkNoDriveOrFault;
    float    tpmsFrontLeftPsi, tpmsFrontRightPsi, tpmsRearLeftPsi, tpmsRearRightPsi;
    float    tpmsTempFrontLeftC, tpmsTempFrontRightC, tpmsTempRearLeftC, tpmsTempRearRightC;

    float    espTempC, imuTempC;   // device temps (°C); written to GPS log as °F
    bool     bleConn; float bleHz, gpsHz, canHz;
    float    diffAgeS;   // GGA field 13: age (s) of corrections in use; -1 = none — the direct 'how stale is my RTK feed' number
    uint8_t  bleState;   // 0 none, 1 connected, 2 subscribed — from ble_linkState()
    uint32_t bleDlv;     // cumulative samples fully delivered over BLE
    uint32_t bleSup;     // cumulative samples produced but never sent

    // Appended (like the 718 block) so all existing columns keep their positions.
    // Age of speed/heading at snapshot time: millis() - gps.velUpdateMillis.
    // ~0-15 normal; one epoch's worth (~50) = the row published via watchdog with
    // the previous epoch's velocity; larger = a real RMC outage. This column is
    // what turns an invisible stale-speed failure into a measured one.
    uint16_t spdAgeMs;
    // GNGSA-derived (1 Hz; values repeat across the ~20 rows of each second):
    // DOPs plus the per-system used-satellite IDs, appended as the trailing
    // columns. gsaSats contains no commas by construction (handleGNGSA).
    float    pdop, hdop, vdop;
    char     gsaSats[128];
    // Velocity provenance (gnss.cpp "Velocity selection"), appended last.
    // speed_mph/heading_deg above carry the PUBLISHED values (blank when invalid);
    // vel_src says where they came from; trk_* are the RTK-track values every
    // epoch, so one field session can validate the track against the receiver's
    // own fresh velocity and against CAN.
    uint8_t  velSrc;
    float    trkSpeedMph, trkHeadingDeg;
};

// ── Queues ────────────────────────────────────────────────────────────────────
static QueueHandle_t     logQueue   = nullptr;  // GPS epoch record (20 Hz)
static QueueHandle_t     imuQueue   = nullptr;  // IMU      250 Hz active / 1 Hz parked
static QueueHandle_t     satQueue   = nullptr;  // SAT GSV  1 Hz
static QueueHandle_t     nmeaQueue  = nullptr;  // raw GGA+RMC 40 lines/s (debug channel)
static QueueHandle_t     canRawQueue = nullptr; // CAN raw sniffer frames (bursty)
static QueueHandle_t     canQueue    = nullptr; // decoded CAN channel snapshots (own cadence)
static volatile uint32_t canRawDrops = 0;       // frames dropped (queue full)
static volatile uint32_t gpsRowDrops = 0;       // GPS epochs refused by a full epoch queue
static bool              gClockSet   = false;   // system clock set from GPS yet?
// Offset that converts a record's millis() timestamp to absolute unix milliseconds
// (unix_ms = record.ms + gUnixMsOffset, until the first millis() wrap). Captured
// once when the clock is set, pairing the GPS sample's UTC against that same
// sample's millis(); published in the canraw #UTC_SYNC marker. Rows themselves are
// converted by toUnixMs() below, which stays exact across the wrap.
static int64_t           gUnixMsOffset = 0;
// millis() is 32-bit and wraps every 49.7 days; a fixed offset added to it
// steps unix_ms back by 2^32 ms at the wrap while utc_time carries on. So
// conversion is relative to an anchor (a millis value and its unix time) using
// the SIGNED 32-bit difference, which is exact across the wrap, and the anchor
// is walked forward whenever a row is more than ~12 days past it, keeping every
// difference far inside int32 range for a session of any length. Single caller
// context (sdLogTask), so the anchor needs no lock. Before the clock is set the
// offset is 0 and rows carry raw millis, as the #UTC_SYNC marker documents.
static uint32_t gAnchorMs     = 0;
static int64_t  gAnchorUnixMs = 0;
static inline uint64_t toUnixMs(uint32_t ms) {
    if (!gClockSet) return (uint64_t)((int64_t)ms + gUnixMsOffset);
    int32_t d = (int32_t)(ms - gAnchorMs);
    if (d > 0x40000000) { gAnchorMs += (uint32_t)d; gAnchorUnixMs += d; d = 0; }
    return (uint64_t)(gAnchorUnixMs + d);
}

// UTC text for a row stamped only with unix milliseconds (the decoded CAN rows).
static void fmtUtcMs(uint64_t ums, char* out, size_t n) {
    time_t sec = (time_t)(ums / 1000);
    struct tm t; gmtime_r(&sec, &t);
    snprintf(out, n, "%04u-%02u-%02uT%02u:%02u:%02u.%03u", (unsigned)(t.tm_year + 1900) % 10000u,
             (unsigned)(t.tm_mon + 1) % 100u, (unsigned)t.tm_mday % 100u, (unsigned)t.tm_hour % 100u,
             (unsigned)t.tm_min % 100u, (unsigned)t.tm_sec % 100u, (unsigned)(ums % 1000));
}

// Raw CAN frame record for sniffer logging (16 bytes → cheap to queue in PSRAM).
struct CanRawRecord { uint32_t ms; uint16_t id; uint8_t dlc; uint8_t data[8]; };

void sdlog_push_can_raw(uint32_t ms, uint16_t id, uint8_t dlc, const uint8_t* data) {
    if (!canRawQueue) return;
    CanRawRecord r; r.ms = ms; r.id = id; r.dlc = (dlc > 8) ? 8 : dlc;
    for (int i = 0; i < 8; i++) r.data[i] = (i < r.dlc) ? data[i] : 0;
    // Timeout 0: never block the CAN task. Drop + count if the writer is behind.
    if (xQueueSendToBack(canRawQueue, &r, 0) != pdTRUE) canRawDrops++;
}
uint32_t sdlog_getCanRawDrops() { return canRawDrops; }

static volatile bool     sdReady    = false;
static SemaphoreHandle_t sdMutex    = nullptr;
// Session stamp as seen by other modules (debug_log names its file from it). sdLogTask owns the
// working copy; it publishes here once per pass. Reads from other tasks may see a stale or
// mid-copy name at worst for one pass, which only delays the debug file's open or rename.
static char              pubStamp[24] = "";
static char              activeLogName[LOG_NAME_MAX] = "";
static volatile uint32_t logFileCount      = 0;
static volatile uint32_t sdWriteFaults     = 0;   // see failChannel in sdLogTask
// Live on-disk size of each open log, for the display + web listing. Indexed by
// the setActive() type: 0=gps 1=can 2=imu 3=sat. Reflects flushed bytes (steps
// each flush) — the true size on the card. Published by sdLogTask every ~250 ms.
static volatile uint32_t activeBytesByType[5] = {0, 0, 0, 0, 0};

// ── Active-files RAM snapshot (for the default /files view) ───────────────────
// The web /files route used to enumerate the ENTIRE SD root under sdMutex —
// O(total files) — which on a full card holds the mutex long enough to make
// sdLogTask drop log rows (its write wait is only 50 ms). The dashboard now
// expands in three stages: ACTIVE (default) → last 5 → all. The default stage
// reads this RAM snapshot of the currently-open files and touches the SD card
// ZERO times, so routine refreshes never contend for sdMutex. The "last 5" and
// "all" stages do an on-demand scan (user-initiated, infrequent).
//   Types: 0=gps 1=can 2=imu 3=sat — matches the four log streams.
static char         activeByType[5][LOG_NAME_MAX] = {{0}};   // + 4 = nmea (debug)
// The CAN raw sniffer dump is held open by sdLogTask while the sniffer is on, but
// it is NOT one of the four typed channels above — so it must be tracked here too,
// or a web delete would try to SD_MMC.remove() it while open (FatFs corruption +
// device reset). Stored WITHOUT a leading slash to match activeByType / the dir
// snapshot naming. Written/read only under activeMux.
static char         canRawName[LOG_NAME_MAX]      = {0};

// Raw CAN hold. While held, the raw log file stays closed and its staging buffer is released; frames keep
// queueing in PSRAM (about ten seconds' worth) and are written to a new raw file when the hold ends.
// The NTRIP source-table scan uses this to free internal RAM while RTK is down.
static volatile bool sRawHold = false;
void sdlog_rawHold(bool hold) { sRawHold = hold; }
bool sdlog_rawIsOpen() { return canRawName[0] != '\0'; }
// The serial-mirror file (debug_log.cpp) is held open the same way and needs the same protection.
static char         debugFileName[LOG_NAME_MAX]   = {0};
static volatile uint32_t debugFileBytes = 0;   // live size of the open mirror file, for the open-file view
static portMUX_TYPE activeMux           = portMUX_INITIALIZER_UNLOCKED;

// ── Directory snapshot — for the web "Last 12" / "Show all" views ─────────────
// CRITICAL: the recent/all views MUST NOT enumerate the SD root from the web
// handler. That scan ran inside the ESPAsyncWebServer (async_tcp) callback, and
// on a card with many files it overran the 5 s Task-WDT → async_tcp abort
// (field capture: "task_wdt: async_tcp (CPU 1) ... Aborting"). It was also the
// exact long-mutex-hold-under-sdMutex pattern the active snapshot above exists to
// avoid (a multi-second root scan starves sdLogTask → dropped rows).
//   Fix: sdLogTask owns the enumeration. The web handler sets dirScanReq and
//   reads this RAM snapshot; the scan happens here, under sdMutex, on the logger
//   task (which is NOT WDT-subscribed). Web handlers touch SD/sdMutex ZERO times.
//   Bounded to the newest dirSnapCap files by timestamp — dirSnapTrunc flags
//   when the card held more than that and older entries were dropped.
//
// The two entry tables live in PSRAM. Internal SRAM is this board's scarcest
// resource (the NTRIP source-table scan needs ~30 KB of it transient and is
// skipped when it can't get that), and these tables are touched only by the
// logger task and the web handlers — never from an ISR, and never in a path
// where PSRAM's slower access matters. If PSRAM is unavailable the code falls
// back to a small internal table so the file views still work, just with the
// older, shorter listing.
#define DIR_SNAPSHOT_PSRAM    512   // entries when PSRAM backs the tables
// Entries when PSRAM is unavailable. Kept small deliberately: dirSnapFallback
// and dirScanTmpFallback below are declared unconditionally, so their bytes are
// reserved in internal SRAM whether or not this path is ever taken — on this
// board the PSRAM allocation succeeds (confirmed in the boot log's "file
// listing in PSRAM" line), so in normal operation this is 16 * 36 B * 2 = 1152 B
// of internal SRAM held for a path that doesn't run — versus 4608 B at the
// original 64. It only matters as a degraded-mode listing if PSRAM allocation
// genuinely fails, where a smaller, clearly-truncated listing (still comfortably
// above the 12 entries the "recent" view asks for) is the right tradeoff for the
// SRAM back.
#define DIR_SNAPSHOT_FALLBACK  16
// Files walked per sdLogTask pass while a directory scan is in progress. Small
// on purpose — see the scan's own comment: this bounds how long it can hold
// sdMutex at a stretch, which bounds how many GPS/CAN/IMU/SAT writes a scan
// can cost, regardless of how many files the card has accumulated.
#define DIR_SCAN_SLICE_FILES 15
struct DirSnapEntry { char name[LOG_NAME_MAX]; uint32_t size; };
// Internal-RAM tables used only when the PSRAM allocation fails.
static DirSnapEntry      dirSnapFallback[DIR_SNAPSHOT_FALLBACK];
static DirSnapEntry      dirScanTmpFallback[DIR_SNAPSHOT_FALLBACK];
static DirSnapEntry*     dirSnap      = nullptr;
static int               dirSnapCap   = 0;        // entries dirSnap/dirScanTmp hold
static int               dirSnapCount = 0;
static bool              dirSnapTrunc = false;
static volatile uint32_t dirSnapGen   = 0;        // bumps when a scan completes
static volatile bool     dirScanReq   = false;    // set once at boot only — see sd_log.h
static SemaphoreHandle_t dirSnapLock  = nullptr;  // guards the published snapshot

// State for the one time-sliced boot scan (sdlog_init sets dirScanReq once;
// serviced in sdLogTask — see its own comment there). File-scope so
// sdlog_dirScanPending() can see whether it's still running.
static File          dirScanDir;
static DirSnapEntry* dirScanTmp    = nullptr;
static int           dirScanCnt    = 0;
static bool          dirScanTrunc  = false;
static bool           dirScanActive = false;

// ── SD capacity cache ─────────────────────────────────────────────────────────
// /sd used to call SD_MMC.totalBytes()/usedBytes() (f_getfree) inside the web
// callback — same async_tcp TWDT exposure as the dir scan on a large/full card,
// since the first f_getfree can walk the whole FAT. sdLogTask refreshes these and
// the web serves the cached values with NO SD access.
static volatile uint64_t capTotal = 0, capUsed = 0;
static uint32_t          capLastMs = 0;
static volatile bool     capRefreshReq = false;   // a delete freed space: refresh now, not in 30 s

// ── Download priority ─────────────────────────────────────────────────────
// Count of file transfers currently in flight (web layer only — see sd_log.h
// for the contract). Gates the capacity refresh and directory scan below;
// never the recording write path.
static volatile int downloadActive = 0;
void sdlog_downloadBegin() { downloadActive++; }
void sdlog_downloadEnd()   { if (downloadActive > 0) downloadActive--; }
bool sdlog_downloadInProgress() { return downloadActive != 0; }

// Publish/clear the active name for one stream. Single writer (sdLogTask), so we
// can compare unlocked and only take the (tiny) critical section when it changes.
static void setActive(int type, const char* name) {
    if (type < 0 || type > 4) return;
    const char* v = name ? ((name[0] == '/') ? name + 1 : name) : "";
    if (strncmp(activeByType[type], v, 32) == 0) return;   // unchanged → no lock
    taskENTER_CRITICAL(&activeMux);
    strncpy(activeByType[type], v, LOG_NAME_MAX - 1);
    activeByType[type][LOG_NAME_MAX - 1] = '\0';
    taskEXIT_CRITICAL(&activeMux);
}
static uint32_t          lastLoggedEpochCs = 0xFFFFFFFF;

// ── Per-channel enables (NVS) ─────────────────────────────────────────────────
// These four are the EFFECTIVE flags the task acts on. While paused they are all
// false and the operator's intended set lives in wantGps/Imu/Can/Sat below.
static volatile bool logGps = true;
static volatile bool logImu = true;
static volatile bool logCan = false;
static volatile bool logSat = false;
static volatile bool logNmea = false;   // raw receiver capture (debug) — see sd_log.h

// ── Master pause ──────────────────────────────────────────────────────────────
// Pause forces every effective flag false and restores the operator's set on
// resume. It is expressed through the existing per-channel flags on purpose:
// clearing a flag already flushes and closes that channel's file (see sdLogTask),
// so pausing closes the open files, which is what makes them deletable from the
// web file view — the reason to pause in the first place. No separate file
// lifecycle exists for it.
//
// RAM only, never NVS: a unit must always come back logging after a power cycle,
// never silently stay paused because of how it was left days earlier.
static volatile bool logPaused = false;
static bool wantGps = true, wantImu = true, wantCan = false, wantSat = false, wantNmea = false;

static void loadLogConfig() {
    Preferences p; p.begin("rcx_log", true);
    logGps = p.getBool("gps", true);
    logImu = p.getBool("imu", true);
    logCan = p.getBool("can", false);
    logSat = p.getBool("sat", false);
    logNmea = p.getBool("nmea", false);
    p.end();
    // Boot is never paused, so the intended set and the effective set agree here.
    wantGps = logGps; wantImu = logImu; wantCan = logCan; wantSat = logSat; wantNmea = logNmea;
}
// Persists the INTENDED set (wantX), never the effective flags: while paused the
// effective flags are all false, and writing those would turn a pause into a
// permanent channel-off across the next reboot.
static void saveLogConfig() {
    Preferences p; p.begin("rcx_log", false);
    p.putBool("gps", wantGps);
    p.putBool("imu", wantImu);
    p.putBool("can", wantCan);
    p.putBool("sat", wantSat);
    p.putBool("nmea", wantNmea);
    p.end();
}

// ── sdlog_push_if_new ─────────────────────────────────────────────────────────
void sdlog_push_if_new() {
    // Bail if the queue isn't up yet, OR if neither channel that this record feeds
    // (GPS, CAN) is enabled. Without the second check we snapshot state and enqueue a
    // LogRecord at 20 Hz even when both are off — the 60-slot queue fills in ~3 s and
    // sdLogTask burns cycles dequeuing records it immediately discards. (IMU and SAT
    // pushes already guard on their own flags; this brings GPS/CAN in line.)
    if (!logQueue || (!logGps && !logCan)) return;

    if (!xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2))) return;
    uint32_t cs  = (uint32_t)gps.second * 100 + (uint32_t)(gps.millisecond / 10);
    uint32_t seq = gps.epochSeq;
    xSemaphoreGive(dataMutex);

    // ── Epoch-COMPLETE gating (Langley stale-speed fix) ──────────────────────
    // The old trigger (timestamp change) fired on GGA — one sentence BEFORE the
    // epoch's RMC — so speed/heading were snapshotted one epoch stale whenever
    // loop() landed inside the GGA→RMC gap (~35-40% of samples under motion).
    // Now: a timestamp change only ARMS the epoch; the row publishes when that
    // epoch's RMC has been applied (epochSeq bump), or after the watchdog if the
    // RMC is genuinely lost — flagged by spd_age_ms instead of silently latched.
    // Exactly one row per epoch: a straggler RMC landing after a watchdog publish
    // does not re-publish (csPending already cleared); it freshens the next epoch.
    static uint32_t lastSeq     = 0xFFFFFFFF;
    static uint32_t csChangedMs = 0;
    static bool     csPending   = false;
    if (cs != lastLoggedEpochCs) {
        lastLoggedEpochCs = cs;
        csChangedMs       = millis();
        csPending         = true;
    }
    if (!csPending) return;                                   // epoch already published
    if (seq == lastSeq && (millis() - csChangedMs) < GNSS_EPOCH_WATCHDOG_MS)
        return;                                               // armed; RMC still inbound
    lastSeq   = seq;
    csPending = false;

    LogRecord rec = {};
    if (!xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) return;

    rec.ms       = millis();
    rec.valid    = gps.valid;
    rec.lat      = gps.latitude;  rec.lon      = gps.longitude;
    rec.altMSL   = gps.altMSL;   rec.geoidSepM= gps.geoidSepM;
    rec.speedMph   = gps.speedValid   ? (float)(gps.speedKnots * 1.15078) : NAN;
    rec.headingDeg = gps.headingValid ? (float)gps.headingDeg : NAN;
    rec.velSrc        = gps.velSrc;
    rec.trkSpeedMph   = isnan(gps.trkSpeedMps) ? NAN : gps.trkSpeedMps * 2.236936f;
    rec.trkHeadingDeg = gps.trkHeadingDeg;
    { uint32_t age = gps.velUpdateMillis ? (millis() - gps.velUpdateMillis) : 65535u;
      rec.spdAgeMs = (uint16_t)(age > 65535u ? 65535u : age); }
    rec.pdop = gps.pdop; rec.hdop = gps.hdop; rec.vdop = gps.vdop;
    strlcpy(rec.gsaSats, gps.gsaSats, sizeof(rec.gsaSats));
    rec.hAccM    = (float)gps.hAccM;  rec.vAccM = (float)gps.vAccM;
    rec.rtkType  = gps.rtkType;  rec.numSV = gps.numSV;  rec.epeValid = gps.epeValid;
    rec.staId    = gps.diffStationId;
    rec.year = gps.year; rec.mon = gps.month;
    rec.day  = gps.day;  rec.hr  = gps.hour;
    rec.min  = gps.minute; rec.sec = gps.second; rec.msec = gps.millisecond;
    rec.ntripConn = status.ntripConnected;
    strncpy(rec.mountpoint, status.mountpoint, 19); rec.mountpoint[19] = '\0';
    rec.ntripKm = status.ntripDistanceKm;
    rec.carrier = status.ntripCarrier; rec.vrs = status.ntripVRS;
    rec.rtcmTotal = rtcmBytesTotal;
    rec.diffAgeS  = (float)gps.diffAgeS;
    rec.bleState  = ble_linkState();
    rec.bleDlv    = ble_txDelivered();
    rec.bleSup    = ble_txSuperseded();
    rec.bleConn = status.bleConnected;
    rec.bleHz = status.blePacketHz; rec.gpsHz = status.gnssHz; rec.canHz = status.canHz;
    rec.rpm             = can.rpm;
    rec.tpsActual       = can.throttleActualPct;
    rec.pedalReq        = can.pedalRequestedPct;
    rec.targetTorque    = can.targetTorquePct;
    rec.actualTorque    = can.actualTorquePct;
    rec.vehicleSpeedKph = can.vehicleSpeedKph;
    rec.coolantC        = can.coolantTempC;
    rec.oilC            = can.oilTempC;
    rec.oilBar          = can.oilPressBar;
    rec.brakeBar        = can.brakePressBar;
    rec.brakeSwitch     = can.brakeSwitch;
    rec.brakeSwitch2    = can.brakeSwitch2;
    rec.wsFL            = can.wsFL_kph;     rec.wsFR = can.wsFR_kph;
    rec.wsRL            = can.wsRL_kph;     rec.wsRR = can.wsRR_kph;
    rec.steerDeg        = can.steerAngleDeg;
    rec.steerRate       = can.steerRateDegPerSec;
    rec.fuelLevel       = can.fuelLevel;
    rec.kickdown        = can.kickdown;
    rec.atmosphericKpa  = can.atmosphericKpa;
    rec.fuelTempC       = can.fuelTempC;
    rec.engineTempC     = can.engineTempC;
    rec.gear            = can.gear;
    rec.pdkGearRaw      = can.pdkGearRaw;
    rec.pdkSelectorRaw  = can.pdkSelectorRaw;
    rec.espTempC        = status.espTempC;
    rec.imuTempC        = status.imuTempC;

    xSemaphoreGive(dataMutex);

    Porsche718ExtraData x718;
    rec.is718 = can_getPorsche718Extra(x718);
    rec.gearValid718 = rec.is718 && !isnan(x718.gearValid) && x718.gearValid > 0.5f;
    rec.intakeAirTempC         = rec.is718 ? x718.intakeAirTempC : NAN;
    rec.manifoldAbsPressureBar = rec.is718 ? x718.manifoldAbsPressureBar : NAN;
    rec.massAirFlowGps         = rec.is718 ? x718.massAirFlowGps : NAN;
    rec.canLateralAccelG       = rec.is718 ? x718.canLateralAccelG : NAN;
    rec.canLongitudinalAccelG  = rec.is718 ? x718.canLongitudinalAccelG : NAN;
    rec.canYawRateDegPerSec    = rec.is718 ? x718.canYawRateDegPerSec : NAN;
    rec.clutchPositionPercent  = rec.is718 ? x718.clutchPositionPercent : NAN;
    rec.outsideTempC           = rec.is718 ? x718.outsideTempC : NAN;
    rec.odometerKm             = rec.is718 ? x718.odometerKm : NAN;
    rec.driveMode              = rec.is718 ? x718.driveMode : NAN;
    rec.psmMode                = rec.is718 ? x718.psmMode : NAN;
    rec.pdkState               = rec.is718 ? x718.pdkState : NAN;
    rec.pdkNoDriveOrFault      = rec.is718 ? x718.pdkNoDriveOrFault : NAN;
    rec.tpmsFrontLeftPsi       = rec.is718 ? x718.tpmsFrontLeftPsi : NAN;
    rec.tpmsFrontRightPsi      = rec.is718 ? x718.tpmsFrontRightPsi : NAN;
    rec.tpmsRearLeftPsi        = rec.is718 ? x718.tpmsRearLeftPsi : NAN;
    rec.tpmsRearRightPsi       = rec.is718 ? x718.tpmsRearRightPsi : NAN;
    rec.tpmsTempFrontLeftC     = rec.is718 ? x718.tpmsTempFrontLeftC : NAN;
    rec.tpmsTempFrontRightC    = rec.is718 ? x718.tpmsTempFrontRightC : NAN;
    rec.tpmsTempRearLeftC      = rec.is718 ? x718.tpmsTempRearLeftC : NAN;
    rec.tpmsTempRearRightC     = rec.is718 ? x718.tpmsTempRearRightC : NAN;

    // A refused epoch is a GPS row that will never reach the card. Counted and reported (at most
    // every 10 s) with the other queues' depths, which show whether the writer is behind overall.
    if (xQueueSendToBack(logQueue, &rec, 0) != pdTRUE) {
        gpsRowDrops = gpsRowDrops + 1;   // one writer (the loop task)
        static uint32_t lastWarnMs = 0;
        if (millis() - lastWarnMs >= 10000) {
            lastWarnMs = millis();
            Serial.printf("⚠️  SD: GPS epoch queue full — %lu epochs not logged so far (raw CAN queue %u, IMU queue %u waiting)\n",
                          (unsigned long)gpsRowDrops,
                          canRawQueue ? (unsigned)uxQueueMessagesWaiting(canRawQueue) : 0u,
                          imuQueue ? (unsigned)uxQueueMessagesWaiting(imuQueue) : 0u);
        }
    }
}
uint32_t sdlog_getGpsRowDrops() { return gpsRowDrops; }

// ── Car activity (drives the IMU poll rate and the IMU / decoded-CAN row rates) ──
// "Active" = moving or recently moving. Speed above IMU_ACTIVE_SPEED_MPS (RTK-track speed or CAN
// vehicle speed) or any accel / gyro deviation keeps the car active; it becomes motionless only
// after IMU_MOTIONLESS_DWELL_MS of none of those.
//
// The receiver's own speed field is deliberately not one of the inputs here. That is a
// workaround, not a judgement of the receiver: with a healthy RTK solution its Doppler speed and
// heading are among the most accurate data the system has, better than its position. They were
// dropped from this decision because the receiver's speed and heading stall (see the stalled
// speed/heading notes in gnss.cpp), and a stalled value would hold the car "active" or "still"
// wrongly. The root cause is not yet fixed. Once a firmware fix makes the receiver's speed and
// heading dependably live, add them back as a speed input alongside the two above. Speed
// is what keeps a smooth straight run, which the accel/gyro thresholds alone would call still,
// at full rate. The float reads are lock-free: an aligned 32-bit read cannot tear, and one
// stale sample only shifts the rate change by a pass.
static volatile bool gCarActive = true;
bool sdlog_carActive() { return gCarActive; }

static bool psramRoomy();   // defined with the queue helpers below
static void updateCarActivity(float ax, float ay, float az, float gx, float gy, float gz) {
    static uint32_t stillSinceMs = 0;
    static bool     wasStill     = false;
    // Stillness is judged against a slow running average of the sensor's own output, not against
    // an ideal 1 g / 0 dps: a unit whose accelerometer scale reads 3 % low, or whose gyro carries a
    // bias of a couple of dps, is perfectly still yet sits permanently outside absolute thresholds.
    static uint32_t refMs   = 0;
    static float    refMag  = 1.0f, refGx = 0.0f, refGy = 0.0f, refGz = 0.0f;
    const uint32_t  nowMs   = millis();
    const float     mag     = sqrtf(ax * ax + ay * ay + az * az);
    if (refMs == 0) { refMag = mag; refGx = gx; refGy = gy; refGz = gz; }
    else {
        const float dt = (float)(nowMs - refMs);
        const float a  = dt / ((float)IMU_MOTIONLESS_REF_TAU_MS + dt);
        refMag += a * (mag - refMag);
        refGx  += a * (gx - refGx);  refGy += a * (gy - refGy);  refGz += a * (gz - refGz);
    }
    refMs = nowMs | 1;   // never 0 (0 = not yet seeded)
    const float accelDevG = fabsf(mag - refMag);
    const float gyroDps   = sqrtf((gx - refGx) * (gx - refGx) + (gy - refGy) * (gy - refGy) + (gz - refGz) * (gz - refGz));
    const float trk = gps.trkSpeedMps, cv = can.vehicleSpeedKph;
    const bool  fastEnough = (!isnan(trk) && trk > IMU_ACTIVE_SPEED_MPS) ||
                             (!isnan(cv)  && cv  > IMU_ACTIVE_SPEED_MPS * 3.6f);
    const bool  still = accelDevG < IMU_MOTIONLESS_ACCEL_G && gyroDps < IMU_MOTIONLESS_GYRO_DPS && !fastEnough;
    const uint32_t now = millis();
    if (still && !wasStill) stillSinceMs = now;   // just went still: start the dwell clock
    wasStill = still;
    gCarActive = !(still && !display_coneModeActive() && (now - stillSinceMs) >= IMU_MOTIONLESS_DWELL_MS);
}

// ── sdlog_push_imu ────────────────────────────────────────────────────────────
// Rate reduction lives HERE, at the producer side, specifically because this is
// the one call in the whole IMU path that only feeds SD logging — the BLE
// stream reads its own snapshot upstream of this function and never sees
// these skips, null or not. See config.h for the thresholds/timings. Called once per IMU
// sample (IMU_POLL_ACTIVE_MS apart while the car is active), and every sample becomes a row.
void sdlog_push_imu(float ax, float ay, float az, float gx, float gy, float gz) {
    updateCarActivity(ax, ay, az, gx, gy, gz);   // also steers the decoded-CAN rate when IMU logging is off
    if (!imuQueue || !logImu) return;
    if (thermal_satCanImuInhibit()) return;   // ≥112°C: SD logging paused (see thermal.h)

    // A return to motion clears the reduced rate on the very next sample; only the transition
    // INTO reduced-rate logging is debounced (the dwell in updateCarActivity). While reduced, the
    // last IMU_PREROLL_MS of samples are kept; when the car wakes they are written ahead of the
    // first full-rate row, so the onset of a launch is in the log at the full sample rate.
    static ImuLogRecord preRoll[IMU_PREROLL_MS / IMU_POLL_ACTIVE_MS + 1];
    static uint16_t     preHead = 0, preCount = 0;
    static bool         wasReduced = false;
    static uint32_t     lastImuWriteMs = 0;
    const uint32_t now = millis();
    const ImuLogRecord r = {now, ax, ay, az, gx, gy, gz};
    constexpr uint16_t kPre = sizeof(preRoll) / sizeof(preRoll[0]);

    if (!gCarActive) {
        preRoll[preHead] = r; preHead = (preHead + 1) % kPre; if (preCount < kPre) preCount++;
        wasReduced = true;
        if (lastImuWriteMs != 0 && (now - lastImuWriteMs) < IMU_MOTIONLESS_LOG_MS) return;
        lastImuWriteMs = now;
        xQueueSendToBack(imuQueue, &r, 0);
        return;
    }
    if (wasReduced) {
        wasReduced = false;
        const uint16_t first = (preHead + kPre - preCount) % kPre;   // oldest kept sample
        for (uint16_t i = 0; i < preCount; i++) {
            const ImuLogRecord& p = preRoll[(first + i) % kPre];
            if (p.ms > lastImuWriteMs) xQueueSendToBack(imuQueue, &p, 0);   // the sample already written is not repeated
        }
        preCount = 0; preHead = 0;
    }
    lastImuWriteMs = now;
    xQueueSendToBack(imuQueue, &r, 0);
}

// ── sdlog_push_can ────────────────────────────────────────────────────────────
// Decoded CAN channel snapshot: every CAN_LOG_ACTIVE_MS while the car is active, every
// CAN_LOG_MOTIONLESS_MS once it has been motionless. Driven from the IMU poll (loop(), core 1).
void sdlog_push_can() {
    if (!canQueue || !logCan) return;
    if (thermal_satCanImuInhibit()) return;   // ≥112°C: SD logging paused (see thermal.h)
    static uint32_t lastPushMs = 0, lastRingMs = 0;
    // Pre-roll: while reduced-rate, a snapshot is taken at the active cadence into a small ring
    // (PSRAM) and only one per CAN_LOG_MOTIONLESS_MS goes to the queue; when the car wakes the
    // ring is written first. Without PSRAM room there is simply no pre-roll.
    constexpr uint16_t kPre = 16;                 // 16 x CAN_LOG_ACTIVE_MS covers IMU_PREROLL_MS
    static_assert(kPre * CAN_LOG_ACTIVE_MS >= IMU_PREROLL_MS, "CAN pre-roll ring too short");
    static LogRecord* ring = nullptr; static bool ringTried = false;
    static uint16_t ringHead = 0, ringCount = 0; static bool wasReduced = false;
    const uint32_t now = millis();
    const bool active = gCarActive;
    if (!active && !ringTried) {
        ringTried = true;
        if (psramRoomy()) ring = (LogRecord*)heap_caps_malloc(kPre * sizeof(LogRecord), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    const bool queueDue = lastPushMs == 0 || (now - lastPushMs) >= (active ? CAN_LOG_ACTIVE_MS : CAN_LOG_MOTIONLESS_MS);
    const bool ringDue  = !active && ring && (lastRingMs == 0 || (now - lastRingMs) >= CAN_LOG_ACTIVE_MS);
    if (!queueDue && !ringDue) return;

    LogRecord rec = {};
    rec.ms = now;
    if (!xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2))) return;   // try again next sample
    rec.canHz           = status.canHz;
    rec.rpm             = can.rpm;
    rec.tpsActual       = can.throttleActualPct;
    rec.pedalReq        = can.pedalRequestedPct;
    rec.targetTorque    = can.targetTorquePct;
    rec.actualTorque    = can.actualTorquePct;
    rec.vehicleSpeedKph = can.vehicleSpeedKph;
    rec.coolantC        = can.coolantTempC;
    rec.oilC            = can.oilTempC;
    rec.oilBar          = can.oilPressBar;
    rec.brakeBar        = can.brakePressBar;
    rec.brakeSwitch     = can.brakeSwitch;
    rec.brakeSwitch2    = can.brakeSwitch2;
    rec.wsFL            = can.wsFL_kph;     rec.wsFR = can.wsFR_kph;
    rec.wsRL            = can.wsRL_kph;     rec.wsRR = can.wsRR_kph;
    rec.steerDeg        = can.steerAngleDeg;
    rec.steerRate       = can.steerRateDegPerSec;
    rec.fuelLevel       = can.fuelLevel;
    rec.kickdown        = can.kickdown;
    rec.atmosphericKpa  = can.atmosphericKpa;
    rec.fuelTempC       = can.fuelTempC;
    rec.engineTempC     = can.engineTempC;
    rec.gear            = can.gear;
    rec.pdkGearRaw      = can.pdkGearRaw;
    rec.pdkSelectorRaw  = can.pdkSelectorRaw;
    xSemaphoreGive(dataMutex);

    Porsche718ExtraData x718;
    rec.is718 = can_getPorsche718Extra(x718);
    rec.gearValid718 = rec.is718 && !isnan(x718.gearValid) && x718.gearValid > 0.5f;
    rec.intakeAirTempC         = rec.is718 ? x718.intakeAirTempC : NAN;
    rec.manifoldAbsPressureBar = rec.is718 ? x718.manifoldAbsPressureBar : NAN;
    rec.massAirFlowGps         = rec.is718 ? x718.massAirFlowGps : NAN;
    rec.canLateralAccelG       = rec.is718 ? x718.canLateralAccelG : NAN;
    rec.canLongitudinalAccelG  = rec.is718 ? x718.canLongitudinalAccelG : NAN;
    rec.canYawRateDegPerSec    = rec.is718 ? x718.canYawRateDegPerSec : NAN;
    rec.clutchPositionPercent  = rec.is718 ? x718.clutchPositionPercent : NAN;
    rec.outsideTempC           = rec.is718 ? x718.outsideTempC : NAN;
    rec.odometerKm             = rec.is718 ? x718.odometerKm : NAN;
    rec.driveMode              = rec.is718 ? x718.driveMode : NAN;
    rec.psmMode                = rec.is718 ? x718.psmMode : NAN;
    rec.pdkState               = rec.is718 ? x718.pdkState : NAN;
    rec.pdkNoDriveOrFault      = rec.is718 ? x718.pdkNoDriveOrFault : NAN;
    rec.tpmsFrontLeftPsi       = rec.is718 ? x718.tpmsFrontLeftPsi : NAN;
    rec.tpmsFrontRightPsi      = rec.is718 ? x718.tpmsFrontRightPsi : NAN;
    rec.tpmsRearLeftPsi        = rec.is718 ? x718.tpmsRearLeftPsi : NAN;
    rec.tpmsRearRightPsi       = rec.is718 ? x718.tpmsRearRightPsi : NAN;
    rec.tpmsTempFrontLeftC     = rec.is718 ? x718.tpmsTempFrontLeftC : NAN;
    rec.tpmsTempFrontRightC    = rec.is718 ? x718.tpmsTempFrontRightC : NAN;
    rec.tpmsTempRearLeftC      = rec.is718 ? x718.tpmsTempRearLeftC : NAN;
    rec.tpmsTempRearRightC     = rec.is718 ? x718.tpmsTempRearRightC : NAN;

    if (!active) {
        if (ring) { ring[ringHead] = rec; ringHead = (ringHead + 1) % kPre; if (ringCount < kPre) ringCount++; lastRingMs = now; }
        wasReduced = true;
        if (!queueDue) return;
    } else if (wasReduced) {
        wasReduced = false;
        if (ring) {
            const uint16_t first = (ringHead + kPre - ringCount) % kPre;
            for (uint16_t i = 0; i < ringCount; i++) {
                const LogRecord& p = ring[(first + i) % kPre];
                if (p.ms > lastPushMs) xQueueSendToBack(canQueue, &p, 0);   // the row already written is not repeated
            }
        }
        ringCount = 0; ringHead = 0;
    }
    lastPushMs = now;
    xQueueSendToBack(canQueue, &rec, 0);
}

// ── sdlog_push_nmea ───────────────────────────────────────────────────────────
// Called by gnss.cpp for every GGA/RMC line; a no-op unless the debug channel is on.
void sdlog_push_nmea(const char* nmeaString) {
    if (!nmeaQueue || !logNmea) return;
    if (thermal_satCanImuInhibit()) return;   // same thermal pause as the sat log
    NmeaLogRecord r;
    r.ms = millis();
    strlcpy(r.sentence, nmeaString, sizeof(r.sentence));
    r.sentence[strcspn(r.sentence, "\r\n")] = '\0';
    xQueueSendToBack(nmeaQueue, &r, 0);
}

// Rows each channel has written to its file, and what the SAT producer did with the rows it was
// handed. The ten-second line in sdLogTask prints them, so a channel that stops while its file is
// still open can be told apart from one that was switched off or never fed.
static volatile uint32_t rowsImu = 0, rowsCan = 0, rowsSat = 0, rowsNmea = 0, rowsGps = 0;
static volatile uint32_t satPushed = 0, satRefused = 0, satGated = 0;
// ── sdlog_push_sat ────────────────────────────────────────────────────────────
void sdlog_push_sat(const char* nmeaString) {
    if (!satQueue || !logSat) { satGated = satGated + 1; return; }
    if (thermal_satCanImuInhibit()) { satGated = satGated + 1; return; }   // ≥112°C: SD logging paused (see thermal.h)
    SatLogRecord r;
    r.ms = millis();
    strlcpy(r.sentence, nmeaString, sizeof(r.sentence));
    // Strip trailing newlines so println doesn't double-space the file
    r.sentence[strcspn(r.sentence, "\r\n")] = '\0'; 
    if (xQueueSendToBack(satQueue, &r, 0) == pdTRUE) satPushed = satPushed + 1;
    else                                              satRefused = satRefused + 1;
}

// ── Helpers ───────────────────────────────────────────────────────────────────
static void fmtF(char* b, size_t n, float v, int prec) {
    if (isnan(v)) { snprintf(b, n, ""); return; }
    char fmt[10]; snprintf(fmt, sizeof(fmt), "%%.%df", prec);
    snprintf(b, n, fmt, v);
}

// Record a freshly-opened file in the maintained "recent/all" snapshot —
// O(dirSnapCap) worst case, no directory access. Called by each of
// openFile()'s callers (gps/can/imu/sat/canraw) after they release sdMutex —
// deliberately NOT from inside openFile() itself, which every caller invokes
// while still holding sdMutex: nesting dirSnapLock inside that hold would
// couple two locks that have no reason to depend on each other. This is what
// makes the on-demand directory scan unnecessary after boot: every file the
// snapshot needs to know about, this firmware just created. Newest-first,
// matching the scan's ordering; the current oldest entry drops once
// dirSnapCap is reached, the same truncation the scan reports.
// The part of a log name that sorts chronologically: everything after the first underscore
// (<type>_<stamp>…), whatever the length of <type> — gps_, canraw_ and debug_ differ.
static const char* nameSortKey(const char* bare) {
    const char* u = strchr(bare, '_');
    return u ? u + 1 : bare;
}

bool sdlog_isLogFileName(const char* bare) {
    if (!bare) return false;
    const size_t n = strlen(bare);
    if (n > 4 && strcmp(bare + n - 4, ".csv") == 0) return true;
    return n > 4 && strncmp(bare, "debug_", 6) == 0 && strcmp(bare + n - 4, ".txt") == 0;
}

// A stored export archive. Deliberately separate from sdlog_isLogFileName(): that predicate also
// decides what goes INTO an archive, and an archive must never contain another archive.
bool sdlog_isArchiveName(const char* bare) {
    if (!bare) return false;
    const size_t n = strlen(bare);
    if (n <= 4 || n >= LOG_NAME_MAX || strcmp(bare + n - 4, ".tar") != 0) return false;
    return !strchr(bare, '/') && !strchr(bare, '\\');
}

static void snapshotNoteOpen(const char* name) {
    const char* bare = (name[0] == '/') ? name + 1 : name;
    if (!dirSnap || dirSnapCap <= 0) return;
    if (!dirSnapLock || xSemaphoreTake(dirSnapLock, pdMS_TO_TICKS(50)) != pdTRUE) return;
    int last = (dirSnapCount < dirSnapCap) ? dirSnapCount : dirSnapCap - 1;
    if (dirSnapCount >= dirSnapCap) dirSnapTrunc = true;
    else dirSnapCount++;
    for (int j = last; j > 0; j--) dirSnap[j] = dirSnap[j - 1];
    strncpy(dirSnap[0].name, bare, LOG_NAME_MAX - 1); dirSnap[0].name[LOG_NAME_MAX - 1] = '\0';
    dirSnap[0].size = 0;
    dirSnapGen++;
    xSemaphoreGive(dirSnapLock);
}

// Update a snapshot entry's size once its file closes — sizes never change
// again after that, so a single write here is the only update a closed entry
// ever needs; the "active" fast path (see sdlog_getActiveFiles) covers the
// file's size while it's still growing.
static void snapshotNoteClose(const char* name, uint32_t finalSize) {
    const char* bare = (name[0] == '/') ? name + 1 : name;
    if (!dirSnapLock || xSemaphoreTake(dirSnapLock, pdMS_TO_TICKS(50)) != pdTRUE) return;
    for (int i = 0; i < dirSnapCount; i++) {
        if (strcmp(dirSnap[i].name, bare) == 0) { dirSnap[i].size = finalSize; break; }
    }
    xSemaphoreGive(dirSnapLock);
}

void sdlog_noteDebugFileOpen(const char* bare) {
    if (!bare || !bare[0]) return;
    snapshotNoteOpen(bare);
    taskENTER_CRITICAL(&activeMux);
    strncpy(debugFileName, bare, sizeof(debugFileName) - 1);
    debugFileName[sizeof(debugFileName) - 1] = '\0';
    debugFileBytes = 0;
    taskEXIT_CRITICAL(&activeMux);
}
void sdlog_noteDebugFileSize(uint32_t bytes) { debugFileBytes = bytes; }
void sdlog_noteDebugFileClosed(const char* bare, uint32_t finalSize) {
    if (bare && bare[0]) snapshotNoteClose(bare, finalSize);
    taskENTER_CRITICAL(&activeMux);
    debugFileName[0] = '\0';
    taskEXIT_CRITICAL(&activeMux);
}

// Public (sd_log.h): remove a snapshot entry once its file is deleted. The
// incremental snapshot is never rebuilt from a fresh scan after boot, only
// ever appended to — so a deletion has to be told explicitly, or the entry
// ghosts in "Show all" forever. Deletion happens from the web task, not
// sdLogTask, hence public; only touches dirSnapLock, never sdMutex.
void sdlog_noteFileDeleted(const char* name) {
    const char* bare = (name[0] == '/') ? name + 1 : name;
    if (!dirSnapLock || xSemaphoreTake(dirSnapLock, pdMS_TO_TICKS(50)) != pdTRUE) return;
    for (int i = 0; i < dirSnapCount; i++) {
        if (strcmp(dirSnap[i].name, bare) == 0) {
            for (int j = i; j < dirSnapCount - 1; j++) dirSnap[j] = dirSnap[j + 1];
            dirSnapCount--;
            dirSnapGen++;
            capRefreshReq = true;   // the free-space figure is now stale
            break;
        }
    }
    xSemaphoreGive(dirSnapLock);
}

// The deep queues are sized for PSRAM. Without it they would land in the internal heap, so each
// falls back to its small size. 1 MB free is far more than they need.
static bool psramRoomy() { return heap_caps_get_free_size(MALLOC_CAP_SPIRAM) > (1024u * 1024u); }

// stdio buffer for the logs that write a few KB/s. It is above SPIRAM_MALLOC_ALWAYSINTERNAL
// (4 KB), so it lives in PSRAM and the six open files do not hold ~24 KB of the scarce internal
// heap. The ESP32-S3 SD host cannot DMA from PSRAM (SOC_SDMMC_PSRAM_DMA_CAPABLE is not set), so
// sdmmc_write_sectors writes such a buffer one 512 B sector per command through a bounce
// buffer. That is acceptable at these rates; the raw CAN log, which is most of the card traffic,
// does not use it (see the staging buffer below).
static const size_t kBulkBufBytes = 16384;

static bool openFile(File& f, const char* name, const char* header, size_t bufBytes) {
    f = SD_MMC.open(name, FILE_WRITE);
    if (!f) { Serial.printf("❌ SD: cannot open %s\n", name); return false; }
    // Must precede the first write. Above 4 KB the buffer needs PSRAM; without it the vfs
    // default (4 KB, internal) stays.
    if (bufBytes != 4096 && (bufBytes < 4096 || psramRoomy())) f.setBufferSize(bufBytes);
    f.println(header);
    // Flush the header NOW. Row-count-based flushing means a channel that never
    // (or slowly) produces rows leaves the header stranded in the FatFs buffer;
    // on power cut the file is a zero-byte stub. (Exactly how sat_*.csv shipped
    // as 0-byte files when the GSV producer was unwired.) One flush per file
    // open is negligible I/O and guarantees every created file is valid CSV.
    f.flush();
    return true;
}

// Guarded close: always holds sdMutex. Returns true if closed cleanly.
// The size recorded for the file list is read back from the directory entry
// AFTER the close. Before the close, size() is the entry as of the last flush
// (FatFs only updates it on sync) and misses the final stdio buffer — up to
// 4 KB short, which is how a paused session listed wrong sizes for closed files.
static bool closeFile(File& f, SemaphoreHandle_t m) {
    if (!f) return true;
    String   nm = f.name();
    String   pth = f.path();
    uint32_t sz = (uint32_t)f.position();   // fallback if the read-back fails
    if (xSemaphoreTake(m, pdMS_TO_TICKS(1000)) == pdTRUE) {
        f.close();
        File r = SD_MMC.open(pth.c_str(), FILE_READ);
        if (r) { sz = (uint32_t)r.size(); r.close(); }
        xSemaphoreGive(m);
    } else {
        f.close(); // last resort — log will warn
        Serial.println("⚠️  SD: close without mutex (bus busy >1s)");
    }
    f = File();
    snapshotNoteClose(nm.c_str(), sz);
    return true;
}

// A single failed println() here is very often NOT the card actually failing.
// ESP-IDF's SDMMC driver bounces every unaligned sector read/write through a
// freshly heap_caps_malloc'd 512 B MALLOC_CAP_DMA buffer — allocated and freed
// on EVERY call, never cached (confirmed open upstream: espressif/esp-idf#13749,
// and the same behaviour reported as far back as #6596 in 2021). 512 B is a
// trivially small ask, but MALLOC_CAP_DMA is a narrower pool than the general
// internal-heap figures this file's own diagnostics track — WiFi's own DMA
// buffers draw from the same pool and churn while a page response streams out,
// so a page load can transiently starve this one small allocation without
// internal free heap looking low at all. Confirmed in the field: a write here
// has failed with internal free heap sitting at its normal ~28-30 KB baseline,
// which only makes sense if the DMA-specific pool was momentarily tighter than
// that general figure. Both upstream issues describe this as transient — the
// buffer is freed again immediately after use — so a short retry is the right
// response, not immediately abandoning the file. Bounded to keep sdMutex's
// worst-case extra hold time small (2 x 5 ms here vs. the 50 ms timeout callers
// already wait for the mutex itself).
static bool printlnRetry(File& f, const char* row) {
    for (int attempt = 0; attempt < 3; attempt++) {
        if (f.println(row) != 0) return true;
        if (attempt < 2) vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

static bool printlnCounted(File& f, const char* row, volatile uint32_t& counter) {
    if (!printlnRetry(f, row)) return false;
    counter = counter + 1;
    return true;
}

// ── Raw CAN staging ───────────────────────────────────────────────────────────
// The raw CAN log is most of the card traffic (about 60 KB/s at 1,600 frames/s). Written row by
// row through stdio it reaches FatFs as partial sectors at arbitrary file offsets, and FatFs
// hands the SD host a buffer address that is then rarely 4-byte aligned, so the driver writes
// one 512 B sector per command through a bounce buffer: a saturated card, GPS rows lost while
// the raw log stays lossless. Rows are therefore collected in this internal-RAM buffer and
// written in blocks of whole sectors. The buffer's start address is kept congruent to the
// file offset modulo 4 (the header leaves the offset unaligned), so the address FatFs passes
// the host for the full sectors is aligned and the block goes out as one multi-block command.
// Every write is a multiple of 4 bytes, which keeps the offset phase; the 0–3 byte tail stays
// staged. The one-second tick writes what is staged and syncs, so a power cut still loses at
// most about a second.
static const size_t kRawStageWrite = 4096;                 // block written once this much is staged
static const size_t kRawStageBytes = kRawStageWrite + 128 + 4;   // plus a row or marker line, plus phase
static uint8_t* sRawStage = nullptr;   // allocation, internal DMA-capable RAM
static uint8_t* sRawData  = nullptr;   // first staged byte: sRawStage + (file offset & 3)
static size_t   sRawLen   = 0;

static bool rawStageAlloc() {
    if (!sRawStage) sRawStage = (uint8_t*)heap_caps_malloc(kRawStageBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    return sRawStage != nullptr;
}
static void rawStageFree() {
    if (sRawStage) heap_caps_free(sRawStage);
    sRawStage = sRawData = nullptr; sRawLen = 0;
}
// Call once the file is open and its header written.
static void rawStageBegin(File& f) { sRawLen = 0; sRawData = sRawStage + ((uint32_t)f.position() & 3u); }
static inline void rawStageAppend(const char* s, size_t n) {
    if (sRawLen + n > kRawStageBytes - 4) return;   // cannot happen: written out at 4 KB and lines are under 128 B
    memcpy(sRawData + sRawLen, s, n); sRawLen += n;
}
// Writes the staged bytes: whole sectors once a block is ready, any multiple of 4 on the tick,
// everything when all is set (file closing). Caller holds sdMutex.
static bool rawStageWrite(File& f, bool tick, bool all) {
    size_t n = all ? sRawLen : tick ? (sRawLen & ~(size_t)3) : (sRawLen >= kRawStageWrite ? (sRawLen & ~(size_t)511) : 0);
    if (!n) return true;
    if (f.write(sRawData, n) != n) return false;
    sRawLen -= n; memmove(sRawData, sRawData + n, sRawLen);
    return true;
}

// Writes out what is staged and releases the buffer; call just before the file is closed.
static void rawStageFinish(File& f, SemaphoreHandle_t m) {
    if (sRawStage && f && xSemaphoreTake(m, pdMS_TO_TICKS(1000)) == pdTRUE) {
        rawStageWrite(f, false, true);
        xSemaphoreGive(m);
    }
    rawStageFree();
}

// Build a log filename that does NOT already exist on the card, so re-enabling
// logging mid-session never truncates a previous file (SD_MMC.open uses "w" =
// truncate). Tries "/<prefix>_<stamp>.csv" first, then "_2", "_3", … Must be
// called while holding sdMutex (exists() touches the SD bus).
static void uniqueLogName(char* out, size_t cap, const char* prefix, const char* stamp) {
    snprintf(out, cap, "/%s_%s.csv", prefix, stamp);
    if (!SD_MMC.exists(out)) return;
    for (int n = 2; n < 1000; n++) {
        snprintf(out, cap, "/%s_%s_%d.csv", prefix, stamp, n);
        if (!SD_MMC.exists(out)) return;
    }
}

// Pick the next free /canraw_NNNN.csv ordinal by scanning the card root. Used
// only when UTC isn't available yet (no GPS lock), so the sniffer file still gets
// a clean, human-sortable name instead of the old cryptic boot-millis value.
// Matches canraw_<digits>.csv ONLY — a UTC-stamped canraw_YYYYMMDD_HHMMSS.csv is
// not all-digits after the prefix, so it never collides with the ordinal series.
// Caller must hold sdMutex (this enumerates the root directory).
static void nextCanRawName(char* out, size_t cap) {
    int maxN = 0;
    File dir = SD_MMC.open("/");
    if (dir) {
        for (;;) {
            File f = dir.openNextFile();
            if (!f) break;
            String nm = String(f.name());
            f.close();
            const char* bare = nm.startsWith("/") ? nm.c_str() + 1 : nm.c_str();
            if (strncmp(bare, "canraw_", 7) == 0) {
                const char* p = bare + 7;
                int n = 0; bool allDigits = (*p != '\0' && *p != '.');
                for (; *p && *p != '.'; p++) {
                    if (*p < '0' || *p > '9') { allDigits = false; break; }
                    n = n * 10 + (*p - '0');
                }
                if (allDigits && n > maxN) maxN = n;
            }
        }
        dir.close();
    }
    snprintf(out, cap, "/canraw_%04d.csv", maxN + 1);
}

// ── Bulk export (.tar) ────────────────────────────────────────────────────────
// "Download all" builds ONE uncompressed tar of every .csv here, in sdLogTask
// (off the async_tcp task, off the WDT), then the web streams that single file
// with the existing single-file streamer. This scales to any file count, never
// parks a huge response in internal RAM, and cannot trip the Task-WDT. The build
// holds sdMutex for its duration, so logging should be stopped while exporting —
// a concurrent recording will see queue gaps (no crash) while the build runs.
// The archive is stored under its own name and stays on the card after the build, listed with the
// logs, so an interrupted download is repeated from its link instead of rebuilt. Set by
// sdlog_requestExport(); read by the web once the state is READY.
static char exportPath[LOG_NAME_MAX + 1] = "/rcx_logs_all.tar";
enum { EXP_IDLE = 0, EXP_BUILDING = 1, EXP_READY = 2, EXP_ERROR = 3 };
static volatile int      exportState = EXP_IDLE;
static volatile bool     exportReq   = false;
static volatile uint64_t exportSize  = 0;
static volatile int      exportCount = 0;     // files included so far (for UI progress)
static volatile bool     exportIsSubset = false;  // last-requested export was a selection, not "all"

// A selected-file export names its members explicitly instead of walking the
// card root. Bounded the same way the batch-delete route bounds its input —
// not for mutex-hold reasons (the full "download all" walk already holds
// sdMutex for however many files exist; a subset never holds it longer), just
// to keep this static table's footprint predictable. 64 * 32 B = 2048 B.
#define EXPORT_SELECT_MAX 64
static char exportSelectNames[EXPORT_SELECT_MAX][LOG_NAME_MAX];
static int  exportSelectCount = 0;

// ustar header (validated against GNU tar). `width` includes the trailing nul.
//
// The digits are produced directly rather than through snprintf. The previous
// version built a format string ("%011o") and passed the value as
// `(unsigned long long)` — but %o is an `unsigned int` conversion, so a 64-bit
// argument against it is a type mismatch and undefined. It happens to work on a
// 64-bit host, which is why it reads as correct code; on xtensa a 64-bit
// argument is passed in an even-aligned register pair, so the 32-bit conversion
// reads a different slot entirely and prints whatever is in it. Field evidence:
// both 12-wide fields (size and mtime) came out as octal 00000000013 = 11 —
// exactly the `width - 1` that the format-building snprintf on the line above
// had just passed in that argument position. Archives were unopenable because
// every file claimed to be 11 bytes long.
//
// Doing the conversion by hand removes the varargs path altogether: no format
// string, no promotion rules, identical behaviour on every target. A value too
// large for the field silently keeps its low digits, which for the 11-digit size
// field means files up to 8 GB are represented exactly.
static void tarOctal(char* dst, int width, uint64_t v) {
    const int digits = width - 1;
    for (int i = digits - 1; i >= 0; i--) { dst[i] = (char)('0' + (int)(v & 7)); v >>= 3; }
    dst[digits] = '\0';
}
static void tarHeader(uint8_t h[512], const char* name, uint64_t size, uint32_t mtime) {
    memset(h, 0, 512);
    strncpy((char*)h + 0, name, 99);
    tarOctal((char*)h + 100, 8, 0644);    // mode
    tarOctal((char*)h + 108, 8, 0);       // uid
    tarOctal((char*)h + 116, 8, 0);       // gid
    tarOctal((char*)h + 124, 12, size);   // size
    tarOctal((char*)h + 136, 12, mtime);  // mtime
    memset(h + 148, ' ', 8);              // checksum field = spaces while summing
    h[156] = '0';                         // typeflag: regular file
    memcpy(h + 257, "ustar", 5); h[263] = '0'; h[264] = '0';
    unsigned sum = 0; for (int i = 0; i < 512; i++) sum += h[i];
    char cs[8]; snprintf(cs, sizeof(cs), "%06o", sum & 0x3FFFF);
    memcpy(h + 148, cs, 6); h[154] = '\0'; h[155] = ' ';
}

static void buildExport() {
    exportState = EXP_BUILDING; exportSize = 0; exportCount = 0;
    const size_t BUFSZ = 4096;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(BUFSZ, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) buf = (uint8_t*)malloc(BUFSZ);      // PSRAM preferred; fall back if absent
    if (!buf) { exportState = EXP_ERROR; Serial.println("📦 Export: no buffer"); return; }

    bool ok = false;
    bool touched = false;                         // the archive's file was replaced or removed
    uint32_t finalSize = 0;
    if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        SD_MMC.remove(exportPath);                // rebuilding a name replaces the earlier archive
        touched = true;
        File tar = SD_MMC.open(exportPath, FILE_WRITE);
        if (tar) {
            uint8_t hdr[512];
            // Header + copy + pad + progress counters, shared by both the "walk
            // everything" and "walk this selection" branches below so they can't
            // silently drift apart.
            // Every member MUST occupy exactly 512 + size + pad bytes in the
            // stream. A tar has no framing: the reader finds header N+1 by
            // counting forward from header N using the size written in header N.
            // Emit one byte fewer than declared and every subsequent header lands
            // mid-content — the extractor reads file data as a header, finds no
            // magic, and reports the archive as garbage. That is the corrupt-tar
            // symptom, and it does not look like a read error, it looks like the
            // archive itself is malformed.
            //
            // The old loop broke out on a short or failed read and then padded
            // from the DECLARED size, so any read failure silently shortened the
            // member and misaligned everything after it. SD reads DO fail here:
            // under DMA-pool exhaustion the driver returns 0x101 and read()
            // comes back short. So this now (a) retries a short read before
            // giving up, and (b) if it still cannot fill the member, zero-fills
            // the shortfall so the geometry the header promised is honoured
            // regardless. A member with a zeroed tail is recoverable; a
            // misaligned archive is not.
            bool degraded = false;
            bool writeFailed = false;   // the card refused data (full, or a write error): the archive is incomplete
            auto writeExact = [&](const uint8_t* p, size_t n) -> bool {
                // FatFs short writes misalign the archive exactly as short reads
                // do, so the return value is checked rather than discarded.
                size_t w = tar.write(p, n);
                if (w != n) { degraded = true; writeFailed = true; return false; }
                return true;
            };
            auto padZeros = [&](uint64_t n) -> bool {
                while (n) {
                    size_t blk = (n > BUFSZ) ? BUFSZ : (size_t)n;
                    memset(buf, 0, blk);
                    if (!writeExact(buf, blk)) return false;
                    n -= blk;
                }
                return true;
            };

            auto addFile = [&](File& f, const char* bareName) {
                uint64_t fsz = f.size();
                time_t mt = f.getLastWrite(); if (mt <= 0) mt = time(nullptr);
                tarHeader(hdr, bareName, fsz, (uint32_t)mt);
                if (!writeExact(hdr, 512)) {
                    Serial.printf("📦 Export: header write failed for %s\n", bareName);
                    return;
                }
                uint64_t done = 0;
                int shortReads = 0;
                while (done < fsz) {
                    size_t want = (fsz - done > BUFSZ) ? BUFSZ : (size_t)(fsz - done);
                    int r = f.read(buf, want);
                    if (r <= 0) {
                        // Transient under DMA pressure — the same 0x101 path the
                        // log writers already retry through. Give the driver a
                        // tick to recover before treating it as terminal.
                        if (++shortReads <= 3) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
                        Serial.printf("📦 Export: read stalled in %s at %llu/%llu B — "
                                      "zero-filling remainder to keep the archive aligned\n",
                                      bareName, (unsigned long long)done,
                                      (unsigned long long)fsz);
                        degraded = true;
                        if (!padZeros(fsz - done)) return;
                        done = fsz;
                        break;
                    }
                    shortReads = 0;
                    if (!writeExact(buf, (size_t)r)) {
                        Serial.printf("📦 Export: write failed in %s\n", bareName);
                        return;
                    }
                    done += (uint64_t)r;
                }
                const size_t pad = (512 - (size_t)(fsz % 512)) % 512;   // pad to 512
                if (pad && !padZeros(pad)) return;
                exportCount++;
                exportSize += 512 + fsz + pad;   // declared geometry, not bytes read
                vTaskDelay(1);                   // breathe between files
            };

            if (exportSelectCount > 0) {
                // Selected subset: open each named file directly rather than
                // walking the root. A name that's since been deleted is skipped
                // silently (nothing to include); one still open for writing is
                // also skipped — reading a file mid-write here would race the
                // logger's own writes to it, the same reason deletes refuse an
                // active file.
                for (int i = 0; i < exportSelectCount; i++) {
                    // The serial mirror is the exception: it stays open for the whole
                    // session, but its writer (DebugLog::drainToFile) runs on this same
                    // task and cannot run while this build holds sdMutex, so the copy
                    // taken here is consistent up to its last ~1 s flush. Without this the
                    // debug file could never be part of a session archive.
                    const bool mirror = strncmp(exportSelectNames[i], "debug_", 6) == 0;
                    if (!mirror && sdlog_isActiveFile(exportSelectNames[i])) continue;
                    String path = "/" + String(exportSelectNames[i]);
                    File f = SD_MMC.open(path, FILE_READ);
                    if (!f) continue;
                    addFile(f, exportSelectNames[i]);
                    f.close();
                }
            } else {
                File dir = SD_MMC.open("/");
                if (dir) {
                    for (;;) {
                        File f = dir.openNextFile();
                        if (!f) break;
                        String nm = String(f.name());
                        const char* bare = nm.startsWith("/") ? nm.c_str() + 1 : nm.c_str();
                        if (!sdlog_isLogFileName(bare)) { f.close(); continue; }   // the logs and the serial mirror only
                        addFile(f, bare);
                        f.close();
                    }
                    dir.close();
                }
            }

            memset(buf, 0, 512);
            writeExact(buf, 512); writeExact(buf, 512);   // tar EOF: 2 zero blocks
            tar.flush();
            exportSize = tar.size();
            finalSize = (uint32_t)exportSize;
            const bool aligned = (exportSize % 512) == 0;
            tar.close();
            // Both of these are structural: a length that is not a multiple of
            // 512 cannot be a valid tar, and `degraded` means at least one member
            // was zero-filled or a write came up short. Say so on the console —
            // the archive still extracts, but the operator should know which one
            // they are holding before they delete the originals from the card.
            if (!aligned || degraded) {
                Serial.printf("📦 Export: ⚠️  archive is %s (%llu B, %s) — "
                              "verify before deleting source files\n",
                              aligned ? "degraded" : "MISALIGNED",
                              (unsigned long long)exportSize,
                              aligned ? "512-aligned" : "NOT 512-aligned");
            }
            // The archive now persists, so an incomplete one must not be left behind to be
            // downloaded later as though it were whole. A full card is the usual cause.
            if (writeFailed) {
                SD_MMC.remove(exportPath);
                Serial.printf("📦 Export: card refused writes (full?) — partial %s removed\n", exportPath + 1);
            } else {
                ok = true;
            }
        }
        xSemaphoreGive(sdMutex);
    }
    free(buf);
    // Listing bookkeeping runs after sdMutex is released: the snapshot has its own lock and the two
    // are never nested (see sdlog_noteFileDeleted). The old entry goes first so a rebuilt name is
    // listed once, then the new archive is added newest-first with its final size.
    if (touched) {
        sdlog_noteFileDeleted(exportPath);
        if (ok) { snapshotNoteOpen(exportPath); snapshotNoteClose(exportPath, finalSize); }
        capRefreshReq = true;                     // the archive took (or returned) card space
    }
    exportState = ok ? EXP_READY : EXP_ERROR;
    Serial.printf("📦 Export %s: %d files, %llu bytes%s\n",
                  ok ? "ready" : "FAILED", exportCount, (unsigned long long)exportSize,
                  exportIsSubset ? " (selection)" : "");
}

// ── sdLogTask ─────────────────────────────────────────────────────────────────
void sdLogTask(void*) {
    delay(800);
    crumb::mark(crumb::SLOT_SDLOG, crumb::S_OPEN);   // card mount and first open are slow and belong to this phase
    SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN, -1, -1, SD_D3_PIN);
    // maxOpenFiles=10 (was the library default of 5). With all four channels
    // logging, gps+imu+can+sat already hold 4 descriptors; the default 5 left
    // exactly 1, so a download, a "Last 12" scan, or the .tar export hit
    // "no free file descriptors" → open() returns invalid → 404 "not found",
    // empty file lists, and 0-file exports. 10 covers 4 logs + 2 simultaneous
    // downloads + a scan (dir+file) with margin. (5th arg requires passing the
    // frequency arg too; BOARD_MAX_SDMMC_FREQ keeps the library's default clock.)
    while (!SD_MMC.begin("/sdcard", true, false, BOARD_MAX_SDMMC_FREQ, 10)) {
        Serial.println("❌ SD: init failed — retry in 5s");
        SD_MMC.end(); delay(5000);
        if (!sdMutex) { vTaskDelete(nullptr); return; }
    }
    Serial.printf("✅ SD: ready, %llu MB free\n",
                  (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / 1048576ULL);
    loadLogConfig();
    sdReady = true;
    dirScanReq = true;   // ONE boot-time scan to discover files from past power
                          // cycles; everything created this session is added
                          // incrementally by openFile()/closeFile() below — see
                          // snapshotNoteOpen/snapshotNoteClose. Nothing else ever
                          // sets this flag again.

    // (Removed) boot_events.csv per-boot trail. The reset reason is still surfaced
    // live on /status ("Last Reset") and stamped into the GPS log, so nothing is
    // lost by dropping the standalone file that cluttered the dashboard file list.

    File    gpsFile, imuFile, canFile, satFile;
    File    nmeaFile;                           // raw receiver capture (debug channel)
    uint32_t flushNmea = 0; bool nmeaErr = false;
    NmeaLogRecord nrec;
    LogRecord     crec;                         // decoded CAN snapshot being written
    File    canRawFile;                         // sniffer raw-frame log (separate file)
    bool    gpsHdr = false, imuHdr = false, canHdr = false, satHdr = false;
    // Each channel flushes on a one-second clock (millis() of its last flush), not every N rows:
    // the flush cadence is then independent of the row rate, so the exposure on power loss is
    // at most about one second per channel whatever the CAN load. Reset to 0 on open.
    uint32_t flushGps = 0, flushImu = 0, flushCan = 0, flushSat = 0, flushCanRaw = 0;
    bool     canRawSyncWritten = false;   // UTC marker written for the OPEN canraw file?
    uint32_t lastOpen  = 0;
    bool    gpsErr = false, canErr = false, imuErr = false, satErr = false;
    char    sessionStamp[24] = "";
    // ── Write-failure recovery ───────────────────────────────────────────────
    // PROTECTIVE NOTE: a failed write leaves the FatFs handle in its sticky error
    // state (FIL.err, set by ABORT) — every later write on that handle fails too,
    // until it is closed. The SDMMC driver produces exactly such transient
    // failures (per-call 512 B DMA bounce buffer, ESP_ERR_NO_MEM under DMA-pool
    // pressure — see printlnRetry). So ANY channel's failed write closes that
    // channel's file at once, and it reopens within the SAME session stamp
    // (uniqueLogName() gives gps_<stamp>_2.csv …), so the session stays one
    // group on the dashboard and in its archive. Previously IMU and SAT only
    // cleared a header flag and kept writing into the dead handle for the rest
    // of the session, and GPS discarded the session stamp, so GPS and CAN came
    // back as a different session.
    // A card that keeps failing (or is full: open() still succeeds, the first
    // write fails) must not be answered with a new file every half second, so
    // the reopen delay doubles with each fault inside a minute of the last, up
    // to a minute, and no reopen is attempted while the card is nearly full.
    bool     recovering   = false;    // a channel failed; reopen it in this session
    uint32_t recoverAt    = 0;        // next reopen attempt (millis)
    uint32_t recoverDelay = 0;        // current reopen delay (backoff)
    uint32_t lastFaultMs  = 0;
    bool     fullNoted    = false;    // "card full" logged for this recovery
    const uint32_t SD_RECOVER_MIN_MS = 500, SD_RECOVER_MAX_MS = 60000;
    const uint64_t SD_MIN_FREE_BYTES = 4ULL * 1024 * 1024;
    auto failChannel = [&](File& f, const char* what) {
        closeFile(f, sdMutex);
        const uint32_t writeFaults = sdWriteFaults + 1;   // single writer (sdLogTask)
        sdWriteFaults = writeFaults;
        const bool streak = lastFaultMs != 0 && (millis() - lastFaultMs) < 60000;
        recoverDelay = streak ? (recoverDelay * 2 > SD_RECOVER_MAX_MS ? SD_RECOVER_MAX_MS : recoverDelay * 2)
                              : SD_RECOVER_MIN_MS;
        lastFaultMs  = millis();
        recovering   = true;
        fullNoted    = false;
        recoverAt    = millis() + recoverDelay;
        capRefreshReq = true;                 // a full card is the usual cause: re-measure it
        Serial.printf("⚠️  SD %s: write failed — file closed, reopening in this session in %lus (fault #%lu)\n",
                      what, (unsigned long)((recoverDelay + 999) / 1000), (unsigned long)writeFaults);
    };

    LogRecord    rec;
    ImuLogRecord irec;
    SatLogRecord srec;

    for (;;) {
        crumb::mark(crumb::SLOT_SDLOG, crumb::S_PUBLISH);
        // ── Publish the currently-open files for the default web view ──────
        // Derived from the live File handles every pass, so we never have to
        // instrument each close site. Names are reconstructed from the shared
        // sessionStamp; setActive() is a no-op when nothing changed.
        {
            // The handle's own name, never one rebuilt from sessionStamp: a file
            // reopened within a session carries a uniqueLogName() suffix (_2, _3…),
            // and a rebuilt name would protect the CLOSED original from web delete
            // while leaving the live file deletable (FatFs corruption + reset).
            setActive(0, gpsFile ? gpsFile.name() : nullptr);
            setActive(1, canFile ? canFile.name() : nullptr);
            setActive(2, imuFile ? imuFile.name() : nullptr);
            setActive(3, satFile ? satFile.name() : nullptr);
            setActive(4, nmeaFile ? nmeaFile.name() : nullptr);
            // Ten-second account of every channel: rows written, what the SAT producer was handed and
            // did with it, the queue depths, which channels are switched on and which have a file
            // open. It goes to the serial mirror like every other line, so a channel that goes quiet
            // while the others run shows here which of those it is: switched off (flag false), never
            // fed (pushed flat, gated rising), fed but not drained (pushed rising, queued rising),
            // or draining into a file that is not there (flag true, no file).
            static uint32_t lastAcctMs = 0;
            if (millis() - lastAcctMs >= 10000) {
                lastAcctMs = millis();
                Serial.printf("💾 [%lus] SD rows gps %lu imu %lu can %lu sat %lu nmea %lu | sat pushed %lu refused %lu gated %lu queued %u | "
                              "on g%d i%d c%d s%d n%d | open g%d i%d c%d s%d n%d raw%d | faults %lu%s\n",
                              (unsigned long)(millis() / 1000UL),
                              (unsigned long)rowsGps, (unsigned long)rowsImu, (unsigned long)rowsCan, (unsigned long)rowsSat, (unsigned long)rowsNmea,
                              (unsigned long)satPushed, (unsigned long)satRefused, (unsigned long)satGated,
                              (unsigned)(satQueue ? uxQueueMessagesWaiting(satQueue) : 0),
                              (int)logGps, (int)logImu, (int)logCan, (int)logSat, (int)logNmea,
                              (int)(bool)gpsFile, (int)(bool)imuFile, (int)(bool)canFile, (int)(bool)satFile, (int)(bool)nmeaFile, (int)(bool)canRawFile,
                              (unsigned long)sdWriteFaults, recovering ? " | recovering" : "");
            }
            // Serial mirror: publish the session stamp, start a new mirror file when the
            // session changes, and move buffered serial text to the card. Without this call
            // the mirror's ring is never emptied and no debug file is ever created.
            if (strcmp(pubStamp, sessionStamp) != 0) {
#if defined(DEBUG_SERIAL_TO_SD) && DEBUG_SERIAL_TO_SD
                if (pubStamp[0] && sessionStamp[0]) DebugLog::closeFile();
#endif
                strncpy(pubStamp, sessionStamp, sizeof(pubStamp) - 1);
                pubStamp[sizeof(pubStamp) - 1] = '\0';
            }
#if defined(DEBUG_SERIAL_TO_SD) && DEBUG_SERIAL_TO_SD
            DebugLog::drainToFile();
#endif
            // Live on-disk sizes for the display + web listing. .size() is an fstat
            // (no media I/O) — compute OUTSIDE the activeMux critical section (never
            // call into the VFS with interrupts disabled), then publish. Rate-limited
            // to ~4 Hz so heavy logging isn't taxed by per-loop fstats.
            static uint32_t lastSizePub = 0;
            if (millis() - lastSizePub >= 250) {
                lastSizePub = millis();
                uint32_t gz = gpsFile ? (uint32_t)gpsFile.size() : 0;
                uint32_t cz = canFile ? (uint32_t)canFile.size() : 0;
                uint32_t iz = imuFile ? (uint32_t)imuFile.size() : 0;
                uint32_t sz4 = satFile ? (uint32_t)satFile.size() : 0;
                uint32_t nz  = nmeaFile ? (uint32_t)nmeaFile.size() : 0;
                taskENTER_CRITICAL(&activeMux);
                activeBytesByType[0] = gz; activeBytesByType[1] = cz;
                activeBytesByType[2] = iz; activeBytesByType[3] = sz4;
                activeBytesByType[4] = nz;
                taskEXIT_CRITICAL(&activeMux);
            }
        }

        crumb::mark(crumb::SLOT_SDLOG, crumb::S_CAPACITY);
        // ── Refresh SD capacity cache (off the async_tcp task) ─────────────
        // First call after mount can be slow (f_getfree walks the FAT); FatFs
        // caches the free count afterward so later calls are cheap. Done here, on
        // the logger task, so the web /sd handler never touches the SD bus.
        // Skipped while a download is in flight (downloadActive != 0): this is a
        // read against the FAT that shares the SD bus with the download's own
        // reads, and the 30 s cadence means skipping a pass costs nothing but a
        // briefly stale free-space figure — it runs on the very next pass after
        // the last transfer ends.
        if ((capTotal == 0 || capRefreshReq || millis() - capLastMs > 30000) && downloadActive == 0) {
            if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
                capRefreshReq = false;
                capTotal  = SD_MMC.totalBytes();
                capUsed   = SD_MMC.usedBytes();
                capLastMs = millis();
                xSemaphoreGive(sdMutex);
            }
        }

        crumb::mark(crumb::SLOT_SDLOG, crumb::S_DIRSCAN);
        // ── Service the one boot-time directory scan (off the async_tcp task) ─
        // dirScanReq is set exactly once, at boot (see sdlog_init), to discover
        // files left over from past power cycles — see sd_log.h for why nothing
        // ever sets it again after that: every file created THIS session is
        // added to the snapshot incrementally by openFile()/closeFile(), so
        // there is never a reason to re-walk the root later.
        //
        // Still time-sliced, DIR_SCAN_SLICE_FILES files per pass, even though
        // it's boot-only: a card can already have hundreds of files at boot
        // (e.g. after a day of thermal-restart cycling), and walking the whole
        // root in one uninterrupted mutex hold is a real, confirmed field
        // failure — the GPS/CAN/IMU/SAT writes below only wait 50ms for sdMutex
        // before silently giving up on that row (see their own comments), and a
        // long-enough scan starves every one of them for its whole duration.
        // Releasing sdMutex between slices lets this loop reach the write
        // blocks every pass instead. Held pending while a download is in
        // flight, same as before — the partial scan just waits.
        if (!dirScanActive && dirScanReq && downloadActive == 0) {
            dirScanReq = false;
            dirScanCnt = 0; dirScanTrunc = false;
            if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                dirScanDir = SD_MMC.open("/");
                xSemaphoreGive(sdMutex);
                dirScanActive = (bool)dirScanDir;
            }
        }
        if (dirScanActive && downloadActive == 0) {
            bool finished = false;
            if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                for (int i = 0; i < DIR_SCAN_SLICE_FILES; i++) {
                    File f = dirScanDir.openNextFile();
                    if (!f) { finished = true; break; }
                    String  nm2 = String(f.name());
                    uint32_t sz = (uint32_t)f.size();
                    f.close();
                    const char* bare = nm2.startsWith("/") ? nm2.c_str() + 1 : nm2.c_str();
                    if (!sdlog_isLogFileName(bare) && !sdlog_isArchiveName(bare)) continue;   // logs, serial mirror, stored archives
                    const char* key  = nameSortKey(bare);
                    // Descending insertion by timestamp key → newest first.
                    int pos = dirScanCnt;
                    while (pos > 0 && strcmp(nameSortKey(dirScanTmp[pos - 1].name), key) < 0) pos--;
                    if (pos >= dirSnapCap) { dirScanTrunc = true; continue; }
                    if (dirScanCnt >= dirSnapCap) {          // table full: drop oldest
                        dirScanTrunc = true;
                        for (int j = dirSnapCap - 1; j > pos; j--) dirScanTmp[j] = dirScanTmp[j - 1];
                    } else {
                        for (int j = dirScanCnt; j > pos; j--) dirScanTmp[j] = dirScanTmp[j - 1];
                        dirScanCnt++;
                    }
                    strncpy(dirScanTmp[pos].name, bare, LOG_NAME_MAX - 1); dirScanTmp[pos].name[LOG_NAME_MAX - 1] = '\0';
                    dirScanTmp[pos].size = sz;
                }
                if (finished) dirScanDir.close();
                xSemaphoreGive(sdMutex);
            }
            if (finished) {
                dirScanActive = false;
                // Merge, never overwrite: on a card with hundreds of files this
                // scan can span many passes (many seconds), and a channel can
                // open mid-walk — snapshotNoteOpen() writes that entry into
                // dirSnap directly, live, the moment it happens. Blindly
                // replacing dirSnap with the scan's own results here would
                // silently erase that entry the instant the scan finishes (the
                // scan only knows what existed on the card when IT looked, not
                // what opened afterward). So: keep everything already in
                // dirSnap (newest — this session's own opens), and only add
                // scan results for names not already present (older — from
                // past power cycles, which is the whole reason this scan runs).
                if (dirSnapLock && xSemaphoreTake(dirSnapLock, pdMS_TO_TICKS(100)) == pdTRUE) {
                    for (int i = 0; i < dirScanCnt && dirSnapCount < dirSnapCap; i++) {
                        bool exists = false;
                        for (int j = 0; j < dirSnapCount; j++)
                            if (strcmp(dirSnap[j].name, dirScanTmp[i].name) == 0) { exists = true; break; }
                        if (!exists) dirSnap[dirSnapCount++] = dirScanTmp[i];
                    }
                    if (dirScanTrunc || dirSnapCount >= dirSnapCap) dirSnapTrunc = true;
                    dirSnapGen++;
                    xSemaphoreGive(dirSnapLock);
                }
            }
        }

        // ── Service a "Download all" export request (off the async_tcp task) ─
        crumb::mark(crumb::SLOT_SDLOG, crumb::S_EXPORT);
        if (exportReq) { exportReq = false; buildExport(); }

        crumb::mark(crumb::SLOT_SDLOG, crumb::S_RAWCAN);
        // ── CAN raw sniffer log (separate file, only while sniffer is ON) ──
        // Independent of GPS *fix* — you sniff on the bench with no lock. It runs alongside
        // the normal gps/can/imu/sat channels: the branches and queues are disjoint, so
        // concurrent capture is safe against corruption, and the sniffer's own drop counters
        // show any shortfall in SD write throughput under heavy bus load.
        //
        // Filename policy:
        //   • UTC known   → canraw_YYYYMMDD_HHMMSS.csv (same stamp as the other
        //                   logs; uniqueLogName() guards collisions).
        //   • UTC unknown → canraw_NNNN.csv, next free 4-digit ordinal on the card
        //                   (clean + sortable; replaces the old cryptic millis name).
        // If the clock locks WHILE this file is open, a #UTC_SYNC marker row is
        // appended (below) so postprocessing can map the early raw-millis rows to
        // absolute UTC. Row format is unix_ms,id_hex,dlc,data_hex; the decoded
        // can_*.csv is a completely separate file and is untouched.
        {
            const bool sniffRequested = can_getSniffer();
            bool sniffOn = sniffRequested && !sRawHold;
            static uint32_t canRawRetryAt = 0;   // after a failed write: reopen no sooner than this
            if (sniffOn && !canRawFile && (canRawRetryAt == 0 || (int32_t)(millis() - canRawRetryAt) >= 0)) {
                char rn[40];
                if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
                    if (sessionStamp[0])
                        uniqueLogName(rn, sizeof(rn), "canraw", sessionStamp);
                    else
                        nextCanRawName(rn, sizeof(rn));   // scans root under sdMutex
                    // Staged rows go out in whole-sector blocks straight from the staging
                    // buffer, so stdio only needs a one-sector buffer; if the staging buffer
                    // cannot be had, rows go through the 16 KB stdio buffer as before.
                    const bool staged = rawStageAlloc();
                    bool ok = openFile(canRawFile, rn, "unix_ms,id,dlc,data", staged ? 512 : kBulkBufBytes);
                    if (ok && staged) rawStageBegin(canRawFile);
                    if (!ok) rawStageFree();
                    xSemaphoreGive(sdMutex);
                    if (ok) {
                        snapshotNoteOpen(rn);
                        canRawRetryAt = 0;                // backoff served
                        canRawSyncWritten = false;        // (re)arm UTC marker for this file
                        // Register as an OPEN file so a web delete (single or bulk)
                        // can't SD_MMC.remove() it out from under us — removing an
                        // open file corrupts FatFs and resets the unit.
                        const char* bare = (rn[0] == '/') ? rn + 1 : rn;
                        taskENTER_CRITICAL(&activeMux);
                        strncpy(canRawName, bare, sizeof(canRawName) - 1);
                        canRawName[sizeof(canRawName) - 1] = '\0';
                        taskEXIT_CRITICAL(&activeMux);
                        Serial.printf("🔎 SD CANraw: logging ALL frames → %s (%s)\n", rn,
                                      sRawStage ? "aligned block writes" : "⚠️ staging buffer unavailable, slow write path");
                    }
                }
            } else if (!sniffOn && canRawFile) {
                rawStageFinish(canRawFile, sdMutex);
                closeFile(canRawFile, sdMutex);
                taskENTER_CRITICAL(&activeMux);
                canRawName[0] = '\0';                      // closed — now deletable
                taskEXIT_CRITICAL(&activeMux);
                Serial.println(sniffRequested ? "🔎 SD CANraw: closed (held for an NTRIP scan; frames queue and resume in a new file)" : "🔎 SD CANraw: closed (sniffer off)");
            }

            // ── UTC sync marker ──────────────────────────────────────────────
            // First instant we have wall-clock while a canraw file is open, drop a
            // line mapping the free-running millis to UTC. Rows logged before this
            // point carry raw millis in column 1 (gUnixMsOffset was 0); the offset
            // is constant, so this single anchor converts them all in post.
            if (canRawFile && gClockSet && !canRawSyncWritten) {
                if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    uint32_t mnow = millis();
                    char sync[88];
                    snprintf(sync, sizeof(sync),
                             "#UTC_SYNC,millis=%lu,unix_ms=%llu,offset_ms=%lld",
                             (unsigned long)mnow,
                             (unsigned long long)toUnixMs(mnow),
                             (long long)gUnixMsOffset);
                    if (sRawStage) { rawStageAppend(sync, strlen(sync)); rawStageAppend("\r\n", 2); }
                    else { canRawFile.println(sync); canRawFile.flush(); }
                    xSemaphoreGive(sdMutex);
                    canRawSyncWritten = true;
                    Serial.printf("🔎 SD CANraw: UTC sync marker → %s\n", sync);
                }
            }
            // Drain the raw queue in a bounded batch so a burst can't monopolise
            // the task. Anything still queued is picked up next pass. Paused
            // under thermal threshold 2b (thermal.h, ≥105°C) — the sniffer
            // stays on and its live web snapshot (can_bus.cpp) is unaffected;
            // only this SD drain pauses, so frames back up in canRawQueue and
            // age out via the existing drop counter exactly as they would on
            // any other queue-full condition, rather than being force-closed.
            if (canRawFile && canRawQueue && !thermal_canSniffInhibit()) {
                CanRawRecord rr; int drained = 0;
                static const char HEXD[] = "0123456789ABCDEF";
                bool rawErr = false;
                while (drained < 512) {
                    // A full staging block goes to the card before the next row is taken. If the
                    // card is busy past the mutex wait, leave the rest in the queue (16384 deep)
                    // for the next pass rather than overrun the stage.
                    if (sRawStage && sRawLen >= kRawStageWrite) {
                        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(50)) != pdTRUE) break;
                        rawErr = !rawStageWrite(canRawFile, false, false);
                        xSemaphoreGive(sdMutex);
                        if (rawErr) break;
                    }
                    if (xQueueReceive(canRawQueue, &rr, 0) != pdTRUE) break;
                    char hx[17];                         // up to 8 bytes -> 16 hex chars
                    int p = 0;
                    for (int b = 0; b < rr.dlc && b < 8; b++) {
                        hx[p++] = HEXD[rr.data[b] >> 4];
                        hx[p++] = HEXD[rr.data[b] & 0x0F];
                    }
                    hx[p] = '\0';
                    char row[48];
                    const int rowLen = snprintf(row, sizeof(row), "%llu,%03X,%u,%s",
                             (unsigned long long)toUnixMs(rr.ms), rr.id, rr.dlc, hx);
                    if (sRawStage) {
                        rawStageAppend(row, (size_t)rowLen); rawStageAppend("\r\n", 2);
                    } else if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                        if (canRawFile.println(row) == 0) rawErr = true;
                        else if (millis() - flushCanRaw >= 1000) { canRawFile.flush(); flushCanRaw = millis(); }
                        xSemaphoreGive(sdMutex);
                    }
                    if (rawErr) break;
                    drained++;
                }
                // One-second tick: write what is staged (a multiple of 4 bytes) and sync, so a
                // power cut loses about a second at most. Runs on idle passes too.
                if (!rawErr && sRawStage && millis() - flushCanRaw >= 1000 &&
                    xSemaphoreTake(sdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    if (rawStageWrite(canRawFile, true, false)) { canRawFile.flush(); flushCanRaw = millis(); }
                    else rawErr = true;
                    xSemaphoreGive(sdMutex);
                }
                if (rawErr) {
                    // Same sticky FatFs error as the channel logs (see
                    // failChannel): close now, reopen as canraw_<stamp>_2.csv.
                    rawStageFree();
                    closeFile(canRawFile, sdMutex);
                    taskENTER_CRITICAL(&activeMux);
                    canRawName[0] = '\0';
                    taskEXIT_CRITICAL(&activeMux);
                    canRawRetryAt = (millis() + 500) | 1;   // never 0 (0 = no backoff)
                    Serial.println("⚠️  SD CANraw: write failed — file closed, reopening");
                }
            }
        }

        crumb::mark(crumb::SLOT_SDLOG, crumb::S_IMU);
        // ── Drain IMU queue (non-blocking, high priority) ──────────────────
        while (imuFile && logImu && xQueueReceive(imuQueue, &irec, 0) == pdTRUE) {
            // Full sensor resolution. One count is 8/32768 = 0.000244 g and
            // 512/32768 = 0.0156 deg/s; logging to 0.0001 g and 0.01 deg/s is
            // finer than one count, so every distinct reading stays distinct
            // (a coarser format merges neighbouring counts and discards real
            // signal — the gyro noise floor is about one 0.1 deg/s step).
            // Longest row: 13-digit ms + 3 x "-8.0000" + 3 x "-512.00" = 61 chars.
            char row[80];
            snprintf(row, sizeof(row), "%llu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f",
                     (unsigned long long)toUnixMs(irec.ms),
                     irec.ax, irec.ay, irec.az, irec.gx, irec.gy, irec.gz);
            if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (!printlnCounted(imuFile, row, rowsImu)) imuErr = true;
                else if (millis() - flushImu >= 1000) { imuFile.flush(); flushImu = millis(); }
                xSemaphoreGive(sdMutex);
            }
            if (imuErr) { imuErr = false; imuHdr = false; failChannel(imuFile, "IMU"); break; }
        }

        crumb::mark(crumb::SLOT_SDLOG, crumb::S_SAT);
        // ── Drain SAT queue (non-blocking) ─────────────────────────────────
        while (satFile && logSat && xQueueReceive(satQueue, &srec, 0) == pdTRUE) {
            char row[128];
            snprintf(row, sizeof(row), "%llu,%s", (unsigned long long)toUnixMs(srec.ms), srec.sentence);
            if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (!printlnCounted(satFile, row, rowsSat)) satErr = true;
                else if (millis() - flushSat >= 1000) { satFile.flush(); flushSat = millis(); }
                xSemaphoreGive(sdMutex);
            }
            if (satErr) { satErr = false; satHdr = false; failChannel(satFile, "SAT"); break; }
        }

        crumb::mark(crumb::SLOT_SDLOG, crumb::S_NMEA);
        // ── Drain raw receiver capture (debug channel, non-blocking) ───────
        while (nmeaFile && logNmea && xQueueReceive(nmeaQueue, &nrec, 0) == pdTRUE) {
            char row[128];
            snprintf(row, sizeof(row), "%llu,%s", (unsigned long long)toUnixMs(nrec.ms), nrec.sentence);
            if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (!printlnCounted(nmeaFile, row, rowsNmea)) nmeaErr = true;
                else if (millis() - flushNmea >= 1000) { nmeaFile.flush(); flushNmea = millis(); }
                xSemaphoreGive(sdMutex);
            }
            if (nmeaErr) { nmeaErr = false; failChannel(nmeaFile, "NMEA"); break; }
        }

        crumb::mark(crumb::SLOT_SDLOG, crumb::S_CAN);
        // ── Decoded CAN channel ────────────────────────────────────────────
        // Rows come from canQueue (sdlog_push_can), not from the GPS epoch record: 50 Hz while the
        // car is active, once a second after a minute of stillness. Drained every pass so the
        // GPS-queue wait below does not set this channel's cadence. The row's utc_time is derived
        // from its unix_ms. Thermal threshold 3 (thermal.h) pauses it at >=112 C.
        // While the bus reads silent (canHz==0, vehicle off) the row rate drops to once every
        // CAN_IDLE_LOG_MS.
        while (canQueue && xQueueReceive(canQueue, &crec, 0) == pdTRUE) {
            char utc[24] = "--:--:--.---";
            if (gClockSet) fmtUtcMs(toUnixMs(crec.ms), utc, sizeof(utc));
            static uint32_t lastCanWriteMs = 0;
            bool canIdle = crec.canHz <= 0.0f;
            bool writeCanNow = !canIdle || lastCanWriteMs == 0 ||
                               (millis() - lastCanWriteMs) >= CAN_IDLE_LOG_MS;
            if (canFile && logCan && writeCanNow && !thermal_satCanImuInhibit()) {
                lastCanWriteMs = millis();
                char rpm[10],tps[8],ped[8],tqt[8],tqa[8],veh[8],ect[8],oilT[8],oilP[8],brk[8];
                // Speeds, steering and CAN yaw keep the car's own resolution (987
                // vehicle speed 0.01 km/h -> 3 decimals in mph; wheel speeds and
                // steering -> 2; 718 yaw 0.01 deg/s -> 2). st holds "-1474.52".
                char brSw[4],brSw2[4],wlf[8],wrf[8],wlr[8],wrr[8],st[10],sr[8];
                char fuel[8],kick[4],baro[8],fuelT[8],engT[8],gear[4],pdkRaw[8],pdkSel[8];
                char iat[8],map[8],maf[10],latG[10],longG[10],canYaw[10],clutch[8];
                char outside[8],odo[12],drive[5],psm[5],pdkState[5],pdkFault[5];
                char tpLF[8],tpRF[8],tpLR[8],tpRR[8],ttLF[8],ttRF[8],ttLR[8],ttRR[8];
                fmtF(rpm,   sizeof(rpm),   crec.rpm,          0);
                fmtF(tps,   sizeof(tps),   crec.tpsActual,    1);
                fmtF(ped,   sizeof(ped),   crec.pedalReq,     1);
                fmtF(tqt,   sizeof(tqt),   crec.targetTorque, 1);
                fmtF(tqa,   sizeof(tqa),   crec.actualTorque, 1);
                fmtF(veh,   sizeof(veh),   kphToMph(crec.vehicleSpeedKph), 3);
                fmtF(ect,   sizeof(ect),   cToF(crec.coolantC), 1);
                fmtF(oilT,  sizeof(oilT),  cToF(crec.oilC), 1);
                fmtF(oilP,  sizeof(oilP),  barToPsi(crec.oilBar), 2);
                fmtF(brk,   sizeof(brk),   barToPsi(crec.brakeBar), 1);
                fmtF(brSw,  sizeof(brSw),  crec.brakeSwitch,  0);
                fmtF(brSw2, sizeof(brSw2), crec.brakeSwitch2, 0);
                fmtF(wlf,   sizeof(wlf),   kphToMph(crec.wsFL), 2);
                fmtF(wrf,   sizeof(wrf),   kphToMph(crec.wsFR), 2);
                fmtF(wlr,   sizeof(wlr),   kphToMph(crec.wsRL), 2);
                fmtF(wrr,   sizeof(wrr),   kphToMph(crec.wsRR), 2);
                fmtF(st,    sizeof(st),    crec.steerDeg,     2);
                fmtF(sr,    sizeof(sr),    crec.steerRate,    1);
                fmtF(fuel,  sizeof(fuel),  crec.fuelLevel,    1);
                fmtF(kick,  sizeof(kick),  crec.kickdown,     0);
                fmtF(baro,  sizeof(baro),  kpaToPsi(crec.atmosphericKpa), 2);
                fmtF(fuelT, sizeof(fuelT), cToF(crec.fuelTempC), 1);
                fmtF(engT,  sizeof(engT),  cToF(crec.engineTempC), 1);
                if (crec.is718 && !crec.gearValid718) gear[0] = '\0';
                else snprintf(gear, sizeof(gear), "%u", (unsigned)crec.gear);
                fmtF(pdkRaw, sizeof(pdkRaw), crec.pdkGearRaw, 0);
                fmtF(pdkSel, sizeof(pdkSel), crec.pdkSelectorRaw, 0);

                fmtF(iat,      sizeof(iat),      cToF(crec.intakeAirTempC), 1);
                fmtF(map,      sizeof(map),      barToPsi(crec.manifoldAbsPressureBar), 2);
                fmtF(maf,      sizeof(maf),      crec.massAirFlowGps, 1);
                fmtF(latG,     sizeof(latG),     crec.canLateralAccelG, 3);
                fmtF(longG,    sizeof(longG),    crec.canLongitudinalAccelG, 3);
                fmtF(canYaw,   sizeof(canYaw),   crec.canYawRateDegPerSec, 2);
                fmtF(clutch,   sizeof(clutch),   crec.clutchPositionPercent, 1);
                fmtF(outside,  sizeof(outside),  cToF(crec.outsideTempC), 1);
                fmtF(odo,      sizeof(odo),      isnan(crec.odometerKm) ? NAN : crec.odometerKm * 0.621371f, 1);
                fmtF(drive,    sizeof(drive),    crec.driveMode, 0);
                fmtF(psm,      sizeof(psm),      crec.psmMode, 0);
                fmtF(pdkState, sizeof(pdkState), crec.pdkState, 0);
                fmtF(pdkFault, sizeof(pdkFault), crec.pdkNoDriveOrFault, 0);
                fmtF(tpLF,     sizeof(tpLF),     crec.tpmsFrontLeftPsi, 1);
                fmtF(tpRF,     sizeof(tpRF),     crec.tpmsFrontRightPsi, 1);
                fmtF(tpLR,     sizeof(tpLR),     crec.tpmsRearLeftPsi, 1);
                fmtF(tpRR,     sizeof(tpRR),     crec.tpmsRearRightPsi, 1);
                fmtF(ttLF,     sizeof(ttLF),     cToF(crec.tpmsTempFrontLeftC), 1);
                fmtF(ttRF,     sizeof(ttRF),     cToF(crec.tpmsTempFrontRightC), 1);
                fmtF(ttLR,     sizeof(ttLR),     cToF(crec.tpmsTempRearLeftC), 1);
                fmtF(ttRR,     sizeof(ttRR),     cToF(crec.tpmsTempRearRightC), 1);

                char row[768];
                snprintf(row, sizeof(row),
                    "%llu,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,"
                    "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
                    (unsigned long long)toUnixMs(crec.ms), utc,
                    rpm, tps, ped, tqt, tqa, veh, ect, oilT, oilP, brk, brSw, brSw2,
                    wlf, wrf, wlr, wrr, st, sr, fuel, kick, baro, fuelT, engT, gear, pdkRaw, pdkSel,
                    iat, map, maf, latG, longG, canYaw, clutch, outside, odo, drive, psm,
                    pdkState, pdkFault, tpLF, tpRF, tpLR, tpRR, ttLF, ttRF, ttLR, ttRR);
                if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    if (!printlnCounted(canFile, row, rowsCan)) {
                        canErr = true; Serial.println("❌ SD CAN: write failed");
                    } else if (millis() - flushCan >= 1000) { canFile.flush(); flushCan = millis(); }
                    xSemaphoreGive(sdMutex);
                }
                if (canErr) {
                    canHdr = false; canErr = false;
                    failChannel(canFile, "CAN");
                }
            }
        }

        crumb::mark(crumb::SLOT_SDLOG, crumb::S_WAIT);
        // ── Wait for GPS+CAN record ────────────────────────────────────────
        if (xQueueReceive(logQueue, &rec, pdMS_TO_TICKS(100)) != pdTRUE) {
            if (!logGps && gpsFile) { closeFile(gpsFile, sdMutex); gpsHdr=false; activeLogName[0]='\0'; }
            if (!logImu && imuFile) { closeFile(imuFile, sdMutex); imuHdr=false; }
            if (!logCan && canFile) { closeFile(canFile, sdMutex); canHdr=false; }
            if (!logSat && satFile) { closeFile(satFile, sdMutex); satHdr=false; }
            if (!logNmea && nmeaFile) closeFile(nmeaFile, sdMutex);
            continue;
        }

        crumb::mark(crumb::SLOT_SDLOG, crumb::S_OPEN);
        // ── Open files on first valid GPS fix ─────────────────────────────
        const bool cardFull = capTotal > 0 && (capTotal - capUsed) < SD_MIN_FREE_BYTES;
        if (recovering && cardFull && !fullNoted) {
            fullNoted = true;
            Serial.println("❌ SD: card full — logging stopped until space is freed");
        }
        bool canOpen = rec.valid && rec.year >= 2024 && rec.year <= 2050
                    && !(recovering && cardFull)
                    && (millis() - lastOpen >= 5000 || lastOpen == 0 ||
                        (recovering && (int32_t)(millis() - recoverAt) >= 0));
        // New logging session: if nothing is currently open, drop the old stamp
        // so the next files use a FRESH (current) timestamp rather than reopening
        // — and truncating — the previous session's files. (Per-channel reopens
        // while other channels stay open keep the shared stamp; uniqueLogName()
        // then guarantees those still land in a new file, not an overwrite.)
        if (canOpen && !recovering && !gpsFile && !canFile && !imuFile && !satFile && !nmeaFile)
            sessionStamp[0] = '\0';
        if (canOpen && sessionStamp[0] == '\0') {
            // The stamp is GPS UTC — the device has no local-time notion at all.
            // Saying so in the filename keeps it unambiguous once a file has been
            // pulled off the card and sits next to an archive named for local time.
            snprintf(sessionStamp, sizeof(sessionStamp), "%04u%02u%02u_%02u%02u%02u_UTC",
                     rec.year, rec.mon, rec.day, rec.hr, rec.min, rec.sec);
        }

        // Set the system clock from the GPS fix the first time we have one. The
        // ESP32 has no battery-backed RTC, so without this it sits at the 1970
        // epoch and every file is stamped ~1979/1980 in the FAT directory. GPS
        // time is UTC; TZ was set to UTC0 in sdlog_init() so mktime() yields the
        // correct UTC epoch. Done once per boot, before any file is opened.
        if (!gClockSet && rec.valid && rec.year >= 2024 && rec.year <= 2050) {
            struct tm t = {};
            t.tm_year = rec.year - 1900; t.tm_mon = rec.mon - 1; t.tm_mday = rec.day;
            t.tm_hour = rec.hr; t.tm_min = rec.min; t.tm_sec = rec.sec;
            time_t epoch = mktime(&t);
            if (epoch > 1700000000) {          // sanity: after 2023-11
                struct timeval tv; tv.tv_sec = epoch; tv.tv_usec = (suseconds_t)rec.msec * 1000;
                settimeofday(&tv, nullptr);
                gUnixMsOffset = (int64_t)epoch * 1000 + rec.msec - (int64_t)rec.ms;
                gAnchorMs     = rec.ms;
                gAnchorUnixMs = (int64_t)epoch * 1000 + rec.msec;
                gClockSet = true;
                Serial.printf("🕐 System clock set from GPS: %04u-%02u-%02u %02u:%02u:%02u UTC (epoch %lu)\n",
                              rec.year, rec.mon, rec.day, rec.hr, rec.min, rec.sec,
                              (unsigned long)epoch);
            }
        }
        
        if (canOpen && sessionStamp[0] != '\0') {
            char name[40];
            if (logGps && !gpsFile) {
                if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                    uniqueLogName(name, sizeof(name), "gps", sessionStamp);
                    bool ok = openFile(gpsFile, name,
                        "unix_ms,utc_time,"
                        "lat,lon,alt_msl_m,geoid_sep_m,speed_mph,heading_deg,"
                        "h_acc_m,v_acc_m,rtk_type,sta_id,num_sats,epe_valid,"
                        "ntrip,mountpoint,ntrip_dist_km,carrier,vrs,rtcm_bytes,"
                        "esp_temp_f,spd_age_ms,pdop,hdop,vdop,gsa_sats,"
                        // diff_age: correction age (s) from GGA field 13; -1 = none in
                        // use. Answers "was RTK starving" per-row without a serial log.
                        // ble: link state (0 none / 1 connected / 2 subscribed).
                        // ble_dlv / ble_sup: cumulative samples delivered over BLE vs
                        // produced-but-never-sent — diffing rows gives the true wire
                        // rate and loss for any time window, from the log alone.
                        "diff_age,ble,ble_dlv,ble_sup,"
                        // vel_src: 0 receiver, 1 RTK track (receiver SOG/COG empty),
                        // 2 RTK track (receiver repeated its last value), 3 none.
                        // spd_age_ms is the age of the RECEIVER's own velocity.
                        "vel_src,trk_spd_mph,trk_hdg_deg", kBulkBufBytes);
                    xSemaphoreGive(sdMutex);
                    if (ok) {
                        snapshotNoteOpen(name);
                        logFileCount++; gpsHdr = true; flushGps = 0; gpsErr = false;
                        setActive(0, gpsFile.name());   // protected from web delete at once
                        strncpy(activeLogName, name, sizeof(activeLogName) - 1);
                        Serial.printf("📝 SD GPS: %s\n", name);
                        lastOpen = millis();
                    }
                }
            }
            if (logCan && !canFile) {
                if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                    uniqueLogName(name, sizeof(name), "can", sessionStamp);
                    // CSV column names = the SAME unified channel labels used by
                    // the BLE (ble_racecapture.cpp) and WiFi (racecapture.cpp)
                    // streams, so a CAN channel has ONE name everywhere. Values
                    // are written in ENGLISH units below (mph/°F/psi) — matching
                    // the stream meta. unix_ms + utc_time are log-only leading
                    // columns (not telemetry channels).
                    bool ok = openFile(canFile, name,
                        "unix_ms,utc_time,"
                        "RPM,TPS,PedalPos,TgtTorque,ActTorque,"
                        "VehicleSpd,CoolantTmp,OilTemp,OilPress,Brake,"
                        "BrakeSw,BrakeSw2,"
                        "WheelSpdLF,WheelSpdRF,WheelSpdLR,WheelSpdRR,"
                        "Steering,SteerRate,FuelLevel,Kickdown,BaroPress,"
                        "FuelTemp,EngineTemp,Gear,PdkGearRaw,PdkSelRaw,"
                        "IAT,MAP,MAF,CANLatG,CANLongG,CANYaw,ClutchPos,"
                        "OutsideTmp,Odometer,DriveMode,PSMMode,PDKState,PDKFault,"
                        "TPMSLF,TPMSRF,TPMSLR,TPMSRR,"
                        "TPMSTmpLF,TPMSTmpRF,TPMSTmpLR,TPMSTmpRR", kBulkBufBytes);
                    xSemaphoreGive(sdMutex);
                    if (ok) { snapshotNoteOpen(name); setActive(1, canFile.name());
                              logFileCount++; canHdr=true; flushCan=0; canErr=false;
                              Serial.printf("📝 SD CAN: %s\n", name); }
                }
            }
            if (logImu && !imuFile) {
                if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                    uniqueLogName(name, sizeof(name), "imu", sessionStamp);
                    bool ok = openFile(imuFile, name,
                        "unix_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps", kBulkBufBytes);
                    xSemaphoreGive(sdMutex);
                    if (ok) { snapshotNoteOpen(name); setActive(2, imuFile.name());
                              logFileCount++; imuHdr=true; flushImu=0;
                              Serial.printf("📝 SD IMU: %s\n", name); }
                }
            }
            if (logSat && !satFile) {
                if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                    uniqueLogName(name, sizeof(name), "sat", sessionStamp);
                    bool ok = openFile(satFile, name, "unix_ms,nmea_sentence", kBulkBufBytes);
                    xSemaphoreGive(sdMutex);
                    if (ok) { snapshotNoteOpen(name); setActive(3, satFile.name());
                              logFileCount++; satHdr=true; flushSat=0;
                              Serial.printf("📝 SD SAT: %s\n", name); }
                }
            }
            if (logNmea && !nmeaFile) {
                if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                    uniqueLogName(name, sizeof(name), "nmea", sessionStamp);
                    bool ok = openFile(nmeaFile, name, "unix_ms,nmea_sentence", kBulkBufBytes);
                    xSemaphoreGive(sdMutex);
                    if (ok) { snapshotNoteOpen(name); setActive(4, nmeaFile.name());
                              logFileCount++; flushNmea=0;
                              Serial.printf("📝 SD NMEA (raw receiver capture): %s\n", name); }
                }
            }
        }

        // Recovery is done once every enabled channel has a file again; until
        // then, retry on the short cadence (the card may still be failing).
        if (recovering && canOpen) {
            const bool allOpen = (!logGps || gpsFile) && (!logCan || canFile) &&
                                 (!logImu || imuFile) && (!logSat || satFile) &&
                                 (!logNmea || nmeaFile);
            if (allOpen) {
                recovering = false;
                Serial.printf("✅ SD: all channels recording again (session %s)\n", sessionStamp);
            } else {
                recoverAt = millis() + recoverDelay;
            }
        }

        // ── Close files whose channel was disabled ─────────────────────────
        if (!logGps && gpsFile) { closeFile(gpsFile, sdMutex); gpsHdr=false; activeLogName[0]='\0'; }
        if (!logCan && canFile) { closeFile(canFile, sdMutex); canHdr=false; }
        if (!logImu && imuFile) { closeFile(imuFile, sdMutex); imuHdr=false; }
        if (!logSat && satFile) { closeFile(satFile, sdMutex); satHdr=false; }
        if (!logNmea && nmeaFile) closeFile(nmeaFile, sdMutex);

        // ── Build UTC string ───────────────────────────────────────────────
        // One pass writes every GPS epoch that has queued, not just the one it woke for. Each pass
        // also drains the raw, IMU, CAN, NMEA and SAT queues first, and with the card busy that
        // can take well over the 50 ms epoch period; one row per pass would then cap the GPS log
        // at the pass rate and overflow the epoch queue while every other channel stayed lossless.
        // The cap bounds one pass to the queue depth. Files are already open for the first record,
        // so the later ones only need the row write.
        bool haveRec = true;
        for (int gpsRows = 0; haveRec; ) {
        char utc[24] = "--:--:--.---";
        if (rec.valid && rec.year >= 2024 && rec.year <= 2050)
            snprintf(utc, sizeof(utc), "%04u-%02u-%02uT%02u:%02u:%02u.%03u",
                     rec.year, rec.mon, rec.day, rec.hr, rec.min, rec.sec, rec.msec);

        crumb::mark(crumb::SLOT_SDLOG, crumb::S_GPS);
        // ── Write GPS record ───────────────────────────────────────────────
        // Thermal threshold 4 (thermal.h) caps this row's SD write at 1 Hz —
        // GPS is the last channel still logging by that point, so it's the
        // last thing to slow, and only the SD write: the epoch/queue cadence
        // above is untouched, and BLE/SoloStorm keeps its own full-rate feed.
        static uint32_t lastGpsWriteMs = 0;
        bool writeGpsNow = !thermal_gpsReduce1Hz() || lastGpsWriteMs == 0 ||
                           (millis() - lastGpsWriteMs) >= GPS_THERMAL_LOG_MS;
        if (gpsFile && logGps && writeGpsNow) {
            lastGpsWriteMs = millis();
            char mpt[21]; strncpy(mpt, rec.mountpoint, 20); mpt[20]='\0';
            for (char* p = mpt; *p; p++) if (*p == ',') *p = '_';
            // Device temps → °F at the output (types.h: convert at the write site).
            // fmtF emits "" for NAN, so a cold-start/sensor-fault cell is blank.
            char espT[8];
            fmtF(espT, sizeof(espT), cToF(rec.espTempC), 1);
            // Speed/heading are blank when gnss.cpp had no current value (neither a
            // receiver velocity nor a usable RTK track) — never a held one.
            char spdS[12], hdgS[12], tSpdS[12], tHdgS[12];
            fmtF(spdS,  sizeof(spdS),  rec.speedMph,      2);
            fmtF(hdgS,  sizeof(hdgS),  rec.headingDeg,    1);
            fmtF(tSpdS, sizeof(tSpdS), rec.trkSpeedMph,   2);
            fmtF(tHdgS, sizeof(tHdgS), rec.trkHeadingDeg, 1);
            char row[512];
            snprintf(row, sizeof(row),
                "%llu,%s,"
                "%.9f,%.9f,%.2f,%.2f,%s,%s,"
                "%.3f,%.3f,%d,%u,%d,%d,"
                "%d,%s,%.1f,%d,%d,%lu,"
                "%s,%u,%.2f,%.2f,%.2f,%s,%.1f,%u,%lu,%lu,"
                "%u,%s,%s",
                (unsigned long long)toUnixMs(rec.ms), utc,
                rec.lat, rec.lon, rec.altMSL, rec.geoidSepM, spdS, hdgS,
                rec.hAccM, rec.vAccM, (int)rec.rtkType, (unsigned)rec.staId,
                (int)rec.numSV, (int)rec.epeValid,
                (int)rec.ntripConn, mpt, rec.ntripKm, (int)rec.carrier, (int)rec.vrs,
                (unsigned long)rec.rtcmTotal,
                espT, (unsigned)rec.spdAgeMs,
                rec.pdop, rec.hdop, rec.vdop, rec.gsaSats, rec.diffAgeS,
                (unsigned)rec.bleState, (unsigned long)rec.bleDlv,
                (unsigned long)rec.bleSup,
                (unsigned)rec.velSrc, tSpdS, tHdgS);
            if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (!printlnCounted(gpsFile, row, rowsGps)) {
                    gpsErr = true; Serial.println("❌ SD GPS: write failed");
                } else if (millis() - flushGps >= 1000) { gpsFile.flush(); flushGps = millis(); }
                xSemaphoreGive(sdMutex);
            }
            if (gpsErr) {
                gpsHdr = false; gpsErr = false; activeLogName[0] = '\0';
                failChannel(gpsFile, "GPS");   // keeps sessionStamp — see failChannel
            }
        }
        haveRec = (++gpsRows < 64) && xQueueReceive(logQueue, &rec, 0) == pdTRUE;
        }

    }
}

// ── sdlog_init ────────────────────────────────────────────────────────────────
// Queue storage lives in PSRAM. The three queues total ~16.5 KB; queue storage
// is plain memcpy'd data — never touched by DMA or ISRs (all senders/receivers
// here are ordinary tasks) — so it is safe in external RAM. Internal SRAM is the
// scarce resource on this build (SDMMC DMA buffers and TCP sockets need it), so
// every KB moved out matters. The StaticQueue_t control blocks stay in internal
// .bss (~90 B each) as FreeRTOS accesses them in critical sections.
// If PSRAM allocation fails (no PSRAM / exhausted) we fall back to a normal
// internal-RAM queue so logging still works.
static StaticQueue_t logQueueCtrl, imuQueueCtrl, satQueueCtrl, canRawQueueCtrl, canQueueCtrl, nmeaQueueCtrl;

static QueueHandle_t createQueuePreferPsram(int len, int itemSz,
                                            StaticQueue_t* ctrl, const char* name) {
    uint8_t* storage = (uint8_t*)heap_caps_malloc((size_t)len * itemSz,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (storage) {
        QueueHandle_t q = xQueueCreateStatic(len, itemSz, storage, ctrl);
        if (q) {
            Serial.printf("📝 SD: %s queue in PSRAM (%d B)\n", name, len * itemSz);
            return q;
        }
        heap_caps_free(storage);
    }
    Serial.printf("⚠️  SD: %s queue using internal RAM (PSRAM unavailable)\n", name);
    return xQueueCreate(len, itemSz);
}

// Borrowed by debug_log (serial-to-SD mirror): the mirror file is written under the same SD
// mutex as the data logs, and named from the same session stamp so it sorts beside its run.
bool sdlog_takeMutex(uint32_t ms) {
    return sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(ms)) == pdTRUE;
}
void sdlog_giveMutex() { if (sdMutex) xSemaphoreGive(sdMutex); }
const char* sdlog_sessionStamp() { return pubStamp; }

void sdlog_init() {
    setenv("TZ", "UTC0", 1); tzset();   // GPS time is UTC; make mktime() interpret it as UTC
    sdMutex = xSemaphoreCreateMutex();
    if (!sdMutex) { Serial.println("❌ SD: mutex alloc failed"); return; }
    dirSnapLock = xSemaphoreCreateMutex();   // guards the web dir-view snapshot
    // Directory-listing tables in PSRAM — see their declaration for why they do
    // not belong in internal SRAM. Both must succeed or neither is used, since
    // the boot scan writes dirScanTmp and merges it into dirSnap.
    {
        size_t bytes = (size_t)DIR_SNAPSHOT_PSRAM * sizeof(DirSnapEntry);
        DirSnapEntry* a = (DirSnapEntry*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        DirSnapEntry* b = (DirSnapEntry*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (a && b) {
            dirSnap = a; dirScanTmp = b; dirSnapCap = DIR_SNAPSHOT_PSRAM;
            Serial.printf("📝 SD: file listing in PSRAM (%d entries, %u B)\n",
                          dirSnapCap, (unsigned)(bytes * 2));
        } else {
            if (a) heap_caps_free(a);
            if (b) heap_caps_free(b);
            dirSnap = dirSnapFallback; dirScanTmp = dirScanTmpFallback;
            dirSnapCap = DIR_SNAPSHOT_FALLBACK;
            Serial.printf("⚠️  SD: file listing in internal RAM (%d entries, PSRAM unavailable)\n",
                          dirSnapCap);
        }
        memset(dirSnap,    0, (size_t)dirSnapCap * sizeof(DirSnapEntry));
        memset(dirScanTmp, 0, (size_t)dirSnapCap * sizeof(DirSnapEntry));
    }
    logQueue = createQueuePreferPsram(60,  sizeof(LogRecord),    &logQueueCtrl, "GPS+CAN");
    if (!logQueue) { Serial.println("⚠️  SD: GPS queue alloc failed"); return; }
    // 1024 rows is about 4 s at the active IMU rate, enough to ride out a card garbage-collection pause.
    imuQueue = createQueuePreferPsram(psramRoomy() ? 1024 : 120, sizeof(ImuLogRecord), &imuQueueCtrl, "IMU");
    if (!imuQueue) { Serial.println("⚠️  SD: IMU queue alloc failed"); return; }
    // Decoded CAN snapshots (one LogRecord each): 256 rows is 5 s at the active rate.
    canQueue = createQueuePreferPsram(psramRoomy() ? 256 : 16, sizeof(LogRecord), &canQueueCtrl, "CAN");
    if (!canQueue) { Serial.println("⚠️  SD: CAN queue alloc failed"); return; }
    satQueue = createQueuePreferPsram(psramRoomy() ? 120 : 30, sizeof(SatLogRecord), &satQueueCtrl, "SAT");   // a ~26-sentence GSV burst plus an 800 ms card pause
    if (!satQueue) { Serial.println("⚠️  SD: SAT queue alloc failed"); return; }
    // Raw receiver capture: GGA+RMC at 20 Hz is 40 lines/s; 80 slots is two
    // seconds of card stall. Created always (PSRAM), used only when switched on.
    nmeaQueue = createQueuePreferPsram(80, sizeof(NmeaLogRecord), &nmeaQueueCtrl, "NMEA");
    // Raw CAN sniffer queue — deep (16384×16B=256KB PSRAM) to ride out SD write
    // stalls (card garbage-collection pauses of ~0.8 s) at a busy bus rate. Drop-on-full
    // keeps the CAN task real-time. 2048 rows when PSRAM is unavailable.
    canRawQueue = createQueuePreferPsram(psramRoomy() ? 16384 : 2048, sizeof(CanRawRecord), &canRawQueueCtrl, "CANraw");
    
    // 12288: the boot directory scan runs in THIS task (moved off the async_tcp
    // callback, which was overrunning the Task-WDT). Its entry table is heap-
    // allocated rather than a stack local, but the scan's String/File frames and
    // the CAN raw sniffer drain are disjoint branches whose frames overlap, and
    // this size keeps margin over whichever path is deeper. Watch the SDLog
    // watermark in the 30 s diagnostic print.
    xTaskCreatePinnedToCore(sdLogTask, "SDLog", 12288, nullptr, 1, nullptr, 0);
    Serial.println("📝 SD logging task started (Core 0)");
}

// ── Accessors ─────────────────────────────────────────────────────────────────
SemaphoreHandle_t sdlog_getMutex()      { return sdMutex; }
bool              sdlog_isReady()       { return sdReady; }
const char* sdlog_getActiveName() { return activeLogName; }
// Log files on the card (.csv, as the Log Files page lists them), from the RAM
// snapshot: the boot scan plus every file opened since, minus deletions. Counting
// only this boot's opens (logFileCount) read as "files on the card" and was not.
uint32_t          sdlog_getFileCount()  { return (uint32_t)dirSnapCount; }
uint32_t          sdlog_getWriteFaults() { return sdWriteFaults; }
uint32_t          sdlog_getActiveGpsSize() { return activeBytesByType[0]; }  // open GPS log bytes

// Copies the currently-open ("active") filenames + live sizes into caller buffers.
// Pure RAM read under a short critical section — performs NO SD access. Returns
// the count (0..5). Backs every web file view so open/growing files always appear
// instantly, independent of the directory scan. `sizes` may be null.
int sdlog_getActiveFiles(char names[][LOG_NAME_MAX], uint32_t* sizes, int cap) {
    int n = 0;
    taskENTER_CRITICAL(&activeMux);
    for (int t = 0; t < 5 && n < cap; t++)
        if (activeByType[t][0]) {
            memcpy(names[n], activeByType[t], 32);
            if (sizes) sizes[n] = activeBytesByType[t];
            n++;
        }
    // The serial mirror is an open file like the channels, so the open-file view lists it too.
    if (n < cap && debugFileName[0]) {
        memcpy(names[n], debugFileName, 32);
        if (sizes) sizes[n] = debugFileBytes;
        n++;
    }
    taskEXIT_CRITICAL(&activeMux);
    return n;
}

// True if `bare` (filename WITHOUT leading slash) is currently held OPEN for
// writing — any of the four typed channels OR the canraw sniffer dump. The web
// delete paths use this to refuse removing a live file: SD_MMC.remove() on an
// open handle corrupts FatFs and resets the unit. Race-safe under activeMux.
bool sdlog_isActiveFile(const char* bare) {
    if (!bare || !bare[0]) return false;
    bool match = false;
    taskENTER_CRITICAL(&activeMux);
    for (int t = 0; t < 5; t++)
        if (activeByType[t][0] && strncmp(activeByType[t], bare, 32) == 0) { match = true; break; }
    if (!match && canRawName[0] && strncmp(canRawName, bare, 32) == 0) match = true;
    if (!match && debugFileName[0] && strncmp(debugFileName, bare, 32) == 0) match = true;
    taskEXIT_CRITICAL(&activeMux);
    return match;
}

// ── Directory snapshot accessors (web recent/all views — RAM only, no SD) ─────
// True for the (brief, time-sliced) duration of the one boot-time scan only —
// dirScanReq is cleared the instant the scan STARTS, dirScanActive stays true
// until it finishes across however many passes it takes.
bool sdlog_dirScanPending() { return dirScanReq || dirScanActive; }

// Copy up to `cap` newest snapshot entries (already sorted newest-first) into the
// caller's buffers. Returns the count; *trunc set if the card holds more files
// than the snapshot cap; *gen returns the snapshot generation (0 = never built).
int sdlog_getDirSnapshot(char names[][LOG_NAME_MAX], uint32_t* sizes, int cap,
                         bool* trunc, uint32_t* gen) {
    return sdlog_getDirSnapshotPage(names, sizes, 0, cap, nullptr, trunc, gen);
}

// Paged read of the same snapshot, newest first. `offset` skips that many of the
// newest entries and `total` reports how many exist, which is what lets the web
// file view page through a card holding far more files than any one JSON response
// should carry — the response is assembled in internal heap, so an unbounded list
// would cost more RAM than the listing is worth. Same locking and same zero SD
// access as every other snapshot accessor.
int sdlog_getDirSnapshotPage(char names[][LOG_NAME_MAX], uint32_t* sizes,
                             int offset, int cap, int* total,
                             bool* trunc, uint32_t* gen) {
    int n = 0;
    if (offset < 0) offset = 0;
    if (dirSnap && dirSnapLock && xSemaphoreTake(dirSnapLock, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (total) *total = dirSnapCount;
        for (int i = offset; i < dirSnapCount && n < cap; i++, n++) {
            memcpy(names[n], dirSnap[i].name, 32);
            if (sizes) sizes[n] = dirSnap[i].size;
        }
        if (trunc) *trunc = dirSnapTrunc;
        if (gen)   *gen   = dirSnapGen;
        xSemaphoreGive(dirSnapLock);
    } else {
        if (total) *total = 0;
        if (trunc) *trunc = false;
        if (gen)   *gen   = 0;
    }
    return n;
}

// SD capacity, cached by sdLogTask. Web /sd serves this without touching the bus.
void sdlog_getCapacity(uint64_t* total, uint64_t* used) {
    if (total) *total = capTotal;
    if (used)  *used  = capUsed;
}

// ── Bulk export accessors ─────────────────────────────────────────────────────
void sdlog_requestExport(const char* namesCSV, const char* archiveName) {
    // Parse and validate BEFORE flipping to BUILDING, so a malformed request
    // can't leave a stale selection from a previous call mixed with a partial
    // new one. Same validation as the batch-delete route: root .csv only, no
    // path separators — this list reaches SD_MMC.open() directly.
    exportSelectCount = 0;
    exportIsSubset = (namesCSV && namesCSV[0]);
    if (exportIsSubset) {
        int start = 0, len = (int)strlen(namesCSV);
        while (start <= len && exportSelectCount < EXPORT_SELECT_MAX) {
            const char* comma = strchr(namesCSV + start, ',');
            int end = comma ? (int)(comma - namesCSV) : len;
            int n = end - start;
            if (n > 0 && n < LOG_NAME_MAX) {
                char cand[LOG_NAME_MAX];
                memcpy(cand, namesCSV + start, n); cand[n] = '\0';
                if (sdlog_isLogFileName(cand) &&
                    !strchr(cand, '/') && !strchr(cand, '\\')) {
                    memcpy(exportSelectNames[exportSelectCount], cand, n + 1);
                    exportSelectCount++;
                }
            }
            if (!comma) break;
            start = end + 1;
        }
    }
    // Flip to BUILDING immediately so a status poll right after the POST already
    // reports progress, even before sdLogTask picks up the request.
    if (exportState != EXP_BUILDING) {
        // The name reaches SD_MMC.open() and the listing, so it is checked here as well as in the web
        // handler: a bare root .tar name or the default for this kind of export.
        const bool nameOk = archiveName && sdlog_isArchiveName(archiveName);
        snprintf(exportPath, sizeof(exportPath), "/%s",
                 nameOk ? archiveName : (exportIsSubset ? "rcx_logs_sel.tar" : "rcx_logs_all.tar"));
        exportState = EXP_BUILDING; exportReq = true;
    }
}
void sdlog_getExportState(int* state, uint64_t* size, int* count) {
    if (state) *state = exportState;
    if (size)  *size  = exportSize;
    if (count) *count = exportCount;
}
const char* sdlog_getExportPath() { return exportPath; }
bool sdlog_exportIsSubset() { return exportIsSubset; }

// These report the operator's INTENDED set, so the dashboard's channel toggles
// keep showing what will run rather than flipping themselves off during a pause.
// Use sdlog_isPaused() to tell whether anything is actually recording right now.
bool sdlog_getLogGps() { return wantGps; }
bool sdlog_getLogImu() { return wantImu; }
bool sdlog_getLogCan() { return wantCan; }
bool sdlog_getLogSat() { return wantSat; }
bool sdlog_getLogNmea() { return wantNmea; }

void sdlog_setLogNmea(bool on) {
    wantNmea = on;
    if (!logPaused) logNmea = on;    // while paused it lands on resume, like the others
    saveLogConfig();
    Serial.printf("👤 USER [%lus]: raw receiver capture %s%s\n", (unsigned long)(millis() / 1000UL), on ? "ON" : "OFF", logPaused ? " (applies on resume)" : "");
}

bool sdlog_isPaused() { return logPaused; }

void sdlog_setPaused(bool paused) {
    if (paused == logPaused) return;
    logPaused = paused;
    if (paused) {
        // Clearing the effective flags is what makes sdLogTask flush and close
        // each open file on its next pass; nothing is written to NVS.
        logGps = false; logImu = false; logCan = false; logSat = false; logNmea = false;
        RCX_USER_ACTION("log PAUSED — channels closed, files released");
    } else {
        logGps = wantGps; logImu = wantImu; logCan = wantCan; logSat = wantSat; logNmea = wantNmea;
        Serial.printf("👤 USER [%lus]: log RESUMED GPS=%d IMU=%d CAN=%d SAT=%d\n",
                      (unsigned long)(millis() / 1000UL), wantGps, wantImu, wantCan, wantSat);
    }
}

void sdlog_setConfig(bool gps, bool imu, bool can, bool sat) {
    wantGps = gps; wantImu = imu; wantCan = can; wantSat = sat;
    // While paused the effective flags stay false and the change lands on resume.
    // Applying it live instead would restart recording behind the operator's back,
    // and resume would then overwrite the choice they just made.
    if (!logPaused) { logGps = gps; logImu = imu; logCan = can; logSat = sat; }
    saveLogConfig();
    Serial.printf("👤 USER [%lus]: log config GPS=%d IMU=%d CAN=%d SAT=%d%s\n",
                  (unsigned long)(millis() / 1000UL), gps, imu, can, sat, logPaused ? " (applies on resume)" : "");
}

#else
void sdlog_init()        {}
void sdlog_push_if_new() {}
void sdlog_push_imu(float,float,float,float,float,float) {}
void sdlog_push_can() {}
bool sdlog_carActive() { return true; }
void sdlog_push_sat(const char*) {}
bool sdlog_getLogNmea() { return false; }
void sdlog_setLogNmea(bool) {}
void sdlog_push_nmea(const char*) {}
void sdLogTask(void*)    {}
SemaphoreHandle_t sdlog_getMutex()      { return nullptr; }
bool              sdlog_isReady()       { return false; }
const char* sdlog_getActiveName() { return ""; }
uint32_t          sdlog_getFileCount()  { return 0; }
uint32_t          sdlog_getWriteFaults() { return 0; }
uint32_t          sdlog_getActiveGpsSize() { return 0; }
int  sdlog_getActiveFiles(char[][LOG_NAME_MAX], uint32_t*, int) { return 0; }
bool sdlog_isActiveFile(const char*) { return false; }
void sdlog_noteFileDeleted(const char*) {}
void sdlog_noteDebugFileOpen(const char*) {}
bool sdlog_takeMutex(uint32_t) { return false; }
void sdlog_giveMutex() {}
const char* sdlog_sessionStamp() { return ""; }
void sdlog_noteDebugFileClosed(const char*, uint32_t) {}
void sdlog_noteDebugFileSize(uint32_t) {}
bool sdlog_isLogFileName(const char*) { return false; }
bool sdlog_isArchiveName(const char*) { return false; }
bool sdlog_dirScanPending() { return false; }
int  sdlog_getDirSnapshot(char[][LOG_NAME_MAX], uint32_t*, int, bool* t, uint32_t* g) {
    if (t) *t = false; if (g) *g = 0; return 0;
}
int  sdlog_getDirSnapshotPage(char[][LOG_NAME_MAX], uint32_t*, int, int, int* n, bool* t, uint32_t* g) {
    if (n) *n = 0; if (t) *t = false; if (g) *g = 0; return 0;
}
bool sdlog_isPaused() { return false; }
bool sdlog_downloadInProgress() { return false; }
void sdlog_setPaused(bool) {}
void sdlog_getCapacity(uint64_t* t, uint64_t* u) { if (t) *t = 0; if (u) *u = 0; }
void sdlog_requestExport(const char*, const char*) {}
void sdlog_getExportState(int* s, uint64_t* sz, int* c) { if (s) *s = 0; if (sz) *sz = 0; if (c) *c = 0; }
const char* sdlog_getExportPath() { return "/rcx_logs_all.tar"; }
bool sdlog_exportIsSubset() { return false; }
void sdlog_push_can_raw(uint32_t, uint16_t, uint8_t, const uint8_t*) {}
uint32_t sdlog_getCanRawDrops() { return 0; }
uint32_t sdlog_getGpsRowDrops() { return 0; }
bool sdlog_getLogGps() { return false; }
bool sdlog_getLogImu() { return false; }
bool sdlog_getLogCan() { return false; }
bool sdlog_getLogSat() { return false; }
void sdlog_setConfig(bool,bool,bool,bool) {}
#endif