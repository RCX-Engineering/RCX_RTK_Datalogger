#pragma once
/*
 * sd_log.h — SD card logger
 * =========================
 * Four separate log files, each independently toggleable from the web UI:
 *
 * gps_YYYYMMDD_HHMMSS.csv   — Position, RTK, NTRIP, accuracy  (20 Hz)
 * imu_YYYYMMDD_HHMMSS.csv   — Accel + Gyro                    (50 Hz)
 * can_YYYYMMDD_HHMMSS.csv   — RPM, speeds, temps, steering     (20 Hz)
 * sat_YYYYMMDD_HHMMSS.csv   — Raw GSV NMEA (Satellites in view)(1 Hz)
 *
 * All files share the same timestamp base (first valid GPS fix).
 * Channel enables persist across reboots via NVS (namespace "rcx_log").
 */

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "types.h"

// IMU record pushed at 50 Hz independently of the GPS epoch.
struct ImuLogRecord { uint32_t ms; float ax, ay, az, gx, gy, gz; };

// SAT record for pushing raw NMEA sentences asynchronously
struct SatLogRecord { uint32_t ms; char sentence[84]; };

// ── Lifecycle ─────────────────────────────────────────────────────────────────
void sdlog_init();          // creates mutex/queues, starts sdLogTask
void sdlog_push_if_new();   // GPS + CAN snapshot (called from loop at 20 Hz)
void sdlog_push_imu(float ax, float ay, float az,
                    float gx, float gy, float gz);  // IMU snapshot (50 Hz)
void sdlog_push_sat(const char* nmeaString);        // Raw GSV string snapshot

void sdLogTask(void*);      // internal FreeRTOS task

// ── Accessors used by webserver.cpp ──────────────────────────────────────────
SemaphoreHandle_t sdlog_getMutex();
bool              sdlog_isReady();
// Width of every buffer that holds a bare log filename (no leading slash).
// The longest name the logger can produce is the sniffer's, at a per-session
// stamp plus a uniqueness ordinal: canraw_YYYYMMDD_HHMMSS_UTC_999.csv — 34
// characters. Every table, API signature and validation buffer that carries a
// filename uses this, so the width is stated once and the compiler catches any
// site that disagrees.
#define LOG_NAME_MAX 40

const char* sdlog_getActiveName();  // active GPS file, or "" if none
uint32_t          sdlog_getFileCount();   // total files opened (no dir scan)
uint32_t          sdlog_getActiveGpsSize(); // on-disk size (bytes) of the open GPS log

// Currently-open ("active") files + live sizes for the file views — RAM only, no
// SD scan. Fills names[] (each char[LOG_NAME_MAX]) and sizes[] (may be null) with up to
// `cap` entries; returns the count. Open files always appear here while growing.
int sdlog_getActiveFiles(char names[][LOG_NAME_MAX], uint32_t* sizes, int cap);

// True if `bare` (filename without leading slash) is currently open for writing
// (any typed channel or the canraw sniffer dump). Web delete paths use this to
// refuse removing a live file (removing an open file corrupts FatFs + resets).
bool sdlog_isActiveFile(const char* bare);

// ── Web dir-view snapshot (recent/all) — built by sdLogTask, read by webserver ─
// The web handler MUST NOT enumerate the SD root itself (that overran the
// async_tcp Task-WDT) — and per a real field failure, sdLogTask itself must
// never re-enumerate it either: a directory walk holds sdMutex for its whole
// duration, and a card that has accumulated hundreds of files (as it can after
// repeated thermal restarts) can hold it long enough to starve the GPS/CAN/
// IMU/SAT writes' own much shorter mutex waits, silently dropping rows. So
// there is exactly ONE scan, ever: once at boot, to discover files left over
// from past power cycles (sd_log.cpp). Every file created after that is added
// to this snapshot incrementally, the moment it's opened/closed — since this
// firmware is the only thing that ever creates these files, it already knows
// about each one without asking the directory again. getDirSnapshot() copies
// the newest entries out (no SD access, ever, after boot); dirScanPending() is
// true only during that one boot-time scan.
// The snapshot tables live in PSRAM (internal SRAM is the scarce resource here),
// which is what lets them hold the whole card's listing rather than a recent
// window. `truncated` still reports the case where even that was exceeded.
bool sdlog_dirScanPending();
int  sdlog_getDirSnapshot(char names[][LOG_NAME_MAX], uint32_t* sizes, int cap,
                          bool* truncated, uint32_t* gen);

// Paged variant, newest first: skips `offset` entries, copies up to `cap`, and
// reports the full entry count in `total`. The web file view pages with this so
// no single JSON response has to hold a whole card's worth of filenames — that
// response is built in internal heap, where a few hundred entries would cost
// more RAM than the listing is worth. `sizes`, `total`, `truncated` and `gen`
// may each be null. No SD access, same as every accessor above.
int  sdlog_getDirSnapshotPage(char names[][LOG_NAME_MAX], uint32_t* sizes,
                              int offset, int cap, int* total,
                              bool* truncated, uint32_t* gen);

// Remove a snapshot entry once its file is actually deleted (web /log/<f>
// DELETE and /sd/deleteold). Safe to call directly from the web task: this
// only touches dirSnapLock, a small RAM-only mutex, never sdMutex or the SD
// bus, the same as the other snapshot accessors above. Without this, a
// deleted file would stay listed forever — the snapshot is never rebuilt
// from a fresh scan after boot, only ever added to (see the note above), so
// a removal has to be told explicitly or it never happens.
void sdlog_noteFileDeleted(const char* name);

// SD capacity, refreshed by sdLogTask. Web /sd reads this with no SD-bus access.
void sdlog_getCapacity(uint64_t* total, uint64_t* used);

// ── Download priority ──────────────────────────────────────────────────────
// The web layer calls these around each file transfer (see webserver.cpp's
// DlState). While a transfer is in flight, sdLogTask defers its own periodic,
// read-only SD maintenance — the capacity refresh and the directory scan —
// since both walk the FAT and would otherwise compete with the transfer's
// reads on the single-lane SD bus. Deferred work is never dropped, only
// delayed: a pending directory scan stays pending and runs on the first pass
// once every transfer has ended. This never touches the recording write path
// — gps/imu/can/sat flush cadences are unconditional — so an in-progress
// download can slow how soon a stale file listing catches up, never whether
// or when a sample gets logged.
void sdlog_downloadBegin();
void sdlog_downloadEnd();

// ── Bulk export (.tar) — built by sdLogTask, streamed by webserver ────────────
// requestExport() queues a build of /export.tar (all CSVs); getExportState()
// reports progress (0=idle 1=building 2=ready 3=error) + size + file count;
// getExportPath() is the file the web streams once state==ready.
// Queue a tar build in sdLogTask. namesCSV null/empty builds every .csv on the
// card (the original "download all"); a comma-separated list of bare filenames
// builds only those (skipping any that are missing or currently open for
// writing). Bounded to EXPORT_SELECT_MAX names — see sd_log.cpp.
void        sdlog_requestExport(const char* namesCSV = nullptr);
void        sdlog_getExportState(int* state, uint64_t* size, int* count);
const char* sdlog_getExportPath();
bool        sdlog_exportIsSubset();   // true if the last built export was a selection, not "all"

// ── CAN sniffer raw-frame logging ─────────────────────────────────────────────
// Called from canBusTask for EVERY frame while sniffer mode is on. Non-blocking:
// the frame is queued (PSRAM) and written by sdLogTask to a separate
// canraw_<stamp>.csv. If the queue is full (SD can't keep up with bus rate) the
// frame is DROPPED and counted — it never blocks the CAN task or any other task.
void     sdlog_push_can_raw(uint32_t ms, uint16_t id, uint8_t dlc, const uint8_t* data);
uint32_t sdlog_getCanRawDrops();   // frames dropped because the raw queue was full

// Atomically update all four flags and save to NVS.
void sdlog_setConfig(bool logGps, bool logImu, bool logCan, bool logSat);

// ── Master pause ─────────────────────────────────────────────────────────────
// Pausing suspends every channel and closes the files they hold open, which is
// what allows the current session's files to be downloaded or deleted from the
// web file view — an open file can never be removed (doing so corrupts FatFs and
// resets the device). Resuming restores the channel set that was running and
// opens fresh files; the paused session's files are complete and closed, not
// appended to. Pause is RAM-only, so a reboot always comes back logging.
//
// The per-channel getters above report the operator's intended set, which does
// not change across a pause; ask sdlog_isPaused() to know whether anything is
// actually recording right now.
bool sdlog_isPaused();
void sdlog_setPaused(bool paused);

// Borrowed by debug_log (serial-to-SD tee): share the SD mutex and the session
// stamp so the mirror file writes under the same lock and matches the run name.
bool        sdlog_takeMutex(uint32_t ms);
void        sdlog_giveMutex();
const char* sdlog_sessionStamp();

// ── Per-channel log enables (persisted in NVS) ───────────────────────────────
bool sdlog_getLogGps();
bool sdlog_getLogImu();
bool sdlog_getLogCan();
bool sdlog_getLogSat();