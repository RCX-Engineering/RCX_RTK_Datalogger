#pragma once
/*
 * bridge_sd_log.h — SD logging for the RCX1 NTRIP base station
 * ============================================================
 * Mirrors the RCX RaceCapture rover's SD approach (SD_MMC 1-bit, Core-0 task,
 * queue-fed CSV) so files/workflow stay convenient and consistent. Produces:
 *
 *   gps_NNNN.csv     — position/fix/survey, one row per epoch (~1 Hz)
 *   sat_NNNN.csv     — one row per satellite per snapshot (from GSV)
 *   status_NNNN.csv  — RTCM "LIGHT": heartbeat of online/stream health (~0.1 Hz).
 *                      Tiny, quick writes, runs essentially forever.
 *   rtcm_NNNN.csv    — RTCM "DETAILED": one row per RTCM frame (type/len/crc/
 *                      per-caster accept status). Verbose; opt-in.
 *   rtcm_NNNN.bin    — (optional) raw capture of the exact RTCM bytes pushed.
 *
 * NNNN is a per-power-cycle session counter in NVS (no GNSS time dependency, no
 * filename collisions). Every CSV row carries a `datetime` column built from RMC
 * date + GGA time-of-day (YYYY-MM-DD HH:MM:SS.ss), blank until the first valid fix,
 * so rows can be correlated to a real calendar session. `ms` (boot millis) remains
 * the monotonic anchor and is always present even before a fix.
 *
 * ARCHITECTURE / WHY: every SD write goes through a FreeRTOS queue drained by a
 * task pinned to Core 0. FAT cluster-allocation stalls (up to ~100 ms) therefore
 * NEVER block the Core-1 loop that drains the 460800-baud GNSS UART. Do not call
 * any SD write directly from loop()/the RTCM parser — only enqueue.
 *
 * RAW .bin FLUSH (do not remove): the raw capture is flushed periodically, not only
 * on rotation. Without periodic flush the file's directory entry/size is not
 * committed until the 64 MB rotation, so a card pulled or inspected mid-session
 * reads 0 bytes despite data being buffered. See drainRawStream() in the .cpp.
 */

#include <Arduino.h>

// ── Feature flags (here, NOT config.h, so config.h stays verbatim) ───────────
#ifndef BRIDGE_LOG_SD_ENABLE
#define BRIDGE_LOG_SD_ENABLE          true
#endif
#ifndef BRIDGE_LOG_GPS_ENABLE
#define BRIDGE_LOG_GPS_ENABLE         true
#endif
#ifndef BRIDGE_LOG_SAT_ENABLE
#define BRIDGE_LOG_SAT_ENABLE         true
#endif
#ifndef BRIDGE_LOG_STATUS_ENABLE        // RTCM "light" heartbeat
#define BRIDGE_LOG_STATUS_ENABLE      true
#endif
#ifndef BRIDGE_LOG_RTCM_DETAIL_ENABLE   // RTCM "detailed" per-frame metadata CSV
#define BRIDGE_LOG_RTCM_DETAIL_ENABLE false
#endif
#ifndef BRIDGE_LOG_RTCM_RAW_ENABLE      // raw .bin of the exact RTCM byte stream
#define BRIDGE_LOG_RTCM_RAW_ENABLE    false
#endif

// OPT-IN SD RECOVERY: if a mount fails at BOTH the normal and slow clock, allow a
// one-shot reformat to recover a card whose FAT was corrupted by an unclean power-off.
// DEFAULT false so a card with merely flaky contacts is never silently wiped and its
// data stays recoverable on a PC. Set to true for a single rescue build, then revert.
#ifndef BRIDGE_SD_FORMAT_ON_FAIL
#define BRIDGE_SD_FORMAT_ON_FAIL      false
#endif

// ─────────────────────────────────────────────────────────────────────────────
// ⚠⚠  SD_MMC PINS — NEVER assign USB / FLASH / PSRAM / STRAPPING GPIOs HERE  ⚠⚠
// On the ESP32-S3 these are RESERVED and must NOT be used for the SD card:
//   • GPIO19, GPIO20  → native USB D−/D+. Using these BREAKS USB flashing and
//                       forces BOOT-button/UART recovery — a royal pain.
//   • GPIO26–GPIO32   → SPI flash.
//   • GPIO33–GPIO37   → octal PSRAM (this board is an ESP32-S3R8, 8 MB PSRAM).
//   • GPIO0,3,45,46   → strapping.
// A compile-time static_assert in bridge_sd_log.cpp enforces the USB/flash/PSRAM
// rule so this mistake cannot ship again.
//
// Board TF-card pins — CONFIRMED against the working rover wiring:
//   CLK=14, CMD=15, D0=16, D3=21. D3/CS has a 10K pull-up on the board, so the
//   firmware does NOT drive it (1-bit mode uses CLK/CMD/D0 only).
#ifndef BRIDGE_SD_CLK_PIN
#define BRIDGE_SD_CLK_PIN  14   // TF_SCLK → SDMMC CLK
#endif
#ifndef BRIDGE_SD_CMD_PIN
#define BRIDGE_SD_CMD_PIN  15   // TF_MOSI → SDMMC CMD
#endif
#ifndef BRIDGE_SD_D0_PIN
#define BRIDGE_SD_D0_PIN   16   // TF_MISO → SDMMC D0
#endif
#ifndef BRIDGE_SD_D3_PIN
#define BRIDGE_SD_D3_PIN   21   // TF_CS → SDMMC D3; 10K pull-up on board (not driven in firmware)
#endif

// ── Cadences / limits ────────────────────────────────────────────────────────
#ifndef BRIDGE_LOG_STATUS_PERIOD_MS
#define BRIDGE_LOG_STATUS_PERIOD_MS  5000    // heartbeat: 1 row / 5 s — starts at SD-ready,
                                             // not at caster-connected, so boot/survey-in
                                             // sequence is always captured for diagnosis.
#endif
#ifndef BRIDGE_LOG_GPS_PERIOD_MS
#define BRIDGE_LOG_GPS_PERIOD_MS     1000    // base position is slow; 1 Hz is plenty
#endif
#ifndef BRIDGE_LOG_SAT_PERIOD_MS
#define BRIDGE_LOG_SAT_PERIOD_MS     1000    // one all-constellation sat snapshot / s
#endif
// Queue depth in rows (each row is a fixed ~353-byte LogLine, so 192 rows ~= 66 KiB).
// SIZING: the sat channel emits one row PER SATELLITE per second — roughly 28 rows/s
// against ~1/s for gps and 0.2/s for status — so production runs at ~30 rows/s. At 96
// the queue held only ~3.3 s of production, which a root-directory scan or an SD
// latency spike exceeds easily; the overflow appears as missing sat rows and gaps in
// gps while status (being far slower) looks fine. 192 gives ~6.5 s of headroom.
// Raising it further costs proportional RAM — check log_drop in the status CSV first.
#ifndef BRIDGE_LOG_QUEUE_DEPTH
// Queue depth x LogLine size is INTERNAL heap, and internal heap is the scarce pool on
// this board — the SD driver's DMA buffers, lwIP sockets and the WiFi stack all come from
// it, while 8 MB of PSRAM sits unused because none of them can use it. At depth 192 with
// a 352-byte line this queue alone held ~68 kB of it, more than half of what is free
// before WiFi starts, which is why a long session ended with sdmmc_cmd reporting
// ESP_ERR_NO_MEM. Depth is what absorbs a Core-0 stall, so it is traded against line
// width rather than cut alone; g_dropped is logged, so an overrun is visible rather than
// silent.
#define BRIDGE_LOG_QUEUE_DEPTH       96
#endif
// Rotate every CSV tag (gps/sat/status/rtcm) and the raw .bin at this size (0 = never),
// starting a new file with a letter suffix (_a, _b, ... _z). Keeps a long unattended
// session from producing one unmanageably huge file and, for gps/sat/status, keeps a
// multi-week deployment well clear of FAT32's ~4 GiB single-file cap. 128 MB is chosen
// so an individual log stays small enough to upload from the road for analysis rather
// than being sized purely by the filesystem limit.
#ifndef BRIDGE_LOG_ROTATE_BYTES
#define BRIDGE_LOG_ROTATE_BYTES      (128UL * 1024UL * 1024UL)
#endif
// Rotation letter suffixes run _a.._z (26 rotations — 3.25 GB per tag at the default
// threshold). Past that, rotation stops and the tag keeps appending to the 'z' file
// rather than wrapping into a colliding or garbage filename.
#ifndef BRIDGE_LOG_ROTATE_MAX_SEQ
#define BRIDGE_LOG_ROTATE_MAX_SEQ    26
#endif

// Snapshot of everything the periodic logs need. loop() fills this from its
// globals and hands it to bridge_sdlog_tick(); the logger owns nothing in the
// .ino, keeping the two decoupled.
struct BridgeLogCaster {
    char     host[40];
    char     mount[24];       // what the caster rejection is usually actually about
    char     state[40];
    bool     enabled;         // operator's switch — separates "turned off" from "held down"
    char     err[56];         // caster's last error, or the quality gate's reason when held
    bool     handshake;
    uint64_t framesWritten;
    uint64_t bytesAccepted;
    uint32_t dropped;
    int32_t  lastWriteAgeS;   // -1 = never
};

struct BridgeLogSnapshot {
    uint32_t nowMs;
    char     utc[16];         // GGA time-of-day "hhmmss.ss", or ""
    char     date[12];        // RMC calendar date "ddmmyy", or "" before first valid fix
    bool     haveFix;
    int      fixQuality;
    int      satellites;
    double   hdop;
    double   lat, lon, altM;
    double   epeM;
    double   epe2dM;          // horizontal (2D) EPE; epeM is 3D-preferred
    double   ecefX, ecefY, ecefZ;
    uint8_t  svinValid;
    uint32_t svinObs, svinTarget;
    bool     svinMeanAccKnown;   // false → svin_meanacc_m is written blank, not 0.000
    double   svinMeanAccM;
    uint8_t  svinMode;     // probed receiver svin mode (0=unconfigured,1=survey-in,2=fixed)
    // PROVENANCE of the coordinate we are casting from. Logged every status row so a
    // session can always be traced back to HOW the position was obtained and what
    // accuracy was assumed for it — a fixed base reports no live accuracy of its own.
    char     posSource[20];   // "survey", "ppp", "manual", "" if none saved
    double   posAccM;         // assumed accuracy of that saved position (m), <0 = unknown
    bool     hotStart;     // true if configureLg290pBaseOnce skipped reconfigure (warm boot)
    bool     baseReady;
    uint32_t rtcmFps, rtcmBps;
    // DMA-capable INTERNAL heap. Not a curiosity: the SD driver, lwIP sockets and the
    // WiFi stack all allocate from it, and PSRAM cannot substitute for any of them. When
    // it runs out the symptoms are scattered and none of them name memory — an SD write
    // fails, a caster cannot reconnect, a rover cannot be accepted — which is exactly the
    // shape of an 11-hour session that ended with sdmmc_cmd reporting ESP_ERR_NO_MEM.
    // Logged every row so the TREND is visible; a single boot-time figure cannot show a leak.
    uint32_t heapFreeInt, heapLargestInt, heapMinInt;
    // Built-in caster: rovers streaming right now, and accepted since boot. This is the
    // correction path a rover actually uses at an event, and it had NO telemetry at all —
    // whether a rover ever reached the device was answerable only from a serial console,
    // which is exactly where the base cannot be when it matters.
    uint8_t  localClients;
    uint32_t localServed;
    uint64_t rtcmValidFrames;
    // UART-INTEGRITY COUNTERS. These distinguish "the receiver produced less RTCM" from
    // "we LOST RTCM off the wire". A loop stall over ~89 ms overruns the 4096-byte GNSS
    // ring at 460800 baud, which shreds frames (CRC/framing failures climb) and tears
    // NMEA sentences. Session 0454 showed both symptoms together — RTCM output dropping
    // while GGA/RMC time jumped by seconds and one epoch logged a corrupted year — with
    // no change in satellite C/N0. Without these counters the two causes look identical.
    // Peak loop-pass duration and peak GNSS UART ring occupancy since the last status
    // row. Together these say WHERE the byte loss comes from: a big loopMaxMs with a
    // high uartHighWater means the loop stalled long enough to overrun the ring (our
    // bug); corruption with both low means bytes are being lost on the wire itself
    // (baud/signal-integrity), which no amount of firmware scheduling will fix.
    uint32_t loopMaxMs;
    uint32_t uartHighWater;
    uint32_t rtcmCrcFailures;
    uint32_t rtcmFramingFailures;
    uint32_t nmeaChecksumFailures;   // REAL corruption — any non-zero rate = UART byte loss
    uint32_t nmeaFramerDesyncs;      // benign: '$' byte inside binary RTCM; expected background
    bool     wifiConnected;
    char     ssid[24];
    char     ip[20];
    float    chipTempF;       // ESP32-S3 die temperature, °F (always valid)
    float    boardTempF;      // QMI8658 board/ambient temperature, °F
    bool     boardTempOk;     // false → board temp column left blank in the CSV

    // ── Boot position-confidence check ("has the base moved?") ──────────────
    // Logged so the decision can be reconstructed from the card afterwards. The check
    // runs for ~30 s at boot, well inside the status log's cadence, so several rows
    // capture it. Without these the ONLY record of why a base re-surveyed was a live
    // dashboard reading that nobody was watching at boot.
    char     pcState[12];     // idle / collecting / confirmed / moved / timeout
    uint32_t pcFixes;         // epochs accumulated so far in the window
    double   pcLat;           // running/measured mean of the boot fixes (0 = none yet)
    double   pcLon;
    double   pcEpeM;          // mean per-fix horizontal EPE across those boot fixes
    double   pcDistM;         // measured distance from that mean to the SAVED coordinate
    double   pcThreshM;       // move threshold this check is being judged against
    BridgeLogCaster caster[2];
};

// Call once from setup() (starts the Core-0 task, mounts the card).
void bridge_sdlog_init();

// Call from loop() — internally rate-limited; emits gps/status rows when due.
void bridge_sdlog_tick(const BridgeLogSnapshot& snap);

// Call from handleNmeaSentence() for every NMEA line — used to extract satellites
// from GSV. Cheap and ignores non-GSV lines.
void bridge_sdlog_feed_nmea(const char* sentence);

// Call from handleValidRtcmFrame() for every CRC-valid frame (detailed/raw tiers).
// `detail` is a decoded, comma-free summary of the frame contents (station ID, ECEF
// position, sat/sig counts, etc.) for the detailed CSV — "" when nothing decoded.
void bridge_sdlog_rtcm_frame(uint16_t type, uint16_t len, bool crcOk,
                             const char* c0status, const char* c1status,
                             const char* detail, const uint8_t* frame);

// Dropped-record counter (queue full) — surface on the dashboard if desired.
// Record a diagnostic event to the SD card. Pair it with a serial print via logEvent()
// in the sketch rather than calling this directly, so the two can never drift apart.
// level: "ok" | "warn" | "fail" | "info". Commas and newlines in msg are neutralised.
void bridge_sdlog_event(const char* level, const char* msg);

uint32_t bridge_sdlog_dropped();
bool     bridge_sdlog_ready();
uint16_t bridge_sdlog_session();

// Runtime channel control (persisted in NVS). ch is a BridgeLogChannel.
enum BridgeLogChannel { BLOG_GPS = 0, BLOG_SAT = 1, BLOG_STATUS = 2, BLOG_RTCM = 3,
                        BLOG_RAW = 4, BLOG_EVENT = 5 };
void bridge_sdlog_set_channel(int ch, bool on);
bool bridge_sdlog_get_channel(int ch);

// Tell the logger a web download is in progress so the task pauses card writes
// (single reader, no contention). MUST be paired: set true before streaming,
// false in all exit paths.
void bridge_sdlog_set_download_active(bool active);
