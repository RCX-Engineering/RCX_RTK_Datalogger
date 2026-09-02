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
static volatile bool     g_sdReady   = false;
static volatile uint32_t g_dropped   = 0;
static uint16_t          g_session   = 0;

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
static void enqueue(const LogLine& ln) {
    if (!g_queue) return;
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
//   ecef_x_m / ecef_y_m / ecef_z_m = base ECEF position components (metres)
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
        case TAG_GPS:    return "ms,utc_time,fix,fix_name,sats,hdop,lat_deg,lon_deg,alt_m,epe_3d_m,epe_2d_m,"
                                "ecef_x_m,ecef_y_m,ecef_z_m,svin_valid,svin_obs_s,"
                                "svin_target_s,svin_meanacc_m,base_ready";
        case TAG_SAT:    return "ms,utc_time,constellation,prn,elev_deg,azim_deg,snr_dbhz";
        case TAG_STATUS: return "ms,utc_time,uptime_s,wifi,ssid,ip,fix,sats,base_ready,"
                                "svin_valid,svin_mode,svin_obs_s,svin_meanacc_m,hot_start,"
                                "rtcm_fps,rtcm_bps,valid_frames,"
                                "c0_host,c0_mount,c0_en,c0_state,c0_hand,c0_frames,c0_bytes,c0_drop,c0_age_s,c0_err,"
                                "c1_host,c1_mount,c1_en,c1_state,c1_hand,c1_frames,c1_bytes,c1_drop,c1_age_s,c1_err,"
                                "lat_deg,lon_deg,acc_m,pos_src,pos_acc_m,"
                                "loop_max_ms,uart_hw,rtcm_crc_fail,rtcm_frame_fail,nmea_cs_fail,nmea_desync,"
                                "esp_temp_f,imu_temp_f,heap_int,heap_largest,heap_min,lc_clients,lc_served,log_drop,"
                                "pc_state,pc_fixes,pc_lat,pc_lon,pc_epe_m,pc_dist_m,pc_thresh_m";
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
static File     g_rawFile;
static uint32_t g_rawBytes = 0;
static uint8_t  g_rawSeq   = 0;

static void openTagFile(uint8_t tag) {
    char name[40];
    if (g_rotateSeq[tag] == 0) {
        snprintf(name, sizeof(name), "/%s_%04u.csv", baseNameForTag(tag), g_session);
    } else {
        // Letter suffix (_a, _b, ...), not a number — see BRIDGE_LOG_ROTATE_MAX_SEQ.
        const char suffix = 'a' + ((g_rotateSeq[tag] - 1) % BRIDGE_LOG_ROTATE_MAX_SEQ);
        snprintf(name, sizeof(name), "/%s_%04u_%c.csv", baseNameForTag(tag), g_session, suffix);
    }
    g_files[tag] = SD_MMC.open(name, FILE_WRITE);
    if (g_files[tag]) {
        g_files[tag].println(headerForTag(tag));
        g_files[tag].flush();
        g_headerWritten[tag] = true;
        g_fileBytes[tag] = 0;
        Serial.printf("📝 SD: logging → %s\n", name);
    } else {
        Serial.printf("❌ SD: cannot open %s\n", name);
    }
}

static void writeLine(const LogLine& ln) {
    const uint8_t tag = ln.tag;
    if (tag >= TAG_COUNT) return;
    if (!g_files[tag] || !g_headerWritten[tag]) openTagFile(tag);
    if (!g_files[tag]) return;

    const size_t n = g_files[tag].println(ln.text);
    g_fileBytes[tag] += n;

    if (++g_writeCount[tag] >= 50) {           // bounded data loss on power-off
        g_files[tag].flush();
        g_writeCount[tag] = 0;
    }
    // Rotate every tag by size so a long session stays manageable and gps/sat/status
    // never approach FAT32's single-file cap on a multi-week deployment. Clamped at the
    // 'z' suffix: past BRIDGE_LOG_ROTATE_MAX_SEQ rotations we stop advancing the letter
    // and just keep appending to the 'z' file, so an extreme session never wraps into a
    // colliding filename or silently overwrites an earlier rotation.
    if (BRIDGE_LOG_ROTATE_BYTES > 0 && g_fileBytes[tag] >= BRIDGE_LOG_ROTATE_BYTES &&
        g_rotateSeq[tag] < BRIDGE_LOG_ROTATE_MAX_SEQ) {
        g_files[tag].flush();
        g_files[tag].close();
        g_rotateSeq[tag]++;
        g_headerWritten[tag] = false;
        openTagFile(tag);
    }
}

static void openRawFile() {
    char name[40];
    if (g_rawSeq == 0) {
        snprintf(name, sizeof(name), "/rtcm_%04u.bin", g_session);
    } else {
        const char suffix = 'a' + ((g_rawSeq - 1) % BRIDGE_LOG_ROTATE_MAX_SEQ);
        snprintf(name, sizeof(name), "/rtcm_%04u_%c.bin", g_session, suffix);
    }
    g_rawFile = SD_MMC.open(name, FILE_WRITE);
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
    bool wroteAny = false;
    while ((got = xStreamBufferReceive(g_rawStream, buf, sizeof(buf), 0)) > 0) {
        if (!g_rawFile) openRawFile();
        if (!g_rawFile) return;
        g_rawFile.write(buf, got);
        g_rawBytes += got;
        wroteAny = true;
        // PERIODIC FLUSH (bug fix): without this the .bin's directory entry/size is
        // only committed on the 64 MB rotation, so a card pulled mid-session — or
        // simply inspected — reads 0 bytes despite data being buffered. Flush every
        // ~16 chunks (~8 KB) to bound loss and keep the on-card size truthful, mirroring
        // the CSV tags' 50-write flush cadence.
        if (++g_rawWritesSinceFlush >= 16) {
            g_rawFile.flush();
            g_rawWritesSinceFlush = 0;
            g_rawLastFlushMs = millis();
        }
        if (BRIDGE_LOG_ROTATE_BYTES > 0 && g_rawBytes >= BRIDGE_LOG_ROTATE_BYTES &&
            g_rawSeq < BRIDGE_LOG_ROTATE_MAX_SEQ) {
            g_rawFile.flush();
            g_rawFile.close();
            g_rawSeq++;
            openRawFile();
        }
    }
    // Commit a low-rate stream (the normal ~400 B/s base) without waiting for 16
    // chunks — but on a TIMER, not on every pass.
    // DO NOT flush unconditionally here. This task drains every ~20 ms, so a per-pass
    // f_sync issues up to 50 FAT + directory writes PER SECOND. That keeps the SDMMC
    // bus and its Core-0 DMA busy essentially continuously (Core 0 is also where the
    // WiFi driver and lwIP tasks live), and it throttles this task's own queue drain
    // badly enough that status rows get dropped on a full queue. A 1 s bound gives the
    // same worst-case data loss at ~1/50th the write amplification.
    const uint32_t nowMs = millis();
    if (wroteAny && g_rawFile && g_rawWritesSinceFlush > 0 &&
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
    for (uint8_t i = 0; i < (sizeof(kAttempts)/sizeof(kAttempts[0])) && !mounted; ++i) {
        if (SD_MMC.begin("/sdcard", true, false, kAttempts[i].freqKhz, 12)) {
            mounted = true;
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
        vTaskDelete(nullptr);
        return;
    }
    const uint64_t freeMB = (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / 1048576ULL;
    // Cast explicitly rather than relying on uint64_t's underlying type. It is
    // "unsigned long long" on the ESP32 and "unsigned long" on a 64-bit host, so an
    // uncast %llu is correct in one place and a mismatch in the other — which is enough
    // to make the off-target check either miss real format bugs or reject this one.
    Serial.printf("✅ SD_MMC: ready, %llu MB free (session %04u)\n",
                  (unsigned long long)freeMB, g_session);
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
        if (g_downloadActive) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

        if (xQueueReceive(g_queue, &ln, pdMS_TO_TICKS(20)) == pdTRUE) {
            writeLine(ln);
        }
        drainRawStream();
    }
}

// ── Public API ───────────────────────────────────────────────────────────────
void bridge_sdlog_init() {
    // Monotonic session id + persisted channel toggles in NVS.
    Preferences p;
    p.begin("bridgelog", false);
    g_session = p.getUShort("session", 0) + 1;
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

    g_queue = xQueueCreate(BRIDGE_LOG_QUEUE_DEPTH, sizeof(LogLine));
    if (!g_queue) { Serial.println("❌ SD: log queue alloc failed"); return; }
    // Always allocate the raw stream so raw capture can be toggled on at runtime.
    g_rawStream = xStreamBufferCreate(16384, 1);
    if (!g_rawStream) Serial.println("⚠️ SD: raw stream alloc failed (raw RTCM unavailable)");

    // Pin to Core 0 so SD latency never lands on the Core-1 GNSS/caster loop.
    // Stack = 10240: SD_MMC.begin()/FATFS plus the 353-byte LogLine that lives on this
    // task's stack make 6144 marginal. 10240 is the value proven stable on the RCX
    // RaceCapture rover's equivalent SD task — match it rather than run close to the edge.
    xTaskCreatePinnedToCore(sdTask, "bridgeSdLog", 10240, nullptr, 1, nullptr, 0);
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
        char line[240];
        // ms,utc_time,fix,fix_name,sats,hdop,lat_deg,lon_deg,alt_m,epe_3d_m,epe_2d_m,
        // ecef_x_m,ecef_y_m,ecef_z_m,svin_valid,svin_obs_s,svin_target_s,svin_meanacc_m,base_ready
        snprintf(line, sizeof(line),
                 "%lu,%s,%d,%s,%d,%.1f,%.9f,%.9f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%lu,%lu,%s,%d",
                 (unsigned long)now, dt, s.fixQuality, fixQualityName(s.fixQuality), s.satellites, s.hdop,
                 s.lat, s.lon, s.altM, s.epeM, s.epe2dM, s.ecefX, s.ecefY, s.ecefZ,
                 (unsigned)s.svinValid, (unsigned long)s.svinObs, (unsigned long)s.svinTarget,
                 accField, s.baseReady ? 1 : 0);
        enqueueText(TAG_GPS, line);
    }

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
        // c0_host..c0_err, c1_host..c1_err, lat_deg,lon_deg,acc_m,esp_temp_f,imu_temp_f
        snprintf(line, sizeof(line),
                 "%lu,%s,%lu,%d,%s,%s,%d,%d,%d,%u,%u,%lu,%s,%d,%lu,%lu,%llu,"
                 "%s,%s,%d,%s,%d,%llu,%llu,%lu,%ld,%s,"
                 "%s,%s,%d,%s,%d,%llu,%llu,%lu,%ld,%s,"
                 "%.9f,%.9f,%.3f,%s,%s,%lu,%lu,%lu,%lu,%lu,%lu,%s,%s,%lu,%lu,%lu,%u,%lu,%lu,"
                 "%s,%lu,%s,%s,%s,%s,%s",
                 (unsigned long)now, dt,
                 (unsigned long)(now / 1000UL),
                 s.wifiConnected ? 1 : 0, s.ssid, s.ip,
                 s.fixQuality, s.satellites, s.baseReady ? 1 : 0,
                 (unsigned)s.svinValid, (unsigned)s.svinMode,
                 (unsigned long)s.svinObs, accField, s.hotStart ? 1 : 0,
                 (unsigned long)s.rtcmFps, (unsigned long)s.rtcmBps,
                 (unsigned long long)s.rtcmValidFrames,
                 s.caster[0].host, s.caster[0].mount, s.caster[0].enabled ? 1 : 0,
                 s.caster[0].state, s.caster[0].handshake ? 1 : 0,
                 (unsigned long long)s.caster[0].framesWritten,
                 (unsigned long long)s.caster[0].bytesAccepted,
                 (unsigned long)s.caster[0].dropped, (long)s.caster[0].lastWriteAgeS,
                 s.caster[0].err,
                 s.caster[1].host, s.caster[1].mount, s.caster[1].enabled ? 1 : 0,
                 s.caster[1].state, s.caster[1].handshake ? 1 : 0,
                 (unsigned long long)s.caster[1].framesWritten,
                 (unsigned long long)s.caster[1].bytesAccepted,
                 (unsigned long)s.caster[1].dropped, (long)s.caster[1].lastWriteAgeS,
                 s.caster[1].err,
                 s.lat, s.lon, s.epeM, s.posSource, posAcc,
                 (unsigned long)s.loopMaxMs, (unsigned long)s.uartHighWater,
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
                 s.pcState, (unsigned long)s.pcFixes,
                 pcLat, pcLon, pcEpe, pcDist, pcThresh);
        enqueueText(TAG_STATUS, line);
    }
}

void bridge_sdlog_feed_nmea(const char* s) {
    if (!g_enSat || !s) return;
    const char* p = s;
    if (*p == '$') ++p;
    if (strlen(p) < 6) return;
    if (strncmp(p + 2, "GSV", 3) != 0) return;     // only GSV carries per-sat data

    const uint32_t now = millis();
    // Capture one ~300 ms window (a full multi-constellation GSV burst) per period.
    if (!g_satCapturing) {
        if (now - g_lastSatMs < BRIDGE_LOG_SAT_PERIOD_MS) return;
        g_satCapturing = true;
        g_satWindowStart = now;
        g_lastSatMs = now;
    } else if (now - g_satWindowStart > 300) {
        g_satCapturing = false;
        return;
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
    // f[0]=xxGSV f[1]=numMsg f[2]=msgNum f[3]=inView, then groups of 4.
    for (int g = 0; g < 4; ++g) {
        const int base = 4 + g * 4;
        if ((size_t)(base + 3) >= n) break;
        if (f[base][0] == '\0') continue;            // empty group = end of list
        char dt[32]; fmtDatetime(dt, sizeof(dt), g_lastDate, g_lastUtc);
        char line[120];
        snprintf(line, sizeof(line), "%lu,%s,%s,%s,%s,%s,%s",
                 (unsigned long)now,
                 dt,
                 cons, f[base], f[base + 1], f[base + 2], f[base + 3]);
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
    if (g_enRaw && g_rawStream && frame && len) {
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

void bridge_sdlog_event(const char* level, const char* msg) {
    if (!g_enEvent || msg == nullptr) return;
    char dt[32]; fmtDatetime(dt, sizeof(dt), g_lastDate, g_lastUtc);
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

uint32_t bridge_sdlog_dropped() { return g_dropped; }
bool     bridge_sdlog_ready()   { return g_sdReady; }
uint16_t bridge_sdlog_session() { return g_session; }
void     bridge_sdlog_set_download_active(bool a) { g_downloadActive = a; }

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
void     bridge_sdlog_event(const char*, const char*) {}
bool     bridge_sdlog_ready()   { return false; }
uint16_t bridge_sdlog_session() { return 0; }
void     bridge_sdlog_set_download_active(bool) {}
bool     bridge_sdlog_get_channel(int) { return false; }
void     bridge_sdlog_set_channel(int, bool) {}

#endif // BRIDGE_LOG_SD_ENABLE
