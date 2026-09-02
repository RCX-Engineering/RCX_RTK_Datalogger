#pragma once

// ================================================================
// ESP32-S3 LCD XBee Bridge - Base Station Configuration
// ================================================================
// Test caster connectivity from a terminal — substitute a configured caster's
// own host, mountpoint, and password:
// curl.exe -v -H "User-Agent: NTRIP Client/1.0" -H "Ntrip-Version: Ntrip/2.0" --user "MOUNT:PASSWORD" http://HOST:2101/MOUNT --output test_stream.bin

// Each unit generates its own name on first boot (from its MAC, so multiple
// units never collide) and stores it in NVS; it doubles as the fallback AP's
// SSID and is renamable from the web dashboard. See loadDeviceName().
//
// The fallback AP is deliberately OPEN and has no compile-time password. It exists so
// the dashboard stays reachable when no configured WiFi is in range — a recovery path,
// which a password the operator cannot look up would defeat. Anyone wanting the AP
// secured sets a password from the dashboard, which is stored in NVS.

// Draw LCD text twice, one pixel apart, so every glyph stem is 2 px instead of 1.
// Thin single-pixel strokes are the first thing lost in direct sunlight. The fonts are
// not anti-aliased, so this thickens the glyphs without blurring them. Set false for the
// original hairline text. See bprint().
static constexpr bool LCD_BOLD_TEXT = true;

// ── WiFi Networks ─────────────────────────────────────────────────────────────
// Configured from the web dashboard's WiFi Networks card and stored in NVS.

// ── Outbound NTRIP Source Casters (servers we push RTCM TO) ──────────────────
// Configured from the web dashboard's Casters card and stored in NVS.

// ── LG290P Base Configuration ────────────────────────────────────────────────
static constexpr uint32_t DEFAULT_SURVEY_IN_SEC = 900;
static constexpr float DEFAULT_SURVEY_IN_ACC_LIMIT_M = 3.20f;
static constexpr uint16_t LG290P_REFERENCE_STATION_ID = 290;

// Set to 1 to send LG290P base-mode + survey-in + RTCM configuration at boot.
// The firmware sends documented PQTM commands and computes NMEA checksums.
static constexpr bool LG290P_CONFIGURE_ON_BOOT = true;

// Set to true only after entering accurate externally-surveyed WGS84 ECEF meters.
static constexpr bool LG290P_USE_FIXED_BASE = false;

// Survey method for establishing a new base coordinate.
//   true  — PPP rover-mode survey (ppp_survey.*): the receiver runs as a rover with
//           Galileo E6 HAS PPP, we average only fixes that pass our own EPE gate, then
//           lock the average in as the fixed base. Better absolute accuracy, and the
//           acceptance criteria are ours to change live.
//   false — the module's internal survey-in (PQTMCFGSVIN mode 1), whose averaging and
//           completion criteria are opaque and cannot be retargeted without restarting.
// There is no fallback between the two, and none is needed: with PPP enabled the rover
// always returns the best solution the available corrections support, degrading on its
// own to an autonomous 3D fix when HAS data is weak. A survey therefore always yields a
// coordinate — the PPP-converged average when it converged, the unfiltered autonomous
// average when it did not — and the saved position records which, so the accuracy is
// never overstated.
//
// The PPP path drives ppp_survey.cpp, whose command sequence is the field-proven one
// from the 7-25-26 build and must not be "corrected" against the spec again: the
// receiver-mode switches there apply live, and PQTMEPE takes no MsgVer in that sequence.
// Setting this false falls back to the module's own internal survey-in.
static constexpr bool BASE_SURVEY_USE_PPP = true;

// ── ONE-SHOT RECOVERY: wipe a stuck base position at boot ────────────────────
// Set true, flash ONCE, let it boot, then set false and flash again.
//
// WHY THIS EXISTS: a saved base position lives in TWO independent non-volatile stores,
// and reflashing the sketch clears NEITHER of them:
//   1. The ESP32's NVS ("rcx1pos"). An Arduino IDE upload only rewrites the program
//      partition, so NVS survives. Only "Erase All Flash Before Sketch Upload" (Tools
//      menu) clears it.
//   2. The LG290P's OWN retained configuration — svinMode 2 plus the fixed ECEF, written
//      by PQTMSAVEPAR. Nothing done to the ESP32 touches this, and it survives erase and
//      reflash, which is why a base can keep publishing an old coordinate even with the
//      ESP32 side completely blank. This store is BATTERY-BACKED: field-proven, clearing a
//      stuck fixed position required disconnecting the LG290P backup battery AND all power.
//      A command-driven escape (rover + survey targets + PQTMSAVEPAR + restart) can fail
//      to take while every command still appears to have been sent, so the firmware now
//      READS BACK the receiver/SVIN mode after every escape instead of assuming it worked
//      (verifyEscapedFixedBase) — do not reintroduce an assumed-success path.
//
// When true, boot erases (1) and commands (2) back into survey-in mode before any other
// GNSS configuration runs — the only thing that clears both together.
static constexpr bool FORCE_CLEAR_POSITION_ON_BOOT = false;

// ── GNSS satellite gating (elevation / C-N0 masks) ───────────────────────────
// These are the module's POSITION-ENGINE masks, written with PQTMCFGELETHD and
// PQTMCFGCNRTHD (the same commands the RCX rover uses). They persist in the module's
// own NVM, so a module previously configured as a rover KEEPS the rover's tight masks
// until something overwrites them — which is why this base was seeing only ~10
// satellites, none below 30 degrees and none under 32 dB-Hz. The base never wrote these
// values; it inherited them.
//
// A BASE WANTS THE OPPOSITE OF A ROVER. The rover masks aggressively because a moving
// car gets multipath off fences and grandstands, and a bad observation there corrupts a
// position that matters instantly. A base is stationary with a clear-ish sky, averages
// over minutes to hours, and its job is to hand the rover as MANY observations as
// possible — the rover applies its own masks. Gating hard at the base discards data the
// rover could have used and starves the survey.
//
// 5 degrees matches the module's own factory default; 10 dB-Hz keeps only signals too
// weak to carry a usable observation.
static constexpr float BASE_ELEVATION_MASK_DEG = 5.0f;
static constexpr float BASE_CNR_MASK_DBHZ      = 10.0f;
static constexpr double LG290P_FIXED_ECEF_X_M = 0.0;
static constexpr double LG290P_FIXED_ECEF_Y_M = 0.0;
static constexpr double LG290P_FIXED_ECEF_Z_M = 0.0;

// How long a PQTMCFGRCVRMODE reading stays trustworthy. The receiver's mode is changed by
// writes that produce no reply, so the cached probe value has to expire or it becomes an
// assertion about the past. Generous: the only consumer uses it to pick a better wording
// for a fault that other conditions detect independently.
static constexpr uint32_t PROBE_MODE_TRUST_MS = 120000;   // 2 minutes

// ── PPP survey recovery ──────────────────────────────────────────────────────
// How long to wait before retrying a survey that failed with no usable fix. Long enough
// that a receiver with genuinely no sky cannot spin on it, short enough that a base which
// was simply started too early recovers on its own well inside a session.
static constexpr uint32_t PPP_RETRY_BACKOFF_MS = 60000;   // 1 minute

// ── Internal heap watchdog ───────────────────────────────────────────────────
// DMA-capable internal RAM is shared by the SD driver, lwIP and the WiFi stack, and
// PSRAM cannot substitute for any of them. Below this the first failures appear — an SD
// write returning ESP_ERR_NO_MEM, a socket that cannot be allocated — so the warning
// fires while there is still room to act on it.
static constexpr uint32_t HEAP_INTERNAL_WARN_BYTES = 40000;
static constexpr uint32_t HEAP_WARN_INTERVAL_MS    = 60000;

// ── Base output watchdog ─────────────────────────────────────────────────────
// How long to wait between attempts to repair a base that is publishing descriptors but
// no MSM observations (see updateBaseOutputWatchdog). Long enough that a receiver which
// genuinely cannot produce observations is not flooded with configuration writes, short
// enough that a base does not spend a whole session publishing nothing usable.
static constexpr uint32_t BASE_OUTPUT_REPAIR_INTERVAL_MS = 120000;   // 2 minutes

// ── Device identity ───────────────────────────────────────────────────────────
// The device name doubles as the fallback AP's SSID, so it inherits 802.11's 31-character
// SSID ceiling. Enforced in setDeviceName() and mirrored by the dashboard's Rename field.
static constexpr size_t DEVICE_NAME_MAX_CHARS = 31;

// ── Correction quality gate (what must be true before we publish RTCM) ───────
// Being "ready" and being "worth listening to" are different questions. baseReady says
// the survey converged; these say the stream leaving this device right now is fit for a
// rover to trust. A base that publishes nothing is an inconvenience; a base that
// publishes corrections referenced to a wrong or unconverged coordinate silently drags
// every connected rover off position, and the rover has no way to tell.
//
// Each condition below is evaluated continuously and must hold for CAST_QUALITY_HOLD_MS
// before the stream is cut, and again for CAST_QUALITY_RECOVER_MS before it resumes.
// The hysteresis is not cosmetic: dropping a caster socket costs a full reconnect and,
// on rtk2go, counts against an abuse threshold that earns multi-hour IP bans, so a
// single late frame must never tear a session down.
//
// CAST_QUALITY_HOLD_MS exceeds the 10 s cadence of the slowest configured message
// (1005/1007/1033) with margin, so a normal inter-frame gap cannot trip it.
static constexpr uint32_t CAST_QUALITY_HOLD_MS    = 20000;
static constexpr uint32_t CAST_QUALITY_RECOVER_MS = 5000;

// Maximum share of framed RTCM candidates allowed to fail CRC or framing, measured over
// a rolling window. Above this the UART is losing bytes and the frames still getting
// through cannot be trusted to be intact. Sessions 0457/0458 ran at zero failures for
// 34 h, so anything sustained above a fraction of a percent is a real fault, not noise.
static constexpr float    CAST_QUALITY_MAX_CRC_FAIL_RATE = 0.02f;   // 2%
static constexpr uint32_t CAST_QUALITY_CRC_WINDOW_MS     = 30000;
// Minimum candidates in the window before the rate is meaningful — below this a single
// failure would read as a 100% failure rate.
static constexpr uint32_t CAST_QUALITY_CRC_MIN_SAMPLES   = 50;

// Ceiling on the reference coordinate's own accuracy. A survey that "completed" at a
// worse figure than this is not a base worth publishing, whatever the module says.
static constexpr float    CAST_QUALITY_MAX_POS_ACC_M = 5.0f;

// ── Hardware Pins ─────────────────────────────────────────────────────────────
static constexpr int PIN_GNSS_RX = 4;
static constexpr int PIN_GNSS_TX = 5;
static constexpr uint32_t GNSS_BAUD = 460800;
static constexpr int PIN_LCD_BL = 46;

// ── System Limits ─────────────────────────────────────────────────────────────
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;
// BAN-SAFETY: rtk2go documents that its abuse thresholds are set LOW for NTRIP
// Server (push-in) connections, that retry rates must be >= 10 s, and that
// repeatedly reconnecting with rejected credentials earns IP bans of ~3 hours
// to several weeks (worst case blocked at their firewall — which then looks
// like a plain TCP connect timeout to EVERY device on this network, including
// the rover's NTRIP client). The old 5 s flat retry was squarely in ban
// territory whenever a push was being rejected. 30 s base + exponential
// backoff (doubling per consecutive failure, capped below) stays well clear.
static constexpr uint32_t NTRIP_RECONNECT_INTERVAL_MS = 30000;
static constexpr uint32_t NTRIP_RECONNECT_MAX_MS = 600000;   // 10 min cap
static constexpr uint32_t NTRIP_CONNECT_TIMEOUT_MS = 1200;
static constexpr uint32_t NTRIP_RESPONSE_TIMEOUT_MS = 5000;
