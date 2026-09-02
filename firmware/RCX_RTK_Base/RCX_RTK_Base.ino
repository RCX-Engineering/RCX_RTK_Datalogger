// =============================================================================
//  RCX1 Caster — DIY mobile NTRIP base station / RTCM source uploader
// =============================================================================
//  This is a DIY mobile NTRIP Caster intended to provide RTK data for SCCA
//  autocross events. Autocross courses change every event, but run-to-run
//  comparisons are important. Highest priority is for precision and consistency.
//  Geographic accuracy is desired, but not necessary.
//  Retain consistency: Do not arbitrarily change any displays or names or web
//  output unless requested.
//
// -----------------------------------------------------------------------------
//  ORIENTATION FOR FUTURE MAINTAINERS (human or AI) — READ BEFORE EDITING
// -----------------------------------------------------------------------------
//  This header documents intent, structure, and the rationale behind decisions
//  that are NOT obvious from the code and that have already been broken once and
//  fixed. Inline comments throughout carry the same caution markers ("do not
//  remove", "BUGFIX", "FIRMWARE-CONFIRMED", "SPEC-PINNED"). Treat all of those as
//  load-bearing: they record a concrete failure that the current code avoids.
//
//  HARDWARE
//    • Board: Waveshare ESP32-S3-LCD-1.47B (ESP32-S3R8, 8 MB octal PSRAM).
//    • GNSS:  Quectel LG290P RTK module on UART1 @ 460800 baud (config.h pins).
//             Firmware verified against LG290P&LGx80P Protocol Spec v1.1 and the
//             on-module version LG290P03AANR01A06S (seen in PQTMVER at boot).
//    • Display: ST7789 172x320 via TFT_eSPI (project-local setup header).
//    • SD: SD_MMC 1-bit (see bridge_sd_log.* for the pin lock-down rationale).
//    • This same PCB is ALSO flashed as the "RCX RaceCapture" ROVER firmware. The
//      two firmwares deliberately SHARE one NVS namespace (rcx_wifi) and otherwise
//      keep strictly separate namespaces so reflashing base<->rover never corrupts
//      or collides. See the NVS MAP below — do not rename namespaces.
//
//  WHAT THIS DEVICE DOES (data flow)
//    LG290P (base mode) emits RTCM3 corrections + PQTM/NMEA status on one UART.
//    loop() on Core 1 drains that UART, frames+CRC-validates RTCM, and pushes each
//    valid frame to every ENABLED caster (NTRIP server/SOURCE upload) over TCP.
//    Survey status / position / fix come from PQTM messages parsed alongside.
//    A Core-0 FreeRTOS task (in bridge_sd_log.cpp) owns all SD writes so FAT
//    cluster-allocation stalls never block the real-time UART drain on Core 1.
//
//  CONCURRENCY MODEL (why it's safe)
//    • Core 1: Arduino loop() — GNSS UART drain, RTCM framing, caster TCP I/O,
//      web server, display. Everything latency-sensitive and RTCM-real-time.
//    • Core 0: SD logger task only (bridge_sd_log.cpp). Fed by a FreeRTOS queue;
//      loop() ENQUEUES, never writes the card directly. This is the single most
//      important structural rule for log integrity — do not call SD writes from
//      loop()/the RTCM parser.
//
//  BOOT SEQUENCE & THE HOT-START PROBE (configureLg290pBaseOnce / lg290pBaseConfigMatches)
//    The LG290P retains its config, ephemeris, and (once converged) position across
//    an ESP reset because the module is separately powered. A full reconfigure ends
//    in PQTMSAVEPAR + PQTMSRR (module reset), which WIPES that warm state and forces
//    a fresh multi-minute survey-in. So on boot we PROBE first (PQTMCFGRCVRMODE,R +
//    PQTMCFGSVIN,R) and, if the module is already a correctly-configured base, we
//    SKIP the reconfigure entirely ("hot start"). Critical, already-broken-once rules:
//      - The probe must wait for the QUERY REPLIES it needs (g_probedRcvrMode AND
//        g_probedSvinMode populated), NOT merely for any UART traffic. Autonomous
//        1 Hz PQTMSVINSTATUS arrives constantly and will trip a naive early-exit
//        before the CFGSVIN,R reply is parsed → false "unconfigured" → needless
//        reset. (Field bug 2026-06-24; see the loop in lg290pBaseConfigMatches.)
//      - Message-rate config for base-only outputs (PQTMSVINSTATUS, RTCM) must be
//        sent AFTER PQTMCFGRCVRMODE,W,2. PQTMSVINSTATUS in particular is base-mode-
//        only and returns ERROR,3 (unsupported command) if enabled before base mode
//        is set (spec §2.3.23). The boot order encodes this — do not reorder.
//      - PQTM message-rate commands REQUIRE a trailing <MsgVer> (…,W,<name>,<rate>,
//        <msgver>); omitting it returns ERROR,1 and the message stays silent. NMEA
//        messages (GGA/GSV/RMC) take NO MsgVer. Both forms are FIRMWARE-CONFIRMED.
//      - PQTMCFGMSGRATE accepts only EXPLICIT RTCM numbers (RTCM3-1005 etc.); the
//        wildcard forms return ERROR,1 and were removed. MSM4 observations are
//        enabled by PQTMCFGRTCM,W,4 (NOT per-message MSGRATE).
//
//  POSITION PERSISTENCE & THE CONFIDENCE CHECK (the "did it move?" decision)
//    On survey-in completion (PQTMSVINSTATUS valid==2) the converged ECEF mean is
//    converted to lat/lon (ecefToLla, Bowring) and AUTO-SAVED to NVS (rcx1pos),
//    tagged with the survey DURATION and achieved accuracy. This is what lets the
//    NEXT boot decide whether the base moved:
//      - On boot with a saved position, checkPositionDisplacement() accumulates
//        PQTMNAV fixes and compares the mean against the saved position using
//        σ_combined = sqrt((mean_EPE/√N)² + saved_hacc²); threshold = clamp(2σ,
//        FLOOR=3 m, CAP=50 m). dist < threshold → CONFIRMED (reuse fixed pos);
//        dist > threshold → MOVED (clear NVS, re-survey).
//      - The 3 m FLOOR is deliberate: autonomous (uncorrected) fixes have a noise
//        floor ~0.3–1 m, so a sub-meter saved position can NEVER be confirmed to
//        sub-meter against autonomous fixes. We'd rather re-survey than ever cast a
//        wrong location, so the threshold never drops below 3 m.
//      - The confirmation WINDOW is tiered by survey duration (posCheckWindowMsForSurvey:
//        <1 h→30 s, 1–6 h→120 s, >6 h→300 s) because a longer survey is a more
//        valuable reference worth more confirmation effort.
//      - The collection clock does NOT start until a real fix exists (PQTMNAV
//        SolType>0 and non-zero lat/lon). Antenna disconnected → it waits forever
//        rather than timing out into a needless re-survey (deliberate: lets you
//        move the box without moving the antenna).
//      - A completed survey leaves the MODULE in svinMode==1 (survey) config, not
//        svinMode==2 (fixed) — the LG290P reports valid==2 when converged but does
//        not rewrite its own SVIN mode. So a hot-skipped, surveyed module probes as
//        mode 1; on CONFIRMED we apply the saved position as a fixed base via
//        g_reconfigPending. This is expected, not a bug.
//
//  CASTING / READINESS GATE (updateBaseReadiness)
//    Casting is gated on AUTHORITATIVE survey completion: PQTMSVINSTATUS <Valid>==2
//    AND meanAcc within the configured limit. It is NOT gated on MSM/1005 presence —
//    the LG290P emits MSM continuously in base mode using its RUNNING (unconverged)
//    survey estimate, so gating on MSM streamed corrections from an unconverged
//    position (the field-observed "casting before survey finished" bug). MSM
//    presence is used ONLY for the dashboard "corrections flowing" light.
//
//  NVS NAMESPACE MAP (do not rename — shared hardware, base<->rover reflash safety)
//    rcx_wifi   — WiFi credentials. SHARED with the RCX RaceCapture rover so creds
//                 survive a base<->rover reflash. Keys: n, s0..s9, p0..p9.
//    rcx1pos    — Saved base position (base only). valid/lat/lon/alt/hacc/src/svsec.
//    rcx1cast   — User-added casters + per-caster enable flags (base only).
//    xbee       — Survey-in config: survey_sec, survey_acc (base only, legacy name).
//    bridgelog  — SD log channel enable flags (base only; see bridge_sd_log.cpp).
//    (Rover-only namespaces rcx_veh / rcx_log / rcx_ntrip are NOT touched here.)
//
//  CASTER MODEL (NtripTarget casters[MAX_CASTERS], runtime casterCount)
//    Fixed-size array, never resized (WiFiClient members aren't safely movable);
//    casterCount tracks how many slots are live. Casters are added from the
//    dashboard and loaded from NVS at boot. host/mountpoint/password are OWNED
//    char[] (not const char* into config.h) so each added caster has its own
//    storage. The detailed SD log keeps two caster status columns (slots 0/1);
//    extra casters still stream.
//
//  BAN SAFETY
//    rtk2go documents low NTRIP-server abuse thresholds and >=10 s retry minimums;
//    repeated rejected pushes earn multi-hour/day IP bans (which then look like a
//    plain TCP timeout to EVERY device on the network, including a rover's client).
//    Hence the 30 s base + exponential backoff reconnect floor and the auth-stall
//    watchdog. Do not lower these for rtk2go or any unknown caster. See config.h.
//
//  COPYRIGHT/IP: none — this is original project code.
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <stdarg.h>          // va_list for logEvent()'s printf-style forwarding
#include <strings.h>         // strncasecmp — newlib declares it here, not in <string.h>
#include <vector>
#include "FS.h"
#include "SD_MMC.h"
#include "esp_heap_caps.h"   // heap_caps_* for the boot RAM diagnostic
#include "esp_mac.h"         // esp_read_mac() — per-device name generation, no WiFi radio needed
#include <lwip/sockets.h>    // select()/fd_set for the non-blocking caster write gate
#include <Wire.h>            // I2C for the QMI8658 board-temperature read
#include "config.h"
#include "bridge_sd_log.h"
#include "bridge_web_page.h"
#include "ppp_web.h"          // PPP manual survey-in (rover PPP → fixed base) + /ppp dashboard

// ================================================================
// ESP32-S3 LCD XBee Bridge - RTCM-validated NTRIP Source Uploader
// ================================================================

// ── Limits ───────────────────────────────────────────────────────────────────
static constexpr size_t RTCM_MAX_PAYLOAD_LEN = 1023;
static constexpr size_t RTCM_MAX_FRAME_LEN = 3 + RTCM_MAX_PAYLOAD_LEN + 3;
static constexpr size_t SERIAL_DRAIN_LIMIT_PER_LOOP = 2048;
// Echo of the LG290P's PERIODIC telemetry ($PQTMPVT / $PQTMEPE / $PQTMSVINSTATUS /
// $PQTMPPPNAV, ~3 lines/s). Defaults FALSE for the USB CDC blocking reason above.
static constexpr bool SERIAL_GNSS_ECHO_ENABLE = false;
// Echo of outbound commands ("LG290P CMD: ...") and their config replies
// ("LG290P RSP: ..."). Defaults FALSE — same USB CDC blocking reason, and
// configureLg290pBaseOnce() can re-run this full sequence during normal
// operation (e.g. after a detected move), not just at boot. Set true for
// bench work with a terminal open.
static constexpr bool SERIAL_GNSS_CMD_ECHO_ENABLE = false;
// Debug ring buffers for the dashboard's raw-download endpoints. They are static arrays
// in internal RAM, which is the pool the SD driver and lwIP allocate their buffers from,
// and together they were holding 49 kB of it. The authoritative raw capture is the SD
// card's rtcm_*.bin, not these — these only need to hold enough recent traffic to answer
// "what is going out right now" from a browser. Sized down accordingly: 8 kB still holds
// roughly a minute of this base's output.
static constexpr size_t CASTER_TX_CAPTURE_SIZE = 4096;
static constexpr size_t RTCM_VALID_CAPTURE_SIZE = 8192;
static constexpr size_t WEB_DUMP_BYTES = 768;
static constexpr size_t NMEA_MAX_LEN = 180;
static constexpr uint32_t LG290P_COMMAND_SPACING_MS = 250;
static constexpr uint32_t LG290P_POST_RESET_WAIT_MS = 4500;
// Bound on the read-back that confirms an escape sequence actually left base/fixed mode
// (verifyEscapedFixedBase). A live module answers PQTMCFGRCVRMODE,R / PQTMCFGSVIN,R in
// well under a second; this only caps the silent-module case, where the verify
// deliberately resolves to "still fixed" rather than to an optimistic assumption.
static constexpr uint32_t LG290P_ESCAPE_VERIFY_MS = 3000;

// ── Robustness tunables (defined here so config.h stays untouched) ────────────
// Uplink stall watchdog: if a Streaming caster writes no frame for this long
// WHILE fresh RTCM is still being parsed, treat the socket as half-open and force
// a reconnect. 1 Hz MSM frames keep lastWriteMs fresh in normal operation, so
// 8 s is comfortably above the real inter-frame gap (the 0.1 Hz 1005 cadence is
// irrelevant — MSM dominates). Also gated on RTCM freshness so a receiver that
// stops producing corrections is never mistaken for a dead uplink.
static constexpr uint32_t NTRIP_UPLINK_STALL_MS = 8000;
// How long a caster may apply continuous backpressure before we stop calling it
// "congested" and treat it as dead. This is a DARK-TIME BUDGET, not just a timeout:
// for its whole duration we are deliberately NOT tearing down a socket that may in
// fact be half-open, so no corrections reach the rover. Longest legitimate congestion
// run observed in the field was ~15 s (session 0453); a rover typically holds RTK
// through ~30 s of correction age. 20 s clears the former with margin and stays under
// the latter. Do NOT raise this into the minutes — that trades a fast reconnect for
// silent dead air, which is worse than the teardown it was added to prevent.
static constexpr uint32_t NTRIP_CONGESTION_MAX_MS = 20000;
// ANTI-HAMMER: if a caster authenticates but we never manage to stream a single frame
// within this window (it accepts the mount, then silently drops or never reads our
// push), holding the socket and re-pushing every epoch just hammers it. Treat that as
// a dead uplink and drop it so the exponential reconnect backoff spaces out retries.
static constexpr uint32_t NTRIP_AUTH_STREAM_TIMEOUT_MS = 12000;

// AUTO-START readiness windows. RTCM_BASE_FRESH_MS must exceed the slowest base
// reference cadence (1005/1006 is configured at ~10 s) plus network/UART jitter,
// or a normal inter-frame gap will read as "base lost". BASE_READY_GRACE_MS is
// the hysteresis hold: once the base is confirmed ready we stay ready through
// brief gaps and only drop after a sustained absence (real survey loss / reset),
// so a single late frame can't tear down live caster sessions and flap reconnects.
static constexpr uint32_t RTCM_BASE_FRESH_MS = 25000;
static constexpr uint32_t BASE_READY_GRACE_MS = 60000;

// PQTMSVINSTATUS is a BASE-MODE-ONLY output. The moment the receiver is put into rover
// mode — a PPP survey, or the forced-rover escape in the boot move check — it stops
// arriving, and surveyStatus keeps holding whatever the module last said, which after a
// fixed hot start is valid==2. A cached value read as if it were current is what let a
// base report "survey-in complete" for a whole session while a fresh survey ran
// underneath it, and what let baseReady latch onto a coordinate that had already been
// discarded. Reads that drive behaviour or display go through svinValidNow(), which
// reports UNKNOWN once the last message is older than this. Sized well above the 1 Hz
// output cadence so ordinary jitter never trips it.
static constexpr uint32_t SVIN_STATUS_STALE_MS = 10000;

// ── Types ────────────────────────────────────────────────────────────────────
enum class CasterState : uint8_t {
    Disabled,
    Held,          // enabled by the operator, withheld by the readiness/quality gates
    WaitingForWifi,
    Connecting,
    AwaitingResponse,
    Authenticated,
    Streaming,
    Error
};

enum class CasterProtocol : uint8_t {
    NtripV1Source
};

struct CasterTxCapture {
    uint8_t data[CASTER_TX_CAPTURE_SIZE];
    size_t head = 0;
    size_t count = 0;
};

struct RtcmValidCapture {
    uint8_t data[RTCM_VALID_CAPTURE_SIZE];
    size_t head = 0;
    size_t count = 0;
};

// Declared here, defined below with the other RTCM type helpers.
static bool rtcmTypeEverSeen(uint16_t type);

// ── What a complete correction stream from this base contains ───────────────
// Every message enableLg290pBaseOutputs() asks the receiver for, plus the MSM4
// observation set that PQTMCFGRTCM enables. PQTMCFGMSGRATE is written without checking
// its acknowledgement at any of its call sites, so a message type the module declines to
// produce leaves no trace at all — a 75-minute capture of 19,050 frames contained not one
// 1007 despite 1007 being requested on every base entry. Comparing this list against the
// types actually decoded turns that silence into a visible gap, and does it without
// having to trust any per-command ack.
// 1007 is deliberately NOT expected. It carries one thing — the antenna descriptor
// string — and 1033 carries that same field plus the antenna serial and the receiver
// type, so 1033 supersedes it entirely (1007 subset of 1008 subset of 1033). More to the
// point, this module has never emitted a 1007: zero across 19,050 frames in one session
// and zero across 70,450 in another, on two different builds, while the request went out
// on every base entry. Decoding its 1033 explains why — the antenna descriptor field is
// zero-length, so a 1007 would have no payload at all. Listing it as expected produced a
// permanent "missing 1007" on the dashboard, and a completeness check that is always
// complaining is one that gets ignored. The real problem it was gesturing at — the empty
// descriptor — is now reported directly by the 1033 decode below.
static constexpr uint16_t EXPECTED_RTCM_TYPES[] = {
    1005,        // stationary antenna reference point
    1033,        // receiver and antenna descriptors (supersedes 1007/1008)
    1019, 1020, 1042, 1046,   // GPS / GLONASS / BeiDou / Galileo ephemeris
    1074, 1084, 1094, 1124,   // MSM4 observations, enabled via PQTMCFGRTCM
    1230         // GLONASS code-phase biases
};

struct RtcmTypeStat {
    uint16_t type = 0;
    uint32_t count = 0;
    uint32_t lastSeenMs = 0;
};

struct NtripTarget {
    // OWNED storage (was const char* into config.h literals). Owning the strings is
    // what makes runtime add/remove possible — a caster added via the web UI has no
    // compile-time literal to point at. Sized generously for NTRIP hostnames/mounts.
    char host[64] = {0};
    uint16_t port = 2101;
    char mountpoint[40] = {0};
    char password[40] = {0};
    CasterProtocol protocol = CasterProtocol::NtripV1Source;
    // Operator enable/disable (persisted in NVS). A disabled caster never connects or
    // streams; it sits in CasterState::Disabled. New casters default enabled.
    bool enabled = true;
    WiFiClient client;
    CasterState state = CasterState::Disabled;
    uint32_t lastAttemptMs = 0;
    uint32_t stateStartMs = 0;
    bool tcpConnected = false;
    bool handshakeAccepted = false;
    bool wroteValidRtcm = false;
    // Bytes for THIS connection only. bytesOffered/bytesAccepted are lifetime totals for
    // the dashboard, and mixing the two scales is what made the auth-stall watchdog lie:
    // it tested a lifetime counter against a per-connection flag, so any reconnect after
    // a session that had ever streamed reported "stalled after first write" no matter
    // what actually went wrong. A stall is about this attempt, so it is measured here.
    uint32_t connBytesOffered  = 0;
    uint32_t connBytesAccepted = 0;
    char responseHeader[1024] = {0};
    size_t responseLen = 0;
    char lastResponseHeader[1024] = {0};
    uint64_t bytesOffered = 0;
    uint64_t bytesAccepted = 0;
    uint64_t framesWritten = 0;
    uint32_t partialWriteCount = 0;
    uint32_t failedWriteCount = 0;
    // Consecutive failed connect/handshake attempts since the last accepted
    // handshake. Drives the exponential reconnect backoff in serviceCaster() —
    // the ban-safety mechanism that keeps a persistently-rejected push (wrong
    // password, unregistered mountpoint, caster outage) from hammering the
    // caster at a fixed rate until the IP gets banned.
    uint8_t  failCount = 0;
    // ── Robustness additions ─────────────────────────────────────────────────
    // Frames dropped because the TCP send buffer couldn't take a WHOLE frame
    // (backpressure). This is congestion, NOT a fault — see sendFrameToCaster().
    uint32_t droppedWriteCount = 0;
    // Last time the send gate deliberately dropped a frame because the socket was
    // not writable. Distinguishes "peer is congested but ALIVE" from "socket is
    // half-open and dead" — the uplink watchdog needs that distinction. See
    // sendFrameToCaster() and the UPLINK WATCHDOG in serviceCaster().
    IPAddress resolvedIp;              // cached DNS result — see connectCaster()
    uint32_t lastBackpressureMs = 0;
    uint32_t congestionStartMs  = 0;   // start of the CURRENT unwritable run, 0 = writable
    // TX COALESCING BUFFER — see flushCasterTx(). RTCM frames are 30-250 B and arrive
    // in bursts; one write() per frame with TCP_NODELAY means one SEGMENT per frame.
    // 1024 B stays under a single 1460 B MSS, so a full buffer is always one segment.
    uint8_t  txBuf[1024];
    size_t   txLen        = 0;
    uint32_t txFirstMs    = 0;
    // millis() of the last fully-written frame. Feeds the uplink stall watchdog
    // in serviceCaster() that catches half-open sockets. 0 = nothing written yet.
    uint32_t lastWriteMs = 0;
    // Per-caster reconnect floor. Some services (e.g. rtk2go) REQUIRE a ban-safe
    // floor (>=10 s; we use 30 s + backoff), so every caster defaults to that
    // strict floor regardless of which service it turns out to be — it remains a
    // fully first-class path (often the primary one on the road), not a
    // deprioritized one.
    uint32_t reconnectBaseMs = NTRIP_RECONNECT_INTERVAL_MS;
    // ── Error reason (surfaced on the dashboard instead of a bare "Error") ────
    // Human-readable cause of the most recent error-stop. Set at each
    // stopCaster(...Error) site; persists through the backoff so the UI can keep
    // showing WHY the last attempt failed while it waits to retry.
    char lastError[56] = "";
    // ── Bandwidth tracking (uplink to this caster) ────────────────────────────
    // firstWriteMs: millis() of the very first accepted frame this power cycle —
    // anchors the lifetime average (bytesAccepted / elapsed). 0 = nothing sent yet.
    uint32_t firstWriteMs = 0;
    // Rolling 1 s window for the live throughput readout (bytes this second →
    // bps, refreshed once per second), mirroring how rtcmStats computes its rate.
    uint32_t bwWindowBytes = 0;   // bytes accumulated in the current 1 s window
    uint32_t bwWindowStartMs = 0; // start of the current window
    uint32_t bwBytesPerSec = 0;   // last completed window's byte count (live rate)
};

enum class RtcmParseState : uint8_t {
    WaitPreamble,
    ReadLen1,
    ReadLen2,
    ReadFrame
};

struct RtcmParser {
    RtcmParseState state = RtcmParseState::WaitPreamble;
    uint8_t frame[RTCM_MAX_FRAME_LEN];
    size_t index = 0;
    size_t expected = 0;
    uint16_t payloadLen = 0;
};

struct RtcmStats {
    uint64_t uartBytes = 0;
    uint64_t nmeaBytes = 0;
    uint32_t nmeaSentences = 0;
    uint32_t rtcmCandidateFrames = 0;
    uint32_t rtcmValidFrames = 0;
    uint32_t rtcmCrcFailures = 0;
    uint32_t rtcmFramingFailures = 0;
    uint64_t rtcmValidBytes = 0;
    uint16_t lastMsgType = 0;
    uint16_t lastMsgLength = 0;
    uint32_t lastValidMs = 0;
    uint32_t framesWindow = 0;
    uint32_t bytesWindow = 0;
    uint32_t framesPerSecond = 0;
    uint32_t bytesPerSecond = 0;
    RtcmTypeStat typeStats[32];
};

struct SurveyStatus {
    uint8_t valid = 0;
    uint32_t obs = 0;
    uint32_t cfgDur = 0;
    double meanX = 0.0;
    double meanY = 0.0;
    double meanZ = 0.0;
    double meanAcc = 0.0;
    uint32_t lastSeenMs = 0;
};

// ── Globals ──────────────────────────────────────────────────────────────────
HardwareSerial GnssSerial(1);
TFT_eSPI tft = TFT_eSPI();
WebServer server(80);
Preferences preferences;

// ── Device identity ───────────────────────────────────────────────────────────
// NVS namespace "rcx1id" — this unit's name. Doubles as the status "dev" field
// and the fallback AP's SSID. Resolved once at boot by loadDeviceName(): the
// stored name if one exists, otherwise generated from the chip's factory MAC
// (unique per unit, no two units default to the same name/SSID) and saved.
// Renamable from the dashboard; a new name takes effect on the next reboot.
static String g_deviceName;

// Caster array: fixed storage (never resized — WiFiClient members aren't safely
// movable), with a runtime count. Slots 0..casterCount-1 are live, loaded from
// NVS (see loadCasters()).
static constexpr int MAX_CASTERS = 6;
NtripTarget casters[MAX_CASTERS];
int casterCount = 0;   // set definitively by loadCasters() in setup()

CasterTxCapture casterTxCaptures[2];
RtcmValidCapture rtcmValidCapture;
RtcmParser rtcmParser;
RtcmStats rtcmStats;
SurveyStatus surveyStatus;

static int activeWifiIndex = -1;
static int wifiAttemptIndex = 0;
static uint32_t wifiAttemptStartMs = 0;

// ── Runtime WiFi network list ─────────────────────────────────────────────────
// NVS namespace "rcx_wifi" — SHARED with the RCX RaceCapture rover firmware.
// Both firmwares run on the same hardware; reflashing one should not lose WiFi
// credentials entered in the other. Keys: "n" (count), "s0".."s9" (SSIDs),
// "p0".."p9" (passwords). Max 10 stored networks, added from the web UI and
// deletable from there.
struct WifiEntry { String ssid; String password; };

// ── TYPES USED IN FUNCTION SIGNATURES MUST LIVE IN THIS BLOCK ────────────────
// The Arduino builder auto-generates prototypes for every function in the .ino and
// injects them at the LAST preprocessor directive that precedes the first function
// definition. Any struct or enum defined after that point does not exist yet when the
// injected prototypes are compiled, so every function taking or returning it fails with
// "does not name a type" — pointing at the definition, which looks correct in isolation.
// Keeping these declarations up here with the other shared types, above all the #ifndef
// config blocks below, is what makes them visible to the injected prototypes.

// Stoplight state for the LCD. The colours these map to, and the reasoning behind the
// filled-block rendering, live with the UI palette further down. Ordered so a plain
// numeric comparison yields worst-of: BAD > WARN > OK > MUTED.
enum UiStatus : uint8_t { ST_MUTED = 0, ST_OK = 1, ST_WARN = 2, ST_BAD = 3 };

// One connected local-NTRIP rover. Definition lives here for the prototype-injection
// reason above; the caster logic itself is further down with the other network code.
struct LocalCasterClient {
    WiFiClient client;
    bool       streaming     = false;   // header accepted, RTCM flowing
    uint32_t   connectedMs   = 0;
    uint32_t   framesWritten = 0;
    uint32_t   framesDropped = 0;
    uint32_t   congestedMs   = 0;       // 0 = writable last we checked; see the poll rate limit
    // A REAL NTRIP client request does not fit in 160 bytes, and this buffer filling was
    // treated as a protocol error and the client dropped. A typical one:
    //
    //   GET /RCX1 HTTP/1.1              ~20
    //   Host: 192.168.50.54:2101        ~26
    //   Ntrip-Version: Ntrip/2.0        ~26
    //   User-Agent: NTRIP <client>      ~35
    //   Authorization: Basic <base64>   ~40
    //   Connection: close               ~19
    //   <blank line>                      2
    //                                  ~168 bytes
    //
    // So a correctly-behaved rover sending a valid request filled the buffer before the
    // terminating blank line arrived, hit the oversize branch, and was disconnected with
    // no reply and no message. Every NTRIP client that sends an Authorization header —
    // which is most of them, since public casters require one — failed deterministically.
    // 512 covers every client this has been checked against with wide margin.
    char       header[512]   = {0};
    size_t     headerLen     = 0;
    bool       ntrip2        = false;   // client asked for Ntrip/2.0; see the reply below
    char       mount[32]     = {0};     // requested path, for the log and the sourcetable
};
static std::vector<WifiEntry> g_wifiNetworks;
static const int WIFI_NVS_MAX = 10;

// ─────────────────────────────────────────────────────────────────────────────
//  TEMPERATURE MONITORING  (CPU die + IMU heat-soak)
// ─────────────────────────────────────────────────────────────────────────────
// Two independent readings, both surfaced on the LCD ("CPU"/"IMU"), the web
// dashboard ("CPU (ESP32)"/"IMU (QMI8658)"), and the status_NNNN.csv heartbeat
// (esp_temp_f / imu_temp_f, in °F), so a thermal event leaves a trail we can line
// up against the reset_reason on the next boot.
//
//   • CPU  — ESP32-S3 internal sensor via temperatureRead(). Measures the SILICON,
//            so it reads well above ambient (self-heating + WiFi). Good for trend
//            and for catching the die approaching throttle/instability. Not ambient.
//            FIELD-VERIFIED (thermal camera): the die sensor is ACCURATE — ~195 °F die
//            read corresponded to ~170 °F package exterior, the expected die-to-case
//            gradient. The ESP32-S3 genuinely runs hot (~90 °C die) even idle on a desk.
//            temperatureRead() returns 0.0 until it lazily installs on first use; we
//            prime it at boot and discard 0.0/out-of-range reads (that was the bogus
//            32 °F = 0 °C seen right after boot).
//   • IMU  — QMI8658 temperature register over I2C (SCL=IO47, SDA=IO48 per the board
//            hardware map). CRITICAL: this is NOT ambient and NOT the LG290P board.
//            The QMI8658 sits on the ESP32-S3-LCD board UNDERNEATH the SD card, in a
//            trapped-air pocket with poor cooling, a few mm from the MCU die. It reads
//            HEAT-SOAK from that dead-air zone (FIELD-VERIFIED ~73 °C closed-case on a
//            desk while the separate LG290P carrier was only ~38 °C / 100 °F by thermal
//            camera). So it must NOT be used as an analog for case ambient or for the
//            LG290P. What it IS good for: it is the temperature the SD card and IMU sit
//            in — a strong proxy for SD-card thermal stress (consumer SD ceiling ≈ 85 °C;
//            this pocket reaching ~73 °C is a prime suspect for prior SD corruption) and
//            for the LCD-adhesive zone, both of which share that hot region.
//            CTRL1 = 0x40 (addr auto-increment, LITTLE-endian — must match the
//            (b[1]<<8)|b[0] read; 0x60/big-endian corrupts the value). The accelerometer
//            is enabled (CTRL2/CTRL7) so the temperature register actually updates.
//
// THRESHOLDS (°C internally; the sensors report °C — we convert to °F only for
// display/logging). The IMU bands are deliberately conservative because the binding
// limits are physical, not electrical, and the IMU reads the hottest internal zone:
//   • PETG enclosure: glass-transition ≈ 80 °C; it starts to soften/creep under load
//     well before that. Case is PETG, so we want RED safely under Tg.
//   • LCD bonding adhesive: a prior ESP32 was lost when the screen adhesive let go from
//     heat. Conservative failure onset ~70 °C. RED is set at 70 °C to stay below BOTH
//     that and PETG Tg, with amber/orange warnings leading up to it.
//   • SD card: the IMU pocket IS the SD card's environment; the same RED at 70 °C keeps
//     it clear of the ~85 °C consumer-card ceiling.
// The CPU bands run hotter because the die normally does; rated ambient max is +85 °C
// and the die sits 20–40 °C above ambient, so RED there flags the silicon itself getting
// into instability territory.
//
// Thermal mitigation levers if these trip in service (PWM backlight dimming via
// LCD_BL_DIM_DUTY, setCpuFrequencyMhz, venting/heat-spreader) are noted at the backlight
// block and were measured to matter most in this convection-starved pocket.
// Board/IMU thresholds. These were far too conservative: the sensor is the QMI8658 die,
// which sits inside a sealed enclosure next to the ESP32 and normally runs well above
// ambient, so 50 °C (122 °F) is ordinary operation rather than a warning. The QMI8658 is
// specified for -40 to +85 °C, and the practical enclosure limit (PETG softening, SD card
// spec) arrives before the sensor's own. Bands set from those limits instead.
static constexpr float BOARD_T_WARN_C = 65.0f;   // 149 °F — amber: warm, still well in spec
static constexpr float BOARD_T_HOT_C  = 75.0f;   // 167 °F — orange: approaching enclosure limits
static constexpr float BOARD_T_CRIT_C = 85.0f;   // 185 °F — red: QMI8658 rated ceiling
static constexpr float CHIP_T_WARN_C  = 70.0f;   // 158 °F — amber
static constexpr float CHIP_T_HOT_C   = 85.0f;   // 185 °F — orange (rated ambient ceiling)
static constexpr float CHIP_T_CRIT_C  = 95.0f;   // 203 °F — red: die into instability territory

static inline float cToF(float c) { return c * 9.0f / 5.0f + 32.0f; }

// QMI8658 (Waveshare onboard IMU). We only use its temperature register.
// Gate the whole thing behind a flag so a missing/different IMU can never wedge boot.
#ifndef BRIDGE_BOARD_TEMP_ENABLE
#define BRIDGE_BOARD_TEMP_ENABLE  true
#endif
static constexpr int   QMI_PIN_SDA   = 48;   // IMU_SDA (board hardware map)
static constexpr int   QMI_PIN_SCL   = 47;   // IMU_SCL
static constexpr uint8_t QMI_WHOAMI_VAL = 0x05;
static constexpr uint8_t QMI_REG_WHOAMI = 0x00;
static constexpr uint8_t QMI_REG_CTRL1  = 0x02;
static constexpr uint8_t QMI_REG_CTRL2  = 0x03;   // accel ODR / full-scale
static constexpr uint8_t QMI_REG_CTRL7  = 0x08;   // sensor enable (aEN/gEN)
static constexpr uint8_t QMI_REG_TEMP_L = 0x33;   // temp = (int16)(H<<8 | L) / 256 °C

static volatile float g_chipTempC   = NAN;   // ESP32 die, always available
static volatile float g_boardTempC  = NAN;   // QMI8658, may be absent
static volatile bool  g_boardTempOk = false; // false → board temp shows "--" everywhere
static uint8_t        g_qmiAddr     = 0;      // 0 = not found / disabled

// Returns one of 4 LCD colours for a temperature, using the per-sensor band set.
static UiStatus tempBandStatus(float c, bool isChip) {
    if (isnan(c)) return ST_MUTED;
    const float warn = isChip ? CHIP_T_WARN_C : BOARD_T_WARN_C;
    const float hot  = isChip ? CHIP_T_HOT_C  : BOARD_T_HOT_C;
    const float crit = isChip ? CHIP_T_CRIT_C : BOARD_T_CRIT_C;
    if (c >= crit) return ST_BAD;
    if (c >= hot)  return ST_WARN;
    if (c >= warn) return ST_WARN;
    return ST_OK;
}

static bool qmiWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(g_qmiAddr);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}
static bool qmiRead(uint8_t reg, uint8_t* dst, size_t len) {
    Wire.beginTransmission(g_qmiAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;   // repeated-start
    const size_t got = Wire.requestFrom((int)g_qmiAddr, (int)len);
    if (got != len) return false;
    for (size_t i = 0; i < len; ++i) dst[i] = Wire.read();
    return true;
}

// Probe the QMI8658 once at boot. Scans the I2C bus and prints what ACKs (so a
// "no board temp" is diagnosable from the serial log), reads WHO_AM_I raw at both
// candidate addresses, and on a match enables the accelerometer so the temperature
// register actually updates. Never blocks beyond the Wire timeout; on any failure
// board temp is disabled (chip temp still works).
static void qmi8658Init() {
#if BRIDGE_BOARD_TEMP_ENABLE
    Wire.begin(QMI_PIN_SDA, QMI_PIN_SCL, 400000);
    Wire.setTimeOut(20);

    // 1) Bus scan — prove whether ANY device answers on SDA=48/SCL=47.
    Serial.print("🔎 I2C scan (SDA=48,SCL=47):");
    uint8_t found = 0;
    for (uint8_t a = 0x08; a < 0x78; ++a) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); ++found; }
    }
    if (!found) Serial.print(" (nothing responded)");
    Serial.println();

    // 2) Identify at the QMI8658 candidate addresses; print the raw WHO_AM_I byte so a
    //    wrong/different IMU chip is obvious (QMI8658 WHO_AM_I @0x00 == 0x05).
    for (uint8_t addr : { (uint8_t)0x6B, (uint8_t)0x6A }) {
        g_qmiAddr = addr;
        uint8_t who = 0xFF;
        const bool ok = qmiRead(QMI_REG_WHOAMI, &who, 1);
        Serial.printf("   @0x%02X WHO_AM_I=%s0x%02X\n", addr, ok ? "" : "(read failed) ", who);
        if (ok && who == QMI_WHOAMI_VAL) {
            // Enable, in order: CTRL1=0x40 → register address AUTO-INCREMENT, LITTLE-endian
            // (must match the (b[1]<<8)|b[0] read below; 0x60 would set big-endian and
            // silently corrupt the temperature). CTRL2 sets a low accel ODR; CTRL7 enables
            // the accelerometer, which is what makes the temperature register update.
            qmiWrite(QMI_REG_CTRL1, 0x40);
            qmiWrite(QMI_REG_CTRL2, 0x04);   // accel ±2g, low ODR — enough to drive temp
            qmiWrite(QMI_REG_CTRL7, 0x01);   // aEN: accelerometer on
            Serial.printf("🌡️  QMI8658 found @0x%02X — board temp enabled\n", addr);
            return;
        }
    }
    g_qmiAddr = 0;
    Serial.println("🌡️  QMI8658 not found — board temp disabled (chip temp still active)");
#endif
}

// Refresh both temperatures; internally rate-limited unless force=true (used to prime
// the chip sensor at boot, which returns 0.0 on its first not-ready reads).
static void readTemps(bool force = false) {
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
    if (!force && lastMs != 0 && now - lastMs < 2000) return;   // 0.5 Hz is plenty
    lastMs = now;

    // ESP32-S3 die. The sensor lazily installs on first use and returns exactly 0.0
    // until ready (that was the bogus 32°F = 0°C right after boot). Treat 0.0 and any
    // out-of-range value as not-ready and KEEP the last good reading (NAN until the
    // first valid one), so the displays show "--" rather than a fake 0°C.
    const float chip = temperatureRead();
    if (chip != 0.0f && chip > -40.0f && chip < 150.0f) g_chipTempC = chip;

    if (g_qmiAddr) {
        uint8_t b[2];
        if (qmiRead(QMI_REG_TEMP_L, b, 2)) {
            const float c = (int16_t)((b[1] << 8) | b[0]) / 256.0f;
            // Same not-ready guard: reject exact 0.0 (register before first sample) and
            // out-of-range; otherwise accept.
            if (c != 0.0f && c > -40.0f && c < 150.0f) { g_boardTempC = c; g_boardTempOk = true; return; }
        }
        g_boardTempOk = false;   // read failure / not-ready → show "--" rather than stale
    }
}

// THE unit's base identity, derived once from the chip's factory MAC (read from eFuse —
// no WiFi radio needed, so this can run before WiFi.mode() and does not disturb boot
// ordering). ONE identifier feeds all three names, so a unit is recognisable everywhere:
//   base ID           "RDxx"                — also the default Centipede mount name
//   device name/title "RCX RTK Base RDxx"   — LCD title, dashboard, status "dev" field
//   fallback AP SSID  same as the device name
// THE 4-CHARACTER LENGTH IS CENTIPEDE'S CONSTRAINT, NOT OURS (do not lengthen): a
// Centipede mount name is exactly 4 characters, uppercase letters and/or digits, per
// docs.centipede.fr/docs/base/Declaration.html. "RD" marks it as an RCX Datalogger; the
// remaining two characters are base-36 of the MAC's low 16 bits, giving 1296 codes.
static String baseIdFromMac() {
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    static const char B36[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    const uint16_t v = (uint16_t)(((mac[4] << 8) | mac[5]) % 1296U);
    const char buf[5] = { 'R', 'D', B36[v / 36U], B36[v % 36U], '\0' };
    return String(buf);
}

// Resolves g_deviceName once at boot: the NVS-stored name if one exists,
// otherwise a name generated from the chip's factory MAC (read directly from
// eFuse — no WiFi radio needed, so this can run before WiFi.mode() and does
// not disturb boot ordering). The generated name is saved so it's stable
// across reboots until the operator renames it.
static void loadDeviceName() {
    Preferences p;
    if (p.begin("rcx1id", true)) {
        String saved = p.getString("name", "");
        p.end();
        if (saved.length() > 0) {
            g_deviceName = saved;
            Serial.printf("Device name: %s (from NVS)\n", g_deviceName.c_str());
            return;
        }
    }
    g_deviceName = String("RCX RTK Base ") + baseIdFromMac();
    Preferences w;
    if (w.begin("rcx1id", false)) {
        w.putString("name", g_deviceName);
        w.end();
    }
    Serial.printf("Device name: %s (generated from MAC)\n", g_deviceName.c_str());
}

// Default Centipede (crtk.net) mount name offered in the dashboard's "Add a caster"
// form. It is the SAME base ID that names the device and the fallback AP, so one unit
// carries one recognisable identity everywhere. Deterministic from eFuse, so it needs
// no NVS entry of its own — and it is only ever a pre-filled default the operator can
// edit before clicking Add, never a claimed or committed registration.
static String g_suggestedMount;
static void loadSuggestedMount() { g_suggestedMount = baseIdFromMac(); }

// Persists a new device name to NVS. Takes effect on the next reboot — the AP
// SSID and status "dev" field both read from g_deviceName, which is only
// resolved once at boot by loadDeviceName().
// 31 characters is the hard 802.11 SSID limit, and this name IS the AP's SSID, so a
// longer one would be silently truncated by the radio and the dashboard would then be
// describing a network that does not exist under that name. The charset is restricted
// to printable ASCII for the same reason: anything outside it is not portable across
// the clients that have to display and select the network.
static bool setDeviceName(const String& name) {
    if (name.length() == 0 || name.length() > DEVICE_NAME_MAX_CHARS) return false;
    for (size_t i = 0; i < name.length(); ++i) {
        const char c = name.charAt(i);
        if (c < 0x20 || c > 0x7E) return false;
    }
    Preferences p;
    if (!p.begin("rcx1id", false)) return false;
    p.putString("name", name);
    p.end();
    return true;
}

static void loadWifiNetworks() {
    g_wifiNetworks.clear();
    // Networks are stored entirely in NVS, added via the dashboard's WiFi
    // Networks card.
    Preferences p;
    if (!p.begin("rcx_wifi", true)) return;   // namespace may not exist yet — that's fine
    const int n = (int)p.getInt("n", 0);
    for (int i = 0; i < n && i < WIFI_NVS_MAX; ++i) {
        char sk[4], pk[4];
        snprintf(sk, sizeof(sk), "s%d", i);
        snprintf(pk, sizeof(pk), "p%d", i);
        String ssid = p.getString(sk, "");
        String pw   = p.getString(pk, "");
        if (ssid.length() > 0) g_wifiNetworks.push_back({ssid, pw});
    }
    p.end();
    Serial.printf("📶 WiFi: %d NVS networks\n", (int)g_wifiNetworks.size());
}

// Returns the number of user-added (NVS) entries currently stored, and their
// packed indices in the NVS list (separate from g_wifiNetworks indices).
static int countNvsNetworks() {
    Preferences p;
    if (!p.begin("rcx_wifi", true)) return 0;
    const int n = (int)p.getInt("n", 0);
    p.end();
    return n;
}

// Add a new network to NVS and to the live list. Returns false if full.
static bool addWifiNetwork(const String& ssid, const String& password) {
    if (ssid.length() == 0 || ssid.length() > 64) return false;
    // Dedupe: if the SSID already exists anywhere in the live list, update the
    // password in place rather than adding a duplicate entry.
    for (auto& e : g_wifiNetworks) {
        if (e.ssid == ssid) {
            e.password = password;
            // Update the stored password; find its NVS slot.
            Preferences p;
            if (!p.begin("rcx_wifi", false)) return false;
            const int n = (int)p.getInt("n", 0);
            for (int i = 0; i < n && i < WIFI_NVS_MAX; ++i) {
                char sk[4]; snprintf(sk, sizeof(sk), "s%d", i);
                if (p.getString(sk, "") == ssid) {
                    char pk[4]; snprintf(pk, sizeof(pk), "p%d", i);
                    p.putString(pk, password);
                    break;
                }
            }
            p.end();
            return true;
        }
    }
    // New entry — check NVS capacity.
    Preferences p;
    if (!p.begin("rcx_wifi", false)) return false;
    const int n = (int)p.getInt("n", 0);
    if (n >= WIFI_NVS_MAX) { p.end(); return false; }
    char sk[4], pk[4];
    snprintf(sk, sizeof(sk), "s%d", n);
    snprintf(pk, sizeof(pk), "p%d", n);
    p.putString(sk, ssid);
    p.putString(pk, password);
    p.putInt("n", n + 1);
    p.end();
    g_wifiNetworks.push_back({ssid, password});
    Serial.printf("📶 Added WiFi network '%s' (NVS slot %d)\n", ssid.c_str(), n);
    return true;
}

// Delete a stored (NVS) network by its ssid. Rewrites the whole NVS list (at
// most 10 entries — trivial cost).
static bool deleteWifiNetwork(const String& ssid) {
    // Find and remove from live list.
    bool found = false;
    for (auto it = g_wifiNetworks.begin(); it != g_wifiNetworks.end(); ++it) {
        if (it->ssid == ssid) { g_wifiNetworks.erase(it); found = true; break; }
    }
    if (!found) return false;
    // Rewrite NVS: collect remaining entries.
    Preferences p;
    if (!p.begin("rcx_wifi", false)) return false;
    int slot = 0;
    for (const auto& e : g_wifiNetworks) {
        char sk[4], pk[4];
        snprintf(sk, sizeof(sk), "s%d", slot);
        snprintf(pk, sizeof(pk), "p%d", slot);
        p.putString(sk, e.ssid);
        p.putString(pk, e.password);
        ++slot;
    }
    p.putInt("n", slot);
    // Blank any remaining old slots that are now past the new count.
    for (int i = slot; i < WIFI_NVS_MAX; ++i) {
        char sk[4], pk[4];
        snprintf(sk, sizeof(sk), "s%d", i);
        snprintf(pk, sizeof(pk), "p%d", i);
        p.remove(sk); p.remove(pk);
    }
    p.end();
    Serial.printf("📶 Deleted WiFi network '%s' (%d NVS entries remain)\n", ssid.c_str(), slot);
    return true;
}
static uint32_t lastLcdUpdateMs = 0;
static uint32_t lastRateUpdateMs = 0;
static uint32_t gnssRxWindowBytes = 0;
static uint32_t gnssRxBps = 0;

// BUFFERING-SAFETY: at most ONE blocking client.connect() per loop pass. Reset at
// the top of loop(), set by serviceCaster() when it spends a connect attempt.
static bool connectAttemptedThisPass = false;

// ── DEBOUNCED WIFI LINK STATE ───────────────────────────────────────────────
// serviceWifi() and serviceCaster() both used to act on a single raw WiFi.status()
// read. One transient non-WL_CONNECTED return (missed beacon, DHCP renew, roam) was
// enough to (a) WiFi.disconnect() a perfectly healthy association and advance to the
// NEXT SSID in the list, and (b) drop BOTH caster sockets into the 30 s backoff. A
// sub-second glitch therefore became a minute-plus outage displayed as "No WiFi" —
// while the always-on RCX1 softAP kept serving the dashboard normally.
// Require the link to read down CONTINUOUSLY for WIFI_DOWN_CONFIRM_MS before anyone
// is allowed to tear anything down. Recovery is instant (no up-side debounce).
static constexpr uint32_t WIFI_DOWN_CONFIRM_MS = 3000;
static uint32_t g_wifiDownSinceMs = 0;
static bool     g_wifiLinkUp      = false;
// Edge tracker for serviceWifi()'s AP-occupancy gate — logs only on pause/resume
// transitions rather than on every deferred check while a device sits on the AP.
static bool     g_wifiSearchPausedForAp = false;
// Rolling exclusivity watchdog: while a device is on the AP, WiFi search stays paused
// for as long as there has been genuine activity within the last WIFI_SEARCH_PAUSE_WINDOW_MS
// — either a dashboard action (markWifiActivity(), called from the mutating /api/*
// handlers) or RTCM actually being written to a local rover (see localCasterBroadcast()).
// Passive polling (/api/status, /api/wifilist) does NOT touch this, and neither does a
// device merely sitting associated with nothing flowing — that is the "rogue or nuisance
// connection" case the window is bounded FOR. Joining the AP itself also counts as one
// activity event (see the station-count transition check in updateWifiLinkState()), so a
// device always gets at least one full window from the moment it connects.
static constexpr uint32_t WIFI_SEARCH_PAUSE_WINDOW_MS = 60000;
static uint32_t g_wifiActivityMs      = 0;
static int      g_apStationsPrevSeen  = 0;
static bool wifiLinkUp() { return g_wifiLinkUp; }
static void markWifiActivity() { g_wifiActivityMs = millis(); }
static void updateWifiLinkState() {
    const uint32_t now = millis();
    // A device newly associating with the AP is itself one activity event — see the
    // watchdog comment above. Must run before either branch below so it is not skipped
    // by the WL_CONNECTED early return.
    // SAMPLED, NOT POLLED (keep the rate limit). softAPgetStationNum() takes the WiFi
    // API lock, which the WiFi and lwIP tasks contend for; called from every loop pass it
    // runs thousands of times a second to answer a question whose answer changes at human
    // speed. One sample per second is finer than the 60 s activity window needs and keeps
    // the lock out of the GNSS drain's path.
    static uint32_t apPollMs = 0;
    if (apPollMs == 0 || (now - apPollMs) >= 1000) {
        apPollMs = now;
        const int apStations = WiFi.softAPgetStationNum();
        if (apStations > g_apStationsPrevSeen) g_wifiActivityMs = now;
        g_apStationsPrevSeen = apStations;
    }
    if (WiFi.status() == WL_CONNECTED) { g_wifiDownSinceMs = 0; g_wifiLinkUp = true; return; }
    if (g_wifiDownSinceMs == 0) g_wifiDownSinceMs = now;
    if (now - g_wifiDownSinceMs >= WIFI_DOWN_CONFIRM_MS) g_wifiLinkUp = false;
}

// ── LOOP STALL MEASUREMENT ──────────────────────────────────────────────────
// Duration of the PREVIOUS loop pass. Any pass longer than a few ms means something
// blocked (SD, TLS, a web handler, a socket). The caster watchdogs below compare
// absolute millis() deltas, so without forgiveness a stall makes every timer fire at
// once on resume and murders whichever caster happened to be mid-handshake.
static uint32_t g_lastLoopMs     = 0;
static uint32_t g_loopStallMs    = 0;
static uint32_t g_loopStallMaxMs = 0;   // high-water mark, for diagnostics
static uint32_t g_uartHighWater  = 0;   // peak GNSS UART ring occupancy, bytes
static constexpr uint32_t CASTER_STALL_FORGIVE_MS = 2000;
// Bounded writability poll for the caster send gate. See casterWritable().
static constexpr long CASTER_WRITE_POLL_US = 30000;   // 30 ms
// Longest a partially-filled TX buffer may be held before being pushed anyway.
// Keeps correction latency bounded; well under NTRIP_UPLINK_STALL_MS.
static constexpr uint32_t CASTER_TX_MAX_HOLD_MS = 400;
// Minimum spacing between writability polls once a socket is known to be congested.
static constexpr uint32_t CASTER_RECHECK_MS = 100;
// Consecutive connect failures on a cached IP before we re-run DNS. Low enough that a
// caster that genuinely changed address recovers quickly, high enough that ordinary
// connect failures don't drag a blocking lookup back into the loop every retry.
static constexpr uint8_t CASTER_RERESOLVE_AFTER_FAILS = 3;

// AUTO-START latch — see updateBaseReadiness(). baseReady is THE gate that decides
// whether the casters stream. Latched so brief RTCM gaps don't drop the service.
static bool baseReady = false;
static uint32_t baseEvidenceMs = 0;

static uint32_t surveyInSec = DEFAULT_SURVEY_IN_SEC;
static float surveyAccLimit = DEFAULT_SURVEY_IN_ACC_LIMIT_M;   // runtime-adjustable from web
// Best (smallest positive) survey mean-accuracy seen this session. The module ZEROES
// meanAcc the instant survey-in completes (valid==2 → fixed mode), so we latch the last
// real value to persist an HONEST accuracy to NVS instead of the old 99 m sentinel.
static double surveyBestMeanAcc = 0.0;
// On-device LCD telemetry toggle. RAM-only and initialized true, so it ALWAYS
// defaults back ON after any restart/power cycle — turning it off via the web UI is
// a temporary, per-session action that never persists (intentional: the operator
// should never power up to a dark screen). Do not persist this to NVS.
static bool displayTelemetryEnabled = true;
// ── LCD BACKLIGHT (IO46) ─────────────────────────────────────────────────────
// FIELD-VERIFIED on this exact board (Waveshare ESP32-S3-LCD-1.47B) via the standalone
// BL_TEST.ino sketch, which stepped through digitalWrite and LEDC at known duties:
//   • Polarity is ACTIVE-HIGH: HIGH / high-duty = ON, LOW / 0 = OFF.
//   • LEDC PWM works with NORMAL polarity — duty 255 is brighter than 128 — so PWM
//     gives real brightness control (used for thermal/idle dimming via LCD_BL_DIM_DUTY).
//   • ROOT CAUSE of the long "backlight won't turn off / stuck on" bug was ORDERING,
//     not polarity: ledcAttach(IO46) was running BEFORE tft.init(), and tft.init() left
//     the pin uncontrolled, so the backlight sat in its default-on state and ledcWrite()
//     did nothing. Fix: attach LEDC AFTER tft.init() (see initDisplay). Keep it that way.
// THINGS THAT WERE TRIED AND ARE WRONG FOR THIS BOARD (do not reintroduce):
//   • P-channel-MOSFET inversion theory (duty 0 = on): WRONG — board is active-high.
//   • Plain digitalWrite-only with no LEDC: works for on/off but loses PWM dimming;
//     we keep LEDC because brightness control is wanted for thermal management.
//   • Query-string POST args for the web toggle: WebServer::arg() missed them; the
//     toggle now sends an x-www-form-urlencoded body and the handler also has a manual
//     body-parse fallback (see handleApiLog).
//   duty 0 = OFF   duty 150 = ~59% (dim)   duty 255 = full brightness
// ── SUNLIGHT-READABLE PALETTE ────────────────────────────────────────────────
// The display is read outdoors, often in direct sun. A dark theme loses almost all
// apparent contrast there: the backlight cannot compete with ambient light, and what
// the eye actually sees is sunlight reflecting off the panel. Painting the panel WHITE
// turns that reflection into the brightest part of the image and makes dark glyphs read
// as silhouettes, so contrast improves as the sun gets stronger instead of collapsing.
//
// Every text colour below is >= 4.5:1 against white (WCAG AA), verified numerically, so
// nothing depends on hue alone — important both in glare and for colour-blind readers,
// where position and text still carry the meaning. UI_LINE is a divider rule, not text,
// and is deliberately light.
//
// Do not reintroduce UI_WARN or bare UI_FG as text colours: yellow on white is
// roughly 1.1:1 and effectively invisible outdoors.
static constexpr uint16_t UI_BG    = 0xFFFF;   // white panel background
static constexpr uint16_t UI_FG    = 0x0000;   // primary values (21:1)
static constexpr uint16_t UI_LABEL = 0x0010;   // field labels, navy (15.9:1)
static constexpr uint16_t UI_OK    = 0x0425;   // stoplight GREEN (0,135,40)  4.67:1
static constexpr uint16_t UI_WARN  = 0xA280;   // stoplight AMBER (165,82,0)  5.52:1
static constexpr uint16_t UI_BAD   = 0x8800;   // stoplight RED   (140,0,0)   9.93:1
// ── STATUS BANNER ────────────────────────────────────────────────────────────
// The title band is painted SOLID with the overall base condition and is the primary
// at-a-glance signal on the panel: one large block of colour, read from across the
// paddock without focusing on any field. Coloured text on individual values is a
// secondary cue and unavoidably hard to tell apart at small sizes — a full-width fill
// is not.
//
// Fills are saturated (not the muted text palette) because a large area can carry
// vivid colour without hurting legibility, and each pairs with whichever of black or
// white gives the higher contrast:
//
//   OK    green  (0,130,35)  + white text   4.98:1
//   WARN  orange (255,160,0) + BLACK text  10.28:1   (white on orange is only ~2:1)
//   BAD   red    (200,0,0)   + white text   6.08:1
static constexpr uint16_t UI_BANNER_OK   = 0x0404;   // green fill
static constexpr uint16_t UI_BANNER_WARN = 0xFD00;   // orange fill
static constexpr uint16_t UI_BANNER_BAD  = 0xC800;   // red fill
static constexpr uint16_t UI_ON_DARK     = 0xFFFF;   // white text on green / red
static constexpr uint16_t UI_ON_LIGHT    = 0x0000;   // black text on orange

// ── STATUS LED ───────────────────────────────────────────────────────────────
// The onboard RGB LED (GPIO38) carries the same overall condition as the banner above, so
// the whole unit reads as one indicator. The banner faces one direction and needs line of
// sight to a small panel; the LED is visible from anywhere around the case and at the
// moment you walk past the car rather than the moment you stop and look at it.
//
// A STOPLIGHT: pure red, yellow and green. The LED gets its own three values rather than
// reusing the banner fills, because the two are read in completely different ways. The
// banner's colours are pigments on a lit white panel and were chosen for their contrast
// against the text printed on them — which is why WARN is an amber (255,160,0) that black
// text can sit on. An LED has no text to carry and no ground to contrast against; it is
// read purely as a hue, at a glance, from a distance, and the only three hues that need no
// interpretation at all are the ones on every traffic light.
//
// So this is the one place the two surfaces intentionally differ, and they still agree on
// the thing that matters: they are driven from the same statusState() call in the same
// pass, so they can never report different CONDITIONS. Keep that mapping in step —
// ST_OK/ST_WARN/ST_BAD to green/yellow/red — and the colours themselves can be tuned here
// freely without touching the panel.
struct LedColour { uint8_t r, g, b; };
static constexpr LedColour LED_OK    = {   0, 255,   0 };   // green
static constexpr LedColour LED_WARN  = { 255, 255,   0 };   // yellow
static constexpr LedColour LED_BAD   = { 255,   0,   0 };   // red
static constexpr LedColour LED_DARK  = {   0,   0,   0 };   // off — see serviceRgbStatus()

static constexpr int     LED_PIN        = 38;   // RGB_IO; not strapping, flash, PSRAM, or
                                                // used by the LCD, SD, GNSS UART or I2C
// COLOUR ORDER — the single most likely thing to be wrong here, so it is named rather than
// left to a default. A WS2812-style bead is fed one byte per channel and the ORDER of those
// bytes is a property of the part, not of the protocol. The Arduino core assumes GRB, which
// is the common case; this bead is RGB, and the mismatch is not subtle — asking for red
// (64,0,0) puts 0,64,0 on the wire, whose second byte the bead reads as green, so a base in
// its worst state glows the colour that means everything is fine. If the colours ever come
// out swapped again, this constant is the one to change: the enum in the core covers all six
// permutations (LED_COLOR_ORDER_RGB / RBG / GRB / GBR / BRG / BGR).
static constexpr rgb_led_color_order_t LED_ORDER = LED_COLOR_ORDER_RGB;
// Output scale, 0..255, applied to every channel at write time so the colours above stay
// readable as the plain saturated values they are. A quarter duty is visible across a
// paddock in daylight and does not destroy the night vision of whoever is crouched over
// the case reading the panel. Linear, not gamma-corrected: at this level the LED's own
// response dominates and correcting for it buys nothing an operator would see.
static constexpr uint16_t LED_BRIGHTNESS = 64;
// Idempotent re-send interval. The LED latches and holds with no further traffic, so this
// is not needed to sustain the display. It exists so a pixel that latched a corrupted
// value — a glitch on the line, a marginal supply during a WiFi transmit burst — repaints
// itself instead of showing the wrong condition until the next state change, which on a
// healthy base can be hours away. It is also what recovers the power-up state: an
// undriven WS2812 comes up showing whatever its register happened to contain, commonly a
// full-brightness white.
static constexpr uint32_t LED_REFRESH_MS = 5000;

static constexpr uint16_t UI_MUTED = 0x5AEB;   // disabled / not-applicable (6.4:1)
static constexpr uint16_t UI_LINE  = 0xAD55;   // section divider rules

// ── STOPLIGHT STATES ─────────────────────────────────────────────────────────
// COLOURED TEXT ON THE WHITE GROUND — no filled blocks. Solid amber and red fields are
// visually heavy on a panel this small and dense, so on the LCD status is carried by the
// glyph colour alone. (The web dashboard keeps its tinted chips; that surface is larger
// and more sparse, and they read well there.)
//
// Text-only puts the whole burden on the colour choice, so these three differ in
// BRIGHTNESS as well as hue. The obvious dark-green/dark-red pair sits at a 1.01:1
// luminance ratio — identical brightness, separable by hue alone, which is precisely
// what direct sun and polarised lenses strip away. Deepening the red and lightening the
// green opens that gap to 2.13:1 while every colour still clears 4.5:1 on white:
//
//   OK    (0,135,40)   4.67:1 on white
//   WARN  (165,82,0)   5.52:1 on white
//   BAD   (140,0,0)    9.93:1 on white     OK vs BAD brightness ratio 2.13:1
//
// Do not "brighten" BAD toward a vivid red: that raises its luminance back toward the
// green and undoes the one cue that survives glare and red/green colour blindness.

// Worst-of, for rolling several field states into one section state. Ordered so that
// BAD beats WARN beats OK beats MUTED.
static inline UiStatus worst(UiStatus a, UiStatus b) { return (a > b) ? a : b; }

// Stoplight for a numeric reading. okAt/warnAt are the thresholds; set higherIsBetter
// when a LARGER number is healthier (satellite count) rather than smaller (HDOP, EPE,
// correction age). Values <= 0 are treated as "no reading yet" and render muted rather
// than falsely green.
static UiStatus stoplight(double v, double okAt, double warnAt, bool higherIsBetter) {
    if (!(v > 0.0)) return ST_MUTED;
    if (higherIsBetter) return (v >= okAt) ? ST_OK : (v >= warnAt) ? ST_WARN : ST_BAD;
    return (v <= okAt) ? ST_OK : (v <= warnAt) ? ST_WARN : ST_BAD;
}

// Colour of a section divider. The rule carries the section's own state, so a glance
// down the left edge tells you WHICH section is unhappy before you read any text.
// ── FAUX-BOLD TEXT ───────────────────────────────────────────────────────────
// TFT_eSPI's bitmap fonts are 1-bit: a glyph stem is a single pixel wide, which is
// exactly what disappears first in bright light. There is no bold variant of Font 2,
// and the taller built-in fonts (4, 6, 7) will not fit the fixed 18 px row pitch this
// layout depends on.
//
// So each string is drawn TWICE, one pixel apart horizontally, widening every stem from
// 1 px to 2 px. This does not smear, because the fonts have no anti-aliasing: the second
// pass turns on whole pixels adjacent to the first, producing a genuinely heavier glyph
// rather than a blurred one. The order matters — the FIRST pass is opaque and paints the
// field background (which is how stale glyphs get cleared and how the WARN/BAD status
// blocks are filled), and the SECOND is transparent so it only adds stems and cannot
// erase the first. The cursor is restored afterwards so chained prints still line up.
//
// Cost is one extra text render per field at the LCD's 1 Hz refresh — negligible.
static uint16_t g_inkFg = UI_FG;
static uint16_t g_inkBg = UI_BG;

static void setInk(uint16_t fg, uint16_t bg) {
    g_inkFg = fg; g_inkBg = bg;
    tft.setTextColor(fg, bg);
}

static void bprint(const String& text) {
    const int16_t x = tft.getCursorX();
    const int16_t y = tft.getCursorY();
    tft.setTextColor(g_inkFg, g_inkBg);   // opaque pass: clears the field, draws the glyphs
    tft.print(text);
    if (!LCD_BOLD_TEXT) return;
    const int16_t xEnd = tft.getCursorX();
    tft.setTextColor(g_inkFg);            // single argument = transparent background
    tft.setCursor(x + 1, y);
    tft.print(text);
    tft.setCursor(xEnd, y);               // resume where the opaque pass finished
    tft.setTextColor(g_inkFg, g_inkBg);
}

static void bprint(const char* text) { bprint(String(text)); }

// Same two-pass treatment for datum-positioned strings (the title).
static void bdrawString(const String& text, int32_t x, int32_t y) {
    tft.setTextColor(g_inkFg, g_inkBg);
    tft.drawString(text, x, y);
    if (!LCD_BOLD_TEXT) return;
    tft.setTextColor(g_inkFg);
    tft.drawString(text, x + 1, y);
    tft.setTextColor(g_inkFg, g_inkBg);
}

// Apply a stoplight state to the next print(): sets the glyph colour, background white.
static void setStatus(UiStatus s) {
    // The background stays UI_BG in every state: padded fields still need an opaque
    // background to clear stale glyphs, but nothing is filled with colour.
    switch (s) {
        case ST_OK:   setInk(UI_OK,    UI_BG); break;
        case ST_WARN: setInk(UI_WARN,  UI_BG); break;
        case ST_BAD:  setInk(UI_BAD,   UI_BG); break;
        default:      setInk(UI_MUTED, UI_BG); break;
    }
}

// Field LABELS take their section's state too. When the whole base is healthy the panel
// reads solid green top to bottom, so "all green" is a genuine all-clear rather than a
// mix of green values and permanently-neutral labels that the eye has to parse.
static void setLabel(UiStatus s) {
    if (s == ST_OK) setInk(UI_OK, UI_BG);
    else            setInk(UI_LABEL, UI_BG);
}

static UiStatus stateToStatus(CasterState state) {
    switch (state) {
        case CasterState::Streaming:        return ST_OK;
        case CasterState::Authenticated:    return ST_WARN;
        case CasterState::Error:            return ST_BAD;
        case CasterState::Connecting:
        case CasterState::AwaitingResponse:
        case CasterState::Held:             return ST_WARN;
        default:                            return ST_MUTED;
    }
}

static uint16_t dividerColor(UiStatus s) {
    switch (s) {
        case ST_OK:   return UI_OK;
        case ST_WARN: return UI_WARN;
        case ST_BAD:  return UI_BAD;
        default:      return UI_LINE;
    }
}

static constexpr uint8_t LCD_BL_FULL_DUTY = 255;   // full brightness
static constexpr uint8_t LCD_BL_DIM_DUTY  = 150;   // dimmed (~59%) — for thermal/idle dimming
static constexpr uint8_t LCD_BL_OFF_DUTY  = 0;     // backlight off
// Set when the display must repaint from scratch (e.g. telemetry re-enabled after a
// fillScreen). Forces the next updateDisplay() to redraw static elements like the title.
static bool g_displayNeedsRedraw = true;
// Deferred reconfigure: web handlers set these; loop() runs the ~8 s LG290P
// reconfigure OUTSIDE the HTTP handler so the browser gets an instant response
// and the blocking reset happens in loop context (where GNSS stays drained).
static volatile bool g_reconfigPending = false;
// Set by the dashboard's "Force base now"; serviced in loop() by serviceForceBase(),
// which blocks for seconds on a module reset and must not run in an HTTP handler.
static volatile bool g_forceBasePending = false;
// Set only by a read-back that actually saw the module in base mode. Everything that
// reports "this base is running on a saved position" keys on this rather than on the
// intent to get there, because those two were the same variable and the module being a
// rover was invisible from every one of them.
static volatile bool g_baseModeConfirmed = false;
// Set when a survey is needed and the PPP rover-mode survey should run. Consumed in
// loop() (never in a web handler or inside configureLg290pBaseOnce, which must stay a
// pure command sequence). See checkPppSurveyCompletion() for the other end.
static volatile bool g_pppSurveyPending = false;
// Set when only the SURVEY TARGETS need rewriting — a warm re-survey with no module reset.
// Consumed in loop(); the rationale and scope live with restartSurveyWarm() below.
static volatile bool g_surveyRestartPending = false;
// Earliest time a pending PPP survey may start. Set when the restart path had to hot-
// restart the module to escape a persisted fixed-base state; zero = no wait.
static uint32_t g_pppSurveyNotBeforeMs = 0;
// Phase flag for servicePppSurveyStart(): the fixed-mode escape has been sent for the
// currently pending survey and we are waiting out the module restart.
static bool g_pppEscapeDone = false;
// Set inside configureLg290pBaseOnce() when its survey branch runs; converted into
// g_pppSurveyPending only at the very end of that function, AFTER the module's base
// config, PQTMSAVEPAR and PQTMSRR have completed. Starting the survey earlier would aim
// rover-mode commands at a module that is still mid-reset.
static bool g_pppSurveyRequested = false;
// Probe results from PQTMCFGRCVRMODE,R / PQTMCFGSVIN,R. Used at boot to decide whether
// the module is ALREADY configured as our base — if so we skip reconfigure+SAVEPAR+SRR
// so the receiver keeps its warm state (hot start) instead of cold-resetting every boot.
static volatile int      g_probedRcvrMode = -1;   // 1=rover, 2=base, -1=unknown
// When g_probedRcvrMode was last set from an actual module reply. The value itself is a
// CACHE of something that changes underneath it: the base-mode write in
// configureLg290pBaseOnce() changes the receiver's mode without producing a reply, so a
// rover reading taken during the boot move check would otherwise stand as current for the
// rest of the session. Anything that treats "rover" as a reason to act must check this.
static volatile uint32_t g_probedRcvrModeMs = 0;
// Satellite gating as READ BACK from the module — see applySatelliteGating(). -1 means
// no reply has been parsed, which is itself the answer when a mask write goes unacked.
static volatile float    g_probedEleMask = -1.0f;
static volatile float    g_probedCnrMask = -1.0f;
// Antenna descriptor as carried in RTCM 1033. Empty is the interesting case — see the
// 1033 decode in describeRtcmFrame().
static char    g_antennaDesc[32] = "";
static uint8_t g_antennaDescLen  = 0;
static bool    g_antennaDescSeen = false;
static volatile uint8_t  g_probedSvinMode = 0;    // 1=survey-in, 2=fixed, 0=unknown
static volatile uint32_t g_probedSvinDur  = 0;    // configured survey duration/count
static volatile float    g_probedSvinAcc  = 0.0f; // configured survey accuracy limit

// ── PPP support, as REPORTED BY THE MODULE ───────────────────────────────────
// PQTMCFGPPP is written in two places (the base configure path and the PPP survey's
// own setup) and neither has ever looked at the answer, so a module whose firmware
// does not implement the command behaved exactly like one where PPP was working: the
// command went out, nothing came back, and the firmware carried on describing itself
// as a PPP base. These hold the module's own reply to PQTMCFGPPP,R so the question "are
// we actually using PPP" is answerable from the dashboard instead of inferred.
// g_pppSupported stays -1 until a reply (or a timeout) settles it. g_signalCfgReply is
// filled only if a $PQTMCFGSIGNAL happens to arrive — nothing here requests one.
// Fix interval as READ BACK from the module, ms; -1 until a PQTMCFGFIXRATE reply arrives.
// 1000 is required for PPP and is forced in base mode; the rover default is 10 Hz (100 ms).
static volatile int  g_probedFixRateMs = -1;
static volatile int  g_pppSupported = -1;          // -1 unknown, 0 command rejected/absent, 1 accepted
static char          g_pppCfgReply[80]    = "";    // raw $PQTMCFGPPP reply, verbatim
static char          g_signalCfgReply[80] = "";    // raw $PQTMCFGSIGNAL reply, verbatim
// Last $PQTMPPPNAV, verbatim. This is the module's own statement about its PPP solution,
// and it is the one thing no version of this firmware has ever looked at: PQTMPPPNAV has
// never been enabled, and PQTMNAV is deliberately left off, so navSolType is permanently
// 0 and nothing here can distinguish a converged PPP fix from an absent one. Captured
// rather than parsed — the field layout is not something to assume — so the sentence can
// be read off the dashboard and decoded from real output.
static char          g_pppNavSentence[100] = "";
static uint32_t      g_pppNavLastMs = 0;
static uint32_t      g_pppQueryMs = 0;             // when the read-back was last requested
// Defined next to the PPP survey launcher; declared here because the dashboard's
// "Read module config" handler calls it and sits earlier in the file.
static void queryPppSupport();

// ── Saved absolute base position (NVS) ───────────────────────────────────────
// Written by manual coordinate entry (POST /api/setpos). Loaded on every boot;
// the boot position-confidence check (PosCheckState) decides whether the antenna
// is still on this spot and the saved fixed position can be reused, or whether we
// must re-survey. There is no scenario where the base streams from a wrong
// position or outputs zeros — an unresolved/failed check always falls to survey-in.
static double   g_savedLat = 0.0, g_savedLon = 0.0, g_savedAlt = 0.0;
static float    g_savedHAcc = 99.0f;
static char     g_savedSource[20] = "";    // "manual", "survey", etc.
static uint32_t g_savedSurveySec = 0;      // survey duration this position came from (0 = manual/unknown)
static bool     g_savedPositionValid = false;
static char     g_savedNote[96] = "";   // how the saved coordinate was produced
// Gross-sanity bound only: if the mean fix is more than this from the saved
// position the statistical test is skipped and we treat it as a definite move
// (different venue). The real decision is the 2σ confidence threshold; this just
// guards against a saved position from a completely different site.
static constexpr double FIXED_POS_GROSS_MOVE_M = 5000.0; // 5 km → obvious venue change
// Set true once the boot confidence check has resolved (Confirmed/Moved/Timeout);
// prevents re-running it and gates streaming until the venue is settled.
static bool     g_displacementCheckDone = false;
// When the confidence check Confirms (or manual entry is accepted), this makes
// configureLg290pBaseOnce() use PQTMCFGSVIN,W,2 (fixed) instead of survey-in.
static bool     g_useNvsSavedPosition = false;
// True when the most recent boot config HOT-SKIPPED (left the receiver untouched).
// Combined with g_probedSvinMode it tells the confidence check whether the module
// is ALREADY a fixed base at the saved position, so we don't reset it needlessly.
static bool     g_bootHotSkipped = false;

static bool loadSavedPosition() {
    Preferences p;
    if (!p.begin("rcx1pos", true)) return false;          // read-only namespace
    if (!p.getBool("valid", false)) { p.end(); return false; }
    g_savedLat  = p.getDouble("lat",  0.0);
    g_savedLon  = p.getDouble("lon",  0.0);
    g_savedAlt  = p.getDouble("alt",  0.0);
    g_savedHAcc = p.getFloat("hacc", 99.0f);
    { String n = p.getString("note", ""); copyLimited(g_savedNote, sizeof(g_savedNote), n.c_str()); }
    g_savedSurveySec = p.getUInt("svsec", 0);             // 0 = manual / legacy save
    p.getString("src", g_savedSource, sizeof(g_savedSource));
    p.end();
    g_savedPositionValid = (g_savedLat != 0.0 || g_savedLon != 0.0);
    return g_savedPositionValid;
}

static bool savePositionToNvs(double lat, double lon, double alt, float hacc,
                              const char* src, uint32_t surveySec = 0,
                              const char* note = nullptr) {
    Preferences p;
    if (!p.begin("rcx1pos", false)) return false;
    p.putBool("valid",  true);
    p.putDouble("lat",  lat);
    p.putDouble("lon",  lon);
    p.putDouble("alt",  alt);
    p.putFloat("hacc", hacc);
    p.putUInt("svsec", surveySec);
    p.putString("src",  src ? src : "?");
    // PROVENANCE NOTE: a free-text record of HOW this coordinate was produced —
    // method, sample count, convergence quality. A saved coordinate outlives the
    // session that made it, and "0.08 m" means something very different coming from a
    // PPP average than from a windowsill survey-in. Human-readable on purpose; this is
    // the only place that context survives a power cycle.
    p.putString("note", note ? note : "");
    p.end();
    g_savedLat = lat; g_savedLon = lon; g_savedAlt = alt;
    g_savedHAcc = hacc;
    g_savedSurveySec = surveySec;
    if (src) copyLimited(g_savedSource, sizeof(g_savedSource), src);
    copyLimited(g_savedNote, sizeof(g_savedNote), note ? note : "");
    g_savedPositionValid = true;
    Serial.printf("✅ NVS: saved base position %.8f,%.8f alt=%.2f acc=%.3fm src=%s svsec=%lu\n",
                  lat, lon, alt, (double)hacc, g_savedSource, (unsigned long)surveySec);
    if (g_savedNote[0]) Serial.printf("   note: %s\n", g_savedNote);
    return true;
}

static void clearSavedPosition() {
    Preferences p;
    if (!p.begin("rcx1pos", false)) return;
    p.putBool("valid", false);
    p.end();
    g_savedPositionValid = false;
    g_useNvsSavedPosition = false;
    g_savedSurveySec = 0;
    Serial.println("⚠️  NVS: saved base position cleared");
}

// Quick flat-Earth approximation for displacement check (accurate to ~0.1% within 100 km).
static double roughDistanceM(double lat1, double lon1, double lat2, double lon2) {
    const double dLatM = (lat2 - lat1) * 111319.5;
    const double dLonM = (lon2 - lon1) * 111319.5 * cos(lat1 * DEG_TO_RAD);
    return sqrt(dLatM * dLatM + dLonM * dLonM);
}

// ── PQTMNAV navigation output (§2.3.53) ──────────────────────────────────────
// PQTMNAV is the LG290P's full navigation solution message. It carries SolType
// plus lat/lon/alt and their per-axis standard deviations, which is exactly what
// the boot position-confidence check needs to decide whether the base has moved.
// (PQTMNAV's best SolType on this module is 12 = RTK fixed. Whether a PPP solution is
// available at all is firmware-dependent — PQTMCFGPPP is absent from earlier LG290P
// firmware — and is established at runtime by queryPppSupport() rather than assumed
// here in either direction. Absolute position comes from a survey or manual entry.)
//
// SolType semantics — VERIFIED against Quectel LG290P&LGx80P Protocol Spec v1.1
// §2.3.53 (do NOT "correct" these from memory; they came from the manufacturer):
//   0 = Not fixed, 1 = Single, 2 = SBAS, 5 = Pseudorange differential,
//   8 = RTK float, 12 = RTK fixed.
static int    navSolType = 0;
static double navLat = 0.0, navLon = 0.0, navAlt = 0.0;
static float  navLatStd = 99.0f, navLonStd = 99.0f, navAltStd = 99.0f;
static uint32_t navLastMs = 0;

// ── Boot position-confidence check ───────────────────────────────────────────
// On boot, if a saved absolute position exists, accumulate fixes for a short window and
// decide — statistically — whether the antenna is still on the saved spot. This is a 2D
// containment test, correct for a trailer that gets nudged/rotated (no fixed distance):
//
//   N_eff     = clamp( window / correlation_time, 1, N )   // corr'd GNSS error → not √N
//   σ_meas    = mean_EPE / √N_eff                          // honest window-mean 1σ / axis
//   σ_total   = sqrt( σ_saved² + σ_meas² )
//   threshold = σ_total · sqrt( -2·ln(1 - P) )             // 2D Rayleigh, P=0.95 → 2.45σ
//   distance < threshold → CONFIRMED (reuse fixed pos)   ; > → MOVED (re-survey)
//
// Self-scaling: precise (PPP/RTK) boot fixes shrink σ_total and detect small moves; coarse
// autonomous fixes widen it and only flag large ones — so no arbitrary metres-floor is
// needed. A generous cap only guards against a pathological EPE; the 5 km gross bound
// still catches an outright venue change.
enum class PosCheckState : uint8_t {
    Idle,        // no saved position, or check already resolved this boot
    Collecting,  // accumulating PQTMNAV fixes (SolType > 0)
    Confirmed,   // mean within threshold → apply/keep saved fixed position
    Moved,       // mean beyond threshold → clear NVS, survey-in
    Timeout      // window elapsed with too few fixes → survey-in (conservative)
};
static PosCheckState g_posCheckState = PosCheckState::Idle;
static uint32_t g_posCheckStartMs = 0;
static uint32_t g_posCheckCount   = 0;        // PQTMNAV fixes accumulated
static double   g_posCheckSumLat  = 0.0;      // running mean accumulators
static double   g_posCheckSumLon  = 0.0;
static double   g_posCheckSumEpe  = 0.0;      // running mean of per-fix horizontal EPE
static uint32_t g_posCheckLastNavMs = 0;      // dedupe: one sample per PQTMNAV epoch
static uint32_t g_posCheckIdleSinceMs = 0;    // when we first started waiting in Idle for a fix
static bool     g_posCheckForcedRover = false; // we switched the module out of fixed mode to measure
static double   g_posCheckFirstLat = 0.0;      // echo detector — see the sampling loop
static double   g_posCheckFirstLon = 0.0;
static bool     g_posCheckAllSame  = false;
// Tunables (defined here, NOT config.h, so config.h stays verbatim).
static constexpr uint32_t POSCHECK_WINDOW_MS    = 30000;  // 30 s collection window
static constexpr uint32_t POSCHECK_MIN_FIXES    = 10;     // need >=10 epochs to decide
// FAILSAFE: the Idle state waits for the first usable fix with NO window clock,
// deliberately (a disconnected antenna on the bench shouldn't burn the window). But it
// must not wait FOREVER — if a fix never arrives, the venue check would never resolve and
// EVERY caster would be held down permanently.
//
// TWO BOUNDS, because "no fix" has two causes with opposite correct responses, and they
// are told apart by whether the receiver is TALKING (GGA arriving), not by satellite count.
// GGA's satellite field counts satellites USED IN THE SOLUTION, so it reads 0 whenever
// there is no fix no matter how good the sky is — it cannot distinguish these at all.
//
//   GGA flowing, still no fix → the receiver is configured and alive; it has no sky, or is
//     reacquiring after a reset. A survey started here fails for the identical reason while
//     costing the saved coordinate, so be patient. Session 0058 spent two minutes on a
//     bench and queued a 45-minute re-survey of an already-surveyed site.
//   No GGA at all → the message rate never applied, which is a configuration fault the
//     survey actually repairs (the survey path re-asserts the NMEA rates). Time out fast;
//     waiting cannot fix a message that was never enabled. This is the case the original
//     failsafe was written for.
//
// Neither timeout erases NVS. The saved coordinate survives in flash and is reloaded on the
// next boot, so even a wrong timeout is recoverable by a power cycle rather than by
// re-entering a surveyed position by hand.
static constexpr uint32_t POSCHECK_IDLE_TIMEOUT_MS   = 900000;  // 15 min: talking, no fix
static constexpr uint32_t POSCHECK_SILENT_TIMEOUT_MS = 120000;  // 2 min: no GGA at all
// Floor covers the SYSTEMATIC bias between an autonomous cold-boot fix and a precise
// (survey/PPP) saved coordinate — several metres, and NOT reduced by averaging (√N only
// shrinks random noise). A 3 m floor let that bias read as "MOVED", wiping the saved
// position and re-surveying on every boot. 10 m clears typical autonomous bias while
// staying far under the gross bound and cap that catch a real venue change.
// ── Statistical move test parameters ─────────────────────────────────────────
// The confirm/re-survey decision is a 2D statistical test, NOT a fixed distance — right
// for a trailer that gets nudged/rotated. Key correction: GNSS position error is
// TEMPORALLY CORRELATED (iono/tropo/orbit/multipath persist ~minutes), so averaging N
// fixes over a short window is NOT √N better than one fix. We form the window-mean
// uncertainty from an EFFECTIVE independent-sample count (window / correlation time,
// clamped ≥1), combine it with the saved coordinate's own accuracy in quadrature, and
// set the threshold at the desired 2D containment radius. This self-scales: precise
// (PPP/RTK) boot fixes detect small moves; coarse autonomous fixes only flag large ones —
// so no arbitrary metres-floor is needed (that floor was a patch for the old √N over-
// optimism, and it's now gone).
static constexpr double   POSCHECK_CORR_TIME_S = 60.0;   // GNSS error correlation time (s)
static constexpr double   POSCHECK_CONFIRM_P   = 0.95;   // containment prob "hasn't moved"
                                                         // raise → 0.99 for fewer re-surveys
static constexpr double   POSCHECK_THRESH_CAP_M = 100.0; // backstop vs pathological EPE only
// Fixed move tolerance. A constant on purpose — see the move test in
// checkPositionDisplacement() for why a quality-scaled threshold was unsafe.
//
// 5 m, on field evidence, and DO NOT widen it again. It was briefly raised to 10 m to stop
// a false MOVED that had really been caused by deciding on ten unconverged fixes taken
// seconds after a receiver restart. Widening the threshold treated the symptom: the actual
// defect was WHERE the fixes came from, and that is fixed separately — the decision now
// waits for the full window and the post-restart settle is discarded. With the measurement
// sound, the tolerance's only job is to separate "same spot" from "moved", and 10 m is too
// coarse for that: it accepted a lock onto an old position 9 m from where the antenna
// actually was. A base is a reference point, and 9 m of silent error in a reference point
// is worse than a re-survey. 5 m has years of field use behind it at this site.
static constexpr double   POSCHECK_MOVE_THRESH_M = 5.0;
// Ignore fixes for this long after forcing rover mode; the first ones are still the
// fixed-mode echo of the configured coordinate.
// Covers the PQTMHOT restart plus first-fix reacquisition after it. Fixes
// arriving before this are the pre-reset fixed-mode echo and would poison the mean.
static constexpr uint32_t POSCHECK_ROVER_SETTLE_MS = 15000;
// Result of the most recent resolved check, surfaced on the dashboard.
// Measured boot-check position, kept for the logs and the dashboard. The accumulators
// above are reset when a check ends, so the RESULT is latched here — otherwise the only
// record of the coordinate the base actually measured at boot disappears the moment the
// decision is made.
static double g_posCheckMeanLat = 0.0;
static double g_posCheckMeanLon = 0.0;
static double g_posCheckMeanEpe = 0.0;
static double g_posCheckDistM   = -1.0;       // measured mean-to-saved distance (<0 = none)
// WHY the saved position was rejected and a fresh survey-in was forced (empty when the
// position was confirmed, or when no check has resolved yet). Surfaced on the dashboard,
// the LCD and the serial log so a re-survey is never unexplained — the operator can tell
// "antenna genuinely moved" apart from "we never got enough fixes to decide".
static char g_posCheckReason[64] = "";
static double g_posCheckThreshM = 0.0;        // computed 2σ threshold
static bool surveyInSkipped = false;
static uint32_t surveyInCommandMs = 0;      // 0 = survey clock not started yet (no fix)
static uint32_t surveyInCompleteMs = 0;
// True after configureLg290pBaseOnce() set the module to survey-in mode but we are
// WAITING for the first valid GPS lat/lon before starting the survey clock. Per the
// requirement: the clock must not run while lat/lon are still 0,0. The LG290P's own
// survey only averages valid 3D fixes anyway, so this keeps OUR displayed countdown
// honest and prevents the clock (and any fallback-timer-based readiness) from
// advancing during the cold-start search for first fix.
static bool surveyAwaitingFirstFix = false;
// Monotonic high-water mark for survey-elapsed seconds. The module's obs counter can
// RESET if the LG290P re-initiates survey-in (e.g. target time reached without the
// accuracy limit being met), which used to make the displayed countdown jump back up
// to full. Per requirement, the countdown must instead sink to 0 and SIT there until
// the accuracy converges (valid==2). We latch the largest elapsed seen and never let
// it decrease; it's reset only when the survey clock is genuinely re-armed on a new fix.
static uint32_t surveyElapsedHighWaterS = 0;
static char lg290pModeText[32] = "initializing";
static char lg290pLastCommand[96] = "";
static char lg290pLastResponse[160] = "";
static char lastGnssSentence[180] = "";
static uint32_t lg290pCommandsSent = 0;
static uint32_t lg290pPqtmResponses = 0;
static uint32_t lg290pLastPqtmResponseMs = 0;

static char nmeaLine[NMEA_MAX_LEN];
static size_t nmeaIndex = 0;
static bool nmeaActive = false;
static int ggaFixQuality = 0;
static int ggaSatellites = 0;
static double ggaHdop = 0.0;
// Reserved differential reference station IDs a PPP solution reports in GGA field 14.
// From Quectel's PPP application note: these are fixed identifiers for the correction
// service, not a real base station, so seeing one is unambiguous evidence that the fix
// came from PPP rather than from a caster or from nothing.
static constexpr int PPP_STATION_ID_B2B = 9001;   // BeiDou B2b PPP
static constexpr int PPP_STATION_ID_HAS = 9002;   // Galileo E6 HAS — what this base uses

static uint32_t ggaLastSeenMs = 0;
// GGA DIFFERENTIAL REFERENCE STATION ID — the receiver's own statement about which
// correction source produced the current fix, and the only direct, per-epoch evidence
// that PPP is being applied. Quectel's PPP application note gives the reserved values:
// 9002 is an E6 HAS solution and 9001 is a B2b PPP solution, and the note names checking
// this field as THE way to confirm a PPP solution has been entered. Everything the survey
// has watched until now — GGA position, PQTMEPE accuracy, E6 satellites in view — is
// consistent with PPP working and equally consistent with it never engaging, which is the
// ambiguity several full sessions of clamped 1.300 m EPE have been sitting inside.
// Nothing consumes this yet beyond reporting: measure first, then decide whether the
// survey's converged claim should depend on it.
static int      ggaDiffStation   = -1;    // -1 = field absent/empty, else the station ID
static double   ggaDiffAgeS      = -1.0;  // correction age in seconds, <0 = none
static uint32_t ggaDiffStationMs = 0;     // when a PPP station ID was last seen
static char ggaUtc[16]  = "";   // GGA time-of-day "hhmmss.ss" for SD log
static char ggaDate[12] = "";   // RMC date "ddmmyy" — paired with ggaUtc for full datetime
static double ggaLat = 0.0;
static double ggaLon = 0.0;
static double ggaAlt = 0.0;
static double ecefX = 0.0;
static double ecefY = 0.0;
static double ecefZ = 0.0;

// New Global for Live Estimated Position Error
static double liveErrorMeters = 0.0;     // 3D-preferred (falls back to 2D) — for display
static double liveErrorMeters2D = 0.0;   // explicit horizontal (2D) EPE — logged separately
static uint32_t liveErrorLastSeenMs = 0;

// ── Utility ──────────────────────────────────────────────────────────────────
static void copyLimited(char* dst, size_t dstSize, const char* src) {
    if (dstSize == 0) return;
    if (src == nullptr) src = "";
    strncpy(dst, src, dstSize - 1);
    dst[dstSize - 1] = '\0';
}

// Like copyLimited but replaces commas with spaces. Use for any free-text field
// written into a CSV (SSID, caster state) so a stray comma can never shift columns.
// ── One call, two destinations ───────────────────────────────────────────────
// Prints to serial AND records to the SD event channel. Every important diagnostic goes
// through here rather than through Serial.printf directly, because the base is normally
// run where no laptop can be attached — a roof, a paddock, open sky — and a finding that
// exists only on the serial port is unavailable exactly when the base is doing its job.
// Routing both from one call is what stops the two records drifting apart.
//
// level: "ok" | "warn" | "fail" | "info" — enough to filter a long session down to the
// moments that mattered. The emoji stays on the serial line only; the CSV gets plain text.
static void logEvent(const char* level, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

static void logEvent(const char* level, const char* fmt, ...) {
    // Sized for the longest diagnostic this firmware emits. At 160 the heap warning was
    // cut off mid-word, on serial and in the file alike, which is worse than terse: a
    // truncated diagnostic reads as a corrupted one.
    char msg[224];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    const char* icon = "ℹ️ ";
    if      (strcmp(level, "ok")   == 0) icon = "✅";
    else if (strcmp(level, "warn") == 0) icon = "⚠️ ";
    else if (strcmp(level, "fail") == 0) icon = "🛑";
    Serial.printf("%s %s\n", icon, msg);
    bridge_sdlog_event(level, msg);
}

static void copyCsvField(char* dst, size_t dstSize, const char* src) {
    copyLimited(dst, dstSize, src);
    for (char* p = dst; *p; ++p) if (*p == ',') *p = ' ';
}

static String htmlEscape(const char* input) {
    String output;
    if (input == nullptr) return output;
    for (const char* p = input; *p; ++p) {
        switch (*p) {
            case '&': output += F("&amp;"); break;
            case '<': output += F("&lt;"); break;
            case '>': output += F("&gt;"); break;
            case '"': output += F("&quot;"); break;
            case '\'': output += F("&#39;"); break;
            default: output += *p; break;
        }
    }
    return output;
}

// Boot progress display. Defined with the other display helpers much further down, but
// declared here because the GNSS probe reports through it long before that point.
// Explicit rather than relying on the Arduino builder's generated prototypes.
static void drawBootSplash(const char* line1, const char* line2);

static String padRight(String str, unsigned int length) {
    if (str.length() > length) return str.substring(0, length);
    while (str.length() < length) str += ' ';
    return str;
}

static uint8_t nmeaChecksum(const char* payload) {
    uint8_t cksum = 0;
    while (*payload) cksum ^= static_cast<uint8_t>(*payload++);
    return cksum;
}

static void sendPqtm(const char* payload) {
    char sentence[160];
    const uint8_t cksum = nmeaChecksum(payload);
    snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", payload, cksum);
    GnssSerial.print(sentence);
    lg290pCommandsSent++;
    copyLimited(lg290pLastCommand, sizeof(lg290pLastCommand), sentence);
    if (SERIAL_GNSS_CMD_ECHO_ENABLE) {
        Serial.print(F("LG290P CMD: "));
        Serial.print(sentence);
    }
}

static void processGnssSerial();

static void waitAndDrainGnss(uint32_t ms) {
    const uint32_t startMs = millis();
    while (millis() - startMs < ms) {
        processGnssSerial();
        delay(1);
    }
}

// ── Capture buffers ──────────────────────────────────────────────────────────
static void appendCapture(CasterTxCapture& capture, const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        capture.data[capture.head] = data[i];
        capture.head = (capture.head + 1) % CASTER_TX_CAPTURE_SIZE;
        if (capture.count < CASTER_TX_CAPTURE_SIZE) ++capture.count;
    }
}

static void appendRtcmValidCapture(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        rtcmValidCapture.data[rtcmValidCapture.head] = data[i];
        rtcmValidCapture.head = (rtcmValidCapture.head + 1) % RTCM_VALID_CAPTURE_SIZE;
        if (rtcmValidCapture.count < RTCM_VALID_CAPTURE_SIZE) ++rtcmValidCapture.count;
    }
}

static size_t oldestIndex(size_t head, size_t count, size_t capacity) {
    return (head + capacity - count) % capacity;
}

static String buildHexAsciiDump(const uint8_t* data, size_t head, size_t count, size_t capacity, size_t maxBytes) {
    if (count == 0) return String("(no bytes captured)");
    const size_t displayCount = min(count, maxBytes);
    const size_t oldest = oldestIndex(head, count, capacity);
    const size_t skip = count - displayCount;
    const size_t start = (oldest + skip) % capacity;
    String dump;
    dump.reserve(displayCount * 4 + 256);
    char line[96];
    for (size_t offset = 0; offset < displayCount; offset += 16) {
        const size_t lineLength = min(static_cast<size_t>(16), displayCount - offset);
        snprintf(line, sizeof(line), "%08lX  ", static_cast<unsigned long>(skip + offset));
        dump += line;
        for (size_t column = 0; column < 16; ++column) {
            if (column < lineLength) {
                const uint8_t value = data[(start + offset + column) % capacity];
                snprintf(line, sizeof(line), "%02X ", value);
                dump += line;
            } else {
                dump += F("   ");
            }
            if (column == 7) dump += ' ';
        }
        dump += F(" |");
        for (size_t column = 0; column < lineLength; ++column) {
            const uint8_t value = data[(start + offset + column) % capacity];
            dump += (value >= 32 && value <= 126) ? static_cast<char>(value) : '.';
        }
        dump += F("|\n");
    }
    return dump;
}

// ── CRC-24Q and RTCM parser ──────────────────────────────────────────────────
static uint32_t crc24q(const uint8_t* data, size_t length) {
    uint32_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint32_t>(data[i]) << 16;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc <<= 1;
            if (crc & 0x1000000UL) crc ^= 0x1864CFBUL;
        }
        crc &= 0xFFFFFFUL;
    }
    return crc & 0xFFFFFFUL;
}

static uint16_t extractRtcmType(const uint8_t* frame, size_t length) {
    if (length < 6) return 0;
    return (static_cast<uint16_t>(frame[3]) << 4) | (static_cast<uint16_t>(frame[4]) >> 4);
}

// MSM = Multiple Signal Messages = the actual per-satellite observations a rover
// needs for RTK. GPS 1071-1077, GLONASS 1081-1087, Galileo 1091-1097, SBAS
// 1101-1107, QZSS 1111-1117, BeiDou 1121-1127, NavIC 1131-1137. A base only emits
// these once it has a valid reference position (i.e. AFTER survey-in converges) —
// which is exactly why they are the correct "real corrections are flowing" signal,
// unlike 1005/1033 which the LG290P emits with a preliminary position during survey.
// Has this message type appeared in the stream at any point since boot? Used by the
// completeness audit — a type that was requested and has never once been decoded was
// declined by the receiver, whatever its PQTMCFGMSGRATE write appeared to do.
static bool rtcmTypeEverSeen(uint16_t type) {
    for (const auto& stat : rtcmStats.typeStats) {
        if (stat.type == type) return stat.count > 0;
    }
    return false;
}

static bool isMsmObservation(uint16_t type) {
    return type >= 1071 && type <= 1137 && (type % 10) >= 1 && (type % 10) <= 7;
}

// Human-readable name for the RTCM message types this base can produce.
static const char* rtcmTypeName(uint16_t type) {
    switch (type) {
        case 1004: return "GPS L1/L2 obs (legacy)";
        case 1005: return "Stationary ARP";
        case 1006: return "Stationary ARP + height";
        case 1007: return "Antenna descriptor";
        case 1008: return "Antenna descriptor + serial";
        case 1012: return "GLONASS L1/L2 obs (legacy)";
        case 1019: return "GPS ephemeris";
        case 1020: return "GLONASS ephemeris";
        case 1033: return "Receiver & antenna descriptors";
        case 1042: return "BeiDou ephemeris";
        case 1044: return "QZSS ephemeris";
        case 1045: case 1046: return "Galileo ephemeris";
        case 1230: return "GLONASS code-phase biases";
    }
    if (isMsmObservation(type)) {
        const char* sys = "?";
        if (type <= 1077) sys = "GPS";
        else if (type <= 1087) sys = "GLONASS";
        else if (type <= 1097) sys = "Galileo";
        else if (type <= 1107) sys = "SBAS";
        else if (type <= 1117) sys = "QZSS";
        else if (type <= 1127) sys = "BeiDou";
        else sys = "NavIC";
        static char buf[28];
        snprintf(buf, sizeof(buf), "%s MSM%u obs", sys, (unsigned)(type % 10));
        return buf;
    }
    return "unknown";
}

// Compact constellation/purpose tag for the LCD (decodes the raw type number into
// something readable: 1074 -> "GPSobs", 1019 -> "GPSeph", 1033 -> "AntDsc").
static const char* rtcmTag(uint16_t type) {
    switch (type) {
        case 1005: return "BaseP";
        case 1006: return "BaseP+";
        case 1007: case 1008: case 1033: return "AntDsc";
        case 1019: return "GPSeph";
        case 1020: return "GLOeph";
        case 1042: return "BDSeph";
        case 1044: return "QZSeph";
        case 1045: case 1046: return "GALeph";
        case 1230: return "GLObia";
    }
    if (isMsmObservation(type)) {
        if (type <= 1077) return "GPSobs";
        if (type <= 1087) return "GLOobs";
        if (type <= 1097) return "GALobs";
        if (type <= 1107) return "SBSobs";
        if (type <= 1117) return "QZSobs";
        if (type <= 1127) return "BDSobs";
        return "NAVobs";
    }
    return "?";
}

static RtcmTypeStat* findOrCreateTypeStat(uint16_t type) {
    RtcmTypeStat* emptySlot = nullptr;
    for (auto& stat : rtcmStats.typeStats) {
        if (stat.type == type) return &stat;
        if (stat.type == 0 && emptySlot == nullptr) emptySlot = &stat;
    }
    if (emptySlot != nullptr) {
        emptySlot->type = type;
        emptySlot->count = 0;
        emptySlot->lastSeenMs = 0;
        return emptySlot;
    }
    return nullptr;
}

// ── NMEA parsing ─────────────────────────────────────────────────────────────
static double nmeaDegreesToDecimal(const char* value, char hemisphere) {
    if (value == nullptr || value[0] == '\0') return 0.0;
    const double raw = atof(value);
    const int deg = static_cast<int>(raw / 100.0);
    double decimal = deg + (raw - deg * 100.0) / 60.0;
    if (hemisphere == 'S' || hemisphere == 'W') decimal = -decimal;
    return decimal;
}

// ── Empirical survey-in scatter (realistic accuracy — DISPLAY/LOG ONLY) ───────
// The receiver's own PQTMSVINSTATUS <MeanAcc> is the standard error of the survey's
// weighted-mean position, computed as though successive ~1 Hz fixes were statistically
// independent. Multipath and atmospheric delay change slowly (correlation on the order
// of tens of seconds to minutes — the same effect documented for network RTK by
// Odolinski, "Temporal correlation in network RTK positioning", GPS Solutions 16,
// 369-379, 2012), so meanAcc systematically understates the true uncertainty,
// especially for shorter surveys. Rather than assume a correlation time we have no
// field data for, we track the ACTUAL horizontal scatter of the raw fixes seen during
// the survey — a measured quantity, not a modeled one — and never report an accuracy
// tighter than that. This is a conservative floor under the receiver's own optimistic
// number, in the spirit of trusting field data over formal-error theory.
//
// Local ENU offsets use a flat equirectangular approximation around the survey's first
// fix; accurate to well under 1 mm over the few-meter scatter radii involved here, so
// it adds no meaningful error of its own.
//
// SCOPE (do not expand): this feeds DISPLAY TEXT and LOG COLUMNS only — the LCD "acc"
// line, the dashboard's accuracy text, and the status CSV. It must never feed
// surveyStatus.meanAcc, surveyBestMeanAcc, g_savedHAcc, or savePositionToNvs()'s acc
// argument — those drive survey completion gating and the hot-start move-check
// tolerance, and stay on the receiver's raw number exactly as before.
struct SurveyScatter {
    bool     active = false;   // true while a survey is in progress and we're accumulating
    uint32_t n = 0;
    double   refLat = 0.0, refLon = 0.0, cosRefLat = 1.0;   // first-fix ENU origin
    double   sumE = 0.0, sumN = 0.0, sumE2 = 0.0, sumN2 = 0.0;
};
static SurveyScatter g_svScatter;

// Called once when a new survey-in starts (see armSurveyClockOnFix()).
static void resetSurveyScatter() {
    g_svScatter = SurveyScatter();
    g_svScatter.active = true;
}

// Called once when the survey converges (see parseSurveyStatus()) — freezes the
// scatter at whatever was observed during the actual survey, matching how g_savedHAcc
// freezes the achieved accuracy of a completed survey.
static void stopSurveyScatter() {
    g_svScatter.active = false;
}

static void accumulateSurveyScatter() {
    if (!g_svScatter.active || (ggaLat == 0.0 && ggaLon == 0.0)) return;
    if (g_svScatter.n == 0) {
        g_svScatter.refLat = ggaLat;
        g_svScatter.refLon = ggaLon;
        g_svScatter.cosRefLat = cos(ggaLat * PI / 180.0);
    }
    constexpr double M_PER_DEG_LAT = 111320.0;   // WGS-84 mean meters/degree latitude
    const double e = (ggaLon - g_svScatter.refLon) * M_PER_DEG_LAT * g_svScatter.cosRefLat;
    const double n = (ggaLat - g_svScatter.refLat) * M_PER_DEG_LAT;
    g_svScatter.sumE  += e;   g_svScatter.sumN  += n;
    g_svScatter.sumE2 += e * e; g_svScatter.sumN2 += n * n;
    g_svScatter.n++;
}

// Empirical 1-sigma horizontal scatter of the raw fixes accumulated so far this survey
// (population std dev of east + north offsets from the first fix, combined as a 2D
// radial sigma). Returns 0 before enough samples have accumulated to mean anything.
static double surveyScatterSigmaM() {
    if (g_svScatter.n < 5) return 0.0;
    const double nD = (double)g_svScatter.n;
    const double varE = g_svScatter.sumE2 / nD - (g_svScatter.sumE / nD) * (g_svScatter.sumE / nD);
    const double varN = g_svScatter.sumN2 / nD - (g_svScatter.sumN / nD) * (g_svScatter.sumN / nD);
    const double v = varE + varN;
    return v > 0.0 ? sqrt(v) : 0.0;
}

// The REALISTIC accuracy shown to a human: the receiver's own figure (or, once fixed,
// the saved position's achieved accuracy), floored at the empirically observed scatter.
// Returns -1.0 when no honest number is available at all (caller falls back to "--").
// Forward declaration: haveSurveyStatus() is defined with the other survey text helpers,
// far below this point. Declared explicitly rather than leaning on the Arduino builder's
// automatic prototype generation, which is order-sensitive for static functions.
static bool haveSurveyStatus();

// The base's honest current accuracy, or -1 when no figure exists.
//
// Sources, in order of authority:
//   1. PQTMSVINSTATUS <MeanAcc>, while the receiver's own survey-in is running.
//   2. The LIVE PPP SURVEY's mean 2D EPE. This is the one that was missing, and its
//      absence is why the field read as nothing for a whole 45-minute window: the PPP
//      survey runs the receiver in ROVER mode, where PQTMSVINSTATUS is not emitted at
//      all, and no other source applies until the survey locks. Source 1 was silent,
//      source 3 had been cleared by the survey that was starting, so the only survey
//      this firmware actually runs was the one case with no accuracy at all.
//   3. The saved coordinate's own accuracy, once fixed — the module ZEROES <MeanAcc>
//      the moment it enters fixed mode, so the honest figure is what the survey that
//      produced the coordinate achieved.
//
// Whatever the source, the result is floored at the observed fix scatter: the receiver's
// own number runs optimistic, and a figure below what the fixes are actually doing is
// not one to publish. See accumulateSurveyScatter().
static double realisticAccuracyM() {
    double base;
    if (haveSurveyStatus() && surveyStatus.meanAcc > 0.0) {
        base = surveyStatus.meanAcc;
    } else if (ppp_survey_active()) {
        const PppSurveyStatus ps = ppp_survey_status();
        // Prefer the running mean over all fixes to the best-ever single epoch: the mean
        // is what the coordinate will actually be built from, and best-so-far only ever
        // improves, so it would report a survey as converging when it was drifting.
        if (!isnan(ps.meanEpe2dAll) && ps.meanEpe2dAll > 0.0f) base = (double)ps.meanEpe2dAll;
        else if (ps.lastEpe2d > 0.0f)                          base = (double)ps.lastEpe2d;
        else return -1.0;
    } else if (g_savedPositionValid && g_savedHAcc > 0.0f) {
        base = (double)g_savedHAcc;
    } else {
        return -1.0;
    }
    const double scatter = surveyScatterSigmaM();
    return scatter > base ? scatter : base;
}

static void updateEcefFromLla() {
    if (ggaLat == 0.0 && ggaLon == 0.0) return;
    const double latRad = ggaLat * PI / 180.0;
    const double lonRad = ggaLon * PI / 180.0;
    const double a = 6378137.0;
    const double e2 = 0.00669437999014;
    const double sinLat = sin(latRad);
    const double n = a / sqrt(1.0 - e2 * sinLat * sinLat);
    ecefX = (n + ggaAlt) * cos(latRad) * cos(lonRad);
    ecefY = (n + ggaAlt) * cos(latRad) * sin(lonRad);
    ecefZ = (n * (1.0 - e2) + ggaAlt) * sinLat;
    accumulateSurveyScatter();   // realistic-accuracy tracking; see block above
}

// ECEF (meters) → geodetic lat/lon (deg) + alt (m), WGS-84, Bowring closed-form.
// Used to convert the survey-in's converged mean ECEF position into lat/lon for NVS
// storage (savePositionToNvs takes lat/lon/alt). Accurate to < 1 mm for terrestrial
// positions — far finer than any GNSS position error, so it adds no meaningful error.
static void ecefToLla(double X, double Y, double Z,
                      double& latDeg, double& lonDeg, double& altM) {
    const double a  = 6378137.0;                 // WGS-84 semi-major axis
    const double e2 = 0.00669437999014;          // first eccentricity squared
    const double b  = a * sqrt(1.0 - e2);        // semi-minor axis
    const double ep2 = (a * a - b * b) / (b * b); // second eccentricity squared
    const double p  = sqrt(X * X + Y * Y);
    if (p < 1e-6) {                              // at/near a pole — avoid div-by-zero
        latDeg = (Z >= 0 ? 90.0 : -90.0);
        lonDeg = 0.0;
        altM   = fabs(Z) - b;
        return;
    }
    const double th = atan2(a * Z, b * p);
    const double sinTh = sin(th), cosTh = cos(th);
    const double lat = atan2(Z + ep2 * b * sinTh * sinTh * sinTh,
                             p - e2 * a * cosTh * cosTh * cosTh);
    const double lon = atan2(Y, X);
    const double sinLat = sin(lat);
    const double n = a / sqrt(1.0 - e2 * sinLat * sinLat);
    latDeg = lat * 180.0 / PI;
    lonDeg = lon * 180.0 / PI;
    altM   = p / cos(lat) - n;
}

// Map a survey duration (seconds) to the boot position-confirmation window (ms).
// A longer survey is a deliberate, high-value reference, so we invest more time
// confirming it on reboot — both because we care more about not losing it, and
// because a higher-quality saved position needs more accumulated fixes to confirm
// the antenna hasn't moved against its tighter saved accuracy.
//   < 1 hr  → 30 s    (a quick grab; cheap to re-survey if it times out)
//   1–6 hr  → 120 s
//   > 6 hr  → 300 s   (a painstaking reference; give it every chance to confirm)
static uint32_t posCheckWindowMsForSurvey(uint32_t surveySec) {
    if (surveySec >= 6UL * 3600UL) return 300000UL;   // > 6 hr
    if (surveySec >= 1UL * 3600UL) return 120000UL;   // 1–6 hr
    return 30000UL;                                   // < 1 hr (incl. 0 = manual/legacy)
}

static void parseCommaFields(char* sentence, char* fields[], size_t maxFields, size_t& fieldCount) {
    fieldCount = 0;
    char* p = sentence;
    if (*p == '$') ++p;
    fields[fieldCount++] = p;
    while (*p && fieldCount < maxFields) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            fields[fieldCount++] = p + 1;
            if (*(p + 1) == '\0') break;
        }
        ++p;
    }
}

static void parseGgaSentence(char* sentence) {
    char work[NMEA_MAX_LEN];
    copyLimited(work, sizeof(work), sentence);
    char* fields[24] = {0};
    size_t count = 0;
    parseCommaFields(work, fields, 24, count);
    if (count < 10) return;
    if (strcmp(fields[0], "GPGGA") != 0 && strcmp(fields[0], "GNGGA") != 0) return;
    copyLimited(ggaUtc, sizeof(ggaUtc), fields[1]);   // hhmmss.ss for SD log
    ggaLat = nmeaDegreesToDecimal(fields[2], fields[3][0]);
    ggaLon = nmeaDegreesToDecimal(fields[4], fields[5][0]);
    ggaFixQuality = atoi(fields[6]);
    ggaSatellites = atoi(fields[7]);
    ggaHdop = atof(fields[8]);
    ggaAlt = atof(fields[9]);
    // Fields 13/14 are the differential age and reference station ID. They are absent on
    // an autonomous fix, so a short sentence is normal and not an error — hence the
    // separate count guard rather than raising the one above, which would start rejecting
    // ordinary GGA. Field 14 is the LAST field, so it still carries the "*checksum" tail;
    // atoi stops at the '*' and returns the ID regardless.
    ggaDiffAgeS    = (count > 13 && fields[13][0]) ? atof(fields[13]) : -1.0;
    const int prevStation = ggaDiffStation;
    ggaDiffStation = (count > 14 && fields[14][0]) ? atoi(fields[14]) : -1;
    if (ggaDiffStation == PPP_STATION_ID_HAS || ggaDiffStation == PPP_STATION_ID_B2B)
        ggaDiffStationMs = millis();
    // Announce only the transition. This is the line that turns "PPP is not working" from
    // an inference into an observation, in both directions, and it belongs on the card
    // rather than only on a console nobody is watching in a paddock.
    if (ggaDiffStation != prevStation) {
        if (ggaDiffStation == PPP_STATION_ID_HAS)
            logEvent("ok", "PPP solution ENGAGED - Galileo E6 HAS (station %d)", ggaDiffStation);
        else if (ggaDiffStation == PPP_STATION_ID_B2B)
            logEvent("ok", "PPP solution ENGAGED - B2b PPP (station %d)", ggaDiffStation);
        else if (prevStation == PPP_STATION_ID_HAS || prevStation == PPP_STATION_ID_B2B)
            logEvent("warn", "PPP solution LOST - fix is no longer PPP-corrected");
    }
    ggaLastSeenMs = millis();
    updateEcefFromLla();
}

static int pvtFixType = 0;   // PQTMPVT <FixType>: 0=no fix, 2=2D, 3=3D (NOT a GGA quality code)

static void parseRmcSentence(char* sentence) {
    // $GNRMC/$GPRMC field 9 = date "ddmmyy". That's all we need; GGA already gives
    // time-of-day. Together they form a complete GPS datetime for log correlation.
    // Layout: [0]=talker+RMC [1]=hhmmss.ss [2]=A/V [3..8]=position/speed/course
    //         [9]=ddmmyy
    // Only store the date when the fix is valid (status=='A'), so we never write a
    // stale date that looks legitimate.
    char work[NMEA_MAX_LEN];
    copyLimited(work, sizeof(work), sentence);
    char* fields[16] = {0};
    size_t count = 0;
    parseCommaFields(work, fields, 16, count);
    if (count < 10) return;
    if (strcmp(fields[0], "GPRMC") != 0 && strcmp(fields[0], "GNRMC") != 0) return;
    if (fields[2][0] != 'A') return;        // 'A'=valid, 'V'=void — date only on valid fix
    if (strlen(fields[9]) >= 6) copyLimited(ggaDate, sizeof(ggaDate), fields[9]);
}

static void parsePqtmPvt(char* sentence) {
    char work[NMEA_MAX_LEN];
    copyLimited(work, sizeof(work), sentence);
    char* fields[24] = {0};
    size_t count = 0;
    parseCommaFields(work, fields, 24, count);
    if (count < 13 || strcmp(fields[0], "PQTMPVT") != 0) return;
    // SEMANTICS BUG FIX (do NOT revert): PQTMPVT field 6 is <FixType> (0/2/3 =
    // none/2D/3D per spec §2.3.24), a DIFFERENT code space from GGA's <Quality>
    // (0/1/2/4/5/7). The old code wrote FixType into ggaFixQuality, so a PVT epoch
    // clobbered a real GGA quality (e.g. 4=RTK fixed → 3=3D) and the LCD label/colour
    // logic — which is keyed to GGA quality — misread it. ggaFixQuality is now owned
    // solely by parseGgaSentence(); PVT keeps its own pvtFixType.
    pvtFixType = atoi(fields[6]);
    ggaSatellites = atoi(fields[7]);
    if (fields[9][0] != '\0' && fields[10][0] != '\0') {
        ggaLat = atof(fields[9]);
        ggaLon = atof(fields[10]);
        ggaAlt = atof(fields[11]);
        ggaLastSeenMs = millis();
        updateEcefFromLla();
    }

    // DATE + TIME from PQTMPVT.
    // VERIFIED field layout from actual sentence:
    //   $PQTMPVT,1,575056000,20260627,154358.000,1,3,...
    //   [0]=PQTMPVT [1]=MsgVer [2]=TOW_ms [3]=Date(yyyymmdd) [4]=UTC(hhmmss.sss) [5]=FixType
    // NOT [4] and [5] as previously coded — that was off by one and produced garbage.
    // PQTMPVT arrives from first satellite epoch — before RMC status='A' — so this
    // populates datetime for all logs from the moment the module has satellite time.
    if (count > 3 && fields[3] && strlen(fields[3]) == 8) {
        // yyyymmdd → ddmmyy for ggaDate (what fmtDatetime expects)
        const char* d = fields[3];           // e.g. "20260628"
        // Reject LG290P cold-start epoch (19800106)
        if (!(d[0]=='1'&&d[1]=='9'&&d[2]=='8'&&d[3]=='0'&&d[4]=='0'&&d[5]=='1'&&d[6]=='0'&&d[7]=='6')) {
            char ddmmyy[7];
            ddmmyy[0] = d[6]; ddmmyy[1] = d[7];   // dd
            ddmmyy[2] = d[4]; ddmmyy[3] = d[5];   // mm
            ddmmyy[4] = d[2]; ddmmyy[5] = d[3];   // yy
            ddmmyy[6] = '\0';
            copyLimited(ggaDate, sizeof(ggaDate), ddmmyy);
        }
    }
    if (count > 4 && fields[4] && strlen(fields[4]) >= 6) {
        const char* t = fields[4];           // e.g. "154358.000"
        // Reject all-zero time when we have no date yet (meaningless pre-fix noise)
        if (!(t[0]=='0'&&t[1]=='0'&&t[2]=='0'&&t[3]=='0'&&t[4]=='0'&&t[5]=='0'&&ggaDate[0]=='\0')) {
            copyLimited(ggaUtc, sizeof(ggaUtc), t);
        }
    }
}

static void parsePqtmNav(char* sentence) {
    // PQTMNAV §2.3.53 (field offsets VERIFIED against Quectel spec v1.1):
    // $PQTMNAV,<MsgVer>,<TimeStatus>,<TimeRef>,<UTC>,<Date>,<TOW>,<WN>,<LeapSec>,
    //   <Res>,<Res>,<SolType>,<Res>,<Lat>,<Lon>,<Alt>,<Sep>,<Res>,<Res>,
    //   <LatStd>,<LonStd>,<AltStd>,...
    // With fields[0]="PQTMNAV": SolType=11, Lat/Lon/Alt=13/14/15, Std=19/20/21.
    // These offsets are IDENTICAL to the old (dead) PPP parser — only the message
    // name and SolType meaning changed. Do NOT shift them.
    // SolType (spec): 0=not fixed, 1=single, 2=SBAS, 5=pseudorange diff,
    //                 8=RTK float, 12=RTK fixed.
    char work[256];
    copyLimited(work, sizeof(work), sentence);
    char* fields[44] = {0};
    size_t count = 0;
    parseCommaFields(work, fields, 44, count);
    if (count < 15 || strcmp(fields[0], "PQTMNAV") != 0) return;
    navSolType = (fields[11] && fields[11][0]) ? atoi(fields[11]) : 0;
    navLat     = (fields[13] && fields[13][0]) ? atof(fields[13]) : 0.0;
    navLon     = (fields[14] && fields[14][0]) ? atof(fields[14]) : 0.0;
    navAlt     = (fields[15] && fields[15][0]) ? atof(fields[15]) : 0.0;
    if (count > 21) {
        navLatStd = (fields[19] && fields[19][0]) ? (float)atof(fields[19]) : 99.0f;
        navLonStd = (fields[20] && fields[20][0]) ? (float)atof(fields[20]) : 99.0f;
        navAltStd = (fields[21] && fields[21][0]) ? (float)atof(fields[21]) : 99.0f;
    }
    navLastMs = millis();
}

static void parsePqtmEpe(char* sentence) {
    char work[NMEA_MAX_LEN];
    copyLimited(work, sizeof(work), sentence);
    char* fields[10] = {0};
    size_t count = 0;
    parseCommaFields(work, fields, 10, count);
    // PQTMEPE layout: $PQTMEPE,<Ver>,<EPE_North>,<EPE_East>,<EPE_Down>,<EPE_2D>,<EPE_3D>*CS
    // 3D error is field 6, 2D is field 5. (Previously read 2D as 3D and Down as 2D —
    // that index swap is why accuracy could read as 0.) Guard empty fields, which
    // some firmware emits before a fix is available.
    if (count >= 7 && strcmp(fields[0], "PQTMEPE") == 0) {
        const double e3d = (fields[6] && fields[6][0]) ? atof(fields[6]) : 0.0;
        const double e2d = (fields[5] && fields[5][0]) ? atof(fields[5]) : 0.0;
        const double epe = (e3d > 0.0) ? e3d : e2d;
        if (epe > 0.0) {
            liveErrorMeters = epe;
            liveErrorLastSeenMs = millis();
        }
        if (e2d > 0.0) liveErrorMeters2D = e2d;   // record horizontal (xy) separately
    }
}

static void parseSurveyConfigResponse(char* sentence) {
    char work[NMEA_MAX_LEN];
    copyLimited(work, sizeof(work), sentence);
    char* fields[16] = {0};
    size_t count = 0;
    parseCommaFields(work, fields, 16, count);
    if (count < 7 || strcmp(fields[0], "PQTMCFGSVIN") != 0 || strcmp(fields[1], "OK") != 0) return;
    const uint8_t mode = static_cast<uint8_t>(atoi(fields[2]));
    const uint32_t cfgCnt = static_cast<uint32_t>(strtoul(fields[3], nullptr, 10));
    surveyStatus.cfgDur = cfgCnt;
    g_probedSvinMode = mode;                       // for the hot-start config check
    g_probedSvinDur  = cfgCnt;
    if (count > 4) g_probedSvinAcc = atof(fields[4]);
    if (mode == 1 && surveyStatus.lastSeenMs == 0) {
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "survey config read");
    } else if (mode == 2) {
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "fixed config read");
    }
}

static void parseSurveyStatus(char* sentence) {
    // SPEC-PINNED PARSE (Quectel LG290P&LGx80P Protocol Spec v1.1 §2.3.23).
    // Autonomous output layout (fields[0]="PQTMSVINSTATUS"):
    //   [1]=MsgVer [2]=TOW [3]=Valid [4]=Res0 [5]=Res1 [6]=Obs [7]=CfgDur
    //   [8]=MeanX [9]=MeanY [10]=MeanZ [11]=MeanAcc
    //   Spec example: $PQTMSVINSTATUS,1,1000,1,,01,20,100,-2484434.36,4875976.97,3266161.34,1.2415*
    // Query response prefixes an "OK": [1]="OK" then the same body shifted by one
    //   ([2]=MsgVer ... [4]=Valid [7]=Obs [8]=CfgDur ... [12]=MeanAcc), so detect the
    //   "OK" form explicitly instead of guessing by field count (the old fragile path
    //   that mis-keyed Valid and made auto-start distrust valid==2). <Valid>: 0=invalid
    //   or mode-off, 1=in-progress, 2=valid (survey converged) — this IS the authoritative
    //   survey-complete signal; updateBaseReadiness() now gates casting on it.
    char work[NMEA_MAX_LEN];
    copyLimited(work, sizeof(work), sentence);
    char* fields[20] = {0};
    size_t count = 0;
    parseCommaFields(work, fields, 20, count);

    if (count < 8 || strcmp(fields[0], "PQTMSVINSTATUS") != 0) return;

    if (strcmp(fields[1], "OK") == 0) {
        // Query-response form: body shifted right by one for the "OK" token.
        if (count < 13) return;
        surveyStatus.valid   = static_cast<uint8_t>(atoi(fields[4]));
        surveyStatus.obs     = static_cast<uint32_t>(strtoul(fields[7], nullptr, 10));
        surveyStatus.cfgDur  = static_cast<uint32_t>(strtoul(fields[8], nullptr, 10));
        surveyStatus.meanX   = atof(fields[9]);
        surveyStatus.meanY   = atof(fields[10]);
        surveyStatus.meanZ   = atof(fields[11]);
        surveyStatus.meanAcc = atof(fields[12]);
        surveyStatus.lastSeenMs = millis();
    } else {
        // Autonomous output form.
        if (count < 12) return;
        surveyStatus.valid   = static_cast<uint8_t>(atoi(fields[3]));
        surveyStatus.obs     = static_cast<uint32_t>(strtoul(fields[6], nullptr, 10));
        surveyStatus.cfgDur  = static_cast<uint32_t>(strtoul(fields[7], nullptr, 10));
        surveyStatus.meanX   = atof(fields[8]);
        surveyStatus.meanY   = atof(fields[9]);
        surveyStatus.meanZ   = atof(fields[10]);
        surveyStatus.meanAcc = atof(fields[11]);
        surveyStatus.lastSeenMs = millis();
    }

    // Latch the best real mean-accuracy while the survey is still converging; it zeroes
    // at valid==2 (fixed mode), and we want an honest number to persist for hot-start.
    if (surveyStatus.meanAcc > 0.0 &&
        (surveyBestMeanAcc <= 0.0 || surveyStatus.meanAcc < surveyBestMeanAcc))
        surveyBestMeanAcc = surveyStatus.meanAcc;

    // Sanity-guard the back-date: a mis-parsed (huge) obs here would otherwise
    // underflow/poison the ESP32 fallback timer. Normal path: surveyInCommandMs
    // was already stamped when the survey clock started, so this rarely runs.
    if (surveyInCommandMs == 0 && surveyStatus.obs > 0 && surveyStatus.obs <= 86400UL)
        surveyInCommandMs = surveyStatus.lastSeenMs - surveyStatus.obs * 1000UL;

    // SURVEY-COMPLETE AUTO-SAVE (do not remove): the instant the receiver reports a
    // converged survey (valid==2) for the first time this session, persist the
    // converged mean position to NVS — tagged with the survey DURATION and achieved
    // mean accuracy. This is what enables hot-start-with-move-detection on the next
    // power cycle: without it, a completed survey lives only in the module and the
    // ESP can't run the position-confidence check, so a reboot can't tell whether the
    // base moved. We only save survey positions here (src="survey"); manual entries
    // are saved by handleApiSetPos. Guard against a bogus zero-ECEF report.
    // NEVER AUTO-SAVE FROM THE MODULE'S SURVEY STATUS WHEN WE RUN THE PPP SURVEY.
    // surveyStatus mirrors PQTMSVINSTATUS, which the module keeps reporting as valid=2
    // from its OWN last completed survey — a result that persists in module flash. So
    // the moment the operator cleared the saved position, the very next PQTMSVINSTATUS
    // (about a second later) re-triggered this block and wrote the SAME coordinate back
    // with src="survey". That is why "clear position" appeared to do nothing: it worked,
    // and was then instantly undone from here, before any new survey had even started.
    //
    // Under BASE_SURVEY_USE_PPP our own survey is authoritative and persists its result
    // in checkPppSurveyCompletion(); the module's internal survey status is stale
    // history and must not write anything.
    if (BASE_SURVEY_USE_PPP) {
        if (surveyStatus.valid == 2 && surveyInCompleteMs == 0) {
            surveyInCompleteMs = surveyStatus.lastSeenMs;   // bookkeeping only
            stopSurveyScatter();
        }
        return;
    }
    if (surveyStatus.valid == 2 && surveyInCompleteMs == 0) {
        surveyInCompleteMs = surveyStatus.lastSeenMs;
        stopSurveyScatter();   // freeze the realistic-accuracy scatter at the achieved survey
        const bool ecefValid = (fabs(surveyStatus.meanX) > 1000.0 ||
                                fabs(surveyStatus.meanY) > 1000.0 ||
                                fabs(surveyStatus.meanZ) > 1000.0);
        // Only auto-save a survey result — never overwrite a trusted manual entry that
        // we're currently confirming, and only when we actually ran a survey this boot
        // (not a hot-skip reusing an existing NVS position).
        const bool surveyingThisBoot = !g_useNvsSavedPosition && !LG290P_USE_FIXED_BASE;
        if (ecefValid && surveyingThisBoot) {
            double sLat, sLon, sAlt;
            ecefToLla(surveyStatus.meanX, surveyStatus.meanY, surveyStatus.meanZ,
                      sLat, sLon, sAlt);
            // Persist an HONEST accuracy. meanAcc reads 0 at completion (fixed mode), so
            // fall back to the best real value latched during convergence; only if we never
            // saw one do we use the survey accuracy LIMIT as a conservative bound (never the
            // old 99 m sentinel, which poisoned the next boot's confidence threshold).
            float acc = (surveyStatus.meanAcc > 0.0) ? (float)surveyStatus.meanAcc
                       : (surveyBestMeanAcc > 0.0)   ? (float)surveyBestMeanAcc
                       : surveyAccLimit;
            savePositionToNvs(sLat, sLon, sAlt, acc, "survey", surveyInSec);
            Serial.printf("📍 Survey converged (%lus, acc=%.3fm) — auto-saved to NVS for hot-start\n",
                          (unsigned long)surveyInSec, (double)acc);
        }
    }
}

// Count of NMEA sentences whose trailing *HH checksum did not match, i.e. sentences
// that arrived CORRUPTED. This is the direct test for UART data loss: a torn sentence
// is how a wrong field (session 0454 logged one epoch with the year as 2032) or a
// stale/duplicated epoch gets into the logs. Monotonic; diff between status rows.
static uint32_t g_nmeaChecksumFailures = 0;
// Benign counterpart: captures started by a '$' byte inside binary RTCM. Tracked
// separately so it cannot mask real corruption. See handleNmeaSentence().
static uint32_t g_nmeaFramerDesyncs = 0;

static bool nmeaChecksumOk(const char* s) {
    if (!s || s[0] != '$') return false;
    const char* star = strrchr(s, '*');
    if (!star || !isxdigit((unsigned char)star[1]) || !isxdigit((unsigned char)star[2]))
        return false;                       // no checksum present — don't count as failure
    uint8_t cs = 0;
    for (const char* p = s + 1; p < star; ++p) cs ^= (uint8_t)*p;
    char buf[3] = { star[1], star[2], 0 };
    return cs == (uint8_t)strtol(buf, nullptr, 16);
}

static void handleNmeaSentence() {
    rtcmStats.nmeaSentences++;
    // ⚠ REJECT corrupted sentences — do NOT go back to merely counting them.
    // Session 0455 logged 18,662 checksum failures in 32.7 h (one every ~6 s) and their
    // rate correlates with RTCM output at r = -0.72, i.e. the UART is losing bytes and
    // tearing sentences. Every torn sentence used to be PARSED ANYWAY, which is how the
    // logs ended up with a year of 2032, a minute field of -284 and a duplicated epoch.
    // A torn GGA carries a corrupted LAT/LON into the position path, so this is a
    // correctness issue, not just cosmetics: one bad fix must never reach the base
    // position logic. A sentence that fails its own checksum is garbage — drop it.
    if (nmeaLine[0] == '$' && strrchr(nmeaLine, '*') && !nmeaChecksumOk(nmeaLine)) {
        // Split the two very different causes, or the metric is unreadable:
        //
        //  a) FRAMER DESYNC (expected, benign). NMEA and binary RTCM share this UART.
        //     feedNmeaParserByte() starts a capture on ANY '$' (0x24), including one
        //     that is just a byte inside an RTCM payload. That capture then runs until
        //     a 0x0A appears, producing a "sentence" of binary garbage. It cannot be
        //     prevented from inside the framer — 0x24 is a legal payload byte — and it
        //     is harmless because the capture is discarded here. Session 0457 measured
        //     ~91/hr of these, uncorrelated with output rate (r=0.006).
        //
        //  b) A REAL sentence whose checksum failed = genuine UART byte loss. This is
        //     the number that matters. In 0455 it ran at ~571/hr alongside 219 RTCM
        //     CRC failures/hr; after the blocking serial writes were removed both went
        //     to zero. Any NON-ZERO value here should be treated as data loss.
        //
        // Test: a real sentence has a printable 5-char header ($ + 5 alphanumerics).
        bool looksReal = true;
        for (int i = 1; i <= 5 && looksReal; ++i)
            if (!isalnum((unsigned char)nmeaLine[i])) looksReal = false;
        if (looksReal) g_nmeaChecksumFailures++;   // (b) real corruption
        else           g_nmeaFramerDesyncs++;      // (a) benign RTCM byte in the stream
        return;
    }
    // Only surface clean, printable NMEA on the dashboard/LCD. At 460800 baud a stray
    // '$' inside binary RTCM can momentarily desync the NMEA framer and capture a line
    // of random bytes; show the last GOOD sentence instead of that garbage.
    bool printable = (nmeaLine[0] == '$');
    for (const char* p = nmeaLine; *p && printable; ++p) {
        if (*p < 0x20 || *p > 0x7E) printable = false;
    }
    if (printable) copyLimited(lastGnssSentence, sizeof(lastGnssSentence), nmeaLine);
    parseGgaSentence(nmeaLine);
    parseRmcSentence(nmeaLine);
    parsePqtmPvt(nmeaLine);
    parsePqtmEpe(nmeaLine);
    parsePqtmNav(nmeaLine);
    parseSurveyStatus(nmeaLine);
    parseSurveyConfigResponse(nmeaLine);
    if (strncmp(nmeaLine, "$PQTMCFGRCVRMODE,OK,", 20) == 0) {
        const int mode = atoi(nmeaLine + 20);
        g_probedRcvrMode = mode;                   // for the hot-start config check
        g_probedRcvrModeMs = millis();
        // Clear the confirmation the moment the module reports anything but base mode.
        // Placed here, in the single parser that sees every PQTMCFGRCVRMODE reply, rather
        // than at each of the eight sites that write rover mode — one of those would
        // eventually be added without the matching clear, and a stale "confirmed" is
        // exactly the failure this flag was introduced to end.
        if (mode != 2) g_baseModeConfirmed = false;
        if (mode == 2) copyLimited(lg290pModeText, sizeof(lg290pModeText), "base");
        else if (mode == 1) copyLimited(lg290pModeText, sizeof(lg290pModeText), "rover");
        else copyLimited(lg290pModeText, sizeof(lg290pModeText), "unknown");
    } else if (strncmp(nmeaLine, "$PQTMCFGRCVRMODE,OK", 19) == 0) {
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "mode command accepted");
    }
    // PPP / signal-configuration read-backs. Captured verbatim rather than parsed: the
    // field layout of both replies varies across firmware revisions, and the only thing
    // asked of them here is whether the module answers OK, ERROR or nothing at all.
    if (strncmp(nmeaLine, "$PQTMCFGPPP", 11) == 0) {
        copyLimited(g_pppCfgReply, sizeof(g_pppCfgReply), nmeaLine);
        g_pppSupported = (strncmp(nmeaLine, "$PQTMCFGPPP,OK", 14) == 0) ? 1 : 0;
    } else if (strncmp(nmeaLine, "$PQTMCFGFIXRATE,OK,", 19) == 0) {
        // The fix interval decides whether PPP can converge at all: the engine needs 1 Hz,
        // base mode pins it there, and rover mode — which is where a PPP survey runs —
        // defaults to 10 Hz. Captured so the answer is observable rather than assumed.
        g_probedFixRateMs = atoi(nmeaLine + 19);
    } else if (strncmp(nmeaLine, "$PQTMCFGSIGNAL", 14) == 0) {
        copyLimited(g_signalCfgReply, sizeof(g_signalCfgReply), nmeaLine);
    } else if (strncmp(nmeaLine, "$PQTMCFGELETHD,OK,", 18) == 0) {
        g_probedEleMask = (float)atof(nmeaLine + 18);
    } else if (strncmp(nmeaLine, "$PQTMCFGCNRTHD,OK,", 18) == 0) {
        g_probedCnrMask = (float)atof(nmeaLine + 18);
    } else if (strncmp(nmeaLine, "$PQTMPPPNAV", 11) == 0) {
        copyLimited(g_pppNavSentence, sizeof(g_pppNavSentence), nmeaLine);
        g_pppNavLastMs = millis();
    }
    if (strncmp(nmeaLine, "$PQTM", 5) == 0) {
        lg290pPqtmResponses++;
        lg290pLastPqtmResponseMs = millis();
        copyLimited(lg290pLastResponse, sizeof(lg290pLastResponse), nmeaLine);
        // Surface the receiver's actual replies (incl. $PQTMCFGMSGRATE OK/ERROR at
        // boot) so config failures are visible — previously only the *sent* command
        // was printed ("LG290P CMD:"), never the response.
        // ⚠ Serial here is USB CDC. With no host reading the port a write can BLOCK for
        // ~100 ms, and this fires ~3x/second. That is the same failure mode as the
        // per-frame RTCM diagnostic: sessions 0442-0455 lost UART bytes continuously
        // (219 RTCM CRC + 193 framing failures/hour) and disabling that one print took
        // both counters to ZERO in 0457. An unattended base must not narrate itself.
        //
        // PERIODIC telemetry (PVT / EPE / SVINSTATUS, ~3 lines/s) and CONFIG REPLIES
        // (CFGMSGRATE OK/ERROR, CFGRCVRMODE, CFGSVIN, SAVEPAR ...) are both gated —
        // the latter can re-fire during normal operation (e.g. after a detected move
        // re-runs the full config sequence), not just at a bench flash.
        {
            const bool periodic = (strncmp(nmeaLine, "$PQTMPVT,",         9) == 0) ||
                                  (strncmp(nmeaLine, "$PQTMEPE,",         9) == 0) ||
                                  (strncmp(nmeaLine, "$PQTMSVINSTATUS,", 16) == 0) ||
                                  (strncmp(nmeaLine, "$PQTMPPPNAV,",     12) == 0);
            const bool show = periodic ? SERIAL_GNSS_ECHO_ENABLE : SERIAL_GNSS_CMD_ECHO_ENABLE;
            if (show) {
                Serial.print(F("LG290P RSP: "));
                Serial.println(nmeaLine);
            }
        }
    }
    bridge_sdlog_feed_nmea(nmeaLine);   // extracts GSV satellites for sat_*.csv
    ppp_survey_feed_nmea(nmeaLine);     // PPP manual survey: consumes GGA/EPE/GSV (E6)
}

static void feedNmeaParserByte(uint8_t c) {
    if (c == '$') {
        nmeaActive = true;
        nmeaIndex = 0;
        nmeaLine[nmeaIndex++] = static_cast<char>(c);
        rtcmStats.nmeaBytes++;
        return;
    }
    if (!nmeaActive) return;
    rtcmStats.nmeaBytes++;
    if (c == '\r') return;
    if (c == '\n') {
        nmeaLine[nmeaIndex] = '\0';
        handleNmeaSentence();
        nmeaActive = false;
        return;
    }
    if (nmeaIndex < sizeof(nmeaLine) - 1) {
        nmeaLine[nmeaIndex++] = static_cast<char>(c);
    } else {
        nmeaActive = false;
    }
}

// ── Caster state and transmission ────────────────────────────────────────────
static const char* stateToString(CasterState state) {
    switch (state) {
        case CasterState::Disabled: return "Disabled";
        case CasterState::Held: return "Held - base not publishable";
        case CasterState::WaitingForWifi: return "No WiFi";
        case CasterState::Connecting: return "Connecting";
        case CasterState::AwaitingResponse: return "Awaiting source response";
        case CasterState::Authenticated: return "Authenticated - waiting RTCM";
        case CasterState::Streaming: return "Streaming valid RTCM";
        case CasterState::Error: return "Error";
        default: return "Unknown";
    }
}

// ── Caster (NTRIP destination) management ────────────────────────────────────
// Forward decls — the management functions below call these, defined just after.
static void setCasterState(NtripTarget& caster, CasterState state);
static void resetCasterTx(NtripTarget& caster);
static void stopCaster(NtripTarget& caster, CasterState nextState, const char* reason = nullptr);

// NVS namespace "rcx1cast" — base-station-specific (parallel to rcx1pos), NOT shared
// with the rover. Casters are added from the web dashboard's Casters card and
// stored here.
// Keys: "n" (caster count), and per slot i: "h{i}" host, "p{i}" port,
// "m{i}" mount, "w{i}" password. Enable flags are stored separately keyed by
// host+mount hash so they survive across reorders: "en_{host}_{mount}" → bool.
// Max MAX_CASTERS total.

static void casterEnableKey(char* dst, size_t cap, const char* host, const char* mount) {
    // Compact, NVS-key-safe (<=15 char) enable key. NVS keys cap at 15 chars, so we
    // can't use the full host+mount; use a short djb2 hash of host|mount instead.
    uint32_t h = 5381;
    for (const char* s = host;  *s; ++s) h = ((h << 5) + h) ^ (uint8_t)*s;
    for (const char* s = mount; *s; ++s) h = ((h << 5) + h) ^ (uint8_t)*s;
    snprintf(dst, cap, "en%08lx", (unsigned long)h);
}

static bool loadCasterEnabled(const char* host, const char* mount, bool defEnabled) {
    char key[16]; casterEnableKey(key, sizeof(key), host, mount);
    Preferences p;
    if (!p.begin("rcx1cast", true)) return defEnabled;
    const bool v = p.getBool(key, defEnabled);
    p.end();
    return v;
}

static void saveCasterEnabled(const char* host, const char* mount, bool enabled) {
    char key[16]; casterEnableKey(key, sizeof(key), host, mount);
    Preferences p;
    if (!p.begin("rcx1cast", false)) return;
    p.putBool(key, enabled);
    p.end();
}

// Populate one caster slot from explicit fields.
static void setCasterFields(NtripTarget& c, const char* host, uint16_t port,
                            const char* mount, const char* pw) {
    copyLimited(c.host, sizeof(c.host), host ? host : "");
    c.port = port;
    copyLimited(c.mountpoint, sizeof(c.mountpoint), mount ? mount : "");
    copyLimited(c.password, sizeof(c.password), pw ? pw : "");
    c.protocol = CasterProtocol::NtripV1Source;
}

// Build the live caster array at boot from NVS, applying each one's persisted
// enable flag. Called once from setup().
static void loadCasters() {
    casterCount = 0;
    // Bootstrap the NVS namespace on first boot. Without this, the read-only
    // loadCasterEnabled() opens below log a scary (but harmless) "nvs_open failed:
    // NOT_FOUND" on every boot until something is written. Creating the namespace
    // once (read-write) makes subsequent read-only opens succeed quietly.
    {
        Preferences boot;
        if (boot.begin("rcx1cast", false)) {
            if (!boot.isKey("init")) boot.putBool("init", true);
            boot.end();
        }
    }
    // Casters stored in NVS.
    Preferences p;
    if (!p.begin("rcx1cast", true)) {
        Serial.printf("📡 Casters: none configured (no NVS namespace yet)\n");
        return;
    }
    const int n = (int)p.getInt("n", 0);
    for (int i = 0; i < n && casterCount < MAX_CASTERS; ++i) {
        char hk[6], pk[6], mk[6], wk[6];
        snprintf(hk, sizeof(hk), "h%d", i); snprintf(pk, sizeof(pk), "p%d", i);
        snprintf(mk, sizeof(mk), "m%d", i); snprintf(wk, sizeof(wk), "w%d", i);
        String host = p.getString(hk, ""), mount = p.getString(mk, ""), pw = p.getString(wk, "");
        uint16_t port = (uint16_t)p.getUInt(pk, 2101);
        if (host.length() == 0 || mount.length() == 0) continue;
        setCasterFields(casters[casterCount], host.c_str(), port, mount.c_str(), pw.c_str());
        casters[casterCount].enabled = loadCasterEnabled(host.c_str(), mount.c_str(), true);
        casterCount++;
    }
    p.end();
    Serial.printf("📡 Casters: %d total from NVS\n", casterCount);
}

// Add a user caster to NVS + live array. Returns false if full or duplicate.
static bool addCaster(const String& host, uint16_t port, const String& mount, const String& pw) {
    if (host.length() == 0 || host.length() >= 64) return false;
    if (mount.length() == 0 || mount.length() >= 40) return false;
    if (casterCount >= MAX_CASTERS) return false;
    // Dedupe against host+mount (a caster is identified by where it pushes).
    for (int i = 0; i < casterCount; ++i) {
        if (host == casters[i].host && mount == casters[i].mountpoint) return false;
    }
    Preferences p;
    if (!p.begin("rcx1cast", false)) return false;
    const int n = (int)p.getInt("n", 0);
    char hk[6], pk[6], mk[6], wk[6];
    snprintf(hk, sizeof(hk), "h%d", n); snprintf(pk, sizeof(pk), "p%d", n);
    snprintf(mk, sizeof(mk), "m%d", n); snprintf(wk, sizeof(wk), "w%d", n);
    p.putString(hk, host); p.putUInt(pk, port);
    p.putString(mk, mount); p.putString(wk, pw);
    p.putInt("n", n + 1);
    p.end();
    // Append to live array.
    setCasterFields(casters[casterCount], host.c_str(), port, mount.c_str(), pw.c_str());
    casters[casterCount].enabled = true;
    saveCasterEnabled(host.c_str(), mount.c_str(), true);
    setCasterState(casters[casterCount], CasterState::WaitingForWifi);
    casterCount++;
    Serial.printf("📡 Added caster %s/%s:%u\n", host.c_str(), mount.c_str(), port);
    return true;
}

// Delete a caster. Rewrites the packed NVS list.
static bool deleteCaster(const String& host, const String& mount) {
    int idx = -1;
    for (int i = 0; i < casterCount; ++i) {
        if (host == casters[i].host && mount == casters[i].mountpoint) { idx = i; break; }
    }
    if (idx < 0) return false;
    // Stop the connection before removing.
    stopCaster(casters[idx], CasterState::Disabled);
    // Shift the live array down to fill the gap. WiFiClient isn't copyable, so move
    // fields explicitly via setCasterFields rather than assigning whole structs.
    for (int i = idx; i < casterCount - 1; ++i) {
        const NtripTarget& src = casters[i + 1];
        setCasterFields(casters[i], src.host, src.port, src.mountpoint, src.password);
        casters[i].enabled = src.enabled;
        casters[i].state = src.state;
    }
    casterCount--;
    // Rewrite the NVS caster list from what remains.
    Preferences p;
    if (!p.begin("rcx1cast", false)) return false;
    int slot = 0;
    for (int i = 0; i < casterCount; ++i) {
        char hk[6], pk[6], mk[6], wk[6];
        snprintf(hk, sizeof(hk), "h%d", slot); snprintf(pk, sizeof(pk), "p%d", slot);
        snprintf(mk, sizeof(mk), "m%d", slot); snprintf(wk, sizeof(wk), "w%d", slot);
        p.putString(hk, casters[i].host); p.putUInt(pk, casters[i].port);
        p.putString(mk, casters[i].mountpoint); p.putString(wk, casters[i].password);
        ++slot;
    }
    p.putInt("n", slot);
    p.end();
    Serial.printf("📡 Deleted caster %s/%s (%d user casters remain)\n", host.c_str(), mount.c_str(), slot);
    return true;
}

// Enable/disable a caster by host+mount. Persists the flag; takes effect next service pass.
static bool setCasterEnabled(const String& host, const String& mount, bool enabled) {
    for (int i = 0; i < casterCount; ++i) {
        if (host == casters[i].host && mount == casters[i].mountpoint) {
            casters[i].enabled = enabled;
            saveCasterEnabled(host.c_str(), mount.c_str(), enabled);
            if (!enabled) {
                stopCaster(casters[i], CasterState::Disabled);
            } else {
                // Re-enabling is a deliberate retry, so the previous rejection has been
                // seen and its job is done. Every other path leaves it standing.
                casters[i].lastError[0] = '\0';
                setCasterState(casters[i], CasterState::WaitingForWifi);
            }
            Serial.printf("📡 Caster %s/%s %s\n", host.c_str(), mount.c_str(),
                          enabled ? "enabled" : "disabled");
            return true;
        }
    }
    return false;
}

static void setCasterState(NtripTarget& caster, CasterState state) {
    if (state == CasterState::Authenticated) caster.failCount = 0;   // creds accepted — reset backoff
    caster.state = state;
    caster.stateStartMs = millis();
}

static void stopCaster(NtripTarget& caster, CasterState nextState, const char* reason) {
    if (caster.client.connected()) caster.client.stop();
    // Discard any coalesced RTCM still buffered for the OLD socket. Carrying it into
    // a fresh connection would inject a stale, mid-burst fragment ahead of the new
    // stream and desync the caster's parser.
    resetCasterTx(caster);
    caster.tcpConnected = false;
    caster.handshakeAccepted = false;
    caster.wroteValidRtcm = false;
    caster.responseLen = 0;
    caster.responseHeader[0] = '\0';
    // Escalate the reconnect backoff on every error-stop (connect fail, rejected
    // handshake, response timeout, mid-stream write failure). Saturating ++.
    if (nextState == CasterState::Error && caster.failCount < 255) caster.failCount++;
    // Record WHY, and KEEP it. A caster rejection is over in milliseconds, and the very
    // next thing that happens to a rejected caster is a transition out of Error — the
    // operator switching it off, or the readiness gate holding it down — so clearing the
    // reason on any non-error stop erased the one artifact worth reading, from both the
    // dashboard and the log, before anyone could read it. Field case: three separate
    // rejected cast attempts across two hours left six Error rows and not one word about
    // why. The reason now survives until the caster actually gets somewhere — cleared
    // only on an accepted handshake, or when the operator re-enables it to try again.
    if (nextState == CasterState::Error && reason) {
        copyLimited(caster.lastError, sizeof(caster.lastError), reason);
    }
    setCasterState(caster, nextState);
}

static void appendResponseChar(NtripTarget& caster, char c) {
    if (caster.responseLen < sizeof(caster.responseHeader) - 1) {
        caster.responseHeader[caster.responseLen++] = c;
        caster.responseHeader[caster.responseLen] = '\0';
    }
}

static bool responseComplete(const char* s) {
    const size_t len = strlen(s);
    if (len >= 4 && strstr(s, "\r\n\r\n") != nullptr) return true;
    if (strncmp(s, "ICY 200 OK", 10) == 0 && strstr(s, "\r\n") != nullptr) return true;
    if (len >= 14 && strncmp(s, "ERROR", 5) == 0 && strstr(s, "\r\n") != nullptr) return true;
    if (len >= 18 && strncmp(s, "SOURCETABLE", 11) == 0 && strstr(s, "ENDSOURCETABLE") != nullptr) return true;
    return false;
}

static bool sourceUploadAccepted(const char* s) {
    if (strncmp(s, "SOURCETABLE", 11) == 0) return false;
    if (strncmp(s, "ICY 200 OK", 10) == 0) return true;
    if (strncmp(s, "HTTP/1.1 200", 12) == 0 || strncmp(s, "HTTP/1.0 200", 12) == 0) return true;
    return false;
}

// Summarize a REJECTED handshake response into a short dashboard reason. Pulls the
// HTTP status line (e.g. "401 Unauthorized" → bad mountpoint password; "409
// Conflict" → mountpoint already in use), or flags a SOURCETABLE/ERROR reply (the
// caster answered with its source list instead of accepting our push — usually a
// wrong/unregistered mountpoint). dst should be >= 48 bytes.
static void summarizeRejectReason(char* dst, size_t dstSize, const char* resp) {
    if (resp == nullptr || resp[0] == '\0') { copyLimited(dst, dstSize, "rejected: no response"); return; }
    if (strncmp(resp, "SOURCETABLE", 11) == 0) { copyLimited(dst, dstSize, "rejected: got source table (bad mount?)"); return; }
    if (strncmp(resp, "ERROR", 5) == 0)        { copyLimited(dst, dstSize, "rejected: caster ERROR"); return; }
    if (strncmp(resp, "HTTP/1.", 7) == 0) {
        // "HTTP/1.x SSS Reason\r\n" → copy "rejected: SSS Reason" up to the CR.
        const char* code = resp + 9;                  // skip "HTTP/1.x "
        char line[40]; size_t i = 0;
        while (code[i] && code[i] != '\r' && code[i] != '\n' && i < sizeof(line) - 1) { line[i] = code[i]; ++i; }
        line[i] = '\0';
        snprintf(dst, dstSize, "rejected: %s", line);
        return;
    }
    // Unknown banner — show its first token so it's at least diagnosable.
    char first[24]; size_t i = 0;
    while (resp[i] && resp[i] != ' ' && resp[i] != '\r' && resp[i] != '\n' && i < sizeof(first) - 1) { first[i] = resp[i]; ++i; }
    first[i] = '\0';
    snprintf(dst, dstSize, "rejected: %s", first);
}

static void sendSourceRequest(NtripTarget& caster) {
    char request[512];
    // NTRIP v1 SOURCE requests take EXACTLY the request line + Source-Agent.
    // DO NOT re-add any of the following — all three were present and all three
    // were wrong:
    //   "Connection: close"  — actively instructs the caster to tear down the
    //        upload after its response. An RTCM push must stay open indefinitely.
    //        rtk2go's parser honours it; Centipede's ignores it, which is why only
    //        rtk2go kept dropping into Error/"Authenticated - waiting RTCM".
    //   "User-Agent" / "Ntrip-Version" — NTRIP *v2* headers. A v1 SOURCE request
    //        has no version negotiation; strict v1 parsers may mis-handle trailing
    //        headers they don't expect.
    // PROVENANCE IN THE AGENT STRING: a caster operator (and our own logs) should be
    // able to see WHERE this base's coordinate came from and how good it is claimed to
    // be, without asking. "survey/1.85m" reads: automatic survey-in, 1.85 m assumed
    // accuracy. Source-Agent is a free-form token per NTRIP v1, so this is spec-safe;
    // keep it ONE header — do not add more (see the comment above).
    char prov[40] = "unsurveyed";
    if (g_savedPositionValid) {
        snprintf(prov, sizeof(prov), "%s/%.2fm",
                 g_savedSource[0] ? g_savedSource : "unknown",
                 (double)g_savedHAcc);
    }
    snprintf(request, sizeof(request),
             "SOURCE %s /%s\r\n"
             "Source-Agent: NTRIP ESP32S3-Base/1.0 (%s)\r\n"
             "\r\n",
             caster.password, caster.mountpoint, prov);
    caster.client.print(request);
}

static void resetCasterTx(NtripTarget& caster) {
    caster.txLen = 0; caster.txFirstMs = 0;
    caster.congestionStartMs = 0; caster.lastBackpressureMs = 0;
    caster.connBytesOffered = 0; caster.connBytesAccepted = 0;
}

static void connectCaster(NtripTarget& caster) {
    if (!wifiLinkUp()) {                  // DEBOUNCED — see updateWifiLinkState()
        setCasterState(caster, CasterState::WaitingForWifi);
        return;
    }
    caster.client.stop();
    caster.client.setTimeout(1);
    caster.responseLen = 0;
    caster.responseHeader[0] = '\0';
    caster.lastAttemptMs = millis();
    setCasterState(caster, CasterState::Connecting);
    // DNS IS NOT BOUNDED BY THE CONNECT TIMEOUT. NetworkClient::connect(const char*,...)
    // calls Network.hostByName() FIRST and only passes timeout_ms to the TCP handshake
    // that follows; the lookup itself runs to lwIP's own retry policy (can be 10-20 s on
    // a slow or unanswered query) and would block this single-threaded loop for all of it.
    // Resolve ONCE, cache the address, and connect by IP thereafter. Re-resolve only after
    // repeated failures on the cached IP, so a genuine DNS change is still picked up.
    if (caster.resolvedIp == IPAddress((uint32_t)0) ||
        caster.failCount >= CASTER_RERESOLVE_AFTER_FAILS) {
        IPAddress ip;
        if (!WiFi.hostByName(caster.host, ip)) {
            stopCaster(caster, CasterState::Error, "DNS lookup failed");
            caster.lastAttemptMs = millis();
            return;
        }
        caster.resolvedIp = ip;
    }
    if (!caster.client.connect(caster.resolvedIp, caster.port, NTRIP_CONNECT_TIMEOUT_MS)) {
        stopCaster(caster, CasterState::Error, "TCP connect failed (host down / banned?)");
        return;
    }
    caster.tcpConnected = true;
    // TCP_NODELAY (keep): we write ONE small RTCM frame per write() at ~5 Hz. With
    // Nagle enabled those sit in the stack waiting for an ACK of the previous
    // segment, and the classic Nagle/delayed-ACK interaction adds hundreds of ms
    // per frame and lets the send buffer back up — which is what pushes write()
    // into its blocking retry loop. Corrections are latency-critical; send now.
    caster.client.setNoDelay(true);
    // SO_KEEPALIVE: without it a silently-dead peer leaves client.connected()==true
    // forever and only the uplink watchdog notices. Probe after 30 s idle.
    {
        const int fd = caster.client.fd();
        if (fd >= 0) {
            int on = 1, idle = 30, intvl = 10, cnt = 3;
            setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE,  &on,    sizeof(on));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,  sizeof(idle));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,   sizeof(cnt));
        }
    }
    sendSourceRequest(caster);
    setCasterState(caster, CasterState::AwaitingResponse);
}

static void serviceCaster(NtripTarget& caster) {
    const uint32_t now = millis();
    if (!wifiLinkUp()) {                  // DEBOUNCED — see updateWifiLinkState()
        stopCaster(caster, CasterState::WaitingForWifi);
        return;
    }
    if (!caster.client.connected()) {
        if (caster.state == CasterState::AwaitingResponse || caster.state == CasterState::Authenticated || caster.state == CasterState::Streaming) {
            // Named separately when it happens straight after acceptance and before a
            // single byte went out: that is the caster closing an idle source, not a
            // network fault, and it points at a different fix. Silent until now, which
            // is why a session that failed exactly this way had to be reconstructed
            // from frame dispositions in the SD log.
            const bool droppedIdle = caster.handshakeAccepted && caster.bytesAccepted == 0;
            const char* why = droppedIdle ? "caster accepted then closed before any data"
                                          : "connection dropped by caster";
            logEvent("fail", "%s/%s: %s (up %lu ms, offered=%llu accepted=%llu)",
                          caster.host, caster.mountpoint, why,
                          (unsigned long)(now - caster.stateStartMs),
                          (unsigned long long)caster.bytesOffered,
                          (unsigned long long)caster.bytesAccepted);
            stopCaster(caster, CasterState::Error, why);
            caster.lastAttemptMs = now;
        }
        // Held belongs in this set alongside the other not-connected states. A caster the
        // gate released still has no socket, and this is the only branch that opens one —
        // omitting it would leave a released caster parked forever with nothing to drive
        // it back to Connecting. Held never escalates failCount (stopCaster only counts
        // Error), so waiting out a survey costs no backoff.
        if (caster.state == CasterState::Error || caster.state == CasterState::WaitingForWifi ||
            caster.state == CasterState::Disabled || caster.state == CasterState::Held) {
            // Exponential backoff: 30 s base, doubling per consecutive failure,
            // capped at NTRIP_RECONNECT_MAX_MS. A healthy first reconnect waits
            // 30 s (ban-safe even for rtk2go's strict server-side thresholds);
            // a persistently-rejected push decays to one attempt per 10 min
            // instead of hammering the caster until the IP gets banned.
            const uint8_t shift = caster.failCount > 5 ? 5 : caster.failCount;
            uint32_t backoff = caster.reconnectBaseMs << shift;
            if (backoff > NTRIP_RECONNECT_MAX_MS) backoff = NTRIP_RECONNECT_MAX_MS;
            if (now - caster.lastAttemptMs >= backoff) {
                // BUFFERING-SAFETY: client.connect() blocks up to
                // NTRIP_CONNECT_TIMEOUT_MS (1200 ms); a dead/banned caster blocks
                // the FULL timeout with zero GNSS draining. Two back-to-back
                // connects = 2.4 s of shredded RTCM — that's how a failing rtk2go
                // push punches dropouts into a healthy Centipede push. Spend at
                // most one connect attempt per loop pass; the other caster waits
                // one pass (a few ms).
                if (!connectAttemptedThisPass) {
                    connectAttemptedThisPass = true;
                    connectCaster(caster);
                }
            }
        }
        return;
    }
    if (caster.state == CasterState::AwaitingResponse) {
        while (caster.client.available() > 0) {
            appendResponseChar(caster, static_cast<char>(caster.client.read()));
        }
        if (responseComplete(caster.responseHeader)) {
            copyLimited(caster.lastResponseHeader, sizeof(caster.lastResponseHeader), caster.responseHeader);
            if (sourceUploadAccepted(caster.responseHeader)) {
                caster.handshakeAccepted = true;
                caster.lastError[0] = '\0';   // the only thing that clears a past rejection
                setCasterState(caster, CasterState::Authenticated);
                // Send whatever was staged during the handshake straight away. This is the
                // point of staging: a caster that drops silent sources gets data from us
                // within milliseconds of accepting, not at the next epoch boundary.
                if (caster.txLen > 0) flushCasterTx(caster, (size_t)(&caster - casters));
            } else {
                char why[56];
                summarizeRejectReason(why, sizeof(why), caster.responseHeader);
                // The caster's verbatim banner says far more than the summary can, and is
                // the difference between "rejected" and "which of mount, password or
                // already-in-use". Printed unconditionally: a rejection is rare and is
                // exactly what someone watching the serial port is there for.
                logEvent("fail", "%s/%s rejected the SOURCE upload: %s",
                              caster.host, caster.mountpoint, why);
                logEvent("fail", "  caster said: %s", caster.responseHeader);
                stopCaster(caster, CasterState::Error, why);
                caster.lastAttemptMs = now;
            }
        } else if (now - caster.stateStartMs > NTRIP_RESPONSE_TIMEOUT_MS || caster.responseLen >= sizeof(caster.responseHeader) - 1) {
            copyLimited(caster.lastResponseHeader, sizeof(caster.lastResponseHeader), caster.responseHeader);
            stopCaster(caster, CasterState::Error, "no handshake response (timeout)");
            caster.lastAttemptMs = now;
        }
    }
    // UPLINK WATCHDOG: a half-open socket (caster restart, NAT/idle timeout) keeps
    // client.connected()==true while nothing actually leaves the device. If we're
    // STILL parsing fresh RTCM but framesWritten has stalled, the uplink is dead —
    // force a reconnect instead of silently going dark. Guards:
    //   - lastWriteMs != 0           : we've actually streamed at least one frame
    //   - fresh RTCM (<2 s)          : don't blame the uplink when the LG290P
    //                                  itself stopped producing corrections
    //                                  (receiver problem, not ours).
    //   - NOT merely congested       : if the send gate is actively reporting the
    //         socket unwritable, the peer is ALIVE and applying backpressure — that is
    //         congestion, not a half-open socket, and tearing the connection down is
    //         the wrong response. Field session 0453 caught exactly this: at 04:13:46
    //         the socket went unwritable for ~15 s with NO loop stall (row cadence held
    //         at 5.00 s), the gate correctly dropped frames, and then this watchdog
    //         killed a perfectly live connection at age 12 s. Only escalate to a
    //         reconnect if congestion PERSISTS past NTRIP_CONGESTION_MAX_MS.
    const bool congested = caster.congestionStartMs != 0 &&
                           (now - caster.lastBackpressureMs) < 3000 &&
                           (now - caster.congestionStartMs) < NTRIP_CONGESTION_MAX_MS;
    //   - AND we must actually be BEHIND: RTCM was produced that we then failed to
    //         send. The old "lastValidMs is fresh (<2 s)" guard was meant to prove the
    //         receiver was still feeding us, but it is too weak — a TRICKLE keeps
    //         lastValidMs fresh while lastWriteMs ages past the 8 s limit, so the
    //         watchdog blamed the socket for the RECEIVER going quiet. Field session
    //         0454, 14:49:59Z: the LG290P dropped from 5 fps to 0-1 fps for ~45 s with
    //         29 sats and a valid fix; a few frames still trickled through, lastValidMs
    //         stayed fresh, and this watchdog tore down a healthy Centipede connection
    //         at age 11 s. 59 such receiver-side dropouts occurred that session.
    //         Comparing the two timestamps directly is exact: if lastWriteMs is at or
    //         after lastValidMs we have sent everything we were given and there is no
    //         uplink stall, however long ago that was.
    const bool behind = rtcmStats.lastValidMs != 0 &&
                        (int32_t)(rtcmStats.lastValidMs - caster.lastWriteMs) > 2000;
    if (caster.state == CasterState::Streaming &&
        caster.lastWriteMs != 0 &&
        now - caster.lastWriteMs > NTRIP_UPLINK_STALL_MS &&
        !congested &&
        behind &&
        now - rtcmStats.lastValidMs < 2000) {
        stopCaster(caster, CasterState::Error, "uplink stalled (half-open socket)");
        caster.lastAttemptMs = now;
        return;
    }

    // AUTH-STALL WATCHDOG (anti-hammer, applies to BOTH casters): the caster took the
    // handshake, but NTRIP_AUTH_STREAM_TIMEOUT_MS later this connection has still not
    // carried a single frame, while fresh RTCM is being produced. Drop it and let the
    // exponential backoff space out the next attempt instead of re-pushing every epoch.
    //
    // What this condition does and does NOT establish: wroteValidRtcm is set the instant
    // any client.write() completes in full, so !wroteValidRtcm means not one byte was
    // ever successfully written. It therefore cannot mean the caster received our data
    // and discarded it — there was no data for it to discard. The stall is on this side
    // of the socket, and the four counters below say which part of it, so the reason
    // reported names the actual failure instead of assigning blame the evidence does not
    // support. bytesOffered counts only what got PAST the writability gate; txLen is what
    // is still sitting in the coalescing buffer; congestionStartMs is set the first time
    // casterWritable() said no and cleared on the first success.
    if (caster.state == CasterState::Authenticated &&
        !caster.wroteValidRtcm &&
        now - caster.stateStartMs > NTRIP_AUTH_STREAM_TIMEOUT_MS &&
        rtcmStats.lastValidMs != 0 &&
        now - rtcmStats.lastValidMs < 2000) {
        const char* why;
        if (caster.connBytesAccepted > 0)      why = "handshake ok, stream stalled after first write";
        else if (caster.connBytesOffered > 0)  why = "handshake ok, socket accepted no bytes";
        else if (caster.congestionStartMs != 0) why = "handshake ok, socket never became writable";
        else if (caster.txLen > 0)             why = "handshake ok, frames buffered but never flushed";
        else                                   why = "handshake ok, no RTCM reached the caster path";
        logEvent("fail", "%s/%s: %s", caster.host, caster.mountpoint, why);
        logEvent("fail", "  this connection: offered=%lu accepted=%lu | lifetime offered=%llu "
                      "accepted=%llu | buffered=%u dropped=%lu congested=%s rtcm=%lu fps",
                      (unsigned long)caster.connBytesOffered,
                      (unsigned long)caster.connBytesAccepted,
                      (unsigned long long)caster.bytesOffered,
                      (unsigned long long)caster.bytesAccepted,
                      (unsigned)caster.txLen,
                      (unsigned long)caster.droppedWriteCount,
                      caster.congestionStartMs ? "yes" : "no",
                      (unsigned long)rtcmStats.framesPerSecond);
        stopCaster(caster, CasterState::Error, why);
        caster.lastAttemptMs = now;
        return;
    }

    // TIME-BASED FLUSH: push a partially-filled coalescing buffer once it has been
    // sitting for CASTER_TX_MAX_HOLD_MS, so a slow trickle can never be held back
    // long enough to look like an uplink stall. Bounded hold, bounded latency.
    if (caster.txLen > 0 && caster.txFirstMs != 0 &&
        (now - caster.txFirstMs) >= CASTER_TX_MAX_HOLD_MS) {
        flushCasterTx(caster, (size_t)(&caster - &casters[0]));
    }

    while (caster.client.available() > 0 && (caster.state == CasterState::Authenticated || caster.state == CasterState::Streaming)) {
        caster.client.read();
    }
}

// Zero-timeout writability poll on the caster socket. NEVER blocks: select() is
// given a {0,0} timeval, so this is a pure "can I write right now?" question. This
// is the guard that keeps a non-draining caster from freezing the GNSS drain.
static bool casterWritable(NtripTarget& caster) {
    const int fd = caster.client.fd();
    if (fd < 0) return false;
    fd_set wset;
    FD_ZERO(&wset);
    FD_SET(fd, &wset);
    // BOUNDED, not zero. A pure {0,0} poll was too strict: lwIP's write-readiness
    // flag is re-armed on ACK reception, so during normal delayed-ACK cadence (and
    // any single retransmit) the socket reads "not writable" for a few tens of ms
    // even with a nearly-empty send buffer. Field session 0450 dropped 1.39% of
    // frames that way, and the drops tracked FRAME COUNT (r=0.42) rather than byte
    // volume (r=0.15) — the signature of per-segment ACK timing, not congestion.
    // 30 ms rides out that jitter while still capping the worst case at ~1/300th of
    // NetworkClient::write()'s 10 s retry ceiling. Do NOT raise this into the
    // hundreds of ms: at 5-10 frames/s that would re-saturate the loop.
    struct timeval tv = { 0, CASTER_WRITE_POLL_US };
    const int r = select(fd + 1, nullptr, &wset, nullptr, &tv);
    return (r > 0) && FD_ISSET(fd, &wset);
}

// Push whatever is sitting in the coalescing buffer. Returns false on a genuine
// socket failure (caller tears the connection down); true means "buffer is empty
// now, or we deliberately held it back because the socket isn't writable".
static bool flushCasterTx(NtripTarget& caster, size_t casterIndex) {
    // Nothing reaches the socket until the caster has accepted the SOURCE upload. Frames
    // may be STAGED before that point (see sendFrameToCaster) but must never be sent:
    // bytes ahead of the handshake response would be read as part of the request.
    if (!caster.handshakeAccepted) return true;
    if (caster.txLen == 0) return true;
    const uint32_t now = millis();
    // POLL RATE LIMIT (keep). casterWritable() blocks up to CASTER_WRITE_POLL_US. While
    // a socket is congested EVERY buffered frame would re-poll, so a 15 s congestion
    // run x 2 casters would spend hundreds of ms per second blocked in select() — a
    // smaller version of the very stall this gate exists to prevent. Once we know the
    // socket is unwritable, don't re-ask more than once per CASTER_RECHECK_MS.
    if (caster.congestionStartMs != 0 &&
        (uint32_t)(now - caster.lastBackpressureMs) < CASTER_RECHECK_MS) {
        return true;                      // still congested as far as we know; hold
    }
    if (!casterWritable(caster)) {
        // Congested, NOT dead. Hold the buffer; if it can't be sent before it fills
        // we drop the OLDEST content, since stale corrections are worthless anyway.
        caster.lastBackpressureMs = now;
        if (caster.congestionStartMs == 0) caster.congestionStartMs = now;
        return true;
    }
    caster.congestionStartMs = 0;
    caster.bytesOffered += caster.txLen;
    caster.connBytesOffered += caster.txLen;
    const size_t want    = caster.txLen;
    const size_t written = caster.client.write(caster.txBuf, want);
    caster.bytesAccepted += written;
    caster.connBytesAccepted += (uint32_t)written;
    caster.txLen     = 0;
    caster.txFirstMs = 0;
    if (written == want) {
        caster.lastWriteMs    = now;         // feeds the uplink stall watchdog
        caster.wroteValidRtcm = true;
        if (caster.firstWriteMs == 0) { caster.firstWriteMs = now; caster.bwWindowStartMs = now; }
        caster.bwWindowBytes += want;
        if (caster.state == CasterState::Authenticated) setCasterState(caster, CasterState::Streaming);
        return true;
    }
    if (written > 0) caster.partialWriteCount++; else caster.failedWriteCount++;
    stopCaster(caster, CasterState::Error,
               (written > 0) ? "partial write (socket error)" : "socket write failed");
    caster.lastAttemptMs = now;
    return false;
}

static const char* sendFrameToCaster(NtripTarget& caster, size_t casterIndex, const uint8_t* frame, size_t length) {
    if (!caster.client.connected()) return "disconnected";

    // Frames arriving BEFORE the handshake completes are STAGED, not discarded.
    //
    // RTCM leaves this base in one burst per epoch, so between the handshake landing and
    // the next burst there is up to a full second in which a freshly authenticated
    // connection sends nothing at all. Discarding those frames guaranteed that window.
    // Field session 0043: crtk accepted the SOURCE upload and closed the socket inside
    // that same second — the next burst, 1008 ms later, already reported the socket gone,
    // and the caster's own counters read offered=0 accepted=0 buffered=0 while the base
    // was producing a clean 3 fps. A caster that drops an idle source cannot be beaten by
    // a design that is always idle for its first epoch.
    //
    // Staging costs nothing: the bytes go into the same coalescing buffer they would use
    // anyway, and flushCasterTx() is still gated on handshakeAccepted below, so nothing
    // reaches the socket before the caster has accepted us. On acceptance, serviceCaster()
    // flushes immediately and the first correction is out in milliseconds instead of
    // whenever the next epoch happens to arrive.
    const bool staging = !caster.handshakeAccepted;
    if (staging && caster.state != CasterState::AwaitingResponse) return "not-auth";

    // BUFFERING-SAFETY: never let a full TCP send buffer block the loop. RTCM is
    // real-time — a correction we can't send RIGHT NOW is stale — so DROP the
    // frame on backpressure rather than stall the loop (which would stop draining
    // the GNSS UART and shred the frames queued behind this one). Gating on a
    // WHOLE frame's worth of space also guarantees writes are never partial, so we
    // can't inject a truncated RTCM frame and desync the caster's parser.
    // Occasionally dropping a 1 Hz correction is harmless — the rover uses the
    // next epoch. Blocking the pipeline to force one frame through is not.
    // DO NOT reinstate availableForWrite() as the gate. It returns 0 both for an
    // EMPTY send buffer and for a FULL one, so the two states are indistinguishable.
    // Field logs showed droppedWriteCount == 0 across 15,800 rows — it never fired
    // once, and every frame went into a blocking write. NetworkClient::write() retries
    // 10x on a 1 s select(), so a peer that stops draining froze this loop for 10 s,
    // starved processGnssSerial(), and overflowed the GNSS UART for BOTH casters.
    //
    // We now COALESCE instead of writing per frame. Why this matters: with
    // TCP_NODELAY set, one write() per RTCM frame produces one TCP SEGMENT per frame.
    // lwIP caps queued segments (TCP_SND_QUEUELEN) independently of byte count, so a
    // burst of small frames exhausts the segment queue while the send buffer is nearly
    // empty. That is exactly what the field data showed: drops tracked FRAME COUNT
    // (r=0.42 in 0450, 0.23 in 0453) and NOT byte volume (r=0.15, 0.01). Batching into
    // one sub-MSS write collapses a whole burst into a single segment.
    if (length > sizeof(caster.txBuf)) return "oversize";     // never split a frame
    if (caster.txLen + length > sizeof(caster.txBuf)) {
        if (!flushCasterTx(caster, casterIndex)) return "failed";
        if (caster.txLen + length > sizeof(caster.txBuf)) {
            // Still no room: the socket is congested and the buffer is full. Drop the
            // BUFFERED (older) bytes, not the new frame — a correction we could not
            // send is stale, and the newest epoch is the one the rover can use.
            caster.droppedWriteCount += (uint32_t)(caster.txLen / 64 + 1);
            caster.txLen = 0;
            caster.txFirstMs = 0;
        }
    }
    memcpy(caster.txBuf + caster.txLen, frame, length);
    caster.txLen += length;
    if (caster.txFirstMs == 0) caster.txFirstMs = millis();

    // Flush when the buffer is over half full, so a normal ~5 Hz stream still leaves
    // promptly. Time-based flushing for a slow trickle happens in serviceCaster().
    if (caster.txLen >= sizeof(caster.txBuf) / 2) {
        if (!flushCasterTx(caster, casterIndex)) return "failed";
    }
    caster.framesWritten++;
    if (casterIndex < 2) appendCapture(casterTxCaptures[casterIndex], frame, length);
    return "sent";
}

// ── RTCM detail decoder (for the "detailed" troubleshooting CSV) ─────────────
// Pulls human-meaningful fields out of the bitstream so the detailed log shows
// WHAT we cast, not just "Sent": for MSM observations the station ID, GNSS epoch
// time, and satellite/signal counts; for 1005/1006 the reference ECEF position and
// antenna height; for ephemeris the satellite ID. Used ONLY for the opt-in detailed
// CSV (troubleshooting), so the cost is irrelevant on the hot path when it's off.
//
// Bit indexing is into the RTCM PAYLOAD (starts at frame[3]); bit 0 = MSB of the
// first payload byte, matching RTCM 10403.3 DF numbering.
static uint64_t rtcmGetBits(const uint8_t* payload, size_t bitPos, uint8_t bitLen) {
    uint64_t v = 0;
    for (uint8_t i = 0; i < bitLen; ++i) {
        const size_t bit = bitPos + i;
        const uint8_t byte = payload[bit >> 3];
        const uint8_t mask = 0x80 >> (bit & 7);
        v = (v << 1) | ((byte & mask) ? 1 : 0);
    }
    return v;
}

static int64_t rtcmGetBitsSigned(const uint8_t* payload, size_t bitPos, uint8_t bitLen) {
    uint64_t v = rtcmGetBits(payload, bitPos, bitLen);
    if (bitLen < 64 && (v & (1ULL << (bitLen - 1))))        // sign-extend
        v |= ~((1ULL << bitLen) - 1);
    return static_cast<int64_t>(v);
}

static uint8_t rtcmPopcount64(uint64_t x) {
    uint8_t c = 0; while (x) { c += (uint8_t)(x & 1); x >>= 1; } return c;
}

// Fill `out` with a decoded, comma-safe (uses ';' internally) detail string for the
// detailed CSV. `frame` includes the 3-byte header; payload begins at frame[3].
static void decodeRtcmDetail(char* out, size_t outSize, uint16_t type,
                             const uint8_t* frame, size_t length) {
    out[0] = '\0';
    if (length < 6) return;
    const uint8_t* p = frame + 3;                 // payload
    const size_t payloadBits = (length - 6) * 8;  // total minus hdr(3)+crc(3)

    if (isMsmObservation(type)) {
        // MSM header: DF002(12) DF003 stnID(12) epoch(30) DF393(1) DF409 IODS(3)
        // reserved(7) DF001(2) DF411(2) DF412(1) DF417(3) DF418(3) then satMask(64),
        // sigMask(32). (epoch is 30 bits for GPS/QZSS/Galileo/BDS; GLONASS packs a
        // 3-bit day + 27-bit tod = 30, so 30 is correct for the offset math here.)
        const size_t need = 12 + 12 + 30 + 1 + 3 + 7 + 2 + 2 + 1 + 3 + 3 + 64 + 32;
        if (payloadBits < need) return;
        size_t b = 12;
        const uint32_t stnId = (uint32_t)rtcmGetBits(p, b, 12); b += 12;
        const uint32_t epoch = (uint32_t)rtcmGetBits(p, b, 30); b += 30;
        b += 1 + 3 + 7 + 2 + 2 + 1 + 3 + 3;       // skip flags to the satellite mask
        const uint64_t satMask = rtcmGetBits(p, b, 64); b += 64;
        const uint32_t sigMask = (uint32_t)rtcmGetBits(p, b, 32);
        snprintf(out, outSize, "stn=%lu;epoch=%lu;sats=%u;sigs=%u",
                 (unsigned long)stnId, (unsigned long)epoch,
                 rtcmPopcount64(satMask), rtcmPopcount64((uint64_t)sigMask));
        return;
    }

    if (type == 1005 || type == 1006) {
        // DF002(12) DF003 stnID(12) DF021 ITRF(6) flags(4) DF025 X(38) (2) DF026 Y(38)
        // (2) DF027 Z(38) [1006: DF028 antenna height(16)].
        const size_t need = 12 + 12 + 6 + 4 + 38 + 2 + 38 + 2 + 38;
        if (payloadBits < need) return;
        size_t b = 12;
        const uint32_t stnId = (uint32_t)rtcmGetBits(p, b, 12); b += 12;
        b += 6 + 4;                                // ITRF realization + indicator flags
        const int64_t ex = rtcmGetBitsSigned(p, b, 38); b += 38;
        b += 2; const int64_t ey = rtcmGetBitsSigned(p, b, 38); b += 38;
        b += 2; const int64_t ez = rtcmGetBitsSigned(p, b, 38); b += 38;
        const double X = ex * 1e-4, Y = ey * 1e-4, Z = ez * 1e-4;   // 0.0001 m resolution
        if (type == 1006 && payloadBits >= b + 16) {
            const uint32_t h = (uint32_t)rtcmGetBits(p, b, 16);     // DF028, 0.0001 m
            snprintf(out, outSize, "stn=%lu;X=%.4f;Y=%.4f;Z=%.4f;antH=%.4f",
                     (unsigned long)stnId, X, Y, Z, h * 1e-4);
        } else {
            snprintf(out, outSize, "stn=%lu;X=%.4f;Y=%.4f;Z=%.4f",
                     (unsigned long)stnId, X, Y, Z);
        }
        return;
    }

    // Ephemeris messages carry a satellite ID right after the 12-bit message number.
    // Widths: 1019 GPS=6, 1020 GLONASS=6, 1042 BDS=6, 1046 Galileo=6, 1044 QZSS=4.
    if (type == 1019 || type == 1020 || type == 1042 || type == 1046 || type == 1044) {
        const uint8_t idBits = (type == 1044) ? 4 : 6;
        if (payloadBits < 12u + idBits) return;
        const uint32_t satId = (uint32_t)rtcmGetBits(p, 12, idBits);
        snprintf(out, outSize, "sat=%lu", (unsigned long)satId);
        return;
    }

    if (type == 1033 || type == 1008) {           // antenna/receiver descriptor strings
        // Decode the antenna descriptor length. Rovers apply antenna phase-centre
        // corrections from an ANTEX table keyed on this string; an empty one means no
        // PCO/PCV correction is applied and the rover carries a systematic vertical bias
        // of a few centimetres against this base. The field is early and fixed-position
        // in both messages — message number (12), reference station ID (12), then an
        // 8-bit character count — so this needs no assumption about the rest of the
        // layout. Verified against a captured stream: the receiver fields decode to
        // sensible strings while this count reads zero.
        if (payloadBits >= 32u) {
            const uint32_t n = (uint32_t)rtcmGetBits(p, 24, 8);
            g_antennaDescLen = (uint8_t)(n > 31 ? 31 : n);
            g_antennaDescSeen = true;
            for (uint32_t i = 0; i < g_antennaDescLen; ++i)
                g_antennaDesc[i] = (char)rtcmGetBits(p, 32 + 8 * i, 8);
            g_antennaDesc[g_antennaDescLen] = '\0';
            snprintf(out, outSize, "ant=%u", (unsigned)n);
        } else {
            snprintf(out, outSize, "descriptor");
        }
        return;
    }
    // Unknown/other: leave detail blank (type+len in the CSV already identify it).
}

// ── LOCAL NTRIP CASTER (serve rovers directly over WiFi) ─────────────────────
// Serves the same validated RTCM stream we push upstream to a rover on the local
// network — the AP the base already runs, or the LAN when a STA uplink exists. This
// is what makes the base usable at a site with no internet at all: a rover joins the
// base's own AP and takes corrections straight from it, no caster in the middle.
//
// RADIO NOTE: the ESP32-S3 has WiFi and Bluetooth LE only — no 802.15.4 radio, so
// Zigbee/Thread are not available on this silicon regardless of firmware.
//
// PROTOCOL: NTRIP v1 server side. A client sends "GET /<mount> HTTP/1.x", we answer
// "ICY 200 OK" and then stream raw RTCM frames. Any mount name is accepted — this
// caster has exactly one stream to give.
//
// ⚠ STABILITY CONTRACT — this shares the loop with the GNSS drain. It must NEVER block.
//   * Writes go through the SAME select()-based writability gate as the upstream
//     casters, and a frame is DROPPED for a client that is not ready. Corrections are
//     time-critical; a stale frame is worthless and a blocked write is what took the
//     uplink down in past sessions.
//   * Header reads are bounded per pass and abandoned on a timeout, so a client that
//     connects and says nothing cannot hold a slot forever.
//   * Slot count is fixed and small; when full, a new connection is closed at once
//     rather than queued.
#ifndef LOCAL_CASTER_PORT
#define LOCAL_CASTER_PORT          2101
#endif
#ifndef LOCAL_CASTER_NAME
// Display name for the built-in caster, everywhere it is shown: the dashboard's caster
// list, the LCD's caster block, and the status JSON. It serves rovers over the base's own
// WiFi — its AP, or the LAN when the base is joined to one — and needs no internet, which
// is what the name is there to say. Rovers connect by address and port; this caster
// accepts any mountpoint, so nothing a rover sends has to match it.
#define LOCAL_CASTER_NAME          "Local-Wifi"
#endif
#ifndef LOCAL_CASTER_MAX_CLIENTS
#define LOCAL_CASTER_MAX_CLIENTS   4
#endif
#ifndef LOCAL_CASTER_HEADER_TIMEOUT_MS
#define LOCAL_CASTER_HEADER_TIMEOUT_MS 5000
#endif

static WiFiServer        g_localCasterServer(LOCAL_CASTER_PORT);
static LocalCasterClient g_localClients[LOCAL_CASTER_MAX_CLIENTS];
static bool              g_localCasterEnabled = true;
static bool              g_localCasterStarted = false;
static uint32_t          g_localCasterServed  = 0;   // lifetime accepted streams

static int localCasterActiveCount() {
    int n = 0;
    for (auto& c : g_localClients) if (c.client.connected() && c.streaming) ++n;
    return n;
}

static void localCasterStop(LocalCasterClient& c) {
    c.client.stop();
    c.streaming   = false;
    c.headerLen   = 0;
    c.header[0]   = '\0';
    c.congestedMs = 0;
    c.ntrip2      = false;
    c.mount[0]    = '\0';
}

// ZERO-TIMEOUT writability poll — deliberately NOT casterWritable()'s 30 ms version.
// casterWritable() waits 30 ms because an upstream caster sits across the internet,
// where delayed-ACK cadence briefly clears the write-ready flag even on a healthy,
// nearly-empty socket (session 0450 lost 1.39% of frames to a zero-timeout poll).
// A local rover is one WiFi hop away: ACKs return in single-digit milliseconds and the
// socket is essentially always writable, so that justification does not carry over.
//
// The cost of copying it here WOULD be severe. This runs per frame per client, so with
// LOCAL_CASTER_MAX_CLIENTS rovers at ~5 frames/s a 30 ms poll is up to 600 ms of the
// loop per second spent inside select() — starving the 460800-baud GNSS drain and
// re-creating exactly the class of stall that shredded RTCM in sessions 0441/0442. A
// zero-timeout poll never blocks, and an occasional dropped frame to one local rover
// costs nothing: the next epoch is a second away and corrections are not cumulative.
static bool localClientWritable(WiFiClient& client) {
    const int fd = client.fd();
    if (fd < 0) return false;
    fd_set wset;
    FD_ZERO(&wset);
    FD_SET(fd, &wset);
    struct timeval tv = { 0, 0 };       // pure poll — never blocks the loop
    const int r = select(fd + 1, nullptr, &wset, nullptr, &tv);
    return (r > 0) && FD_ISSET(fd, &wset);
}

static void localCasterBegin() {
    if (g_localCasterStarted || !g_localCasterEnabled) return;
    g_localCasterServer.begin();
    g_localCasterServer.setNoDelay(true);
    g_localCasterStarted = true;
    Serial.printf("📡 %s caster listening on port %d\n", LOCAL_CASTER_NAME, LOCAL_CASTER_PORT);
}

static void localCasterEnd() {
    for (auto& c : g_localClients) localCasterStop(c);
    if (g_localCasterStarted) {
        g_localCasterServer.end();
        g_localCasterStarted = false;
        Serial.printf("📡 %s caster stopped\n", LOCAL_CASTER_NAME);
    }
}

// Pull the mountpoint and the requested NTRIP version out of a completed request.
// Only the request line and one header are needed, so this does not need to be a general
// HTTP parser — but it does need to be tolerant, because clients vary in spacing, case
// and line endings, and a parser that is strict about any of those rejects real rovers.
static void parseLocalCasterRequest(LocalCasterClient& c) {
    c.mount[0] = '\0';
    c.ntrip2   = false;

    // Request line: GET <path> HTTP/x.y — take the path, minus its leading slash.
    const char* p = c.header + 3;
    while (*p == ' ') ++p;
    if (*p == '/') ++p;
    size_t n = 0;
    while (*p && *p != ' ' && *p != '\r' && *p != '\n' && n < sizeof(c.mount) - 1)
        c.mount[n++] = *p++;
    c.mount[n] = '\0';

    // Ntrip-Version: Ntrip/2.0 — matched case-insensitively, since the header name's case
    // is not guaranteed by the spec and clients do differ.
    for (const char* q = c.header; *q; ++q) {
        if ((*q == 'N' || *q == 'n') && strncasecmp(q, "Ntrip-Version:", 14) == 0) {
            if (strstr(q, "2.0") != nullptr) c.ntrip2 = true;
            break;
        }
    }
}

// Answer a sourcetable request with this base's single mountpoint, then close — which is
// what the protocol requires: a sourcetable response is complete in itself.
static void sendLocalSourcetable(LocalCasterClient& c) {
    char body[320];
    const int written = snprintf(body, sizeof(body),
        "STR;%s;%s;RTCM 3.3;1005(10),1074(1),1084(1),1094(1),1124(1),1230(10);"
        "2;GNSS;%s;USA;%.4f;%.4f;0;0;LG290P;none;N;N;9600;\r\n"
        "ENDSOURCETABLE\r\n",
        g_deviceName.c_str(), g_deviceName.c_str(), LOCAL_CASTER_NAME,
        ggaLat, ggaLon);
    // Clamp to what was actually written. Announcing a length the body does not have
    // leaves the client waiting for bytes that will never arrive.
    const int bodyLen = (written < 0) ? 0
                      : (written >= (int)sizeof(body) ? (int)sizeof(body) - 1 : written);
    char head[200];
    snprintf(head, sizeof(head),
        "%s\r\nServer: %s\r\nContent-Type: gnss/sourcetable\r\n"
        "Content-Length: %d\r\nConnection: close\r\n\r\n",
        c.ntrip2 ? "HTTP/1.1 200 OK\r\nNtrip-Version: Ntrip/2.0" : "SOURCETABLE 200 OK",
        LOCAL_CASTER_NAME, bodyLen);
    c.client.print(head);
    c.client.write((const uint8_t*)body, (size_t)bodyLen);
    c.client.flush();   // the caller closes immediately; do not lose the body to it
}

// Accept new connections and complete pending NTRIP handshakes. Called once per loop
// pass; every operation here is non-blocking or bounded.
static void serviceLocalCaster() {
    if (!g_localCasterEnabled) { if (g_localCasterStarted) localCasterEnd(); return; }
    if (!g_localCasterStarted) localCasterBegin();
    if (!g_localCasterStarted) return;

    if (g_localCasterServer.hasClient()) {
        int slot = -1;
        for (int i = 0; i < LOCAL_CASTER_MAX_CLIENTS; ++i) {
            if (!g_localClients[i].client.connected()) { slot = i; break; }
        }
        WiFiClient incoming = g_localCasterServer.accept();
        if (slot < 0) {
            incoming.stop();          // all slots busy — refuse immediately, never queue
        } else {
            LocalCasterClient& c = g_localClients[slot];
            localCasterStop(c);
            c.client        = incoming;
            c.client.setNoDelay(true);
            c.streaming     = false;
            c.connectedMs   = millis();
            c.framesWritten = 0;
            c.framesDropped = 0;
        }
    }

    const uint32_t now = millis();
    for (auto& c : g_localClients) {
        if (!c.client.connected()) { if (c.streaming) localCasterStop(c); continue; }
        if (c.streaming) {
            while (c.client.available() > 0) c.client.read();   // discard client chatter
            continue;
        }
        // Header phase: read what has arrived, bounded by the buffer, and give up if the
        // client never finishes its request.
        while (c.client.available() > 0 && c.headerLen < sizeof(c.header) - 1) {
            const int ch = c.client.read();
            if (ch < 0) break;
            c.header[c.headerLen++] = (char)ch;
            c.header[c.headerLen]   = '\0';
        }
        if (strstr(c.header, "\r\n\r\n") != nullptr || strstr(c.header, "\n\n") != nullptr) {
            if (strncmp(c.header, "GET", 3) != 0) {
                c.client.print("HTTP/1.0 400 Bad Request\r\n\r\n");
                logEvent("warn", "%s: rejected non-GET request", LOCAL_CASTER_NAME);
                localCasterStop(c);
                continue;
            }
            parseLocalCasterRequest(c);

            // A request for "/" is a SOURCETABLE request, not a stream request. Clients
            // that let the operator pick a mountpoint from a list ask for this first, and
            // answering it with RTCM — which is what happened before, since every GET was
            // treated as a stream request — gives them binary where they expect a table.
            // The client shows no mountpoints and the connection looks broken from the
            // rover end while the base logs a successful connection.
            if (c.mount[0] == '\0') {
                sendLocalSourcetable(c);
                logEvent("ok", "%s: served sourcetable", LOCAL_CASTER_NAME);
                localCasterStop(c);
                continue;
            }

            // Reply in the protocol the client asked for. A strict Ntrip/2.0 client is
            // entitled to reject "ICY 200 OK" — that is the NTRIP 1.0 response, and this
            // caster sent it unconditionally. Clients that accept either were fine;
            // clients that check were not, and there is no way to tell which from the
            // base's side, so it now answers whichever was requested.
            if (c.ntrip2) {
                c.client.print("HTTP/1.1 200 OK\r\n"
                               "Ntrip-Version: Ntrip/2.0\r\n"
                               "Server: " LOCAL_CASTER_NAME "\r\n"
                               "Content-Type: gnss/data\r\n"
                               "Cache-Control: no-store, no-cache, max-age=0\r\n"
                               "Pragma: no-cache\r\n"
                               "Connection: close\r\n\r\n");
            } else {
                c.client.print("ICY 200 OK\r\n\r\n");   // NTRIP 1.0 accept
            }
            c.streaming = true;
            g_localCasterServed++;
            logEvent("ok", "%s: rover connected to /%s as Ntrip/%s (%d active)",
                          LOCAL_CASTER_NAME, c.mount, c.ntrip2 ? "2.0" : "1.0",
                          localCasterActiveCount());
        } else if (c.headerLen >= sizeof(c.header) - 1) {
            // Now genuinely abnormal rather than routine. Logged, because silently
            // dropping a client is what made this invisible for so long.
            c.client.print("HTTP/1.0 431 Request Header Fields Too Large\r\n\r\n");
            logEvent("warn", "%s: request header exceeded %u bytes - client dropped",
                          LOCAL_CASTER_NAME, (unsigned)sizeof(c.header));
            localCasterStop(c);
        } else if ((uint32_t)(now - c.connectedMs) > LOCAL_CASTER_HEADER_TIMEOUT_MS) {
            logEvent("warn", "%s: client sent no complete request in %u ms - dropped",
                          LOCAL_CASTER_NAME, (unsigned)LOCAL_CASTER_HEADER_TIMEOUT_MS);
            localCasterStop(c);
        }
    }
}

// Fan one validated RTCM frame out to every streaming local rover. Drops rather than
// blocks: see the stability contract above.
static void localCasterBroadcast(const uint8_t* frame, size_t length) {
    if (!g_localCasterEnabled || !g_localCasterStarted) return;
    const uint32_t now = millis();
    for (auto& c : g_localClients) {
        if (!c.streaming) continue;
        if (!c.client.connected()) { localCasterStop(c); continue; }
        // POLL RATE LIMIT (keep) — same reasoning as flushCasterTx(). localClientWritable()
        // blocks up to CASTER_WRITE_POLL_US (30 ms). Re-polling every frame for every
        // congested rover would cost up to LOCAL_CASTER_MAX_CLIENTS x 30 ms per frame,
        // which at ~5 frames/s is hundreds of ms per second stolen from the GNSS drain —
        // the exact stall this gate exists to prevent. Once a rover is known unwritable,
        // don't re-ask more often than CASTER_RECHECK_MS.
        if (c.congestedMs != 0 && (uint32_t)(now - c.congestedMs) < CASTER_RECHECK_MS) {
            c.framesDropped++;
            continue;
        }
        if (!localClientWritable(c.client)) {
            c.congestedMs = now;
            c.framesDropped++;
            continue;
        }
        c.congestedMs = 0;
        const size_t w = c.client.write(frame, length);
        if (w == length) {
            c.framesWritten++;
            markWifiActivity();   // RTCM is actually reaching a rover — see its declaration
        }
        else {
            // A partial or failed write on a local socket means the rover is gone or
            // wedged. Drop it rather than carry a desynchronised RTCM stream.
            c.framesDropped++;
            localCasterStop(c);
        }
    }
}

static void handleValidRtcmFrame(const uint8_t* frame, size_t length) {
    const uint16_t type = extractRtcmType(frame, length);
    rtcmStats.rtcmValidFrames++;
    rtcmStats.rtcmValidBytes += length;
    rtcmStats.framesWindow++;
    rtcmStats.bytesWindow += length;
    rtcmStats.lastMsgType = type;
    rtcmStats.lastMsgLength = length;
    rtcmStats.lastValidMs = millis();
    RtcmTypeStat* typeStat = findOrCreateTypeStat(type);
    if (typeStat != nullptr) {
        typeStat->count++;
        typeStat->lastSeenMs = millis();
    }
    appendRtcmValidCapture(frame, length);
    // Push to every enabled caster. The detailed SD log keeps two status columns
    // (slots 0/1); additional casters still stream, their status just isn't
    // broken out in that two-column CSV.
    const char* status0 = "disabled";
    const char* status1 = "disabled";
    for (int i = 0; i < casterCount; ++i) {
        if (!casters[i].enabled) continue;
        const char* st = sendFrameToCaster(casters[i], i, frame, length);
        if (i == 0) status0 = st;
        else if (i == 1) status1 = st;
    }
    localCasterBroadcast(frame, length);   // serve local rovers the same validated frame
    char detail[80];
    decodeRtcmDetail(detail, sizeof(detail), type, frame, length);
    bridge_sdlog_rtcm_frame(type, (uint16_t)length, true, status0, status1, detail, frame);
}

static void resetRtcmParser() {
    rtcmParser.state = RtcmParseState::WaitPreamble;
    rtcmParser.index = 0;
    rtcmParser.expected = 0;
    rtcmParser.payloadLen = 0;
}

static void feedRtcmParserByte(uint8_t b) {
    switch (rtcmParser.state) {
        case RtcmParseState::WaitPreamble:
            if (b == 0xD3) {
                rtcmParser.frame[0] = b;
                rtcmParser.index = 1;
                rtcmParser.state = RtcmParseState::ReadLen1;
            }
            break;
        case RtcmParseState::ReadLen1:
            if ((b & 0xFC) != 0) {
                rtcmStats.rtcmFramingFailures++;
                resetRtcmParser();
                if (b == 0xD3) feedRtcmParserByte(b);
                return;
            }
            rtcmParser.frame[rtcmParser.index++] = b;
            rtcmParser.payloadLen = (static_cast<uint16_t>(b & 0x03) << 8);
            rtcmParser.state = RtcmParseState::ReadLen2;
            break;
        case RtcmParseState::ReadLen2:
            rtcmParser.frame[rtcmParser.index++] = b;
            rtcmParser.payloadLen |= b;
            if (rtcmParser.payloadLen > RTCM_MAX_PAYLOAD_LEN) {
                rtcmStats.rtcmFramingFailures++;
                resetRtcmParser();
                return;
            }
            rtcmParser.expected = 3 + rtcmParser.payloadLen + 3;
            rtcmParser.state = RtcmParseState::ReadFrame;
            break;
        case RtcmParseState::ReadFrame:
            if (rtcmParser.index >= RTCM_MAX_FRAME_LEN) {
                rtcmStats.rtcmFramingFailures++;
                resetRtcmParser();
                if (b == 0xD3) feedRtcmParserByte(b);
                return;
            }
            rtcmParser.frame[rtcmParser.index++] = b;
            if (rtcmParser.index >= rtcmParser.expected) {
                rtcmStats.rtcmCandidateFrames++;
                const size_t crcInputLen = rtcmParser.expected - 3;
                const uint32_t calculated = crc24q(rtcmParser.frame, crcInputLen);
                const uint32_t received = (static_cast<uint32_t>(rtcmParser.frame[crcInputLen]) << 16) |
                                          (static_cast<uint32_t>(rtcmParser.frame[crcInputLen + 1]) << 8) |
                                          static_cast<uint32_t>(rtcmParser.frame[crcInputLen + 2]);
                if (calculated == received) {
                    handleValidRtcmFrame(rtcmParser.frame, rtcmParser.expected);
                } else {
                    rtcmStats.rtcmCrcFailures++;
                    // Log the rejected frame (crc_ok=0) so the detailed RTCM CSV makes
                    // validity auditable instead of implicitly "all valid". Type bits
                    // may be garbage on a corrupt frame — that's expected.
                    bridge_sdlog_rtcm_frame(extractRtcmType(rtcmParser.frame, rtcmParser.expected),
                                            (uint16_t)rtcmParser.expected, false, "crc-fail", "", "", rtcmParser.frame);
                }
                resetRtcmParser();
            }
            break;
    }
}

// Probe the module's SAVED state and report whether it already matches the base
// config we'd otherwise write. If true, the caller skips reconfigure+SAVEPAR+SRR so
// the receiver retains its warm time/ephemeris/position (hot start) — and, if it had
// already finished survey-in, stays fixed for an instant base on reboot.
static bool lg290pBaseConfigMatches() {
    g_probedRcvrMode = -1;
    g_probedSvinMode = 0;
    g_probedSvinDur  = 0;
    g_probedSvinAcc  = 0.0f;
    ggaFixQuality    = 0;        // clear so we measure the CURRENT fix during the probe
    sendPqtm("PQTMCFGRCVRMODE,R");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMCFGSVIN,R");
    // Drain until we have the QUERY REPLIES the hot-start decision actually needs
    // (both RCVRMODE,R and SVIN,R parsed) OR the timeout expires.
    //
    // BUGFIX (field log 2026-06-24): the old early-exit broke the instant ANY UART
    // traffic arrived (e.g. an autonomous 1 Hz PQTMSVINSTATUS). That fired BEFORE the
    // PQTMCFGSVIN,R reply was parsed, so g_probedSvinMode was still 0 → the config
    // check fell through to "unconfigured" → needless reconfigure+reset on a module
    // that was actually correctly configured. We must wait for the replies that set
    // g_probedRcvrMode and g_probedSvinMode, not just for the module to make noise.
    //
    // The 60 s ceiling still handles the antenna-disconnected / module-silent case:
    // if the replies never come (module not really responding to commands), we treat
    // it as cold and reconfigure. A live module answers both queries in well under a
    // second, so the warm path still costs almost nothing.
    const uint32_t probeStartMs = millis();
    uint32_t lastProbeSecShown = 0xFFFFFFFFUL;   // boot progress only; see drawBootSplash
    const uint32_t deadline = millis() + 60000;
    while (millis() < deadline) {
        processGnssSerial();
        // Exit as soon as BOTH query replies have populated the probe state — that's
        // everything the decision below reads.
        if (g_probedRcvrMode != -1 && g_probedSvinMode != 0) break;
        if ((millis() - probeStartMs) / 1000UL != lastProbeSecShown) {
            lastProbeSecShown = (millis() - probeStartMs) / 1000UL;
            char l2[32];
            snprintf(l2, sizeof(l2), "reading config  %lus", (unsigned long)lastProbeSecShown);
            drawBootSplash("Configuring GNSS", l2);
        }
        delay(5);
    }
    // If the receiver mode reply never came, re-poll once near the end in case the
    // first query got lost in a burst of RTCM/autonomous output, then give it a moment.
    if (g_probedRcvrMode == -1 || g_probedSvinMode == 0) {
        sendPqtm("PQTMCFGRCVRMODE,R");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        sendPqtm("PQTMCFGSVIN,R");
        const uint32_t retryDeadline = millis() + 3000;
        while (millis() < retryDeadline) {
            processGnssSerial();
            if (g_probedRcvrMode != -1 && g_probedSvinMode != 0) break;
            delay(5);
        }
    }
    if (g_probedRcvrMode != 2) return false;        // not in base mode → must configure
    if (LG290P_USE_FIXED_BASE) {
        if (g_probedSvinMode != 2) return false;    // fixed base
    } else {
        // Normally a survey-in base. BUT if we have a saved absolute position, last
        // session may have left the module as a FIXED base at that position
        // (svinMode==2, SAVEPAR'd). That's the warm/correct state — accept it so a
        // power-trip reboot doesn't tear it down and re-survey. The displacement check
        // still confirms the venue via the live fix before trusting it.
        if (g_probedSvinMode == 2 && g_savedPositionValid) {
            // fixed at saved position — accepted as configured
        } else if (g_probedSvinMode == 1) {
            // A survey-in-configured base. Do NOT tear it down over a survey-PARAM
            // mismatch (duration/accuracy) when there is warm state worth keeping:
            //   • g_probedSvinMode == 2  → the module is already in FIXED BASE mode.
            //     This is the authoritative warm-state signal: the LG290P can only be
            //     in svinMode 2 if a survey converged and PQTMSAVEPAR was written, so
            //     the position is real and retained across the ESP reset. Accept it.
            //   • g_savedPositionValid   → we have a saved NVS coordinate to validate
            //     against and fall back on regardless of survey params.
            //
            // NOTE: surveyStatus.valid is NOT usable here — it's populated only by
            // parsing live PQTMSVINSTATUS sentences, which haven't arrived yet when
            // this probe runs. g_probedSvinMode comes from PQTMCFGSVIN,R which IS
            // parsed synchronously during the probe window above.
            //
            // FIELD BUG (2026-06-28): surveyInSec was changed 3600 → 900 between
            // sessions. The old unconditional dur/acc check returned false on every
            // battery-backed reboot, forcing reconfigure + PQTMSRR that WIPED the
            // converged base and re-ran a full survey — the opposite of a hot start.
            // Survey params only matter when about to run a NEW survey; when the module
            // is already in fixed base mode (svinMode==2) or we have a saved position,
            // they are irrelevant. The boot confidence check (checkPositionDisplacement)
            // still validates the venue and forces re-survey if the antenna moved.
            const bool warmWorthKeeping = (g_probedSvinMode == 2) || g_savedPositionValid;
            if (!warmWorthKeeping) {
                if (g_probedSvinDur != surveyInSec) return false;
                if (fabsf(g_probedSvinAcc - static_cast<float>(surveyAccLimit)) > 0.05f) return false;
            }
        } else {
            return false;                            // unconfigured/unknown → must configure
        }
    }
    // HOT-START DECISION (rewritten — see field bug below):
    // At this point the module is confirmed already configured as a base (RCVRMODE=2
    // and SVIN mode/params match, or fixed-at-saved-position). That IS the warm state
    // we want to preserve: the LG290P kept its RTC/ephemeris/almanac across the ESP
    // reboot, so leaving it ALONE lets it reacquire fastest (typically < 10 s).
    //
    // OLD BUG: the gate used to be `return ggaFixQuality > 0`, requiring a fix to ALREADY
    // be present within the ~2.5 s probe window. But right after the ESP's UART comes up
    // the first GGA often still reads quality 0 (mid-reacquisition), so the gate failed,
    // forced a full reconfigure + PQTMSRR reset, and that reset WIPED the warm state —
    // turning a ~10 s warm reacquire into a cold start that sat at 0/0 for minutes. The
    // guard meant to prevent a cold start was the thing causing it.
    //
    // NEW RULE: a module that is correctly base-configured AND showing any sign of life on
    // the GNSS UART (we received at least one GGA/SVINSTATUS this probe, fix or not) is
    // treated as hot — preserve it. Only a module that is silent on the UART (no GGA at
    // all → not actually running) falls through to the deterministic reconfigure+reset.
    const bool moduleAlive = (ggaLastSeenMs != 0) || (surveyStatus.lastSeenMs != 0);
    const bool haveLiveFix = (ggaFixQuality > 0);
    const uint32_t waitedMs = millis() - probeStartMs;
    if (haveLiveFix)
        Serial.printf("🔥 Hot start: live fix present after %lums — preserving config\n", (unsigned long)waitedMs);
    else if (moduleAlive)
        Serial.printf("🔥 Hot start: UART alive (no fix yet) after %lums — preserving config, no reset\n", (unsigned long)waitedMs);
    else
        Serial.printf("❄️  No GNSS UART activity in %lums — treating as cold, will reconfigure+reset\n", (unsigned long)waitedMs);
    return moduleAlive;
}

// Enable the PQTM/NMEA telemetry messages this firmware CONSUMES: PQTMSVINSTATUS
// (survey/base-ready), PQTMPVT (datetime+pos), PQTMEPE (error estimate), GGA/RMC
// (fix+date+the venue check's live position), GSV (sat detail). This is NON-DESTRUCTIVE
// — CFGMSGRATE only sets output rates; it does not reset the module, wipe the survey,
// or move the position. Safe to call on a hot-started (warm) base. Must be called only
// AFTER the module is confirmed in base mode (PQTMSVINSTATUS returns ERROR,3 otherwise).
static void enableLg290pTelemetryMessages() {
    sendPqtm("PQTMCFGMSGRATE,W,PQTMSVINSTATUS,1,1");  // MsgVer 1; base-mode-only output
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMCFGMSGRATE,W,PQTMPVT,1,1");          // MsgVer 1 ("always 1" per spec)
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMCFGMSGRATE,W,GGA,1");                // NMEA — no MsgVer (OK)
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    // RMC provides the calendar date (ddmmyy) which GGA lacks.
    sendPqtm("PQTMCFGMSGRATE,W,RMC,1");                // NMEA — no MsgVer (OK)
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
#if BRIDGE_LOG_SD_ENABLE
    sendPqtm("PQTMCFGMSGRATE,W,GSV,1");                // per-satellite detail for sat_*.csv
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
#endif
    // PQTMEPE MsgVer 2 = the 6-field N/E/D/2D/3D form parsePqtmEpe() expects.
    sendPqtm("PQTMCFGMSGRATE,W,PQTMEPE,1,2");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    // NOTE: we intentionally do NOT enable PQTMNAV. It only exists on LG290P(03)+ firmware
    // (spec v1.1), so on earlier modules the enable is rejected and it never flows. The
    // boot position-confidence check reads GGA/PQTMPVT (core messages) instead, so it works
    // on any revision. Leaving PQTMNAV off avoids a spurious ERROR on every boot.
}

// Read back the module's ACTUAL receiver mode and SVIN mode after an escape sequence,
// and report whether it really left base/fixed operation.
//
// WHY THIS EXISTS (field failure, battery-backed retention): the escape sequences in
// this firmware write rover + survey-in targets, PQTMSAVEPAR, then restart — and then
// ASSUMED they worked, stamping g_probedRcvrMode/g_probedSvinMode with the values they
// intended. Those two variables are what every downstream "is the module still a fixed
// base?" guard tests, including the forced-rover switch in checkPositionDisplacement().
// So when an escape silently failed, the one check whose whole purpose is to catch that
// had already been told the escape succeeded. The module stayed a fixed base, echoed its
// stored coordinate into GGA, the venue check compared the saved position against itself,
// found 0 m, and confirmed "un-moved" — regardless of where the base physically was.
//
// PQTMSAVEPAR's acknowledgement is never checked anywhere in this firmware, and the
// LG290P's retained configuration is held in BATTERY-BACKED storage (proven in the
// field: only disconnecting the backup battery AND all power cleared a stuck fixed
// position; PQTMSAVEPAR-based clearing did not). So an escape CAN fail to take while
// every command appears to have been sent. Trust the read-back, never the intent.
//
// Populates g_probedRcvrMode / g_probedSvinMode with what the module REPORTS. Returns
// true only if it is confirmed out of base mode and out of fixed-SVIN mode. On a probe
// timeout it returns false: unknown is treated as "still fixed", because the failure
// this guards against is silent and the conservative branch (re-survey) is cheap
// compared to casting a wrong coordinate.
static bool verifyEscapedFixedBase(const char* context) {
    g_probedRcvrMode = -1;
    g_probedSvinMode = 0;
    sendPqtm("PQTMCFGRCVRMODE,R");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMCFGSVIN,R");
    // Bounded wait for BOTH replies — the same reply-driven pattern as the boot probe,
    // not a bare drain, because autonomous telemetry would otherwise satisfy a naive wait
    // before the config replies land.
    const uint32_t deadline = millis() + LG290P_ESCAPE_VERIFY_MS;
    while (millis() < deadline) {
        processGnssSerial();
        if (g_probedRcvrMode != -1 && g_probedSvinMode != 0) break;
        delay(5);
    }
    const bool replied = (g_probedRcvrMode != -1 && g_probedSvinMode != 0);
    const bool escaped = replied && (g_probedRcvrMode != 2) && (g_probedSvinMode != 2);
    if (!replied) {
        logEvent("warn", "Escape verify (%s): no config reply in %lums - treating as STILL FIXED",
                      context, (unsigned long)LG290P_ESCAPE_VERIFY_MS);
    } else if (!escaped) {
        logEvent("warn", "Escape verify (%s): module STILL reports rcvrMode=%d svinMode=%d",
                      context, g_probedRcvrMode, g_probedSvinMode);
    } else {
        logEvent("ok", "Escape verify (%s): module confirmed rover (rcvrMode=%d svinMode=%d)",
                      context, g_probedRcvrMode, g_probedSvinMode);
    }
    return escaped;
}

// Base-mode output configuration: RTCM message set and rates, port protocols, and the
// PQTM/NMEA telemetry rates.
//
// MUST BE RE-APPLIED AFTER ANY RETURN TO BASE MODE (this is why it is a function rather
// than inline). Per the PQTMCFGRCVRMODE spec, entering base mode "will automatically
// disable NMEA message output and enable RTCM MSM4 and RTCMv3 1005" — the module resets
// its output configuration to base defaults. The PPP survey leaves base mode for rover
// mode and comes back, so without re-applying this the pinned rates (1033/1007 at 10 s,
// the ephemeris set) revert to defaults and the following PQTMSAVEPAR persists them.
// ── Satellite gating: written once at boot, before anything reads a fix ─────
// The elevation and C/N0 masks live in the module's NVM and survive every mode change,
// power cycle and reconfigure. A module that was ever a rover keeps the rover's much
// tighter masks — the 30 deg / 32 dBHz pair that once left this base tracking ten
// satellites — and nothing about becoming a base clears them.
//
// They were previously written only from enableLg290pBaseOutputs(), which is reached
// well into the boot and not at all on some paths. Everything that reads a fix before
// that point — the boot position check, the PPP survey's first samples — was measuring
// through whatever masks the module happened to be carrying. This applies them once,
// early, on every path, then READS THEM BACK: these are exactly the kind of unacked
// write this project has been bitten by, and a mask that silently failed to apply is
// invisible in every downstream symptom.
static void applySatelliteGating() {
    char mb[48];
    snprintf(mb, sizeof(mb), "PQTMCFGELETHD,W,%.1f", (double)BASE_ELEVATION_MASK_DEG);
    sendPqtm(mb);
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    snprintf(mb, sizeof(mb), "PQTMCFGCNRTHD,W,%.1f", (double)BASE_CNR_MASK_DBHZ);
    sendPqtm(mb);
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    g_probedEleMask = -1.0f;
    g_probedCnrMask = -1.0f;
    sendPqtm("PQTMCFGELETHD,R");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMCFGCNRTHD,R");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    const bool eleOk = g_probedEleMask >= 0.0f &&
                       fabs((double)g_probedEleMask - BASE_ELEVATION_MASK_DEG) < 0.05;
    const bool cnrOk = g_probedCnrMask >= 0.0f &&
                       fabs((double)g_probedCnrMask - BASE_CNR_MASK_DBHZ) < 0.05;
    if (eleOk && cnrOk) {
        logEvent("ok", "Satellite gating: elevation %.1f deg, C/N0 %.1f dBHz (confirmed)",
                      (double)g_probedEleMask, (double)g_probedCnrMask);
    } else {
        logEvent("warn", "Satellite gating NOT confirmed - asked for %.1f deg / %.1f dBHz; "
                      "module reports %.1f / %.1f. Stale masks starve every fix from here on.",
                      (double)BASE_ELEVATION_MASK_DEG, (double)BASE_CNR_MASK_DBHZ,
                      (double)g_probedEleMask, (double)g_probedCnrMask);
    }
}

static void enableLg290pBaseOutputs() {
    // Masks are owned by applySatelliteGating(), which runs at boot before any fix is
    // read. Re-asserted here because a full reconfigure is also the point at which a
    // module that was replaced or externally reconfigured gets brought back into line.
    applySatelliteGating();
    // PQTMCFGRTCM fields: <MSM_Type>,<MSM_Mode>,<MSM_Elev_Thresh>,<EPH_Mode>,...
    // The -90 elevation threshold is DELIBERATE and means NO elevation limit on RTCM
    // observations — Quectel's own base-station application note uses exactly this value
    // for "output RTCM MSM4 with no limitation on elevation threshold". Every satellite
    // the receiver tracks is passed to the rover, which is what a base should do: it is
    // the ROVER's job to apply its own mask, and discarding low-elevation observations
    // here would throw away data the rover might want.
    //
    // Effective elevation gating still exists — it is the receiver's own tracking MASK,
    // which defaults to 5 degrees. Do not also set 5 here: that would gate the same
    // satellites twice and could only ever remove observations, never add them.
    sendPqtm("PQTMCFGRTCM,W,4,0,-90,07,06,1,0");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMCFGPROT,W,1,1,00000005,00000005");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    // ── RTCM output message rates ────────────────────────────────────────────
    // FIRMWARE-CONFIRMED against the module's own boot RSP log. The receiver reports its
    // type and firmware in RTCM 1033, so the revision this base is actually running is
    // readable from its own output stream rather than assumed here:
    //   * PQTMCFGMSGRATE accepts only EXPLICIT message numbers. The wildcard forms
    //     "RTCM3-107X/108X/109X/112X" return ERROR,1 and were removed.
    //   * The MSM OBSERVATION set (1074/1084/1094/1124) is enabled by the
    //     PQTMCFGRTCM,W,4 command above (MSM4), NOT per-message here. Those are the
    //     actual corrections a rover needs; do not try to re-enable them via MSGRATE.
    sendPqtm("PQTMCFGMSGRATE,W,RTCM3-1005,10");   // stationary ARP (OK — explicit number)
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    // 1033/1007 are STATIC descriptor strings (receiver + antenna type). They never
    // change while running, so a rover needs them roughly once per 10 s — the same
    // cadence as 1005. Left unset, the LG290P's default emits 1033 far more often than
    // that; in the session 0454 boot log it was the single most frequent message type,
    // outnumbering GPS MSM observations. That is pure uplink waste on a metered/tethered
    // link and pure noise in the RTCM logs. Pin both to the 1005 cadence.
    sendPqtm("PQTMCFGMSGRATE,W,RTCM3-1033,10");   // receiver & antenna descriptors
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    // GLONASS code-phase biases. Required whenever GLONASS observations (1084) are
    // published: GLONASS uses FDMA, so each satellite transmits on its own frequency and
    // every receiver design imposes a slightly different inter-frequency code bias. A
    // rover of a different make cannot resolve GLONASS ambiguities against this base
    // without knowing them, and silently degrades to using GLONASS for positioning only —
    // or drops it. A base that publishes 1084 and not 1230 is publishing observations its
    // consumers cannot fully use. Same 10 s cadence as the other slow-changing messages.
    sendPqtm("PQTMCFGMSGRATE,W,RTCM3-1230,10");   // GLONASS L1/L2 code-phase biases
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    // Ephemeris messages — explicit numbers, accepted syntax; let a rover fix faster.
    sendPqtm("PQTMCFGMSGRATE,W,RTCM3-1019,1");    // GPS ephemeris
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMCFGMSGRATE,W,RTCM3-1020,1");    // GLONASS ephemeris
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMCFGMSGRATE,W,RTCM3-1042,1");    // BeiDou ephemeris
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMCFGMSGRATE,W,RTCM3-1046,1");    // Galileo ephemeris
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    // ── Proprietary PQTM + NMEA telemetry message rates ──────────────────────
    // FIRMWARE-CONFIRMED: PQTM messages REQUIRE a trailing <MsgVer> in MSGRATE
    // (format: ...,W,<name>,<rate>,<msgver>). Omitting it returns ERROR,1 — that was
    // the bug that left survey-status/PVT/EPE silent (accuracy stuck at 0). NMEA
    // messages (GGA/GSV/RMC) take no MsgVer. Same set is re-asserted on hot-start.
    enableLg290pTelemetryMessages();
}

static void configureLg290pBaseOnce(bool forceReconfigure = false) {
    if (!LG290P_CONFIGURE_ON_BOOT) {
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "config disabled");
        return;
    }

    // ── ROVER-FIRST BOOT (PPP design) ────────────────────────────────────────
    // The base NEVER starts in base mode. The module's own flash usually holds a
    // persisted fixed-base config from the previous session, and booting into it is what
    // produced every "echoed old coordinate" failure in the field logs: the survey and
    // the move check were fed the stored position instead of live solutions. Boot
    // therefore goes straight to ROVER mode: PPP runs there, the move check gets genuine
    // independent fixes there, and base mode is entered exactly once — by the PPP lock
    // sequence after a survey completes, or by the confidence check adopting a confirmed
    // saved position (both of which call back in here with the flags set).
    //
    // This branch must stay FIRST: the hot-start probe below would otherwise ACCEPT a
    // persisted base config, which is precisely the state we are escaping.
    if (BASE_SURVEY_USE_PPP && !LG290P_USE_FIXED_BASE && !g_useNvsSavedPosition) {
        g_bootHotSkipped = false;
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "rover (PPP survey)");
        surveyInCommandMs      = 0;
        surveyInCompleteMs     = 0;
        surveyAwaitingFirstFix = true;    // clock arms on the first genuine rover fix
        {
            char sv[96];
            sendPqtm("PQTMCFGRCVRMODE,W,1");
            waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
            snprintf(sv, sizeof(sv), "PQTMCFGSVIN,W,1,%lu,%.2f,0,0,0",
                     (unsigned long)surveyInSec, (double)surveyAccLimit);
            sendPqtm(sv);                 // survey-in targets, so no fixed state persists
            waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
            sendPqtm("PQTMSAVEPAR");      // rover is now also the module's boot state
            waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
            sendPqtm("PQTMHOT");          // restart makes it take; hot start keeps ephemeris
            waitAndDrainGnss(LG290P_POST_RESET_WAIT_MS);
        }
        // VERIFY, DO NOT ASSUME. This used to stamp g_probedRcvrMode/g_probedSvinMode = 1
        // straight from intent. Those are the variables the venue check consults to decide
        // whether it must force the module out of fixed mode, so asserting success here
        // disabled that guard exactly when it was needed — see verifyEscapedFixedBase().
        if (!verifyEscapedFixedBase("boot rover-first")) {
            // PQTMHOT did not dislodge it. Escalate to a full system reset: PQTMSRR costs
            // the retained ephemeris (slower reacquisition) but is the stronger restart,
            // and a base that is still fixed here would otherwise validate its old
            // coordinate against itself. Correctness outranks time-to-first-fix.
            Serial.println("🔁 Rover escape did not take — escalating to PQTMSRR");
            char sv[96];
            sendPqtm("PQTMCFGRCVRMODE,W,1");
            waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
            snprintf(sv, sizeof(sv), "PQTMCFGSVIN,W,1,%lu,%.2f,0,0,0",
                     (unsigned long)surveyInSec, (double)surveyAccLimit);
            sendPqtm(sv);
            waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
            sendPqtm("PQTMSAVEPAR");
            waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
            sendPqtm("PQTMSRR");
            waitAndDrainGnss(LG290P_POST_RESET_WAIT_MS);
            if (!verifyEscapedFixedBase("boot rover-first retry")) {
                // Still fixed after both restarts. Leave the probed values reporting the
                // module's real state so checkPositionDisplacement()'s forced-rover branch
                // and the PPP echo detector both stay armed, and say so plainly: on this
                // hardware the retained config is battery-backed, so the operator-level
                // recovery is disconnecting the LG290P backup battery and all power.
                Serial.println("❌ Module REFUSED to leave fixed base mode after PQTMHOT and "
                               "PQTMSRR. Its retained config is battery-backed — clearing it "
                               "requires disconnecting the LG290P backup battery and all power. "
                               "Position confidence checks will treat this session as untrusted.");
            }
        }
        enableLg290pTelemetryMessages();  // GGA/EPE/GSV rates for the check + survey
        // The telemetry set above uses the BASE-mode PQTMEPE form (with MsgVer). In rover
        // mode the module wants the bare form (field-proven; documented in ppp_survey.cpp)
        // — send it too so the move check has live EPE before the survey re-asserts it.
        sendPqtm("PQTMCFGMSGRATE,W,PQTMEPE,1");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        // 1 Hz FOR EVERY ROVER PHASE, not just the survey. Rover mode's documented default is
        // 10 Hz; base mode is pinned at 1 Hz. Without this the device changes rate underneath
        // itself every time it crosses between modes, and the rover phases are the ones where
        // that matters: the PPP engine needs 1 Hz, and this base self-surveys as a rover
        // precisely because it is moved between venues and has to re-establish its own
        // position. Nothing here wants 10 Hz — the base publishes corrections at 1 Hz, the
        // venue check averages over tens of seconds, and a stationary receiver has nothing to
        // say ten times a second. Setting it once at boot also keeps every fix-count-based
        // window (the venue check's minimum, the survey's scatter window) meaning the same
        // number of SECONDS in every phase, which is how all of them were sized.
        sendPqtm("PQTMCFGFIXRATE,W,1000");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        sendPqtm("PQTMCFGFIXRATE,R");          // read back — the answer is worth having in the log
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        // The boot escape above IS a forced-rover restart, so the fixes arriving next are
        // the same post-reset fixes the venue check discards when it performs the escape
        // itself: the first ones out of a restarted receiver, still reacquiring, and on a
        // module that was fixed they are the stored coordinate echoed back. Only the check's
        // own escape used to set this, so on the normal boot path the settle was skipped and
        // the decision was taken on exactly the fixes it exists to reject.
        g_posCheckForcedRover = true;
        logEvent("ok", "Boot: rover mode, fix interval %d ms", g_probedFixRateMs);
        Serial.println("🛰️  Boot: module set to ROVER (PPP design — base mode only after lock/adopt)");
        // The PPP survey starts through servicePppSurveyStart() unless a saved position
        // exists, in which case the confidence check runs first on these rover fixes and
        // either adopts it (fixed base via the NVS branch) or falls through to a survey.
        if (!g_savedPositionValid) {
            g_pppEscapeDone        = true;                 // boot just performed the escape
            g_pppSurveyNotBeforeMs = 0;                    // settle already waited out above
            g_pppSurveyRequested   = true;                 // converted to pending below
        }
        // Hand the survey to loop() the same way the base path does.
        if (g_pppSurveyRequested) { g_pppSurveyRequested = false; g_pppSurveyPending = true; }
        return;
    }

    // HOT-START PRESERVATION (do not remove): re-sending the full config then
    // PQTMSAVEPAR + PQTMSRR forces a receiver reset that wipes retained time/
    // ephemeris/position (a cold start) and restarts survey-in from zero — adding
    // minutes of delay on every boot. The boot log proved the module is usually
    // already configured (RCVRMODE,R=2, SVIN,R matches), so probe first and skip all
    // of that unless the caller explicitly forces it (a user-requested param change).
    if (!forceReconfigure && lg290pBaseConfigMatches()) {
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "base retained (hot start)");
        g_bootHotSkipped = true;   // module left untouched; g_probedSvinMode is current
        // CRITICAL (do not remove): a warm boot skips the full reconfigure, but the
        // module's battery-backed config may not have our telemetry messages enabled at
        // the rates we need (survey status, PVT, EPE, GGA). Without them the boot
        // confidence check has no live fix to compare and would eventually time out to a
        // survey-in. Re-assert the message rates here; it's non-destructive (no reset, no
        // survey teardown, position untouched) so the hot-start benefit is preserved.
        enableLg290pTelemetryMessages();
        // The identical argument applies to the RTCM OUTPUT set, and skipping it is worse:
        // MSM4 is enabled by the PQTMCFGRTCM write inside enableLg290pBaseOutputs(), and
        // the 1005/1033 cadence by its PQTMCFGMSGRATE writes. A retained configuration
        // with MSM off brings the base up looking entirely healthy — fixed mode, survey
        // valid, RTCM frames flowing — while publishing nothing but descriptors. Session
        // 0029 spent 18.5 minutes in exactly that state: 2,254 copies each of 1005 and
        // 1033 at 500 ms, and zero observations of any constellation. These writes are
        // non-destructive in the same way the telemetry ones are (no SAVEPAR, no reset).
        enableLg290pBaseOutputs();
        // This path is taken because lg290pBaseConfigMatches() just READ the module and
        // found it already correctly configured as a base, so the confirmation is earned
        // here the same way it is at the end of a full reconfigure — by a read-back, not
        // by assumption. Without it a hot-started base would report itself as not yet in
        // base mode for the whole session.
        g_baseModeConfirmed = (g_probedRcvrMode == 2);
        return;   // leave survey timers untouched; autonomous PQTMSVINSTATUS drives them
    }
    g_bootHotSkipped = false;      // we are about to (re)write config + reset
    copyLimited(lg290pModeText, sizeof(lg290pModeText), "configuring base");

    // Survey clock starts later, gated on first valid fix (see armSurveyClockOnFix()).
    // Don't stamp it here — a long cold-start search would otherwise burn countdown
    // time while lat/lon are still 0,0. Fixed/NVS branches need no survey clock at all.
    surveyInCommandMs = 0;
    surveyInCompleteMs = 0;
    surveyAwaitingFirstFix = false;

    // 1. Send all Survey, RSID, and Message Rate Configurations FIRST
    if (LG290P_USE_FIXED_BASE) {
        char payload[160];
        snprintf(payload, sizeof(payload), "PQTMCFGSVIN,W,2,0,0,%.4f,%.4f,%.4f",
                 LG290P_FIXED_ECEF_X_M, LG290P_FIXED_ECEF_Y_M, LG290P_FIXED_ECEF_Z_M);
        sendPqtm(payload);
    } else if (g_useNvsSavedPosition && g_savedPositionValid) {
        // NVS saved position (manual entry, or confidence-confirmed reuse). Convert
        // saved lat/lon/alt to ECEF and configure as fixed base — skips survey-in.
        // FAILSAFE: the boot confidence check (PosCheckState) confirmed the antenna is
        // still on this spot (mean fix within the 2σ threshold) before setting this flag,
        // OR the operator asserted it via /api/setpos. If the saved position is wrong or
        // the antenna moved, the check resolves Moved/Timeout, clears the flag, and we
        // fall through to survey-in — so zeros/wrong-venue can never happen.
        const double latR = g_savedLat * DEG_TO_RAD;
        const double lonR = g_savedLon * DEG_TO_RAD;
        const double sinLat = sin(latR);
        const double a = 6378137.0, e2 = 6.6943799901e-3;
        const double n = a / sqrt(1.0 - e2 * sinLat * sinLat);
        const double sx = (n + g_savedAlt) * cos(latR) * cos(lonR);
        const double sy = (n + g_savedAlt) * cos(latR) * sin(lonR);
        const double sz = (n * (1.0 - e2) + g_savedAlt) * sinLat;
        char payload[160];
        snprintf(payload, sizeof(payload), "PQTMCFGSVIN,W,2,0,0,%.4f,%.4f,%.4f", sx, sy, sz);
        sendPqtm(payload);
        Serial.printf("📍 Using saved NVS position: %.8f,%.8f alt=%.2f (src=%s, acc=%.3fm)\n",
                      g_savedLat, g_savedLon, g_savedAlt, g_savedSource, (double)g_savedHAcc);
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "fixed (NVS)");
    } else {
        // TEST: enable Galileo E6 HAS PPP so the module's NATIVE survey-in averages a
        // PPP-corrected solution whenever corrections are available. HAS corrects
        // GPS+Galileo; if E6 is weak/absent the survey just proceeds on the ordinary
        // solution (graceful fallback — no separate rover-mode path needed). Fields:
        // mode=2 (E6 HAS), datum=1 (WGS84), timeout=120 s, hAcc=0.10 m, vAcc=0.15 m.
        // ⚠ $PQTMCFGPPP is the one command not in the public spec — on first boot confirm
        //   the module answers "$PQTMCFGPPP,OK" in the serial log. Delete these two lines
        //   (send + drain) to revert to plain survey-in.
        sendPqtm("PQTMCFGPPP,W,2,1,120,0.10,0.15");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);

        char payload[128];
        snprintf(payload, sizeof(payload), "PQTMCFGSVIN,W,1,%lu,%.2f,0,0,0",
                 static_cast<unsigned long>(surveyInSec), static_cast<double>(surveyAccLimit));
        sendPqtm(payload);
        surveyAwaitingFirstFix = true;   // hold the clock until first valid lat/lon
        // (Unreachable under BASE_SURVEY_USE_PPP: the rover-first branch above returns
        // before this point whenever a survey is needed. Kept for the internal-survey
        // design, and as the landing spot if the rover-first branch is ever disabled.)
        if (BASE_SURVEY_USE_PPP) {
            // PPP SURVEY IS THE DEFAULT PATH. The module only computes a position in
            // ROVER mode — in base mode it echoes the coordinate it was given — so a
            // PPP-quality survey can only be done by converging as a rover, averaging
            // under our own acceptance criteria, then handing the result back as a fixed
            // base. ppp_survey.cpp runs exactly that sequence and is started from loop()
            // once this configure finishes, so the rest of the base output configuration
            // below (RTCM message set and rates) is written and persisted first and
            // survives the temporary switch to rover mode.
            //
            // The PQTMCFGSVIN,W,1 above is still issued so the module holds a valid
            // survey configuration if the PPP attempt is abandoned; the PPP survey
            // supersedes it moments later.
            g_pppSurveyRequested = true;   // consumed at the END of this function
            copyLimited(lg290pModeText, sizeof(lg290pModeText), "PPP survey pending");
        } else {
            copyLimited(lg290pModeText, sizeof(lg290pModeText), "survey: awaiting fix (internal)");
        }
    }
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);

    {
        char payload[64];
        snprintf(payload, sizeof(payload), "PQTMCFGRSID,W,%u", LG290P_REFERENCE_STATION_ID);
        sendPqtm(payload);
    }
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);

    // 1. Set Receiver Mode to Base FIRST.
    // SPEC §2.3.23 NOTE: "The module must be Base station mode to execute this command"
    // (referring to PQTMSVINSTATUS). By extension, all RTCM output rate config and any
    // other base-station-specific PQTM messages must also be sent AFTER base mode is set.
    // Sending them before (as the old code did) returns ERROR,3 (unsupported command) for
    // PQTMSVINSTATUS and may silently misbehave for RTCM rates. Set base mode first,
    // THEN configure its outputs.
    sendPqtm("PQTMCFGRCVRMODE,W,2");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    // Read the mode straight back. The write above has no reply of its own, so without
    // this the probe state keeps whatever it last held — and on the saved-position path
    // that is the ROVER reading taken moments earlier by the boot move check's forced
    // escape. The reply is consumed by the PQTMCFGRCVRMODE,OK parser, which also stamps
    // g_probedRcvrModeMs, so a base entered this way stops looking like a rover to
    // everything downstream.
    sendPqtm("PQTMCFGRCVRMODE,R");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);

    enableLg290pBaseOutputs();
    // PPP (Galileo E6 HAS) IS available since LG290P firmware v2.01 and is now enabled at
    // the start of survey-in (see the survey-in branch above) so the surveyed coordinate
    // benefits from HAS corrections when E6 is being received. The PQTMSAVEPAR below
    // persists that setting, so it also stays active on later hot-start boots.
    // PQTMCFGPPP is absent from earlier LG290P firmware, so this write can be rejected.
    // Its reply is captured (see g_pppSupported) rather than assumed, and the dashboard
    // reports what the module actually said — a rejected PPP enable is otherwise
    // indistinguishable from a working one, since neither changes anything visible.

    // 3. Save ALL parameters to flash
    sendPqtm("PQTMSAVEPAR");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);

    // 4. Restart to apply base mode. PQTMSRR — a full system reset — NOT PQTMHOT.
    //
    // This used a hot start, on the reasoning that the spec asks for save-then-restart and
    // a hot start is a restart that preserves the retained almanac. That reasoning is
    // wrong on this module and the codebase already knew it in two other places:
    //
    //   ppp_survey.cpp, the base-mode entry at the end of a survey, says outright
    //   "PQTMSRR, not PQTMHOT ... field-proven ... a hot start was tried ... the mode
    //   change taking effect matters more than the reacquisition time."
    //
    //   the rover-first escape above tries PQTMHOT, VERIFIES, and escalates to PQTMSRR
    //   when the mode did not take — the same lesson, learned in the other direction.
    //
    // This function was the one base-mode transition that used a hot start AND never
    // checked the result, so when the mode silently failed to change there was nothing to
    // catch it: a confirmed saved position would set every downstream flag while the
    // receiver stayed a rover and published nothing. Forcing a survey instead reached base
    // mode through ppp_survey's PQTMSRR sequence, which is why that route worked and this
    // one did not.
    //
    // The cost is a cold reacquisition, paid once per reconfigure. That is the right trade:
    // a base that takes an extra minute to reacquire is inconvenient, a base that never
    // enters base mode is useless, and the verification below no longer has to rely on an
    // escalation path to reach the sequence that actually works.
    sendPqtm("PQTMSRR");
    waitAndDrainGnss(LG290P_POST_RESET_WAIT_MS);

    // Hand the PPP survey to loop() only now: the module has finished its reset and is
    // responsive, so ppp_survey_begin()'s rover-mode commands will actually be received.
    if (g_pppSurveyRequested) { g_pppSurveyRequested = false; g_pppSurveyPending = true; }

    // 5. VERIFY the mode change took, and escalate if it did not.
    //
    // This read-back already existed and its answer was thrown away — the function ended
    // by setting the mode text to "base fixed REQUESTED" and returning, with nothing
    // anywhere checking whether the request was honoured. That is the whole failure: a
    // confirmed saved position sets g_useNvsSavedPosition, queues this reconfigure, and
    // every consumer downstream then reports a base on a saved position while the module
    // is still sitting in rover mode, publishing nothing. No retry, no error, no trace.
    //
    // It is also the same shape as the escape-verify bug and the stale-probe bug before
    // it: the intent was recorded, the result never was. verifyEscapedFixedBase() exists
    // precisely because writing a mode is not the same as entering it, and the reverse
    // direction had no equivalent.
    //
    // The restart above is already the full reset, so this is not a cheaper-attempt
    // fallback — it is the check that the proven sequence actually worked.
    g_probedRcvrMode = -1;
    g_probedSvinMode = 0;
    sendPqtm("PQTMCFGRCVRMODE,R");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMCFGSVIN,R");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);

    if (g_probedRcvrMode != 2) {
        // The save-and-reset above is the field-proven sequence, so reaching here means
        // something beyond command ordering is wrong — most often the battery-backed
        // retained config, which has needed a physical power-down to clear before. One
        // more attempt costs seconds; giving up silently costs a session.
        logEvent("warn", "Base mode did not take after save + reset (module reports mode %d) - "
                         "retrying once", (int)g_probedRcvrMode);
        sendPqtm("PQTMSAVEPAR");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        sendPqtm("PQTMSRR");
        waitAndDrainGnss(LG290P_POST_RESET_WAIT_MS);
        g_probedRcvrMode = -1;
        g_probedSvinMode = 0;
        sendPqtm("PQTMCFGRCVRMODE,R");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        sendPqtm("PQTMCFGSVIN,R");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        // A full reset discards the retained state the outputs were written against, so
        // re-assert them rather than assuming they survived.
        if (g_probedRcvrMode == 2) enableLg290pBaseOutputs();
    }

    if (g_probedRcvrMode == 2) {
        g_baseModeConfirmed = true;
        logEvent("ok", "Base mode confirmed by read-back (svinMode=%d)", (int)g_probedSvinMode);
        copyLimited(lg290pModeText, sizeof(lg290pModeText),
                    LG290P_USE_FIXED_BASE ? "fixed base" : "base (survey)");
    } else {
        // Left false so baseUsingSavedPosition() keeps reporting "not yet in base mode"
        // instead of a saved position the module is not actually serving, and so the
        // retry below re-arms.
        g_baseModeConfirmed = false;
        logEvent("fail", "Base mode NOT entered - module still reports mode %d after save "
                         "and full reset; the base is a rover and is publishing nothing",
                 (int)g_probedRcvrMode);
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "base mode FAILED");
    }
}

// ── Is the receiver actually tracking satellites? ────────────────────────────
// A module hot-started as a FIXED base reports PQTMSVINSTATUS <Valid>=2 continuously
// from its retained configuration, at the full 1 Hz, with no antenna connected. It is
// not lying: the survey it is describing did converge, once, at some other time and
// place. It simply has no bearing on whether this device is presently able to produce
// corrections. Field-observed on the bench with the antenna off: the dashboard read
// "Survey (converged)", valid 2, 1.446 m accuracy, while the position was 0,0 and not
// one satellite was in view.
//
// This cannot be fixed by clearing the cached value, which is what resetSurveyBookkeeping()
// does for the rover-mode case — the module re-asserts valid=2 within a second, because
// it is a live base-mode output rather than a stale one. The claim has to be
// CORROBORATED instead: a converged survey that no satellite signal supports is not
// reported as converged.
//
// Satellite COUNT rather than fix quality is the primary test, because it distinguishes
// "no antenna" from "antenna connected, still acquiring" — a receiver with sky view
// reports satellites well before it reports a fix. Fix quality is accepted as an
// alternative so a module that reports a position without a satellite count in GGA
// (base mode varies by firmware) is not held down by a field it never populates.
static constexpr uint32_t SIGNAL_EVIDENCE_FRESH_MS = 15000;

static bool receiverHasSignal() {
    if (ggaLastSeenMs == 0) return false;                              // no GGA at all
    if ((millis() - ggaLastSeenMs) >= SIGNAL_EVIDENCE_FRESH_MS) return false;
    return (ggaSatellites > 0) || (ggaFixQuality > 0);
}

// Age of the last PQTMSVINSTATUS. See SVIN_STATUS_STALE_MS for why a cached value must
// never be read as a current one.
static bool svinStatusFresh() {
    return surveyStatus.lastSeenMs != 0 &&
           (millis() - surveyStatus.lastSeenMs) < SVIN_STATUS_STALE_MS;
}

// The receiver's survey state as it stands RIGHT NOW: 0 = invalid/off or unknown,
// 1 = in progress, 2 = valid/converged. Every gate, display and log field that asks
// "has the survey finished" uses this rather than surveyStatus.valid directly, so a
// receiver that has gone quiet reports "not finished" instead of repeating its last
// answer forever. The two call sites inside parseSurveyStatus() are exempt: they run
// on the message that was just parsed, which is fresh by construction.
static uint8_t svinValidNow() {
    if (!svinStatusFresh()) return 0;
    // A convergence claim the receiver cannot presently support is not reported as one.
    // See receiverHasSignal(). Demoted to 0 rather than 1 because "in progress" would be
    // its own false statement — nothing is progressing without an antenna.
    if (surveyStatus.valid == 2 && !receiverHasSignal()) return 0;
    return surveyStatus.valid;
}

static uint32_t surveyTargetSeconds() {
    if (LG290P_USE_FIXED_BASE) return 0;
    // Under the PPP survey the window is OURS: ppp_survey.cpp times it, and the operator
    // can retarget it live without the receiver being told. The module's cfgDur belongs
    // to its own internal survey and would be stale here, which previously made a
    // "keep surveying" change appear to have no effect.
    if (BASE_SURVEY_USE_PPP) return surveyInSec;
    // Internal survey: trust the receiver's cfgDur only if plausible; the SVINSTATUS
    // field layout varies by firmware and the parser guesses by field count, so a
    // mis-indexed value can be garbage. Fall back to the configured duration.
    if (surveyStatus.cfgDur > 0 && surveyStatus.cfgDur <= 86400UL) return surveyStatus.cfgDur;
    return surveyInSec;
}

// Start the survey clock the moment we first have a valid GPS lat/lon (not 0,0),
// per the requirement that the countdown must not advance while still searching for
// first fix. Only arms when configureLg290pBaseOnce() set survey-in mode AND the
// clock hasn't started. Called every loop pass; cheap and idempotent.
static void armSurveyClockOnFix() {
    if (!surveyAwaitingFirstFix) return;
    const bool haveValidFix = (ggaFixQuality > 0) && (ggaLat != 0.0 || ggaLon != 0.0);
    if (!haveValidFix) return;
    surveyInCommandMs = millis();          // NOW the survey clock starts
    surveyAwaitingFirstFix = false;
    surveyElapsedHighWaterS = 0;           // fresh survey → restart the monotonic latch
    resetSurveyScatter();                  // fresh survey → restart realistic-accuracy tracking
    copyLimited(lg290pModeText, sizeof(lg290pModeText), "surveying");
    logEvent("ok", "First fix acquired (%.7f;%.7f) - survey clock started", ggaLat, ggaLon);
}

static uint32_t surveyElapsedSeconds() {
    if (LG290P_USE_FIXED_BASE) return 0;
    // A PPP SURVEY OWNS ITS OWN CLOCK — do not fall through to surveyStatus.obs below.
    // That counter is the MODULE's internal survey-in progress, a base-mode-only output
    // with nothing to do with the rover-mode survey running here, and it lies in two ways.
    // If the module has not actually left base mode — which the escape cannot always
    // achieve, its retained config being battery-backed — it keeps publishing
    // PQTMSVINSTATUS carrying the PREVIOUS survey's obs count, and republishes it within a
    // second of resetSurveyBookkeeping() clearing it. The high-water latch below then pins
    // the clock at that stale value, so "Apply & restart survey" and "Start new survey"
    // look like they did nothing while a new survey is in fact running underneath. Even
    // with a clean escape the counter is the wrong quantity. Reading elapsed from the
    // survey we actually started takes the receiver out of the loop, and it restarts when
    // ppp_survey_begin() does, which is what those buttons promise.
    if (BASE_SURVEY_USE_PPP) {
        if (ppp_survey_active()) return ppp_survey_status().elapsedS;
        return (surveyInCommandMs == 0) ? 0 : (millis() - surveyInCommandMs) / 1000UL;
    }
    // Prefer receiver obs ONLY when it's sane (see surveyTargetSeconds note);
    // otherwise use the monotonic ESP32 timer so the countdown stays meaningful
    // even if the SVINSTATUS parse is off. This keeps the displayed timer honest
    // and decoupled from the readiness gate (which is RTCM-driven, not parse-driven).
    const uint32_t target = surveyTargetSeconds();
    uint32_t elapsed;
    if (surveyStatus.obs > 0 && surveyStatus.obs <= target + 600UL) elapsed = surveyStatus.obs;
    else if (surveyInCommandMs == 0) elapsed = 0;
    else elapsed = (millis() - surveyInCommandMs) / 1000UL;
    // Latch monotonic: never let elapsed decrease within one survey. If the module's
    // obs counter resets (re-survey because the accuracy limit wasn't met by the target
    // time), the countdown holds at 0 and waits for convergence instead of jumping back
    // to full. Reset happens only in armSurveyClockOnFix() when a new survey starts.
    if (elapsed > surveyElapsedHighWaterS) surveyElapsedHighWaterS = elapsed;
    return surveyElapsedHighWaterS;
}

// True when this base is running on a saved coordinate the boot check CONFIRMED, rather
// than on a survey. No survey is running, none is queued, and none ever will be for this
// coordinate — the whole point of confirming a saved position is to skip one.
//
// The survey helpers below had no way to express that. They ask svinValidNow(), which the
// module answers 0 for a manually-set fixed position (PQTMCFGSVIN,W,2 runs no survey, so
// there is no survey status to report valid), and surveyElapsedSeconds(), which reads 0
// because the survey clock was never armed. Target minus zero is the full window, so the
// dashboard sat at a frozen 45:00 countdown while the base beside it was fixed, ready and
// publishing at 6 fps. Same condition updateBaseReadiness() uses to latch on this path.
static bool baseUsingSavedPosition() {
    // g_useNvsSavedPosition is the DECISION to use the saved coordinate, set the instant
    // the boot check confirms. It says nothing about whether the module has been switched
    // to base mode yet, and that switch involves a write, a save and a restart that can
    // fail. Keying the display on the decision alone is what let the survey timer report
    // a saved position while the receiver was still a rover — the countdown stopped, the
    // mode never changed, and nothing on the dashboard disagreed.
    return g_useNvsSavedPosition && g_savedPositionValid && g_baseModeConfirmed;
}

static int32_t surveyRemainingSeconds() {
    if (LG290P_USE_FIXED_BASE) return 0;
    // A RUNNING PPP SURVEY OUTRANKS EVERY RECEIVER-SIDE "already done" signal. Each of the
    // conditions below reports on the MODULE's survey state, and during a rover-mode survey
    // that state is at best irrelevant and at worst a stale claim from the survey this one
    // replaced: a module that did not leave base mode keeps asserting svin valid == 2, which
    // zeroes the countdown for a survey that has only just started. If ppp_survey is running,
    // there is a survey in progress by definition and the countdown is meaningful.
    const bool pppRunning = BASE_SURVEY_USE_PPP && ppp_survey_active();
    if (!pppRunning && (surveyInSkipped || baseUsingSavedPosition() ||
                        svinValidNow() == 2)) return 0;
    const uint32_t elapsed = surveyElapsedSeconds();
    const uint32_t target = surveyTargetSeconds();
    if (elapsed >= target) return 0;
    return static_cast<int32_t>(target - elapsed);
}

// Durations are always rendered hh:mm:ss. A fixed-width form keeps the LCD countdown
// from changing length as it crosses an hour or a minute boundary (which would leave
// stale glyphs behind in the padded, opaque-background fields the display draws into),
// and makes survey targets directly comparable at a glance.
static String formatDuration(uint32_t seconds) {
    char buffer[24];
    const uint32_t h = seconds / 3600UL;
    const uint32_t m = (seconds % 3600UL) / 60UL;
    const uint32_t s = seconds % 60UL;
    snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu",
             static_cast<unsigned long>(h), static_cast<unsigned long>(m),
             static_cast<unsigned long>(s));
    return String(buffer);
}

static bool isRtcmBaseActive() {
    for (const auto& stat : rtcmStats.typeStats) {
        // CORRECTNESS FIX (field-confirmed): readiness must key on MSM OBSERVATIONS
        // (1071-1137), NOT on 1005/1006/1033. The LG290P emits a preliminary 1005
        // and 1033 *during* survey-in with an un-converged position, so gating on
        // 1005 made the casters go "ready" and stream useless descriptor-only data
        // before survey completed. MSM observations only appear once the base has a
        // valid fixed reference position, so they are the true "real corrections are
        // flowing" signal. Do NOT revert this to 1005/1006.
        if (isMsmObservation(stat.type) && stat.lastSeenMs > 0 &&
            (millis() - stat.lastSeenMs < RTCM_BASE_FRESH_MS)) return true;
    }
    return false;
}

// True if the base is currently emitting real per-satellite observations (used for
// the dashboard "corrections flowing" indicator). Same basis as readiness above.
static bool observationsFlowing() { return isRtcmBaseActive(); }

// ── Base readiness: the automatic start gate ─────────────────────────────────
// AUTO-START: the casters stream ONLY while baseReady is true, so this must flip
// true on its own once the receiver's survey-in has genuinely CONVERGED — without
// the operator pressing Force, and WITHOUT casting during an unfinished survey.
//
// AUTHORITATIVE GATE = PQTMSVINSTATUS <Valid>==2 (spec §2.3.23: 0=invalid/off,
// 1=in-progress, 2=valid/converged). This is the module's OWN determination that
// the weighted-mean survey dropped below the configured 3D accuracy limit.
//
// Why NOT MSM observations: the LG290P emits MSM4 (and 1005) continuously in base
// mode using its RUNNING survey estimate — they flow all through survey-in. Gating
// on them streamed corrections referenced to an unconverged position the moment a
// fix appeared (the field-observed "casting before survey-in finished" bug). MSM
// presence is therefore used ONLY for the dashboard "corrections flowing" light
// (observationsFlowing()), never for readiness.
//
// Corroborating accuracy check: require meanAcc to be reported AND within the limit
// so a spurious valid==2 with poor accuracy can't open the gate. Bypass paths
// (Force, fixed base, hot-start-confirmed NVS position) skip survey entirely.
static void updateBaseReadiness() {
    const uint32_t now = millis();
    if (surveyInSkipped || LG290P_USE_FIXED_BASE ||
        (g_useNvsSavedPosition && g_savedPositionValid)) {   // operator/fixed/hot-confirmed override
        baseReady = true;
        baseEvidenceMs = now;
        return;
    }
    // Genuine survey completion: module says valid==2 AND the mean accuracy it
    // reported is at/below our configured limit (meanAcc>0 means it actually has one).
    // A completed survey reports valid==2. The module ZEROES meanAcc the instant it
    // switches to fixed mode at completion, so requiring meanAcc>0 here made a normal
    // completion never latch (base_ready stuck 0, nothing ever streamed — see field log
    // 0075). Accept valid==2 when meanAcc is either unreported (<=0 → fixed/complete) or
    // genuinely within the limit; still reject a valid==2 whose reported accuracy is
    // ABOVE the limit (a poor survey).
    // svinValidNow() already refuses to return 2 without satellite signal behind it, so
    // the readiness latch inherits that corroboration and cannot arm on a retained
    // fixed-base configuration alone.
    const bool surveyConverged = (svinValidNow() == 2) &&
                                 (surveyStatus.meanAcc <= 0.0 ||
                                  surveyStatus.meanAcc <= static_cast<double>(surveyAccLimit));
    if (surveyConverged) {
        baseEvidenceMs = now;
        baseReady = true;
        return;
    }
    // No completion this pass: hold ready through short gaps (e.g. a missed
    // PQTMSVINSTATUS line) once we've LATCHED; drop only after a sustained absence.
    if (baseReady && baseEvidenceMs != 0 && now - baseEvidenceMs < BASE_READY_GRACE_MS) return;
    baseReady = false;
}

static bool surveyIsReady() {
    return baseReady;   // latched in updateBaseReadiness(); must run earlier in loop()
}

// ── Correction quality: is what we are about to publish fit to publish? ──────
// baseReady answers "did the survey converge". This answers "is the stream leaving this
// device right now something a rover should steer by", which is a different question and
// has failed independently in the field. Session 0022 is the case that motivated it: the
// receiver was in rover mode running a replacement survey, producing no RTCM at all,
// while the casters sat authenticated on crtk.net advertising the mount. A rover that
// connected would have found a live mount publishing nothing.
//
// Conditions, in the order updateCastQuality() evaluates them — most specific first, so
// the reason reported to the operator is the most informative one available:
//   1. The receiver must not be RECENTLY READ as being in rover mode. A rover has no
//      reference coordinate to reference corrections to, so this is a real fault — but
//      only while the reading is current, because the probe value is a cache of a mode
//      that changes without announcing itself. An unknown result (-1) or a stale one is
//      not a fault; both fall through to the observation-based conditions, which need no
//      probe. This condition therefore only ever improves the REASON reported, never the
//      verdict: condition 3 catches an actual rover on its own, since a rover emits no
//      MSM. checkPppSurveyCompletion() stamps g_probedRcvrMode = 2 from its lock sequence
//      rather than from a read-back, and that assumption is covered the same way.
//   2. The receiver must be tracking satellites (receiverHasSignal()). Listed separately
//      from the MSM test below, which would also fail, because a disconnected antenna is
//      a different thing for an operator to be told than an absent message type.
//   3. MSM observations must actually be arriving. isRtcmBaseActive() is deliberately
//      keyed on MSM and not on 1005/1033, which the module emits during survey-in from
//      an unconverged position — see its own comment. This is what catches "advertising
//      a mount that produces nothing".
//   4. Frames must be intact. A sustained CRC/framing failure rate means the UART is
//      losing bytes, and a frame that survived is no more trustworthy than the ones
//      that did not; sessions 0455 vs 0457 showed this run at 219/hr and then at zero.
//   5. The reference coordinate's own accuracy must be inside CAST_QUALITY_MAX_POS_ACC_M.
//      A survey that "converged" at 12 m is a base that will pull rovers 12 m off.
//
// The result is hysteretic (see CAST_QUALITY_HOLD_MS): a fault must persist before the
// stream is cut, and clear before it resumes, so no single late frame costs a caster
// session or spends an rtk2go reconnect.
static char     g_castQualityReason[72] = "";
static uint32_t g_castFaultSinceMs = 0;      // 0 = no fault currently observed
static uint32_t g_castCleanSinceMs = 0;      // 0 = not currently clean
static bool     g_castQualityOk    = true;   // hysteretic verdict; the one loop() reads

// Rolling CRC/framing failure rate. Counters are cumulative, so the rate is taken from
// deltas over a window rather than from the lifetime totals, which would keep a base
// blocked forever over a fault that has since been fixed.
static uint32_t g_crcWindowStartMs  = 0;
static uint32_t g_crcWindowCand     = 0;
static uint32_t g_crcWindowFail     = 0;
static float    g_crcFailRate       = 0.0f;

static void updateCrcFailRate() {
    const uint32_t now  = millis();
    const uint32_t cand = rtcmStats.rtcmCandidateFrames;
    const uint32_t fail = rtcmStats.rtcmCrcFailures + rtcmStats.rtcmFramingFailures;
    if (g_crcWindowStartMs == 0) {
        g_crcWindowStartMs = now; g_crcWindowCand = cand; g_crcWindowFail = fail;
        return;
    }
    if (now - g_crcWindowStartMs < CAST_QUALITY_CRC_WINDOW_MS) return;
    const uint32_t dCand = cand - g_crcWindowCand;
    const uint32_t dFail = fail - g_crcWindowFail;
    // Below the minimum sample count the window says nothing, so the previous rate is
    // held rather than replaced with a figure computed from a handful of frames.
    if (dCand >= CAST_QUALITY_CRC_MIN_SAMPLES)
        g_crcFailRate = (float)dFail / (float)dCand;
    g_crcWindowStartMs = now; g_crcWindowCand = cand; g_crcWindowFail = fail;
}

// Evaluates the five conditions and applies the hysteresis. Must run after
// updateBaseReadiness() and before the streaming gate.
static void updateCastQuality() {
    updateCrcFailRate();
    const uint32_t now = millis();
    char reason[72];
    reason[0] = '\0';

    // A rover reading only counts while it is still current. See g_probedRcvrModeMs: the
    // module's mode changes without announcing itself, so an old reading is a memory of
    // where the receiver used to be, not a statement about where it is. A stale one falls
    // through to the observation-based conditions below, which need no probe at all — a
    // receiver actually in rover mode produces no MSM and is caught there regardless.
    const bool rcvrModeFresh = g_probedRcvrModeMs != 0 &&
                               (now - g_probedRcvrModeMs) < PROBE_MODE_TRUST_MS;
    if (ppp_survey_active()) {
        // Checked before anything probe-derived. A running survey is the authoritative
        // reason the base is not publishable, it needs no read-back, and it cannot go
        // stale. Without it the reason decayed from "receiver is in rover mode" to
        // "no MSM observations from the receiver" the moment the mode reading aged out —
        // technically true, since a rover emits no MSM, but it describes a symptom of the
        // survey rather than the survey, and it made a normal 45-minute survey look like
        // a fault for 43 of those minutes.
        const PppSurveyStatus sv = ppp_survey_status();
        const uint32_t left = (sv.durationS > sv.elapsedS) ? (sv.durationS - sv.elapsedS) : 0;
        snprintf(reason, sizeof(reason), "survey in progress - %s remaining",
                 formatDuration(left).c_str());
    } else if (g_probedRcvrMode == 1 && rcvrModeFresh) {
        copyLimited(reason, sizeof(reason), "receiver is in rover mode - no reference position");
    } else if (!receiverHasSignal()) {
        // Ahead of the MSM test, which would also catch this: a module with no antenna
        // emits 1005 from its configured coordinate but no observations, so the MSM test
        // reports "no observations" when "no satellites" is the actionable fact.
        copyLimited(reason, sizeof(reason), "no satellites tracked - check the antenna");
    } else if (!isRtcmBaseActive()) {
        copyLimited(reason, sizeof(reason), "no MSM observations from the receiver");
    } else if (g_crcFailRate > CAST_QUALITY_MAX_CRC_FAIL_RATE) {
        snprintf(reason, sizeof(reason), "%.1f%% of RTCM frames failing CRC/framing",
                 (double)(g_crcFailRate * 100.0f));
    } else if (!surveyInSkipped && !LG290P_USE_FIXED_BASE) {
        // The accuracy condition is the only one of the five an operator can knowingly
        // override, so it is the only one Force and a compile-time fixed ECEF are exempt
        // from. Both are explicit assertions that this coordinate is the one to publish;
        // silently refusing to publish it would make "Force base now" a button that does
        // nothing. The other three conditions stay live even under Force — a rover with no
        // reference position, a stream with no observations in it, and frames failing CRC
        // are not states anyone would choose.
        //
        // realisticAccuracyM() returns -1.0 when there is no honest figure at all, which
        // compares below the limit and therefore does not block. That is deliberate: an
        // unknown accuracy is caught by the MSM condition instead, since a base with no
        // reference position produces no MSM observations to publish.
        const double acc = realisticAccuracyM();
        if (acc > (double)CAST_QUALITY_MAX_POS_ACC_M)
            snprintf(reason, sizeof(reason), "reference accuracy %.2fm exceeds %.2fm limit",
                     acc, (double)CAST_QUALITY_MAX_POS_ACC_M);
    }

    const bool faultNow = (reason[0] != '\0');
    if (faultNow) {
        g_castCleanSinceMs = 0;
        if (g_castFaultSinceMs == 0) g_castFaultSinceMs = now;
        copyLimited(g_castQualityReason, sizeof(g_castQualityReason), reason);
        if (g_castQualityOk && (now - g_castFaultSinceMs) >= CAST_QUALITY_HOLD_MS) {
            g_castQualityOk = false;
            logEvent("fail", "Corrections withheld: %s", g_castQualityReason);
        }
    } else {
        g_castFaultSinceMs = 0;
        if (g_castCleanSinceMs == 0) g_castCleanSinceMs = now;
        if (!g_castQualityOk && (now - g_castCleanSinceMs) >= CAST_QUALITY_RECOVER_MS) {
            g_castQualityOk = true;
            g_castQualityReason[0] = '\0';
            logEvent("ok", "Correction quality restored - publishing resumed");
        }
    }
}

// ── Base output watchdog: does the stream contain what a base is for? ───────
// Every PQTMCFGRTCM / PQTMCFGMSGRATE write in this firmware is issued without checking
// its acknowledgement, so a configuration that silently fails to apply looks exactly like
// one that worked. The distinguishing evidence is in the output itself: a base publishing
// 1005 and 1033 but no MSM observations of any constellation is configured as a base and
// producing nothing a rover can use.
//
// This watches for that state and repairs it by re-asserting the output configuration
// once, rather than by assuming the original write took. enableLg290pBaseOutputs() is
// non-destructive — no PQTMSAVEPAR, no reset, no survey teardown — so the repair costs
// a few paced commands and cannot restart an established base.
//
// Deliberately keyed on descriptors present AND observations absent: "nothing at all" is
// a dead receiver, which is a different fault with its own handling, and re-writing the
// RTCM set would not help it.
static uint32_t g_baseOutputRepairMs = 0;

// Warn once per threshold crossing when DMA-capable internal heap runs low. Below roughly
// 40 kB the SD driver cannot allocate a DMA buffer and lwIP cannot open a socket, so the
// visible failures are an SD write error, a caster that will not reconnect and a rover
// that cannot be accepted — three unrelated-looking symptoms of one cause. Naming it here
// costs one line and saves reconstructing it from a driver's own error code.
static uint32_t g_lastHeapWarnMs = 0;

static void updateHeapWatchdog() {
    // SAMPLED, NOT POLLED (keep the rate limit). heap_caps_get_free_size() walks the
    // allocator's region list under the heap lock — the same lock every malloc on both
    // cores needs, including the SD driver's DMA buffers and lwIP's pbufs. Running it on
    // every loop pass put contention on the allocator purely to read a number that only
    // matters once per warning interval. The status log samples the heap on its own 5 s
    // cadence and is unaffected by this.
    const uint32_t now = millis();
    static uint32_t heapPollMs = 0;
    if (heapPollMs != 0 && (now - heapPollMs) < 1000) return;
    heapPollMs = now;
    const uint32_t freeInt =
        (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (freeInt >= HEAP_INTERNAL_WARN_BYTES) return;
    if (g_lastHeapWarnMs != 0 && (now - g_lastHeapWarnMs) < HEAP_WARN_INTERVAL_MS) return;
    g_lastHeapWarnMs = now;
    logEvent("warn", "Internal DMA heap low: %lu B free; largest %lu B; min-since-boot %lu B",
                  (unsigned long)freeInt,
                  (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                  (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

static void updateBaseOutputWatchdog() {
    // Only meaningful once the module is actually a fixed base and has had time to settle.
    if (g_probedSvinMode != 2 || !surveyIsReady()) { g_baseOutputRepairMs = 0; return; }
    const uint32_t now = millis();

    const bool descriptorsFlowing =
        rtcmTypeEverSeen(1005) &&
        rtcmStats.lastValidMs != 0 && (now - rtcmStats.lastValidMs) < RTCM_BASE_FRESH_MS;
    if (!descriptorsFlowing) { g_baseOutputRepairMs = 0; return; }

    if (isRtcmBaseActive()) { g_baseOutputRepairMs = 0; return; }   // observations present

    // Descriptors without observations. Repair at most once per interval so a receiver
    // that genuinely cannot produce MSM is not hammered with configuration writes.
    if (g_baseOutputRepairMs != 0 && (now - g_baseOutputRepairMs) < BASE_OUTPUT_REPAIR_INTERVAL_MS)
        return;
    g_baseOutputRepairMs = now;
    logEvent("warn", "Base publishing descriptors with no MSM observations - "
                     "re-asserting RTCM output configuration");
    enableLg290pBaseOutputs();
}

// ── Clearing a survey's state: ONE place, used by every path that starts one ──
// baseReady is a LATCH (see updateBaseReadiness) and surveyStatus is a CACHE of the
// receiver's last PQTMSVINSTATUS. Neither clears itself. Every path that abandons a
// coordinate and starts a fresh survey must clear both, or the base carries the old
// survey's verdict into the new one: field session 0022 reported base_ready=1 and
// svin_valid=2 in every row of an 8.5-minute log while a PPP survey it had just
// started was 17% through its window, and the casters sat in
// "Authenticated - waiting RTCM" advertising a mount that was producing nothing. The
// move check cleared the saved position correctly; it was these two that were left
// holding the hot start's answer. Anything that queues a survey calls this.
// A base that holds a good coordinate and never entered base mode is a dead end. Every
// path that adopts a position — a PPP lock, a confirmed venue check, an operator's manual
// entry — writes base mode into the receiver and then stops. If that write did not take,
// the state is stable and self-consistent: the coordinate is saved and correct, the check
// is done, no survey is queued, and nothing left in the system ever asks again. The base
// sits for the whole session as a rover, publishing nothing, while the dashboard reports a
// saved position. Session 0029 was the same shape one level down (a base publishing
// descriptors and no observations) and was fixed by watching the OUTPUT rather than
// trusting the write; this closes the case where the mode itself never took.
//
// IT ASKS THE RECEIVER RATHER THAN INFERRING FROM THE STREAM. The obvious test — are MSM
// observations flowing? — is not sound here. MSM carry the receiver's OWN raw observations,
// which a rover has just as much as a base, and the RTCM output configuration lives in the
// module's retained NVM, so a receiver that leaves base mode can carry on emitting the same
// message set it was last told to. Streaming RTCM is therefore not proof of base mode, and
// keying on it could suppress this repair forever in exactly the state that needs it. A
// read-back is two commands and a few hundred milliseconds, it is the project's standing
// rule for every other mode question, and it is self-healing: if the mode did take and only
// the confirmation was missed — a slow reply after the lock, say — this notices and records
// it instead of resetting a working base.
static uint32_t g_baseModeRepairMs = 0;

static void updateBaseModeWatchdog() {
    // Only meaningful once a coordinate has been adopted and the boot check has resolved.
    if (!g_useNvsSavedPosition || !g_displacementCheckDone) { g_baseModeRepairMs = 0; return; }
    // Anything already driving the receiver owns it; do not queue a second reconfigure.
    if (ppp_survey_active() || g_pppSurveyPending || g_reconfigPending ||
        g_forceBasePending  || g_surveyRestartPending) { g_baseModeRepairMs = 0; return; }
    if (g_baseModeConfirmed) { g_baseModeRepairMs = 0; return; }

    const uint32_t now = millis();
    // Spaced like the output watchdog, and for the same reason: a receiver that genuinely
    // cannot enter base mode must not be flooded with configuration writes, but a base
    // must not spend a session in a state it can recover from on its own.
    if (g_baseModeRepairMs != 0 && (now - g_baseModeRepairMs) < BASE_OUTPUT_REPAIR_INTERVAL_MS)
        return;
    g_baseModeRepairMs = now;

    g_probedRcvrMode = -1;
    sendPqtm("PQTMCFGRCVRMODE,R");
    const uint32_t deadline = millis() + LG290P_ESCAPE_VERIFY_MS;
    while ((int32_t)(millis() - deadline) < 0) {
        processGnssSerial();
        if (g_probedRcvrMode != -1) break;
        delay(5);
    }
    if (g_probedRcvrMode == 2) {
        g_baseModeConfirmed = true;      // it did take; only the confirmation was missed
        logEvent("ok", "Base mode confirmed by watchdog read-back");
        return;
    }
    if (g_probedRcvrMode < 0) {
        logEvent("warn", "Base-mode check: no reply from receiver in %lums - retrying later",
                 (unsigned long)LG290P_ESCAPE_VERIFY_MS);
        return;                          // no reply is not evidence of rover; do not reset on it
    }
    logEvent("warn", "Saved position adopted but receiver reports mode %d - "
                     "re-entering base mode", (int)g_probedRcvrMode);
    g_reconfigPending = true;
}

// Drop the LIVE copy of the saved position, deliberately WITHOUT touching NVS.
//
// A survey exists to produce a new coordinate, which means the old one stops describing
// this base the moment the survey starts. But the runtime mirror of it kept being consulted
// anyway: realisticAccuracyM() falls back to g_savedHAcc, baseUsingSavedPosition() reads
// g_savedPositionValid, and the dashboard, LCD and status log all report pos_src and
// pos_acc from these. So a restarted survey went on quoting the accuracy of the coordinate
// it was in the middle of replacing — and when the old coordinate was the WRONG one, which
// is the usual reason for restarting, the displayed error was measured from the wrong
// point and looked plausible.
//
// NVS IS LEFT ALONE ON PURPOSE. Clearing it would be the tidier-looking change and it is
// the wrong one: if this survey then fails, is aborted, or the unit is power-cycled
// mid-window, the old coordinate is the only thing standing between the operator and
// re-entering a surveyed position by hand. A stale coordinate in flash is recoverable; a
// deleted one is not. The survey overwrites it on completion, so nothing accumulates.
// clearSavedPosition() remains the separate, deliberate act for a coordinate that is known
// to be wrong — such as a confirmed venue change.
static void forgetSavedPositionInMemory() {
    if (!g_savedPositionValid && !g_useNvsSavedPosition) return;   // nothing live to drop
    g_savedPositionValid  = false;
    g_useNvsSavedPosition = false;
    g_baseModeConfirmed   = false;   // a survey returns the module to rover; it is no longer a base
    g_savedLat = g_savedLon = g_savedAlt = 0.0;
    g_savedHAcc      = 0.0f;         // 0 reads as "unknown" to realisticAccuracyM(), not as 0.000 m
    g_savedSurveySec = 0;
    g_savedSource[0] = '\0';
    g_savedNote[0]   = '\0';
    Serial.println("ℹ️  Live saved position released for this survey (NVS copy retained)");
}

static void resetSurveyBookkeeping() {
    baseReady               = false;   // release the latch — this survey has not converged
    baseEvidenceMs          = 0;
    surveyInSkipped         = false;
    surveyInCompleteMs      = 0;
    surveyStatus.valid      = 0;
    surveyStatus.obs        = 0;
    surveyStatus.meanAcc    = 0.0;
    surveyStatus.lastSeenMs = 0;       // no cached receiver verdict carries into the new survey
    surveyBestMeanAcc       = 0.0;
    surveyElapsedHighWaterS = 0;
    surveyInCommandMs       = 0;       // clock re-arms on the next fix, not at boot
    surveyAwaitingFirstFix  = true;    // armSurveyClockOnFix() restarts it
    resetSurveyScatter();              // realistic-accuracy tracking restarts with the survey
    // The saved position is bookkeeping from the PREVIOUS survey exactly as the latch and
    // the cached receiver verdict are, and it is released here for the same reason: every
    // path that starts a survey calls this one function, so none of them can forget.
    forgetSavedPositionInMemory();
}

static String surveyTimerText() {
    if (LG290P_USE_FIXED_BASE) return String("Fixed Base");
    if (surveyInSkipped) return String("Instant Base");
    if (baseUsingSavedPosition()) return String("Saved Position");
    if (!receiverHasSignal()) return String("No signal");
    // Same precedence as surveyRemainingSeconds(): once a PPP survey is running, the
    // module's own survey verdict describes a survey that is over, and reporting
    // "Complete" over a countdown that has just been restarted is the specific way this
    // read wrong on the panel.
    if (BASE_SURVEY_USE_PPP && ppp_survey_active())
        return formatDuration(static_cast<uint32_t>(surveyRemainingSeconds()));
    if (svinValidNow() == 2) return String("Complete");
    if (surveyAwaitingFirstFix) return String("Awaiting fix");
    return formatDuration(static_cast<uint32_t>(surveyRemainingSeconds()));
}

static String surveyCountdownText() {
    if (LG290P_USE_FIXED_BASE) return String("fixed-base mode");
    if (surveyInSkipped) return String("Instant Base manually triggered");
    if (baseUsingSavedPosition()) return String("using confirmed saved position");
    if (!receiverHasSignal()) return String("no satellite signal - check the antenna");
    if (svinValidNow() == 2) return String("complete");
    if (surveyAwaitingFirstFix) return String("waiting for first GPS fix");
    return formatDuration(static_cast<uint32_t>(surveyRemainingSeconds()));
}

static bool haveSurveyStatus() {
    return surveyStatus.lastSeenMs > 0;
}

static bool havePositionFallback() {
    return ggaLastSeenMs > 0 && ggaLat != 0.0 && ggaLon != 0.0;
}

static String surveyObsText() {
    if (haveSurveyStatus())
        return formatDuration(surveyStatus.obs) + " / " + formatDuration(surveyStatus.cfgDur);
    if (surveyInCommandMs > 0) return formatDuration(surveyElapsedSeconds()) + " ESP32 timer";
    return String("--");
}

static String surveyMeanEcefFallbackText(double value, uint8_t decimals = 4) {
    const unsigned int decimalPlaces = static_cast<unsigned int>(decimals);
    if (haveSurveyStatus()) return String(static_cast<double>(value), decimalPlaces);
    if (havePositionFallback()) return String(static_cast<double>(value), decimalPlaces) + " current";
    return String("--");
}

static String surveyDataSourceText() {
    if (haveSurveyStatus()) return String("receiver PQTMSVINSTATUS");
    if (isRtcmBaseActive()) return String("active RTCM base implicit");
    if (havePositionFallback()) return String("current GGA/PVT fallback only");
    return String("no receiver status and no position fallback");
}

static String surveyMeanAccFieldText(uint8_t decimals = 4) {
    const unsigned int decimalPlaces = static_cast<unsigned int>(decimals);
    // REALISTIC ACCURACY (do not simplify back to printing meanAcc directly). Two
    // things are folded in here: (1) the LG290P ZEROES PQTMSVINSTATUS <MeanAcc> the
    // moment it enters fixed mode, so a fixed base would otherwise report 0.0000 m
    // forever — a claim of PERFECT accuracy — for the rest of the session; once fixed,
    // the honest figure is the accuracy achieved by the survey that produced the saved
    // coordinate (g_savedHAcc). (2) realisticAccuracyM() floors whichever of those two
    // numbers at the empirically observed fix scatter — see accumulateSurveyScatter()
    // above for why the receiver's own number runs optimistic.
    // Without satellite signal the only number available is the accuracy of a survey
    // that finished elsewhere, which reads as a live measurement beside a "complete"
    // timer. See receiverHasSignal().
    if (!receiverHasSignal()) return String("-- (no signal)");
    {
        // realisticAccuracyM() owns the source order, including the live PPP survey.
        // Gating this on haveSurveyStatus() first is what made the field read as
        // unavailable through an entire rover-mode survey that had a perfectly good
        // number in it.
        const double acc = realisticAccuracyM();
        if (acc >= 0.0) {
            const char* suffix = " m";
            if (ppp_survey_active())                    suffix = " m (survey)";
            else if (!(surveyStatus.meanAcc > 0.0))     suffix = " m (saved)";
            return String(acc, decimalPlaces) + suffix;
        }
    }
    if (isRtcmBaseActive()) return String("Base Active");
    if (ggaHdop > 0.0) return String("SVIN unavailable; HDOP ") + String(static_cast<double>(ggaHdop), 1);
    return String("receiver SVIN unavailable");
}

static String surveyMeanEcefText(double value, uint8_t decimals = 4) {
    return surveyMeanEcefFallbackText(value, decimals);
}

static String lg290pCommandChannelText() {
    if (lg290pPqtmResponses > 0) {
        const uint32_t age = lg290pLastPqtmResponseMs == 0 ?
        0 : (millis() - lg290pLastPqtmResponseMs) / 1000UL;
        return String("PQTM responses seen: ") + String(lg290pPqtmResponses) + "; last age " + String(age) + " s";
    }
    if (lg290pCommandsSent > 0 && rtcmStats.nmeaSentences > 0) {
        return String("NO PQTM responses; UART RX is alive, but LG290P command RX/status output is not confirmed") ;
    }
    if (lg290pCommandsSent > 0) {
        return String("commands sent; waiting for receiver response");
    }
    return String("no commands sent yet");
}

static String surveyStateText() {
    if (LG290P_USE_FIXED_BASE) return String("Fixed ECEF configured");
    if (surveyInSkipped) return String("Instant Fixed Mode (Manual)");
    if (baseUsingSavedPosition())
        return String("Fixed on saved position - boot check confirmed, no survey needed");
    // Ahead of both the awaiting-fix state and the SVINSTATUS diagnostics. With no antenna
    // the receiver still reports a retained survey as valid, and "awaiting first fix" —
    // while true — is indistinguishable from an ordinary cold start. Naming the missing
    // signal is the more specific and more actionable answer.
    if (!receiverHasSignal()) return String("No satellite signal - check the antenna");
    if (surveyAwaitingFirstFix) return String("Awaiting first GPS fix");
    if (surveyStatus.lastSeenMs == 0) {
        return String("No PQTMSVINSTATUS received; using fallback display");
    }
    // A stale message means the receiver has left base mode (PQTMSVINSTATUS is
    // base-mode-only), which is a different situation from "the survey says invalid"
    // and is worth naming rather than folding into it.
    if (!svinStatusFresh()) return String("Stale - receiver not in base mode");
    if (surveyStatus.valid == 1) return String("In progress");
    if (surveyStatus.valid == 2) return String("Valid/complete");
    return String("Invalid/not started");
}

// ── UART processing ──────────────────────────────────────────────────────────
static void processGnssSerial() {
    size_t processed = 0;
    // Sample ring occupancy BEFORE draining. This is the closest thing to an overflow
    // counter the Arduino API exposes: if this approaches the 4096-byte ring size we are
    // on the edge of losing bytes, and anything at the size is already losing them.
    {
        const int avail = GnssSerial.available();
        if (avail > 0 && (uint32_t)avail > g_uartHighWater) g_uartHighWater = (uint32_t)avail;
    }
    while (GnssSerial.available() > 0 && processed < SERIAL_DRAIN_LIMIT_PER_LOOP) {
        const int value = GnssSerial.read();
        if (value < 0) break;
        const uint8_t b = static_cast<uint8_t>(value);
        processed++;
        rtcmStats.uartBytes++;
        gnssRxWindowBytes++;
        feedNmeaParserByte(b);
        feedRtcmParserByte(b);
    }
}

static void updateMetrics() {
    const uint32_t now = millis();
    if (lastRateUpdateMs == 0) {
        lastRateUpdateMs = now;
        return;
    }
    // RATE WINDOW = 5 s, NOT 1 s (do not revert). RTCM does not arrive smoothly: the
    // LG290P emits its whole message set in one clump at each epoch boundary. A 1 s
    // window therefore samples a burst that is roughly 1 s long, and where the window
    // edge falls decides whether it catches 0 frames or 10. Field session 0454 proves
    // it: on a stream whose true rate was 5.09 +/- 0.29 fps, this counter reported
    // anything from 0 to 31, and correlated with the real rate at only r=0.51. Twelve
    // status rows read "rtcm_fps < 4" while the receiver was in fact producing >4.5.
    // That noise is not cosmetic — rtcm_fps drives the LCD, the dashboard and the CSV,
    // so it manufactures phantom "RTCM dropouts" during diagnosis. A 5 s window spans
    // several epochs and averages the clumping out. Everything else on this tick
    // (gnssRxBps, per-caster bwBytesPerSec) becomes a 5 s mean too, which is likewise
    // steadier and matches the status-log cadence.
    const uint32_t elapsed = now - lastRateUpdateMs;
    if (elapsed < 5000) return;
    gnssRxBps = (gnssRxWindowBytes * 1000UL) / elapsed;
    rtcmStats.framesPerSecond = (rtcmStats.framesWindow * 1000UL) / elapsed;
    rtcmStats.bytesPerSecond = (rtcmStats.bytesWindow * 1000UL) / elapsed;
    // Per-caster live uplink rate — same 1 s tick. bwWindowBytes is the bytes
    // accepted since the last roll; convert to bytes/sec and reset the window.
    for (auto& c : casters) {
        c.bwBytesPerSec   = (c.bwWindowBytes * 1000UL) / elapsed;
        c.bwWindowBytes   = 0;
        c.bwWindowStartMs = now;
    }
    gnssRxWindowBytes = 0;
    rtcmStats.framesWindow = 0;
    rtcmStats.bytesWindow = 0;
    lastRateUpdateMs = now;
}

// ── Wi-Fi ────────────────────────────────────────────────────────────────────
static void connectToWifiIndex(int index) {
    const int total = (int)g_wifiNetworks.size();
    if (total <= 0) return;
    if (index < 0 || index >= total) index = 0;
    wifiAttemptIndex = index;
    activeWifiIndex = -1;
    wifiAttemptStartMs = millis();
    // No WiFi.disconnect() here: WiFi.begin() supersedes a pending or current
    // association on its own. Calling disconnect() between attempts tears down
    // and rebuilds the LwIP network interface — on this hardware that has twice
    // been root-caused to WiFi connections silently getting stuck and, on the
    // companion firmware, to the web server's listen socket being orphaned when
    // the interface rebuilds underneath it. Do not reintroduce it.
    // Open networks (no password) get the single-argument overload rather than an
    // empty passphrase string — this is what actually tells the WiFi stack to treat
    // the network as open rather than trying to negotiate security against it.
    const WifiEntry& target = g_wifiNetworks[wifiAttemptIndex];
    if (target.password.length() == 0) {
        WiFi.begin(target.ssid.c_str());
    } else {
        WiFi.begin(target.ssid.c_str(), target.password.c_str());
    }
    Serial.printf("📶 Trying WiFi [%d/%d]: %s%s\n",
                  wifiAttemptIndex + 1, total,
                  target.ssid.c_str(),
                  target.password.length() == 0 ? " (open)" : "");
}

static void serviceWifi() {
    const int total = (int)g_wifiNetworks.size();
    if (total <= 0) return;
    if (wifiLinkUp()) {                   // DEBOUNCED — a blip must not start a rescan
        if (activeWifiIndex == -1 && WiFi.status() == WL_CONNECTED) {
            activeWifiIndex = wifiAttemptIndex;
            Serial.printf("📶 Connected: %s  %s\n",
                          g_wifiNetworks[activeWifiIndex].ssid.c_str(),
                          WiFi.localIP().toString().c_str());
        }
        return;
    }
    activeWifiIndex = -1;
    if (millis() - wifiAttemptStartMs < WIFI_CONNECT_TIMEOUT_MS) return;
    // ROLLING 60s EXCLUSIVITY WATCHDOG. AP and STA share one radio, so every WiFi.begin()
    // below forces it briefly off-channel to find the target SSID — if the SoftAP's
    // channel differs from where it lands, the SoftAP is dragged along with it, and
    // whatever is associated (a rover, someone's phone, a laptop on the dashboard) drops
    // and has to reconnect. Pausing indefinitely for as long as ANYTHING is associated
    // trades that for a different failure: one idle or unintended device parked on the AP
    // would block the base from ever finding its own uplink again. So the pause is bounded:
    // it holds only while there has been genuine activity — a dashboard action or RTCM
    // actually flowing to a rover, both tracked in g_wifiActivityMs (see its declaration)
    // — within the last WIFI_SEARCH_PAUSE_WINDOW_MS. A device merely sitting connected
    // with nothing happening stops protecting itself once that window lapses, and search
    // resumes even though it is still associated.
    //
    // This does NOT apply to a network the user explicitly picks: handleApiAddWifi()'s
    // Add & Connect calls connectToWifiIndex() directly, bypassing serviceWifi() entirely,
    // so that click always takes effect immediately regardless of AP state or this window.
    const uint32_t now         = millis();
    const bool apOccupied      = WiFi.softAPgetStationNum() > 0;
    const uint32_t sinceActivity = (uint32_t)(now - g_wifiActivityMs);
    const bool withinActivity  = sinceActivity < WIFI_SEARCH_PAUSE_WINDOW_MS;
    if (apOccupied && withinActivity) {
        if (!g_wifiSearchPausedForAp) {
            g_wifiSearchPausedForAp = true;
            // sinceActivity < WIFI_SEARCH_PAUSE_WINDOW_MS is guaranteed by withinActivity
            // above, using the SAME `now` — so this subtraction cannot underflow. Do not
            // split it back into two separate millis() calls; a fresh call here could
            // cross the window boundary between the check and this line and wrap the
            // unsigned subtraction to a huge bogus value for one log line.
            Serial.printf("📶 WiFi search paused — AP active, %lus left in window\n",
                          (unsigned long)((WIFI_SEARCH_PAUSE_WINDOW_MS - sinceActivity) / 1000UL));
        }
        wifiAttemptStartMs = now;          // re-arm the same window, do not tight-check
        return;
    }
    if (g_wifiSearchPausedForAp) {
        g_wifiSearchPausedForAp = false;
        Serial.println(apOccupied ? "📶 WiFi search resumed — activity window lapsed"
                                   : "📶 WiFi search resumed — AP is clear");
    }
    connectToWifiIndex((wifiAttemptIndex + 1) % total);
}

// ── Web server ───────────────────────────────────────────────────────────────
static int32_t ageSeconds(uint32_t lastSeenMs) {
    if (lastSeenMs == 0) return -1;
    return static_cast<int32_t>((millis() - lastSeenMs) / 1000UL);
}

static void sendRingDownload(const uint8_t* data, size_t head, size_t count, size_t capacity, const char* filename) {
    server.sendHeader("Content-Disposition", String("attachment; filename=\"") + filename + "\"");
    server.sendHeader("Cache-Control", "no-store");
    server.setContentLength(count);
    server.send(200, "application/octet-stream", "");
    WiFiClient client = server.client();
    size_t index = oldestIndex(head, count, capacity);
    size_t remaining = count;
    while (remaining > 0 && client.connected()) {
        const size_t contiguous = min(remaining, capacity - index);
        const size_t written = client.write(&data[index], contiguous);
        if (written == 0) break;
        remaining -= written;
        index = (index + written) % capacity;
        delay(0);
    }
}

static void handleRtcmRaw() {
    sendRingDownload(rtcmValidCapture.data, rtcmValidCapture.head, rtcmValidCapture.count,
                     RTCM_VALID_CAPTURE_SIZE, "validated_rtcm_recent.bin");
}

static void handleCaster0Raw() {
    sendRingDownload(casterTxCaptures[0].data, casterTxCaptures[0].head, casterTxCaptures[0].count,
                     CASTER_TX_CAPTURE_SIZE, "caster1_valid_rtcm_written.bin");
}

static void handleCaster1Raw() {
    sendRingDownload(casterTxCaptures[1].data, casterTxCaptures[1].head, casterTxCaptures[1].count,
                     CASTER_TX_CAPTURE_SIZE, "caster2_valid_rtcm_written.bin");
}

// Space left in a snprintf accumulation buffer, saturating at zero.
//
// WHY THIS EXISTS (do not go back to writing `sizeof(buf) - n` directly). snprintf()
// returns the length it WOULD have written, not the length it wrote, so the running total
// keeps climbing after the buffer is full. Once it passes the buffer size, `sizeof(buf) - n`
// is unsigned subtraction below zero: it wraps to roughly 4 billion, and the very next
// snprintf is told it has unlimited room and writes off the end of a static buffer into
// whatever follows it. Every call in the status document is a link in one chain, so a
// single overlong field anywhere ahead of it arms that. With this helper the size argument
// reaches zero instead, snprintf writes nothing, and the document truncates — a stale
// dashboard rather than corrupted memory. The per-record guards in the caster and RTCM-type
// loops stay: they keep the JSON VALID at realistic configurations, and this keeps it SAFE
// at any configuration.
static inline size_t jsonRoom(int n, size_t cap) {
    return (n >= 0 && (size_t)n < cap) ? cap - (size_t)n : 0;
}

// JSON-safe copy: drop quotes/backslashes/control chars from free-text fields.
static const char* jsonSanitize(char* dst, size_t cap, const char* src) {
    size_t j = 0;
    for (size_t i = 0; src && src[i] && j < cap - 1; ++i) {
        char c = src[i];
        if (c == '"' || c == '\\' || (unsigned char)c < 0x20) continue;
        dst[j++] = c;
    }
    dst[j] = '\0';
    return dst;
}

static void handleApiStatus() {
    // Sized for up to MAX_CASTERS entries + RTCM type table. Each caster entry now also
    // carries its last rejection banner (up to 120 sanitized characters), so the caster
    // array is the term that grows with configuration rather than a fixed cost.
    static char buf[5632];
    int n = 0;
    const bool wifi = (WiFi.status() == WL_CONNECTED);
    String ip = wifi ? WiFi.localIP().toString() : String("");
    const int curWifiIdx = (activeWifiIndex >= 0) ? activeWifiIndex : wifiAttemptIndex;
    const char* ssid = (wifi && curWifiIdx < (int)g_wifiNetworks.size())
                       ? g_wifiNetworks[curWifiIdx].ssid.c_str() : "";
    const bool venueConfirmed = g_displacementCheckDone || !g_savedPositionValid;
    const bool castersHeld = !(surveyIsReady() && venueConfirmed && g_castQualityOk);  // intentionally not broadcasting yet
    n += snprintf(buf + n, jsonRoom(n, sizeof buf),
        "{\"dev\":\"%s\",\"suggestMount\":\"%s\",\"up\":%lu,\"wifi\":%d,\"ssid\":\"%s\",\"ip\":\"%s\",\"ready\":%d,\"held\":%d,"
        "\"local\":{\"on\":%d,\"port\":%d,\"clients\":%d,\"served\":%lu,\"apip\":\"%s\","
        "\"name\":\"%s\"},",
        g_deviceName.c_str(), g_suggestedMount.c_str(), (unsigned long)(millis() / 1000UL), wifi ? 1 : 0, ssid, ip.c_str(),
        baseReady ? 1 : 0, castersHeld ? 1 : 0,
        g_localCasterEnabled ? 1 : 0, LOCAL_CASTER_PORT, localCasterActiveCount(),
        (unsigned long)g_localCasterServed, WiFi.softAPIP().toString().c_str(),
        LOCAL_CASTER_NAME);
    n += snprintf(buf + n, jsonRoom(n, sizeof buf),
        "\"svin\":{\"state\":\"%s\",\"valid\":%u,\"obs\":%lu,\"target\":%lu,\"acc\":%.3f,\"accknown\":%d,"
        "\"remain\":%ld,\"skipped\":%d,\"fixed\":%d,\"sig\":%d,\"saved\":%d},",
        surveyStateText().c_str(), (unsigned)svinValidNow(),
        (unsigned long)surveyElapsedSeconds(), (unsigned long)surveyTargetSeconds(),
        // Realistic (scatter-floored) accuracy — see realisticAccuracyM(). Same fixed-mode
        // substitution as surveyMeanAccFieldText(): meanAcc is 0 once fixed, so this falls
        // back to the saved coordinate's own accuracy.
        //
        // "accknown" carries whether there is a figure at all, because the number alone
        // cannot say so. realisticAccuracyM() returns -1 when neither a survey result nor a
        // saved coordinate exists, and clamping that to 0 published the most precise-looking
        // value in the format for the least-known state in the system: a base that has been
        // powered on for two seconds, or one running a rover-mode PPP survey — where
        // PQTMSVINSTATUS never arrives at all, so the whole 45-minute window reads as
        // 0.000 m. Unknown is now distinguishable from perfect, and the dashboard shows a
        // dash for it.
        // lint: sentinel-ok — the clamp is paired with "accknown" in the same object, so
        // "unknown" is carried explicitly rather than being encoded as the value 0.
        fmax(realisticAccuracyM(), 0.0), realisticAccuracyM() >= 0.0 ? 1 : 0,
        (long)surveyRemainingSeconds(), surveyInSkipped ? 1 : 0, LG290P_USE_FIXED_BASE ? 1 : 0,
        receiverHasSignal() ? 1 : 0,
        // Running on a boot-confirmed saved coordinate: no survey, and none pending. The
        // page cannot infer this — valid is 0 and remain is 0 on this path just as they
        // are before a survey starts — so it is stated explicitly.
        baseUsingSavedPosition() ? 1 : 0);
    n += snprintf(buf + n, jsonRoom(n, sizeof buf),
        "\"pos\":{\"fix\":%d,\"sats\":%d,\"hdop\":%.2f,\"lat\":%.9f,\"lon\":%.9f,\"alt\":%.3f,\"epe\":%.3f,\"epe2d\":%.3f},",
        ggaFixQuality, ggaSatellites, ggaHdop, ggaLat, ggaLon, ggaAlt,
        liveErrorMeters > 0 ? liveErrorMeters : 0.0, liveErrorMeters2D > 0 ? liveErrorMeters2D : 0.0);
    // Temperatures in °F. chipOk is always 1 (internal sensor); boardOk reflects QMI8658
    // presence/health. The web page colour-codes these against the same bands as the LCD.
    n += snprintf(buf + n, jsonRoom(n, sizeof buf),
        "\"temp\":{\"chipF\":%.1f,\"boardF\":%.1f,\"chipOk\":%d,\"boardOk\":%d},",
        isnan(g_chipTempC) ? 0.0f : cToF(g_chipTempC),
        g_boardTempOk ? cToF(g_boardTempC) : 0.0f,
        isnan(g_chipTempC) ? 0 : 1, g_boardTempOk ? 1 : 0);
    n += snprintf(buf + n, jsonRoom(n, sizeof buf),
        "\"rtcm\":{\"fps\":%lu,\"bps\":%lu,\"valid\":%llu,\"cand\":%lu,\"crcfail\":%lu,\"framefail\":%lu,\"obs\":%d},",
        (unsigned long)rtcmStats.framesPerSecond, (unsigned long)rtcmStats.bytesPerSecond,
        (unsigned long long)rtcmStats.rtcmValidFrames, (unsigned long)rtcmStats.rtcmCandidateFrames,
        (unsigned long)rtcmStats.rtcmCrcFailures, (unsigned long)rtcmStats.rtcmFramingFailures,
        observationsFlowing() ? 1 : 0);
    n += snprintf(buf + n, jsonRoom(n, sizeof buf), "\"casters\":[");
    for (int i = 0; i < casterCount; ++i) {
        // A caster record is the widest repeating element in this document — host, mount,
        // state, the caster's verbatim banner and a dozen counters, roughly 550 bytes each
        // — and up to MAX_CASTERS of them precede everything else. Stop before the fields
        // that follow lose their room, so a crowded configuration drops the last caster's
        // detail rather than the survey, position and diagnostic sections the dashboard
        // needs to explain why nothing is casting.
        if (jsonRoom(n, sizeof buf) < 1400) break;
        // Lifetime average uplink rate: total accepted bytes / seconds since first frame.
        const uint32_t elapsedS = casters[i].firstWriteMs
            ? (uint32_t)((millis() - casters[i].firstWriteMs) / 1000UL) : 0;
        const unsigned long avgBps = (elapsedS > 0)
            ? (unsigned long)(casters[i].bytesAccepted / elapsedS) : 0UL;
        char errEsc[80], hostEsc[80], mountEsc[60], respEsc[120];
        n += snprintf(buf + n, jsonRoom(n, sizeof buf),
            "%s{\"host\":\"%s\",\"port\":%u,\"mount\":\"%s\",\"state\":\"%s\",\"en\":%d,"
            "\"hand\":%d,\"frames\":%llu,\"bytes\":%llu,\"drop\":%lu,\"age\":%ld,\"live\":%lu,\"avg\":%lu,"
            "\"err\":\"%s\",\"resp\":\"%s\",\"offered\":%llu,\"buf\":%u,\"cong\":%d}",
            i ? "," : "",
            jsonSanitize(hostEsc, sizeof(hostEsc), casters[i].host), (unsigned)casters[i].port,
            jsonSanitize(mountEsc, sizeof(mountEsc), casters[i].mountpoint),
            stateToString(casters[i].state),
            casters[i].enabled ? 1 : 0,
            casters[i].handshakeAccepted ? 1 : 0,
            (unsigned long long)casters[i].framesWritten, (unsigned long long)casters[i].bytesAccepted,
            (unsigned long)casters[i].droppedWriteCount, (long)ageSeconds(casters[i].lastWriteMs),
            (unsigned long)casters[i].bwBytesPerSec, avgBps,
            jsonSanitize(errEsc, sizeof(errEsc), casters[i].lastError),
            jsonSanitize(respEsc, sizeof(respEsc), casters[i].lastResponseHeader),
            (unsigned long long)casters[i].bytesOffered, (unsigned)casters[i].txLen,
            casters[i].congestionStartMs ? 1 : 0);
    }
    n += snprintf(buf + n, jsonRoom(n, sizeof buf), "],");
    n += snprintf(buf + n, jsonRoom(n, sizeof buf),
        "\"log\":{\"gps\":%d,\"sat\":%d,\"status\":%d,\"rtcm\":%d,\"raw\":%d,\"event\":%d,"
        "\"disp\":%d,\"sd\":%d,\"drop\":%lu,\"session\":%u},",
        bridge_sdlog_get_channel(BLOG_GPS) ? 1 : 0, bridge_sdlog_get_channel(BLOG_SAT) ? 1 : 0,
        bridge_sdlog_get_channel(BLOG_STATUS) ? 1 : 0, bridge_sdlog_get_channel(BLOG_RTCM) ? 1 : 0,
        bridge_sdlog_get_channel(BLOG_RAW) ? 1 : 0,
        bridge_sdlog_get_channel(BLOG_EVENT) ? 1 : 0, displayTelemetryEnabled ? 1 : 0,
        bridge_sdlog_ready() ? 1 : 0, (unsigned long)bridge_sdlog_dropped(), (unsigned)bridge_sdlog_session());
    // cfg mirrors what the OPERATOR set. Distinct from svin.target, which may reflect the
    // receiver's own configured duration; the dashboard's input field must round-trip
    // this value or a "keep surveying" change looks like it did nothing.
    n += snprintf(buf + n, jsonRoom(n, sizeof buf),
                  "\"pppsol\":{\"sta\":%d,\"age\":%.1f,\"fixms\":%d},",
                  ggaDiffStation, ggaDiffAgeS, g_probedFixRateMs);
    n += snprintf(buf + n, jsonRoom(n, sizeof buf), "\"cfg\":{\"acc\":%.2f,\"time\":%lu,\"ppp\":%d},",
                  (double)surveyAccLimit, (unsigned long)surveyInSec,
                  BASE_SURVEY_USE_PPP ? 1 : 0);
    // Correction-quality gate. "ok" is the hysteretic verdict the streaming gate reads;
    // "why" is empty when ok. crcrate is surfaced because it is the one condition whose
    // threshold an operator may legitimately want to argue with.
    {
        char qEsc[80];
        n += snprintf(buf + n, jsonRoom(n, sizeof buf),
                      "\"qual\":{\"ok\":%d,\"why\":\"%s\",\"crcrate\":%.4f},",
                      g_castQualityOk ? 1 : 0,
                      jsonSanitize(qEsc, sizeof(qEsc), g_castQualityReason),
                      (double)g_crcFailRate);
    }
    // What the RECEIVER says about PPP, as opposed to what this firmware asked for.
    // "sup" is -1 until PQTMCFGPPP,R has been answered (or gone unanswered); "e6" is the
    // Galileo E6 satellite count the PPP survey's own GSV monitor is seeing, which is
    // the direct measure of whether HAS corrections can arrive at all.
    {
        char pEsc[100], sEsc[100], nEsc[120];
        const PppSurveyStatus pst = ppp_survey_status();
        n += snprintf(buf + n, jsonRoom(n, sizeof buf),
                      "\"ppp\":{\"sup\":%d,\"cfg\":\"%s\",\"sig\":\"%s\",\"e6\":%lu,\"e6ok\":%d,\"act\":%d,"
                      "\"nav\":\"%s\",\"navage\":%ld},",
                      (int)g_pppSupported,
                      jsonSanitize(pEsc, sizeof(pEsc), g_pppCfgReply),
                      jsonSanitize(sEsc, sizeof(sEsc), g_signalCfgReply),
                      (unsigned long)pst.e6Sats, pst.e6Ok ? 1 : 0,
                      // Whether a rover-mode survey is running. The E6 count is only
                      // meaningful while it is: once the base is fixed the receiver stops
                      // tracking for a solution and the count falls to zero, which is
                      // normal and not worth a red zero on the dashboard.
                      ppp_survey_active() ? 1 : 0,
                      jsonSanitize(nEsc, sizeof(nEsc), g_pppNavSentence),
                      (long)ageSeconds(g_pppNavLastMs));
    }
    // What the RECEIVER says its own configuration is, from the last PQTMCFG*,R probe.
    // This is the visible result of "Read module config" — previously the replies only
    // went to the serial log, which is gated off by default, so the button looked inert.
    n += snprintf(buf + n, jsonRoom(n, sizeof buf),
                  "\"mod\":{\"mode\":%d,\"svin\":%u,\"dur\":%lu,\"acc\":%.2f},",
                  (int)g_probedRcvrMode, (unsigned)g_probedSvinMode,
                  (unsigned long)g_probedSvinDur, (double)g_probedSvinAcc);
    // Decoded RTCM type table: {t:type, n:name, c:count, age:ms-since-seen, obs:isMSM}
    // Completeness audit: which of EXPECTED_RTCM_TYPES has the receiver never actually
    // produced. An empty list is a complete stream; anything in it is a message a rover
    // is entitled to and is not getting.
    n += snprintf(buf + n, jsonRoom(n, sizeof buf), "\"missing\":[");
    {
        bool firstMissing = true;
        for (uint16_t want : EXPECTED_RTCM_TYPES) {
            if (rtcmTypeEverSeen(want)) continue;
            n += snprintf(buf + n, jsonRoom(n, sizeof buf), "%s%u", firstMissing ? "" : ",", (unsigned)want);
            firstMissing = false;
        }
    }
    n += snprintf(buf + n, jsonRoom(n, sizeof buf), "],");
    // Antenna descriptor from RTCM 1033. Reported separately from the completeness list
    // because an EMPTY descriptor is a different kind of problem from a missing message:
    // the message is present and well-formed, and the field a rover needs is blank.
    {
        char aEsc[48];
        n += snprintf(buf + n, jsonRoom(n, sizeof buf), "\"ant\":{\"seen\":%d,\"len\":%u,\"desc\":\"%s\"},",
                      g_antennaDescSeen ? 1 : 0, (unsigned)g_antennaDescLen,
                      jsonSanitize(aEsc, sizeof(aEsc), g_antennaDesc));
    }
    n += snprintf(buf + n, jsonRoom(n, sizeof buf), "\"types\":[");
    bool first = true;
    const uint32_t nowMs = millis();
    for (const auto& stat : rtcmStats.typeStats) {
        if (stat.type == 0) continue;
        // Reserve the tail: the position-check, antenna, PPP and last-sentence sections all
        // follow this list — the saved-position block alone is ~400 bytes and the trailing
        // message block ~420 — and 96 bytes covered only the last of them.
        if (jsonRoom(n, sizeof buf) < 900) break;
        char nm2[40];
        jsonSanitize(nm2, sizeof(nm2), rtcmTypeName(stat.type));
        n += snprintf(buf + n, jsonRoom(n, sizeof buf),
            "%s{\"t\":%u,\"n\":\"%s\",\"c\":%lu,\"age\":%lu,\"obs\":%d}",
            first ? "" : ",", (unsigned)stat.type, nm2, (unsigned long)stat.count,
            (unsigned long)(stat.lastSeenMs ? nowMs - stat.lastSeenMs : 0),
            isMsmObservation(stat.type) ? 1 : 0);
        first = false;
    }
    n += snprintf(buf + n, jsonRoom(n, sizeof buf), "],");
    // Boot position-confidence check + live PQTMNAV solution + saved absolute position.
    const char* pcStateStr[] = {"idle","collecting","confirmed","moved","timeout"};
    const int pcIdx = static_cast<int>(g_posCheckState);
    const char* navSolStr =
        navSolType == 12 ? "RTK fixed" : navSolType == 8 ? "RTK float" :
        navSolType == 5  ? "PR diff"   : navSolType == 2 ? "SBAS"      :
        navSolType == 1  ? "single"    : "no fix";
    n += snprintf(buf + n, jsonRoom(n, sizeof buf),
        "\"poscheck\":{\"state\":\"%s\",\"n\":%lu,\"need\":%lu,\"dist\":%.3f,\"thresh\":%.3f,"
        "\"elapsed\":%lu,\"soltype\":%d,\"sol\":\"%s\",\"reason\":\"%s\"},"
        "\"savedpos\":{\"valid\":%d,\"lat\":%.9f,\"lon\":%.9f,\"alt\":%.3f,\"hacc\":%.3f,\"src\":\"%s\",\"svsec\":%lu,\"note\":\"%s\"},",
        pcStateStr[(pcIdx >= 0 && pcIdx < 5) ? pcIdx : 0],
        (unsigned long)g_posCheckCount, (unsigned long)POSCHECK_MIN_FIXES,
        g_posCheckDistM, g_posCheckThreshM,
        g_posCheckStartMs ? (unsigned long)((millis() - g_posCheckStartMs) / 1000UL) : 0UL,
        navSolType, navSolStr, g_posCheckReason,
        g_savedPositionValid ? 1 : 0,
        g_savedLat, g_savedLon, g_savedAlt, (double)g_savedHAcc, g_savedSource,
        (unsigned long)g_savedSurveySec, g_savedNote);
    char pq[170], nm[200];
    n += snprintf(buf + n, jsonRoom(n, sizeof buf), "\"msg\":{\"pqtm\":\"%s\",\"nmea\":\"%s\"}}",
        jsonSanitize(pq, sizeof(pq), lg290pLastResponse),
        jsonSanitize(nm, sizeof(nm), lastGnssSentence));
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", buf);
}

// ── Warm survey restart (NO module reset — see why) ──────────────────────────
// configureLg290pBaseOnce() ends in PQTMSAVEPAR + a restart, which is heavier than a
// target change needs: it re-runs the whole module configuration. None of that is
// needed to change survey TARGETS, because no receiver
// MODE change is involved — PQTMCFGSVIN applies live, and the module is already in base
// mode with its RTCM rates, telemetry rates and PPP settings persisted from the last
// full configure. Re-issuing PQTMCFGSVIN alone retargets the survey while the receiver
// keeps tracking, so the restart costs seconds instead of minutes.
//
// A RESTART is required after a PQTMCFGRCVRMODE change (the spec is explicit: save, then
// restart, or the module keeps operating in the previous mode), and on this module that
// restart has to be PQTMSRR — the PPP survey's rover→base handoff ends in PQTMSRR for
// exactly that reason, and a hot start in the same role was observed NOT to apply the
// mode change. Do not generalise this warm path to anything that switches receiver mode.
//
// SCOPE: survey targets only. Anything that changes the module's OUTPUT configuration
// (message set, protocol, rates) still goes through configureLg290pBaseOnce(true).
// The flag itself is declared with the other deferred-work flags near the top of the file,
// because updateBaseModeWatchdog() has to see it to know the receiver is already being
// driven.

static void restartSurveyWarm() {
    // Clear the "we already have a position" state FIRST, for BOTH survey paths. These
    // flags are what make a base skip surveying, and the PPP branch below returns early —
    // so leaving them set meant a restart request quietly did nothing and the base
    // stayed fixed on the old coordinate.
    resetSurveyBookkeeping();
    g_useNvsSavedPosition   = false;   // do not re-adopt the saved coordinate
    g_displacementCheckDone = true;    // and do not let the boot check re-confirm it
    g_posCheckState         = PosCheckState::Idle;

    if (BASE_SURVEY_USE_PPP) {
        ppp_survey_abort();

        // ESCAPE FIXED MODE FIRST — this is the step whose absence made "clear position"
        // and "restart survey" appear to do nothing. The module is a PERSISTED fixed
        // base (svinMode 2, PQTMSAVEPAR'd), and from that state a bare rover-mode
        // command does not take effect (field-proven: the Chesapeake→Hampton
        // move-detection failure). The PPP survey's own configure step sends exactly
        // that bare command, so the module stayed a fixed base, kept echoing its stored
        // coordinate with a tiny EPE, every echoed fix sailed through the convergence
        // gate — and the "survey" locked the very coordinate the operator was trying to
        // discard. Rover + save + hot restart is the sequence that demonstrably escapes
        // fixed mode; the hot start retains ephemeris so reacquisition is quick.
        // The fixed-mode escape runs in servicePppSurveyStart() — the single gate every
        // survey start passes through — so queuing is all this path needs to do.
        g_pppSurveyPending = true;
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "PPP survey restart");
        Serial.printf("🔄 PPP survey restart: window %s, EPE gate %.2fm\n",
                      formatDuration(surveyInSec).c_str(), (double)surveyAccLimit);
        return;
    }
    char payload[128];
    snprintf(payload, sizeof(payload), "PQTMCFGSVIN,W,1,%lu,%.2f,0,0,0",
             static_cast<unsigned long>(surveyInSec), static_cast<double>(surveyAccLimit));
    sendPqtm(payload);
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    // Reset OUR survey bookkeeping to match the module's fresh survey.
    resetSurveyBookkeeping();
    copyLimited(lg290pModeText, sizeof(lg290pModeText), "survey restart (warm)");
    Serial.printf("🔄 Warm survey restart: target %lus, acc limit %.2fm (no module reset)\n",
                  static_cast<unsigned long>(surveyInSec), static_cast<double>(surveyAccLimit));
}

static void handleApiConfig() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // POST /api/config?time=<s>&acc=<m>[&restart=1]
    // Survey targets take effect IMMEDIATELY and by default WITHOUT disturbing the
    // receiver: surveyInSec drives the countdown, and surveyAccLimit is the corroborating
    // accuracy gate in updateBaseReadiness(), both evaluated on the ESP every pass. An
    // in-progress survey therefore keeps its accumulated observations while the operator
    // retargets it — tightening the limit simply holds readiness until the module's
    // reported accuracy meets the new figure.
    // restart=1 additionally pushes the new targets into the module and starts its survey
    // over (warm — no receiver reset; see restartSurveyWarm).
    bool changed = false;
    if (server.hasArg("time")) { long t = server.arg("time").toInt(); if (t >= 30 && t <= 86400) { surveyInSec = (uint32_t)t; changed = true; } }
    if (server.hasArg("acc"))  { float a = server.arg("acc").toFloat(); if (a >= 0.1f && a <= 100.0f) { surveyAccLimit = a; changed = true; } }
    if (changed) {
        preferences.begin("xbee", false);
        preferences.putUInt("survey_sec", surveyInSec);
        preferences.putFloat("survey_acc", surveyAccLimit);
        preferences.end();
    }
    if (changed) {
        // PUSH THE WINDOW ONLY. surveyAccLimit is the MODULE's internal survey-in accuracy
        // target (default 3.20 m) and is not the same quantity as the PPP acceptance gate,
        // which is a per-epoch EPE limit around 0.30 m. Passing it here replaced a decimetre
        // gate with a 3.20 m one — and, through PPP_SURVEY_EPE_3D_RATIO, a 6.40 m 3D
        // partner — so every fix passed, including plain autonomous ones, and the survey
        // locked an unconverged average while reporting it as PPP-converged. It also made
        // the effect invisible: the gate widened whenever ANY field on the card was saved,
        // including a change to the window alone, and reverted only at the next reboot.
        // The survey's own limits are set from PPP_SURVEY_EPE_* and are changed on the PPP
        // page, which is where they are shown; zeros here leave them untouched, matching
        // what servicePppSurveyStart() passes when it launches a survey.
        ppp_survey_set_criteria(0.0f, 0.0f, 0, surveyInSec);
    }
    const bool restart = server.hasArg("restart") && server.arg("restart") == "1";
    if (restart) g_surveyRestartPending = true;   // deferred to loop(); no blocking in the handler
    server.send(200, "text/plain", changed ? (restart ? "ok restart" : "ok live") : "unchanged");
}

static void handleApiLog() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // DIAGNOSTIC: print everything the server received so we can see if the POST is
    // arriving and what it contains, regardless of how args were sent.
    Serial.printf("🔧 /api/log: method=%s args=%d body='%s'\n",
                  server.method() == HTTP_POST ? "POST" : "GET",
                  server.args(),
                  server.arg("plain").c_str());   // "plain" = raw body on WebServer

    // WebServer::arg() reliably reads POST body fields (application/x-www-form-urlencoded)
    // but sometimes misses URL query-string params on POST. If normal arg() fails, fall
    // back to manually parsing the raw body from server.arg("plain").
    String ch = server.arg("ch");
    String onStr = server.arg("on");
    if (ch.isEmpty()) {
        // Manual parse of raw body "ch=disp&on=0"
        String body = server.arg("plain");
        int ci = body.indexOf("ch=");
        if (ci >= 0) {
            int end = body.indexOf('&', ci + 3);
            ch = (end < 0) ? body.substring(ci + 3) : body.substring(ci + 3, end);
        }
        int oi = body.indexOf("on=");
        if (oi >= 0) {
            int end = body.indexOf('&', oi + 3);
            onStr = (end < 0) ? body.substring(oi + 3) : body.substring(oi + 3, end);
        }
    }
    if (ch.isEmpty()) { server.send(400, "text/plain", "ch?"); return; }
    bool on = onStr.toInt() != 0;

    Serial.printf("   → ch='%s' on=%d\n", ch.c_str(), on);

    if      (ch == "gps")    bridge_sdlog_set_channel(BLOG_GPS, on);
    else if (ch == "sat")    bridge_sdlog_set_channel(BLOG_SAT, on);
    else if (ch == "status") bridge_sdlog_set_channel(BLOG_STATUS, on);
    else if (ch == "rtcm")   bridge_sdlog_set_channel(BLOG_RTCM, on);
    else if (ch == "raw")    bridge_sdlog_set_channel(BLOG_RAW, on);
    else if (ch == "event")  bridge_sdlog_set_channel(BLOG_EVENT, on);
    else if (ch == "disp")   {
        displayTelemetryEnabled = on;
        Serial.printf("🖥️  Display toggle: on=%d → ledcWrite(%d) [active-high]\n",
                      on, on ? LCD_BL_FULL_DUTY : LCD_BL_OFF_DUTY);
        if (!on) {
            tft.fillScreen(UI_BG);
            ledcWrite(PIN_LCD_BL, LCD_BL_OFF_DUTY);    // duty 0 → backlight off
        } else {
            ledcWrite(PIN_LCD_BL, LCD_BL_FULL_DUTY);   // full duty → backlight on
            g_displayNeedsRedraw = true;
        }
    }
    else { server.send(400, "text/plain", "bad ch"); return; }
    server.send(200, "text/plain", "ok");
}

// Lock into fixed base at the current position — the dashboard's "Force base now".
//
// This was written when the module lived in base mode and ran the receiver's own
// survey-in, where writing PQTMCFGSVIN,W,2 was the whole job. Under the rover-first
// architecture it was a no-op with an "ok" reply: the module is a ROVER whenever a PPP
// survey is running, a survey-in configuration written to a rover changes nothing, the
// survey kept running and re-issued its own rover commands over the top, and no RTCM
// output configuration was ever applied so nothing could stream even if base mode had
// taken. All four are handled below, in the order the module requires.
//
// Deferred to loop context via g_forceBasePending rather than run inside the HTTP
// handler: the sequence includes a module reset and blocks for seconds, and the
// established rule in this firmware is that anything that heavy runs in loop() where it
// cannot stall the web server or starve the GNSS UART.
static void handleApiForce() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // A coordinate is required, and 0,0 is the receiver's "no fix" placeholder, never a
    // real base position. Refusing loudly beats the old silent "ok" on no fix.
    if (ecefX == 0.0 && ecefY == 0.0) {
        server.send(409, "text/plain",
                    "no position yet - force needs a live fix to lock onto");
        return;
    }
    g_forceBasePending = true;
    server.send(200, "text/plain", "ok");
}

// Runs in loop(). See handleApiForce() for why.
static void serviceForceBase() {
    if (!g_forceBasePending) return;
    g_forceBasePending = false;
    if (ecefX == 0.0 && ecefY == 0.0) return;   // fix lost between request and service

    const double fx = ecefX, fy = ecefY, fz = ecefZ;
    Serial.printf("📌 Force base at %.4f,%.4f,%.4f\n", fx, fy, fz);

    // 1. Stop the PPP survey. Left running it advances its own state machine and writes
    //    rover-mode commands over everything below.
    ppp_survey_abort();
    g_pppSurveyPending   = false;
    g_pppSurveyRequested = false;

    // 2. Receiver mode to base. Without this the coordinate goes to a rover, which has
    //    no reference position to apply it to. Save + reset is mandatory for a mode
    //    change out of rover — a bare write does not take.
    char payload[160];
    sendPqtm("PQTMCFGRCVRMODE,W,2");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    snprintf(payload, sizeof(payload), "PQTMCFGSVIN,W,2,0,0,%.4f,%.4f,%.4f", fx, fy, fz);
    sendPqtm(payload);
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMSAVEPAR");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMSRR");
    waitAndDrainGnss(LG290P_POST_RESET_WAIT_MS);

    // 3. Confirm the mode took, rather than assuming it. A forced base that silently
    //    stayed a rover is the failure this whole sequence exists to avoid.
    g_probedRcvrMode = -1;
    sendPqtm("PQTMCFGRCVRMODE,R");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);

    // 4. Output configuration, or the base publishes descriptors and no observations.
    applySatelliteGating();
    enableLg290pBaseOutputs();

    if (g_probedRcvrMode == 2) {
        g_baseModeConfirmed = true;   // read-back saw base mode; see baseUsingSavedPosition()
        surveyInSkipped    = true;    // operator asserted this coordinate
        surveyInCompleteMs = millis();
        g_displacementCheckDone = true;
        // Persist it, so this is a real base across a reboot rather than until the next
        // power cycle. Accuracy is the live scatter — an operator-forced coordinate has
        // no survey behind it and must not claim one.
        const double acc = surveyScatterSigmaM();
        double la, lo, al;
        ecefToLla(fx, fy, fz, la, lo, al);
        savePositionToNvs(la, lo, al, (float)(acc > 0.0 ? acc : 0.0), "forced",
                          0, "operator-forced at current position");
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "fixed (forced)");
        logEvent("ok", "Force base complete - module confirmed in base mode");
    } else {
        logEvent("fail", "Force base FAILED - module still reports mode %d after "
                         "save+reset; coordinate NOT adopted", (int)g_probedRcvrMode);
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "force failed");
    }
}

static void handleApiReconfig() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // POST /api/reconfig[?full=1]
    // Default is a WARM re-survey: re-issue the survey targets only, leaving the
    // receiver tracking (no restart at all, so no loss of time/ephemeris and no cold
    // re-acquisition). full=1 is the heavyweight path that rewrites the entire module
    // configuration and resets it — needed only after changing the output message set
    // or recovering a module whose configuration is in doubt.
    if (server.hasArg("full") && server.arg("full") == "1") g_reconfigPending = true;
    else                                                    g_surveyRestartPending = true;
    server.send(200, "text/plain", "ok");
}

static void handleApiQuery() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // Lightweight (~40 ms): a few reads with short drains — safe inline.
    sendPqtm("PQTMCFGRCVRMODE,R");
    waitAndDrainGnss(20);
    sendPqtm("PQTMCFGSVIN,R");
    waitAndDrainGnss(20);
    sendPqtm("PQTMCFGPROT,R,1,1");
    waitAndDrainGnss(20);
    // Adds ~500 ms to this request (two paced reads). Acceptable: it is an explicit
    // operator action whose entire purpose is to ask the receiver what it is doing.
    queryPppSupport();
    server.send(200, "text/plain", "ok");
}

static void handleApiLocalCast() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // POST /api/localcast  body: on=0|1
    if (!server.hasArg("on")) { server.send(400, "text/plain", "missing on"); return; }
    g_localCasterEnabled = (server.arg("on") == "1");
    Preferences lp;
    if (lp.begin("rcx1id", false)) { lp.putBool("localcast", g_localCasterEnabled); lp.end(); }
    if (!g_localCasterEnabled) localCasterEnd();   // drop rovers immediately when disabled
    server.send(200, "text/plain", "ok");
}

static void handleApiSetName() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // POST /api/setname  body: name=...
    // Persists a new device name to NVS. Takes effect on the next reboot (the AP
    // SSID and status "dev" field are only re-resolved at boot).
    if (!server.hasArg("name")) { server.send(400, "text/plain", "missing name"); return; }
    const String name = server.arg("name");
    if (!setDeviceName(name)) {
        server.send(400, "text/plain", "invalid name (1-31 printable ASCII characters)");
        return;
    }
    server.send(200, "text/plain", "ok - reboot to apply");
}

static void handleApiAddWifi() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // POST /api/addwifi  body: ssid=...&pw=...&connect=0|1 (connect defaults to 1)
    // Adds a network to NVS and the live rotation. With connect=1 (default) it also
    // takes effect immediately: if we're not already on that network, it triggers a
    // connection attempt now. connect=0 just saves it for later without touching
    // whatever we're currently connected to.
    if (!server.hasArg("ssid")) { server.send(400, "text/plain", "missing ssid"); return; }
    const String ssid = server.arg("ssid");
    const String pw   = server.hasArg("pw") ? server.arg("pw") : String("");
    const bool doConnect = server.hasArg("connect") ? server.arg("connect").toInt() != 0 : true;
    if (ssid.length() == 0)  { server.send(400, "text/plain", "empty ssid"); return; }
    if (ssid.length() > 64)  { server.send(400, "text/plain", "ssid too long"); return; }
    if (!addWifiNetwork(ssid, pw)) {
        server.send(500, "text/plain", "NVS full or write failed");
        return;
    }
    // Immediately try this network if we're not already connected to it.
    if (doConnect &&
        (WiFi.status() != WL_CONNECTED || g_wifiNetworks[activeWifiIndex < 0 ? 0 : activeWifiIndex].ssid != ssid)) {
        // Find its index and try it.
        for (int i = 0; i < (int)g_wifiNetworks.size(); ++i) {
            if (g_wifiNetworks[i].ssid == ssid) { connectToWifiIndex(i); break; }
        }
    }
    server.send(200, "text/plain", "ok");
}

static void handleApiDelWifi() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // POST /api/delwifi  body: ssid=...
    // Removes a saved (NVS) network.
    if (!server.hasArg("ssid")) { server.send(400, "text/plain", "missing ssid"); return; }
    const String ssid = server.arg("ssid");
    if (!deleteWifiNetwork(ssid)) {
        server.send(404, "text/plain", "not found");
        return;
    }
    // If we just deleted the network we're connected to, cycle to the next one.
    if (WiFi.status() == WL_CONNECTED && activeWifiIndex >= 0
        && activeWifiIndex < (int)g_wifiNetworks.size() + 1) {
        // Reconnect from scratch since the active index may have shifted.
        connectToWifiIndex(0);
    }
    server.send(200, "text/plain", "ok");
}

static void handleApiWifiList() {
    // GET /api/wifilist — stored passwords are never included in the response.
    char buf[3072]; int n = 0;
    n += snprintf(buf + n, sizeof(buf) - n, "{\"connected\":%d,\"ssid_active\":\"%s\",\"networks\":[",
                  WiFi.status() == WL_CONNECTED ? 1 : 0,
                  (WiFi.status() == WL_CONNECTED && activeWifiIndex >= 0
                   && activeWifiIndex < (int)g_wifiNetworks.size())
                  ? g_wifiNetworks[activeWifiIndex].ssid.c_str() : "");
    for (int i = 0; i < (int)g_wifiNetworks.size(); ++i) {
        char ssidEsc[80];
        n += snprintf(buf + n, sizeof(buf) - n,
                      "%s{\"ssid\":\"%s\",\"active\":%d}",
                      i ? "," : "",
                      jsonSanitize(ssidEsc, sizeof(ssidEsc), g_wifiNetworks[i].ssid.c_str()),
                      (i == activeWifiIndex) ? 1 : 0);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]}");
    server.send(200, "application/json", buf);
}

static void handleApiCasterEnable() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // POST /api/casteren  body: host=..&mount=..&en=0|1
    if (!server.hasArg("host") || !server.hasArg("mount") || !server.hasArg("en")) {
        server.send(400, "text/plain", "need host, mount, en"); return;
    }
    const bool en = server.arg("en").toInt() != 0;
    if (setCasterEnabled(server.arg("host"), server.arg("mount"), en))
        server.send(200, "text/plain", "ok");
    else
        server.send(404, "text/plain", "caster not found");
}

// GET /api/mountcheck?host=<host>&port=<n>&mount=<NAME>
// Fetches the caster's NTRIP source table and reports whether that mount is currently
// being served. Answers {"ok":1,"inuse":0|1} or {"ok":0,"err":"..."} when the caster
// could not be reached.
//
// WHAT THIS CAN AND CANNOT TELL YOU (do not oversell it in the UI): the source table
// lists mounts that are LIVE RIGHT NOW. A mount that is registered to someone else but
// whose base happens to be offline does not appear, so "not in use" means "free at this
// instant", not "unclaimed". It catches the collision that matters — two bases fighting
// over one mount — and nothing more.
//
// The whole exchange is bounded and interleaved with dlPump() so the 460800-baud GNSS
// stream keeps draining; a caster that accepts the socket and then goes quiet cannot
// stall the loop past MOUNTCHECK_TIMEOUT_MS.
static constexpr uint32_t MOUNTCHECK_TIMEOUT_MS = 6000;

// Defined with the SD download helpers below; declared here rather than relying on the
// Arduino builder's auto-generated prototypes.
static void dlPump();

static void handleApiMountCheck() {
    if (WiFi.status() != WL_CONNECTED) {
        server.send(200, "application/json", "{\"ok\":0,\"err\":\"no internet connection\"}");
        return;
    }
    String host  = server.hasArg("host")  ? server.arg("host")  : String("crtk.net");
    String mount = server.hasArg("mount") ? server.arg("mount") : String("");
    const uint16_t port = server.hasArg("port") ? (uint16_t)server.arg("port").toInt() : 2101;
    if (mount.length() == 0) {
        server.send(200, "application/json", "{\"ok\":0,\"err\":\"no mount name given\"}");
        return;
    }

    WiFiClient c;
    c.setTimeout(3);
    if (!c.connect(host.c_str(), port, 4000)) {
        server.send(200, "application/json", "{\"ok\":0,\"err\":\"caster unreachable\"}");
        return;
    }
    c.setNoDelay(true);
    c.printf("GET / HTTP/1.0\r\nUser-Agent: NTRIP RCX1\r\nConnection: close\r\n\r\n");

    // Scan the table line by line rather than buffering it whole — a busy caster's
    // source table runs to tens of kilobytes and would not fit in RAM comfortably.
    const String needle = String("STR;") + mount + ";";
    String line;
    bool   inUse = false;
    const uint32_t deadline = millis() + MOUNTCHECK_TIMEOUT_MS;
    while (c.connected() && (int32_t)(millis() - deadline) < 0) {
        while (c.available() > 0) {
            const int ch = c.read();
            if (ch < 0) break;
            if (ch == '\n') {
                if (line.startsWith(needle)) { inUse = true; line = ""; break; }
                line = "";
            } else if (ch != '\r' && line.length() < 200) {
                line += (char)ch;
            }
        }
        if (inUse) break;
        dlPump();          // keep the GNSS/RTCM path alive while we wait on the socket
        delay(2);
    }
    c.stop();

    char out[64];
    snprintf(out, sizeof(out), "{\"ok\":1,\"inuse\":%d}", inUse ? 1 : 0);
    server.send(200, "application/json", out);
}

static void handleApiCasterAdd() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // POST /api/casteradd  body: host=..&port=..&mount=..&pw=..
    if (!server.hasArg("host") || !server.hasArg("mount")) {
        server.send(400, "text/plain", "need host, mount"); return;
    }
    const String host  = server.arg("host");
    const String mount = server.arg("mount");
    const String pw    = server.hasArg("pw") ? server.arg("pw") : String("");
    const uint16_t port = server.hasArg("port") ? (uint16_t)server.arg("port").toInt() : 2101;
    if (port == 0) { server.send(400, "text/plain", "bad port"); return; }
    if (casterCount >= MAX_CASTERS) { server.send(507, "text/plain", "max casters reached"); return; }
    if (addCaster(host, port, mount, pw)) server.send(200, "text/plain", "ok");
    else server.send(409, "text/plain", "duplicate or invalid");
}

static void handleApiCasterDel() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // POST /api/casterdel  body: host=..&mount=..
    if (!server.hasArg("host") || !server.hasArg("mount")) {
        server.send(400, "text/plain", "need host, mount"); return;
    }
    if (deleteCaster(server.arg("host"), server.arg("mount")))
        server.send(200, "text/plain", "ok");
    else
        server.send(400, "text/plain", "not found");
}

static void handleApiClearPos() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // POST /api/clearpos — erase the saved position AND start a fresh survey now.
    //
    // Erasing NVS alone was not enough, and that is why this appeared dead: the MODULE
    // was still a fixed base (svinMode 2, persisted by PQTMSAVEPAR), so it kept
    // publishing the old coordinate until a reboot — and on reboot the hot-start probe
    // found a validly configured fixed base and accepted it, skipping the survey again.
    // From the dashboard there was no way out of a saved coordinate at all.
    clearSavedPosition();
    g_useNvsSavedPosition   = false;
    g_displacementCheckDone = true;   // nothing left to check against
    g_posCheckState         = PosCheckState::Idle;
    g_posCheckReason[0]     = '\0';
    g_surveyRestartPending  = true;   // deferred to loop() → restartSurveyWarm()
    Serial.println("🗑️  Saved position cleared — starting a fresh survey");
    server.send(200, "text/plain", "cleared, starting new survey");
}

static void handleApiSetPos() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    // POST /api/setpos  body: lat=..&lon=..&alt=..  — manual known-coordinate entry.
    // Saves to the rcx1pos NVS slot, then applies it as a fixed base immediately.
    // Intended for a surveyed/post-processed reference (e.g. CSRS-PPP, OPUS) you trust.
    if (!server.hasArg("lat") || !server.hasArg("lon")) {
        server.send(400, "text/plain", "need lat and lon");
        return;
    }
    const double lat = server.arg("lat").toDouble();
    const double lon = server.arg("lon").toDouble();
    const double alt = server.hasArg("alt") ? server.arg("alt").toDouble() : 0.0;
    // Range validation — reject obvious garbage / null island.
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 ||
        (lat == 0.0 && lon == 0.0) || alt < -1000.0 || alt > 9000.0) {
        server.send(400, "text/plain", "lat/lon/alt out of range");
        return;
    }
    // Gross-typo failsafe: if we currently have a rough fix and the entered point is
    // absurdly far from it (>100 km), refuse — almost certainly a fat-finger. With no
    // fix yet, trust the operator (they may be pre-loading before acquisition).
    if (ggaFixQuality > 0 && (ggaLat != 0.0 || ggaLon != 0.0)) {
        const double distKm = roughDistanceM(ggaLat, ggaLon, lat, lon) / 1000.0;
        if (distKm > 100.0) {
            char m[96];
            snprintf(m, sizeof(m), "entry is %.0f km from current fix — refusing (typo?)", distKm);
            server.send(409, "text/plain", m);
            return;
        }
    }
    if (!savePositionToNvs(lat, lon, alt, 0.0f, "manual")) {
        server.send(500, "text/plain", "NVS write failed");
        return;
    }
    g_useNvsSavedPosition  = true;
    g_displacementCheckDone = true;   // operator asserted position — don't second-guess
    g_reconfigPending      = true;    // apply as fixed base now (full reconfigure in loop)
    server.send(200, "text/plain", "ok");
}

// ── SD downloads (stability-critical) ────────────────────────────────────────
// Every byte read from the card is interleaved with dlPump() so the 460800-baud
// GNSS UART stays drained and RTCM keeps flowing to the casters DURING a download.
// The log task is paused (bridge_sdlog_set_download_active) so there is exactly
// one reader. See bridge_sd_log.cpp for the pause/flush handshake.
static void dlPump() { processGnssSerial(); yield(); }

static const char* dlBaseName(const char* nm) { const char* s = strrchr(nm, '/'); return s ? s + 1 : nm; }
static bool dlIsLog(const char* nm) {
    return !strncmp(nm, "gps_", 4) || !strncmp(nm, "sat_", 4) ||
           !strncmp(nm, "status_", 7) || !strncmp(nm, "rtcm_", 5);
}
static long dlSession(const char* nm) { const char* u = strchr(nm, '_'); return u ? atol(u + 1) : -1; }

enum DlSet { DL_CUR = 0, DL_RECENT = 1, DL_ALL = 2 };
static bool dlInSet(const char* nm, int set, uint16_t cur) {
    if (!dlIsLog(nm)) return false;
    long s = dlSession(nm);
    if (s < 0) return false;
    if (set == DL_ALL) return true;
    if (set == DL_CUR) return (uint16_t)s == cur;
    return (long)cur - s < 12;   // recent = last 12 sessions
}

static char     g_dlNames[128][40];
static uint32_t g_dlSizes[128];
static int      g_dlCount = 0;
static void dlCollect(int set) {
    g_dlCount = 0;
    File root = SD_MMC.open("/");
    if (!root) return;
    const uint16_t cur = bridge_sdlog_session();
    int scanned = 0;
    for (File f = root.openNextFile(); f && g_dlCount < 128; f = root.openNextFile()) {
        // GRACEFUL DEGRADATION (keep): this runs inside a web handler, i.e. inside
        // loop(). An unyielded walk freezes the GNSS drain for however long the root
        // directory takes — which scales with how many sessions are on the card. Pump
        // the UART every 16 entries so cast-stream health never depends on file count.
        if ((++scanned & 0x0F) == 0) dlPump();
        if (f.isDirectory()) continue;
        const char* nm = dlBaseName(f.name());
        if (dlInSet(nm, set, cur)) {
            strncpy(g_dlNames[g_dlCount], nm, 39);
            g_dlNames[g_dlCount][39] = '\0';
            g_dlSizes[g_dlCount] = f.size();
            g_dlCount++;
        }
    }
    root.close();
}

static void dlOctal(char* p, int len, uint32_t v) {   // len-1 octal digits + null
    for (int i = len - 2; i >= 0; --i) { p[i] = '0' + (v & 7); v >>= 3; }
    p[len - 1] = '\0';
}
static void dlTarHeader(uint8_t* h, const char* name, uint32_t size) {
    memset(h, 0, 512);
    strncpy((char*)h, name, 99);
    memcpy(h + 100, "0000644", 7);
    memcpy(h + 108, "0000000", 7);
    memcpy(h + 116, "0000000", 7);
    dlOctal((char*)(h + 124), 12, size);
    dlOctal((char*)(h + 136), 12, 0);
    memset(h + 148, ' ', 8);          // checksum field = spaces during calc
    h[156] = '0';                     // typeflag: normal file
    memcpy(h + 257, "ustar", 5);
    h[263] = '0'; h[264] = '0';
    uint32_t sum = 0; for (int i = 0; i < 512; ++i) sum += h[i];
    char cs[8]; dlOctal(cs, 7, sum);  // 6 octal digits + null
    memcpy(h + 148, cs, 7); h[155] = ' ';
}

static void dlStreamFileBytes(WiFiClient& client, File& f, uint32_t declared) {
    uint8_t chunk[1460];
    uint32_t remaining = declared;
    while (remaining > 0 && client.connected()) {
        const size_t want = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
        const int r = f.read(chunk, want);
        if (r <= 0) break;
        client.write(chunk, r);
        remaining -= r;
        dlPump();                      // keep GNSS/RTCM alive between chunks
    }
    if (remaining > 0) {               // file shorter than declared: pad to keep Content-Length exact
        uint8_t z[256] = {0};
        while (remaining > 0) { size_t w = remaining < sizeof(z) ? remaining : sizeof(z); client.write(z, w); remaining -= w; }
    }
}

static void handleDownload() {
    if (!bridge_sdlog_ready()) { server.send(503, "text/plain", "SD not ready"); return; }
    bridge_sdlog_set_download_active(true);
    for (int i = 0; i < 5; ++i) { dlPump(); delay(10); }   // let the log task flush + settle

    if (server.hasArg("f")) {
        String fn = server.arg("f");
        if (fn.indexOf('/') >= 0 || !dlIsLog(fn.c_str())) { server.send(400, "text/plain", "bad file"); bridge_sdlog_set_download_active(false); return; }
        char path[48]; snprintf(path, sizeof(path), "/%s", fn.c_str());
        File f = SD_MMC.open(path, FILE_READ);
        if (!f) { server.send(404, "text/plain", "not found"); bridge_sdlog_set_download_active(false); return; }
        const uint32_t size = f.size();
        server.sendHeader("Content-Disposition", String("attachment; filename=\"") + fn + "\"");
        server.sendHeader("Cache-Control", "no-store");
        server.setContentLength(size);
        server.send(200, "application/octet-stream", "");
        WiFiClient client = server.client();
        dlStreamFileBytes(client, f, size);
        f.close();
    } else {
        String set = server.hasArg("set") ? server.arg("set") : String("current");
        const int s = set == "all" ? DL_ALL : (set == "recent" ? DL_RECENT : DL_CUR);
        dlCollect(s);
        uint64_t total = 0;
        for (int i = 0; i < g_dlCount; ++i) total += 512 + (((uint64_t)g_dlSizes[i] + 511) / 512) * 512;
        total += 1024;
        char dlname[48]; snprintf(dlname, sizeof(dlname), "rcx1_%s_%04u.tar", set.c_str(), bridge_sdlog_session());
        server.sendHeader("Content-Disposition", String("attachment; filename=\"") + dlname + "\"");
        server.sendHeader("Cache-Control", "no-store");
        server.setContentLength(total);
        server.send(200, "application/x-tar", "");
        WiFiClient client = server.client();
        uint8_t hdr[512];
        for (int i = 0; i < g_dlCount && client.connected(); ++i) {
            char path[48]; snprintf(path, sizeof(path), "/%s", g_dlNames[i]);
            File f = SD_MMC.open(path, FILE_READ);
            if (!f) continue;
            dlTarHeader(hdr, g_dlNames[i], g_dlSizes[i]);
            client.write(hdr, 512); dlPump();
            dlStreamFileBytes(client, f, g_dlSizes[i]);
            const uint32_t pad = (512 - (g_dlSizes[i] & 511)) & 511;
            if (pad) { uint8_t z[512] = {0}; client.write(z, pad); }
            f.close();
        }
        uint8_t z[1024] = {0}; client.write(z, 1024);   // two zero blocks = end of archive
    }
    bridge_sdlog_set_download_active(false);
}

// CACHED — do not make this rescan on every request. Building the list walks the whole
// root directory, and the walk runs with the Core-0 SD log task PAUSED (the pause is
// what keeps a concurrent reader off the shared bus and FAT). The card accumulates 4-5
// files per session, so after a few hundred sessions the walk takes seconds, and the
// dashboard was requesting it every 30 s. Each request therefore stalled logging for
// seconds at a time; at ~30 rows/s of production the queue overflowed and rows were
// silently dropped — heaviest on the sat channel, which is most of that rate.
//
// The listing changes only when a file is created, rotated or deleted, so it is served
// from a snapshot and only rebuilt on an explicit ?refresh=1 or after LOGS_CACHE_TTL_MS.
static constexpr uint32_t LOGS_CACHE_TTL_MS = 300000;   // 5 minutes
static String   g_logsCacheJson;
static uint32_t g_logsCacheMs    = 0;
static int      g_logsCacheScope = -1;

static void invalidateLogsCache() { g_logsCacheScope = -1; }

static void handleLogsJson() {
    if (!bridge_sdlog_ready()) { server.send(200, "application/json", "{\"files\":[],\"scope\":\"current\"}"); return; }
    String scope = server.hasArg("scope") ? server.arg("scope") : "current";
    const int set = scope == "all" ? DL_ALL : (scope == "recent" ? DL_RECENT : DL_CUR);
    const bool forced = server.hasArg("refresh") && server.arg("refresh") == "1";
    if (!forced && g_logsCacheScope == set && g_logsCacheJson.length() &&
        (uint32_t)(millis() - g_logsCacheMs) < LOGS_CACHE_TTL_MS) {
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "application/json", g_logsCacheJson);
        return;
    }
    bridge_sdlog_set_download_active(true);
    for (int i = 0; i < 3; ++i) { dlPump(); delay(5); }

    dlCollect(set);   // CUR = current session; RECENT = last 12 sessions; ALL = everything
    // Order newest-first by session, then cap "recent" to the 12 most-recent FILES.
    int order[128];
    for (int i = 0; i < g_dlCount; ++i) order[i] = i;
    for (int i = 1; i < g_dlCount; ++i) {           // insertion sort, session desc (small N)
        const int k = order[i];
        const long ks = dlSession(g_dlNames[k]);
        int j = i - 1;
        while (j >= 0 && dlSession(g_dlNames[order[j]]) < ks) { order[j + 1] = order[j]; --j; }
        order[j + 1] = k;
    }
    const int limit = (set == DL_RECENT && g_dlCount > 12) ? 12 : g_dlCount;

    String out = "{\"scope\":\"" + scope + "\",\"total\":" + String(g_dlCount) + ",\"files\":[";
    bool first = true;
    for (int idx = 0; idx < limit; ++idx) {
        const int i = order[idx];
        out += String(first ? "" : ",") + "{\"n\":\"" + g_dlNames[i] + "\",\"kb\":" +
               String((uint32_t)((g_dlSizes[i] + 1023) / 1024)) + "}";
        first = false;
        if (out.length() > 6000) break;
    }
    out += "]}";
    bridge_sdlog_set_download_active(false);
    g_logsCacheJson  = out;
    g_logsCacheMs    = millis();
    g_logsCacheScope = set;
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", out);
}

// POST /api/logdel  body: f=<filename>  OR  session=<NNNN>
// Deletes one log file, or every log file belonging to a session, from the SD card.
// Files for the ACTIVE session are never eligible — the logger only ever holds an open
// write handle for the current session (see openTagFile()/openRawFile(), both keyed on
// g_session), so anything from an older session is guaranteed closed.
//
// SINGLE READER (do not remove): closed files still share one SD_MMC bus and one FAT with
// the Core-0 log task. Directory-walking and unlinking underneath a task that is
// allocating clusters for the current session risks corrupting the very logs we are
// keeping. Pause the log task for the duration exactly as the download path does, and
// release it on EVERY exit — hence the single exit point below.
static void handleApiLogDelete() {
    markWifiActivity();   // deliberate dashboard action — see g_wifiActivityMs
    if (!bridge_sdlog_ready()) { server.send(503, "text/plain", "SD not ready"); return; }
    const uint16_t cur = bridge_sdlog_session();
    int deletedCount = 0;
    bridge_sdlog_set_download_active(true);
    for (int i = 0; i < 3; ++i) { dlPump(); delay(5); }   // let the task flush and settle

    int         code = 200;
    const char* err  = nullptr;

    if (server.hasArg("f")) {
        String fn = server.arg("f");
        const long sess = (fn.indexOf('/') >= 0 || !dlIsLog(fn.c_str())) ? -1 : dlSession(fn.c_str());
        if (sess < 0)                    { code = 400; err = "bad file"; }
        else if ((uint16_t)sess == cur)  { code = 400; err = "cannot delete the active session's log"; }
        else {
            char path[48]; snprintf(path, sizeof(path), "/%s", fn.c_str());
            if (SD_MMC.remove(path)) deletedCount = 1;
            else                     { code = 404; err = "not found"; }
        }
    } else if (server.hasArg("session")) {
        const long sess = server.arg("session").toInt();
        if (sess < 0)                    { code = 400; err = "bad session"; }
        else if ((uint16_t)sess == cur)  { code = 400; err = "cannot delete the active session's log"; }
        else {
            dlCollect(DL_ALL);
            for (int i = 0; i < g_dlCount; ++i) {
                if (dlSession(g_dlNames[i]) != sess) continue;
                char path[48]; snprintf(path, sizeof(path), "/%s", g_dlNames[i]);
                if (SD_MMC.remove(path)) deletedCount++;
                if ((i & 0x0F) == 0) dlPump();   // keep the GNSS drain alive on a big sweep
            }
            if (deletedCount == 0) { code = 404; err = "session not found"; }
        }
    } else {
        code = 400; err = "need f= or session=";
    }

    bridge_sdlog_set_download_active(false);   // SINGLE EXIT — always released
    invalidateLogsCache();                     // the listing just changed
    if (err) { server.send(code, "text/plain", err); return; }
    char resp[24]; snprintf(resp, sizeof(resp), "deleted %d", deletedCount);
    server.send(200, "text/plain", resp);
}

static void handleRoot() {
    // STABILITY: static asset straight from flash — zero per-request heap churn.
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html", BRIDGE_INDEX_HTML);
}

// ── Display ──────────────────────────────────────────────────────────────────
static void initDisplay() {
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(UI_BG);
    tft.setTextWrap(false);
    // Backlight LAST, after tft.init() — attaching LEDC before tft.init() let the
    // display init leave IO46 uncontrolled (the stuck-on-backlight bug). Active-HIGH,
    // normal duty. DO NOT move these two lines above tft.init(). See LCD BACKLIGHT block.
    ledcAttach(PIN_LCD_BL, 5000, 8);
    ledcWrite(PIN_LCD_BL, LCD_BL_FULL_DUTY);
    // The status LED comes up with the panel, and comes up DARK: the overall status is not
    // meaningful until the receiver has been probed and WiFi has been tried, and a green
    // LED during boot would be a claim the firmware is in no position to make yet. This is
    // also the write that clears an undriven LED's power-up state, which is usually white.
    // rgbLedWrite() configures the pin on first use, so there is nothing else to set up.
    rgbLedWriteOrdered(LED_PIN, LED_ORDER, LED_DARK.r, LED_DARK.g, LED_DARK.b);
}

// Overall status colour for the RCX1 title:
//   GREEN  = corrections are reaching at least one caster (streaming end-to-end)
//   RED    = a problem: no WiFi, no GNSS fix, or a caster has errored
//   YELLOW = data is good but we're still waiting on survey-in / caster connect
// Overall unit health, shown in the title and its divider. Computed once per redraw —
// it walks the caster list and queries the WiFi driver, so it is not free.
static UiStatus statusState() {
    bool anyStreaming = false, anyError = false;
    for (int i = 0; i < casterCount; ++i) {
        if (!casters[i].enabled) continue;           // a disabled caster isn't a problem
        if (casters[i].state == CasterState::Streaming) anyStreaming = true;
        if (casters[i].state == CasterState::Error)     anyError = true;
    }
    if (anyStreaming) return ST_OK;
    const bool wifiUp    = (WiFi.status() == WL_CONNECTED);
    const bool haveFix   = (ggaFixQuality > 0);
    if (!wifiUp || !haveFix || anyError) return ST_BAD;
    return ST_WARN;
}

static uint16_t statusColor() {
    const UiStatus s = statusState();
    return (s == ST_OK) ? UI_OK : (s == ST_BAD) ? UI_BAD : UI_WARN;
}

// Short label for the GGA fix-quality indicator.
static const char* fixQualityLabel(int q) {
    switch (q) {
        case 0: return "No Fix";
        case 1: return "3D";
        case 2: return "DGPS";
        case 4: return "RTK Fix";
        case 5: return "RTK Flt";
        case 6: return "DeadRck";
        case 7: return "Base";
        default: return "?";
    }
}

// Compact caster state for a single-line readout.
static const char* casterShort(CasterState s) {
    switch (s) {
        case CasterState::Streaming:        return "Stream";
        case CasterState::Authenticated:    return "Auth";
        case CasterState::AwaitingResponse: return "Wait";
        case CasterState::Connecting:       return "Conn";
        case CasterState::WaitingForWifi:   return "NoWiFi";
        case CasterState::Error:            return "Err";
        case CasterState::Disabled:         return "Off";
        case CasterState::Held:             return "Held";
        default:                            return "...";
    }
}

// Frame counter shortened to fit (<=6 chars): 14211, 124.5k, 3.2M.
static String compactCount(uint64_t v) {
    if (v < 100000ULL)   return String((unsigned long)v);
    if (v < 10000000ULL) return String((double)v / 1000.0, 1) + "k";
    return String((double)v / 1000000.0, 1) + "M";
}

// Paint a minimal boot screen IMMEDIATELY, before the blocking GNSS probe in setup().
// The probe waits up to 60 s for the module's PQTMCFGRCVRMODE,R / PQTMCFGSVIN,R replies;
// a cold module answers slowly, so without this the panel sits black (backlight on, no
// content) for up to a minute and looks dead. This guarantees visible feedback within a
// second of power, and updateDisplay() takes over with live data once the probe returns.
// Boot progress. Safe to call as often as you like: the screen is cleared only on the
// FIRST call, after which just the two text lines are repainted over a padded opaque
// background — no full-screen flicker, and no cost beyond two short string draws.
// Purely a rendering helper; it changes no state and no order of operations.
static void drawBootSplash(const char* line1, const char* line2) {
    static bool splashDrawn = false;
    if (!splashDrawn) {
        tft.fillScreen(UI_BG);
        tft.setTextColor(UI_OK, UI_BG);
        tft.setTextDatum(TC_DATUM);
        tft.drawString("RCX RTK Base", 172 / 2, 90);
        tft.setTextDatum(TL_DATUM);
        splashDrawn = true;
    }
    tft.setTextColor(UI_LABEL, UI_BG);
    tft.setCursor(6, 130); tft.print(padRight(line1 ? line1 : "", 26));
    tft.setTextColor(UI_FG, UI_BG);
    tft.setCursor(6, 150); tft.print(padRight(line2 ? line2 : "", 26));
    g_displayNeedsRedraw = true;   // force updateDisplay() to repaint the title cleanly later
}

static void updateDisplay() {
    const uint32_t now = millis();
    if (now - lastLcdUpdateMs < 1000) return;
    lastLcdUpdateMs = now;
    int y = 4;

    // ── Status banner: whole band filled with the overall base condition ─────
    // ANTI-FLICKER: neither the text nor the fill changes between states of the same
    // colour, so the band is repainted ONLY when the condition changes. Repainting a
    // filled rectangle every second would visibly strobe.
    const UiStatus overallSt = statusState();
    {
        static uint16_t lastBannerBg = 0x0001;         // impossible initial → forces first draw
        static bool bannerDrawn = false;
        const uint16_t bg = (overallSt == ST_OK)  ? UI_BANNER_OK
                          : (overallSt == ST_BAD) ? UI_BANNER_BAD : UI_BANNER_WARN;
        const uint16_t fg = (overallSt == ST_WARN) ? UI_ON_LIGHT : UI_ON_DARK;
        if (g_displayNeedsRedraw || !bannerDrawn || bg != lastBannerBg) {
            tft.setTextFont(4);
            // Fill from the very top edge so the band reads as a header bar rather than
            // a floating swatch; the 4 px above y would otherwise stay white.
            tft.fillRect(0, 0, 172, y + 28, bg);
            setInk(fg, bg);
            tft.setTextDatum(TC_DATUM);                // top-center
            // Font 4 fits ~13 characters across 172 px, so this is a fixed label rather
            // than the full device name ("RCX RTK Base RDxx").
            bdrawString("RCX RTK Base", 172 / 2, y + 2);
            tft.setTextDatum(TL_DATUM);                // restore top-left for the rest
            lastBannerBg = bg;
            bannerDrawn = true;
        }
    }
    g_displayNeedsRedraw = false;                      // consumed for this pass
    y += 28;
    // No rule under the banner — the filled band already separates it from the fields.
    y += 6;
    tft.setTextFont(2);

    // ── Survey: accuracy + countdown(survey-in)/FIXED, then lat/lon ──────────
    String svStatus;
    if (svinValidNow() == 2 || LG290P_USE_FIXED_BASE ||
        (g_useNvsSavedPosition && g_savedPositionValid)) svStatus = "FIXED";
    else if (surveyInSkipped) svStatus = "INSTANT";
    // The panel a person looks at from across a paddock must not read FIXED while the
    // antenna is off. svinValidNow() already withholds the convergence claim; this names
    // the reason rather than falling through to a countdown that is not counting.
    else if (!receiverHasSignal()) svStatus = "NO SIGNAL";
    else if (surveyAwaitingFirstFix) svStatus = "WAIT FIX";
    else {
        const int32_t rem = surveyRemainingSeconds();
        svStatus = rem > 0 ? ("T-" + formatDuration((uint32_t)rem)) : String("CONV");
    }
    tft.setCursor(0, y);
    // Section state drives the labels and the closing divider, so the whole band agrees.
    // REALISTIC accuracy (scatter-floored, see realisticAccuracyM()) — shown both during
    // survey-in and, unlike a meanAcc-only readout, after FIXED too (meanAcc zeroes on
    // completion; this falls back to the achieved/saved accuracy instead of "--").
    // Computed here because the section's status, its labels and its divider all depend
    // on it, and those are drawn before the value itself.
    const double lcdAcc = realisticAccuracyM();
    // SCALE THE STOPLIGHT TO THE SURVEY THAT IS ACTUALLY RUNNING. surveyAccLimit is the
    // MODULE's internal survey-in gate — the figure its own survey pauses and restarts
    // against — and it has no authority over a PPP survey, which is judged against its own
    // EPE and scatter criteria on the /ppp page. Left unscaled, the band read green for any
    // accuracy under 3.2 m, which covers the entire range in which a PPP survey is FAILING:
    // a receiver pinned at the 1.300 m EPE floor, never converging, showed the same green as
    // a converged decimetre solution. Judging against the live PPP limits makes green mean
    // PPP-grade and red mean not converged, which is the question being asked of the number.
    double accGreen = surveyAccLimit, accAmber = surveyAccLimit * 3.0;
    if (BASE_SURVEY_USE_PPP) {
        float lim2d = 0.0f, lim3d = 0.0f; uint32_t minS = 0, durS = 0;
        ppp_survey_get_criteria(lim2d, lim3d, minS, durS);
        if (lim2d > 0.0f) { accGreen = lim2d; accAmber = lim2d * 3.0; }
    }
    const UiStatus accSt = (lcdAcc < 0.0) ? ST_MUTED
                         : stoplight(lcdAcc, accGreen, accAmber, false);
    const UiStatus svinSt = (svStatus == "FIXED") ? ST_OK : ST_WARN;
    const bool     havePos = (ggaLat != 0.0 || ggaLon != 0.0);
    const UiStatus posSt   = havePos ? ST_OK : ST_MUTED;
    const UiStatus surveySt = worst(worst(accSt, svinSt), posSt);

    setLabel(surveySt);
    bprint("acc ");
    setStatus(accSt);
    bprint(padRight(lcdAcc >= 0.0 ? String(lcdAcc, 2) + "m" : String("--"), 8));
    setStatus(svinSt);
    bprint(padRight(svStatus, 10));
    y += 18;
    tft.setCursor(0, y);
    setLabel(surveySt);
    bprint("Lat ");
    setStatus(posSt);
    bprint(padRight(String(ggaLat, 7), 22));
    y += 18;
    tft.setCursor(0, y);
    setLabel(surveySt);
    bprint("Lon ");
    setStatus(posSt);
    bprint(padRight(String(ggaLon, 7), 22));
    y += 20;
    tft.drawLine(0, y, 172, y, dividerColor(surveySt));
    y += 6;

    // ── Quality / lock ───────────────────────────────────────────────────────
    tft.setCursor(0, y);
    const UiStatus fixSt  = (ggaFixQuality == 4 || ggaFixQuality == 7) ? ST_OK
                          : (ggaFixQuality == 0)                       ? ST_BAD
                                                                       : ST_WARN;
    const UiStatus satSt  = stoplight(ggaSatellites, 12, 7, true);
    const UiStatus epeSt  = stoplight(liveErrorMeters, 1.0, 3.0, false);
    const UiStatus hdopSt = stoplight(ggaHdop, 1.0, 2.0, false);
    const UiStatus qualSt = worst(worst(fixSt, satSt), worst(epeSt, hdopSt));

    setLabel(qualSt);
    bprint("Fix ");
    setStatus(fixSt);
    bprint(padRight(String(fixQualityLabel(ggaFixQuality)), 9));
    setLabel(qualSt);
    bprint("Sat ");
    setStatus(satSt);
    bprint(padRight(String(ggaSatellites), 4));
    y += 18;
    tft.setCursor(0, y);
    setLabel(qualSt);
    bprint("EPE ");
    setStatus(epeSt);
    bprint(padRight(liveErrorMeters > 0.0 ? String(liveErrorMeters, 2) + "m" : String("--"), 8));
    setLabel(qualSt);
    bprint("HDOP ");
    setStatus(hdopSt);
    bprint(padRight(ggaHdop > 0.0 ? String(ggaHdop, 1) : String("--"), 5));
    y += 20;
    tft.drawLine(0, y, 172, y, dividerColor(qualSt));
    y += 6;

    // ── Which corrections are flowing, DECODED (number + readable tag) ───────
    // Two columns of "NNNN tag", MSM observations first (green), the rest in cyan.
    // ANTI-FLICKER: the old code wiped this whole lower region with a fillRect every
    // second, which made the screen visibly blink. Instead we use a FIXED layout
    // (fixed row count, so casters/footer never move) and draw each cell padded to a
    // fixed width with an OPAQUE background — each cell overwrites its own prior glyphs
    // in place. Only cells past the current list length get blanked, so nothing flashes.
    const int corrTop = y;
    // FIXED height: 3 rows x 2 cols = 6 cells. Sized to the four MSM observation types
    // plus two more (1005/1006 station coords, 1033 antenna descriptor); sporadic
    // ephemeris types beyond that are omitted here and remain on the dashboard's full
    // type table. Three rows rather than four is what keeps the temperature readout on
    // screen alongside the 3-row caster block below.
    const int corrRows = 3;
    const int maxCells = corrRows * 2;
    // Collect the currently-flowing types into an ordered list (MSM first).
    uint16_t cellTypes[6]; int nCells = 0;
    for (int pass = 0; pass < 2 && nCells < maxCells; ++pass) {
        for (const auto& st : rtcmStats.typeStats) {
            if (st.type == 0 || st.lastSeenMs == 0) continue;
            if ((now - st.lastSeenMs) > 5000) continue;          // only currently flowing
            const bool msm = isMsmObservation(st.type);
            if ((pass == 0) != msm) continue;                    // MSM observations first
            if (nCells >= maxCells) break;
            cellTypes[nCells++] = st.type;
        }
    }
    for (int i = 0; i < maxCells; ++i) {
        const int col = i % 2, row = i / 2;
        const int cx = col ? 88 : 0;
        const int cyp = corrTop + row * 18;
        if (i == 0 && nCells == 0) {
            tft.setCursor(cx, cyp);
            setStatus(ST_BAD);
            bprint(padRight("no corrections", 14));
        } else if (i < nCells) {
            const uint16_t t = cellTypes[i];
            const bool msm = isMsmObservation(t);
            // Compose "NNNN tag" padded to a fixed 13-char field so it overwrites cleanly.
            String s = String(t) + " " + rtcmTag(t);
            tft.setCursor(cx, cyp);
            // MSM observation types are the payload a rover actually needs; the rest
            // (1005/1033 and friends) are supporting metadata, healthy but not the point.
            setStatus(msm ? ST_OK : ST_MUTED);
            bprint(padRight(s, 13));
        } else if (!(i == 0 && nCells == 0)) {
            // Blank a now-unused cell (list shrank) without touching the rest.
            tft.fillRect(cx, cyp, col ? (172 - 88) : 88, 16, UI_BG);
        }
    }
    y = corrTop + corrRows * 18;   // trimmed from +4 — pure buffer (row heights already
                                    // counted above), reclaimed for the fixed caster block
    tft.drawLine(0, y, 172, y, dividerColor(nCells == 0 ? ST_BAD : ST_OK));
    y += 6;

    // ── Casters: one compact line each (FIXED 3-row block, opaque padded fields) ──
    // FIXED HEIGHT (do not make this variable again): always exactly CASTER_LCD_ROWS
    // rows, in BOTH the sta and !sta branches below, so the footer (SSID/IP/temp)
    // never shifts — regardless of caster count, and regardless of whether WiFi is up.
    // The web UI shows the full list; the LCD is a summary. Unused rows are blanked
    // (not skipped) so nothing flashes, same pattern as the corrections block above.
    // No STA uplink → every caster would just read "No WiFi" (we can't cast without
    // internet), which tells you nothing you don't already know. Use this same space
    // for what actually matters out in the field: the AP SSID/IP to reach the
    // dashboard directly.
    constexpr int CASTER_LCD_ROWS = 3;
    const bool sta = (WiFi.status() == WL_CONNECTED);
    if (!sta) {
        // No uplink means no casting, so this space carries the one thing that IS
        // actionable: how to reach the dashboard and configure the unit. Three lines,
        // exactly filling the fixed caster block so nothing below shifts.
        tft.setCursor(0, y);
        // No uplink is a WARN, not a fault — the base still surveys and logs, and the
        // local caster still serves rovers on this AP. The address lines are the remedy,
        // so they stay plain and maximally legible rather than being alarm-coloured.
        setStatus(ST_WARN);
        bprint(padRight("To configure, join AP", 24));
        y += 18;
        tft.setCursor(0, y);
        setInk(UI_FG, UI_BG);
        bprint(padRight(g_deviceName, 24));
        y += 18;
        tft.setCursor(0, y);
        setInk(UI_FG, UI_BG);
        bprint(padRight("http://" + WiFi.softAPIP().toString(), 24));
        y += 18;
        for (int i = 3; i < CASTER_LCD_ROWS; ++i) { tft.fillRect(0, y, 172, 16, UI_BG); y += 18; }
    } else {
        // Active (enabled) casters first, then disabled — each group in its original
        // configured order — so a caster actually streaming is never bumped off the
        // fixed 3-row LCD block by one that's merely configured but switched off.
        int order[MAX_CASTERS]; int nOrder = 0;
        for (int i = 0; i < casterCount; ++i) if (casters[i].enabled)  order[nOrder++] = i;
        for (int i = 0; i < casterCount; ++i) if (!casters[i].enabled) order[nOrder++] = i;

        // The local caster takes the FIRST row when enabled — it is the one target that
        // works with no internet at all, so it is the most useful thing to see in the
        // field. It occupies a row of the fixed block like any upstream caster.
        const bool showLocal = g_localCasterEnabled;
        const int  localRows = showLocal ? 1 : 0;
        if (showLocal) {
            const int clients = localCasterActiveCount();
            tft.setCursor(0, y);
            const UiStatus lSt = (clients > 0) ? ST_OK : ST_WARN;
            setLabel(lSt);
            bprint(padRight(LOCAL_CASTER_NAME, 11));
            setStatus(lSt);
            bprint(padRight(clients > 0 ? "SERVING" : "ready", 7));
            setStatus(lSt);
            bprint(padRight(String(clients) + " rvr", 6));
            y += 18;
        }
        const int slots = CASTER_LCD_ROWS - localRows;
        const int shown = (nOrder < slots) ? nOrder : slots;
        for (int row = 0; row < slots; ++row) {
            if (row >= shown) { tft.fillRect(0, y, 172, 16, UI_BG); y += 18; continue; }
            const NtripTarget& c = casters[order[row]];
            const UiStatus cSt = c.enabled ? stateToStatus(c.state) : ST_MUTED;
            tft.setCursor(0, y);
            setLabel(cSt);
            bprint(padRight(String(c.host), 11));
            if (!c.enabled) {
                setStatus(ST_MUTED);
                bprint(padRight("off", 7));
                bprint(padRight("", 6));
            } else {
                setStatus(cSt);
                bprint(padRight(String(casterShort(c.state)), 7));
                // Frame count shares the row's state: a climbing count under a green
                // block is the confirmation that corrections are actually moving.
                setStatus(cSt);
                bprint(padRight(compactCount(c.framesWritten), 6));
            }
            y += 18;
        }
    }

    // ── Footer: SSID line, then the IP on its OWN line — IP is NEVER truncated ──
    // The IP is the one thing you must always be able to read to reach the dashboard,
    // so it gets a dedicated line at full width with no trimming. The SSID/AP name
    // sits above it and may be shortened to fit, since it's far less critical.
    // When offline (!sta) the AP name and URL are already spelled out above in the
    // caster block, so this line carries the AP IP once more at guaranteed full width.
    String ip   = sta ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    const int wIdx = (activeWifiIndex >= 0) ? activeWifiIndex : wifiAttemptIndex;
    String name = sta && wIdx < (int)g_wifiNetworks.size()
                  ? g_wifiNetworks[wIdx].ssid
                  : g_deviceName;
    if (name.length() > 28) name = name.substring(0, 28);   // SSID may trim; IP never does
    tft.setCursor(0, y);
    const UiStatus netSt = sta ? ST_OK : ST_WARN;
    setStatus(netSt);
    bprint(padRight(name, 28));
    y += 18;
    tft.setCursor(0, y);
    // The IP stays plain black on white in every state: it is the recovery path, and a
    // solid alarm fill behind an address hurts legibility exactly when it is needed most.
    setInk(UI_FG, UI_BG);
    bprint(padRight(ip, 22));

    // ── Temperature (chip die + board ambient), each colour-coded to its band ──
    // ALWAYS ON SCREEN (do not let it fall off again): the vertical budget above is
    // sized so this row lands at y=294 and ends at 310, inside the 320 px panel, with
    // the 3-row corrections block and the 3-row caster block both at fixed height. The
    // guard is a backstop against a future layout change, not an expected path — die
    // temperature is a primary health signal and belongs on the device itself.
    y += 18;
    if (y + 16 <= 320) {
        tft.setCursor(0, y);
        const UiStatus chipSt  = tempBandStatus(g_chipTempC, true);
        const UiStatus boardSt = tempBandStatus(g_boardTempC, false);
        setLabel(worst(chipSt, boardSt));
        bprint("CPU ");
        setStatus(chipSt);
        bprint(padRight(isnan(g_chipTempC) ? String("--")
                           : String((int)lroundf(cToF(g_chipTempC))) + "F", 7));
        setLabel(worst(chipSt, boardSt));
        bprint("IMU ");
        setStatus(boardSt);
        bprint(padRight(g_boardTempOk ? String((int)lroundf(cToF(g_boardTempC))) + "F"
                                         : String("--"), 6));
    }
}

// ── PPP survey completion → saved base position (the handoff) ────────────────
// WHY THIS EXISTS: ppp_survey.cpp locks the averaged PPP coordinate into the LG290P
// itself (PQTMCFGSVIN mode 2 + PQTMSAVEPAR) but has no knowledge of OUR NVS. Without
// this hook the ESP side ends a successful PPP survey with no saved position at all,
// so the next boot's confidence check has nothing to compare against and falls through
// to a fresh survey-in — throwing away the very coordinate the survey just produced.
//
// WHY PPP RATHER THAN THE MODULE'S INTERNAL SURVEY-IN: the LG290P computes a position
// only in ROVER mode; in base mode it echoes the coordinate it was given. PPP is a
// rover-mode function, so "survey on PPP" can only mean: converge as a rover, capture
// the average, then switch to base with that coordinate — which is exactly the sequence
// ppp_survey.cpp runs. It also puts the acceptance criteria under our control (EPE-gated
// sampling, minimum sample count) instead of the module's opaque internal gate.
static PppSurveyState g_lastPppState = PPP_IDLE;

// Launch a PPP survey that configureLg290pBaseOnce() asked for. Deferred to loop()
// because ppp_survey_begin() drives its own paced command sequence and must not run
// inside the boot configure or an HTTP handler.
// Every PPP survey start comes through here — boot, restart button, clear-position —
// and EVERY one is preceded by the fixed-mode escape. The boot log proved why: the
// module boots as a PERSISTED fixed base (its own flash), the boot path started the
// survey with no escape, and the survey ran against the fixed-mode echo of the old
// coordinate ("First fix acquired" at the exact saved position). The proven PPP
// sequence's bare rover switch works from survey-in base mode, but demonstrably NOT
// from persisted-fixed — so the escape must run first, always. It is harmless when the
// module is already a rover, and two phases keep the loop non-blocking.
// ── Is PPP actually in use? Ask the receiver ─────────────────────────────────
// Both PQTMCFGPPP write sites fire and forget, so a module whose firmware lacks the
// command is indistinguishable from one where PPP is running. Read queries are safe on
// this module (they never restart the nav engine — see gnss configuration contract), so
// the honest answer costs one read. The reply lands in g_pppCfgReply and is shown on the
// dashboard verbatim; it is not parsed, because the field layout varies by firmware and
// only the OK/ERROR verdict is being asked for. Whether E6 signals are being tracked is
// answered from the GSV stream instead — ppp_survey's E6 monitor counts them directly,
// which needs no command and cannot misconfigure anything.
static void queryPppSupport() {
    g_pppSupported = -1;
    g_pppCfgReply[0]    = '\0';
    g_signalCfgReply[0] = '\0';
    g_pppQueryMs = millis();
    sendPqtm("PQTMCFGPPP,R");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    // PQTMCFGSIGNAL is NOT queried here. The command exists, but its read syntax is not
    // something this project has verified against the module, and it is the one command
    // in reach that governs which signals the receiver tracks at all. A query whose
    // argument list is wrong is not guaranteed to be treated as a query, and the failure
    // mode — a receiver that tracks nothing, persisted in NVM across reboots — is far
    // more expensive than the diagnostic is worth. It goes back in when a real reply has
    // been seen and its form is known, not before.
    if (g_pppSupported < 0) {
        // No reply at all within the drain. Not proof of absence, but the module answers
        // every command it implements, so this is the expected shape of "not supported".
        logEvent("warn", "PQTMCFGPPP,R unanswered - this firmware most likely has no PPP "
                         "command; HAS corrections are NOT being applied");
    } else if (g_pppSupported == 0) {
        logEvent("warn", "PPP not available: %s", g_pppCfgReply);
    } else {
        logEvent("ok", "PPP config: %s", g_pppCfgReply);
    }
}

static void servicePppSurveyStart() {
    if (!g_pppSurveyPending) return;
    if (ppp_survey_active()) return;          // one survey at a time

    if (!g_pppEscapeDone) {
        // Phase 1: force the module out of any persisted fixed-base state, then give it
        // the post-restart settle before phase 2 sends the survey's own commands.
        char sv[96];
        sendPqtm("PQTMCFGRCVRMODE,W,1");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        snprintf(sv, sizeof(sv), "PQTMCFGSVIN,W,1,%lu,%.2f,0,0,0",
                 (unsigned long)surveyInSec, (double)surveyAccLimit);
        sendPqtm(sv);
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        sendPqtm("PQTMSAVEPAR");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        sendPqtm("PQTMHOT");
        g_pppSurveyNotBeforeMs = millis() + LG290P_POST_RESET_WAIT_MS;
        g_pppEscapeDone = true;
        // Deliberately NOT stamping g_probedRcvrMode/g_probedSvinMode here. This used to
        // record what the commands were meant to achieve, which made every downstream
        // "still fixed?" guard read back our own intent instead of the module's state.
        // Phase 2 reads the truth once the restart has settled.
        logEvent("info", "Cleared persisted base state before PPP survey (rover + save + hot restart)");
        return;
    }
    // Phase 2: module restarted and settled — hand over to the proven survey sequence.
    if (g_pppSurveyNotBeforeMs != 0 && (int32_t)(millis() - g_pppSurveyNotBeforeMs) < 0) return;
    // Confirm the escape actually took before surveying. A survey started against a module
    // still in fixed mode averages the echoed stored coordinate and re-locks the very
    // position the operator was replacing — ppp_survey.cpp's echo detector catches that,
    // but only after burning the entire survey window. Catching it here costs one read.
    if (!verifyEscapedFixedBase("PPP survey start")) {
        logEvent("warn", "PPP escape did not take - escalating to PQTMSRR before surveying");
        char sv[96];
        sendPqtm("PQTMCFGRCVRMODE,W,1");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        snprintf(sv, sizeof(sv), "PQTMCFGSVIN,W,1,%lu,%.2f,0,0,0",
                 (unsigned long)surveyInSec, (double)surveyAccLimit);
        sendPqtm(sv);
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        sendPqtm("PQTMSAVEPAR");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        sendPqtm("PQTMSRR");
        waitAndDrainGnss(LG290P_POST_RESET_WAIT_MS);
        if (!verifyEscapedFixedBase("PPP survey start retry")) {
            // On the card, not just the console. A module that refuses to leave base mode
            // produces two symptoms that look like separate faults — RTCM still streaming
            // while the firmware reports rover, and a survey countdown that will not
            // restart — and the only thing tying them together is this line.
            logEvent("fail", "Module STILL fixed after PQTMSRR - surveying anyway; RTCM will "
                             "keep streaming from the old coordinate and the echo detector "
                             "will abort if it is replaying it");
            Serial.println("   The retained config is battery-backed: clearing it requires "
                           "disconnecting the LG290P backup battery and all power.");
        }
    }
    g_pppSurveyPending = false;
    g_pppEscapeDone    = false;               // next survey escapes again
    // The module is a rover from here: PQTMSVINSTATUS stops, so anything still holding
    // the previous survey's verdict would hold it for the whole window.
    resetSurveyBookkeeping();
    ppp_survey_set_criteria(0.0f, 0.0f, 0, surveyInSec);   // duration only; limits kept
    ppp_survey_begin(GnssSerial, surveyInSec);
    // Deliberately NOT probing PQTMCFGPPP here. PPP is a rover function and the survey's
    // own first step is the switch to rover mode, so a read issued at this instant would
    // reach a receiver still in base mode and a refusal there would say nothing about
    // whether PPP is supported. The survey writes PQTMCFGPPP itself one step later, in
    // rover mode, and the reply lands in g_pppSupported through the same parser hook that
    // captures a read — so the authoritative answer arrives on its own, from the write
    // that actually matters, a couple of seconds from now.
    g_pppSupported = -1;
    g_pppCfgReply[0] = '\0';
    logEvent("ok", "PPP survey started (window %s)", formatDuration(surveyInSec).c_str());
}

// Report an echoed-coordinate abort loudly: it means the module ignored the rover-mode
// switch and is still a fixed base, which is the failure that silently re-locked the old
// coordinate. Edge-triggered so it prints once per occurrence.
// ── A failed survey must not be a dead end ──────────────────────────────────
// ppp_survey has TWO failure exits and only one of them was ever handled.
//
//   echoFault    the receiver returned identical fixes — still in fixed base mode,
//                echoing its stored coordinate. Retried once below.
//   no valid fix at all — "no sky, nothing to lock" (ppp_survey.cpp). NOT handled
//                anywhere. checkPppSurveyCompletion() returns unless PPP_DONE, and the
//                branch below only fires on echoFault, so this state had no reader.
//
// The second one is the realistic field failure: powered up inside a trailer, an antenna
// bumped loose, a cable seated badly. The survey runs its full 45-minute window, collects
// zero valid fixes, lands in PPP_FAILED, and then NOTHING happens — no position, no base
// mode, no corrections, no retry, and no message, because ppp_survey_active() is false in
// PPP_FAILED so even the quality gate stops saying "survey in progress" and reports the
// generic "no MSM observations" instead. The base sits dead until somebody notices and
// power-cycles it, which at an event is an hour of a session gone.
//
// A survey that failed for want of sky should simply be tried again once there is sky.
// Retries are spaced by PPP_RETRY_BACKOFF_MS so a genuinely blind receiver cannot spin,
// and each attempt is announced, so the event log shows exactly how many times it tried.
static uint32_t g_pppFailRetryMs    = 0;
static uint32_t g_pppFailRetryCount = 0;

// Put the BASE's satellite masks back the moment a PPP survey ends, whatever its outcome.
//
// The survey runs deliberately tight masks (see PPP_SURVEY_ELE_MASK_DEG) because PPP is
// computing one absolute position from its own observations and cannot difference away a
// low, weak satellite's errors. A base wants the opposite — every observation it can give a
// rover. So the tight masks must not outlive the survey, and "outlive" has to include the
// paths that are easy to forget: an abort, a failure for want of sky, an operator hitting
// Force base. Leaving them set is not cosmetic; a mask left over from other use is precisely
// what once starved this base to ten satellites with nothing below 30 degrees, and it would
// do it again silently.
//
// Watching ppp_survey_active() fall is the one test that catches every ending, which is why
// this is here rather than inside the success path.
static void serviceSurveyMaskRestore() {
    static bool wasActive = false;
    const bool active = ppp_survey_active();
    if (wasActive && !active) {
        logEvent("info", "PPP survey ended - restoring base satellite masks");
        applySatelliteGating();
    }
    wasActive = active;
}

static void servicePppSurveyRecovery() {
    const PppSurveyStatus st = ppp_survey_status();
    const uint32_t now = millis();

    // How long the survey has actually been running, measured HERE. The status struct
    // cannot answer this: its elapsedS is synthesised, and for any state at or past
    // LOCKING — which includes FAILED — it simply reports the full configured duration.
    // A survey aborted five seconds in therefore looks, from the outside, exactly like one
    // whose window expired. Timing it from this side is the only way to tell them apart,
    // and it covers a survey started from the PPP page as well as one started here.
    static uint32_t sActiveSinceMs = 0;
    static uint32_t sWindowS       = 0;
    if (ppp_survey_active()) {
        if (sActiveSinceMs == 0) sActiveSinceMs = now;
        if (st.durationS)        sWindowS       = st.durationS;
    } else if (st.state == PPP_DONE || st.state == PPP_IDLE) {
        sActiveSinceMs = 0;                     // finished cleanly; nothing to recover
        sWindowS       = 0;
    }

    // ONLY the no-sky failure retries. PPP_FAILED is also where a DELIBERATE stop lands:
    // ppp_survey_abort() sets it, and the dashboard's Abort button, /api/pppabort and
    // "Force base now" all call that. Retrying on any non-echo failure re-queued a survey
    // about a minute after the operator stopped one — and after Force, that survey escapes
    // the module to rover mode and discards the coordinate the operator just asserted.
    //
    // TWO conditions identify the failure this exists for, and both are needed. No valid
    // fix was ever seen, AND the window ran for its full length. The fix count alone is not
    // enough: an abort in the first seconds of a long window also has zero fixes, and that
    // is a plausible thing to do — a wrong duration typed in and noticed immediately — so
    // on its own it would restart the very survey the operator just cancelled.
    const bool windowRanOut = (sActiveSinceMs != 0) && (sWindowS != 0) &&
                              ((now - sActiveSinceMs) >= (sWindowS * 1000UL));
    if (st.state != PPP_FAILED || st.echoFault || st.validFixes > 0 || !windowRanOut) {
        g_pppFailRetryMs = 0;
        return;
    }
    if (g_pppSurveyPending || ppp_survey_active()) return;   // a retry is already queued
    if (g_pppFailRetryMs == 0) { g_pppFailRetryMs = now; return; }   // start the backoff clock
    if ((now - g_pppFailRetryMs) < PPP_RETRY_BACKOFF_MS) return;
    // Only retry once the receiver is actually seeing satellites. Restarting into the same
    // blind sky would burn another window and teach nothing; waiting costs only the backoff.
    if (!receiverHasSignal()) { g_pppFailRetryMs = now; return; }
    g_pppFailRetryMs = now;
    g_pppFailRetryCount++;
    sActiveSinceMs     = 0;           // the retry re-measures its own window from scratch
    sWindowS           = 0;
    g_pppEscapeDone    = false;
    g_pppSurveyPending = true;
    logEvent("warn", "PPP survey failed with no usable fix - retry %lu now that satellites "
                     "are being tracked", (unsigned long)g_pppFailRetryCount);
}

static void reportPppEchoFault() {
    static bool handled = false;
    static bool retried = false;
    const PppSurveyStatus st = ppp_survey_status();
    if (st.state == PPP_FAILED && st.echoFault) {
        if (!handled) {
            handled = true;
            logEvent("fail", "PPP survey aborted early: receiver returned identical fixes - it "
                             "is still in FIXED base mode; echoing its stored coordinate");
            if (!retried) {
                // One automatic recovery: requeue the survey, which re-runs the escape
                // (rover + save + hot restart) in servicePppSurveyStart(). Once only —
                // if the module ignores the escape twice, looping would just thrash it,
                // and the persistent ❌ line plus PPP_FAILED on the dashboard is the
                // honest signal that the module itself is refusing the mode change.
                retried = true;
                g_pppEscapeDone    = false;
                g_pppSurveyPending = true;
                logEvent("warn", "Retrying once: forcing the mode escape and restarting the survey");
            }
        }
    } else if (st.state != PPP_FAILED) {
        handled = false;
        if (st.state == PPP_DONE) retried = false;   // a successful survey re-arms the retry
    }
}

static void checkPppSurveyCompletion() {
    const PppSurveyStatus st = ppp_survey_status();
    const PppSurveyState prev = g_lastPppState;
    g_lastPppState = st.state;
    if (st.state != PPP_DONE || prev == PPP_DONE) return;   // edge-triggered

    double lat, lon, alt;
    ecefToLla(st.ecefX, st.ecefY, st.ecefZ, lat, lon, alt);

    // Accuracy of the AVERAGE, floored at the best single-epoch EPE actually observed.
    // The standard error of the mean is the optimistic figure — the same formal-error
    // trap as the receiver's survey meanAcc — so it never claims better than the best
    // convergence the session genuinely demonstrated.
    const uint32_t nUsed = st.lockUsedConverged ? st.samples : st.samplesAll;
    // Per-epoch accuracy of the set we actually locked: the converged path is bounded by
    // the best EPE PPP demonstrated, the autonomous path by the typical EPE across the
    // whole window. Divided by sqrt(n) for the mean, then floored at that per-epoch
    // figure so the saved accuracy never claims better than the data supports.
    const double perEpoch = st.lockUsedConverged
                            ? (isnan(st.bestEpe2d)   ? 0.0 : (double)st.bestEpe2d)
                            : (isnan(st.meanEpe2dAll) ? 0.0 : (double)st.meanEpe2dAll);
    const double sem = (nUsed > 0 && perEpoch > 0.0) ? perEpoch / sqrt((double)nUsed) : 0.0;
    float acc = (float)((perEpoch > sem) ? perEpoch : sem);
    if (!(acc > 0.0f)) acc = PPP_SURVEY_EPE_2D_LIMIT_M;   // never persist a zero/NaN accuracy

    char note[96];
    if (st.lockUsedConverged)
        snprintf(note, sizeof(note), "PPP converged avg, %lu samples over %s, best EPE %.2fm",
                 (unsigned long)nUsed, formatDuration(st.elapsedS).c_str(),
                 isnan(st.bestEpe2d) ? -1.0 : (double)st.bestEpe2d);
    else
        snprintf(note, sizeof(note), "Autonomous 3D avg (PPP unconverged), %lu fixes over %s, mean EPE %.1fm",
                 (unsigned long)nUsed, formatDuration(st.elapsedS).c_str(),
                 isnan(st.meanEpe2dAll) ? -1.0 : (double)st.meanEpe2dAll);

    savePositionToNvs(lat, lon, alt, acc, st.lockUsedConverged ? "ppp" : "ppp-auto",
                      st.elapsedS, note);

    // RESTORE BASE OUTPUTS (do not remove). The survey ran in ROVER mode and the lock
    // sequence switched back to base; per the PQTMCFGRCVRMODE spec that switch resets the
    // module's output configuration to base defaults, dropping our pinned RTCM rates and
    // the PQTM/NMEA telemetry rates the dashboard and logs depend on. ppp_survey.cpp has
    // already waited out the post-reset settle, so the module is listening again here.
    enableLg290pBaseOutputs();
    sendPqtm("PQTMSAVEPAR");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    // READ THE MODE BACK — do not stamp it from intent. The lock sequence wrote base mode
    // and a fixed svin, but writing a mode is not entering it: that is the same assumption
    // that let configureLg290pBaseOnce() report a base while the module sat in rover mode
    // publishing nothing, and the reason every other transition in this file verifies. The
    // probe values matter beyond reporting — a later "restart survey" consults them to
    // decide whether an escape restart is needed — so a wrong value is acted on, not just
    // displayed. The reply is parsed by the PQTMCFGRCVRMODE,OK handler, which also stamps
    // g_probedRcvrModeMs and clears g_baseModeConfirmed if the answer is not base mode.
    g_probedRcvrMode = -1;
    g_probedSvinMode = 0;
    sendPqtm("PQTMCFGRCVRMODE,R");
    waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
    sendPqtm("PQTMCFGSVIN,R");
    // Waits on the REPLY, not on a fixed interval, exactly as verifyEscapedFixedBase does.
    // One command spacing is not a reply deadline: the module has just restarted and is
    // reacquiring, so a slow answer would read as a failed lock and would leave
    // g_baseModeConfirmed false on a base that had in fact entered base mode correctly —
    // which updateBaseModeWatchdog() would then "repair" with a needless module reset.
    const uint32_t lockVerifyDeadline = millis() + LG290P_ESCAPE_VERIFY_MS;
    while ((int32_t)(millis() - lockVerifyDeadline) < 0) {
        processGnssSerial();
        if (g_probedRcvrMode != -1 && g_probedSvinMode != 0) break;
        delay(5);
    }
    if (g_probedRcvrMode == 2) {
        g_baseModeConfirmed = true;
        logEvent("ok", "PPP lock confirmed by read-back (svinMode=%d)", (int)g_probedSvinMode);
    } else {
        // Not a reason to discard the coordinate — it is saved and correct, and the
        // quality gate independently refuses to publish a stream with no observations in
        // it. It is a reason to say so, because the alternative is a base that looks
        // locked and casts nothing. updateBaseModeWatchdog() retries from here.
        logEvent("fail", "PPP lock wrote base mode but the module reports mode %d - "
                         "coordinate is saved; the base is not yet publishing from it",
                 (int)g_probedRcvrMode);
    }
    // The module is already fixed at this coordinate, so adopt it directly rather than
    // re-running the boot check against a position we just established ourselves.
    g_useNvsSavedPosition   = true;
    g_displacementCheckDone = true;
    g_posCheckReason[0]     = '\0';
    surveyInCompleteMs      = millis();
    Serial.printf("📍 PPP survey locked and saved: %.8f,%.8f alt=%.2f acc=%.3fm\n",
                  lat, lon, alt, (double)acc);
}

// ── Boot position-confidence check (called every loop pass) ───────────────────
// Replaces the old single-fix 5 km venue check. If a saved absolute position
// exists, accumulate PQTMNAV fixes for up to POSCHECK_WINDOW_MS and decide
// statistically whether the antenna is still on the saved spot:
//
//   σ_combined = sqrt( (mean_EPE / √N)² + saved_hacc² )
//   threshold  = clamp( 2 · σ_combined, FLOOR, CAP )
//   dist < threshold → CONFIRMED (reuse saved fixed pos)  ; else MOVED (re-survey)
//
// FAILSAFE (Steve's requirement): the base NEVER streams from a wrong position and
// NEVER blocks on zeros. Confirmed reuses the saved pos; Moved clears NVS and
// survey-ins; a window that ends with < POSCHECK_MIN_FIXES resolves Timeout →
// survey-in (conservative). Streaming is gated until this resolves (see loop()).
//
// A warm-rebooted module is already a fixed base emitting PQTMNAV from the saved
// position, so Confirmed leaves it untouched (no needless reset/SAVEPAR). A
// surveying module gets the saved fixed position applied via g_reconfigPending.
static void resolvePosCheckConfirmed() {
    g_posCheckReason[0]     = '\0';   // confirmed — nothing to explain
    g_useNvsSavedPosition   = true;
    g_posCheckState         = PosCheckState::Confirmed;
    g_displacementCheckDone = true;
    if (g_bootHotSkipped && g_probedSvinMode == 2 && !g_posCheckForcedRover) {
        // Already a fixed base at the saved position AND we never disturbed it —
        // correct as-is. Do NOT reconfigure (needless reset + SAVEPAR every reboot).
        Serial.printf("✅ CONFIRMED %.2fm < %.2fm — module already fixed at saved pos, no reset (src=%s)\n",
                      g_posCheckDistM, g_posCheckThreshM, g_savedSource);
    } else {
        // NOTE: when g_posCheckForcedRover is set we deliberately fall into the
        // reconfigure branch. We took the module OUT of fixed mode to get independent
        // fixes, so it is sitting in rover mode right now and MUST be put back — the
        // old "no reset needed" fast path would leave it as a rover casting nothing.
        g_reconfigPending = true;   // surveying / not yet fixed → apply saved fixed pos
        Serial.printf("✅ CONFIRMED %.2fm < %.2fm — applying saved fixed position (src=%s, acc=%.3fm)\n",
                      g_posCheckDistM, g_posCheckThreshM, g_savedSource, (double)g_savedHAcc);
    }
}

static void resolvePosCheckMoved(bool gross) {
    // Antenna moved (or saved pos is from a different venue). Saved pos must NOT be used.
    g_useNvsSavedPosition   = false;
    g_posCheckState         = PosCheckState::Moved;
    g_displacementCheckDone = true;
    clearSavedPosition();   // stale — don't tempt a future boot to reuse it
    // The hot start's survey verdict belongs to the coordinate just discarded. Without
    // this the base reports "survey-in complete" and stays baseReady through the whole
    // replacement survey (see resetSurveyBookkeeping).
    resetSurveyBookkeeping();
    if (gross) {
        snprintf(g_posCheckReason, sizeof(g_posCheckReason),
                 "moved %.0fm (>%.0fm gross) - different site", g_posCheckDistM, FIXED_POS_GROSS_MOVE_M);
        Serial.printf("⚠️  MOVED %.1fm > %.0fm gross bound — different venue, survey-in\n",
                      g_posCheckDistM, FIXED_POS_GROSS_MOVE_M);
    } else {
        snprintf(g_posCheckReason, sizeof(g_posCheckReason),
                 "moved %.2fm (>%.2fm limit)", g_posCheckDistM, g_posCheckThreshM);
        Serial.printf("⚠️  MOVED %.2fm > %.2fm threshold — antenna moved, survey-in\n",
                      g_posCheckDistM, g_posCheckThreshM);
    }
    if (g_bootHotSkipped && g_probedSvinMode == 2 && !g_posCheckForcedRover) {
        // Module hot-skipped while fixed at the OLD position — that's now WRONG.
        // Force a survey-in reconfigure at the new venue (never stream a stale fix).
        //
        // g_posCheckForcedRover MUST be excluded here (mirrors resolvePosCheckConfirmed):
        // g_bootHotSkipped/g_probedSvinMode are a snapshot from the boot probe and are
        // never updated by the forced-rover escape above, so without this guard the
        // condition stays true even after we've already switched the module to rover.
        // That routed MOVED through a redundant configureLg290pBaseOnce(true) instead of
        // the purpose-built PPP escape/launch path in servicePppSurveyStart().
        g_reconfigPending = true;
        Serial.println("   module was fixed at old venue — forcing survey-in reconfigure");
    } else if (BASE_SURVEY_USE_PPP) {
        // Rover-first boot: the module is a ROVER right now and nothing else starts a
        // survey. Queue the PPP survey or the base idles as a rover forever.
        g_pppSurveyPending = true;
    }
    // (Internal-survey design: boot already configured survey-in — nothing to do.)
}

static void checkPositionDisplacement() {
    if (g_displacementCheckDone) return;
    if (!g_savedPositionValid) {            // no saved pos → straight to survey-in
        g_displacementCheckDone = true;
        g_posCheckState = PosCheckState::Idle;
        return;
    }
    const uint32_t now = millis();
    // Tiered collection window: longer surveys earn longer confirmation (see
    // posCheckWindowMsForSurvey). g_savedSurveySec==0 (manual/legacy) → 30 s.
    // The settle is discarded time, so it is added to the window rather than taken out of
    // it — otherwise forcing the module to rover halves the fixes the decision rests on.
    const uint32_t windowMs = posCheckWindowMsForSurvey(g_savedSurveySec) +
                              (g_posCheckForcedRover ? POSCHECK_ROVER_SETTLE_MS : 0);

    // LIVE FIX SOURCE — GGA (+ PQTMPVT, which also populates ggaLat/Lon/LastSeen). These
    // are CORE messages present on every LG290P firmware revision. We deliberately do NOT
    // use PQTMNAV here: PQTMNAV was only added in the LG290P(03) firmware/spec v1.1, so on
    // an earlier module the enable is rejected, it never flows, and this check would time
    // out and force a needless survey-in on EVERY boot (the "random survey-in" bug). GGA
    // quality > 0 with a real lat/lon is all the venue check needs (its threshold floor is
    // 3 m, so single-point accuracy is plenty to tell "same spot" from "different venue").
    const int      liveFixQual = ggaFixQuality;
    const double   liveLat     = ggaLat;
    const double   liveLon     = ggaLon;
    const uint32_t liveFixMs   = ggaLastSeenMs;

    // Enter Collecting on the first usable fix. We require a real GGA fix (quality > 0)
    // AND non-zero lat/lon. While the antenna is disconnected these stay 0/zero, so the
    // check parks in Idle — the timeout clock (below) still bounds that so a truly dead
    // input can't hold the casters forever, but a brief antenna glitch won't burn the window.
    if (g_posCheckState == PosCheckState::Idle) {
        if (g_posCheckIdleSinceMs == 0) g_posCheckIdleSinceMs = now;   // start the failsafe clock
        if (liveFixQual <= 0 || liveFixMs == 0 || (liveLat == 0.0 && liveLon == 0.0)) {
            // Pick the applicable bound from what the receiver is doing right now — see
            // POSCHECK_IDLE_TIMEOUT_MS. The clock itself runs from Idle entry and is never
            // reset, so a receiver that falls silent partway through cannot extend its own
            // deadline by going quiet.
            const bool ggaFlowing = (ggaLastSeenMs != 0) &&
                                    ((now - ggaLastSeenMs) < SIGNAL_EVIDENCE_FRESH_MS);
            const uint32_t limitMs = ggaFlowing ? POSCHECK_IDLE_TIMEOUT_MS
                                                : POSCHECK_SILENT_TIMEOUT_MS;
            if (now - g_posCheckIdleSinceMs >= limitMs) {
                g_posCheckState         = PosCheckState::Timeout;
                g_displacementCheckDone = true;
                g_useNvsSavedPosition   = false;
                // The other Timeout exits call this and this one did not. All of them
                // abandon the saved coordinate and start a survey, and baseReady is a
                // set-only latch while surveyStatus caches a base-mode-only output —
                // neither clears itself, so whichever path skips this carries the previous
                // verdict into the new survey. That is the 0022 failure exactly.
                resetSurveyBookkeeping();
                if (ggaFlowing) {
                    snprintf(g_posCheckReason, sizeof(g_posCheckReason),
                             "receiver reports no fix in %lus - cannot verify site",
                             (unsigned long)(limitMs / 1000UL));
                    Serial.printf("⏱️  Position check: receiver talking but no fix in %lus → survey-in\n",
                                  (unsigned long)(limitMs / 1000UL));
                    logEvent("warn", "Venue check timed out: receiver reports no fix in %lus",
                             (unsigned long)(limitMs / 1000UL));
                } else {
                    snprintf(g_posCheckReason, sizeof(g_posCheckReason),
                             "no GGA from receiver in %lus - cannot verify site",
                             (unsigned long)(limitMs / 1000UL));
                    Serial.printf("⏱️  Position check: no GGA from receiver in %lus → survey-in\n",
                                  (unsigned long)(limitMs / 1000UL));
                    logEvent("warn", "Venue check timed out: no GGA from receiver in %lus "
                                     "- message rate may not have applied",
                             (unsigned long)(limitMs / 1000UL));
                }
                if (BASE_SURVEY_USE_PPP) g_pppSurveyPending = true;   // rover-first: queue it
            }
            return;
        }
        // ⚠⚠ ROVER MODE FIRST — WITHOUT THIS THE WHOLE CHECK IS A TAUTOLOGY ⚠⚠
        // In fixed-base mode the LG290P does NOT compute a position: it echoes its
        // CONFIGURED coordinate back out in GGA/PVT. Verified from the session 0454 boot
        // log — converting the configured ECEF to geodetic reproduces the reported GGA
        // lat/lon to every printed digit, and dist came out as exactly 0.00 m.
        // So when the module hot-starts already fixed, this check was comparing the saved
        // position against ITSELF and confirmed unconditionally. Field consequence: the
        // base once booted into fixed mode in a MOVING CAR 100 km from the saved site and
        // declared its position confirmed. Tightening the threshold cannot fix that —
        // 0.00 m passes any threshold. The fix is that the fixes we average must be
        // INDEPENDENT measurements, which means rover mode.
        // DO NOT REMOVE. DO NOT reorder this after the sampling loop.
        if (g_probedSvinMode == 2 || g_probedRcvrMode == 2) {
            // SAVE + RESET IS MANDATORY — a bare PQTMCFGRCVRMODE,W,1 DOES NOT TAKE EFFECT.
            // The spec is explicit: "After switching the module's working mode, save the
            // configuration and then reset the module. Otherwise, it will continue to
            // operate in the original mode." Without this the module stayed FIXED, kept
            // echoing its configured coordinate, dist came out ~0.00 m, and the check
            // confirmed the saved position no matter where the base actually was — the
            // exact tautology the rover switch exists to prevent.
            sendPqtm("PQTMCFGRCVRMODE,W,1");
            waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
            // SVIN MUST ALSO BE RESET HERE, not just RCVRMODE. A module that hot-started
            // fixed has SVIN mode 2 (fixed, with a coordinate) saved in its own NVS. That
            // register is orthogonal to RCVRMODE and PQTMSAVEPAR persists whatever is
            // currently staged — if we save+reset with SVIN still at 2, the fixed
            // coordinate survives the hot start alongside the rover switch, which is
            // exactly the "failed to clear the previous fixed position" failure this
            // block exists to prevent. servicePppSurveyStart()'s escape and the rover-
            // first boot branch both reset SVIN for the same reason; this path was
            // missing it.
            {
                char svReset[64];
                snprintf(svReset, sizeof(svReset), "PQTMCFGSVIN,W,1,%lu,%.2f,0,0,0",
                         (unsigned long)surveyInSec, (double)surveyAccLimit);
                sendPqtm(svReset);
            }
            waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
            sendPqtm("PQTMSAVEPAR");
            waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
            // HOT START, not PQTMSRR. Both restart the receiver and both make the mode
            // change take effect, but PQTMSRR is a full system reset that discards the
            // retained time, ephemeris, almanac and last known position — so the module
            // re-acquires from scratch and the check waits minutes for a first fix at a
            // poor-sky site. PQTMHOT keeps all of that, so the fixes we are about to
            // average start arriving almost immediately.
            //
            // Retaining the last position does NOT weaken the move test: a hot start
            // seeds the search, it does not report the seed as a fix. The receiver is in
            // ROVER mode here and solves independently, and the echo detector below
            // catches the case where it does not.
            sendPqtm("PQTMHOT");
            g_posCheckForcedRover = true;
            Serial.println("📍 Position check: module was FIXED — switching to rover + reset "
                           "so the fixes are independent (a fixed base echoes its own coordinate)");
        }
        g_posCheckState     = PosCheckState::Collecting;
        g_posCheckMeanLat   = 0.0;   // clear the previous check's result
        g_posCheckMeanLon   = 0.0;
        g_posCheckMeanEpe   = 0.0;
        g_posCheckDistM     = -1.0;
        g_posCheckStartMs   = now;
        g_posCheckCount     = 0;
        g_posCheckSumLat    = g_posCheckSumLon = g_posCheckSumEpe = 0.0;
        g_posCheckAllSame   = false;   // set on the first sample; see the echo detector
        g_posCheckFirstLat  = 0.0;
        g_posCheckFirstLon  = 0.0;
        g_posCheckLastNavMs = 0;
        copyLimited(lg290pModeText, sizeof(lg290pModeText), "pos check...");
        Serial.printf("📍 Position check: collecting GNSS fixes (window %lus, ≥%lu fixes; saved survey %lus, acc %.3fm)\n",
                      (unsigned long)(windowMs / 1000UL), (unsigned long)POSCHECK_MIN_FIXES,
                      (unsigned long)g_savedSurveySec, (double)g_savedHAcc);
    }
    if (g_posCheckState != PosCheckState::Collecting) return;

    // SETTLE: the first fixes after a rover-mode switch are still the old fixed-mode
    // echo or an unconverged solution. Discard them or they poison the mean toward 0.00 m
    // and re-create the tautology in a subtler form.
    if (g_posCheckForcedRover &&
        (now - g_posCheckStartMs) < POSCHECK_ROVER_SETTLE_MS) return;

    // Accumulate one sample per NEW fix epoch (dedupe by GGA timestamp), quality > 0.
    if (liveFixMs != g_posCheckLastNavMs && liveFixQual > 0 &&
        (liveLat != 0.0 || liveLon != 0.0)) {
        g_posCheckLastNavMs = liveFixMs;
        g_posCheckSumLat += liveLat;
        g_posCheckSumLon += liveLon;
        // Per-fix HORIZONTAL (2D) EPE. Must be liveErrorMeters2D, NOT liveErrorMeters:
        // the latter is 3D-preferred (vertical error is typically ~2x horizontal and it
        // dominates the RSS), but the move test below is a 2D Rayleigh containment test
        // in the horizontal plane. Feeding it a 3D sigma inflates the threshold and makes
        // the test TOO PERMISSIVE — field example from session 0454:
        //   $PQTMEPE,2,1.825,1.825,4.572,2.581,5.250  → 2D 2.581 m, 3D 5.250 m
        // which produced thresh = 10.99 m where the correct 2D input gives ~6.0 m. A
        // real displacement between those two figures would have been waved through.
        double epe = liveErrorMeters2D;
        if (epe <= 0.0) epe = 2.5;
        g_posCheckSumEpe += epe;
        // ECHO DETECTOR (failsafe, independent of any mode probe). A module echoing its
        // configured coordinate emits BIT-IDENTICAL lat/lon every epoch; a real GNSS
        // solution always jitters at least in the 7th decimal. If every sampled fix is
        // identical, we are not measuring anything and MUST NOT confirm — regardless of
        // what the mode registers claim. This catches the failure even if a future
        // firmware ignores the mode switch again.
        if (g_posCheckCount == 0) {
            g_posCheckFirstLat = liveLat;
            g_posCheckFirstLon = liveLon;
            g_posCheckAllSame  = true;
        } else if (liveLat != g_posCheckFirstLat || liveLon != g_posCheckFirstLon) {
            g_posCheckAllSame = false;
        }
        g_posCheckCount++;
    }

    // DECIDE AT THE END OF THE WINDOW, NOT AT THE MINIMUM COUNT (do not revert). This
    // used to also stop the moment POSCHECK_MIN_FIXES arrived, which at 1 Hz meant the
    // whole decision rested on the first ten seconds after a receiver restart — the fixes
    // least likely to be converged, and the ones a re-survey is then judged against. The
    // window is the averaging time the tiers were chosen for; the minimum count is a floor
    // below which the result is refused, not a trigger for taking it early. Nothing is
    // waiting on this beyond the streaming gate, and streaming from an unverified position
    // is the thing being avoided.
    const bool windowElapsed = (now - g_posCheckStartMs) >= windowMs;
    if (!windowElapsed) return;  // keep collecting

    // Decide. Need a minimum sample count for the statistics to mean anything.
    if (g_posCheckCount < POSCHECK_MIN_FIXES) {
        g_posCheckState         = PosCheckState::Timeout;
        g_displacementCheckDone = true;
        g_useNvsSavedPosition   = false;
        resetSurveyBookkeeping();   // survey-in follows — see the MOVED path
        snprintf(g_posCheckReason, sizeof(g_posCheckReason),
                 "only %lu/%lu fixes in %lus - too few to verify",
                 (unsigned long)g_posCheckCount, (unsigned long)POSCHECK_MIN_FIXES,
                 (unsigned long)(windowMs / 1000UL));
        Serial.printf("⏱️  Position check TIMEOUT — only %lu/%lu fixes in %lus → survey-in\n",
                      (unsigned long)g_posCheckCount, (unsigned long)POSCHECK_MIN_FIXES,
                      (unsigned long)(windowMs / 1000UL));
        // g_posCheckForcedRover excluded — see the identical guard in resolvePosCheckMoved.
        if (g_bootHotSkipped && g_probedSvinMode == 2 && !g_posCheckForcedRover) g_reconfigPending = true;
        else if (BASE_SURVEY_USE_PPP) g_pppSurveyPending = true;   // rover-first: queue it
        return;
    }

    // ECHO CHECK BEFORE ANY DECISION. Identical fixes across the whole window mean the
    // receiver was replaying a stored coordinate rather than solving — there is no
    // measurement here to confirm against, so re-survey. Confirming would reinstate the
    // "accepted the previous site's coordinates" failure.
    if (g_posCheckAllSame && g_posCheckCount >= 2) {
        g_posCheckState         = PosCheckState::Timeout;
        g_displacementCheckDone = true;
        g_useNvsSavedPosition   = false;
        resetSurveyBookkeeping();   // survey-in follows — see the MOVED path
        snprintf(g_posCheckReason, sizeof(g_posCheckReason),
                 "receiver echoed a fixed coordinate (%lu identical fixes) - cannot verify",
                 (unsigned long)g_posCheckCount);
        Serial.printf("⚠️  Position check: %lu IDENTICAL fixes — receiver is echoing its "
                      "configured coordinate, not solving. Forcing survey-in.\n",
                      (unsigned long)g_posCheckCount);
        if (BASE_SURVEY_USE_PPP) g_pppSurveyPending = true;   // rover-first: queue it
        return;
    }

    const double N        = (double)g_posCheckCount;
    const double meanLat  = g_posCheckSumLat / N;
    const double meanLon  = g_posCheckSumLon / N;
    const double meanEpe  = g_posCheckSumEpe / N;
    g_posCheckDistM       = roughDistanceM(meanLat, meanLon, g_savedLat, g_savedLon);
    // Latch for the status CSV and dashboard — this is the coordinate the base actually
    // measured at boot, and it is what the saved position was judged against.
    g_posCheckMeanLat = meanLat;
    g_posCheckMeanLon = meanLon;
    g_posCheckMeanEpe = meanEpe;

    // Gross sanity bound first: a saved pos from a completely different site is caught
    // here regardless of EPE, and doesn't need the statistical test below.
    if (g_posCheckDistM > FIXED_POS_GROSS_MOVE_M) { resolvePosCheckMoved(true); return; }

    // ── Move test: FIXED 5 m tolerance ───────────────────────────────────────
    // Deliberately a constant, not a computed confidence radius. The old computed
    // threshold scaled with measurement quality, which meant WORSE fixes produced a
    // LARGER threshold and confirmed more easily — bad data granted itself permission.
    // A constant cannot be gamed that way. It is a coarse test: it catches a different
    // venue, not a nudged tripod. Sub-5 m moves must be handled by clearing the saved
    // position manually. That trade is intentional — do not "improve" it by making the
    // threshold a function of EPE again.
    const double sigSaved = (g_savedHAcc < 99.0f) ? (double)g_savedHAcc : 0.0;
    g_posCheckThreshM     = POSCHECK_MOVE_THRESH_M;

    Serial.printf("📍 Pos check: N=%lu meanEPE=%.2fm σsaved=%.2fm rover=%d dist=%.2fm thresh=%.2fm (fixed)\n",
                  (unsigned long)g_posCheckCount, meanEpe, sigSaved,
                  g_posCheckForcedRover ? 1 : 0, g_posCheckDistM, g_posCheckThreshM);
    (void)N;

    // Confirmed REUSES the saved (high-accuracy) coordinate, not this window mean — the
    // mean only decides same-spot vs moved. It is latched above for the logs regardless.
    if (g_posCheckDistM < g_posCheckThreshM) resolvePosCheckConfirmed();
    else                          resolvePosCheckMoved(false);
}

// ── Entry points ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
    // NON-BLOCKING CONSOLE (do not remove). On this board Serial is USB CDC, and the class
    // default is to wait up to 100 ms for a host to drain the endpoint before giving up on
    // a write. With no terminal attached — which is every session that matters, since the
    // base runs on a roof or in a paddock — that wait is paid on EVERY print, on Core 1,
    // with the 460800-baud GNSS UART undrained behind it. The 4096-byte RX ring holds only
    // ~89 ms at line rate, so a single print could overrun it; sessions 0442-0455 lost bytes
    // continuously at 219 RTCM CRC and 193 framing failures per hour until the offending
    // print was gated off, and 0457 then measured zero.
    //
    // Gating individual prints treated the symptom one site at a time and left every future
    // print a latent repeat. A zero timeout removes the mechanism: a write with no host
    // attached is discarded immediately instead of blocking. It changes nothing when a
    // terminal IS attached and the endpoint is being drained, which is the only time the
    // output is being read anyway. The compile-time echo flags stay as they are — this is a
    // floor under them, not a replacement for them.
    Serial.setTxTimeoutMs(0);
#endif
    delay(500);

    // Resolves this unit's name (NVS-stored, or generated from its MAC on first
    // boot). Reads NVS + eFuse only — no WiFi radio involved — so it's safe this
    // early and doesn't affect the GNSS-probe/WiFi.mode() ordering below.
    loadDeviceName();
    loadSuggestedMount();
    {   // Local caster on/off survives reboots; on by default (it is the only way to
        // get corrections to a rover at a site with no internet).
        Preferences lp;
        if (lp.begin("rcx1id", true)) { g_localCasterEnabled = lp.getBool("localcast", true); lp.end(); }
    }

    qmi8658Init();   // probe board temp sensor (non-wedging; disables itself if absent)
    // Prime the ESP32-S3 die sensor: its first reads return 0.0 (not ready). Spin briefly
    // (force past the rate-limit) until we get a real value, so the boot line isn't 0°C.
    for (int i = 0; i < 20 && isnan(g_chipTempC); ++i) { readTemps(true); delay(15); }

    // BOOT DIAGNOSTIC (added for the restart investigation): print WHY the chip last
    // reset and how much RAM we have. A "frequent restart" complaint is unsolvable
    // without this — it converts each reboot into a labelled event in the serial log:
    //   • POWERON/RTC_SW/EXT      → clean (power button, upload, manual reset)
    //   • TASK_WDT/INT_WDT        → a task or ISR hung (e.g. a blocking call on a core)
    //   • PANIC                   → crash/exception (bad ptr, stack overflow, assert)
    //   • BROWNOUT                → 3.3V rail sagged (power/USB/SD current spike) — NOT software
    // Read the line printed right after the next restart to know which branch we're in.
    {
        esp_reset_reason_t rr = esp_reset_reason();
        const char* rs = "OTHER";
        switch (rr) {
            case ESP_RST_POWERON:  rs = "POWERON";   break;  // cold power-up
            case ESP_RST_SW:       rs = "SW_RESET";  break;  // esp_restart()/upload
            case ESP_RST_PANIC:    rs = "PANIC";     break;  // exception/abort/stack overflow
            case ESP_RST_INT_WDT:  rs = "INT_WDT";   break;  // interrupt watchdog
            case ESP_RST_TASK_WDT: rs = "TASK_WDT";  break;  // task watchdog (hung loop)
            case ESP_RST_WDT:      rs = "OTHER_WDT"; break;
            case ESP_RST_BROWNOUT: rs = "BROWNOUT";  break;  // power rail sag — hardware, not code
            case ESP_RST_DEEPSLEEP:rs = "DEEPSLEEP"; break;
            case ESP_RST_EXT:      rs = "EXT_RESET"; break;
            default: break;
        }
        Serial.printf("\n🔄 BOOT: reset_reason=%s | heap: free=%u largest=%u | psram_free=%u | chip=%s board=%s\n",
                      rs,
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                      (unsigned)ESP.getFreePsram(),
                      isnan(g_chipTempC) ? "n/a" : (String(cToF(g_chipTempC), 0) + "F").c_str(),
                      g_boardTempOk ? (String(cToF(g_boardTempC), 0) + "F").c_str() : "n/a");
    }

    preferences.begin("xbee", true);
    surveyInSec = preferences.getUInt("survey_sec", DEFAULT_SURVEY_IN_SEC);
    surveyAccLimit = preferences.getFloat("survey_acc", DEFAULT_SURVEY_IN_ACC_LIMIT_M);
    preferences.end();
    // Load saved absolute base position from NVS — written either by manual /api/setpos
    // entry OR auto-saved on survey-in completion (tagged with survey duration + accuracy).
    // The boot confidence check in loop() decides whether to reuse it this session.
    if (loadSavedPosition()) {
        Serial.printf("📍 NVS: loaded saved position %.8f,%.8f (src=%s, acc=%.3fm, survey=%lus)\n",
                      g_savedLat, g_savedLon, g_savedSource, (double)g_savedHAcc,
                      (unsigned long)g_savedSurveySec);
    }

    initDisplay();
    drawBootSplash("Booting...", g_savedPositionValid ? "saved base loaded" : "no saved base");
    drawBootSplash("Starting SD log", "");
    bridge_sdlog_init();   // starts the Core-0 SD logging task
    // BUFFERING-SAFETY: the LG290P streams RTCM+NMEA at 460800 baud (~46 KB/s).
    // The ESP32 default UART RX ring is only 256 bytes (~5.5 ms of data) — any
    // loop stall longer than that (a blocking client.connect(), a dashboard
    // render, a WiFi roam) overruns the FIFO and SHREDS RTCM frames (shows up as
    // climbing rtcmFramingFailures/rtcmCrcFailures and sagging fps). 4 KB buys
    // ~89 ms of slack. MUST be called before begin(). Highest-ROI fix in here —
    // do not remove.
    GnssSerial.setRxBufferSize(4096);
    GnssSerial.begin(GNSS_BAUD, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
    GnssSerial.setTimeout(1);

    // ONE-SHOT RECOVERY (config.h). Placed here deliberately: it must run AFTER
    // GnssSerial.begin() above — otherwise every command below is written to a closed
    // port and silently does nothing — and BEFORE configureLg290pBaseOnce(), so the
    // probe sees the module already back in survey-in mode.
    if (FORCE_CLEAR_POSITION_ON_BOOT) {
        drawBootSplash("Clearing position", "ESP NVS + module");
        Serial.println("🧹 FORCE_CLEAR_POSITION_ON_BOOT: wiping saved position (ESP NVS + module)");
        clearSavedPosition();
        // Put the MODULE back into survey-in and persist that, otherwise it keeps its own
        // fixed coordinate regardless of anything the ESP32 does. Same escape sequence
        // the restart path uses: rover, survey-in targets, save, hot restart.
        char sv[96];
        sendPqtm("PQTMCFGRCVRMODE,W,1");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        snprintf(sv, sizeof(sv), "PQTMCFGSVIN,W,1,%lu,%.2f,0,0,0",
                 (unsigned long)surveyInSec, (double)surveyAccLimit);
        sendPqtm(sv);
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        sendPqtm("PQTMSAVEPAR");
        waitAndDrainGnss(LG290P_COMMAND_SPACING_MS);
        // PQTMSRR: this changes receiver mode, and a hot start does not reliably apply a
        // mode change on this module. Same reason as configureLg290pBaseOnce().
        sendPqtm("PQTMSRR");
        waitAndDrainGnss(LG290P_POST_RESET_WAIT_MS);
        Serial.println("🧹 Module returned to survey-in. Set FORCE_CLEAR_POSITION_ON_BOOT=false and reflash.");
    }

    // Satellite gating before anything else touches the module. The boot position check
    // and the PPP survey both start collecting fixes shortly after this point, and a
    // stale rover mask would silently starve every one of them. See applySatelliteGating().
    drawBootSplash("Satellite gating", "elevation + C/N0");
    applySatelliteGating();

    // The probe below can block up to ~60 s waiting for the module's config replies when
    // GNSS is cold — tell the operator so the screen isn't a mystery during that wait.
    drawBootSplash("Configuring GNSS", "waiting for module");
    configureLg290pBaseOnce();
    drawBootSplash("GNSS configured", lg290pModeText);

    WiFi.mode(WIFI_AP_STA);
    // WiFi.begin() writes STA config to the driver's OWN internal flash NVS on
    // every call by default — separate from and invisible to our "rcx_wifi"
    // Preferences namespace. A flash write briefly disables the instruction
    // cache on both cores, which stalls the GNSS UART ISR; that exact mechanism
    // is the confirmed root cause of this project's worst prior data-corruption
    // regression (see history). With several stored networks and some out of
    // range, our rotation calls WiFi.begin() roughly every 10 s indefinitely
    // while unconnected — that write was firing continuously. RAM-only storage
    // here does not affect what's saved in our own "rcx_wifi" namespace at all.
    WiFi.persistent(false);
    // The Arduino core's own auto-reconnect keeps retrying a failed SSID in the
    // background on its own clock, independently of our 10 s rotation below. If
    // our rotation calls WiFi.begin() for the NEXT network while that background
    // retry is mid-attempt, the driver rejects it ("wifi:sta is connecting,
    // cannot set config" / ESP_ERR_WIFI_STATE) and the rotation stalls instead of
    // advancing. connectToWifiIndex()/serviceWifi() already implement our own
    // rotation+retry, so the built-in one only fights it. Confirmed present by
    // this exact error recurring in a boot log even with no explicit
    // WiFi.disconnect() call anywhere in the connection path.
    WiFi.setAutoReconnect(false);
    loadWifiNetworks();   // load NVS-stored networks into g_wifiNetworks
    // OPEN AP BY DESIGN (do not add a compile-time password back). This AP is the
    // recovery path used when no configured WiFi is in range — it is how the operator
    // reaches the dashboard to fix the WiFi settings in the first place. A password
    // baked into the firmware defeats that, and a short one is worse still: WPA2
    // requires >= 8 characters, so anything shorter makes softAP() fail outright with
    // "passphrase too short!" and leaves NO dashboard at all. The operator can set an
    // AP password from the dashboard; it is stored in NVS, not compiled in.
    const bool apOk = WiFi.softAP(g_deviceName.c_str());
    // softAP()'s return value was previously discarded entirely — if it fails
    // internally we'd have had zero visibility into that. Log it either way.
    if (apOk) {
        Serial.printf("AP started: SSID=%s IP=%s\n",
                      g_deviceName.c_str(), WiFi.softAPIP().toString().c_str());
    } else {
        Serial.printf("AP FAILED to start: SSID=%s\n", g_deviceName.c_str());
    }
    connectToWifiIndex(0);
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/status", HTTP_GET, handleApiStatus);
    server.on("/api/config", HTTP_POST, handleApiConfig);
    server.on("/api/setname", HTTP_POST, handleApiSetName);
    server.on("/api/localcast", HTTP_POST, handleApiLocalCast);
    server.on("/api/log", HTTP_ANY, handleApiLog);
    server.on("/api/force", HTTP_POST, handleApiForce);
    server.on("/api/reconfig", HTTP_POST, handleApiReconfig);
    server.on("/api/query", HTTP_POST, handleApiQuery);
    server.on("/api/addwifi",  HTTP_POST, handleApiAddWifi);
    server.on("/api/delwifi",  HTTP_POST, handleApiDelWifi);
    server.on("/api/wifilist", HTTP_GET,  handleApiWifiList);
    server.on("/api/casteren",  HTTP_POST, handleApiCasterEnable);
    server.on("/api/casteradd", HTTP_POST, handleApiCasterAdd);
    server.on("/api/mountcheck", HTTP_GET, handleApiMountCheck);
    server.on("/api/casterdel", HTTP_POST, handleApiCasterDel);
    server.on("/api/clearpos", HTTP_POST, handleApiClearPos);
    server.on("/api/setpos", HTTP_POST, handleApiSetPos);
    ppp_web_register(server, GnssSerial);  // PPP survey: /ppp page + /api/ppp,/api/pppstart,/api/pppabort
    server.on("/logs.json", HTTP_GET, handleLogsJson);
    server.on("/dl", HTTP_GET, handleDownload);
    server.on("/api/logdel", HTTP_POST, handleApiLogDelete);
    server.on("/rtcm.raw", HTTP_GET, handleRtcmRaw);
    server.on("/caster0/raw", HTTP_GET, handleCaster0Raw);
    server.on("/caster1/raw", HTTP_GET, handleCaster1Raw);
    server.begin();

    // Build the caster list from NVS, applying each one's persisted enable flag.
    // Must come AFTER the array exists (global) and is safe to call here since no
    // streaming happens until the loop's readiness gate.
    loadCasters();

    // Per-caster reconnect floor (see NtripTarget::reconnectBaseMs). Every caster
    // gets the strict 30 s + backoff floor: casters are added from the dashboard
    // in whatever order the operator chooses, so there's no fixed slot to treat
    // as a known, no-ban-risk target. Some services (e.g. rtk2go) document a low
    // abuse threshold and require >=10 s retries; the strict floor keeps every
    // caster clear of that regardless of which one it turns out to be. That
    // strictness does NOT make any caster second-class — it stays a fully
    // first-class path.
    for (int i = 0; i < casterCount; ++i) {
        casters[i].reconnectBaseMs = NTRIP_RECONNECT_INTERVAL_MS;
        setCasterState(casters[i], casters[i].enabled ? CasterState::WaitingForWifi
                                                      : CasterState::Disabled);
    }
}

static void serviceLogging() {
#if BRIDGE_LOG_SD_ENABLE
    static uint32_t lastTickMs = 0;
    const uint32_t now = millis();
    if (now - lastTickMs < 200) return;   // build/tick at 5 Hz; the logger gates each file further
    lastTickMs = now;

    static BridgeLogSnapshot snap;
    snap.nowMs = now;
    copyLimited(snap.utc,  sizeof(snap.utc),  ggaUtc);
    copyLimited(snap.date, sizeof(snap.date), ggaDate);
    snap.haveFix = ggaFixQuality > 0;
    snap.fixQuality = ggaFixQuality;
    snap.satellites = ggaSatellites;
    snap.hdop = ggaHdop;
    snap.lat = ggaLat;
    snap.lon = ggaLon;
    snap.altM = ggaAlt;
    snap.epeM = liveErrorMeters;
    snap.epe2dM = liveErrorMeters2D;
    snap.ecefX = ecefX;
    snap.ecefY = ecefY;
    snap.ecefZ = ecefZ;
    snap.svinValid = svinValidNow();
    snap.svinObs = surveyElapsedSeconds();
    snap.svinTarget = surveyTargetSeconds();
    // svin_meanacc_m: the REALISTIC (scatter-floored) accuracy — see realisticAccuracyM().
    // meanAcc is ZERO in fixed mode (module clears it at survey completion); logging that
    // raw would write 0.000 m — a perfect-accuracy claim — for entire sessions, so this
    // falls back to the saved coordinate's accuracy once fixed, same as the dashboard text.
    // -1 means no honest figure exists yet. Writing 0.000 for that claims sub-millimetre
    // accuracy on a row where nothing is known, and it is indistinguishable in the CSV
    // from a genuine reading. svinMeanAccKnown drives a BLANK field instead — the same
    // convention pos_src and the temperature columns already use for "no value".
    const double realAcc = realisticAccuracyM();
    snap.svinMeanAccKnown = (realAcc >= 0.0);
    snap.svinMeanAccM = snap.svinMeanAccKnown ? realAcc : 0.0;
    // Provenance of the coordinate we are casting from — logged every row. posAccM is
    // deliberately the RAW saved accuracy (not scatter-floored): it documents what the
    // hot-start move-check actually assumed, for cross-checking that math from the logs.
    copyLimited(snap.posSource, sizeof(snap.posSource),
                g_savedPositionValid && g_savedSource[0] ? g_savedSource : "");
    snap.posAccM = g_savedPositionValid ? (double)g_savedHAcc : -1.0;

    // Boot position-confidence check, logged every row so the whole 30 s window is
    // reconstructable from the card: the coordinate the base measured, its EPE, and how
    // far that landed from the saved coordinate against the threshold it was judged on.
    // While Collecting these are the RUNNING mean (the window is only ~6 status rows, so
    // the progression is worth having); after resolution they are the latched result.
    {
        const int pcIdx = static_cast<int>(g_posCheckState);
        static const char* kPcNames[] = { "idle", "collecting", "confirmed", "moved", "timeout" };
        copyLimited(snap.pcState, sizeof(snap.pcState),
                    (pcIdx >= 0 && pcIdx < 5) ? kPcNames[pcIdx] : "?");
        snap.pcFixes = g_posCheckCount;
        if (g_posCheckState == PosCheckState::Collecting && g_posCheckCount > 0) {
            const double n = (double)g_posCheckCount;
            snap.pcLat  = g_posCheckSumLat / n;
            snap.pcLon  = g_posCheckSumLon / n;
            snap.pcEpeM = g_posCheckSumEpe / n;
        } else {
            snap.pcLat  = g_posCheckMeanLat;
            snap.pcLon  = g_posCheckMeanLon;
            snap.pcEpeM = g_posCheckMeanEpe;
        }
        snap.pcDistM   = g_posCheckDistM;
        snap.pcThreshM = g_posCheckThreshM;
    }
    // g_probedSvinMode is unsigned and uses 0 for "unknown"; its companion
    // g_probedRcvrMode is signed and uses -1. A negative test here was written as if the
    // two shared a sentinel, and the compiler reports it as always false. Kept explicit
    // rather than silently dropped, because the mismatch between the two variables is the
    // thing worth seeing at a glance.
    snap.svinMode = g_probedSvinMode;   // 0 = unknown, 1 = survey-in, 2 = fixed
    // DMA-capable internal heap, sampled every status row. SD, lwIP and WiFi all draw on
    // it and PSRAM cannot stand in for any of them, so its exhaustion shows up as
    // unrelated-looking failures elsewhere. heapMinInt is the low-water mark since boot,
    // which is what distinguishes a slow leak from ordinary churn.
    snap.heapFreeInt    = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    snap.heapLargestInt = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    snap.heapMinInt     = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    snap.localClients   = (uint8_t)localCasterActiveCount();
    snap.localServed    = g_localCasterServed;
    snap.hotStart = g_bootHotSkipped;
    snap.baseReady = baseReady;
    snap.rtcmFps = rtcmStats.framesPerSecond;
    snap.rtcmBps = rtcmStats.bytesPerSecond;
    snap.rtcmValidFrames = rtcmStats.rtcmValidFrames;
    // UART-integrity counters — see bridge_sd_log.h. Monotonic totals; diff them
    // between rows to see whether a drop in RTCM output came with data loss.
    // Peak-since-last-row, then reset so each row reports its own interval.
    snap.loopMaxMs             = g_loopStallMaxMs;
    snap.uartHighWater         = g_uartHighWater;
    g_loopStallMaxMs           = 0;
    g_uartHighWater            = 0;
    snap.rtcmCrcFailures       = rtcmStats.rtcmCrcFailures;
    snap.rtcmFramingFailures   = rtcmStats.rtcmFramingFailures;
    snap.nmeaChecksumFailures  = g_nmeaChecksumFailures;
    snap.nmeaFramerDesyncs     = g_nmeaFramerDesyncs;
    snap.wifiConnected = (WiFi.status() == WL_CONNECTED);
    const int snapWIdx = (activeWifiIndex >= 0) ? activeWifiIndex : wifiAttemptIndex;
    copyCsvField(snap.ssid, sizeof(snap.ssid),
                snap.wifiConnected && snapWIdx < (int)g_wifiNetworks.size()
                ? g_wifiNetworks[snapWIdx].ssid.c_str() : "");
    copyLimited(snap.ip, sizeof(snap.ip),
                snap.wifiConnected ? WiFi.localIP().toString().c_str() : "");
    snap.chipTempF  = isnan(g_chipTempC) ? NAN : cToF(g_chipTempC);
    snap.boardTempF = g_boardTempOk ? cToF(g_boardTempC) : NAN;
    snap.boardTempOk = g_boardTempOk;
    for (int i = 0; i < 2; ++i) {
        // snap.caster[] has two slots (the SD status CSV schema). If fewer than two
        // casters are configured, the remaining slot stays blank rather than
        // reading out of range.
        if (i >= casterCount) {
            snap.caster[i].host[0] = '\0'; snap.caster[i].err[0] = '\0';
            snap.caster[i].mount[0] = '\0'; snap.caster[i].enabled = false;
            continue;
        }
        copyLimited(snap.caster[i].host, sizeof(snap.caster[i].host), casters[i].host);
        copyCsvField(snap.caster[i].mount, sizeof(snap.caster[i].mount), casters[i].mountpoint);
        // Without this, "Disabled" in the CSV covers both "the operator switched it off"
        // and "the firmware is holding it down", and the two demand opposite responses.
        snap.caster[i].enabled = casters[i].enabled;
        copyCsvField(snap.caster[i].state, sizeof(snap.caster[i].state), stateToString(casters[i].state));
        snap.caster[i].handshake = casters[i].handshakeAccepted;
        snap.caster[i].framesWritten = casters[i].framesWritten;
        snap.caster[i].bytesAccepted = casters[i].bytesAccepted;
        snap.caster[i].dropped = casters[i].droppedWriteCount;
        snap.caster[i].lastWriteAgeS = ageSeconds(casters[i].lastWriteMs);
        // The caster's own last error, and — when the gate is what is holding it down —
        // the gate's reason instead. Without this the CSV records that nothing was
        // published and nothing about why, so a caster rejecting the connection is
        // reconstructable only from a dashboard someone happened to be watching.
        if (casters[i].state == CasterState::Held && g_castQualityReason[0])
            copyCsvField(snap.caster[i].err, sizeof(snap.caster[i].err), g_castQualityReason);
        else
            copyCsvField(snap.caster[i].err, sizeof(snap.caster[i].err), casters[i].lastError);
    }
    bridge_sdlog_tick(snap);
#endif
}

// Drive the onboard RGB LED from the same condition the LCD title band shows.
//
// Deliberately NOT called from inside updateDisplay(): loop() skips that entirely when the
// panel is switched off, which would freeze the LED on whatever colour it happened to be
// showing at the moment the screen went dark — an indicator asserting a condition nobody
// is updating. The display switch is the operator saying "go dark", so it takes the LED
// with it, and both come back together.
//
// Rate-limited to 1 Hz here because statusState() walks the caster table and calls
// WiFi.status(), which takes the WiFi API lock that the WiFi and lwIP tasks contend for.
// It answers a question that changes at human speed and has no business running thousands
// of times a second on the core draining the GNSS UART. The write itself is one 24-bit
// shift, about 30 us on the wire, and only happens on a change or when the refresh falls
// due — three orders of magnitude inside the 89 ms the UART ring can absorb.
static void serviceRgbStatus() {
    static uint32_t lastMs = 0, lastWriteMs = 0;
    static uint8_t  lastKey = 0xFF;               // no state maps to this → forces a write
    const uint32_t now = millis();
    if (lastMs != 0 && (now - lastMs) < 1000) return;
    lastMs = now;

    // The panel's background is WHITE, so "off" here cannot borrow UI_BG the way the
    // display code does: on an emitter white is the brightest output there is, the exact
    // opposite of dark.
    uint8_t   key = 0;
    LedColour c   = LED_DARK;
    if (displayTelemetryEnabled) {
        const UiStatus st = statusState();
        key = (st == ST_OK) ? 1 : (st == ST_BAD) ? 3 : 2;
        c   = (st == ST_OK) ? LED_OK : (st == ST_BAD) ? LED_BAD : LED_WARN;
    }
    if (key == lastKey && (now - lastWriteMs) < LED_REFRESH_MS) return;
    const bool first = (lastKey == 0xFF);
    lastKey     = key;
    lastWriteMs = now;

    const uint8_t r = (uint8_t)(c.r * LED_BRIGHTNESS / 255u);
    const uint8_t g = (uint8_t)(c.g * LED_BRIGHTNESS / 255u);
    const uint8_t b = (uint8_t)(c.b * LED_BRIGHTNESS / 255u);
    rgbLedWriteOrdered(LED_PIN, LED_ORDER, r, g, b);

    // Reported once, on the first write only. A bead showing the wrong colour and a bead the
    // firmware never reached look identical on the bench, and they have completely different
    // causes. Printing the values that were REQUESTED, next to what is actually glowing,
    // also identifies a byte-order mismatch immediately: asking for 64,0,0 and seeing green
    // is the whole diagnosis, and LED_ORDER is the fix.
    if (first) Serial.printf("💡 Status LED: pin %d, order %d, first colour requested %u,%u,%u\n",
                             LED_PIN, (int)LED_ORDER, (unsigned)r, (unsigned)g, (unsigned)b);
}

// Main loop — runs on Core 1. ORDER MATTERS and is dependency-driven, not arbitrary:
//   1. processGnssSerial()      drains the UART, frames RTCM, updates fix/survey state.
//   2. serviceWifi()            non-blocking WiFi rotation across g_wifiNetworks.
//   3. armSurveyClockOnFix()    starts the survey countdown only once a real fix exists.
//   4. updateBaseReadiness()    computes baseReady — MUST run before the caster service
//                               so a not-ready base never streams (see its own header).
//   4b. updateCastQuality()     computes g_castQualityOk from the live stream — also
//                               before the caster service, for the same reason.
//   5. checkPositionDisplacement() the boot "did it move?" state machine; sets
//                               g_useNvsSavedPosition / g_displacementCheckDone, may set
//                               g_reconfigPending. MUST run before the streaming gate.
//   6. g_reconfigPending        deferred reconfigure from a web change or the confidence
//                               check, executed HERE (in loop context) not in the HTTP
//                               handler, because it blocks ~8 s on the module reset.
//   7. streaming gate           stream to each ENABLED caster only when base is ready AND
//                               the venue is confirmed; otherwise hold every caster down.
//   8. serviceLogging / updateMetrics / updateDisplay — observers, run last.
// Do not reorder 3–7 without re-checking the data dependencies above.
void loop() {
    // Measure how long the PREVIOUS pass took before doing anything else.
    {
        const uint32_t loopNow = millis();
        if (g_lastLoopMs != 0) {
            g_loopStallMs = loopNow - g_lastLoopMs;
            if (g_loopStallMs > g_loopStallMaxMs) g_loopStallMaxMs = g_loopStallMs;
        }
        g_lastLoopMs = loopNow;
    }
    updateWifiLinkState();              // debounced link state for wifi + casters
    processGnssSerial();
    server.handleClient();
    serviceWifi();
    serviceForceBase();                 // operator-forced fixed base (loop context only)
    serviceSurveyMaskRestore();         // tight PPP masks never outlive the survey
    servicePppSurveyRecovery();         // a survey that failed for want of sky retries itself
    servicePppSurveyStart();            // launch a pending PPP survey (loop context only)
    ppp_survey_tick();                  // advance the PPP manual survey-in state machine
    checkPppSurveyCompletion();         // on completion, persist the coordinate + provenance
    reportPppEchoFault();               // loud diagnostic if the module echoed instead of solving
    
    armSurveyClockOnFix();              // start survey clock on first valid lat/lon
    updateBaseReadiness();              // AUTO-START gate — must run before serviceCaster()
    updateCastQuality();                // correction-quality gate — also before serviceCaster()
    updateBaseOutputWatchdog();         // repairs a base that is publishing no observations
    updateBaseModeWatchdog();           // repairs a base that never entered base mode at all
    updateHeapWatchdog();               // names internal-heap exhaustion before it surfaces elsewhere
    checkPositionDisplacement();        // boot position-confidence state machine (sets g_useNvsSavedPosition)
    if (g_reconfigPending) {            // deferred from web handler — ~8 s, runs here not in HTTP
        g_reconfigPending = false;
        g_surveyRestartPending = false; // a full reconfigure already restarts the survey
        configureLg290pBaseOnce(true);  // user-requested change → force reconfigure + reset
    } else if (g_surveyRestartPending) { // warm re-survey — targets only, no module reset
        g_surveyRestartPending = false;
        restartSurveyWarm();
    }
    serviceLocalCaster();               // accept/handshake local rovers (non-blocking)
    connectAttemptedThisPass = false;   // BUFFERING-SAFETY: one connect per pass; see serviceCaster()
    // STALL FORGIVENESS (keep): every caster watchdog below tests `now - <timestamp>`
    // against a fixed threshold. If the pass we just measured blocked for seconds,
    // those deltas are inflated by the stall itself and ALL of them fire at once on
    // resume — the response (5 s), auth-stall (12 s) and uplink-stall (8 s) timers
    // alike. That is why a caster caught mid-handshake got torn down while a caster
    // already Streaming survived: processGnssSerial() runs first and refreshes
    // lastWriteMs before its watchdog is ever evaluated. Slide the timestamps forward
    // by the stall so a watchdog only ever fires on time the caster actually had.
    if (g_loopStallMs > CASTER_STALL_FORGIVE_MS) {
        const uint32_t nowFg = millis();
        // CLAMP (do not remove). Sliding a timestamp forward by the raw stall can push
        // it PAST now — e.g. a write 1 s ago plus a 3 s stall lands 2 s in the future.
        // Every age check here is unsigned (`now - ts`), so a future timestamp wraps to
        // ~4.29e9 ms and every watchdog fires INSTANTLY — the exact opposite of what
        // forgiveness is for. This was live: session 0450 logged c0_age_s = 4294963 on
        // an Error row. Never advance a timestamp beyond now.
        auto forgive = [&](uint32_t& ts) {
            if (ts == 0) return;
            if ((uint32_t)(nowFg - ts) > g_loopStallMs) ts += g_loopStallMs;
            else                                        ts  = nowFg;
        };
        for (int i = 0; i < casterCount; ++i) {
            forgive(casters[i].stateStartMs);
            forgive(casters[i].lastWriteMs);
            forgive(casters[i].lastAttemptMs);
        }
    }
    // FAILSAFE: with a saved fixed position, hold ALL streaming until the boot
    // confidence check resolves (up to POSCHECK_WINDOW_MS while it averages PQTMNAV
    // fixes). A warm hot-skipped module is already emitting MSM from its saved fixed
    // position; without this gate we could stream corrections referenced to a position
    // we haven't yet confirmed the antenna still occupies. g_displacementCheckDone is
    // set the instant the check resolves Confirmed/Moved/Timeout (and is immediately
    // true when there's no saved position, so a normal survey-in boot is never delayed).
    const bool venueConfirmed = g_displacementCheckDone || !g_savedPositionValid;
    // g_castQualityOk is the "don't publish bad data" gate — see updateCastQuality().
    // It is ANDed here rather than folded into surveyIsReady() so the two stay separable:
    // readiness is about the survey, quality is about the bytes.
    const bool readyToStream = surveyIsReady() && venueConfirmed && g_castQualityOk;
    for (int i = 0; i < casterCount; ++i) {
        if (readyToStream && casters[i].enabled) {
            serviceCaster(casters[i]);
        } else {
            // Two different situations that used to log identically as "Disabled", which
            // made a gate-held caster indistinguishable from one the operator had switched
            // off — and left "why is nothing casting" unanswerable from a log alone.
            stopCaster(casters[i], casters[i].enabled ? CasterState::Held
                                                      : CasterState::Disabled);
        }
    }
    
    serviceLogging();
    readTemps();              // refresh chip + board temps (internally rate-limited to 0.5 Hz)
    updateMetrics();
    if (displayTelemetryEnabled) updateDisplay();
    serviceRgbStatus();
}