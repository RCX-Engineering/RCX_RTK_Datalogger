/*
 * ntrip.cpp — NTRIP client
 * =========================
 * Refactored from RaceBox_LG290P.ino.  Logic is identical; code is split
 * into functions callable from wifiNtripTask without the monolithic .ino context.
 */

#include "ntrip.h"
#include "config.h"
#include "types.h"
#include "gnss.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <math.h>
#include <esp_heap_caps.h>   // heap_caps_get_free_size (low-heap scan guard)
#include "debug_log.h"   // LAST include: serial-to-SD tee macro must not touch library headers above

// ── Runtime caster table ──────────────────────────────────────────────────────
// The casters the client actually uses are assembled at boot into this table from
// two sources, in priority order:
//   1. The compile-time casters in config.cpp (ntripCasters[] / ntripPreferredMpt[]).
//   2. Any casters persisted in NVS namespace "rcx_ntrip" (added at runtime via
//      ntrip_saveCaster(), e.g. from a future web form or a provisioning step).
// Using one runtime table — instead of indexing ntripCasters[] directly everywhere —
// is what lets an NVS-added caster be counted and scanned without a recompile. The
// table is bounded by NTRIP_MAX_CASTERS, which also sizes lastCasterFetch[] and is
// the bound scanCaster() enforces, so the count can never index any array out of range.
struct NtripCasterRT {
    char host[48];
    char port[8];
    char user[64];
    char pass[40];
    char prefMpt[64];   // preferred mountpoint ("" = auto-select nearest)
    bool enabled;       // false = kept in the table and listed, but never scanned
    bool isDefault;     // true = compile-time row; listable and disableable, not deletable
    // Position in the array this row came from: ntripCasters[] for a default,
    // the NVS list for an added caster. Held explicitly because loadCasters()
    // skips blank rows, so a row's runtime index does NOT reliably equal its
    // source index — deriving one from the other would attach a saved enable
    // flag to the wrong caster as soon as any row were left blank.
    int  srcIdx;
};
static NtripCasterRT casters[NTRIP_MAX_CASTERS];
static int           castersCount = 0;   // total active = compile-time + valid NVS

// Set by the caster-management entry points, which are called from the web
// server's task. The table must NOT be rebuilt underneath a scan in progress —
// castersCount and activeCasterIdx would disagree for the rest of that pass — so
// mutations only raise this flag and ntrip_loop() applies the change at the top
// of its next iteration, where nothing is mid-scan.
static volatile bool castersReloadPending = false;

// Operator-requested full reset (web dashboard button). Raised from the
// async_tcp task, consumed at the top of ntrip_loop() — the one point where
// no scan is in flight — because all NTRIP state is owned by wifiNtripTask
// and must never be mutated from another task. Same pattern as
// castersReloadPending.
static volatile bool operatorResetPending = false;

// A position good enough to spend caster budget on. gps.valid alone is NOT
// sufficient: TinyGPS++ reports a location "valid" the moment coordinate
// fields parse, and during acquisition the receiver can emit 0.000000
// placeholders — so valid can be true while the position is the null island.
// Geographic mountpoint scoring against 0,0 is garbage, and each scan it
// triggers burns that caster's 5-minute source-table budget, delaying the
// REAL scan once a fix arrives. Require a genuine fix and a coordinate that
// cannot be the placeholder. (A true fix at 0.0000,0.0000 is open ocean off
// West Africa — not a use case this rover serves.)
static inline bool ntripGoodFix(const GnssData& g) {
    return g.valid && g.fixType >= 2 &&
           (fabs(g.latitude) > 0.0001 || fabs(g.longitude) > 0.0001);
}

// Assemble the runtime table. Called once from ntrip_init(). A caster is "valid"
// (and therefore counted) only if it has a non-empty host — this is exactly the
// "increment the caster count if a valid caster is in/added to NVS" rule: blank or
// half-written NVS slots are skipped, never counted.
static void loadCasters() {
    castersCount = 0;

    // 1. Compile-time casters (config.cpp). ntripPreferredMpt[] is parallel to
    //    ntripCasters[] and guaranteed ≥ ntripCasterCount by a static_assert in
    //    config.cpp, so indexing it here is safe.
    // Enable flags live in the same NVS namespace as added casters. Defaults are
    // keyed by their position in ntripCasters[] ("d<i>"), added casters by their
    // position in the NVS list ("e<i>"). Reordering rows in config.cpp would
    // therefore re-associate a default's saved flag — that is a source change
    // requiring a reflash, so it is a build-time concern, not a field one.
    Preferences flags;
    const bool haveFlags = flags.begin("rcx_ntrip", true);

    for (int i = 0; i < ntripCasterCount && castersCount < NTRIP_MAX_CASTERS; i++) {
        if (!ntripCasters[i][0] || !ntripCasters[i][0][0]) continue;   // skip blank host
        strlcpy(casters[castersCount].host,    ntripCasters[i][0], sizeof(casters[0].host));
        strlcpy(casters[castersCount].port,    ntripCasters[i][1], sizeof(casters[0].port));
        strlcpy(casters[castersCount].user,    ntripCasters[i][2], sizeof(casters[0].user));
        strlcpy(casters[castersCount].pass,    ntripCasters[i][3], sizeof(casters[0].pass));
        strlcpy(casters[castersCount].prefMpt, ntripPreferredMpt[i], sizeof(casters[0].prefMpt));
        casters[castersCount].isDefault = true;
        casters[castersCount].srcIdx    = i;
        // Absent flag means enabled: a default the operator has never touched is on.
        char fkey[8];
        snprintf(fkey, sizeof(fkey), "d%d", i);
        casters[castersCount].enabled = haveFlags ? flags.getBool(fkey, true) : true;
        castersCount++;
    }
    if (haveFlags) flags.end();

    // 2. NVS casters. Stored as count "n" plus per-index keys h<i>/p<i>/u<i>/w<i>/m<i>.
    Preferences p;
    if (p.begin("rcx_ntrip", true)) {              // read-only open
        int n = p.getInt("n", 0);
        for (int i = 0; i < n && castersCount < NTRIP_MAX_CASTERS; i++) {
            char key[8], host[48] = "";
            snprintf(key, sizeof(key), "h%d", i); p.getString(key, host, sizeof(host));
            if (host[0] == '\0') continue;          // invalid slot — skip, do NOT count
            strlcpy(casters[castersCount].host, host, sizeof(casters[0].host));
            snprintf(key, sizeof(key), "p%d", i); p.getString(key, casters[castersCount].port,    sizeof(casters[0].port));
            snprintf(key, sizeof(key), "u%d", i); p.getString(key, casters[castersCount].user,    sizeof(casters[0].user));
            snprintf(key, sizeof(key), "w%d", i); p.getString(key, casters[castersCount].pass,    sizeof(casters[0].pass));
            snprintf(key, sizeof(key), "m%d", i); p.getString(key, casters[castersCount].prefMpt, sizeof(casters[0].prefMpt));
            if (casters[castersCount].port[0] == '\0') strlcpy(casters[castersCount].port, "2101", sizeof(casters[0].port));
            casters[castersCount].isDefault = false;
            casters[castersCount].srcIdx    = i;
            snprintf(key, sizeof(key), "e%d", i);
            casters[castersCount].enabled = p.getBool(key, true);
            castersCount++;
        }
        p.end();
    }

    if (castersCount == 0) {
        // Defensive: a misconfigured build with zero valid casters would otherwise
        // loop scanning nothing. Log loudly so it's obvious in the serial monitor.
        Serial.println("⚠️  NTRIP: no valid casters configured (check config.cpp / NVS)");
    } else {
        Serial.printf("📡 NTRIP: %d caster(s) active (%d compile-time + %d from NVS)\n",
                      castersCount, ntripCasterCount, castersCount - ntripCasterCount);
    }
}

// ── Module state ──────────────────────────────────────────────────────────────
static WiFiClient    ntripClient;
static char          activeMountpoint[64] = "";
static int           activeCasterIdx      = 0;
static double        activeBestDistSq     = 1e18;
static double        activeMountLat       = NAN;
static double        activeMountLon       = NAN;
static float         activeMountDistKm    = -1.0f;
static int8_t        activeMountCarrier   = -1;
static bool          activeMountVRS       = false;

static unsigned long lastMountpointSearch = 0;
static unsigned long lastConnectAttempt   = 0;
static unsigned long lastGgaSend          = 0;
static unsigned long lastRescanTime       = 0;
static unsigned long lastCasterFetch[NTRIP_MAX_CASTERS] = {0};
static int           ntripFailCount       = 0;

// True when activeMountpoint is a caster's configured preferred mount (own base).
// Gates the hourly rescan-for-closer (never abandon our own base for a public one)
// and selects the cooldown path when the mount proves bad at runtime.
static bool          activeIsPreferred    = false;
// Per-caster cooldown: millis() before which the preferred mountpoint of caster i
// must not be retried. Set when the mount proved bad AT RUNTIME (silent stream
// 2 sessions in a row, or RTCM 1005/1006 baseline beyond NTRIP_MAX_BASELINE_KM)
// so the geographic fallback gets a fair chance instead of phase-0 re-grabbing
// the bad mount on every selection pass. 0 = no cooldown.
static unsigned long prefMptCooldownUntil[NTRIP_MAX_CASTERS] = {0};
// No-data watchdog: millis() of the last RTCM byte actually read this session,
// and the count of consecutive sessions that connected but went silent.
static unsigned long lastRtcmByteMs       = 0;
static uint8_t       silentSessionCount   = 0;

// millis() at which the current NTRIP session last became connected. Used by the
// quick-drop detector in ntrip_loop: a session that drops within
// NTRIP_RECONNECT_FLOOR_MS of connecting is treated as a soft failure (failCount++)
// so an unstable/half-dead mountpoint escalates the backoff instead of looping at
// the floor forever. 0 = no successful connect yet this session.
static unsigned long lastConnectOkMs      = 0;
// Edge-detector for the connected→disconnected transition (see ntrip_loop).
static bool          wasNtripConnected    = false;

volatile uint32_t rtcmBytesTotal = 0;
static uint32_t   rtcmBytesLast  = 0;
static uint32_t   lastRtcmLog    = 0;

// ── Utility ───────────────────────────────────────────────────────────────────
static float distanceKm(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371.0088;
    double dLat = radians(lat2-lat1), dLon = radians(lon2-lon1);
    double a = sin(dLat/2)*sin(dLat/2)
             + cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
    return (float)(R * 2.0 * atan2(sqrt(a), sqrt(1.0-a)));
}

// ── RTCM 1005/1006 baseline sniffer ──────────────────────────────────────────
// A direct (no source-table) connect to a preferred mountpoint has no caster
// coordinates to validate against — Centipede doesn't even LIST undeclared
// bases — so the imposter/baseline check moves into the stream itself: extract
// the base antenna reference point (ECEF, 0.1 mm units) from the first RTCM
// 1005 or 1006 message forwarded and compute the true 3D baseline to the rover.
// This is strictly stronger than the old source-table check (tables can lie or
// be stale; 1005/1006 is what the receiver actually uses) and it also gives the
// dashboard a real baseline figure for scanned mounts. Runs once per session:
// after a baseline is established the sniffer goes idle (zero per-byte cost
// beyond one branch in the forward loop).

// CRC-24Q (RTCM3 frame integrity). Bitwise, no table — runs only on candidate
// 1005/1006 frames (≤27 bytes) until the baseline is known, so speed is moot.
static uint32_t crc24q(const uint8_t* d, size_t n) {
    uint32_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint32_t)d[i] << 16;
        for (int b = 0; b < 8; b++) {
            crc <<= 1;
            if (crc & 0x1000000) crc ^= 0x1864CFB;
        }
    }
    return crc & 0xFFFFFF;
}

// MSB-first bit-field extractor (RTCM payloads are big-endian bit streams).
static uint64_t rtcmBits(const uint8_t* p, int firstBit, int len) {
    uint64_t v = 0;
    for (int i = 0; i < len; i++) {
        int bit = firstBit + i;
        v = (v << 1) | ((p[bit >> 3] >> (7 - (bit & 7))) & 1u);
    }
    return v;
}

// WGS84 geodetic → ECEF. Used to place the rover next to the base's native
// ECEF coordinates so the baseline is a plain 3D Euclidean distance — no
// iterative ECEF→geodetic conversion needed.
static void llaToEcef(double latDeg, double lonDeg, double hM, double out[3]) {
    const double a = 6378137.0, e2 = 6.69437999014e-3;
    double lat = radians(latDeg), lon = radians(lonDeg);
    double sl = sin(lat), cl = cos(lat);
    double N = a / sqrt(1.0 - e2 * sl * sl);
    out[0] = (N + hM) * cl * cos(lon);
    out[1] = (N + hM) * cl * sin(lon);
    out[2] = (N * (1.0 - e2) + hM) * sl;
}

static bool baselineChecked = false;   // true once 1005/1006 seen this session
static bool baselineTooFar  = false;   // set by sniffer; acted on in ntrip_loop
// Base station ECEF, captured the first time 1005/1006 is decoded this session. The base is
// stationary, so this is fixed for the session — but the ROVER moves, so the displayed
// distance must be recomputed from this + the live rover position (see ntrip_updateBaseDist),
// NOT frozen at the connect-time value. baseEcefValid gates that recompute.
static bool   baseEcefValid = false;
static double baseEcef[3]   = {0, 0, 0};
static struct {
    enum BsnState : uint8_t { SYNC, LEN1, LEN2, BODY } st = SYNC;
    uint16_t payloadLen = 0;   // RTCM payload length from the 2-byte header
    uint16_t need = 0;         // total frame bytes (3 hdr + payload + 3 CRC)
    uint16_t idx  = 0;         // bytes consumed of the current frame
    bool     keep = false;     // store this frame? (only 1005/1006-sized ones)
    uint8_t  buf[32];          // 1005 frame = 25 B, 1006 = 27 B — fits easily
} bsn;

static void baselineSnifferReset() {
    bsn.st = bsn.SYNC; bsn.idx = 0; bsn.keep = false;
    baselineChecked = false; baselineTooFar = false;
    baseEcefValid = false;   // new session → forget the old base position
}

// Decode a complete, CRC-valid candidate frame in bsn.buf.
static void baselineDecodeFrame(const GnssData& g) {
    const uint8_t* pl = bsn.buf + 3;                       // payload start
    uint16_t msg = (uint16_t)rtcmBits(pl, 0, 12);
    if (msg != 1005 && msg != 1006) return;                // other small frame
    if (msg == 1005 && bsn.payloadLen != 19) return;       // exact lengths only
    if (msg == 1006 && bsn.payloadLen != 21) return;

    // DF025/DF026/DF027: ECEF X/Y/Z, 38-bit signed, 0.0001 m resolution.
    // Payload bit offsets: X@34, Y@74 (34+38+2), Z@114 (74+38+2).
    auto s38 = [&](int off) -> double {
        uint64_t raw = rtcmBits(pl, off, 38);
        if (raw & (1ULL << 37)) raw |= ~((1ULL << 38) - 1); // sign-extend
        return (double)(int64_t)raw * 1e-4;
    };
    double bx = s38(34), by = s38(74), bz = s38(114);

    // Plausibility: a real antenna sits near the Earth's surface. A mis-synced
    // or garbage frame that survived CRC by fluke must not trigger a disconnect.
    double r = sqrt(bx*bx + by*by + bz*bz);
    if (r < 6.2e6 || r > 6.5e6) return;

    double rov[3];
    llaToEcef(g.latitude, g.longitude, g.altMSL, rov);
    double dx = bx-rov[0], dy = by-rov[1], dz = bz-rov[2];
    float km = (float)(sqrt(dx*dx + dy*dy + dz*dz) / 1000.0);

    // Keep the base position so the distance can be refreshed against the live rover
    // position as it drives — the sniffer itself latches off after this (baselineChecked),
    // which is why the displayed distance used to freeze at the connect-time value.
    baseEcef[0] = bx; baseEcef[1] = by; baseEcef[2] = bz;
    baseEcefValid = true;

    baselineChecked   = true;
    activeMountDistKm = km;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        status.ntripDistanceKm = km;
        xSemaphoreGive(dataMutex);
    }
    if (km > (float)NTRIP_MAX_BASELINE_KM) {
        Serial.printf("❌ NTRIP: RTCM %u says base %s is %.1f km away (cap %.0f km) — imposter or wrong mount, disconnecting\n",
                      msg, activeMountpoint, km, (double)NTRIP_MAX_BASELINE_KM);
        baselineTooFar = true;        // acted on in ntrip_loop after the burst
    } else {
        Serial.printf("📡 NTRIP: baseline verified from RTCM %u — %s is %.2f km away\n",
                      msg, activeMountpoint, km);
    }
}

// Feed forwarded RTCM bytes. Frames larger than the buffer (MSM etc.) are
// length-counted and skipped without storing; only 1005/1006-sized frames are
// kept and CRC-checked. Misalignment self-heals on the next 0xD3 preamble.
static void baselineSnifferFeed(const uint8_t* d, size_t n, const GnssData& g) {
    for (size_t i = 0; i < n && !baselineChecked; i++) {
        uint8_t b = d[i];
        switch (bsn.st) {
            case bsn.SYNC:
                if (b == 0xD3) { bsn.buf[0] = b; bsn.idx = 1; bsn.st = bsn.LEN1; }
                break;
            case bsn.LEN1:
                if (b & 0xFC) { bsn.st = bsn.SYNC; break; }   // reserved bits set → resync
                bsn.buf[1] = b; bsn.payloadLen = (uint16_t)(b & 0x03) << 8;
                bsn.idx = 2; bsn.st = bsn.LEN2;
                break;
            case bsn.LEN2:
                bsn.buf[2] = b; bsn.payloadLen |= b;
                bsn.need = 3 + bsn.payloadLen + 3;
                bsn.keep = (bsn.need <= sizeof(bsn.buf));     // store small frames only
                bsn.idx = 3; bsn.st = bsn.BODY;
                break;
            case bsn.BODY:
                if (bsn.keep) bsn.buf[bsn.idx] = b;
                bsn.idx++;
                if (bsn.idx >= bsn.need) {
                    if (bsn.keep) {
                        uint32_t want = ((uint32_t)bsn.buf[bsn.need-3] << 16) |
                                        ((uint32_t)bsn.buf[bsn.need-2] <<  8) |
                                         (uint32_t)bsn.buf[bsn.need-1];
                        if (crc24q(bsn.buf, bsn.need - 3) == want)
                            baselineDecodeFrame(g);
                    }
                    bsn.st = bsn.SYNC;
                }
                break;
        }
    }
}

// Reconnect/scan backoff in ms.
//
// BAN-SAFETY FLOOR (important): rtk2go and most public casters ban clients that
// reconnect faster than ~30 s. The exponential schedule below only protects us
// Reconnect/scan backoff in ms.
//
// COLD-START LATENCY vs BAN-SAFETY — the two requirements pull in opposite
// directions, so we split the behaviour by ntripFailCount:
//
//   failCount == 0  → "first attempt" (fresh boot, or the one prompt retry right
//     after a long healthy session ends). Return NTRIP_INITIAL_RETRY_MS (a few
//     seconds) with NO 30 s floor, so a unit powered on right before a run reaches
//     RTK as soon as WiFi + GPS are ready instead of idling for 30 s. This cannot
//     create a ban-triggering loop: a failCount==0 pass does at most ONE connect,
//     and source-table fetches are independently rate-limited to 5 min per caster.
//
//   failCount  > 0  → genuine retry (connect refused, or the quick-drop detector
//     fired because a session died within NTRIP_RECONNECT_FLOOR_MS of connecting).
//     THIS is where reconnect loops would form, so we clamp to a 30 s floor — under
//     rtk2go's ~30 s ban threshold no path can reconnect — and let the exponential
//     schedule climb toward the 60 s cap.
// Forward declaration — connectNtrip is defined after the mountpoint-selection
// functions but called from findNearestMountpoint's Phase-0 direct-connect path.
static NtripConnResult connectNtrip(const GnssData& g);

#define NTRIP_RECONNECT_FLOOR_MS 30000UL
static unsigned long ntripBackoffMs() {
    if (ntripFailCount == 0) return NTRIP_INITIAL_RETRY_MS;   // prompt; no floor (see above)
    unsigned long b = NTRIP_CONNECT_INITIAL_RETRY_MS * (1UL << min(ntripFailCount, 6));
    if (b < NTRIP_RECONNECT_FLOOR_MS) b = NTRIP_RECONNECT_FLOOR_MS;          // ban-safety floor
    return min(b, (unsigned long)NTRIP_CONNECT_BACKOFF_MAX_MS);             // 60 s cap
}

// ── Base64 credentials ────────────────────────────────────────────────────────
static String ntripBase64() {
    String creds = String(casters[activeCasterIdx].user)
                 + ":" + String(casters[activeCasterIdx].pass);
    const char* t = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    String b64;
    for (int i = 0; i < (int)creds.length(); i += 3) {
        uint32_t n = ((uint32_t)(uint8_t)creds[i] << 16)
                   | ((i+1 < (int)creds.length() ? (uint32_t)(uint8_t)creds[i+1] : 0) << 8)
                   | ((i+2 < (int)creds.length() ? (uint32_t)(uint8_t)creds[i+2] : 0));
        b64 += t[(n>>18)&63]; b64 += t[(n>>12)&63];
        b64 += (i+1<(int)creds.length()) ? t[(n>>6)&63] : '=';
        b64 += (i+2<(int)creds.length()) ? t[n&63]      : '=';
    }
    return b64;
}

// ── Source table scan for one caster ─────────────────────────────────────────
static bool scanCaster(int ci, const GnssData& g,
                        char* outMpt, double* outDistSq,
                        double* outLat, double* outLon,
                        int8_t* outCarrier, bool* outVRS) {
    // Bounds guard: lastCasterFetch[] / activeCasterIdx are sized to
    // NTRIP_MAX_CASTERS. Refuse out-of-range indices so adding casters beyond
    // the array size can never write out of bounds.
    if (ci < 0 || ci >= NTRIP_MAX_CASTERS) return false;
    if (!ntripGoodFix(g)) return false;   // backstop: a source-table fetch scored
                                          // against a placeholder position wastes
                                          // this caster's 5-minute budget — no call
                                          // path may spend it without a real fix
    if (lastCasterFetch[ci] != 0 &&
        millis() - lastCasterFetch[ci] < NTRIP_SOURCE_TABLE_MIN_INTERVAL_MS) {
        // Not a failure — the per-caster source-table rate limit. Say so, or this
        // prints as a misleading "no response" in findNearestMountpoint.
        Serial.printf("rate-limited (next table fetch in %lus)\n",
                      (NTRIP_SOURCE_TABLE_MIN_INTERVAL_MS - (millis() - lastCasterFetch[ci])) / 1000UL);
        return false;
    }
    lastCasterFetch[ci] = millis();
    // SIDE EFFECT: scanCaster sets the global activeCasterIdx to the caster it is
    // currently probing, because ntripBase64() (called below) builds credentials
    // from ntripCasters[activeCasterIdx]. This is safe only because all NTRIP work
    // runs in the single wifiNtripTask. Callers that loop over casters (e.g.
    // rescanForBetter) MUST restore activeCasterIdx afterward if they choose not to
    // switch — otherwise the global is left pointing at the last-probed caster while
    // activeMountpoint still belongs to the connected one (wrong-credentials bug on
    // the next reconnect). findNearestMountpoint always overwrites it at the end, so
    // it is unaffected.
    activeCasterIdx = ci;

    // ── Low-heap guard ───────────────────────────────────────────────────────
    // A source-table scan needs ~25-30 KB of transient internal RAM (lwIP rx
    // pbufs for the inbound table + TLS-free TCP overhead). If internal heap is
    // already tight, starting a scan tips the whole system over: SDMMC writes
    // fail with ESP_ERR_NO_MEM (0x101) because the driver can't allocate its
    // DMA buffer, and AsyncTCP can't accept browser connections (web page dead).
    // Skip the scan and let the caller retry on the next pass instead.
    {
        size_t freeInt = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (freeInt < 30000) {
            Serial.printf("⚠️  NTRIP: skipping scan — internal heap low (%u B free, largest block %u B)\n",
                          (unsigned)freeInt,
                          (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            lastCasterFetch[ci] = 0;   // don't burn the rate-limit slot on a skip
            return false;
        }
        Serial.printf("📡 Scan start: %u B internal heap free\n", (unsigned)freeInt);
    }

    WiFiClient c;
    c.setTimeout(5000);
    if (!c.connect(casters[ci].host, atoi(casters[ci].port))) {
        // TCP-level failure: host down, port blocked, DNS, or — the classic for
        // rtk2go — this network's public IP is banned at their firewall (silent
        // drop, looks exactly like a connect timeout).
        Serial.printf("TCP connect to %s:%s FAILED (host down / port blocked / IP banned?)\n",
                      casters[ci].host, casters[ci].port);
        return false;
    }

    c.printf("GET / HTTP/1.0\r\nUser-Agent: NTRIP RCX_Datalogger/1.0\r\n"
             "Ntrip-Version: Ntrip/1.0\r\nAuthorization: Basic %s\r\n\r\n",
             ntripBase64().c_str());

    unsigned long t = millis() + 5000;
    while (!c.available() && millis() < t) delay(30);
    if (!c.available()) { c.stop(); return false; }

    // readLine: pull one '\n'-terminated line into a fixed buffer with ZERO heap
    // allocation. The old code did `String line = c.readStringUntil('\n')` for
    // every one of up to 2000 lines — thousands of malloc/free cycles of varying
    // sizes that fragmented internal RAM exactly while ~25 KB of lwIP pbufs were
    // also held for the inbound table. That fragmentation is what starved the
    // SDMMC driver (0x101 no-mem on every write) and AsyncTCP (web page refused).
    // Overlong lines are truncated and the remainder is drained byte-by-byte.
    auto readLine = [&c](char* buf, size_t bufSz) -> int {
        size_t i = 0; unsigned long dl = millis() + 3000;
        while (millis() < dl) {
            int b = c.read();
            if (b < 0) {                       // nothing buffered yet
                if (!c.connected()) break;     // connection ended → return what we have
                delay(1);                      // yield (also feeds IDLE0/TWDT)
                continue;
            }
            if (b == '\n') { buf[i] = '\0'; return (int)i; }
            if (b == '\r') continue;
            if (i < bufSz - 1) buf[i++] = (char)b;
            // else: overlong line — keep draining until '\n' without storing
        }
        buf[i] = '\0';
        return (i > 0) ? (int)i : -1;          // -1 = timeout/EOF with nothing read
    };

    static char lineBuf[256];          // reused every iteration (single task)
    static char* fields[18];           // pointers into lineBuf

    // Drain HTTP response headers (blank line or SOURCETABLE marker ends them).
    { bool inHdr = true; unsigned long dl = millis() + 3000;
      while (inHdr && millis() < dl) {
          int len = readLine(lineBuf, sizeof(lineBuf));
          if (len < 0) { c.stop(); return false; }
          if (len == 0 || strncmp(lineBuf, "SOURCETABLE", 11) == 0) inHdr = false;
      }
      if (millis() >= dl) { c.stop(); return false; } }

    // Parse source table line-by-line. delay(1) every 10 lines feeds IDLE0 so
    // the Task Watchdog never fires on large tables (rtk2go has 600+ STR rows).
    double bestEff=1e18, bestAct=1e18, bestLat=NAN, bestLon=NAN;
    char bestMpt[64]=""; int8_t bestCar=1; bool bestVRS=false;
    int stationsSeen=0;   // STR rows that passed the format/carrier/fee filters (diagnostic)
    unsigned long scanDeadline = millis() + 15000;   // hard cap on total scan time
    // The guard above turns a scan away below 30000 B on the strength of an
    // estimate — "~25-30 KB transient" — that has never been measured against a
    // real table. This tracks what the scan actually costs, sampled at the same
    // cadence as the yield below, so the threshold can be set from the number
    // instead of the estimate. It is the difference between the scan-start and
    // scan-low figures that matters, not either one alone.
    size_t scanMinFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    for (int n = 0; n < 2000 && millis() < scanDeadline; n++) {

        // Yield every 10 lines so IDLE0 runs and TWDT is fed.
        if (n % 10 == 0) {
            delay(1);
            const size_t nowFree =
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (nowFree < scanMinFree) scanMinFree = nowFree;
        }

        int len = readLine(lineBuf, sizeof(lineBuf));
        if (len < 0) break;                                   // EOF / timeout
        if (strncmp(lineBuf, "ENDSOURCETABLE", 14) == 0) break; // table complete — stop now
        if (strncmp(lineBuf, "STR;", 4) != 0) continue;

        // Split into fields in place (lineBuf already holds the raw line).
        int fi = 0;
        char* p = lineBuf;
        while (fi < 18) {
            fields[fi++] = p;
            char* semi = strchr(p, ';');
            if (!semi) break;
            *semi = '\0'; p = semi + 1;
        }
        if (fi < 11) continue;  // need at least lat/lon fields

        // Field 3 = format — must contain "RTCM"
        if (fi > 3 && *fields[3] && !strstr(fields[3], "RTCM")) continue;

        // Carrier from field 5, fallback to MSM7 detection in field 4
        int8_t carrier = (fi > 5 && *fields[5]) ? (int8_t)atoi(fields[5]) : 0;
        if (carrier == 0 && fi > 4 && *fields[4]) {
            carrier = (strstr(fields[4],"1077") || strstr(fields[4],"1087") ||
                       strstr(fields[4],"1097") || strstr(fields[4],"1117") ||
                       strstr(fields[4],"1127")) ? 2 : 1;
        } else if (carrier == 0) { carrier = 1; }
        if (carrier < 1) continue;

        // Field 16 = fee — skip paid stations
        if (fi > 16 && strcmp(fields[16], "Y") == 0) continue;

        // VRS: NMEA field 11 = "1" and not SNIP relay
        bool isSnip = (fi > 7 && strstr(fields[7], "SNIP") != nullptr);
        stationsSeen++;   // passed format/carrier/fee — a real candidate to score
        bool vrs    = !isSnip && (fi > 11 && strcmp(fields[11], "1") == 0);

        double stLat = atof(fields[9]), stLon = atof(fields[10]);
        double dL = stLat - g.latitude;
        double dO = stLon - g.longitude;
        
        // Apply cosine scaling to account for longitude convergence
        double cosLat = cos(radians(g.latitude));
        double scaledDo = dO * cosLat;
        
        double act = (dL * dL) + (scaledDo * scaledDo);
        double eff = act * (carrier >= 2 ? 0.60 : 1.0) * (vrs ? 0.70 : 1.0);

        // Preferred-mountpoint boost: if this caster has a configured preferred
        // mount (e.g. your own base station "RCX1"), and this line IS that mount,
        // force eff to -1 so it always wins over any geographic score from any
        // other caster. This gives graceful detection + failover:
        //   • RCX1 in source table  → eff=-1 → always wins
        //   • RCX1 absent (not broadcasting) → normal geographic scoring applies
        //     and rtk2go's nearest wins automatically
        const char* pref = casters[ci].prefMpt;
        if (pref[0] && strncmp(fields[1], pref, 63) == 0) {
            float km = distanceKm(g.latitude, g.longitude, stLat, stLon);
            if (km <= (float)NTRIP_MAX_BASELINE_KM) {
                bestEff=-1; bestAct=act; bestLat=stLat; bestLon=stLon; // Record true distance (act), not 0
                bestCar=carrier; bestVRS=vrs;
                strncpy(bestMpt, fields[1], 63); bestMpt[63]='\0';
                break;   // found our verified LOCAL mount — no need to read the rest
            } else {
                // Listed coordinates fail the baseline cap. Log it — the old
                // silent `continue` made "RCX1 listed with bogus/zero coords"
                // indistinguishable from "RCX1 not in the table at all".
                Serial.printf("\n⚠️  pref mount %s IS in %s's table but listed at %.1f km (cap %.0f km) — ignoring table entry\n",
                              pref, casters[ci].host, km, (double)NTRIP_MAX_BASELINE_KM);
                continue; // Imposter or bad coordinates! Ignore and keep scanning.
            }
        }

        if (eff < bestEff) {
            bestEff=eff; bestAct=act; bestLat=stLat; bestLon=stLon;
            bestCar=carrier; bestVRS=vrs;
            strncpy(bestMpt, fields[1], 63); bestMpt[63]='\0';
        }
    }
    c.stop();
    // DIAGNOSTIC: distinguish "table fetched but no station passed the filters"
    // (bestMpt empty → format/carrier/fee/baseline rejected everything) from
    // "found a candidate". This is the line that tells us whether Cyber_1 was
    // seen-and-rejected vs never-seen. stationsSeen counts STR rows scored.
    Serial.printf("📡 Scan %s: %d stations scored, best=%s\n",
                  casters[ci].host, stationsSeen,
                  bestMpt[0] ? bestMpt : "(none passed filters)");
    Serial.printf("📡 Scan end: %u B internal heap free (low during scan %u B)\n",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                  (unsigned)scanMinFree);
    if (!bestMpt[0]) return false;
    *outDistSq=bestAct; *outLat=bestLat; *outLon=bestLon;
    if (outCarrier) *outCarrier=bestCar; if (outVRS) *outVRS=bestVRS;
    // strncpy does NOT null-terminate when the source is exactly 63 chars, so
    // terminate explicitly. (Callers' buffers are zero-init today, but relying on
    // that is fragile — a future caller with an uninitialised buffer would read
    // past the string.)
    strncpy(outMpt, bestMpt, 63);
    outMpt[63] = '\0';
    return true;
}

// ── Mountpoint selection ──────────────────────────────────────────────────────
// Scans all configured casters. Preferred-mount logic lives inside scanCaster:
//   • ntripPreferredMpt[ci] non-empty → that mountpoint wins with eff=-1 if found
//     in the source table; absent (not broadcasting) → falls through to geographic
//   • All casters are always scanned so rtk2go is tried even when Centipede fails
static bool findNearestMountpoint(const GnssData& g) {
    if (!ntripGoodFix(g)) { Serial.println("📡 Waiting for GPS fix..."); return false; }

    // ── Phase 0: DIRECT preferred-mountpoint connect (own base) ──────────────
    // Centipede explicitly documents that an undeclared / pending-validation base
    // does NOT appear in the source table but IS reachable by naming the
    // mountpoint manually. The old logic could only connect to mounts it
    // discovered in a source table, which made an own-base mount like RCX1
    // permanently unreachable — the scan saw only declared stations (e.g. MCAN,
    // 738 km away), failed the baseline cap, and aborted. So: try a direct
    // `GET /<prefMpt>` first, no table required. The caster arbitrates liveness
    // when ntrip_loop performs the connect:
    //   • mount live      → "ICY 200 OK" + RTCM   → connected, done
    //   • mount not live  → "SOURCETABLE 200 OK"  → NTRIP_STALE → cooldown +
    //                        immediate re-selection, which falls through to the
    //                        geographic scan below (public base, e.g. rtk2go)
    //   • caster dead/ban → TCP fail              → NTRIP_FAIL → backoff; after
    //                        5× the mount is cooled down and the scan takes over
    // Baseline/imposter validation happens IN-STREAM via the RTCM 1005/1006
    // sniffer (there are no table coordinates to check for an unlisted mount).
    for (int ci = 0; ci < castersCount; ci++) {
        if (!casters[ci].enabled) continue;      // disabled from the dashboard
        if (!casters[ci].prefMpt[0]) continue;
        if (prefMptCooldownUntil[ci] && millis() < prefMptCooldownUntil[ci]) {
            Serial.printf("📡 [pref] %s/%s in cooldown (%lus left) — using geographic scan\n",
                          casters[ci].host, casters[ci].prefMpt,
                          (prefMptCooldownUntil[ci] - millis()) / 1000UL);
            continue;
        }
        // SELECT the preferred mount — do NOT connect here. Connecting is
        // exclusively ntrip_loop's job (one connect path, one backoff timer);
        // an inline connect from inside selection bypassed that gate and left
        // the loop's state machine fighting itself. Phase 0 just nominates the
        // mountpoint and returns; the connection block performs the GET on this
        // same loop pass. If the caster answers SOURCETABLE (mount not
        // streaming = base powered off), the STALE handler in ntrip_loop cools
        // this mount down and re-arms selection immediately, which then falls
        // through to the geographic scan below and finds a public base.
        activeCasterIdx = ci;
        strncpy(activeMountpoint, casters[ci].prefMpt, sizeof(activeMountpoint)-1);
        activeMountpoint[sizeof(activeMountpoint)-1] = '\0';
        activeMountDistKm  = -1.0f;   // unknown until RTCM 1005/1006 arrives
        activeMountCarrier = 2;       // own base — LG290P is multi-band
        activeMountVRS     = false;
        activeBestDistSq   = -1.0;    // eff=-1 semantics: no scan result outranks it
        activeIsPreferred  = true;
        Serial.printf("📡 [pref] selected %s:%s/%s for direct connect (no source table needed)\n",
                      casters[ci].host, casters[ci].port, activeMountpoint);
        lastRescanTime = millis();
        return true;
    }

    double bestD=1e18, bestLat=NAN, bestLon=NAN; char bestMpt[64]="";
    int bestCi=0; int8_t bestCar=-1; bool bestVRS=false;

    for (int ci=0; ci<castersCount; ci++) {
        if (!casters[ci].enabled) continue;      // disabled from the dashboard
        char mpt[64]=""; double distSq=1e18, sLat=NAN, sLon=NAN;
        int8_t car=-1; bool vrs=false;
        // Private / own-base casters (those with a configured preferred
        // mountpoint, e.g. Centipede→RCX1) are DIRECT-CONNECT ONLY and were
        // already handled in Phase 0. Their public source table lists only other
        // people's far-away bases, so scanning it geographically just produces
        // "nearest base too far" noise for a mount we would never use. RCX1 is
        // private/undeclared and never appears in the table anyway (see
        // config.cpp). Only truly public casters (prefMpt=="") are scanned here.
        if (casters[ci].prefMpt[0]) {
            Serial.printf("📡 [%d/%d] %s — private base, direct-only (skipping geographic scan)\n",
                          ci+1, castersCount, casters[ci].host);
            continue;
        }
        Serial.printf("📡 [%d/%d] %s ", ci+1, castersCount, casters[ci].host);
        if (scanCaster(ci, g, mpt, &distSq, &sLat, &sLon, &car, &vrs)) {
            float km=distanceKm(g.latitude,g.longitude,sLat,sLon);
            bool preferred = (casters[ci].prefMpt[0] &&
                              strncmp(mpt, casters[ci].prefMpt, 63)==0);
            Serial.printf("-> %s (%.1fkm)%s\n", mpt, km, preferred?" ⭐":"");
            if (distSq < bestD) { bestD=distSq; bestCi=ci; bestLat=sLat; bestLon=sLon;
                                   bestCar=car; bestVRS=vrs; strncpy(bestMpt,mpt,63); }
                                   
            // Quick-exit ONLY if the preferred mountpoint is explicitly found.
            // Geographic quick-exit has been removed to ensure global minimum distance is found across all casters.
            // NOTE: distSq now carries the TRUE distance² (the "record act, not 0"
            // edit), so the old test (preferred && distSq <= 0) could never fire —
            // the preferred mount was silently competing on raw distance against
            // every other caster. The `preferred` bool alone is the correct test:
            // scanCaster only returns the preferred mount after its ≤NTRIP_MAX_
            // BASELINE_KM imposter check passed.
            if (preferred) {
                activeCasterIdx=ci; activeBestDistSq=distSq;
                activeMountLat=sLat; activeMountLon=sLon;
                activeMountDistKm=km; activeMountCarrier=car; activeMountVRS=vrs;
                activeIsPreferred=true;
                strncpy(activeMountpoint,mpt,sizeof(activeMountpoint)-1);
                activeMountpoint[sizeof(activeMountpoint)-1]='\0';
                lastRescanTime=millis(); return true; 
            }
        } else { Serial.println("no response"); }
    }
    if (bestMpt[0]) {
        float finalKm = distanceKm(g.latitude, g.longitude, bestLat, bestLon);
        
        // If the best station is too far (and we know it isn't RCX1 because 
        // RCX1 would have quick-exited inside the loop above).
        if (finalKm > (float)NTRIP_MAX_BASELINE_KM) {
            Serial.printf("❌ Nearest station %s is too far (%.1fkm). Aborting connection.\n", bestMpt, finalKm);
            return false;
        }

        activeCasterIdx=bestCi; activeBestDistSq=bestD;
        activeMountLat=bestLat; activeMountLon=bestLon;
        activeMountCarrier=bestCar; activeMountVRS=bestVRS;
        activeMountDistKm=finalKm;
        activeIsPreferred=false;
        strncpy(activeMountpoint,bestMpt,sizeof(activeMountpoint)-1);
        activeMountpoint[sizeof(activeMountpoint)-1]='\0';
        Serial.printf("📡 Best: %s (%.1fkm)\n", activeMountpoint, activeMountDistKm);
        lastRescanTime=millis(); return true;
    }
    Serial.println("❌ No casters responded with usable stations"); return false;
}

// ── Preferred-mount liveness probe ────────────────────────────────────────────
// Checks whether a caster's preferred mountpoint (own base) is currently
// streaming, WITHOUT disturbing the active NTRIP session: a separate short-lived
// WiFiClient sends GET /<mount> and only the first response line is examined.
//   "ICY 200" / "...200..."   → mount is live (base online and accepted by caster)
//   "SOURCETABLE ..."         → mount not streaming (base offline)
//   TCP fail / timeout        → caster unreachable
// Used while connected to a public fallback mount so the rover migrates back to
// RCX1 within minutes of the base coming online — the source table can never
// tell us this, because a private/undeclared mount is never listed.
static bool probePreferredLive(int ci) {
    if (ci < 0 || ci >= castersCount || !casters[ci].prefMpt[0]) return false;
    if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) < 30000) return false;

    WiFiClient c;
    if (!c.connect(casters[ci].host, atoi(casters[ci].port))) return false;

    // Build credentials for THIS caster without touching activeCasterIdx (the
    // active session may belong to a different caster).
    int saved = activeCasterIdx;
    activeCasterIdx = ci;
    String auth = ntripBase64();
    activeCasterIdx = saved;

    c.printf("GET /%s HTTP/1.0\r\nUser-Agent: NTRIP RCX_Datalogger/1.0\r\n"
             "Ntrip-Version: Ntrip/1.0\r\nAuthorization: Basic %s\r\n\r\n",
             casters[ci].prefMpt, auth.c_str());
    unsigned long to = millis() + 5000;
    while (!c.available() && millis() < to) delay(10);
    bool live = false;
    if (c.available()) {
        String first = c.readStringUntil('\n');
        live = (first.indexOf("SOURCETABLE") < 0) && (first.indexOf("200") >= 0);
    }
    c.stop();
    return live;
}

static void rescanForBetter(const GnssData& g) {
    if (!ntripGoodFix(g)) return;
    Serial.println("🔄 Background rescan...");
    // Remember which caster we are actually connected to. scanCaster() clobbers
    // the global activeCasterIdx as a side effect (see note in scanCaster), so if
    // we decide NOT to switch we must restore it — otherwise activeCasterIdx ends
    // up pointing at the last caster we probed while activeMountpoint still belongs
    // to the connected caster, and the NEXT reconnect would use the wrong host +
    // credentials with the wrong mountpoint (repeated rejects → wasted reconnects
    // → ban risk). This is the bug this save/restore prevents.
    int connectedCasterIdx = activeCasterIdx;

    double bestD=1e18, bLat=NAN, bLon=NAN; char bestMpt[64]="";
    int bestCi=activeCasterIdx; int8_t bestCar=-1; bool bestVRS=false;
    for (int ci=0; ci<castersCount; ci++) {
        if (!casters[ci].enabled) continue;      // disabled from the dashboard
        char mpt[64]=""; double dSq=1e18, sLat=NAN, sLon=NAN; int8_t car=-1; bool vrs=false;
        if (scanCaster(ci,g,mpt,&dSq,&sLat,&sLon,&car,&vrs) && dSq<bestD) {
            bestD=dSq; bestCi=ci; bLat=sLat; bLon=sLon; bestCar=car; bestVRS=vrs;
            strncpy(bestMpt,mpt,63); bestMpt[63]='\0'; }
    }
    if (bestMpt[0] && bestD < activeBestDistSq * NTRIP_IMPROVE_THRESHOLD) {
        Serial.printf("📡 Switching to closer: %s\n", bestMpt);
        ntripClient.stop();
        activeCasterIdx=bestCi; activeBestDistSq=bestD;
        activeMountLat=bLat; activeMountLon=bLon; activeMountCarrier=bestCar; activeMountVRS=bestVRS;
        activeMountDistKm=distanceKm(g.latitude,g.longitude,bLat,bLon);
        activeIsPreferred=false;
        strncpy(activeMountpoint,bestMpt,sizeof(activeMountpoint)-1);
        activeMountpoint[sizeof(activeMountpoint)-1]='\0';
        if (xSemaphoreTake(dataMutex,portMAX_DELAY)) { status.ntripConnected=false; xSemaphoreGive(dataMutex); }
    } else {
        // No worthwhile improvement — stay put. Restore the global the scan loop
        // clobbered so the connected caster's index remains correct.
        activeCasterIdx = connectedCasterIdx;
    }
    lastRescanTime=millis();
}

static NtripConnResult connectNtrip(const GnssData& g) {
    if (ntripClient.connected()) return NTRIP_OK;
    if (WiFi.status() != WL_CONNECTED) return NTRIP_FAIL;
    if (activeMountDistKm >= 0.0f)
        Serial.printf("🔗 NTRIP %s:%s/%s (%.1f km, carrier L%d%s)\n",
                      casters[activeCasterIdx].host, casters[activeCasterIdx].port,
                      activeMountpoint, activeMountDistKm,
                      (int)activeMountCarrier, activeMountVRS ? ", VRS" : "");
    else
        Serial.printf("🔗 NTRIP %s:%s/%s (baseline TBD from RTCM 1005/1006, carrier L%d%s)\n",
                      casters[activeCasterIdx].host, casters[activeCasterIdx].port,
                      activeMountpoint,
                      (int)activeMountCarrier, activeMountVRS ? ", VRS" : "");
    // RTK baseline sanity — single-base RTK degrades hard past ~30-50 km and
    // rarely fixes past ~70 km (atmospheric decorrelation). If the scan only
    // found a distant mount, corrections will flow but RTK will never engage —
    // which looks exactly like "connected but no RTK". Flag it loudly.
    if (!activeMountVRS && activeMountDistKm > 50.0f)
        Serial.printf("⚠️  NTRIP: baseline %.0f km is long for single-base RTK — expect FLOAT at best\n",
                      activeMountDistKm);
    if (!ntripClient.connect(casters[activeCasterIdx].host,
                             atoi(casters[activeCasterIdx].port))) {
        Serial.printf("⚠️  NTRIP: TCP connect to %s:%s failed (host down / port blocked / IP banned?)\n",
                      casters[activeCasterIdx].host, casters[activeCasterIdx].port);
        return NTRIP_FAIL;
    }

    // Include Ntrip-GGA immediately so NTRIP casters 
    // that require rover position before streaming don't wait for the 10s
    // periodic GGA send. gnss_buildGGA returns "" when GPS not yet valid.
    String gga = gnss_buildGGA(g);
    String ggaHdr = gga.length() ? "Ntrip-GGA: " + gga : "";  // gga already ends \r\n
    // DIAGNOSTIC: many mounts (VRS especially) send NO RTCM until they receive a
    // valid rover GGA. If we connect during a fix dropout, gga is empty and the
    // mount stays silent — looks identical to "connected but no data". Make that
    // visible so the debug log distinguishes it from a network stall.
    Serial.printf("📡 NTRIP connect %s | GGA %s (valid=%d, %d sats)\n",
                  activeMountpoint,
                  gga.length() ? "UPLOADED" : "EMPTY — mount may withhold RTCM",
                  g.valid ? 1 : 0, g.numSV);

    ntripClient.printf("GET /%s HTTP/1.0\r\nUser-Agent: NTRIP RCX_Datalogger/1.0\r\n"
                       "Ntrip-Version: Ntrip/1.0\r\nAuthorization: Basic %s\r\n%s\r\n",
                       activeMountpoint, ntripBase64().c_str(), ggaHdr.c_str());

    unsigned long to=millis()+5000;
    while (!ntripClient.available() && millis()<to) delay(10);
    String resp=ntripClient.readStringUntil('\n');
    if (resp.indexOf("SOURCETABLE")>=0) {
        ntripClient.stop(); activeMountpoint[0]='\0'; activeBestDistSq=1e18;
        return NTRIP_STALE; }
    if (resp.indexOf("200")<0) {
        Serial.printf("⚠️  NTRIP rejected: %s\n", resp.c_str());
        ntripClient.stop(); return NTRIP_FAIL; }

    // Drain HTTP response headers — wait through TCP packet gaps rather than
    // exiting on available()==0. The old loop bailed between packets, leaving
    // header bytes in the stream that then corrupted the RTCM data to the LG290P.
    //
    // ⚠️  ONLY for HTTP-style responses. NTRIP rev1 casters (rtk2go/SNIP when the
    // client requests Ntrip/1.0, BKG, most public casters) reply with a bare
    // "ICY 200 OK\r\n" and binary RTCM follows IMMEDIATELY — no header block, no
    // blank line. Running the drain on an ICY response reads readStringUntil('\n')
    // on binary RTCM for up to 3 s, eating corrections and starting the forward
    // mid-frame. (The LG290P resyncs on the 0xD3 preamble, but first corrections
    // are delayed and the first RTCM byte counter looks broken.)
    if (!resp.startsWith("ICY")) {
        unsigned long hdrTo = millis() + 3000;
        while (millis() < hdrTo) {
            while (!ntripClient.available() && millis() < hdrTo) delay(5);
            if (!ntripClient.available()) break;
            String l = ntripClient.readStringUntil('\n');
            l.trim();
            if (l.length() == 0) break;  // blank line = end of HTTP headers
        }
    }

    // Handshake is done — everything past this point is the steady-state
    // session, dominated by the periodic GGA keep-alive print() below. Bound
    // its blocking duration now (not any earlier): the connect/header-drain
    // reads above rely on the WiFiClient's original default timeout to wait
    // through normal TCP packet gaps, and shortening that mid-handshake would
    // risk re-introducing the truncated-header bug the drain loop above was
    // written to fix. NTRIP_CLIENT_IO_TIMEOUT_MS exists for a different
    // problem: the ESP32-S3 has ONE 2.4 GHz radio shared by WiFi and BLE via
    // time-sliced coexistence, so a keep-alive print() left to block for
    // seconds on a lossy mobile hotspot can cost a scheduled BLE connection
    // event — which reads as a dropped 20 Hz BLE frame with no BLE-side cause
    // at all. Re-applied on every fresh connect (not just once at boot) so a
    // reconnect after ntripClient.stop() is never left on the wider default.
    ntripClient.setTimeout(NTRIP_CLIENT_IO_TIMEOUT_MS);

    Serial.printf("✅ NTRIP: %s%s\n", activeMountpoint, gga.length() ? " (GGA sent)" : "");
    lastConnectOkMs = millis();   // mark session start for quick-drop detection
    lastRtcmByteMs  = millis();   // arm the no-data watchdog for this session
    baselineSnifferReset();       // re-verify baseline each session
    if (xSemaphoreTake(dataMutex,portMAX_DELAY)) {
        status.ntripConnected=true;
        status.ntripCarrier=activeMountCarrier;
        status.ntripVRS=activeMountVRS;
        // strncpy with sizeof() won't terminate if the source fills the buffer
        // exactly; terminate explicitly so the dashboard never reads a runaway
        // string. (Hosts/mountpoints are short today, but config is user-edited.)
        strncpy(status.mountpoint,activeMountpoint,sizeof(status.mountpoint)-1);
        status.mountpoint[sizeof(status.mountpoint)-1]='\0';
        strncpy(status.casterHost,casters[activeCasterIdx].host,sizeof(status.casterHost)-1);
        status.casterHost[sizeof(status.casterHost)-1]='\0';
        status.ntripDistanceKm=activeMountDistKm;
        xSemaphoreGive(dataMutex); }
    return NTRIP_OK;
}

// ── Public API ────────────────────────────────────────────────────────────────
void ntrip_init() {
    loadCasters();   // build runtime table (compile-time + NVS) before any scan/connect
    activeMountpoint[0]='\0'; activeBestDistSq=1e18;
    ntripFailCount=0; lastConnectAttempt=0; lastMountpointSearch=0;
    activeIsPreferred=false; silentSessionCount=0; lastRtcmByteMs=0;
    for (int i=0;i<NTRIP_MAX_CASTERS;i++) prefMptCooldownUntil[i]=0;
    baselineSnifferReset();
}

// Persist a new caster to NVS and rebuild the runtime table so it takes effect
// immediately (no reboot). Returns false if the table is already full
// (NTRIP_MAX_CASTERS) or the host is empty. Intended to be called from a future
// provisioning path (e.g. a webserver form); safe to call at runtime because the
// next ntrip_loop() simply sees the larger castersCount.
bool ntrip_saveCaster(const char* host, const char* port,
                      const char* user, const char* pass, const char* prefMpt) {
    if (!host || !host[0]) return false;
    if (castersCount >= NTRIP_MAX_CASTERS) {
        Serial.println("⚠️  NTRIP: caster table full — cannot add");
        return false;
    }
    Preferences p;
    if (!p.begin("rcx_ntrip", false)) return false;   // read-write
    int n = p.getInt("n", 0);
    char key[8];
    snprintf(key,sizeof(key),"h%d",n); p.putString(key, host);
    snprintf(key,sizeof(key),"p%d",n); p.putString(key, (port && port[0]) ? port : "2101");
    snprintf(key,sizeof(key),"u%d",n); p.putString(key, user    ? user    : "");
    snprintf(key,sizeof(key),"w%d",n); p.putString(key, pass    ? pass    : "");
    snprintf(key,sizeof(key),"m%d",n); p.putString(key, prefMpt ? prefMpt : "");
    p.putInt("n", n + 1);
    p.end();
    // Hand the rebuild to ntrip_loop() rather than doing it here. This function
    // runs on the web server's task, and rebuilding the table underneath a scan
    // in progress would leave castersCount and activeCasterIdx disagreeing for
    // the remainder of that pass. The new caster is picked up on the next loop
    // iteration, which is immediate at the loop's service rate.
    castersReloadPending = true;
    Serial.printf("📡 NTRIP: caster '%s' saved to NVS\n", host);
    return true;
}

// ── Caster list management (called from the web server task) ─────────────────
// Every entry point here only writes NVS and raises castersReloadPending; none
// of them touch the runtime table directly. See castersReloadPending.

int ntrip_casterCount() { return castersCount; }

bool ntrip_casterInfo(int i, NtripCasterInfo* out) {
    if (i < 0 || i >= castersCount || !out) return false;
    out->host      = casters[i].host;
    out->port      = casters[i].port;
    out->mount     = casters[i].prefMpt;
    out->enabled   = casters[i].enabled;
    out->isDefault = casters[i].isDefault;
    out->active    = ntripClient.connected() && (i == activeCasterIdx);
    return true;
}

bool ntrip_setCasterEnabled(int i, bool enabled) {
    if (i < 0 || i >= castersCount) return false;
    if (casters[i].enabled == enabled) return true;

    Preferences p;
    if (!p.begin("rcx_ntrip", false)) return false;
    char key[8];
    snprintf(key, sizeof(key), casters[i].isDefault ? "d%d" : "e%d", casters[i].srcIdx);
    p.putBool(key, enabled);
    p.end();

    casters[i].enabled = enabled;
    Serial.printf("📡 NTRIP: %s %s\n", enabled ? "enabled" : "disabled", casters[i].host);
    // Only disturb the live session if the caster being switched off is the one
    // currently feeding corrections. Disabling an idle caster should cost nothing.
    if (!enabled && i == activeCasterIdx && ntripClient.connected()) castersReloadPending = true;
    return true;
}

bool ntrip_removeCaster(int i) {
    // Compile-time defaults have no NVS record to erase and would reappear on the
    // next boot, so removal is refused rather than silently failing. Disable them.
    if (i < 0 || i >= castersCount || casters[i].isDefault) return false;

    const int nvsIdx = casters[i].srcIdx;
    Preferences p;
    if (!p.begin("rcx_ntrip", false)) return false;
    const int n = p.getInt("n", 0);
    if (nvsIdx < 0 || nvsIdx >= n) { p.end(); return false; }

    // Read the surviving entries, then rewrite the list compactly. Rewriting is
    // necessary rather than blanking one slot: every key is indexed by position,
    // so leaving a hole would leave each following entry's enable flag attached
    // to the wrong caster.
    struct Row { String h, prt, u, w, m; bool en; };
    Row rows[NTRIP_MAX_CASTERS];
    int kept = 0;
    for (int k = 0; k < n && kept < NTRIP_MAX_CASTERS; k++) {
        if (k == nvsIdx) continue;
        char key[8];
        snprintf(key, sizeof(key), "h%d", k); rows[kept].h = p.getString(key, "");
        if (rows[kept].h.length() == 0) continue;
        snprintf(key, sizeof(key), "p%d", k); rows[kept].prt = p.getString(key, "");
        snprintf(key, sizeof(key), "u%d", k); rows[kept].u   = p.getString(key, "");
        snprintf(key, sizeof(key), "w%d", k); rows[kept].w   = p.getString(key, "");
        snprintf(key, sizeof(key), "m%d", k); rows[kept].m   = p.getString(key, "");
        snprintf(key, sizeof(key), "e%d", k); rows[kept].en  = p.getBool(key, true);
        kept++;
    }

    // Default enable flags share this namespace and must survive the rewrite.
    bool defEnabled[NTRIP_MAX_CASTERS];
    for (int d = 0; d < ntripCasterCount && d < NTRIP_MAX_CASTERS; d++) {
        char key[8];
        snprintf(key, sizeof(key), "d%d", d);
        defEnabled[d] = p.getBool(key, true);
    }

    p.clear();
    p.putInt("n", kept);
    for (int k = 0; k < kept; k++) {
        char key[8];
        snprintf(key, sizeof(key), "h%d", k); p.putString(key, rows[k].h);
        snprintf(key, sizeof(key), "p%d", k); p.putString(key, rows[k].prt);
        snprintf(key, sizeof(key), "u%d", k); p.putString(key, rows[k].u);
        snprintf(key, sizeof(key), "w%d", k); p.putString(key, rows[k].w);
        snprintf(key, sizeof(key), "m%d", k); p.putString(key, rows[k].m);
        snprintf(key, sizeof(key), "e%d", k); p.putBool(key, rows[k].en);
    }
    for (int d = 0; d < ntripCasterCount && d < NTRIP_MAX_CASTERS; d++) {
        char key[8];
        snprintf(key, sizeof(key), "d%d", d); p.putBool(key, defEnabled[d]);
    }
    p.end();

    Serial.printf("📡 NTRIP: caster '%s' removed\n", casters[i].host);
    castersReloadPending = true;
    return true;
}


bool ntrip_connected()       { return ntripClient.connected(); }
const char* ntrip_mountpoint() { return activeMountpoint; }

void ntrip_loop(HardwareSerial& gpsSerial, const GnssData& g) {
    // ── Apply pending caster-list changes ─────────────────────────────────
    // Deliberately the first thing in the loop: this is the one point where no
    // scan is in flight and no index into the table is being held. Changing the
    // caster list restarts selection from scratch, which is the honest response
    // to the operator having changed what may be selected.
    if (castersReloadPending) {
        castersReloadPending = false;
        Serial.println("📡 NTRIP: caster list changed — reselecting");
        loadCasters();
        ntripClient_reset();
    }

    // ── Operator reset (web dashboard) ────────────────────────────────────
    // Clears every timer that can hold NTRIP back — the per-caster 5-minute
    // source-table rate limits, preferred-mount cooldowns, connect backoff and
    // the current session — and forces immediate reselection. This is the
    // recovery lever for a boot where WiFi came up late or associated with an
    // unwanted network: the caster budgets got spent against the wrong (or no)
    // uplink, and without this the operator waits out cooldowns that protect
    // the casters from a loop, not from a deliberate one-shot human action.
    if (operatorResetPending) {
        operatorResetPending = false;
        for (int i = 0; i < NTRIP_MAX_CASTERS; i++) lastCasterFetch[i] = 0;
        ntripClient_reset();          // closes TCP, clears mountpoint, cooldowns,
                                      // backoff and fail count; next selection
                                      // pass fires promptly (failCount==0 path)
        Serial.println("📡 NTRIP: operator reset — timers cleared, reselecting now");
    }

    // ── Connection-drop edge detector ─────────────────────────────────────
    // Detect the moment the TCP session goes from up to down, in ONE place,
    // rather than scattering connected() checks through the function. A drop
    // that happens within NTRIP_RECONNECT_FLOOR_MS of connecting means the
    // mountpoint accepted us then died (half-dead base, caster hiccup); we count
    // that as a failure so the backoff grows past the floor instead of retrying
    // the same bad mount every 30 s indefinitely (which also looks abusive to the
    // caster). A drop after a long healthy session is normal churn — don't penalise.
    bool nowConnected = ntripClient.connected();
    if (wasNtripConnected && !nowConnected) {
        unsigned long upMs = (lastConnectOkMs != 0) ? (millis() - lastConnectOkMs) : 0xFFFFFFFFUL;
        if (upMs < NTRIP_RECONNECT_FLOOR_MS) {
            ntripFailCount++;   // unstable mountpoint — escalate backoff
            Serial.printf("⚠️  NTRIP: dropped after %lus (unstable) — backoff escalates\n",
                          upMs / 1000UL);
        } else {
            Serial.println("⚠️  NTRIP dropped (healthy session ended)");
        }
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            status.ntripConnected = false;
            xSemaphoreGive(dataMutex);
        }
    }
    wasNtripConnected = nowConnected;

    // Refresh the base distance against the LIVE rover position. The baseline sniffer
    // latches off after its one-shot imposter check (baselineChecked), so without this the
    // displayed distance froze at whatever it was when RTCM 1005/1006 first arrived — e.g.
    // stuck at "40 km" while driving toward a base a few km away. The base is stationary and
    // its ECEF was captured then; this is just the cheap distance math re-run, no re-sniff.
    // Throttled to ~1 Hz (position moves metres/sec; sub-second refresh is pointless) and
    // gated on a valid fix so a dropped fix doesn't write a garbage distance.
    if (baseEcefValid && nowConnected && g.valid) {
        static uint32_t lastDistMs = 0;
        if (millis() - lastDistMs > 1000) {
            lastDistMs = millis();
            double rov[3];
            llaToEcef(g.latitude, g.longitude, g.altMSL, rov);
            double dx = baseEcef[0]-rov[0], dy = baseEcef[1]-rov[1], dz = baseEcef[2]-rov[2];
            float km = (float)(sqrt(dx*dx + dy*dy + dz*dz) / 1000.0);
            activeMountDistKm = km;
            if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
                status.ntripDistanceKm = km;
                xSemaphoreGive(dataMutex);
            }
        }
    }

    // ── Mountpoint selection ──────────────────────────────────────────────
    if (activeMountpoint[0]=='\0') {
        if (millis()-lastMountpointSearch >= ntripBackoffMs()) {
            if (!ntripGoodFix(g)) {
                // Waiting for a usable position. Deliberately do NOT stamp
                // lastMountpointSearch: the wait must not consume the search
                // window, so the first selection fires the instant a real fix
                // lands instead of up to a full backoff later. Burning the
                // prompt first-attempt windows on placeholder 0,0 positions
                // was exactly how a late fix used to push RTK acquisition out
                // by minutes. Print at most every 10 s.
                static uint32_t lastWaitMsg = 0;
                if (millis() - lastWaitMsg > 10000) {
                    lastWaitMsg = millis();
                    Serial.println("📡 NTRIP: selection waiting for GPS fix");
                }
            } else {
                lastMountpointSearch=millis();
                bool ok=findNearestMountpoint(g);
                if (ok) ntripFailCount=0;
                else    ntripFailCount++;
            }
        }
    }

    // ── Connection ───────────────────────────────────────────────────────
    if (activeMountpoint[0] && !ntripClient.connected()) {
        if (millis()-lastConnectAttempt >= ntripBackoffMs()) {
            lastConnectAttempt=millis();
            NtripConnResult r=connectNtrip(g);
            if (r==NTRIP_OK) ntripFailCount=0;
            else if (r==NTRIP_STALE) {
                // Caster answered "SOURCETABLE" — the mountpoint isn't streaming.
                // connectNtrip already cleared activeMountpoint. For a PREFERRED
                // mount this is simply "own base is powered off": the caster is
                // healthy, so don't punish the backoff — but DO cool the mount
                // down (else the next selection just re-picks it forever and the
                // geographic scan never runs) and re-arm selection NOW so the
                // scan happens on the next pass instead of after a backoff wait.
                if (activeIsPreferred) {
                    Serial.printf("📡 [pref] %s not streaming on %s — will re-check in %lus; scanning public casters now\n",
                                  casters[activeCasterIdx].prefMpt,
                                  casters[activeCasterIdx].host,
                                  NTRIP_PREF_PROBE_INTERVAL_MS / 1000UL);
                    prefMptCooldownUntil[activeCasterIdx] = millis() + NTRIP_PREF_PROBE_INTERVAL_MS;
                    activeIsPreferred    = false;
                    lastMountpointSearch = 0;   // re-select immediately → scan path
                    lastConnectAttempt   = 0;   // and connect promptly to whatever it finds
                }
                // Non-preferred STALE (a scanned public mount vanished): the
                // normal selection backoff triggers a fresh scan.
            }
            else if (r==NTRIP_FAIL) {
                ntripFailCount++;
                // If the retained mountpoint refuses repeatedly, it may be
                // offline. Drop it so the next tick re-scans for a live one.
                // The scan itself is still rate-limited (5 min) per caster,
                // so this cannot flood the caster.
                if (ntripFailCount >= 5) {
                    Serial.println("⚠️  NTRIP: mountpoint failed 5× — will rescan");
                    if (activeIsPreferred) {
                        // Cool the preferred mount down, or phase-0 selection
                        // re-picks it on the very next pass and the public-caster
                        // scan is starved forever.
                        prefMptCooldownUntil[activeCasterIdx] = millis() + NTRIP_PREF_PROBE_INTERVAL_MS;
                        activeIsPreferred = false;
                    }
                    activeMountpoint[0] = '\0';
                    activeBestDistSq    = 1e18;
                }
            }
        }
    }

    // ── Forward RTCM to LG290P ────────────────────────────────────────────
    if (ntripClient.connected()) {
        // Drain ALL buffered RTCM per pass (bounded). A full multi-constellation
        // MSM epoch arrives as a several-KB burst once per second; a single
        // 512-byte read per 10 ms task tick lets bursts queue in lwIP and the
        // receiver sees corrections late — stale corrections degrade or prevent
        // RTK even though "bytes are flowing". 8×512 B per pass at ~100 Hz task
        // rate is far above any caster's data rate while still bounding the
        // time this task can spend here.
        bool gotBytes = false;
        for (int burst = 0; burst < 8; burst++) {
            int av = ntripClient.available();
            if (av <= 0) break;
            uint8_t buf[512];
            int n = ntripClient.read(buf, min(av, (int)sizeof(buf)));
            if (n <= 0) break;
            gpsSerial.write(buf, n);
            rtcmBytesTotal += n;
            gotBytes = true;
            // Establish the true baseline from the stream's own RTCM 1005/1006
            // (idles via one branch once baselineChecked is true).
            if (!baselineChecked) baselineSnifferFeed(buf, (size_t)n, g);
        }
        if (gotBytes) { lastRtcmByteMs = millis(); silentSessionCount = 0; }

        // ── Baseline verdict ─────────────────────────────────────────────
        // The sniffer found RTCM 1005/1006 placing the base beyond the cap:
        // wrong/imposter mount. Drop it, cool the preferred mount down so the
        // geographic fallback isn't immediately preempted by phase-0 again.
        if (baselineTooFar) {
            baselineTooFar = false;
            ntripClient.stop();
            if (activeIsPreferred) prefMptCooldownUntil[activeCasterIdx] = millis() + NTRIP_PREF_COOLDOWN_MS;
            activeMountpoint[0] = '\0';
            activeBestDistSq    = 1e18;
            activeIsPreferred   = false;
            ntripFailCount++;
            if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
                status.ntripConnected = false;
                xSemaphoreGive(dataMutex);
            }
            wasNtripConnected = false;   // we handled the drop; suppress edge detector
            return;
        }

        // ── No-data watchdog: "connected" must mean "functioning" ────────
        // A caster can accept the GET (ICY 200 OK) and then send nothing —
        // historically this looked healthy ("connected: yes") while the LG290P
        // starved. Any healthy mount sends corrections every second; a silence
        // of NTRIP_DATA_TIMEOUT_MS means the session is dead weight. Drop it.
        // Two silent sessions in a row condemn the mountpoint itself: clear it
        // (and cool down a preferred mount) so selection falls through to the
        // next caster — e.g. RCX1 accepted-but-silent → rtk2go's nearest base.
        if (millis() - lastRtcmByteMs > NTRIP_DATA_TIMEOUT_MS) {
            silentSessionCount++;
            // The GGA-upload state at THIS moment is the key clue: a VRS/single-base
            // mount that needs position will stay silent if our fix is invalid now.
            Serial.printf("⚠️  NTRIP: %s accepted the connection but sent no RTCM for %lus — dropping (silent session %u) [rover fix valid=%d — VRS mounts need a valid GGA]\n",
                          activeMountpoint, (millis() - lastRtcmByteMs) / 1000UL,
                          (unsigned)silentSessionCount, g.valid ? 1 : 0);
            ntripClient.stop();
            ntripFailCount++;
            if (silentSessionCount >= 2) {
                Serial.printf("⚠️  NTRIP: %s silent twice in a row — abandoning mountpoint\n", activeMountpoint);
                if (activeIsPreferred) prefMptCooldownUntil[activeCasterIdx] = millis() + NTRIP_PREF_COOLDOWN_MS;
                activeMountpoint[0] = '\0';
                activeBestDistSq    = 1e18;
                activeIsPreferred   = false;
                silentSessionCount  = 0;
            }
            if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
                status.ntripConnected = false;
                xSemaphoreGive(dataMutex);
            }
            wasNtripConnected = false;   // we handled the drop; suppress edge detector
            return;
        }

        // GGA feedback — keeps VRS and position-gated casters happy.
        // VRS mounts synthesize corrections for the position in our LAST GGA. At
        // a fixed 10 s cadence, a moving car drives hundreds of metres past that
        // virtual base between updates, so the effective baseline grows and the
        // solution sags toward FLOAT until the next GGA snaps the base back. So:
        // when the active mount is a VRS AND we're moving, feed GGA every ~1 s.
        // Single-base mounts (RCX1, Centipede/rtk2go physical bases) ignore GGA
        // content — the base is fixed — so they keep the 10 s keep-alive cadence
        // and we don't add needless uplink traffic. Only advance lastGgaSend on a
        // real send; if GPS wasn't valid the attempt skipped and must retry soon.
        unsigned long ggaInterval =
            (activeMountVRS && g.speedKnots > 1.0f) ? 1000UL : 10000UL;
        if (millis()-lastGgaSend > ggaInterval) {
            String gga=gnss_buildGGA(g);
            if (gga.length()) {
                // Timed on purpose: this write is now bounded by
                // NTRIP_CLIENT_IO_TIMEOUT_MS (set once per session in
                // connectNtrip()), but a send that actually hits that bound
                // is exactly the event worth seeing in the log — it is the
                // field-provable link between a WiFi radio stall and a
                // dropped BLE frame, instead of an inference after the fact.
                uint32_t ggaT0 = millis();
                ntripClient.print(gga);
                uint32_t ggaBlockedMs = millis() - ggaT0;
                if (ggaBlockedMs > 20) {
                    Serial.printf("⚠️  NTRIP: GGA keep-alive send blocked %lums "
                                  "(WiFi/BLE radio contention window)\n",
                                  (unsigned long)ggaBlockedMs);
                }
                lastGgaSend=millis();   // advance ONLY on successful send
            }
        }

        // ── Migrate back to our own base when it comes online ────────────
        // While riding a public fallback mount, probe the preferred mount(s)
        // every NTRIP_PREF_PROBE_INTERVAL_MS (at standstill only — the switch
        // interrupts the RTCM stream for a couple of seconds). A private mount
        // never appears in any source table, so this probe is the ONLY way to
        // notice the base powering up after the rover.
        static unsigned long lastPrefProbe = 0;
        if (!activeIsPreferred &&
            millis()-lastPrefProbe > NTRIP_PREF_PROBE_INTERVAL_MS &&
            g.valid && g.speedKnots < NTRIP_RESCAN_MAX_SPEED_KNOTS) {
            lastPrefProbe = millis();
            for (int ci = 0; ci < castersCount; ci++) {
                if (!casters[ci].enabled) continue;   // disabled from the dashboard
                if (!casters[ci].prefMpt[0]) continue;
                if (prefMptCooldownUntil[ci] && millis() < prefMptCooldownUntil[ci]) continue;
                if (probePreferredLive(ci)) {
                    Serial.printf("📡 NTRIP: preferred mount %s is now live — switching from %s\n",
                                  casters[ci].prefMpt, activeMountpoint);
                    ntripClient.stop();
                    activeCasterIdx = ci;
                    strncpy(activeMountpoint, casters[ci].prefMpt, sizeof(activeMountpoint)-1);
                    activeMountpoint[sizeof(activeMountpoint)-1] = '\0';
                    activeMountDistKm  = -1.0f;
                    activeMountCarrier = 2;
                    activeMountVRS     = false;
                    activeBestDistSq   = -1.0;
                    activeIsPreferred  = true;
                    lastConnectAttempt = 0;      // reconnect promptly on next pass
                    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
                        status.ntripConnected = false;
                        xSemaphoreGive(dataMutex);
                    }
                    wasNtripConnected = false;   // deliberate switch, not a failure
                    return;
                }
            }
        }

        // Background rescan for a closer base — ONLY while stopped, and ONLY
        // when riding a geographically-selected public mount. Never rescan away
        // from our own preferred base: no source-table result can outrank it.
        if (!activeIsPreferred &&
            millis()-lastRescanTime > NTRIP_RESCAN_INTERVAL_MS &&
            g.valid && g.speedKnots < NTRIP_RESCAN_MAX_SPEED_KNOTS) {
            rescanForBetter(g);
        }
    }

    // ── RTCM diagnostic log (every 5 s) ───────────────────────────────────
    // This single line diagnoses the whole corrections chain at a glance:
    //   bytes>0 + RTK=0 persisting  → receiver-side: baseline too long,
    //       mountpoint format/constellation mismatch, or receiver RTK config.
    //   bytes>0 + RTK=1/2           → chain healthy.
    //   "no bytes" while connected  → caster-side: mount silent, VRS waiting
    //       for GGA, or TLS-only (NTRIPS) endpoint reached over plain TCP.
    if (millis()-lastRtcmLog > 5000) {
        uint32_t nb=rtcmBytesTotal-rtcmBytesLast; rtcmBytesLast=rtcmBytesTotal;
        lastRtcmLog=millis();
        if (nb>0) Serial.printf("📡 RTCM: %u B/5s (total %u) | RTK=%s hAcc=%.3fm | %s %.1fkm L%d%s\n",
                                nb, rtcmBytesTotal,
                                g.rtkType==2 ? "FIXED" : g.rtkType==1 ? "FLOAT" : "none",
                                g.hAccM, activeMountpoint, activeMountDistKm,
                                (int)activeMountCarrier, activeMountVRS ? " VRS" : "");
        else if (ntripClient.connected())
                  Serial.println("⚠️  RTCM: connected but no bytes — mount silent / VRS awaiting GGA / TLS-only endpoint?");
        else if (activeMountpoint[0]) {
            // Mountpoint chosen but not connected: show when the next connect
            // attempt fires so a silent stretch is self-explanatory.
            unsigned long bo  = ntripBackoffMs();
            unsigned long el  = millis() - lastConnectAttempt;
            unsigned long rem = (el >= bo) ? 0 : (bo - el) / 1000UL;
            Serial.printf("⚠️  RTCM: no bytes — not connected to %s; next connect attempt in %lus (fail #%d)\n",
                          activeMountpoint, rem, ntripFailCount);
        } else {
            // No mountpoint at all: show when the next selection (phase-0 direct
            // + source-table scan) fires, and whether GPS is gating it.
            unsigned long bo  = ntripBackoffMs();
            unsigned long el  = millis() - lastMountpointSearch;
            unsigned long rem = (el >= bo) ? 0 : (bo - el) / 1000UL;
            Serial.printf("⚠️  RTCM: no bytes — no mountpoint; next selection in %lus (fail #%d%s)\n",
                          rem, ntripFailCount, ntripGoodFix(g) ? "" : ", waiting for GPS fix");
        }
    }
}


// ── ntripClient_reset ─────────────────────────────────────────────────────
// Hard-resets all NTRIP state.  Call when WiFi drops/reconnects so the
// client performs a fresh mountpoint scan on the next ntrip_loop() tick.
void ntripClient_reset() {
    ntripClient.stop();
    activeMountpoint[0]   = '\0';
    activeCasterIdx       = 0;
    activeBestDistSq      = 1e18;
    activeMountLat        = NAN;
    activeMountLon        = NAN;
    activeMountDistKm     = -1.0f;
    activeMountCarrier    = -1;
    activeMountVRS        = false;
    lastMountpointSearch  = 0;
    lastConnectAttempt    = 0;
    lastGgaSend           = 0;
    lastRescanTime        = 0;
    ntripFailCount        = 0;
    activeIsPreferred     = false;
    silentSessionCount    = 0;
    lastRtcmByteMs        = 0;
    for (int i = 0; i < NTRIP_MAX_CASTERS; i++) prefMptCooldownUntil[i] = 0;
    baselineSnifferReset();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        status.ntripConnected = false;
        status.mountpoint[0]  = '\0';
        xSemaphoreGive(dataMutex);
    }
    Serial.println("🔄 NTRIP: client reset");
}

// Soft reset for a transient WiFi drop. Keeps the mountpoint so the reconnect
// is a single GET rather than a source-table scan; clears the backoff so the
// first attempt after WiFi returns is prompt. Does NOT touch lastCasterFetch[],
// so the 5-minute source-table rate limit still protects the caster if a
// genuine rescan becomes necessary later (e.g. mountpoint went stale).
// Raise the operator-reset flag. Callable from any task (web handler): the
// actual state mutation happens inside ntrip_loop() on wifiNtripTask, which
// owns all NTRIP state. Returns immediately.
void ntrip_requestReset() {
    operatorResetPending = true;
}

void ntrip_onWifiLost() {
    ntripClient.stop();
    lastConnectAttempt = 0;   // allow one immediate retry when WiFi is back
    ntripFailCount     = 0;   // prior failures were WiFi's fault, not the caster's
    lastGgaSend        = 0;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        status.ntripConnected = false;
        xSemaphoreGive(dataMutex);
    }
    Serial.printf("🔄 NTRIP: WiFi lost — TCP closed, mountpoint '%s' retained\n",
                  activeMountpoint[0] ? activeMountpoint : "(none)");
}
