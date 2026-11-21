/*
 * ntrip.cpp — NTRIP client
 * =========================
 * Refactored from RaceBox_LG290P.ino.  Logic is identical; code is split
 * into functions callable from wifiNtripTask without the monolithic .ino context.
 */

#include "ntrip.h"
#include "config.h"
#include "types.h"
#include "breadcrumb.h"
#include "gnss.h"
#include "sd_log.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <math.h>
#include <errno.h>
#include <esp_heap_caps.h>   // heap_caps_get_free_size (low-heap scan guard)
#include <lwip/sockets.h>     // select() — non-blocking writability test for the GGA keep-alive
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
static volatile bool castersRebuilding    = false;   // the table is being rebuilt (the request flag is already cleared)

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

// A caster addressed by a private IPv4 literal (10/8, 172.16/12, 192.168/16) can
// only be reached on the network that owns that subnet. The compiled-in base-AP
// caster (192.168.4.1) is the case this exists for: on the field hotspot or any
// other network the address is unroutable or belongs to someone else's router,
// and every attempt costs a connect timeout plus a 30-60 s backoff step — five of
// them before selection moved on, which held a fresh unit's first corrections
// back by ~4 minutes. Hostnames and public literals are always eligible; DNS and
// the caster decide those. On the base's own AP the station address is
// 192.168.4.x/24, so the base caster passes and is tried first as before.
static bool casterOnThisNetwork(int ci) {
    unsigned o[4]; char tail;
    if (sscanf(casters[ci].host, "%u.%u.%u.%u%c", &o[0], &o[1], &o[2], &o[3], &tail) != 4 ||
        o[0] > 255 || o[1] > 255 || o[2] > 255 || o[3] > 255)
        return true;                                       // hostname
    const bool priv = o[0] == 10 || (o[0] == 172 && (o[1] & 0xF0) == 16) ||
                      (o[0] == 192 && o[1] == 168);
    if (!priv) return true;
    if (WiFi.status() != WL_CONNECTED) return false;
    const IPAddress me = WiFi.localIP(), mask = WiFi.subnetMask();
    for (int k = 0; k < 4; k++)
        if (((uint8_t)o[k] & mask[k]) != (me[k] & mask[k])) return false;
    return true;
}

// A caster is dialled only while the station holds a WiFi address. WL_CONNECTED means associated AND
// holding an IP, so this is false during a drop, a reconnect sweep and before the first join. A dial
// attempted without a link fails at the TCP layer exactly like a banned or down host, and spending
// failure counts, cooldowns and host blocks on it punishes casters for the station's own outage.
static bool linkUp() { return WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0); }

// ── Internet gate ────────────────────────────────────────────────────────────
// Is the internet reachable at all, judged without touching any caster? A bare TCP connect to two
// public resolvers by address (no DNS lookup, so a cached name cannot mask an outage).
static bool internetReachable() {
    static const IPAddress anchors[2] = {IPAddress(1, 1, 1, 1), IPAddress(8, 8, 8, 8)};
    for (const IPAddress& a : anchors) {
        WiFiClient c;
        if (c.connect(a, 53, 1500)) { c.stop(); return true; }
    }
    return false;
}

// A caster whose host is a private-range address literal lives on the local network (the base's own
// access point, a LAN caster): the internet has no bearing on whether it can be reached.
static bool casterIsPrivateLiteral(int ci) {
    unsigned o[4]; char tail;
    if (sscanf(casters[ci].host, "%u.%u.%u.%u%c", &o[0], &o[1], &o[2], &o[3], &tail) != 4 ||
        o[0] > 255 || o[1] > 255 || o[2] > 255 || o[3] > 255)
        return false;
    return o[0] == 10 || (o[0] == 172 && (o[1] & 0xF0) == 16) || (o[0] == 192 && o[1] == 168);
}

static bool          internetOk       = false;   // a check has passed since the last link/session/dial failure
static unsigned long lastNetCheck     = 0;       // millis() of the last check (0 = none yet since the reset)
static uint8_t       netCheckFails    = 0;       // consecutive failed checks with the link up
static unsigned long lastBypassDial   = 0;
static String        anchorOkSsid;               // the network on which a check last passed
// Once a mount has delivered corrections in this power-on session the unit stays on that caster and mount:
// switching mid-event changes the reference station, and with it the coordinate frame every logged position
// is solved against. While locked, no failure path and no "closer base" search abandons the mount; failures
// only space the retries (the ban-safety schedule in ntripBackoffMs). A power cycle, the operator's reset,
// a changed caster list, a mount found to be beyond the baseline limit, or a vehicle that has left
// the baseline limit's reach releases it.
static bool          mountLocked      = false;
static bool          lockBarred       = false;   // a release by the operator holds until the next session starts
static volatile bool lockReleasePending = false;   // raised by the web task when the operator edits the caster list; consumed by ntrip_loop

static void internetGateReset() { internetOk = false; lastNetCheck = 0; netCheckFails = 0; lastBypassDial = 0; }

// May caster `ci` be dialled now? Needs the WiFi link, and for any caster off the local network a
// passed internet check. Checks are spaced NTRIP_NET_RETRY_MS apart; between them the answer is the
// last one, with nothing sent anywhere. Nothing is counted against any caster when this says no.
static bool dialPermitted(int ci) {
    if (!linkUp()) return false;
    if (ci >= 0 && ci < castersCount && casterIsPrivateLiteral(ci)) return true;
    const unsigned long now = millis();
    // A confirmation is good for one check interval, so a dial never relies on an answer older than
    // that: an idle period between dials is when the internet is most likely to have gone away.
    if (lastNetCheck != 0 && now - lastNetCheck < NTRIP_NET_RETRY_MS) return internetOk;
    lastNetCheck = now | 1UL;
    if (internetReachable()) {
        if (!internetOk) Serial.println("📡 NTRIP: internet confirmed — caster dials enabled");
        internetOk = true; netCheckFails = 0; anchorOkSsid = WiFi.SSID();
        return true;
    }
    internetOk = false;
    if (netCheckFails < 255) netCheckFails++;
    if (netCheckFails == 1 || netCheckFails % 6 == 0)
        Serial.printf("📡 NTRIP: internet not reachable (check %u) — no caster dialled, checking again in %lu s\n",
                      (unsigned)netCheckFails, NTRIP_NET_RETRY_MS / 1000UL);
    // A network whose checks have never once passed may simply filter the public resolvers while the
    // casters work: allow one dial per interval so corrections still arrive. A network whose checks
    // passed before and fail now has lost its internet, and nothing is dialled until they pass again.
    if (netCheckFails >= NTRIP_NET_CHECKS_BEFORE_BYPASS && anchorOkSsid != WiFi.SSID() &&
        (lastBypassDial == 0 || now - lastBypassDial >= NTRIP_NET_BYPASS_DIAL_MS)) {
        lastBypassDial = now | 1UL;
        Serial.println("📡 NTRIP: internet checks have never passed on this network — one caster dial allowed (checks may be filtered here)");
        return true;
    }
    return false;
}

// The gate's last answer without probing: used inside a scan, whose caller has just passed the
// probing check (selectionPermitted), so a deep call path never carries the probe's frames.
static bool dialPermittedCached(int ci) {
    if (!linkUp()) return false;
    if (ci >= 0 && ci < castersCount && casterIsPrivateLiteral(ci)) return true;
    return internetOk;
}

// May mountpoint selection start? Yes when the link is up and either a caster on the local network
// is configured (it needs no internet) or the internet gate allows a dial.
static bool selectionPermitted() {
    if (!linkUp()) return false;
    for (int ci = 0; ci < castersCount; ci++)
        if (casters[ci].enabled && casterIsPrivateLiteral(ci)) return true;
    return dialPermitted(-1);
}

// Assemble the runtime table. Called once from ntrip_init(). A caster is "valid"
// (and therefore counted) only if it has a non-empty host — this is exactly the
// "increment the caster count if a valid caster is in/added to NVS" rule: blank or
// half-written NVS slots are skipped, never counted.
static void loadCasters() {
    // The table is rebuilt in place and the count published only when it is complete: the web
    // server's task lists the casters from another core, and a count that dipped to the three
    // built-ins mid-rebuild showed an operator a list without the casters they had just added.
    int cnt = 0;

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

    for (int i = 0; i < ntripCasterCount && cnt < NTRIP_MAX_CASTERS; i++) {
        if (!ntripCasters[i][0] || !ntripCasters[i][0][0]) continue;   // skip blank host
        strlcpy(casters[cnt].host,    ntripCasters[i][0], sizeof(casters[0].host));
        strlcpy(casters[cnt].port,    ntripCasters[i][1], sizeof(casters[0].port));
        strlcpy(casters[cnt].user,    ntripCasters[i][2], sizeof(casters[0].user));
        strlcpy(casters[cnt].pass,    ntripCasters[i][3], sizeof(casters[0].pass));
        strlcpy(casters[cnt].prefMpt, ntripPreferredMpt[i], sizeof(casters[0].prefMpt));
        casters[cnt].isDefault = true;
        casters[cnt].srcIdx    = i;
        // Absent flag means enabled: a default the operator has never touched is on.
        char fkey[8];
        snprintf(fkey, sizeof(fkey), "d%d", i);
        casters[cnt].enabled = haveFlags ? flags.getBool(fkey, true) : true;
        cnt++;
    }
    if (haveFlags) flags.end();

    // 2. NVS casters. Stored as count "n" plus per-index keys h<i>/p<i>/u<i>/w<i>/m<i>.
    Preferences p;
    if (p.begin("rcx_ntrip", true)) {              // read-only open
        int n = p.getInt("n", 0);
        for (int i = 0; i < n && cnt < NTRIP_MAX_CASTERS; i++) {
            char key[8], host[48] = "";
            snprintf(key, sizeof(key), "h%d", i); p.getString(key, host, sizeof(host));
            if (host[0] == '\0') continue;          // invalid slot — skip, do NOT count
            strlcpy(casters[cnt].host, host, sizeof(casters[0].host));
            snprintf(key, sizeof(key), "p%d", i); p.getString(key, casters[cnt].port,    sizeof(casters[0].port));
            snprintf(key, sizeof(key), "u%d", i); p.getString(key, casters[cnt].user,    sizeof(casters[0].user));
            snprintf(key, sizeof(key), "w%d", i); p.getString(key, casters[cnt].pass,    sizeof(casters[0].pass));
            snprintf(key, sizeof(key), "m%d", i); p.getString(key, casters[cnt].prefMpt, sizeof(casters[0].prefMpt));
            if (casters[cnt].port[0] == '\0') strlcpy(casters[cnt].port, "2101", sizeof(casters[0].port));
            casters[cnt].isDefault = false;
            casters[cnt].srcIdx    = i;
            snprintf(key, sizeof(key), "e%d", i);
            casters[cnt].enabled = p.getBool(key, true);
            cnt++;
        }
        p.end();
    }

    castersCount = cnt;
    if (cnt == 0) {
        // Defensive: a misconfigured build with zero valid casters would otherwise
        // loop scanning nothing. Log loudly so it's obvious in the serial monitor.
        Serial.println("⚠️  NTRIP: no valid casters configured (check config.cpp / NVS)");
    } else {
        Serial.printf("📡 NTRIP: %d caster(s) active (%d compile-time + %d from NVS)\n",
                      cnt, ntripCasterCount, cnt - ntripCasterCount);
    }
}

// ── Module state ──────────────────────────────────────────────────────────────
// NTRIP User-Agent token sent with every caster request (source-table fetch, mount
// probe, session GET). PROTECTIVE NOTE: this is the exact string the field-proven
// firmware presented when casters (rtk2go/SNIP Cybernet_1 among them) served RTCM to
// this rover. Casters may filter or log by client name, so do not change it without a
// field check against a live caster; it must begin with "NTRIP" per the NTRIP spec.
#define NTRIP_USER_AGENT "NTRIP RCX_RaceCapture/1.0"

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
// Network-layer failure state (see NTRIP_NET_RETRY_MS). lastFailWasNet selects the
// fixed 10 s retry in ntripBackoffMs(); netFailCount counts consecutive ones toward
// the reachability probe. Neither touches ntripFailCount, the caster-facing count.
static bool          lastFailWasNet       = false;
static uint8_t       netFailCount         = 0;
// Per selection pass: scans that actually dialled a caster, and how many of those
// failed at the TCP/DNS layer. All dialled + all network failures = no internet.
static uint8_t       scanDialled          = 0;
static uint8_t       scanNetFailed        = 0;
static bool          scanHeapSkipped      = false;   // a scan was refused for internal RAM: nothing was dialled
static bool          scanNoLink           = false;   // a scan was refused because the WiFi link was down

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

// ── RTK heap policy ──────────────────────────────────────────────────────────
// Internal RAM is the scarce resource on this unit: with WiFi, the web server and BLE up
// it settles at 25-28 KB free, and a source-table scan, a CANraw recording or a file
// download each take a share of it. RTCM streaming itself held at every level seen in the
// field (down to 3 KB free during a log download); what RAM decides is whether RTK can be
// STARTED — the scan that selects a mount, and a fresh TCP session after a drop.
//   Scan cost, measured in the field on two scans: 33,056 B free at start and 17,088 B at
//   the trough (16.0 KB), and 28,104 B at start and 18,128 B at the trough (10.0 KB). The
//   cost varies with what else is allocating during the scan; design to the larger figure.
//   The firmware itself allocates nothing during a scan (static line buffer, WiFiClient on
//   the task stack); the cost is the socket, the 1,436 B client read buffer and the inbound
//   table data held in lwIP until it is read.
//   RTK_SCAN_MIN_FREE_IDLE   no RTCM flowing: RTK is the priority, so the scan is allowed
//                            whenever its trough stays near 10 KB. A single 30,000 B gate
//                            sits above the normal 25-28 KB level, and a unit that had come
//                            up without corrections would never try rtk2go at all.
//   RTK_SCAN_MIN_FREE_STREAM RTCM is flowing: a background rescan only looks for a closer
//                            mount and must never cost the live stream, so it keeps the
//                            stricter 30,000 B gate.
//   RTK_HEAP_CRIT_*          below either, a fresh TCP session may not get its buffers.
// The 10 KB trough is an engineering margin, not a measured limit: the log shows the card
// and the dashboard healthy at 17 KB and at 8 KB, the SD host unreliable only when the
// DMA-capable pool (a narrower pool than these figures) runs short. The skip lines and the
// inhibit lines below carry the free and largest-block figures so the next log settles it.
static const size_t RTK_SCAN_MIN_FREE_IDLE   = 26000;
static const size_t RTK_SCAN_MIN_FREE_STREAM = 30000;
// With the raw CAN log held closed the scan's worst measured cost (16 KB) still leaves the 8 KB critical floor.
static const size_t RTK_SCAN_MIN_FREE_RAWHELD = 24000;
static const size_t RTK_HEAP_CRIT_FREE       = 8000;
static const size_t RTK_HEAP_CRIT_BLOCK      = 2048;
static unsigned long rtkHeapSkipMs   = 0;   // millis() of the last scan refused for RAM while RTK was down; 0 = none pending
static uint8_t       rtkInhibitNow   = RTK_INH_NONE;

static inline size_t heapFreeInt()    { return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); }
static inline size_t heapLargestInt() { return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); }

// RTCM bytes arrived in the last 5 s on a live session.
static inline bool rtkStreaming() {
    return lastRtcmByteMs != 0 && (millis() - lastRtcmByteMs) < 5000UL && ntripClient.connected();
}

// Once a second: decide whether RAM is holding RTK back, publish it to status.rtkInhibit (read
// by the LCD, the status LED and the dashboard) and log every change with the figures.
//   RTK_INH_HEAP_CRIT  free or largest block below the critical floor — a reconnect may fail
//   RTK_INH_HEAP_SCAN  RTK is down and the last mount selection was refused for RAM
static void serviceRtkHeapState() {
    static unsigned long lastEval = 0;
    const unsigned long now = millis();
    if (now - lastEval < 1000UL) return;
    lastEval = now;
    const size_t f = heapFreeInt(), b = heapLargestInt();
    const bool streaming = rtkStreaming();
    if (streaming) rtkHeapSkipMs = 0;             // corrections are flowing: nothing left to start
    uint8_t want = RTK_INH_NONE;
    if (f < RTK_HEAP_CRIT_FREE || b < RTK_HEAP_CRIT_BLOCK) want = RTK_INH_HEAP_CRIT;
    else if (!streaming && rtkHeapSkipMs != 0 && (now - rtkHeapSkipMs) < 120000UL) want = RTK_INH_HEAP_SCAN;
    if (want == rtkInhibitNow) return;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;   // try again next second
    status.rtkInhibit = want; xSemaphoreGive(dataMutex);
    if (want == RTK_INH_NONE)
        Serial.printf("✅ RTK inhibit cleared — RAM recovered (%u B free, largest block %u B)\n", (unsigned)f, (unsigned)b);
    else
        Serial.printf("⛔ RTK %s — %s (%u B free, largest block %u B; mount selection needs %u B, critical floor %u B / %u B)\n",
                      streaming ? "AT RISK" : "INHIBITED", rtkInhibitText(want),
                      (unsigned)f, (unsigned)b, (unsigned)RTK_SCAN_MIN_FREE_IDLE,
                      (unsigned)RTK_HEAP_CRIT_FREE, (unsigned)RTK_HEAP_CRIT_BLOCK);
    rtkInhibitNow = want;
}

// Form of the mount GET sent to the caster. Two are kept because the caster decides, from
// the request alone, whether to stream to a session it has already answered with 200:
//   REQ_PLAIN   the NTRIP Rev1 request every basic rover sends — GET, User-Agent,
//               Authorization, blank line — with the rover's GGA sent as a plain line
//               right after the reply. Fallback form.
//   REQ_HEADERS the same plus "Ntrip-Version: Ntrip/1.0" and an "Ntrip-GGA:" header
//               carrying the GGA inside the request. First choice: it is the form
//               rtk2go streams to this rover (Cybernet_1, field-verified), and it
//               serves any caster that wants the position before replying.
// PROTECTIVE NOTE: a session that ends having delivered no RTCM switches the form for the
// next attempt, and the form that last delivered RTCM is kept. Do not pin one form: a
// caster that accepts one form and closes or starves the other is exactly the failure
// this exists for, and only the alternation lets the rover find the working form on its
// own during a drive.
enum NtripReqStyle : uint8_t { REQ_PLAIN = 0, REQ_HEADERS = 1 };
static NtripReqStyle ntripReqStyle        = REQ_HEADERS;
static const char* reqStyleName(NtripReqStyle st) { return st == REQ_PLAIN ? "plain Rev1" : "Rev1 + Ntrip-Version/Ntrip-GGA headers"; }
static bool          firstBytesLogged     = false;   // first RTCM bytes of this session logged yet?
// A session that delivered nothing: try the other request form next time.
static void ntripSwitchReqStyle(const char* why) {
    ntripReqStyle = (ntripReqStyle == REQ_PLAIN) ? REQ_HEADERS : REQ_PLAIN;
    Serial.printf("📡 NTRIP: %s — next attempt uses the %s request\n", why, reqStyleName(ntripReqStyle));
}

// ── ntripAlive ────────────────────────────────────────────────────────────
// Session liveness, including a caster that closes the socket gracefully.
//
// PROTECTIVE NOTE: this helper peeks the socket itself and classifies the result —
// recv()==0 is the caster's orderly close (FIN), recv()<0 with anything but
// EWOULDBLOCK/EAGAIN is a reset or local fault (errno logged) — then logs how the
// session ended, which is the one fact that tells "caster refused us after login"
// from a local fault. WiFiClient::connected() (core 3.3.8) must NOT run first: it
// does the same peek and, on a close, just marks the client disconnected, so the
// classification and the log line below were never reached.
// Use it wherever "is the session still up" decides anything.
// One-line snapshot of the local conditions at the moment a session ends, so a close can be
// told apart as caster-side (link and heap healthy) or local (WiFi dropping, internal heap
// exhausted for lwIP, STA RSSI collapsing). Formatted into a static buffer: no heap use here.
static const char* linkContext() {
    static char buf[96];
    snprintf(buf, sizeof(buf), "wifi=%d rssi=%d heap=%u largest=%u",
             (int)WiFi.status(), (int)WiFi.RSSI(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    return buf;
}

static bool ntripAlive() {
    // Peek the socket BEFORE WiFiClient::connected(). connected() runs the same recv and, on a
    // caster's FIN or reset, silently marks the client closed (lwIP reports ERR_CLSD as recv()==0
    // with errno ENOTCONN, which connected() treats as "disconnected") — calling it first meant a
    // close never reached the classification and log below.
    const int sfd = ntripClient.fd();
    if (sfd < 0) return false;                      // no socket: closed by us, nothing to report
    uint8_t dummy;
    errno = 0;
    const int res = recv(sfd, &dummy, 1, MSG_DONTWAIT | MSG_PEEK);
    if (res > 0) return true;                       // data waiting: alive
    if (res == 0 && ntripClient.available() > 0) return true;   // closed, but bytes already read off the
                                                                // socket are still buffered: forward them first
    if (res == 0) {                                 // orderly shutdown by the caster
        Serial.printf("📡 NTRIP: caster closed the connection (FIN) after %lu ms | %s\n",
                      lastConnectOkMs ? (unsigned long)(millis() - lastConnectOkMs) : 0UL, linkContext());
        ntripClient.stop();
        return false;
    }
    if (errno == EWOULDBLOCK || errno == EAGAIN || errno == ENOENT) return ntripClient.connected();   // idle, still open
    const int lostErrno = errno;
    Serial.printf("📡 NTRIP: connection lost (errno %d) after %lu ms | %s\n", lostErrno,
                  lastConnectOkMs ? (unsigned long)(millis() - lastConnectOkMs) : 0UL, linkContext());
    ntripClient.stop();
    return false;
}
// Edge-detector for the connected→disconnected transition (see ntrip_loop).
static bool          wasNtripConnected    = false;

// RTCM bytes this session has delivered: rtcmBytesTotal at the moment the caster accepted the
// request is the zero point. A session that ends with none is a mount that logs us in and sends
// nothing, however many times the handshake itself succeeds.
static uint32_t      sessionStartBytes    = 0;
static unsigned long sessionConnectMs     = 0;   // when the current session connected
static bool          sessionPromoted      = false; // this session's mount already moved to the front of the nearby-mount cache
static uint8_t       emptySessionCount    = 0;   // consecutive sessions that ended with no RTCM at all

// Mountpoints set aside after NTRIP_EMPTY_SESSIONS_BEFORE_ABANDON empty sessions in a row. The
// geographic scan skips them until the cooldown ends; without this the scan re-picks the same
// nearest mount on every pass and the unit retries one dead mount for the whole drive.
struct BadMount { char name[64]; int8_t caster; unsigned long until; };
static BadMount badMounts[8];
static void markMountBad(int caster, const char* name) {
    int slot = 0;
    for (int i = 0; i < 8; i++) {
        if (badMounts[i].caster == caster && strncmp(badMounts[i].name, name, 63) == 0) { slot = i; break; }   // refresh in place
        if (badMounts[i].until == 0 || (long)(millis() - badMounts[i].until) >= 0) slot = i;                 // else reuse an expired slot
    }
    strncpy(badMounts[slot].name, name, 63); badMounts[slot].name[63] = '\0';
    badMounts[slot].caster = (int8_t)caster;
    badMounts[slot].until  = millis() + NTRIP_BAD_MOUNT_COOLDOWN_MS;
    if (badMounts[slot].until == 0) badMounts[slot].until = 1;   // 0 means "empty slot"
}
static bool mountIsBad(int caster, const char* name) {
    for (int i = 0; i < 8; i++)
        if (badMounts[i].until && badMounts[i].caster == caster && strncmp(badMounts[i].name, name, 63) == 0 &&
            (long)(millis() - badMounts[i].until) < 0) return true;
    return false;
}

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
    if (lastFailWasNet) return NTRIP_NET_RETRY_MS;            // never reached a caster (see config.h)
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

// ── Nearby-mount cache ───────────────────────────────────────────────────────
// A source-table scan costs 10-16 KB of internal RAM for its duration (see the RTK heap
// policy), more than a unit with every log channel open has to spare. The table is only
// needed to LEARN which public mounts are near; once learned, the answer is reusable. So the
// best few public mounts from each completed scan are kept in NVS, and the mount that has
// actually delivered corrections is moved to the front. When a scan is refused for RAM while
// RTK is down, selection falls back to this list: a plain direct connect, no table.
//   • Written only from a COMPLETE table (ENDSOURCETABLE seen) and only when the list changed.
//   • Used only when no source table could be read in a selection pass (refused for RAM or inside the
//     5-minute table budget; never when the caster was unreachable) and no corrections flow, so it cannot
//     disturb a working connection. A fresh scan always wins whenever one can run.
//   • Candidates are re-checked against the current position and the baseline cap on use, so
//     a list learned at another site selects nothing here, and mounts set aside by
//     markMountBad() are skipped.
struct NearMount {
    char    host[48];
    char    mpt[64];
    float   lat, lon;
    int8_t  carrier;
    uint8_t vrs;
    uint8_t pad[2];
};
#define NMC_MAX             4
#define NMC_PROMOTE_AFTER_MS 60000UL   // corrections must flow this long before a mount is trusted
#define NMC_PROMOTE_MIN_BYTES 4096U
static const char* const NMC_NS = "rcx_nmc";

static int nmcLoad(NearMount* out) {
    Preferences p;
    if (!p.begin(NMC_NS, true)) return 0;
    int n = p.getUChar("n", 0);
    size_t len = p.getBytesLength("e");
    if (n < 1 || n > NMC_MAX || len != (size_t)n * sizeof(NearMount)) { p.end(); return 0; }
    p.getBytes("e", out, len);
    p.end();
    for (int i = 0; i < n; i++) {                 // a corrupt blob must never yield an unterminated name
        out[i].host[sizeof(out[i].host) - 1] = '\0';
        out[i].mpt[sizeof(out[i].mpt) - 1]   = '\0';
    }
    return n;
}

static void nmcSave(const NearMount* in, int n) {
    if (n < 1 || n > NMC_MAX) return;
    NearMount cur[NMC_MAX];
    int cn = nmcLoad(cur);
    if (cn == n && memcmp(cur, in, (size_t)n * sizeof(NearMount)) == 0) return;   // unchanged: no flash write
    // A flash write stops both cores while the cache is off; leave a breadcrumb for it.
    crumb::Scope flashWrite(crumb::SLOT_NET, crumb::N_NVS, crumb::N_NTRIP);
    Preferences p;
    if (!p.begin(NMC_NS, false)) return;
    p.putBytes("e", in, (size_t)n * sizeof(NearMount));
    p.putUChar("n", (uint8_t)n);
    p.end();
    Serial.printf("💾 Nearby-mount cache saved: %d mount(s), first %s\n", n, in[0].mpt);
}

// Moves (or adds) one mount to the front of the cache. Called once corrections have flowed
// from a public mount for NMC_PROMOTE_AFTER_MS.
static void nmcPromote(int ci, const char* mpt, double lat, double lon, int8_t carrier, bool vrs) {
    if (ci < 0 || ci >= castersCount || !mpt[0] || isnan(lat) || isnan(lon)) return;
    NearMount old[NMC_MAX], nw[NMC_MAX];
    memset(nw, 0, sizeof(nw));
    int on = nmcLoad(old);
    strncpy(nw[0].host, casters[ci].host, sizeof(nw[0].host) - 1);
    strncpy(nw[0].mpt,  mpt,              sizeof(nw[0].mpt)  - 1);
    nw[0].lat = (float)lat; nw[0].lon = (float)lon;
    nw[0].carrier = carrier; nw[0].vrs = vrs ? 1 : 0;
    int n = 1;
    for (int i = 0; i < on && n < NMC_MAX; i++) {
        if (strcmp(old[i].host, nw[0].host) == 0 && strcmp(old[i].mpt, nw[0].mpt) == 0) continue;
        nw[n++] = old[i];
    }
    nmcSave(nw, n);
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
    if (!linkUp()) { scanNoLink = true; return false; }   // before anything is stamped or spent
    if (!dialPermittedCached(ci)) { scanNoLink = true; return false; }   // internet not confirmed: nothing is dialled or counted
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
    const unsigned long prevFetch = lastCasterFetch[ci];   // restored if the table is never reached
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

    struct RawHoldGuard { bool on = false; ~RawHoldGuard() { if (on) sdlog_rawHold(false); } } holdGuard;   // releases on every return

    // ── Low-heap guard ───────────────────────────────────────────────────────
    // A source-table scan holds about 16 KB of transient internal RAM (measured; see the
    // RTK heap policy above). If internal heap is already tight, starting a scan can tip
    // the whole system over: SDMMC writes fail with ESP_ERR_NO_MEM (0x101) because the
    // driver can't allocate its DMA buffer, and AsyncTCP can't accept browser connections
    // (web page dead). Skip the scan and let the caller retry on the next pass instead.
    // With no RTCM flowing the gate is lower (RTK is what the unit is for); the skip then
    // also raises the RTK-inhibited indicator, so a unit that cannot start RTK says so.
    {
        const bool   streaming = rtkStreaming();
        size_t minFree   = streaming ? RTK_SCAN_MIN_FREE_STREAM : RTK_SCAN_MIN_FREE_IDLE;
        size_t freeInt = heapFreeInt();
        // RTK is down and RAM is short: the raw CAN log is the one large internal-RAM consumer that can be
        // set aside for the length of the scan. Its frames keep queueing in PSRAM and are written to a new
        // file afterwards. The log files themselves cost about 0.5 KB each, so only this one is held.
        if (freeInt < minFree && !streaming && sdlog_rawIsOpen()) {
            const size_t before = freeInt;
            sdlog_rawHold(true); holdGuard.on = true;
            for (int w = 0; w < 50 && sdlog_rawIsOpen(); w++) delay(50);   // the SD task closes it within a loop pass
            delay(100);
            freeInt = heapFreeInt();
            minFree = RTK_SCAN_MIN_FREE_RAWHELD;
            Serial.printf("⏸️  NTRIP: raw CAN log held for the scan (%u B free before, %u B after)\n", (unsigned)before, (unsigned)freeInt);
        }
        if (freeInt < minFree) {
            Serial.printf("⚠️  NTRIP: skipping scan — internal heap low (%u B free, need %u B, largest block %u B)\n",
                          (unsigned)freeInt, (unsigned)minFree, (unsigned)heapLargestInt());
            lastCasterFetch[ci] = 0;   // don't burn the rate-limit slot on a skip
            if (!streaming) rtkHeapSkipMs = millis() | 1UL;
            scanHeapSkipped = true;
            return false;
        }
        rtkHeapSkipMs = 0;
        Serial.printf("📡 Scan start: %u B internal heap free\n", (unsigned)freeInt);
    }

    WiFiClient c;
    c.setTimeout(5000);
    scanDialled++;
    if (!c.connect(casters[ci].host, atoi(casters[ci].port))) {
        // TCP-level failure: host down, port blocked, DNS, or — the classic for
        // rtk2go — this network's public IP is banned at their firewall (silent
        // drop, looks exactly like a connect timeout).
        Serial.printf("TCP connect to %s:%s FAILED (host down / port blocked / IP banned?)\n",
                      casters[ci].host, casters[ci].port);
        // Nothing reached the caster, so no table was fetched: give the 5-minute
        // budget back, or an internet outage spends it and the first selection
        // after the internet returns is refused as rate-limited.
        lastCasterFetch[ci] = prevFetch;
        scanNetFailed++;
        return false;
    }

    c.printf("GET / HTTP/1.0\r\nUser-Agent: " NTRIP_USER_AGENT "\r\n"
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
    NearMount cand[NMC_MAX]; float candEff[NMC_MAX]; int nCand = 0;   // best public mounts, kept for the nearby-mount cache
    bool sawEnd = false;                                                // the table was read to ENDSOURCETABLE
    unsigned long scanDeadline = millis() + 15000;   // hard cap on total scan time
    // The guard above turns a scan away below its gate on the strength of an
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
        if (strncmp(lineBuf, "ENDSOURCETABLE", 14) == 0) { sawEnd = true; break; } // table complete — stop now
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

        // A mount that logged us in and sent nothing, session after session, stays out of the running
        // until its cooldown ends (see markMountBad).
        if (mountIsBad(ci, fields[1])) continue;

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

        // Nearby-mount cache: keep the best few public mounts inside the baseline cap, in score order.
        // distanceKm() runs only for a row that would enter the list.
        if (!pref[0] && (nCand < NMC_MAX || eff < (double)candEff[nCand - 1])) {
            if (distanceKm(g.latitude, g.longitude, stLat, stLon) <= (float)NTRIP_MAX_BASELINE_KM) {
                int pos = (nCand < NMC_MAX) ? nCand++ : NMC_MAX - 1;
                while (pos > 0 && (double)candEff[pos - 1] > eff) {   // shift worse entries down
                    cand[pos] = cand[pos - 1]; candEff[pos] = candEff[pos - 1]; pos--;
                }
                memset(&cand[pos], 0, sizeof(NearMount));
                strncpy(cand[pos].host, casters[ci].host, sizeof(cand[pos].host) - 1);
                strncpy(cand[pos].mpt,  fields[1],        sizeof(cand[pos].mpt)  - 1);
                cand[pos].lat = (float)stLat; cand[pos].lon = (float)stLon;
                cand[pos].carrier = carrier;  cand[pos].vrs = vrs ? 1 : 0;
                candEff[pos] = (float)eff;
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
    if (sawEnd && nCand > 0) nmcSave(cand, nCand);   // complete table only; writes flash only when the list changed
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

// Selects the best cached public mount for the current position. Returns false when the cache
// is empty, holds nothing within the baseline cap, or every entry is on a disabled / off-network /
// set-aside caster. Scores exactly as the scan does, so the same mount would have won a fresh scan.
// A caster found unreachable while the internet is up (down, or firewalled) is left out of the cache
// fallback for the length of the table budget; a different mount on the same host would fail the same way.
static unsigned long nmcHostBlockedUntil[NTRIP_MAX_CASTERS];   // 0 = not blocked
static char          nmcLastPick[64]  = "";   // the mount this fallback last selected
static int           nmcLastPickCi    = -1;
static uint32_t      nmcLastPickBytes = 0;    // rtcmBytesTotal when it was selected
static unsigned long nmcLastPickMs    = 0;    // millis() when it was selected

static uint8_t       lastGoodPicks    = 0;   // last-good selections since corrections last flowed
static unsigned long lastGoodPickMs   = 0;   // millis() of the latest one
static const uint8_t LASTGOOD_PRIORITY_TRIES = 3;   // attempts made ahead of a scan before other choices get a turn
static unsigned long lastGoodNotBefore = 0;   // 0 = no wait; set when the active host was just found unreachable

// Pick spacing and dead-pick bookkeeping shared by the last-good and cached-mount selections.
// Returns false while the previous pick (which delivered nothing) is inside the ban-safety floor.
static bool nmcPickGate() {
    // The active host was just found unreachable: neither fallback re-dials it before the failure back-off ends.
    if (lastGoodNotBefore && (long)(millis() - lastGoodNotBefore) < 0) return false;
    // A mount picked here that has delivered nothing since (the caster answered, but the mount is not
    // streaming) is set aside before choosing again; without this the same dead mount is re-picked on
    // every pass. An attempt that never reached a caster (lastFailWasNet — a tunnel, a dropped hotspot)
    // says nothing about the mount and sets nothing aside.
    // Picks that deliver nothing are spaced by the ban-safety floor, so an unreachable or refusing host is
    // not dialled in a tight loop through this path.
    // The wait after a pick that delivered nothing is the current failure back-off (never below the floor),
    // so a host that keeps failing is dialled on the same escalating schedule as any retained mount.
    if (nmcLastPick[0] && rtcmBytesTotal == nmcLastPickBytes) {
        unsigned long wait = ntripBackoffMs();
        if (wait < NTRIP_RECONNECT_FLOOR_MS) wait = NTRIP_RECONNECT_FLOOR_MS;
        if (millis() - nmcLastPickMs < wait) return false;
    }
    if (nmcLastPick[0] && nmcLastPickCi >= 0 && rtcmBytesTotal == nmcLastPickBytes && !lastFailWasNet)
        markMountBad(nmcLastPickCi, nmcLastPick);
    return true;
}

// ── Last-good mount ────────────────────────────────────────────────────────────
// The public mount that most recently delivered corrections in this session. A mount that worked does
// not stop working because the link dropped (a tunnel, a hotspot hiccup), so after any drop that
// clears the active mount it is re-selected at once: no source table, no RAM gate, one GET.
// Own-base mounts are not tracked here; Phase 0 re-selects those directly.
static NearMount lastGood;
static bool      lastGoodValid = false;
static const uint32_t LASTGOOD_MIN_BYTES = 1024U;   // corrections actually received before a mount is trusted

static void lastGoodRemember() {
    if (lastGoodValid && lastGood.carrier == activeMountCarrier && strcmp(lastGood.mpt, activeMountpoint) == 0 &&
        strcmp(lastGood.host, casters[activeCasterIdx].host) == 0) return;
    memset(&lastGood, 0, sizeof(lastGood));
    strncpy(lastGood.host, casters[activeCasterIdx].host, sizeof(lastGood.host) - 1);
    strncpy(lastGood.mpt,  activeMountpoint,              sizeof(lastGood.mpt)  - 1);
    lastGood.lat = (float)activeMountLat; lastGood.lon = (float)activeMountLon;
    lastGood.carrier = activeMountCarrier; lastGood.vrs = activeMountVRS ? 1 : 0;
    lastGoodValid = true;
}

// priority = true : the first LASTGOOD_PRIORITY_TRIES attempts after corrections stop, ahead of any scan.
// priority = false: the final fallback when the scan and the nearby-mount cache offered nothing; once the
//                   priority attempts are spent it re-dials no faster than the source-table budget.
static bool selectLastGoodMount(const GnssData& g, bool priority) {
    if (!lastGoodValid) return false;
    if (priority && lastGoodPicks >= LASTGOOD_PRIORITY_TRIES) return false;
    if (!priority && lastGoodPicks >= LASTGOOD_PRIORITY_TRIES && millis() - lastGoodPickMs < NTRIP_SOURCE_TABLE_MIN_INTERVAL_MS) return false;
    if (!nmcPickGate()) return false;
    int ci = -1;
    for (int k = 0; k < castersCount; k++)
        if (casters[k].enabled && !casters[k].prefMpt[0] && strcmp(casters[k].host, lastGood.host) == 0) { ci = k; break; }
    if (ci < 0 || !casterOnThisNetwork(ci)) return false;
    // Set-aside status does not apply: this mount delivered corrections in this power-on session, and a base
    // that dropped out may well be back. Other selections still skip it while it is set aside.
    // The host block set by an unreachable-while-the-internet-is-up verdict does not apply here: this mount
    // worked, and the pick spacing above already keeps a down host from being dialled in a loop.
    const float km = distanceKm(g.latitude, g.longitude, lastGood.lat, lastGood.lon);
    if (km > (float)NTRIP_MAX_BASELINE_KM) { lastGoodValid = false; return false; }   // the vehicle has moved on
    const double cosLat = cos(radians(g.latitude));
    const double dL = (double)lastGood.lat - g.latitude, dO = ((double)lastGood.lon - g.longitude) * cosLat;
    activeCasterIdx = ci; activeBestDistSq = dL * dL + dO * dO;
    activeMountLat = lastGood.lat; activeMountLon = lastGood.lon;
    activeMountDistKm = km; activeMountCarrier = lastGood.carrier; activeMountVRS = lastGood.vrs != 0;
    activeIsPreferred = false;
    strncpy(activeMountpoint, lastGood.mpt, sizeof(activeMountpoint) - 1);
    activeMountpoint[sizeof(activeMountpoint) - 1] = '\0';
    lastRescanTime = millis();
    rtkHeapSkipMs = 0;
    strncpy(nmcLastPick, activeMountpoint, sizeof(nmcLastPick) - 1); nmcLastPick[sizeof(nmcLastPick) - 1] = '\0';
    nmcLastPickCi = ci; nmcLastPickBytes = rtcmBytesTotal; nmcLastPickMs = millis();
    lastGoodPicks++; lastGoodPickMs = millis();
    Serial.printf("📡 Last-good mount re-selected: %s on %s (%.1f km) — no source table needed\n",
                  activeMountpoint, casters[ci].host, km);
    return true;
}

static bool selectCachedMount(const GnssData& g) {
    if (!nmcPickGate()) return false;
    NearMount list[NMC_MAX];
    int n = nmcLoad(list);
    if (!n) return false;
    const double cosLat = cos(radians(g.latitude));
    int best = -1, bestCi = -1; double bestEff = 1e18, bestAct = 0; float bestKm = 0;
    for (int i = 0; i < n; i++) {
        int ci = -1;
        for (int k = 0; k < castersCount; k++)
            if (casters[k].enabled && !casters[k].prefMpt[0] && strcmp(casters[k].host, list[i].host) == 0) { ci = k; break; }
        if (ci < 0 || !casterOnThisNetwork(ci) || mountIsBad(ci, list[i].mpt)) continue;
        if (ci < NTRIP_MAX_CASTERS && nmcHostBlockedUntil[ci] && (long)(millis() - nmcHostBlockedUntil[ci]) < 0) continue;
        float km = distanceKm(g.latitude, g.longitude, list[i].lat, list[i].lon);
        if (km > (float)NTRIP_MAX_BASELINE_KM) continue;
        double dL = (double)list[i].lat - g.latitude, dO = ((double)list[i].lon - g.longitude) * cosLat;
        double act = dL * dL + dO * dO;
        double eff = act * (list[i].carrier >= 2 ? 0.60 : 1.0) * (list[i].vrs ? 0.70 : 1.0);
        if (eff < bestEff) { bestEff = eff; bestAct = act; best = i; bestCi = ci; bestKm = km; }
    }
    if (best < 0) return false;
    activeCasterIdx = bestCi; activeBestDistSq = bestAct;
    activeMountLat = list[best].lat; activeMountLon = list[best].lon;
    activeMountDistKm = bestKm; activeMountCarrier = list[best].carrier; activeMountVRS = list[best].vrs != 0;
    activeIsPreferred = false;
    strncpy(activeMountpoint, list[best].mpt, sizeof(activeMountpoint) - 1);
    activeMountpoint[sizeof(activeMountpoint) - 1] = '\0';
    lastRescanTime = millis();
    rtkHeapSkipMs = 0;   // a mount is selected: RTK is no longer waiting on RAM
    strncpy(nmcLastPick, activeMountpoint, sizeof(nmcLastPick) - 1); nmcLastPick[sizeof(nmcLastPick) - 1] = '\0';
    nmcLastPickCi = activeCasterIdx; nmcLastPickBytes = rtcmBytesTotal; nmcLastPickMs = millis();
    Serial.printf("📡 Cached mount selected: %s on %s (%.1f km) — no source table this pass\n",
                  activeMountpoint, casters[activeCasterIdx].host, activeMountDistKm);
    return true;
}

// ── Mountpoint selection ──────────────────────────────────────────────────────
// Scans all configured casters. Preferred-mount logic lives inside scanCaster:
//   • ntripPreferredMpt[ci] non-empty → that mountpoint wins with eff=-1 if found
//     in the source table; absent (not broadcasting) → falls through to geographic
//   • All casters are always scanned so rtk2go is tried even when Centipede fails
static bool findNearestMountpoint(const GnssData& g) {
    scanDialled = scanNetFailed = 0; scanNoLink = false; scanHeapSkipped = false;
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
        if (!casterOnThisNetwork(ci)) {          // private address on another subnet
            Serial.printf("📡 [pref] %s/%s — not on this network's subnet, skipped\n",
                          casters[ci].host, casters[ci].prefMpt);
            continue;
        }
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

    // A public mount that delivered corrections earlier in this session comes back first: it needs no
    // source table and therefore no scan RAM. Own-base mounts were handled in Phase 0 above.
    if (!rtkStreaming() && selectLastGoodMount(g, true)) return true;

    double bestD=1e18, bestLat=NAN, bestLon=NAN; char bestMpt[64]="";
    int bestCi=0; int8_t bestCar=-1; bool bestVRS=false;

    for (int ci=0; ci<castersCount; ci++) {
        if (!casters[ci].enabled) continue;      // disabled from the dashboard
        if (!casterOnThisNetwork(ci)) continue;  // private address on another subnet
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
    // No source table was read this pass for a reason other than the network (refused for RAM, or inside
    // the 5-minute table budget) and no corrections flow: fall back to a mount learned earlier. A scan
    // that could not reach its caster is left to the existing network back-off.
    if (!rtkStreaming()) {
        if (scanNetFailed == 0 && selectCachedMount(g)) return true;
        if (selectLastGoodMount(g, false)) return true;
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
    if (heapFreeInt() < RTK_SCAN_MIN_FREE_STREAM || !linkUp()) return false;

    WiFiClient c;
    if (!c.connect(casters[ci].host, atoi(casters[ci].port))) return false;

    // Build credentials for THIS caster without touching activeCasterIdx (the
    // active session may belong to a different caster).
    int saved = activeCasterIdx;
    activeCasterIdx = ci;
    String auth = ntripBase64();
    activeCasterIdx = saved;

    c.printf("GET /%s HTTP/1.0\r\nUser-Agent: " NTRIP_USER_AGENT "\r\n"
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
        if (!casterOnThisNetwork(ci)) continue;  // private address on another subnet
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
    if (!dialPermitted(activeCasterIdx)) return NTRIP_NOLINK;   // no link, or the internet is not confirmed: nothing was dialled
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
        return NTRIP_NETFAIL;
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
    Serial.printf("📡 NTRIP connect %s | %s request | GGA %s (valid=%d, %d sats)\n",
                  activeMountpoint, reqStyleName(ntripReqStyle),
                  gga.length() ? (ntripReqStyle == REQ_PLAIN ? "follows the reply" : "in header")
                               : "EMPTY — mount may withhold RTCM",
                  g.valid ? 1 : 0, g.numSV);

    if (ntripReqStyle == REQ_PLAIN)
        ntripClient.printf("GET /%s HTTP/1.0\r\nUser-Agent: " NTRIP_USER_AGENT "\r\n"
                           "Authorization: Basic %s\r\n\r\n",
                           activeMountpoint, ntripBase64().c_str());
    else
        ntripClient.printf("GET /%s HTTP/1.0\r\nUser-Agent: " NTRIP_USER_AGENT "\r\n"
                           "Ntrip-Version: Ntrip/1.0\r\nAuthorization: Basic %s\r\n%s\r\n",
                           activeMountpoint, ntripBase64().c_str(), ggaHdr.c_str());

    unsigned long to=millis()+5000;
    while (!ntripClient.available() && millis()<to) delay(10);
    String resp=ntripClient.readStringUntil('\n');
    {   // The caster's own words, in the log: a mount that answers 200 and then goes quiet is told
        // apart from one that refuses only by this line.
        String r = resp; r.trim();
        Serial.printf("📡 NTRIP reply from %s/%s: %s\n", casters[activeCasterIdx].host, activeMountpoint,
                      r.length() ? r.c_str() : "(none within 5 s)");
    }
    if (resp.indexOf("SOURCETABLE")>=0) {
        ntripClient.stop();
        if (mountLocked) {   // a mount that was delivering is off the air for now: it stays selected and is retried
            Serial.printf("⚠️  NTRIP: %s is not streaming right now — keeping it, retrying on the failure back-off\n", activeMountpoint);
            return NTRIP_FAIL;
        }
        activeMountpoint[0]='\0'; activeBestDistSq=1e18;
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
    // session. Bound its blocking READS now (Stream::setTimeout; on core 3.3.8 it
    // does not reach writes — the keep-alive in ntrip_loop is made non-blocking
    // by a writability check instead), not any earlier: the connect/header-drain
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
    // PROTECTIVE NOTE: the session is live from THIS moment, so the drop edge-detector must know
    // it now. It used to learn it only at the top of the next ntrip_loop pass; a caster that answers
    // ICY 200 and closes before then was never counted as a drop — no failure count, no empty-session
    // count, status left "connected" — and the reconnect ran at the 4 s first-attempt spacing forever
    // (rtk2go bans IPs for exactly that).
    wasNtripConnected = true;
    lastGgaSend       = 0;       // the rover's GGA goes out on the very next pass (the plain request carries none)
    firstBytesLogged  = false;
    sessionStartBytes = rtcmBytesTotal;   // bytes delivered by THIS session are counted from here
    lockBarred = false;
    sessionConnectMs  = millis();
    sessionPromoted   = false;
    lastRtcmByteMs  = millis();   // arm the no-data watchdog for this session
    baselineSnifferReset();       // re-verify baseline each session
    gnss_rtcmStreamStart();   // the receiver must not be left mid-frame from the last session
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
    memset(badMounts, 0, sizeof(badMounts)); emptySessionCount = 0; sessionStartBytes = 0;
    memset(nmcHostBlockedUntil, 0, sizeof(nmcHostBlockedUntil)); nmcLastPick[0] = '\0'; lastGoodValid = false; lastGoodNotBefore = 0; lastGoodPicks = 0;
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

// True from the moment the list was changed until ntrip_loop() has rebuilt the table from NVS.
// The web page keeps asking while this is set, so it never shows the list from before the change.
bool ntrip_casterReloadPending() { return castersReloadPending || castersRebuilding; }

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
    lockReleasePending = true;   // the operator changed what may be selected: a held mount may change
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
    serviceRtkHeapState();   // RAM verdict for the LCD / LED / dashboard; one cheap compare a second

    // ── Apply pending caster-list changes ─────────────────────────────────
    // Deliberately the first thing in the loop: this is the one point where no
    // scan is in flight and no index into the table is being held. Changing the
    // caster list restarts selection from scratch, which is the honest response
    // to the operator having changed what may be selected.
    if (lockReleasePending) {
        lockReleasePending = false;
        lockBarred = true;   // the session already running does not re-lock; the next session may
        if (mountLocked) {
            mountLocked = false;
            Serial.println("📌 NTRIP: caster list edited — the held mount may change");
        }
    }
    if (castersReloadPending) {
        castersRebuilding    = true;
        castersReloadPending = false;
        Serial.println("📡 NTRIP: caster list changed — reselecting");
        loadCasters();
        ntripClient_reset();
        castersRebuilding    = false;
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
        for (int i = 0; i < NTRIP_MAX_CASTERS; i++) {
            lastCasterFetch[i]      = 0;   // source-table rate limit
            nmcHostBlockedUntil[i]  = 0;   // host blocked after unreachable/ban-looking failures
            prefMptCooldownUntil[i] = 0;   // preferred-mount cooldown
        }
        lastGoodNotBefore = 0;             // wait before retrying a just-failed active host
        emptySessionCount = 0;
        memset(badMounts, 0, sizeof(badMounts));   // mounts set aside after empty sessions
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
    bool nowConnected = ntripAlive();
    if (wasNtripConnected && !nowConnected) {
        unsigned long upMs = (lastConnectOkMs != 0) ? (millis() - lastConnectOkMs) : 0xFFFFFFFFUL;
        const uint32_t sessBytes = rtcmBytesTotal - sessionStartBytes;
        if (upMs < NTRIP_RECONNECT_FLOOR_MS) {
            ntripFailCount++;   // unstable mountpoint — escalate backoff
            Serial.printf("⚠️  NTRIP: %s dropped after %lus with %lu RTCM bytes received (unstable) — backoff escalates\n",
                          activeMountpoint, upMs / 1000UL, (unsigned long)sessBytes);
        } else {
            Serial.printf("⚠️  NTRIP dropped after %lus, %lu RTCM bytes received (healthy session ended)\n",
                          upMs / 1000UL, (unsigned long)sessBytes);
        }
        if (sessBytes == 0) {
            ntripSwitchReqStyle("session closed with no RTCM");
            // Logged in, nothing delivered, closed from the far end. Three of these in a row condemn
            // the mount: set it aside and select the next one, instead of dialling it every 30 s.
            if (++emptySessionCount >= NTRIP_EMPTY_SESSIONS_BEFORE_ABANDON && !mountLocked) {
                Serial.printf("⚠️  NTRIP: %s closed %u sessions in a row without sending RTCM — setting it aside for %lu min and selecting another mount\n",
                              activeMountpoint, (unsigned)emptySessionCount, NTRIP_BAD_MOUNT_COOLDOWN_MS / 60000UL);
                if (activeIsPreferred) { prefMptCooldownUntil[activeCasterIdx] = millis() + NTRIP_PREF_COOLDOWN_MS; activeIsPreferred = false; }
                else                    markMountBad(activeCasterIdx, activeMountpoint);
                activeMountpoint[0] = '\0';
                activeBestDistSq    = 1e18;
                emptySessionCount   = 0;
                ntripFailCount      = 0;
                lastMountpointSearch = 0;   // select again at once
            }
        } else {
            emptySessionCount = 0;
        }
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            status.ntripConnected = false;
            xSemaphoreGive(dataMutex);
        }
    }
    wasNtripConnected = nowConnected;

    // Without a live session there is no base to measure against. The distance published to the
    // display, web page and log is cleared (-1 = unknown) and the stored base position dropped, so
    // the last value never outlives the connection that produced it, whichever path ended it.
    if (!nowConnected) {
        baseEcefValid = false;
        // The published mountpoint name goes with it (activeMountpoint itself is kept for the
        // retry); the log and display then show a mount only while a session to it is up.
        if ((status.ntripDistanceKm >= 0.0f || status.mountpoint[0]) && xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            status.ntripDistanceKm = -1.0f;
            status.mountpoint[0]   = '\0';
            xSemaphoreGive(dataMutex);
        }
    }

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
            if (mountLocked && km > (float)NTRIP_MAX_BASELINE_KM) {
                mountLocked = false;   // the vehicle has left the baseline limit's reach: this base no longer serves it
                Serial.printf("📌 NTRIP: %s is now %.0f km away — released\n", activeMountpoint, km);
            }
            if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
                status.ntripDistanceKm = km;
                xSemaphoreGive(dataMutex);
            }
        }
    }

    // ── Mountpoint selection ──────────────────────────────────────────────
    if (activeMountpoint[0]=='\0' && linkUp() && selectionPermitted()) {
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
                if (!ok && scanNoLink && scanDialled == 0) {
                    // The link dropped between the check and the scan: nothing was dialled, so nothing is
                    // counted, and selection is due again the moment the link is back.
                    lastMountpointSearch = 0;
                } else {
                const bool netOnly = scanDialled > 0 && scanNetFailed == scanDialled;
                if (ok) { ntripFailCount=0; lastFailWasNet=false; }
                else if (netOnly && !(linkUp() && internetReachable())) {
                    // Every caster dialled failed and the internet check fails too: the internet went
                    // away mid-scan. Nothing is counted against the casters, and no caster is dialled
                    // again until a check passes.
                    internetOk = false; lastNetCheck = millis() | 1UL;
                    lastFailWasNet = true;
                    Serial.printf("📡 NTRIP: internet unreachable — no caster will be dialled until a check passes (every %lu s)\n",
                                  NTRIP_NET_RETRY_MS / 1000UL);
                }
                else if (scanHeapSkipped && scanDialled == 0) {
                    // Refused for RAM before anything was dialled: an inhibited attempt, not a caster's
                    // failure. Nothing is counted, so the back-off does not grow while the heap is short;
                    // selection is tried again at the first-attempt spacing and runs the moment RAM allows.
                }
                else {
                    // A caster-level failure, or casters that fail while the internet answers (down, or
                    // this IP banned at their firewall): the escalating schedule.
                    ntripFailCount++; lastFailWasNet=false;
                }
                }
            }
        }
    }

    // ── Connection ───────────────────────────────────────────────────────
    // A mountpoint is kept across a WiFi drop (ntrip_onWifiLost), but the station
    // may have come back on a different network. If the retained caster is a
    // private address that this network cannot reach, reselect now instead of
    // spending five failed connects (and their backoff) discovering it.
    if (activeMountpoint[0] && !ntripClient.connected() && !casterOnThisNetwork(activeCasterIdx)) {
        Serial.printf("📡 NTRIP: %s is not on this network's subnet — reselecting\n",
                      casters[activeCasterIdx].host);
        mountLocked         = false;   // the caster cannot be reached from this network at all
        activeMountpoint[0] = '\0';
        activeBestDistSq    = 1e18;
        activeIsPreferred   = false;
        lastMountpointSearch = 0;
        lastConnectAttempt   = 0;
    }
    {   // Any end of a connected session, whatever the cause, invalidates the internet confirmation: the
        // redial waits for a fresh check, because the session may have ended with the internet going away.
        static bool wasConnected = false;
        if (ntripClient.connected()) wasConnected = true;
        else if (wasConnected) { wasConnected = false; internetOk = false; }
    }
    if (activeMountpoint[0] && !ntripClient.connected()) {
        if (linkUp() && millis()-lastConnectAttempt >= ntripBackoffMs()) {
            lastConnectAttempt=millis();
            NtripConnResult r=connectNtrip(g);
            if (r == NTRIP_NOLINK) lastConnectAttempt = 0;   // link dropped mid-pass: try again the moment it is back
            else if (r != NTRIP_NETFAIL) { lastFailWasNet = false; netFailCount = 0; }
            // NOT reset here on NTRIP_OK. An accepted request is not a working session: a mount that
            // answers 200 and closes before sending a byte used to wipe the failure count on every
            // connect, so the retry never climbed past the 30 s floor and the mount was never given
            // up. The count is cleared where RTCM bytes actually arrive (see the read loop).
            if (r==NTRIP_OK) { /* failure count cleared on the first RTCM byte */ }
            else if (r==NTRIP_NETFAIL) {
                // The GET never reached the caster. Ask the open internet (two public resolvers, never a
                // caster) which kind of failure this was. Keep the mountpoint either way, so the
                // reconnect is one GET, no source-table fetch.
                internetOk = false;
                if (linkUp() && internetReachable()) {
                    // The internet answers, so only THIS host failed: down, or this IP banned at its
                    // firewall (which also looks like a connect timeout). The escalating schedule applies
                    // (30 s floor, 60 s cap), and repeated failures set the host aside.
                    internetOk = true; lastNetCheck = millis() | 1UL;
                    lastFailWasNet = false;
                    ntripFailCount++;
                    if (mountLocked) {
                        Serial.printf("⚠️  NTRIP: %s unreachable while the internet is up — it delivered earlier, so it stays selected; retrying on the failure back-off\n",
                                      casters[activeCasterIdx].host);
                    } else if (ntripFailCount >= 5) {
                        // The same five-failure rule a refused login follows: about four minutes of failed
                        // dials with the internet confirmed up each time. A mount that never delivered is
                        // set aside and another is selected; the host is tried again later.
                        Serial.printf("⚠️  NTRIP: %s unreachable 5x while the internet is up — selecting another mount (it is retried later)\n",
                                      casters[activeCasterIdx].host);
                        if (activeIsPreferred) {
                            prefMptCooldownUntil[activeCasterIdx] = millis() + NTRIP_PREF_PROBE_INTERVAL_MS;
                            activeIsPreferred = false;
                        }
                        if (activeCasterIdx >= 0 && activeCasterIdx < NTRIP_MAX_CASTERS)
                            nmcHostBlockedUntil[activeCasterIdx] = (millis() + NTRIP_SOURCE_TABLE_MIN_INTERVAL_MS) | 1UL;
                        lastGoodNotBefore = (millis() + ntripBackoffMs()) | 1UL;   // the same host is retried on the failure back-off
                        activeMountpoint[0] = '\0';
                        activeBestDistSq    = 1e18;
                        lastMountpointSearch = 0;   // reselect promptly among the others
                    } else {
                        Serial.printf("⚠️  NTRIP: %s unreachable while the internet is up — keeping the mount, retrying on the failure back-off\n",
                                      casters[activeCasterIdx].host);
                    }
                } else {
                    // The internet itself is gone (a hotspot that lost its data, a link that dropped a
                    // moment ago). Nothing is counted against the caster and none is dialled until a check
                    // passes; the check repeats every NTRIP_NET_RETRY_MS.
                    lastFailWasNet = true;
                    netFailCount   = 0;
                    lastNetCheck   = millis() | 1UL;
                    Serial.printf("📡 NTRIP: internet unreachable — holding %s, no caster dialled until a check passes (every %lu s)\n",
                                  activeMountpoint, NTRIP_NET_RETRY_MS / 1000UL);
                }
            }
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
                if (ntripFailCount >= 5 && !mountLocked) {
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
    if (ntripAlive()) {
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
            if (!firstBytesLogged) {
                // Proof of what the caster actually streams (RTCM3 frames begin D3 0x).
                firstBytesLogged = true;
                char hx[3 * 16 + 1]; int k = 0;
                for (int i = 0; i < n && i < 16; i++) k += snprintf(hx + k, sizeof(hx) - k, "%02X ", buf[i]);
                Serial.printf("📡 NTRIP: first bytes from %s after %lu ms (%s request): %s\n", activeMountpoint,
                              (unsigned long)(millis() - lastConnectOkMs), reqStyleName(ntripReqStyle), hx);
            }
            gnss_forwardRtcm(gpsSerial, buf, (size_t)n);   // never blocks loop(); see gnss.cpp
            rtcmBytesTotal += n;
            gotBytes = true;
            // Establish the true baseline from the stream's own RTCM 1005/1006
            // (idles via one branch once baselineChecked is true).
            if (!baselineChecked) baselineSnifferFeed(buf, (size_t)n, g);
        }
        if (gotBytes) { lastRtcmByteMs = millis(); silentSessionCount = 0; emptySessionCount = 0; ntripFailCount = 0; lastGoodPicks = 0; }
        if (gotBytes && !mountLocked && !lockBarred && activeMountpoint[0] && rtcmBytesTotal - sessionStartBytes >= LASTGOOD_MIN_BYTES) {
            mountLocked = true;
            Serial.printf("📌 NTRIP: %s/%s is delivering corrections — staying on it for this session\n",
                          casters[activeCasterIdx].host, activeMountpoint);
        }
        if (gotBytes && !activeIsPreferred && activeMountpoint[0] && rtcmBytesTotal - sessionStartBytes >= LASTGOOD_MIN_BYTES)
            lastGoodRemember();
        // A public mount that has delivered corrections for a sustained period goes to the front of
        // the nearby-mount cache (own-base mounts are direct-connect and never need it).
        if (gotBytes && !sessionPromoted && !activeIsPreferred && activeMountpoint[0] &&
            millis() - sessionConnectMs >= NMC_PROMOTE_AFTER_MS &&
            rtcmBytesTotal - sessionStartBytes >= NMC_PROMOTE_MIN_BYTES) {
            sessionPromoted = true;
            nmcPromote(activeCasterIdx, activeMountpoint, activeMountLat, activeMountLon, activeMountCarrier, activeMountVRS);
        }

        // ── Baseline verdict ─────────────────────────────────────────────
        // The sniffer found RTCM 1005/1006 placing the base beyond the cap:
        // wrong/imposter mount. Drop it, cool the preferred mount down so the
        // geographic fallback isn't immediately preempted by phase-0 again.
        if (baselineTooFar) {
            baselineTooFar = false;
            mountLocked = false;   // a mount beyond the baseline limit is not a reference worth keeping
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
            if (rtcmBytesTotal == sessionStartBytes) ntripSwitchReqStyle("session silent");
            if (silentSessionCount >= 2 && !mountLocked) {
                Serial.printf("⚠️  NTRIP: %s silent twice in a row — abandoning mountpoint\n", activeMountpoint);
                if (activeIsPreferred) prefMptCooldownUntil[activeCasterIdx] = millis() + NTRIP_PREF_COOLDOWN_MS;
                else                    markMountBad(activeCasterIdx, activeMountpoint);
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
            // Send only if the socket can take it NOW. NetworkClient::write (core
            // 3.3.8) ignores Stream::setTimeout: when the TCP send buffer is full it
            // waits in select() up to 1 s per try, 10 tries, so a stalled uplink
            // could hold this task ~10 s per keep-alive. A zero-timeout select first
            // makes the write itself non-blocking: a socket that cannot accept ~80
            // bytes right now skips this keep-alive and tries on the next pass
            // (lastGgaSend is not advanced), and the no-data watchdog still owns
            // deciding the session is dead.
            bool sockReady = false;
            {
                const int sfd = ntripClient.fd();
                if (sfd >= 0) {
                    fd_set ws; FD_ZERO(&ws); FD_SET(sfd, &ws);
                    struct timeval tz = {0, 0};
                    sockReady = select(sfd + 1, nullptr, &ws, nullptr, &tz) > 0;
                }
            }
            String gga = sockReady ? gnss_buildGGA(g) : String();
            if (gga.length()) {
                // Timed on purpose: a send that still takes measurable time is
                // exactly the event worth seeing in the log — the field-provable
                // link between a WiFi radio stall and a dropped BLE frame,
                // instead of an inference after the fact.
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
        if (!activeIsPreferred && !mountLocked &&
            millis()-lastPrefProbe > NTRIP_PREF_PROBE_INTERVAL_MS &&
            g.valid && g.speedKnots < NTRIP_RESCAN_MAX_SPEED_KNOTS) {
            lastPrefProbe = millis();
            for (int ci = 0; ci < castersCount; ci++) {
                if (!casters[ci].enabled) continue;   // disabled from the dashboard
                if (!casters[ci].prefMpt[0]) continue;
                if (!casterOnThisNetwork(ci)) continue;   // private address on another subnet
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
        if (!activeIsPreferred && !mountLocked &&
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
    mountLocked           = false;
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
    lastFailWasNet        = false;
    netFailCount          = 0;
    internetGateReset();
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

void ntrip_onWifiUp() {
    // Cooldowns, back-offs, host blocks and the source-table budget are deadlines on the millisecond
    // clock, so they kept running through the outage: one that expired while the link was away is
    // already over, one that did not has only its remainder left. Nothing is cleared here. What the
    // link's return does change is the internet gate (internetGateReset in ntrip_onWifiLost), which
    // has to see a fresh passing check before any caster is dialled.
    Serial.printf("🔄 NTRIP: WiFi link up (%s) — %s once the internet check passes\n",
                  WiFi.localIP().toString().c_str(), activeMountpoint[0] ? "reconnecting" : "selecting");
}

void ntrip_onWifiLost() {
    ntripClient.stop();
    // The caster-facing timers (last attempt, failure count, cooldowns, host blocks) are left alone:
    // they are deadlines on the millisecond clock and keep running while the link is away, so on its
    // return whatever has expired is dialled at once and whatever has not waits only its remainder.
    lastFailWasNet     = false;
    netFailCount       = 0;
    internetGateReset();   // the link is gone; the first dial after it returns waits for a fresh check
    lastGgaSend        = 0;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        status.ntripConnected = false;
        xSemaphoreGive(dataMutex);
    }
    Serial.printf("🔄 NTRIP: WiFi lost — TCP closed, mountpoint '%s' retained\n",
                  activeMountpoint[0] ? activeMountpoint : "(none)");
}
