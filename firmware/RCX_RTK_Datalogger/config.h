#pragma once
/*
 * config.h — Personal settings for RCX Datalogger
 * =================================================
 * Edit this file for your WiFi, NTRIP credentials, CAN bus pins,
 * and feature toggles.  This file will NOT be overwritten by
 * firmware updates — it lives in your sketch folder.
 */

// ── WiFi ──────────────────────────────────────────────────────────────────────
// There are NO WiFi credentials in this file, or anywhere else in the source.
// The station list is held entirely in NVS and managed from the web dashboard
// (add / remove / reorder); list order is connection priority. See wifi_mgr.h.
//
// A freshly-flashed unit therefore knows no networks — which is exactly why the
// configuration AP below is permanent. Join it, open the dashboard, add a
// network. It is also what allows a base-only field setup with no internet at
// all: the rover joins the base's AP and takes corrections directly from it.

// The device name doubles as the configuration AP's SSID. By default it is this
// prefix plus the last two eFuse MAC bytes, so two units in the same paddock
// never advertise the same name; it can be renamed from the dashboard and the
// new name persists in NVS. Keep the prefix short — name and SSID share the
// 32-character 802.11 limit.
#define DEVICE_NAME_PREFIX   "RCX Datalogger "

//
// AP passphrase. Fewer than 8 characters (the WPA2 minimum) leaves the AP OPEN,
// which is the default: a unit that has never been configured must be joinable
// without a credential the operator has no way to look up. Set a passphrase here
// if the unit will live somewhere the AP should not be open to anyone nearby.
#define WIFI_AP_PASSWORD     ""
#define WIFI_AP_CHANNEL      1
#define WIFI_AP_MAX_CLIENTS  4

// Per-network association timeout while rotating through the stored list.
#define WIFI_CONNECT_TIMEOUT_MS  20000

// After a station link drops, how long the network that was connected gets
// exclusive retries before the full priority search restarts from the top.
#define WIFI_RECONNECT_WINDOW_MS 30000

// Per-attempt association timeout inside that window. Deliberately shorter than
// WIFI_CONNECT_TIMEOUT_MS so several attempts fit in the window instead of one
// long attempt consuming the entire budget.
#define WIFI_RECONNECT_ATTEMPT_MS 10000

// Spacing between station reconnect sweeps while disconnected. Also bounds the
// idle cost of AP-only operation, where every sweep returns immediately because
// the list is empty.
#define WIFI_RETRY_INTERVAL_MS   15000

// While someone is USING the dashboard over the configuration AP — a device joined
// AND a page or poll served within WIFI_AP_ACTIVITY_MS — the station search
// (scan + association attempts) is paused: every scan and WiFi.begin() retunes
// the one radio away from the AP's channel and drops the phone off the page. A
// device that merely auto-joined the open AP pauses nothing. An edit to the
// network list (add/enable/reorder) still gets one immediate sweep, and the pause
// lifts for one sweep after WIFI_AP_CLIENT_PAUSE_MAX_MS so an idle open page can
// never hold the unit off its network (and RTK) for long.
#define WIFI_AP_ACTIVITY_MS          120000
#define WIFI_AP_CLIENT_PAUSE_MAX_MS  180000

// Station power save WIFI_PS_MIN_MODEM instead of sleep-off, so the 2.4 GHz
// coexistence arbiter can grant the BLE controller its connection anchors —
// see radio-discipline rule 5 in wifi_mgr.cpp for the mechanism and the
// 2026-08-22 field evidence. false restores the always-on radio behavior.
#define WIFI_MODEM_SLEEP_FOR_COEX  true

// ── NTRIP Casters ─────────────────────────────────────────────────────────────
// columns: host, port, username, password
// Definitions live in config.cpp — add/remove rows there.
extern const char* ntripCasters[][4];
extern const int   ntripCasterCount;

// Upper bound on configured casters. The per-caster rate-limit array
// (lastCasterFetch[]) is sized to this. If you add casters to config.cpp,
// keep their total <= NTRIP_MAX_CASTERS or raise this value. scanCaster()
// bounds-checks against it so an over-count can never corrupt memory.
#define NTRIP_MAX_CASTERS  8

// Optional: preferred mountpoint for each caster, indexed to match
// ntripCasters[]. Set to your known mountpoint name to skip the full
// source-table scan and connect with top priority. Leave "" for auto-select.
// Size must always be >= ntripCasterCount.
extern const char* ntripPreferredMpt[];

// ── NTRIP Behaviour ───────────────────────────────────────────────────────────
//#define NTRIP_QUICK_THRESHOLD_DEG      0.45  //replaced with km
#define NTRIP_MAX_BASELINE_KM      100.0 //km
#define NTRIP_RESCAN_INTERVAL_MS       3600000   // 1 hr
#define NTRIP_IMPROVE_THRESHOLD        0.75
#define NTRIP_SOURCE_TABLE_MIN_INTERVAL_MS   300000
#define NTRIP_CONNECT_INITIAL_RETRY_MS        15000

// No-data watchdog: a connected mountpoint that sends zero RTCM bytes for this
// long is dead weight (healthy bases send every second) — drop and retry/fail
// over. This is what turns "connected: yes" into "connected AND functioning".
#define NTRIP_DATA_TIMEOUT_MS                 20000

// Cooldown applied to a PREFERRED mountpoint that proved bad at runtime (silent
// twice in a row, or RTCM 1005/1006 baseline beyond NTRIP_MAX_BASELINE_KM).
// While cooling down, phase-0 direct connect skips it so the geographic
// fallback (rtk2go nearest) actually gets used instead of being preempted.
#define NTRIP_PREF_COOLDOWN_MS               600000   // 10 min
// A session that is accepted and then closed by the caster with no RTCM delivered counts against its
// mountpoint. After this many in a row the mountpoint is set aside for NTRIP_BAD_MOUNT_COOLDOWN_MS and
// the scan selects the next candidate.
#define NTRIP_EMPTY_SESSIONS_BEFORE_ABANDON  3
#define NTRIP_BAD_MOUNT_COOLDOWN_MS          900000   // 15 min

// While riding a public fallback mount, probe the preferred mount (own base)
// this often (at standstill) to notice it coming online. A private/undeclared
// mount NEVER appears in a source table, so this probe is the only detection
// path. One short GET per probe — trivial caster load.
#define NTRIP_PREF_PROBE_INTERVAL_MS         300000   // 5 min

// First-attempt / cold-start retry spacing (ms). Used ONLY while ntripFailCount==0
// (fresh boot, or one prompt retry after a long healthy session). Kept small so a
// unit powered on right before a run reaches RTK as soon as WiFi + GPS are ready,
// instead of waiting out the 30 s ban floor that applies to genuine retries. This
// is ban-safe: a failCount==0 pass triggers at most ONE connect, and source-table
// fetches are independently rate-limited (NTRIP_SOURCE_TABLE_MIN_INTERVAL_MS).
#define NTRIP_INITIAL_RETRY_MS                 4000

// Hourly rescan-for-closer-base only runs while the car is at/near rest, because the
// scan blocks the NTRIP task for several seconds and can re-switch the RTCM stream.
// Speed is GPS speed in knots; 2.0 kn ≈ 3.7 km/h ≈ 2.3 mph — clearly "stopped" while
// tolerating GPS speed jitter at standstill. Raise if your standstill jitter is higher.
#define NTRIP_RESCAN_MAX_SPEED_KNOTS           2.0f
// Cap the reconnect backoff at 60 s. Rationale: the common outage is WiFi
// staying associated while the WAN/internet drops, so WiFi.status() never goes
// down and the watchdog's WiFi-edge reset never fires. Without a tight cap the
// exponential backoff climbs to many minutes, leaving RTK down long after the
// internet returns. 60 s bounds the reconnect delay while staying gentle on the
// caster (rtk2go bans sub-30 s reconnect loops; 1/min is fine). The source-table
// fetch is independently rate-limited (NTRIP_SOURCE_TABLE_MIN_INTERVAL_MS), so a
// 60 s connect cadence can never flood it with sourcetable requests.
#define NTRIP_CONNECT_BACKOFF_MAX_MS         60000

// No caster is dialled until the internet itself is confirmed up. A caster dial that fails tells
// nothing about WHY (hotspot lost its data, WiFi dropped a moment ago, host down, IP banned at the
// caster's firewall: all look like a connect timeout), and repeating it every few seconds is exactly
// the pattern a caster's firewall reads as a reconnect loop. So after a link drop, a session drop or a
// failed dial, the client asks the open internet instead (a bare TCP connect to two public resolvers
// by address, never a caster) and dials only once that answers. The check repeats at this spacing
// while it fails, and a caster is dialled the moment it first passes.
#define NTRIP_NET_RETRY_MS                   10000
// A dial that fails while the internet check passes is that host's own failure (down, or banned): it
// counts like a refused login. Five in a row on the 30 s floor / 60 s cap schedule (about four minutes)
// set aside a mount that has never delivered corrections; a mount that has delivered is never set aside.
// Safety valve for a network that filters the check itself (its public resolvers unreachable while
// casters work): on a network where no check has EVER passed, after this many consecutive failed
// checks with the WiFi link up, ONE caster dial is allowed per NTRIP_NET_BYPASS_DIAL_MS until a check
// passes. A network whose checks passed before and fail now has lost its internet: no valve.
#define NTRIP_NET_CHECKS_BEFORE_BYPASS           6
#define NTRIP_NET_BYPASS_DIAL_MS             60000

// Bounds blocking READS on the persistent NTRIP connection (ntripClient) once
// its connect handshake is complete (Stream::setTimeout). It does NOT bound
// writes on core 3.3.8 — NetworkClient::write waits in select() on its own fixed
// 1 s x 10 schedule — so the GGA keep-alive below is sent only when a zero-timeout
// select says the socket can take it (ntrip.cpp). The concern behind both is the
// periodic ~10 s GGA keep-alive sent while riding a non-VRS single-base mount
// (RCX1, rtk2go). The ESP32-S3 has ONE 2.4 GHz radio shared
// by WiFi and BLE via time-sliced coexistence: a keep-alive write stalled for
// seconds on a lossy mobile hotspot (routine in a moving car) can hold that
// radio long enough for the coexistence arbiter to skip a scheduled BLE
// connection event — which reads as a dropped 20 Hz BLE frame with no
// BLE-side cause at all. A stalled send just retries on the next keep-alive
// cycle instead of blocking wifiNtripTask (and the shared radio) for seconds.
// 250 ms is generous for a read on a working link while capping a stalled one
// to a small fraction of a second. Applied only AFTER the connect
// handshake / header drain in connectNtrip(), which still relies on the
// WiFiClient default timeout to wait through normal TCP packet gaps.
#define NTRIP_CLIENT_IO_TIMEOUT_MS              250

// ── User-supplied CAN databases (DBC) ────────────────────────────────────────
// Uploaded .dbc files live in this directory on the SD card. Files are staged
// whole in PSRAM before being written, so DBC_MAX_FILE_BYTES bounds both the
// staging allocation and the largest file that can be accepted. A vehicle
// database describing the channels this logger records is a few tens of KB;
// the limit is set well above that but far below a full manufacturer database,
// which would not fit in memory and is not the intended input.
#define DBC_DIR              "/dbc"
#define DBC_MAX_FILE_BYTES   (192 * 1024)
#define DBC_MAX_FILES        12

// Import audit and channel ceiling.
//
// Every telemetry sample carries the full channel array, so wire cost scales
// with channel count and not with how often a channel changes. The ceiling is
// therefore a real limit, not a formality. Two things bound it: a sample must
// fit inside the sample window at the telemetry rate, and the receiving app has
// its own maximum. Raising DBC_MAX_CHANNELS without measuring both produces
// dropped frames rather than an error.
//
// DBC_MAX_CHANNELS is set to match the CAN channel capacity published for the
// RaceCapture/Track hardware this protocol belongs to. That figure describes a
// load the receiving app is known to carry, which is a firmer basis than any
// number derived from the wire budget alone. The receiving app publishes no
// ceiling of its own; if one is ever measured and it is lower, this constant
// follows the measurement rather than the hardware figure.
//
// BLE_CHUNK_BYTES mirrors the notification payload the BLE layer settles on
// once a central negotiates a larger MTU; it is used to estimate how many
// notifications a sample would need.
#define DBC_MAX_CHANNELS     100
#define DBC_REPORT_BYTES     4096
#define BLE_CHUNK_BYTES      500
#define CAN_TELEMETRY_HZ     20

// ── BLE Device Name ───────────────────────────────────────────────────────────
// *** "RCX Datalogger" IS FIELD-VERIFIED — DO NOT RENAME. EVER. ***
// SoloStorm has successfully scanned, connected, subscribed, and streamed with
// this exact name (confirmed by the owner on hardware). SoloStorm's RaceCapture
// BLE source does NOT require a "RaceCapture" name prefix — it finds the device
// regardless. This name is the owner's deliberate branding choice and has been
// "helpfully" changed by AI assistants multiple times, breaking nothing but
// wasting debugging time. If you (an AI) are reading this and considering
// renaming it to "RaceBox Mini ...", "RaceCapture ...", or anything else: stop.
// The name is not the problem. Look at the protocol stream instead.
#define BLE_DEVICE_NAME     "RCX Datalogger"

// ── RaceCapture WiFi server ───────────────────────────────────────────────────
// The RaceCapture app connects to this device on RACECAPTURE_PORT over TCP.
// The RaceCapture app must be on the same WiFi network.
// Set to false to disable the TCP server and save ~4 KB RAM.
#define RACECAPTURE_ENABLE  true
#define RACECAPTURE_PORT    7223

// ── Diagnostics ───────────────────────────────────────────────────────────────
// Set true to print task stack high-water marks + free heap every 30 s.
// Enables detection of near-overflow tasks. Leave false in production.
// Output tagged 🔍 in Serial Monitor.
#define DEBUG_WATERMARKS    true    // ← set false once firmware is stable

// Mirror ALL Serial output to a debug_<stamp>.txt file on the SD card, so field
// serial logs are capturable without a serial monitor (e.g. from a phone in the
// car). Compiled in and ON by default; it can be switched off from the web page.
// Costs an 8 KB RAM ring + one bounded write batch per logger pass when ON.
// This only controls whether the mirror mechanism is COMPILED IN at all. Whether
// it's actually armed at boot is a runtime choice — see DEBUG_SERIAL_TO_SD_FORCE_ON
// immediately below, and the web-page toggle (persisted to NVS) at /debug_log.
#define DEBUG_SERIAL_TO_SD  true

// One line per operator action (web page, buttons), stamped with seconds since boot — the same
// clock as the "#UTC_SYNC" marker in the canraw file and the 30 s "SD rows" line — so a log
// can be read against what the person did. `Serial` resolves at the use site, so the line
// reaches the debug mirror wherever debug_log.h is included.
#define RCX_USER_ACTION(text) Serial.printf("👤 USER [%lus]: %s\n", (unsigned long)(millis() / 1000UL), (text))

// Runtime override: when true, the mirror is armed at every boot NO MATTER WHAT
// the web toggle / NVS says — the web checkbox still shows state but can't turn
// it off. Leave false for the normal case: the mirror is on by default (first boot,
// before the web page has ever written a preference), and the web toggle at /debug_log
// turns it on or off from there, persisted across reboots. Switching it off closes the
// debug file, so it is complete and downloadable without a writer behind it.
// Has no effect unless DEBUG_SERIAL_TO_SD above is also true.
#define DEBUG_SERIAL_TO_SD_FORCE_ON  false

// Prints the raw QMI8658 temperature register bytes (🌡 tagged) every read so a
// USB-connected bench session can see exactly what the register returns. Use it
// to diagnose the 262 °F (rail) IMU reading; set false once the temp is trusted.
#define IMU_TEMP_DEBUG      false   // was true for bench diagnosis of the 0x7FFF rail; the
                                    // reject-the-rail logic is trusted now, so silence the
                                    // per-read raw dump. Flip back to true to re-diagnose.

// ── CAN Bus (ESP32-S3 built-in TWAI + SN65HVD230 transceiver) ────────────────
// Set CAN_ENABLE false to compile without CAN support (all channels emit null).
// Hardware: SN65HVD230 breakout — only 2 GPIO pins + 3.3V + GND needed.
//   SN65HVD230 RXD → CAN_RX_PIN    SN65HVD230 TXD → CAN_TX_PIN (never driven)
//   SN65HVD230 CANH/L → Porsche 987.2 PT-CAN (OBD port or chassis splice)
#define CAN_ENABLE   true
#define CAN_RX_PIN   GPIO_NUM_8   // SN65HVD230 RXD output → ESP32 RX
#define CAN_TX_PIN   GPIO_NUM_9   // SN65HVD230 TXD input  ← ESP32 TX (never driven)

// ── SD Card Logging ───────────────────────────────────────────────────────────
#define SD_LOG_ENABLE       true
#define SAT_LOG_ENABLE      true
#define SD_CLK_PIN          14   // TF_SCLK → SDMMC CLK
#define SD_CMD_PIN          15   // TF_MOSI → SDMMC CMD
#define SD_D0_PIN           16   // TF_MISO → SDMMC D0
#define SD_D3_PIN           21   // TF_CS   → SDMMC D3 (10K pull-up on board)

// ── Web Server ────────────────────────────────────────────────────────────────
#define WEBSERVER_ENABLE    true
#define WEBSERVER_PORT      80

// ── GNSS hardware constants ───────────────────────────────────────────────────
// Increment GNSS_CONFIG_VERSION whenever configureLG290P() changes so the
// receiver is automatically reconfigured once after a firmware update.
// Between boots the LG290P retains settings in its own flash (PQTMSAVEPAR),
// so we skip reconfiguration and let it warm-start.
//
//   *** PROTECTIVE NOTE (for future AI/human edits) ***
//   ANY change to GNSS_RATE_MS (or to the body of configureLG290P) MUST bump
//   GNSS_CONFIG_VERSION below, or the change NEVER reaches the receiver.
//   Why: the rate lives in the LG290P's OWN flash. gnss_init() skips
//   configureLG290P() whenever the NVS stamp already equals this version, so a
//   new GNSS_RATE_MS value sits in the source code but is never transmitted —
//   the module keeps running at its previously-saved rate. This is exactly how
//   the firmware got stuck at 10 Hz with this constant set to 50: GNSS_RATE_MS
//   was changed to 50 (20 Hz) but the version was left at 1, so on a board
//   already stamped v1 the receiver was never told and stayed at 100 ms.
//   Field/forum-confirmed: $PQTMCFGFIXRATE,W,50 only takes effect after the
//   nav-engine restarts — configureLG290P() ends with PQTMSAVEPAR, which
//   provides exactly that restart. Bumping this version is what triggers it.
// NOTE: GNSS_CONFIG_VERSION only governs the FULL config (constellations, message
// rates, protocols). The fix-rate is now applied independently and only when the
// module is MEASURED running at the wrong rate (see gnss_init) — so you no longer
// need to bump this version just to change GNSS_RATE_MS. Bump it only when the
// body of configureLG290P() genuinely changes.
#define GNSS_CONFIG_VERSION_BASE  9
// The EFFECTIVE config version is computed below in config.h once PPP_NAV_DEBUG is known:
// flipping that bench flag automatically changes the version, forcing configureLG290P() to
// run on the next boot so the PQTMPPPNAV enable actually reaches the module. Without that,
// the hot-start path would skip config and the diagnostic would silently do nothing — the
// exact silent-no-op class we keep guarding against. Use GNSS_CONFIG_VERSION everywhere.
#define GNSS_RATE_MS        50    // 50 ms = 20 Hz (LG290P max; datasheet "Max. 20 Hz")

// ── UART drain budget ─────────────────────────────────────────────────────────
// Max bytes gnss_loop() consumes per pass. Bounds the CPU time GNSS parsing can
// take away from BLE/IMU in any single loop() iteration: the once-per-second GSV
// block (~1.5 kB on the wire) is spread across several passes instead of being
// chewed through in one ~30 ms bite that eats a BLE sample slot. The remainder
// waits in the 4096 B RX ring (~89 ms of line time at 460800) — nothing is lost.
// Budget must exceed worst-case inflow between passes: a 30 ms BLE stall accrues
// ~1.4 kB, and loop() iterates every ~1-2 ms when idle, so 512 B/pass drains far
// faster than the line can fill. Do NOT "fix" a backlog by raising this past
// ~1024 — that recreates the unbounded-drain latency spike this exists to remove.
#define GNSS_DRAIN_BUDGET_BYTES  512

// ── Epoch-complete watchdog ───────────────────────────────────────────────────
// sdlog_push_if_new() publishes a GPS row when the epoch is COMPLETE (its RMC has
// been applied), not when its timestamp first changes (GGA). If RMC never lands,
// this timeout publishes the row anyway with spd_age_ms exposing the staleness.
// Must exceed the worst in-epoch GGA→RMC gap (~35 ms during the GSV second) and
// stay under one epoch (50 ms) so a dropped RMC costs latency, not a lost row.
#define GNSS_EPOCH_WATCHDOG_MS   45

// ── Velocity from the RTK track ───────────────────────────────────────────────
// Under live RTK the LG290P drops its own SOG/COG for the rest of a second at a
// time (runs of 19 epochs at 20 Hz, rising with speed — 2026-08-30 NCCAR,
// 2026-09-10 CSC) while position keeps updating cleanly at 20 Hz. For those epochs
// gnss.cpp publishes velocity differentiated from the track instead: a quadratic
// least-squares fit to the last GNSS_TRACK_PTS positions, evaluated at the newest
// epoch (no lag under constant acceleration). Its velocity noise is 13.6x the
// per-epoch position noise per axis — ~0.05 m/s for the few millimetres of a FIXED
// solution. The fit's own residuals estimate that noise, and a track noisier than
// GNSS_TRACK_MAX_SIGMA_MPS is not used — a solution jump or metre-class wander then
// leaves the channel empty rather than wrong.
#define GNSS_TRACK_PTS            7      // positions kept (7 = 300 ms at 20 Hz)
#define GNSS_TRACK_MIN_PTS        5      // fit needs this many inside the span
#define GNSS_TRACK_SPAN_MS        320    // oldest usable position, ms before the newest
#define GNSS_TRACK_MAX_SIGMA_MPS  0.25f  // reject a fit whose velocity 1-sigma exceeds this
#define GNSS_TRACK_MIN_HDG_MPS    0.5f   // track heading needs at least this speed AND
#define GNSS_TRACK_HDG_SIGMA_K    8.0f   // this many velocity 1-sigmas (≤ ~7° of heading noise);
                                         // below either, the last heading is held
// An RMC whose SOG and COG text both repeat the previous epoch's exactly, at a
// speed above this, is a held value, not a measurement: 3-decimal knots and
// 2-decimal degrees cannot both stay identical across 50 ms while moving.
#define GNSS_VEL_REPEAT_MIN_KN    2.0
// ── RTK HOLD ──────────────────────────────────────────────────────────────────
// <Timeout> field of $PQTMCFGRTK — "the max differential age of RTK fix" (seconds):
// how long the LG290P keeps computing a high-precision, model-predicted RTK fix
// after the LAST RTCM correction, before dropping to FLOAT/3D. This is Quectel's
// "RTK HOLD" feature (observation-data model prediction) and it bridges brief
// NTRIP/RTCM dropouts — exactly the gaps a cellular NTRIP feed hits mid-run.
// Range 1–600 s; module default is 120. 600 = the ~10 min max hold Quectel
// advertises. Lower it if you'd rather see an honest FLOAT drop than a fix coasting
// on stale corrections. This is written ONLY inside configureLG290P(), so any change
// here MUST bump GNSS_CONFIG_VERSION above or it never reaches the receiver.
#define RTK_HOLD_TIMEOUT_S 600    // seconds [1..600]; max differential age for RTK HOLD
// ── Multipath / occlusion masks (tunable LIVE via the web page; persisted in NVS) ─
// These are only the DEFAULTS used on a fresh module / config-version bump. Once set,
// the live values live in NVS ("rcx_gnss": ele/cnr) and the module's own flash, so
// changing these #defines only matters for a first-time flash. Both take effect at the
// position engine and cost no module reset to change.
//
// Elevation mask ($PQTMCFGELETHD, range [-90,90], module default 5°, -90 = no limit):
// sats below this angle are dropped. Low elevations are where occlusion and NLOS
// reflections concentrate, so a mask sheds the worst offenders — at the cost of
// geometry/DOP if pushed too far. 20° discards the bottom fifth of the sky: enough to cut
// the horizon-grazing multipath that fenced and grandstand venues generate, while leaving
// a full constellation for the fix. Watch the sat count on the web page — with 4
// constellations you should hold well above 15 usable SVs here.
#define GNSS_ELE_MASK_DEG_DEFAULT  20.0f   // degrees
// C/N0 mask ($PQTMCFGCNRTHD, range [0,99], module default 10 dB-Hz, 0 = no limit):
// signals weaker than this are dropped. Reflections/NLOS are usually weaker than the
// direct ray, so this rejects them — but too high also drops legitimate weak-but-direct
// sats, and it stacks with the elevation mask above: both cut in the same direction, so
// their combined satellite loss is larger than either alone. 20 dB-Hz sits above the
// module's own 10 dB-Hz floor without reaching into the band where healthy low-elevation
// and newly-risen satellites live.
#define GNSS_CNR_MASK_DBHZ_DEFAULT 20.0f   // dB-Hz
// ── Mask DEFAULTS version ─────────────────────────────────────────────────────
//   *** PROTECTIVE NOTE — why this exists (read before editing the two values above) ***
//   The live masks are loaded from NVS ("rcx_gnss": ele/cnr), NOT from the #defines. On any
//   board that has already booted this firmware, NVS holds the OLD values, so changing the
//   #defines alone reaches nothing — exactly the class of silent no-op that GNSS_CONFIG_VERSION
//   was invented to prevent for the fix rate. Bumping this version tells gnss_init() to adopt
//   the new defaults ONCE (overwriting NVS), then leave the user's web-page tuning alone.
//   So: change a mask default above → bump this. Do not reuse GNSS_CONFIG_VERSION for it, or a
//   future constellation/message-rate change would silently clobber field-tuned masks.
#define GNSS_MASK_DEFAULTS_VERSION 2
// ── PPP (Precise Point Positioning) ───────────────────────────────────────────
// Correction source of LAST resort, sitting BELOW RTK in the module's own priority:
//     RTK FIXED (incl. RTK HOLD coasting) → RTK FLOAT → PPP → 3D GPS
// Enabling PPP does NOT cost you RTK — the nav engine still solves RTK whenever RTCM is
// flowing (PQTMPPPNAV <SolType>: 12 = RTK fixed outranks 7 = PPP converged). And if PPP
// never converges, the module simply reports the ordinary 3D solution. So the fallback
// ladder is automatic; no host logic decides it.
//
//   *** REQUIREMENTS AND HONEST LIMITS — read before trusting a PPP fix ***
//   - Needs LG290P firmware v2.01+ ($PQTMCFGPPP returns ERROR,3 on older builds; that is
//     exactly what the caster reported on ...A06S before its upgrade). The boot log prints
//     the read-back, so an unsupported build can never silently no-op.
//   - E6 HAS needs an antenna that actually passes the Galileo E6 band (~1278 MHz). An
//     L1/L2/L5-only antenna will never converge no matter what this is set to.
//   - Convergence is 10-20 MINUTES in open sky, and v2.01 has NO fast re-acquisition: one
//     cycle slip (canopy, trailer, tunnel of trees) costs the full convergence again. It is
//     not a fallback that "kicks in" mid-run — it is a slow background solution that either
//     happens to be converged when RTK drops, or isn't.
//   - Accuracy is decimetre-class AT BEST, it is more multipath-vulnerable than RTK, and it
//     reports optimistic accuracy while wrong (field-observed: 1.5 m error at 3 cm reported).
//     Better than vanilla 3D GPS, far worse than RTK FIXED. Log it, don't trust it blind.
//   - Fix rate: PPP is documented/tested by Quectel and SparkFun at 1 Hz. We run 20 Hz and
//     deliberately do NOT drop the rate for PPP. If the module refuses to converge at 20 Hz,
//     that is the answer and PPP is simply unavailable on this rover — check the boot
//     read-back and watch for GGA station ID 9002 before assuming it works.
// Mode: 0 = off, 1 = BeiDou B2b, 2 = Galileo E6 HAS (use 2 in the US/EU).
// Changing any of these constants requires a GNSS_CONFIG_VERSION bump to reach the module;
// the live mode is otherwise owned by NVS ("rcx_gnss": ppp) and the web toggle.
// Default OFF: for a rover on live RTCM, PPP never outranks an RTK solution, needs 10-20
// minutes to converge, loses that convergence to a single cycle slip, and reports
// optimistic accuracy while wrong — so it earns nothing on a normal run and is one more
// variable in the nav engine when a run goes wrong. Turn it on from the web page for the
// case it is actually for: a session expected to lose corrections for a long stretch.
#define GNSS_PPP_MODE_DEFAULT   0       // 0=off 1=B2b 2=E6 HAS
#define GNSS_PPP_DATUM          1       // 1=WGS84  2=PPP original  3=CGCS2000
#define GNSS_PPP_TIMEOUT_S      120     // $PQTMCFGPPP <Timeout>, range [90,180], default 120
#define GNSS_PPP_HOR_STD_M      0.10f   // horizontal convergence threshold, m [0,5]
#define GNSS_PPP_VER_STD_M      0.15f   // vertical convergence threshold, m   [0,5]
// ── PPP convergence diagnostic (BENCH ONLY) ───────────────────────────────────
// Set true to enable the module's PQTMPPPNAV output and print the PPP engine's <SolType>
// on the serial log. Fix rate is NOT touched — this runs at the normal 20 Hz. It answers
// the one question the SparkFun field reports leave open: does the PPP engine make progress
// at 20 Hz (SolType climbs 0→…→7), or does it only work at their tested 1 Hz (SolType stuck
// at 0)? Leave FALSE for normal builds — the parser and the extra 20 msg/s output are
// compiled out entirely when false. Enabling it changes configureLG290P()'s output, so it
// is paired with the GNSS_CONFIG_VERSION bump below to guarantee the message actually gets
// written on the next boot rather than being skipped by the hot-start path.
#define PPP_NAV_DEBUG           false
// Effective config version = base, +100 when the bench diagnostic is on. This makes flipping
// PPP_NAV_DEBUG a version change, so configureLG290P() re-runs and writes the PQTMPPPNAV
// enable — and flipping it back OFF is ALSO a version change, so the module gets reconfigured
// without the message too (no stale 20 msg/s stream left running). +100 offset keeps the two
// tracks from ever colliding as the base version climbs.
#if defined(PPP_NAV_DEBUG) && PPP_NAV_DEBUG
  #define GNSS_CONFIG_VERSION  (GNSS_CONFIG_VERSION_BASE + 100)
#else
  #define GNSS_CONFIG_VERSION  (GNSS_CONFIG_VERSION_BASE)
#endif
#define GPS_RX_PIN           4    // Orange: LG290P TX → ESP32 GPIO4
#define GPS_TX_PIN           5    // Yellow: LG290P RX → ESP32 GPIO5

// ── IMU (QMI8658) I2C pins ────────────────────────────────────────────────────
#define IMU_SDA_PIN         48
#define IMU_SCL_PIN         47
#define IMU_ADDR            0x6B

// ── SD logging rate reduction while idle ──────────────────────────────────────
// Power/file-size trim only — the values here trade a small amount of buffer
// dwell time for not chattering between rates on a brief lull. SD logging
// only; the BLE/SoloStorm stream is untouched and always runs full-rate,
// null values included, so this has no effect on anything a driver sees live.
//
// IMU: once the accelerometer magnitude and gyro have stayed within both thresholds of their own
// slow running average (so sensor scale and bias errors do not matter), and the car is
// below IMU_ACTIVE_SPEED_MPS, continuously for IMU_MOTIONLESS_DWELL_MS, SD logging drops
// from the active rate to IMU_MOTIONLESS_LOG_MS between rows; the decoded CAN channel
// follows the same state. A single sample back above threshold
// restores full rate immediately — slow to slow down, instant to speed up. The IMU is read at
// the active rate even while parked, and the last IMU_PREROLL_MS of IMU samples and decoded-CAN
// snapshots are written out when the car wakes, so the first moments of motion are logged too.
// The dwell is long enough that a stop in a grid queue, a cone placement or a
// static calibration window is logged at full rate; only a unit left parked
// drops. Never applied in the LCD's cone survey mode, where the still dwells
// ARE the data (pose segmentation, and the gravity / gyro-bias estimate the
// cone algorithm takes from each dwell). Thermal inhibit (thermal.h) is
// separate and still applies in every mode.
#define IMU_MOTIONLESS_GYRO_DPS   1.0f     // gyro change from its running average below this = still (parked noise p99 0.3 °/s)
#define IMU_MOTIONLESS_ACCEL_G    0.02f    // |accel| change from its running average below this = still (parked noise peaks 0.014 g)
#define IMU_MOTIONLESS_DWELL_MS   60000    // continuous stillness required
#define IMU_MOTIONLESS_REF_TAU_MS 1000     // accel magnitude and gyro are compared with their own running average over this time
#define IMU_MOTIONLESS_LOG_MS     1000     // reduced-rate row interval (1 Hz)
#define IMU_ACTIVE_SPEED_MPS      1.0f     // RTK-track or CAN speed above this keeps the car active
#define IMU_POLL_ACTIVE_MS        4        // IMU read + row interval while active (250 Hz)
#define IMU_POLL_IDLE_MS          20       // IMU read interval during a calibration capture (50 Hz); otherwise the IMU is always read at the active rate
#define IMU_PREROLL_MS            250      // while reduced-rate, the last this-many ms of samples are kept and written out ahead of the first full-rate row, so a launch loses nothing
#define IMU_PUBLISH_MS            50       // shared-snapshot interval (20 Hz): BLE streams at 20 Hz and the display needs no more
#define CAN_LOG_ACTIVE_MS         20       // decoded CAN row interval while active (50 Hz)
#define CAN_LOG_MOTIONLESS_MS     1000     // decoded CAN row interval once motionless (1 Hz)

// CAN: while status.canHz reads zero (bus silent — vehicle off), SD logging
// drops to once every CAN_IDLE_LOG_MS. Any measured frame rate
// above zero restores full rate on the very next row.
#define CAN_IDLE_LOG_MS           10000    // reduced-rate row interval (0.1 Hz)

// GPS SD row interval once thermal threshold 4 engages (thermal.h) — the last
// and smallest cut, since GPS is the last channel still running by that point.
#define GPS_THERMAL_LOG_MS        1000     // reduced-rate row interval (1 Hz)

