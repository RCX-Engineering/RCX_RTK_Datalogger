/*
 * ble_racecapture.cpp — RaceCapture/Pro JSON protocol over BLE NUS
 * =================================================================
 *
 * ============================================================================
 *  RACECAPTURE / SOLOSTORM BLE PROTOCOL — FIELD-VERIFIED REFERENCE
 *
 *  ▸ Primary source: autosportlabs/RaceCapture-Pro_firmware
 *      src/logger/loggerApi.c   — api_send_sample_record, api_getMeta,
 *                                 json_sendResult, rc values
 *      src/api/api.h            — API_SUCCESS=1, API_SUCCESS_NO_RETURN,
 *                                 API_ERROR_UNKNOWN_MSG=0
 *  ▸ Proven implementation reference: donour/racepi
 *      arduino/uRacePi2/include/rc_podium_protocol.h  — meta shape,
 *                                                       channel names
 *
 *  *** WHAT ACTUALLY WORKS WITH SOLOSTORM OVER BLE (June 2026) ***
 *  SoloStorm's "RaceCapture BLE Logger" device type was built against DIY
 *  BLE implementations, not real RaceCapture hardware (real RCP is
 *  BT-Classic SPP or WiFi — it never speaks BLE). Several multi-session
 *  debug runs revealed the following FIELD-VERIFIED truths. Where the
 *  firmware source and actual SoloStorm behavior conflict, actual behavior
 *  is listed. DO NOT change protocol shape without re-verifying on device.
 *
 * ============================================================================
 *
 * TRANSPORT — Nordic UART Service (NUS) over BLE
 *   Service : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   TX (notify, device→app) : 6E400003-...   ← we push JSON here
 *   RX (write,  app→device) : 6E400002-...   ← app sends commands here
 *   Set preferred MTU 512; Android negotiates ~509-byte payloads. Until the
 *   central raises it, payload is MTU-3 (=20 at the 23-byte default), so chunk
 *   accordingly. ESP32-S3 is BLE-only (no BT Classic).
 *   Every message is one JSON object on a single line, terminated "\r\n".
 *   The app parses line-by-line.
 *
 * GATT TABLE — DO NOT CHANGE SERVICE ORDER WITHOUT RE-TESTING
 *   Service order in the attribute table is FIXED at init time. Android caches
 *   the handle layout per device MAC address. Adding, removing, or reordering
 *   services between firmware builds shifts all handles below the change —
 *   Android keeps a stale cache and reads/writes the wrong handles, causing
 *   "connects but spins" or no RX commands arriving. After ANY build that
 *   changes the GATT table, forget the device in Android Bluetooth settings
 *   AND remove/re-add it inside SoloStorm before testing.
 *
 *   Current table order (field-proven — do not reorder):
 *     1. 0x1800  GAP  (manual: 2A00 device name, 2A01 appearance)
 *        NOTE: NimBLE auto-registers GAP+GATT internally AND we create a
 *        manual 0x1800 here. This produces a duplicate GAP service which is
 *        technically spec-invalid, but it was present in the build that first
 *        worked with SoloStorm. Removing it shifts all subsequent handles and
 *        breaks existing Android caches. Leave it as-is.
 *     2. 0x180A  Device Information Service (2A24/25/26/29)
 *     3. NUS service (TX notify, RX write)
 *
 *   Advertising: primary ADV packet carries the device name + 180A UUID.
 *   The 128-bit NUS UUID rides in the scan response (enableScanResponse=true)
 *   because the primary ADV packet budget is only 31 bytes.
 *
 * MESSAGE KINDS (device → app)
 *   1. {"ver":{...}}     — version info (response to getVer)
 *   2. {"meta":[...]}    — channel definitions (response to getMeta, and
 *                          embedded in every sample at tick==0 or tick%100==0)
 *      SHAPE: top-level {"meta":[...]}, NOT s-wrapped.
 *      This is the FIELD-PROVEN shape SoloStorm parses. The firmware emits
 *      {"meta":[...]} standalone from api_getMeta, and embeds "meta":[...]
 *      inside {"s":{...}} in samples. SoloStorm on BLE accepts both forms.
 *   3. {"s":{"t":<tick>,"meta":[...optional...],"d":[...]}} — sample record
 *   4. {"<cmd>":{"rc":N}} — command ACK
 *
 * COMMAND / RESPONSE  (dispatch_api in src/api/api.c)
 *   The app writes JSON commands to the RX characteristic:
 *     {"getMeta":null}  {"getVer":null}  {"setTelemetry":{"rate":50}}
 *   EVERY command must receive a reply or the app eventually disconnects.
 *
 *   Two response classes:
 *     • API_SUCCESS_NO_RETURN commands write their own full response object
 *       and MUST NOT also send an rc ACK (double-response confuses the parser):
 *         getMeta        → {"meta":[...]}
 *         getVer         → {"ver":{...}}
 *     • All other commands (including setTelemetry, startStreaming,
 *       stopStreaming, and any unknown command) get an rc ACK:
 *         {"<cmd>":{"rc":1}}\r\n    (API_SUCCESS = 1)
 *         {"<cmd>":{"rc":0}}\r\n    (API_ERROR_UNKNOWN_MSG = 0, for truly
 *                                    unknown commands — we send rc:1 for all
 *                                    since SoloStorm only checks for a reply)
 *
 *   NB: firmware uses "getVer" (not "getVersion") — strstr("getVer") matches
 *   both. This build does NOT implement getStatus or getCapabilities; unknown
 *   commands get a generic rc:1 ACK via the catch-all enqueueAck path, which
 *   is enough to keep SoloStorm from disconnecting.
 *
 *   *** CONTEXT RULE — NEVER transmit from the BLE host-task callback ***
 *   processCommand() and onWrite() run inside the NimBLE host task. Calling
 *   notify() or vTaskDelay() there stalls the BLE stack and drops the link
 *   (learned via a multi-session regression). Commands ONLY SET FLAGS or
 *   ENQUEUE to the ackQueue ring buffer. tick(), called from loop(), does all
 *   actual sending. Serial.printf() is safe in the host-task context.
 *
 * SAMPLE RECORD  (api_send_sample_record in src/logger/loggerApi.c)
 *   {"s":{"t":<tick>,"meta":[...optional...],"d":[v0,v1,...,bitmask0,...]}}\r\n
 *
 *   "t"    : monotonic tick counter, reset to 0 on each CCCD subscribe so
 *            the very first sample always embeds meta (tick==0).
 *   "meta" : embedded when (tick==0 || tick%100==0) — METADATA_SAMPLE_
 *            INTERVAL in firmware. This keeps channel→value binding alive
 *            mid-stream. The first sample after subscribe MUST include it.
 *   "d"    : SPARSE — only populated channels appear, in channel-index order.
 *            Values are followed by one or more uint32 bitmask words. Bit i
 *            set means channel i is present in d. One uint32 word covers each
 *            block of 32 channels; additional words are appended for channel
 *            counts above 32. Channels absent (bit clear) are simply omitted
 *            from d; the bitmask words tell the parser which indices are
 *            present.
 *            E.g. if only channels 0,2,7 are populated:
 *              "d":[v0,v2,v7,0b10000101]
 *
 *   IMPORTANT: The sparse format with bitmask is FIELD-PROVEN to work with
 *   SoloStorm. An earlier attempt at always-full d-arrays with a constant
 *   all-ones bitmask (mirroring the donour/racepi style) caused SoloStorm to
 *   spin. The firmware-native sparse format is what SoloStorm actually parses.
 *
 * CHANNEL 0 = "Interval"  (firmware: getUptimeAsInt = (int)getUptime())
 *   Device uptime in ms as a 32-bit int. Always populated (bit 0 always set).
 *
 * CHANNEL 1 = "Utc"  (firmware: GPS_get_UTC_time = getMillisSinceEpochAs-
 *   LongLong) → milliseconds since the Unix epoch as int64 (millis_t=int64_t).
 *   NOT milliseconds-within-day. We compute it from parsed GPS date+time via
 *   utcEpochMs(). OMIT (clear bit 1) until GPS date is valid (year≥2020) —
 *   the app tolerates a missing Utc channel, but a frozen 1970-epoch value
 *   causes wrong lap timestamps or disconnects.
 *   ▸ ESP32 newlib-nano %lld is unreliable — format int64 manually (appendI64).
 *   ▸ Keep meta min/max as double literals — float32 can't represent 1.3e12
 *     exactly, and dtostrf of a mangled float produces wrong meta bounds.
 *
 * CHANNELS 2-6 = lap/timing (LapCount, LapTime, Distance, ElapsedTime,
 *   CurrentLap). Present in meta so SoloStorm can display lap overlays, but
 *   never populated in samples (bits 2-6 always clear). SoloStorm computes
 *   its own laps from GPS track detection; our values would conflict.
 *
 * GPS CHANNELS 7-14: always populated when g.valid, never when not.
 *   Speed in mph (knots*1.15078) — unit is declared EXPLICITLY as "mph". An
 *   empty unit was tried and is FIELD-PROVEN WRONG: SoloStorm mangled the
 *   speed on both the 987.2 and 718. Explicit units are required (the
 *   wheel-speed channels work for exactly this reason). See the channel table.
 *   Altitude in ft (m*3.28084) — unit declared EXPLICITLY as "ft", same reason.
 *   GPSDOP: we send hAccM (horizontal accuracy in metres) as a DOP proxy —
 *   the LG290P does not output a separate DOP sentence in the default config.
 *   GPSQual: firmware GpsSignalQuality enum — 0=no fix, 1=2D, 2=3D, 3=DGNSS/RTK.
 *
 * IMU CHANNELS 15-18: always populated (IMU is always running; NaN on I2C
 *   failure but the ch() lambda handles that at the call site).
 *
 * CAN CHANNELS 19+: each populated only when its source value is not NaN.
 *   When the car is off / CAN bus silent, all CAN bits are clear and d shrinks.
 *   This is correct and intentional — sparse format handles it cleanly.
 *
 * META ENTRY FORMAT  (json_channelConfig in loggerApi.c)
 *   {"nm":<name>,"ut":<units>,"min":<n>,"max":<n>,"prec":<digits>,"sr":<Hz>}
 *   Channel names must match SoloStorm's internal map exactly: GPSSats,
 *   GPSDOP, GPSQual (NOT Satellites, DOP, GPSQuality).
 *
 * STREAMING BEHAVIOUR
 *   streaming is set true at onConnect and cleared at onDisconnect. It is
 *   also a live switch: startStreaming/stopStreaming commands toggle it. We
 *   do NOT gate the stream on receiving setTelemetry — a SoloStorm session
 *   was field-observed where data flowed before setTelemetry arrived and the
 *   connection held. All transmission is blocked until txSubscribed=true
 *   (the CCCD write), regardless of the streaming flag.
 *
 * SUBSCRIPTION SEQUENCE (field-verified healthy connect — June 2026)
 *   connect → onConnect (streaming=true) → MTU negotiate (512 observed) →
 *   app writes CCCD → onSubscribe (txSubscribed=true, sampleCount=0) →
 *   tick() drains ackQueue, handles pendingMeta/pendingVersion, then streams.
 *   First sample has embedded meta (sampleCount==0 at time of send).
 * ============================================================================
 */

#include "ble_racecapture.h"
#include "config.h"
#include "types.h"
#include "can_porsche718_extra.h"
#include <NimBLEDevice.h>
#include <esp_heap_caps.h>   // PSRAM allocation for the BLE TX message slot
#include <math.h>

// ── NUS UUIDs ─────────────────────────────────────────────────────────────────
#define NUS_SERVICE  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ── Module state ──────────────────────────────────────────────────────────────
static NimBLEServer* pServer     = nullptr;
static NimBLECharacteristic* pTxChar     = nullptr;
static bool bleClientConn               = false;
static volatile bool streaming          = false;
static volatile bool pendingMeta        = false;
static volatile bool pendingVersion     = false;

// RX line accumulator
static char rxBuf[256];
static int  rxLen = 0;

// Negotiated ATT MTU (updated by onMTUChange). Default 23 = BLE minimum until
// the central negotiates higher. Notification payload max is (MTU - 3).
static volatile uint16_t negotiatedMTU = 23;

float ble_packetHz = 0.0f;

// ── Per-send diagnostic logging gate ─────────────────────────────────────────
// The TX-mirror logs below run at the SAMPLE RATE (20 Hz). Over USB-CDC they
// were a real throughput killer: when the device runs untethered (in the car,
// no serial host draining the port) the CDC TX buffer fills and Serial.printf
// BLOCKS loop() up to the CDC timeout on EVERY send — collapsing the BLE rate
// to ~3-4 Hz and causing sample drops. (Belt-and-suspenders: setup() now also
// calls Serial.setTxTimeoutMs(0) so CDC writes never block when no host reads.)
// Keep this OFF for normal/field use; flip to 1 (or define in config.h) only
// when bench-debugging the wire protocol with USB attached.
#ifndef BLE_TX_DEBUG
#define BLE_TX_DEBUG 0
#endif
#if BLE_TX_DEBUG
  #define BLE_LOG(...) Serial.printf(__VA_ARGS__)
#else
  #define BLE_LOG(...) do {} while (0)
#endif

// Connection handle of the current central, captured at onConnect so we can
// request a fast connection interval once the link is up (see onSubscribe).
// 0xFFFF = none. THIS is the other half of the 20 Hz story: a central that
// negotiates a slow connection interval (Android/iOS commonly default to
// 30-100 ms for power saving) caps notifications at ~1 per interval — a ~100 ms
// interval pins the stream at ~10 Hz no matter how fast we push, and a slow
// initial interval during the getMeta handshake is the "only 3-4 Hz at first"
// symptom. We request 15-30 ms after subscribe to lift that cap. Some stacks
// may refuse — then throughput is whatever the central allows (no worse).
static volatile uint16_t bleConnHandle = 0xFFFF;

// Last time the (large) meta object was sent, for the getMeta-storm guard in
// tick(). Module-scope so onConnect can reset it to 0 — that guarantees the
// first meta of every new connection always fires (never gets rate-limited away,
// which would leave SoloStorm spinning).
static uint32_t bleLastMetaMs = 0;

// ── Channel table ─────────────────────────────────────────────────────────────
struct BleChannel { const char* nm; const char* ut; double mn, mx; int prec, sr; };

static const BleChannel kCh[] = {
    // Firmware: Interval = getUptimeAsInt() = (int)getUptime(), uptime ms as int32
    {"Interval",      "ms",      0,     2147483647.0f, 0,  1},
    // Firmware: Utc = GPS_get_UTC_time() = getMillisSinceEpochAsLongLong(), Unix epoch ms as int64
    // Max: year 2400 = 13,569,465,600,000 ms since epoch (double is exact; float32 would mangle it)
    {"Utc",           "ms",      0,  13569465600000.0, 0,  1},
    {"LapCount",      "",        0,           999,     0,  1},
    {"LapTime",       "Min",     0,        9999.9f,    4,  5},
    {"Distance",      "mi",      0,         999.9f,    3,  5},
    {"ElapsedTime",   "Min",     0,       99999.9f,    4,  1},
    {"CurrentLap",    "",        0,           999,     0,  1},
    {"Latitude",      "Degrees", -180,        180,     9, 20},
    {"Longitude",     "Degrees", -180,        180,     9, 20},
    // Speed: unit MUST be explicit. We send mph (knots*1.15078), so ut="mph".
    // FIELD BUG (June 2026): this was ut="" with a note claiming SoloStorm
    // "defaults to mph" on an empty unit. It does NOT — on both the 987.2 and
    // 718 the Speed gauge read WILDLY off while the explicitly-united wheel-speed
    // channels ("kph") read correctly. An unitless speed gives SoloStorm no
    // scale, so it mangles the value. The WiFi RaceCapture path (racecapture.cpp)
    // already declares this channel "mph" with the same value and reads right.
    // Mirror it here. DO NOT revert to an empty unit string.
    {"Speed",         "mph",     0,           200,     2, 20},
    {"Heading",       "Deg",     0,           360,     1, 20},
    // Altitude: same empty-unit bug as Speed (latent — verify on next run).
    // We send feet (m*3.28084), so ut="ft". Was "" which is the identical
    // unitless-mangle hazard. If you prefer metric, switch to value g.altMSL
    // (no *3.28084) AND ut="m" together — never an empty unit.
    {"Altitude",      "ft",      0,         13000,     1, 20},
    {"GPSSats",       "",        0,            20,     0,  1},
    {"GPSDOP",        "",        0,            20,     1,  1},
    {"GPSQual",       "",        0,             5,     0,  1},
    {"AccelX",        "G",      -3,             3,     2, 20},
    {"AccelY",        "G",      -3,             3,     2, 20},
    {"AccelZ",        "G",      -3,             3,     2, 20},
    {"Yaw",           "Deg/Sec",-120,         120,     0, 20},
    // ── CAN channels — ENGLISH UNITS, RaceCapture-standard naming ─────────────
    // RULES (enforced; do not violate without re-checking against firmware):
    //   • Label length: RaceCapture firmware char label[12] with
    //     validate_channel_config_label() rejecting strlen() >= 12. MAX 11 CHARS.
    //   • Unit length:  char units[8], strlen() >= 8 rejected. MAX 7 CHARS.
    //   • Units are ENGLISH everywhere (mph, F, psi, Deg). The CanData struct
    //     stays metric (decoders unchanged); conversion happens in sendSample()
    //     at the value-fill site, exactly like the SD CAN log does. Meta units
    //     here MUST match the converted values sent below — keep them in lockstep.
    //   • These labels are identical across BLE, WiFi (racecapture.cpp) and the
    //     SD CAN log header (sd_log.cpp) so one channel == one name everywhere.
    // Names that had to change to fit 11 chars (old → new):
    //   TargetTorque→TgtTorque, ActualTorque→ActTorque, VehicleSpeed→VehicleSpd,
    //   BrakeSwitch→BrakeSw, BrakeSwitch2→BrakeSw2, SteeringRate→SteerRate,
    //   PdkSelectorRaw→PdkSelRaw (was 14 — silently truncated before this fix).
    {"RPM",          "RPM",     0,   10000,  0, 20},  // engine RPM
    {"TPS",          "%",       0,     100,  1, 20},  // 987: actual TPS; 718: pedal-position alias for SoloStorm
    {"PedalPos",     "%",       0,     100,  1, 20},  // driver accelerator pedal request / PPS
    {"TgtTorque",    "%",       0,     100,  1, 20},  // 987.2 0x246 candidate (was TargetTorque)
    {"ActTorque",    "%",       0,     100,  1, 20},  // 987.2 0x246 candidate (was ActualTorque)
    {"VehicleSpd",   "mph",     0,     200,  1, 20},  // vehicle speed, kph→mph (was VehicleSpeed/kph)
    {"CoolantTmp",   "F",     -40,     300,  1,  1},  // coolant temp, C→F (was CoolantTemp/C)
    {"OilTemp",      "F",     -40,     300,  1,  1},  // oil temp, C→F
    {"OilPress",     "psi",     0,     145,  2,  5},  // oil pressure, bar→psi
    {"Brake",        "psi",     0,    2900,  1, 20},  // brake pressure, bar→psi (200 bar≈2900 psi)
    {"BrakeSw",      "",        0,       1,  0, 20},  // brake switch 0/1 (was BrakeSwitch)
    {"BrakeSw2",     "",        0,       1,  0, 20},  // second brake switch (was BrakeSwitch2)
    {"WheelSpdLF",   "mph",     0,     200,  1, 20},  // wheel speed, kph→mph
    {"WheelSpdRF",   "mph",     0,     200,  1, 20},
    {"WheelSpdLR",   "mph",     0,     200,  1, 20},
    {"WheelSpdRR",   "mph",     0,     200,  1, 20},
    {"Steering",     "Deg",  -600,     600,  1, 20},  // steering angle
    {"SteerRate",    "Deg/Sec", -2000, 2000, 1, 20},  // steering rate (was SteeringRate)
    {"FuelLevel",    "L",       0,     127,  0,  1},  // fuel level — 718 sends LITERS (7-bit 0-127); 987 unresolved (NaN)
    {"Kickdown",     "",        0,       1,  0, 20},  // kickdown 0/1
    {"BaroPress",    "psi",     0,      18,  2,  1},  // atmospheric pressure, kPa→psi (120 kPa≈17.4 psi)
    {"FuelTemp",     "F",     -40,     300,  1,  1},  // fuel temp, C→F
    {"EngineTemp",   "F",     -40,     300,  1,  1},  // engine/compartment temp, C→F
    {"Gear",         "",        0,       8,  0, 20},  // gear 0=unknown,1-7,8=R
    {"PdkGearRaw",   "",        0,     255,  0, 20},  // raw 0x440 data[1]
    {"PdkSelRaw",    "",        0,     255,  0, 20},  // raw 0x440 data[2] (was PdkSelectorRaw, 14 chars)
    // 718 supplemental channels (append-only: existing indices remain stable).
    {"IAT",          "F",     -40,     350,  1,  1},
    {"MAP",          "psi",     0,      75,  2, 20},  // absolute manifold pressure
    {"MAF",          "g/s",     0,    1000,  1, 20},  // absent until passive ID is identified
    {"CANLatG",      "G",      -3,       3,  3, 20},
    {"CANLongG",     "G",      -3,       3,  3, 20},
    {"CANYaw",       "Deg/Sec", -250,   250,  1, 20},
    {"ClutchPos",    "%",       0,     100,  1, 20},
    {"OutsideTmp",   "F",     -60,     160,  1,  1},
    {"Odometer",     "mi",      0,  700000,  1,  1},
    {"DriveMode",    "",        0,      15,  0,  5},
    {"PSMMode",      "",        0,      15,  0,  5},
    {"PDKState",     "",        0,       3,  0, 20},
    {"PDKFault",     "",        0,       1,  0, 20},
    {"TPMSLF",       "psi",     0,     100,  1,  1},
    {"TPMSRF",       "psi",     0,     100,  1,  1},
    {"TPMSLR",       "psi",     0,     100,  1,  1},
    {"TPMSRR",       "psi",     0,     100,  1,  1},
    {"TPMSTmpLF",    "F",     -40,     300,  1,  1},
    {"TPMSTmpRF",    "F",     -40,     300,  1,  1},
    {"TPMSTmpLR",    "F",     -40,     300,  1,  1},
    {"TPMSTmpRR",    "F",     -40,     300,  1,  1},
};
static const int kChCount = (int)(sizeof(kCh) / sizeof(kCh[0]));

static uint32_t sampleCount = 0;
static volatile bool txSubscribed = false;

// Pending-ACK ring buffer. processCommand() runs in the NimBLE host-task
// callback context, where notify() must not be called
// stalls the BLE stack and drops the link. So commands only ENQUEUE an ACK
// here; tick() drains the queue and sends from the safe loop() context.
static char     ackQueue[8][24];
static volatile uint8_t ackHead = 0, ackTail = 0;

static void enqueueAck(const char* cmd) {
    uint8_t next = (uint8_t)((ackHead + 1) % 8);
    if (next == ackTail) return;            // full — drop (should never happen)
    strncpy(ackQueue[ackHead], cmd, 23);
    ackQueue[ackHead][23] = '\0';
    ackHead = next;
}

// ── BLE TX engine: one message slot, newest-wins, atomic, non-blocking ───────
// Replaces the blocking per-chunk retry sender. Field evidence that drove each
// property (2026-08-22 drive + 2026-08-23 Langley + 2026-08-24 trunk test):
//
//  * NON-BLOCKING — the old sender waited up to 100 ms per chunk in loop();
//    a 9-chunk meta cost ~940 ms of loop() blockage, and the trunk-test IMU
//    log caught 60 stalls of 1000-1099 ms during handshake churn. gnss_loop
//    shares loop(): at open-sky NMEA volume those stalls overrun the 4 KB
//    UART buffer and destroy epochs. Here, tick() advances the in-flight
//    message one serialized notification at a time — a
//    congested notify fails fast and retries on the next pass (~ms later),
//    never parking loop().
//
//  * NEWEST-WINS, ONE SLOT — a sample that has not begun transmitting is
//    REPLACED by the next sample, never queued behind it. This preserves the
//    deliberate no-stale-backlog watchdog intent of the old 25 ms abandon
//    policy structurally (at most one sample of latency can ever exist)
//    while converting its failure mode from "discard" to "supersede":
//    SoloStorm keys every row to the sample's own Utc, so late delivery is
//    nearly free — loss is what hurt. The measured ~1% baseline loss was
//    ordinary 2-3-event RF hiccups colliding with the 25 ms deadline.
//
//  * ATOMIC — once a message's first chunk is on the air, the remainder is
//    never abandoned on congestion (only on disconnect, when there is no
//    receiver left to confuse). The old mid-message abandon emitted torn
//    JSON; sustained torn frames during a bad-mode minute is the leading
//    explanation for the SoloStorm crash that lost two race runs (2026-08-23).
//
// Critical messages (meta, version, ACKs) stage only into an idle slot; their
// pending flags/queues persist across passes until staging succeeds, so the
// getMeta-storm guard and drain semantics are unchanged. The slot buffer
// lives in PSRAM: 6 KB of internal SRAM is a third of the free pool.
static const size_t TX_MSG_CAP = 6144;          // largest message is the ~4.4 KB meta

static char*             txMsg         = nullptr; // PSRAM, allocated at init
static size_t            txLen         = 0;       // staged message length; 0 = idle
static size_t            txOff         = 0;       // next byte to send
static bool              txCritical    = false;   // meta/version/ACK vs sample
static bool              txIsMeta      = false;   // completion stamps the resend guard
static uint32_t          txLastChunkMs = 0;       // wire-serialization clock (see txPump)
static uint32_t          metaQuietMs   = 0;       // samples stay off the wire until this time
static volatile uint32_t txDelivered  = 0;       // SAMPLES fully notified
static volatile uint32_t txSuperseded = 0;       // samples replaced or blocked before send

// Stage a message into the slot. critical=true (meta/version/ACK): only an
// idle slot accepts it — an unstarted sample in the slot yields to it. A
// sample stages into an idle slot or replaces an unstarted sample; if the
// slot is mid-transmission or holds a critical message, the sample is
// dropped and counted (production outruns a congested link by design —
// newest data always takes priority over older undelivered data).
static bool txStage(const char* buf, size_t len, bool critical, bool isMeta = false) {
    if (!txMsg || len == 0 || len > TX_MSG_CAP) return false;
    if (txLen != 0) {
        bool unstartedSample = (txOff == 0 && !txCritical);
        if (!unstartedSample) {
            if (!critical) txSuperseded++;      // sample lost to an in-flight message
            return false;
        }
        txSuperseded++;                          // unstarted sample replaced
    }
    memcpy(txMsg, buf, len);
    txLen = len; txOff = 0; txCritical = critical; txIsMeta = isMeta;
    return true;
}

// Advance the in-flight message: at most ONE notification per call, never
// sooner than 4 ms after the previous one. This is the wire envelope the
// June 2026 trial-and-error sender proved against SoloStorm, restored here
// after its removal caused a hard handshake spin (2026-08-25): with chunks
// queued back-to-back, every multi-chunk meta was rejected by the client
// while single-chunk samples flowed — the strict one-notification-in-flight
// serialization is load-bearing client-facing behavior, not an internal
// timing detail. NimBLE itself is not the variable: notify() deep-copies
// into its own mbuf at call time (NimBLECharacteristic::sendValue), so the
// constraint lives in the client's reassembly, and the client is not ours
// to fix. Chunking is unchanged: min(MTU-3, 500) bytes per notify.
// notify() may internally wait up to 10 ms for a free mbuf (library
// behavior, present in the old sender too) — the bounded worst case here.
static void txPump() {
    if (txLen == 0) return;
    if (!bleClientConn || !pTxChar) {
        // Link is gone — there is no receiver; a partial message dies with
        // its connection and can never be seen torn.
        txLen = 0; txOff = 0; txIsMeta = false;
        return;
    }
    if (!txSubscribed) {
        // No subscription yet (or a CCCD rewrite in progress). An UNSTARTED
        // message WAITS here: the subscribe-time meta push legitimately
        // stages during the central's own CCCD setup race and must survive
        // to become the first message on the wire the moment the
        // subscription lands. Destroying it silently while its staging had
        // already stamped the resend guard ate the client's first real
        // getMeta and fed the 2026-08-25 spin. A message already partly
        // sent cannot complete for this subscription and is dropped.
        if (txOff > 0) { txLen = 0; txOff = 0; txIsMeta = false; }
        return;
    }
    uint32_t now = millis();
    if (txLastChunkMs != 0 && now - txLastChunkMs < 4u) return;
    size_t chunk = (negotiatedMTU > 23) ? (size_t)(negotiatedMTU - 3) : 20;
    if (chunk > 500) chunk = 500;
    size_t n = (txLen - txOff < chunk) ? (txLen - txOff) : chunk;
    pTxChar->setValue((const uint8_t*)(txMsg + txOff), n);
    if (!pTxChar->notify()) {
        BLE_LOG("   ↳ notify deferred off=%u/%u crit=%d\n",
                (unsigned)txOff, (unsigned)txLen, (int)txCritical);
        return;                                  // retry from txOff next pass
    }
    txLastChunkMs = millis();
    txOff += n;
    if (txOff >= txLen) {
        if (txIsMeta) {
            // The resend guard measures what the client actually RECEIVED:
            // stamp at completion, never at staging. A meta that never
            // finishes leaves the guard open, so the next getMeta is
            // answered immediately — the proven improving shape.
            bleLastMetaMs = millis();
            // Post-meta quiet: the old sender followed every meta with a
            // 15 ms yield and a skipped sample; SoloStorm accepted metas
            // under that envelope. Keep samples off the wire briefly so
            // the meta response is not tailgated by a sample notification.
            metaQuietMs = bleLastMetaMs + 15u;
        } else if (!txCritical) {
            txDelivered++;
        }
        txLen = 0; txOff = 0; txIsMeta = false;
    }
}

// Instrumentation accessors (lock-free volatile reads; consumed by sd_log).
uint8_t  ble_linkState()    { return txSubscribed ? 2 : (bleClientConn ? 1 : 0); }
uint32_t ble_txDelivered()  { return txDelivered; }
uint32_t ble_txSuperseded() { return txSuperseded; }


// appendFloat takes double so high-precision lat/lon (prec 9) is not degraded.
// float32 cast would quantise longitude to ~2.4 m, destroying RTK accuracy.
static int appendFloat(char* buf, int pos, int sz, double v, int prec) {
    if (pos >= sz - 20) return pos;
    char tmp[28];
    dtostrf(v, 1, prec, tmp);
    int n = strlen(tmp);
    if (pos + n < sz) { memcpy(buf + pos, tmp, n); pos += n; }
    return pos;
}

// Append int64 without relying on %lld (unreliable on ESP32 newlib-nano).
// Manual digit conversion guarantees the full epoch ms value (~1.78e12) is
// emitted correctly.
static int appendI64(char* buf, int pos, int sz, int64_t v) {
    if (pos >= sz - 24) return pos;
    char tmp[24];
    int n = 0;
    bool neg = (v < 0);
    uint64_t u = neg ? (uint64_t)(-(v + 1)) + 1ULL : (uint64_t)v;
    if (u == 0) tmp[n++] = '0';
    while (u > 0) { tmp[n++] = (char)('0' + (int)(u % 10)); u /= 10; }
    if (neg && pos < sz - 1) buf[pos++] = '-';
    while (n > 0 && pos < sz - 1) buf[pos++] = tmp[--n];
    return pos;
}

// Milliseconds since Unix epoch — matches firmware GPS_get_UTC_time() which
// returns getMillisSinceEpochAsLongLong(): GNSS week*604800000 + TOW_ms
// + 315964800000 (Jan 6 1980 → Jan 1 1970 offset), interpolated via uptime.
// Verified correct against Python datetime for multiple dates.
static int64_t utcEpochMs(const GnssData& g) {
    if (!g.valid || g.year < 2020) return 0;
    int y = g.year, m = g.month, d = g.day;
    y -= (m <= 2);
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    int64_t yoe = y - era * 400;
    int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int64_t days = era * 146097 + doe - 719468;     // days since 1970-01-01
    return (days * 86400LL
            + (int64_t)g.hour   * 3600
            + (int64_t)g.minute * 60
            + g.second) * 1000LL + g.millisecond;
}

// ── Protocol response builders ────────────────────────────────────────────────
static bool sendVersion() {
    char buf[160];
    int n = snprintf(buf, sizeof(buf),
        "{\"ver\":{\"name\":\"RCX\",\"fname\":\"%s\",\"major\":2,\"minor\":14,\"bugfix\":0}}\r\n",
        BLE_DEVICE_NAME);
    return txStage(buf, (size_t)n, /*critical=*/true);
}

static int writeChannelArray(char* buf, int pos, int sz) {
    if (pos < sz - 2) buf[pos++] = '[';
    for (int i = 0; i < kChCount; i++) {
        const BleChannel& ch = kCh[i];
        if (pos < sz - 200) {
            char mnStr[32], mxStr[32];
            dtostrf(ch.mn, 1, ch.prec, mnStr);
            dtostrf(ch.mx, 1, ch.prec, mxStr);
            pos += snprintf(buf + pos, sz - pos,
                "%s{\"nm\":\"%s\",\"ut\":\"%s\",\"min\":%s,\"max\":%s,\"prec\":%d,\"sr\":%d}",
                (i == 0) ? "" : ",",
                ch.nm, ch.ut, mnStr, mxStr, ch.prec, ch.sr);
        }
    }
    if (pos < sz - 2) buf[pos++] = ']';
    return pos;
}

// sendMeta — standalone channel-definition response.
// Wire shape: {"meta":[{"nm":...},...]}\r\n  (top-level, NOT s-wrapped).
// This is the FIELD-PROVEN shape. Sent in response to getMeta, and the same
// array is also embedded inline inside sample records at tick==0 / tick%100.
static bool sendMeta() {
    Serial.println("📋 sendMeta() called");
    // 66 channels produce about 4.4 kB of metadata; keep headroom so the JSON
    // array is never silently truncated before BLE chunking.
    static char buf[6144];
    int pos = 0, sz = (int)sizeof(buf);
    pos += snprintf(buf + pos, sz - pos, "{\"meta\":");
    pos = writeChannelArray(buf, pos, sz);
    pos += snprintf(buf + pos, sz - pos, "}\r\n");
    return txStage(buf, (size_t)pos, /*critical=*/true, /*isMeta=*/true);
}

// sendSample — emits one telemetry record.
// Wire shape: {"s":{"t":<tick>,"meta":[...opt...],"d":[...sparse...,bitmask]}}\r\n
//   "meta" is embedded when sampleCount==0 (post-subscribe) or sampleCount%100==0.
//   "d" is SPARSE: only populated channels appear; bit i in bitmask = channel i present.
//   Channels without data (GPS pre-fix, CAN silent) are simply omitted — bit clear.
// FIELD-PROVEN: this sparse+bitmask format works. Always-full arrays with a
// constant bitmask caused SoloStorm to spin (tested June 2026, reverted).
static void sendSample(const GnssData& g, const ImuData& m, const CanData& c) {
    static char buf[4096];
    int pos = 0, sz = (int)sizeof(buf);

    static constexpr int kMaskWordCount = (kChCount + 31) / 32;
    uint32_t bitmasks[kMaskWordCount] = {};
    int chIdx = 0;
    bool first = true;

    // Meta is sent standalone (on subscribe and getMeta), NOT embedded in samples.
    // Embedding meta in sample t=0 congested the BLE TX queue right when
    // SoloStorm's getMeta reply needed to fire, causing it to fail silently.
    pos += snprintf(buf + pos, sz - pos, "{\"s\":{\"t\":%lu,", (unsigned long)sampleCount);
    pos += snprintf(buf + pos, sz - pos, "\"d\":[");
    sampleCount++;

    auto addVal = [&](double v, int prec) {
        if (!first && pos < sz - 2) buf[pos++] = ',';
        pos = appendFloat(buf, pos, sz, v, prec);
        first = false;
    };

    auto addI64 = [&](int64_t v) {
        if (!first && pos < sz - 2) buf[pos++] = ',';
        pos = appendI64(buf, pos, sz, v);
        first = false;
    };

    auto markPopulated = [&]() {
        const int word = chIdx / 32;
        const int bit  = chIdx % 32;
        if (word < kMaskWordCount) {
            bitmasks[word] |= (uint32_t(1) << bit);
        }
    };

    auto ch = [&](bool populated, double v, int prec) {
        if (populated) { markPopulated(); addVal(v, prec); }
        chIdx++;
    };
    auto chI64 = [&](bool populated, int64_t v) {
        if (populated) { markPopulated(); addI64(v); }
        chIdx++;
    };

    const bool gpsOk  = g.valid;
    const bool timeOk = g.valid && (g.year >= 2020);
    const bool canOk  = !isnan(c.rpm);

    // Ch 0 — Interval: uptime ms as int (firmware: getUptimeAsInt = (int)getUptime())
    chI64(true, (int64_t)(uint32_t)millis());

    // Ch 1 — Utc: ms since Unix epoch (firmware: GPS_get_UTC_time() =
    //   getMillisSinceEpochAsLongLong(), confirmed int64 via millis_t = int64_t).
    //   Omit entirely if date not yet valid to avoid SoloStorm rejecting t=0.
    chI64(timeOk, utcEpochMs(g));

    // Ch 2-6 — lap/timing, not implemented
    chIdx += 5;

    // GPS channels — prec 9 = 0.11 mm lat / 0.09 mm lon (LG290P NMEA ceiling)
    ch(gpsOk, g.latitude,              9);
    ch(gpsOk, g.longitude,             9);
    ch(gpsOk, g.speedKnots * 1.15078,  2);  // knots → mph (channel unit "mph" — NOT empty)
    ch(gpsOk, g.headingDeg,            1);
    ch(gpsOk, g.altMSL * 3.28084,      1);  // m → ft (channel unit "ft" — NOT empty)
    ch(gpsOk, (double)g.numSV,         0);
    ch(gpsOk, g.hAccM,                 1);  // horizontal accuracy as GPSDOP proxy

    {   // GPSQual — firmware GpsSignalQuality enum: 0=no fix, 1=2D, 2=3D, 3=DGNSS/RTK
        double q = 0.0;
        if (g.valid) {
            if      (g.rtkType > 0)   q = 3.0;
            else if (g.fixType >= 3)  q = 2.0;
            else                      q = 1.0;
        }
        ch(gpsOk, q, 0);
    }

    // IMU — AccelX/Y/Z in G (prec 2); Yaw in Deg/Sec (prec 0 per firmware meta)
    ch(true, m.ax, 2);
    ch(true, m.ay, 2);
    ch(true, m.az, 2);
    ch(true, m.gz, 0);

    // ── CAN channels — values converted METRIC → ENGLISH to match the meta ────
    // CanData is metric (decoders unchanged); convert here via the helpers in
    // types.h so the wire values agree with the units declared in kCh[] above
    // AND with the WiFi/SD paths. The populated-flag still tests the underlying
    // metric field (NaN check is conversion-invariant; kphToMph(NAN)=NAN etc.).
    // SoloStorm keys its throttle trace to the standard TPS channel. The 718
    // does not yet have a confirmed throttle-plate broadcast, so expose its
    // accelerator-pedal position as TPS at the BLE transport layer only. The
    // 987.2 path continues to send its decoded actual TPS unchanged.
    Porsche718ExtraData x718;
    const bool is718 = can_getPorsche718Extra(x718);
    const float soloThrottlePct = is718 ? c.pedalRequestedPct : c.throttleActualPct;

    ch(!isnan(c.rpm),                c.rpm,                            0);
    ch(!isnan(soloThrottlePct),      soloThrottlePct,                  1);
    ch(!isnan(c.pedalRequestedPct),  c.pedalRequestedPct,             1);
    ch(!isnan(c.targetTorquePct),    c.targetTorquePct,               1);
    ch(!isnan(c.actualTorquePct),    c.actualTorquePct,               1);
    ch(!isnan(c.vehicleSpeedKph),    kphToMph(c.vehicleSpeedKph),     1);  // kph→mph
    ch(!isnan(c.coolantTempC),       cToF(c.coolantTempC),            1);  // °C→°F
    ch(!isnan(c.oilTempC),           cToF(c.oilTempC),                1);  // °C→°F
    ch(!isnan(c.oilPressBar),        barToPsi(c.oilPressBar),         2);  // bar→psi
    ch(!isnan(c.brakePressBar),      barToPsi(c.brakePressBar),       1);  // bar→psi
    ch(!isnan(c.brakeSwitch),        c.brakeSwitch,                   0);
    ch(!isnan(c.brakeSwitch2),       c.brakeSwitch2,                  0);
    ch(!isnan(c.wsFL_kph),           kphToMph(c.wsFL_kph),            1);  // kph→mph
    ch(!isnan(c.wsFR_kph),           kphToMph(c.wsFR_kph),            1);  // kph→mph
    ch(!isnan(c.wsRL_kph),           kphToMph(c.wsRL_kph),            1);  // kph→mph
    ch(!isnan(c.wsRR_kph),           kphToMph(c.wsRR_kph),            1);  // kph→mph
    ch(!isnan(c.steerAngleDeg),      c.steerAngleDeg,                 1);
    ch(!isnan(c.steerRateDegPerSec), c.steerRateDegPerSec,            1);
    ch(!isnan(c.fuelLevel),          c.fuelLevel,                     0);  // 718 liters (integer)
    ch(!isnan(c.kickdown),           c.kickdown,                      0);
    ch(!isnan(c.atmosphericKpa),     kpaToPsi(c.atmosphericKpa),      2);  // kPa→psi
    ch(!isnan(c.fuelTempC),          cToF(c.fuelTempC),               1);  // °C→°F
    ch(!isnan(c.engineTempC),        cToF(c.engineTempC),             1);  // °C→°F
    ch(c.gear > 0,                   (double)c.gear,                  0);
    ch(!isnan(c.pdkGearRaw),         c.pdkGearRaw,                    0);
    ch(!isnan(c.pdkSelectorRaw),     c.pdkSelectorRaw,                0);

    // 718-only data is sparse and disappears entirely on the 987.2 profile.
    // This leaves the validated shared/987 channel values and decoders untouched.
    ch(is718 && !isnan(x718.intakeAirTempC),         cToF(x718.intakeAirTempC),              1);
    ch(is718 && !isnan(x718.manifoldAbsPressureBar), barToPsi(x718.manifoldAbsPressureBar),  2);
    ch(is718 && !isnan(x718.massAirFlowGps),         x718.massAirFlowGps,                    1);
    ch(is718 && !isnan(x718.canLateralAccelG),       x718.canLateralAccelG,                  3);
    ch(is718 && !isnan(x718.canLongitudinalAccelG),  x718.canLongitudinalAccelG,             3);
    ch(is718 && !isnan(x718.canYawRateDegPerSec),    x718.canYawRateDegPerSec,               1);
    ch(is718 && !isnan(x718.clutchPositionPercent),  x718.clutchPositionPercent,             1);
    ch(is718 && !isnan(x718.outsideTempC),           cToF(x718.outsideTempC),                1);
    ch(is718 && !isnan(x718.odometerKm),             x718.odometerKm * 0.621371f,            1);
    ch(is718 && !isnan(x718.driveMode),              x718.driveMode,                         0);
    ch(is718 && !isnan(x718.psmMode),                x718.psmMode,                           0);
    ch(is718 && !isnan(x718.pdkState),               x718.pdkState,                          0);
    ch(is718 && !isnan(x718.pdkNoDriveOrFault),      x718.pdkNoDriveOrFault,                 0);
    ch(is718 && !isnan(x718.tpmsFrontLeftPsi),       x718.tpmsFrontLeftPsi,                  1);
    ch(is718 && !isnan(x718.tpmsFrontRightPsi),      x718.tpmsFrontRightPsi,                 1);
    ch(is718 && !isnan(x718.tpmsRearLeftPsi),        x718.tpmsRearLeftPsi,                   1);
    ch(is718 && !isnan(x718.tpmsRearRightPsi),       x718.tpmsRearRightPsi,                  1);
    ch(is718 && !isnan(x718.tpmsTempFrontLeftC),     cToF(x718.tpmsTempFrontLeftC),          1);
    ch(is718 && !isnan(x718.tpmsTempFrontRightC),    cToF(x718.tpmsTempFrontRightC),         1);
    ch(is718 && !isnan(x718.tpmsTempRearLeftC),      cToF(x718.tpmsTempRearLeftC),           1);
    ch(is718 && !isnan(x718.tpmsTempRearRightC),     cToF(x718.tpmsTempRearRightC),          1);

    for (int i = 0; i < kMaskWordCount; ++i) {
        if (!first && pos < sz - 20) pos += snprintf(buf + pos, sz - pos, ",%lu", (unsigned long)bitmasks[i]);
        else if   (pos < sz - 20)    pos += snprintf(buf + pos, sz - pos, "%lu",  (unsigned long)bitmasks[i]);
    }
    if (pos < sz - 8) pos += snprintf(buf + pos, sz - pos, "]}}\r\n");

    // Fire on first sample of each connection, then every 10 s
    static uint32_t lastDbg = 0;
    if (sampleCount == 1 || millis() - lastDbg > 10000) {
        lastDbg = millis();
        char epochStr[32];
        int ep = appendI64(epochStr, 0, sizeof(epochStr), utcEpochMs(g));
        epochStr[ep] = '\0';
        Serial.printf("📤 GPS valid=%d %04u-%02u-%02u %02u:%02u:%02u.%03u epochMs=%s\n",
                      (int)g.valid, g.year, g.month, g.day,
                      g.hour, g.minute, g.second, g.millisecond, epochStr);
        Serial.printf("   BLE_TX: %.*s%s\n", pos > 300 ? 300 : pos, buf,
                      pos > 300 ? " ...[truncated]" : "");
    }

    txStage(buf, (size_t)pos, /*critical=*/false);
}

// ── Command processing ────────────────────────────────────────────────────────
// The RaceCapture protocol requires an ACK for every command received:
//   {"commandName":{"rc":1}}\r\n  for success (API_SUCCESS = 1 in firmware)
//   {"commandName":{"rc":0}}\r\n  for unknown (API_ERROR_UNKNOWN_MSG = 0)
// Commands that write their own full response (getMeta, getVer) skip this
// because they are declared API_SUCCESS_NO_RETURN in the firmware.

static bool sendResult(const char* cmd, int rc) {
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "{\"%s\":{\"rc\":%d}}\r\n", cmd, rc);
    return txStage(buf, (size_t)n, /*critical=*/true);
}

// Extract the top-level key from {"commandName":...}
static void extractCmd(const char* json, char* out, size_t outLen) {
    const char* p = strchr(json, '"');
    if (!p) { strncpy(out, "unknown", outLen); return; }
    p++;
    const char* e = strchr(p, '"');
    if (!e) { strncpy(out, "unknown", outLen); return; }
    size_t n = (size_t)(e - p);
    if (n >= outLen) n = outLen - 1;
    memcpy(out, p, n); out[n] = '\0';
}

static void processCommand(const char* json) {
    // Commands that write their own full response (API_SUCCESS_NO_RETURN in
    // firmware) — set a flag; tick() sends from loop() context. NO rc ACK.
    if (strstr(json, "getMeta"))  { pendingMeta    = true; return; }
    if (strstr(json, "getVer"))   { pendingVersion = true; return; } // matches "getVer" AND "getVersion"

    // Stream control — toggle the flag; still fall through to enqueue rc ACK.
    if (strstr(json, "startStreaming")) streaming = true;
    if (strstr(json, "stopStreaming"))  streaming = false;

    // setTelemetry is SoloStorm's actual start/stop command — it does NOT send
    // startStreaming. rate>0 = begin streaming AND (RacePi handshake) expects a
    // meta reply as the completion signal; rate:0 = stop. Drive streaming here
    // explicitly rather than relying solely on onConnect having set it — a
    // field log showed setTelemetry:50 arriving with no sample flow because
    // streaming state wasn't (re)asserted by the command. (Still ACKed below.)
    if (strstr(json, "setTelemetry")) {
        // crude rate sniff: "rate":0 (with optional spaces) means stop.
        const char* r = strstr(json, "rate");
        bool rateZero = false;
        if (r) { const char* c = r + 4; while (*c && (*c==':'||*c=='"'||*c==' ')) c++; rateZero = (*c=='0'); }
        if (rateZero) {
            streaming = false;
        } else {
            streaming   = true;
            pendingMeta = true;   // rate>0 → answer with meta (handshake completion)
        }
    }

    // All other commands (setTelemetry, startStreaming, stopStreaming, any
    // unknown command including getStatus/getCapabilities which are not
    // implemented) get {"cmd":{"rc":1}}. We send rc:1 for everything —
    // SoloStorm only checks that a reply arrives, not the value.
    // NEVER stage or pump BLE TX here — this runs in the NimBLE host-task callback.
    char cmd[24]; extractCmd(json, cmd, sizeof(cmd));
    enqueueAck(cmd);
}

// ── TX callbacks — subscription gate ─────────────────────────────────────────
class TxCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo& connInfo, uint16_t subValue) override {
        txSubscribed = (subValue & 0x0001);
        sampleCount  = 0;
        // Send standalone meta immediately on subscribe — before any sample.
        // Previously meta was embedded inside sample t=0, which congested the
        // BLE TX queue right at the moment SoloStorm sends {"getMeta":null}.
        // The getMeta reply then hit a full queue, failed all retries, and was
        // silently dropped — SoloStorm spun forever. Sending meta standalone
        // here (via tick() draining pendingMeta) clears it before samples flow.
        if (txSubscribed) {
            pendingMeta = true;

            // ── Request a FAST connection interval (the 20 Hz enabler) ───────
            // A BLE peripheral can notify at most ~once per connection interval.
            // Centrals commonly default to a slow interval (Android power-save,
            // iOS 30 ms+), which caps the stream well below 20 Hz regardless of
            // how fast loop() pushes — this is the real reason it "maxed at
            // 10 Hz". We ask for 15-30 ms (12-24 × 1.25 ms), latency 0. Done
            // HERE (post-discovery, post-CCCD) not at onConnect, because
            // requesting tight params mid-discovery makes some stacks drop the
            // link. The central may refuse or clamp the request; if so we simply
            // run at whatever it grants — never worse than before. FIELD-VERIFIED
            // (2026-08-22 drive, arrival-lattice fit): the central GRANTS the
            // 15 ms request — the interval is not the delivery bottleneck.
            // (This is a conn-param request only — it does NOT touch the GATT
            //  table, so it can't shift handles or break Android's attribute cache.)
            //
            // SUPERVISION TIMEOUT: 5 s (500 × 10 ms), matching common central
            // defaults. The timeout is the central's patience before declaring
            // the link dead. FIELD EVIDENCE (2026-08-22 drive): the master
            // routinely leaves this link idle for gaps up to 3.3 s while its
            // radio serves other duties, and the link survives — meaning the
            // central kept its own multi-second timeout rather than honoring an
            // earlier 400 ms request. Never request an aggressive timeout here:
            // a central that HONORS 400 ms turns every sub-second radio gap
            // into a hard mid-run disconnect.
            if (pServer && connInfo.getConnHandle() != 0xFFFF) {
                pServer->updateConnParams(connInfo.getConnHandle(),
                                          12  /*min 15 ms*/, 24 /*max 30 ms*/,
                                          0   /*latency*/,  500 /*timeout 5 s*/);
            }
        }
        // Granted-parameter evidence: connInfo here reflects the live link, so
        // this one line answers "what interval/timeout did the central actually
        // give us" — a number that was never observable in field logs before.
        // (Serial.printf is safe in host-task context per the CONTEXT RULE.)
        Serial.printf("📡 BLE TX %s (subValue=0x%04X, MTU=%u, interval=%.2f ms, latency=%u, timeout=%u ms)\n",
                      txSubscribed ? "SUBSCRIBED ✅" : "unsubscribed",
                      subValue, (unsigned)negotiatedMTU,
                      connInfo.getConnInterval() * 1.25f,
                      (unsigned)connInfo.getConnLatency(),
                      (unsigned)connInfo.getConnTimeout() * 10u);
    }
};

// ── BLE callbacks ─────────────────────────────────────────────────────────────
class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
        std::string val = c->getValue();
        for (char ch : val) {
            if (ch == '\n' || ch == '\r') {
                if (rxLen > 0) {
                    rxBuf[rxLen] = '\0';
                    // ONLY delta from the field-proven working build: log every
                    // inbound command verbatim. Serial-only, host-task-safe
                    // (printf is fine here; only notify/vTaskDelay are not).
                    Serial.printf("\xF0\x9F\x93\xA5 BLE RX: %s\n", rxBuf);
                    processCommand(rxBuf);
                    rxLen = 0;
                }
            } else if (rxLen < (int)sizeof(rxBuf) - 1) {
                rxBuf[rxLen++] = ch;
            }
        }
    }
};

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo& connInfo) override {
        bleClientConn  = true;
        txSubscribed   = false;
        streaming      = true;
        rxLen          = 0;
        pendingMeta    = false;
        pendingVersion = false;
        bleConnHandle  = connInfo.getConnHandle();  // for the conn-param request
        bleLastMetaMs  = 0;                          // first meta must always fire
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            status.bleConnected = true;
            xSemaphoreGive(dataMutex);
        }
        Serial.println("✅ BLE: client connected");
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        bleClientConn = false;
        txSubscribed  = false;
        streaming     = false;
        negotiatedMTU = 23;
        bleConnHandle = 0xFFFF;
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            status.bleConnected = false;
            xSemaphoreGive(dataMutex);
        }
        Serial.println("🔌 BLE: disconnected — re-advertising");
        NimBLEDevice::startAdvertising();
    }
    void onMTUChange(uint16_t mtu, NimBLEConnInfo&) override {
        negotiatedMTU = mtu;
        Serial.printf("📏 BLE: MTU negotiated = %u (payload %u bytes)\n",
                      mtu, (unsigned)(mtu - 3));
    }
};

// ── Public init ───────────────────────────────────────────────────────────────
void ble_racecapture_init() {
    // TX message slot lives in PSRAM: TX_MSG_CAP bytes of internal SRAM would
    // be a third of the steady-state free pool. notify() copies out of it into
    // the stack's own buffers, so PSRAM access latency is invisible on the air.
    txMsg = (char*)heap_caps_malloc(TX_MSG_CAP, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!txMsg) txMsg = (char*)heap_caps_malloc(TX_MSG_CAP, MALLOC_CAP_8BIT);
    if (!txMsg) Serial.println("⛔ BLE: TX slot allocation failed — BLE TX disabled");

    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(512);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // ── GATT table — order is fixed; see header comment before changing ──────
    // Manual 0x1800 GAP service. NimBLE registers GAP+GATT (0x1800/0x1801)
    // automatically, so this IS a duplicate service in the attribute table.
    // That's technically non-conformant, but it was present in the first build
    // that connected to SoloStorm, and removing it shifts all subsequent
    // attribute handles — breaking existing Android GATT caches. Leave it.
    {
        NimBLEService* svc = pServer->createService("1800");
        svc->createCharacteristic("2A00", NIMBLE_PROPERTY::READ)->setValue(BLE_DEVICE_NAME);
        uint16_t ap = 0;
        svc->createCharacteristic("2A01", NIMBLE_PROPERTY::READ)->setValue((uint8_t*)&ap, 2);
        svc->start();
    }
    {
        NimBLEService* svc = pServer->createService("180A");
        auto mk = [&](const char* uuid, const char* v) {
            svc->createCharacteristic(uuid, NIMBLE_PROPERTY::READ)->setValue(v);
        };
        mk("2A24", "RCX Datalogger");
        mk("2A25", "RCX_BLE_01");
        mk("2A26", "2.14.0");
        mk("2A29", "RCX Engineering");
        svc->start();
    }

    NimBLEService* nus = pServer->createService(NUS_SERVICE);
    pTxChar = nus->createCharacteristic(NUS_TX_CHAR, NIMBLE_PROPERTY::NOTIFY);
    pTxChar->setCallbacks(new TxCallbacks());
    auto* rx = nus->createCharacteristic(NUS_RX_CHAR,
                    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rx->setCallbacks(new RxCallbacks());
    nus->start();

    // Primary ADV packet budget is 31 bytes: flags(3) + name(2+14) + 180A(4)
    // = 23 bytes, leaving only 8 bytes — not enough for the 18-byte 128-bit
    // NUS UUID. NUS rides in the scan response instead. The device name and
    // 180A UUID stay in the primary packet so SoloStorm's device-name filter
    // finds us during scanning before it needs to issue a scan request.
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->setName(BLE_DEVICE_NAME);
    adv->addServiceUUID(NUS_SERVICE);
    adv->addServiceUUID("180A");
    adv->enableScanResponse(true);
    adv->start();

    Serial.printf("✅ BLE: advertising as \"%s\"\n", BLE_DEVICE_NAME);
}

// ── Public tick — call at ~25 Hz from loop() ─────────────────────────────────
void ble_racecapture_tick(const GnssData& g, const ImuData& m, const CanData& c) {
    if (!bleClientConn) {
        // A message staged for a connection dies with that connection: never
        // carry it into the next one (it would arrive stale, and if partially
        // sent, torn). Loop context only — no race with the host task.
        txLen = 0; txOff = 0; txIsMeta = false;
        return;
    }

    // Heartbeat: confirm tick() is actually running while connected. Gated
    // behind BLE_TX_DEBUG (default off) — at 1 Hz it was burying more important
    // boot/GPS/NTRIP lines in the serial log.
    static uint32_t lastBeat = 0;
    if (millis() - lastBeat >= 1000) {
        lastBeat = millis();
        BLE_LOG("💓 tick alive | streaming=%d txSub=%d pendMeta=%d ackH=%d ackT=%d\n",
                (int)streaming, (int)txSubscribed, (int)pendingMeta,
                (int)ackHead, (int)ackTail);
    }

    // Advance whatever message is in flight — a bounded, non-blocking step.
    // The engine sends at most one serialized notification and fails fast
    // on congestion, so loop() is never parked here regardless of link state.
    txPump();

    // Stage at most one new message per pass, priority ACK → version → meta →
    // sample. Staging fails harmlessly while another message is in flight; a
    // pending flag or queue entry simply survives to the next pass (~ms away).
    // ACKs drain first, then version/meta. setTelemetry therefore gets its
    // rc:1 before the meta fires — field-observed and non-problematic.
    // (An ordered FIFO that serialised all responses was tried and reverted;
    // the flags+drain approach is simpler and proven to work.)
    if (ackTail != ackHead) {
        if (sendResult(ackQueue[ackTail], 1)) {   // {"cmd":{"rc":1}}\r\n (API_SUCCESS)
            BLE_LOG("🎫 ACK staged: %s\n", ackQueue[ackTail]);
            ackTail = (uint8_t)((ackTail + 1) % 8);
        }
    } else if (pendingVersion) {
        if (sendVersion()) { BLE_LOG("🏷️ version staged\n"); pendingVersion = false; }
    } else if (pendingMeta) {
        // getMeta-storm guard. SoloStorm fires {"getMeta":null} repeatedly
        // during the handshake. The FIRST meta (on subscribe) always fires
        // because onConnect resets bleLastMetaMs to 0; after that, at most one
        // resend per 250 ms — extra requests inside the window are discarded
        // because the client already has the channel map. The guard is
        // stamped by txPump at meta COMPLETION, so it measures what the
        // client received, never what was merely staged. A request arriving
        // while a meta is already in flight is being answered by that very
        // meta — discard it. A meta that cannot stage yet (a short critical
        // message in flight) keeps its flag and retries next pass.
        if (txIsMeta && txLen != 0) {
            pendingMeta = false;                  // the in-flight meta is the answer
        } else if (bleLastMetaMs != 0 && millis() - bleLastMetaMs < 250) {
            pendingMeta = false;                  // storm request — discard
        } else if (sendMeta()) {
            pendingMeta = false;
        }
    }

    if (!streaming)     return;

    // Epoch-locked cadence. The sample stream is driven by the GNSS epoch
    // itself (gps.epochSeq, which increments once per COMPLETED epoch after
    // its RMC is applied — the same key sdlog_push_if_new() trusts), not by
    // a free-running timer. A timer gate ("50 ms since my last send") always
    // runs slightly long — 50 ms plus the loop residual at the moment the
    // gate is checked — and resetting it to now keeps that residual instead
    // of correcting it. Measured on the 2026-08-26 commute: mean send
    // interval 50.312 ms against the receiver's hard 50.0 ms epoch grid, so
    // the accumulated lag crossed one full epoch every 160.5 epochs and that
    // epoch's sample was never built — a periodic miss every 8.0 s (period
    // fold R=0.638 vs 0.180 noise floor), the ~0.6-0.9% loss class observed
    // on every drive and every central since June. Epoch-driven sending
    // cannot drift: one completed epoch in, one sample out.
    // The 100 ms fallback keeps the stream alive if the GNSS goes silent
    // (module or UART fault — a tunnel does not qualify, the LG290P holds
    // 20 Hz output with zero satellites); it is deliberately far from the
    // 50 ms grid so it can never beat against real epochs, and a 10 Hz
    // stream of stale-position samples is itself the fault signature.
    static uint32_t lastEpochSeq = 0;
    static uint32_t lastSend     = 0;
    uint32_t now = millis();
    bool newEpoch = (g.epochSeq != lastEpochSeq);
    if (!newEpoch && now - lastSend < 100u) return;

    if (!txSubscribed) return;  // block stream until subscription confirmed

    // Meta wire adjacency — the rest of the June-proven envelope: no sample
    // notification is wedged between a getMeta and its meta, around a meta
    // in flight, or inside the 15 ms quiet after one. The old blocking
    // sender enforced all three implicitly (the send owned the wire and the
    // post-send yield skipped a sample); the engine enforces them
    // explicitly. The 50 ms cadence stamp is taken AFTER this barrier, so a
    // deferred sample is retried on the next pass (~ms later) and goes out
    // the instant the wire frees, rather than forfeiting its whole slot.
    // Costs a few tens of ms of stream latency per handshake — less than the
    // old sender's outright skip — and nothing once the client stops asking
    // for metas, which is the whole of a normal session.
    if (pendingMeta || (txIsMeta && txLen != 0) || (int32_t)(millis() - metaQuietMs) < 0) {
        return;                       // epoch still pending; retried next pass
    }
    lastEpochSeq = g.epochSeq;
    lastSend     = now;

    sendSample(g, m, c);
    txPump();   // first chunk of the fresh sample leaves on the pass that built it

    // Delivered-truth rate: blePacketHz counts samples FULLY notified to the
    // central (txDelivered), not send attempts — the display and log now show
    // what SoloStorm actually receives.
    static uint32_t lastDlv = 0, lastRate = 0;
    if (now - lastRate >= 2000) {
        ble_packetHz = (ble_txDelivered() - lastDlv) / ((now - lastRate) / 1000.0f);
        lastDlv = ble_txDelivered();
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
            status.blePacketHz = ble_packetHz;
            xSemaphoreGive(dataMutex);
        }
        lastRate = now;
    }
}

bool ble_racecapture_connected() { return bleClientConn; }
