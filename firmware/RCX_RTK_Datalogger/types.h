#pragma once
/*
 * types.h — Shared data structures for RCX Datalogger
 * =====================================================
 * All modules include this file.  Global instances are defined
 * in RCX_RaceCapture.ino and declared extern here.
 *
 * Concurrency note: every field is protected by dataMutex.
 * Always take the mutex before reading or writing any struct.
 * Short critical sections only — do NOT hold mutex across I/O.
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ── GNSS fix data ────────────────────────────────────────────────────────────
struct GnssData {
    double   latitude    = 0.0;
    double   longitude   = 0.0;
    double   altMSL      = 0.0;      // metres, MSL
    double   altWGS84    = 0.0;      // metres, WGS84 ellipsoid
    double   geoidSepM   = 0.0;      // GGA field 11: ellipsoid − MSL
    double   speedKnots  = 0.0;
    double   headingDeg  = 0.0;
    double   hAccM       = 99.9;     // horizontal accuracy metres
    double   vAccM       = 99.9;
    bool     epeValid    = false;    // true = hAccM/vAccM from PQTMEPE
    uint32_t epeMillis   = 0;        // millis() of last PQTMEPE message
    uint8_t  fixType     = 0;        // 0=none 2=2D 3=3D
    uint8_t  rtkType     = 0;        // 0=none 1=float 2=fixed
    // PPP (Precise Point Positioning) — Galileo E6 HAS / BeiDou B2b, no base station needed.
    // A PPP fix reports GGA quality 5, the SAME code as RTK FLOAT, so rtkType alone cannot
    // tell them apart. pppActive is the disambiguation, driven by diffStationId:
    //   9001 = PPP from B2b, 9002 = PPP from E6 HAS, anything else = a real RTCM base.
    // Precedence when reporting to a human: FIXED > PPP > FLOAT > 3D. (A PPP solution is
    // decimetre-class; an RTK FLOAT off our own base is usually better, but a FLOAT is only
    // reported as FLOAT when corrections are actually flowing — so the two never collide.)
    bool     pppActive   = false;    // true = quality-5 fix is a PPP solution, not RTK float
    uint16_t diffStationId = 0;      // GGA field 14: differential reference station ID
    double   diffAgeS    = -1.0;     // GGA field 13: age (s) of differential/RTK corrections in use; -1 = none
    uint8_t  numSV       = 0;
    uint16_t year        = 2026;
    uint8_t  month       = 5;
    uint8_t  day         = 22;
    uint8_t  hour        = 0;
    uint8_t  minute      = 0;
    uint8_t  second      = 0;
    uint16_t millisecond = 0;        // sub-second from NMEA
    uint32_t timeUpdateMillis = 0;   // host millis() when GNSS time last updated
    uint32_t velUpdateMillis  = 0;   // host millis() when speed/heading last committed (RMC).
                                     // (millis() - velUpdateMillis) IS the speed-channel age:
                                     // logged as spd_age_ms, available to BLE for validity.
    uint32_t epochSeq         = 0;   // increments once per epoch, AFTER that epoch's RMC has
                                     // been applied → "epoch complete". sdlog_push_if_new()
                                     // keys on this instead of the timestamp so a row can no
                                     // longer be snapshotted in the GGA→RMC gap (the stale
                                     // speed/heading race found in the Langley logs).
    float    pdop = 0.0f;            // ┐ from $GNGSA at 1 Hz (0 = none seen yet).
    float    hdop = 0.0f;            // │ Identical across the per-system GSA group,
    float    vdop = 0.0f;            // ┘ so any sentence of the group is authoritative.
    char     gsaSats[128] = "";      // used-sat IDs by system, "G:05 09 15/R:68 79" —
                                     // no commas by construction, safe as one CSV field
    uint32_t gsaMillis = 0;          // host millis() of last GSA update
    bool     valid       = false;
};

// ── IMU data (QMI8658) ───────────────────────────────────────────────────────
struct ImuData {
    float ax = 0.0f;   // g (longitudinal: positive = forward)
    float ay = 0.0f;   // g (lateral: positive = right)
    float az = 1.0f;   // g (vertical: positive = up)
    float gx = 0.0f;   // deg/s
    float gy = 0.0f;
    float gz = 0.0f;
};

// ── CAN bus decoded channels ───────────────────────────────────────────────
// NAN = not yet received, unmapped, or bus silent. Boolean-style channels are
// stored as floats so the stream/log layers can emit null when unavailable.
// Candidate channels are logged for validation, not for control decisions.
struct CanData {
    float   rpm                = NAN;  // engine RPM                         (987:0x242 / 718:0x105)
    float   throttleActualPct  = NAN;  // TPS actual throttle plate %        (987:0x242 / 718:0x105)
    float   pedalRequestedPct  = NAN;  // PPS / driver pedal demand %        (987:0x246)
    float   targetTorquePct    = NAN;  // candidate target torque %          (987:0x246)
    float   actualTorquePct    = NAN;  // candidate actual torque %          (987:0x246)
    float   vehicleSpeedKph    = NAN;  // vehicle speed km/h                 (987:0x14A / 718:0x30B)
    float   coolantTempC       = NAN;  // engine coolant temperature °C      (987:0x245 / 718:0x640)
    float   oilTempC           = NAN;  // engine oil temp °C                 (987:0x441 / 718:0x640)
    float   oilPressBar        = NAN;  // oil pressure bar                   (987:0x441 / 718:0x107)
    float   brakePressBar      = NAN;  // brake pressure bar candidate       (987:0x44B / 718:0x106)
    float   brakeSwitch        = NAN;  // brake switch, 0/1 when mapped      (718:0x106; 987 unresolved)
    float   brakeSwitch2       = NAN;  // second brake switch, 0/1           (987 unresolved)
    float   wsFL_kph           = NAN;  // wheel speed front-left  km/h       (987:0x24A / 718:0x103)
    float   wsFR_kph           = NAN;  // wheel speed front-right km/h       (987:0x24A / 718:0x103)
    float   wsRL_kph           = NAN;  // wheel speed rear-left   km/h       (987:0x24A / 718:0x103)
    float   wsRR_kph           = NAN;  // wheel speed rear-right  km/h       (987:0x24A / 718:0x103)
    float   steerAngleDeg      = NAN;  // steering angle degrees             (987:0x0C2 / 718:0x086)
    float   steerRateDegPerSec = NAN;  // steering rate deg/s                (987:0x0C2 / 718:0x086)
    float   fuelLevel          = NAN;  // fuel level; units map-specific     (718 liters, 987 unresolved)
    float   kickdown           = NAN;  // kickdown active, 0/1               (987:0x246)
    float   atmosphericKpa     = NAN;  // atmospheric/barometric pressure    (987:0x246)
    float   fuelTempC          = NAN;  // fuel temperature °C                (987 unresolved)
    float   engineTempC        = NAN;  // engine/compartment temp candidate  (987:0x441 candidate)
    uint8_t gear               = 0;    // gear candidate: 0=unknown/N/P, 1-7=fwd, 8=R
    float   pdkGearRaw         = NAN;  // raw 0x440 data[1], logged to identify 7th gear
    float   pdkSelectorRaw     = NAN;  // raw 0x440 data[2]
};

// ── System / connection status ───────────────────────────────────────────────
struct SystemStatus {
    bool     wifiConnected   = false;
    bool     wifiAttempting  = false;
    bool     ntripConnected  = false;
    bool     bleConnected    = false;
    bool     rcConnected     = false;   // RaceCapture TCP client connected
    bool     canBusOk        = false;   // TWAI driver init succeeded (built-in CAN + SN65HVD230)
    char     wifiSSID[33]    = "---";
    char     ipAddress[20]   = "---";
    char     mountpoint[64]  = "---";
    char     casterHost[40]  = "---";
    float    ntripDistanceKm = -1.0f;
    int8_t   ntripCarrier    = -1;   // -1=unknown 1=L1 2=L1+L2
    bool     ntripVRS        = false;
    float    blePacketHz     = 0.0f;
    float    gnssHz          = 0.0f;
    float    canHz           = 0.0f;  // measured CAN frame rate
    // ── Device temperatures (°C internally; converted to °F at every output) ──
    // NAN = not yet read or transient sensor fault → outputs emit blank / "--".
    //   espTempC — ESP32-S3 SILICON die (dieTempReadWideC in the .ino, 50-125°C
    //              range — see its own comment for why). Self-heats well above
    //              ambient; good for trend and catching the die nearing throttle.
    //   imuTempC — QMI8658 register. The IMU barely self-heats, so this tracks the
    //              board/enclosure heat-soak — the number that matters for the PETG
    //              case (Tg ≈ 80 °C) and the LCD bond. NOT ambient air temp.
    float    espTempC        = NAN;
    float    imuTempC        = NAN;
};

// ── Global shared instances — defined in RCX_RaceCapture.ino ────────────────
extern GnssData      gps;
extern ImuData       imu;
extern CanData       can;
extern SystemStatus  status;
extern SemaphoreHandle_t dataMutex;

// ── RTCM flow counter — written by wifiNtripTask, read for diagnostics ───────
extern volatile uint32_t rtcmBytesTotal;

// ── Last reset reason — captured ONCE in setup() (esp_reset_reason decoded to
// text), read by sd_log (boot_events.csv) and webserver (/status). This is the
// hinge for diagnosing the field restarts: it says WHAT kind of reset happened
// (brownout / panic / task-WDT / power-on) so the temp trail in the GPS log can
// be lined up against it on the next boot. Never written after setup().
extern char g_resetReasonStr[32];

// ── Metric → English unit conversions (single source of truth) ───────────────
// The CAN decoders (can_porsche987.h / can_porsche718.h) and the CanData struct
// ALWAYS store metric (kph, °C, bar, kPa). Every OUTPUT path — BLE stream
// (ble_racecapture.cpp), WiFi stream (racecapture.cpp), and the SD CAN log
// (sd_log.cpp) — emits ENGLISH units to match the channel meta sent to
// SoloStorm. Convert at the value-fill site using these helpers so all three
// paths agree exactly and the math lives in ONE place. NaN propagates (a NaN in
// stays NaN out) so "no data" channels still drop out of the sparse stream.
//   kph → mph : ×0.621371      bar → psi : ×14.5038
//   °C  → °F  : ×9/5 + 32      kPa → psi : ×0.145038
static inline float kphToMph(float kph) { return isnan(kph) ? NAN : kph * 0.621371f; }
static inline float cToF(float c)       { return isnan(c)   ? NAN : c * 9.0f / 5.0f + 32.0f; }
static inline float barToPsi(float bar) { return isnan(bar) ? NAN : bar * 14.5038f; }
static inline float kpaToPsi(float kpa) { return isnan(kpa) ? NAN : kpa * 0.145038f; }
