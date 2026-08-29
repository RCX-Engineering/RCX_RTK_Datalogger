/*
 * racecapture.cpp — RaceCapture/Pro JSON WiFi TCP emulator
 * ==========================================================
 * Reference implementation cross-checked against:
 *   - RacePi by Donour Sizemore (github.com/donour/racepi)
 *   - autosportlabs/RaceCapture-Pro_firmware JSON API source
 *
 * Channel ordering MUST match the array in buildChannelList() exactly,
 * or the app will decode values into the wrong gauge channels.
 *
 * Architecture:
 *   - AsyncTCP is NOT used here; instead we use a raw WiFiServer
 *     polled from wifiNtripTask (Core 0).  This avoids an extra RTOS
 *     task and fits neatly alongside the existing NTRIP loop.
 *   - JSON strings are built with snprintf into a fixed 2KB buffer.
 *     No dynamic allocation, no ArduinoJson dependency.
 *   - The 20 Hz data tick uses the same vTaskDelayUntil-free approach
 *     as the rest of the project — call racecapture_loop() at least
 *     every 50 ms from the WiFi task and it self-throttles.
 */

#include "racecapture.h"
#include "config.h"
#include "types.h"
#include "can_porsche718_extra.h"
#include <WiFi.h>

#if defined(RACECAPTURE_ENABLE) && RACECAPTURE_ENABLE

// ── Channel table ─────────────────────────────────────────────────────────────
// Each entry defines one channel.  The d[] array in the stream packet
// must be populated in the same order as this table.
// Fields: name, unit, precision, min, max, sample_rate_Hz
struct RcChannel {
    const char* name;
    const char* unit;
    uint8_t     prec;
    float       minVal;
    float       maxVal;
    uint8_t     sampleRate;  // Hz
};

// ── Channel index constants ────────────────────────────────────────────────────
// Keep in sync with the array in buildChannelList() and fillSample()
enum RcIdx {
    RC_Interval = 0,
    RC_Utc,
    RC_LapCount,
    RC_LapTime,
    RC_Distance,
    RC_ElapsedTime,
    RC_CurrentLap,
    RC_GPSLatitude,
    RC_GPSLongitude,
    RC_GPSSpeed,
    RC_GPSHeading,
    RC_GPSAltitude,
    RC_GPSSats,
    RC_GPSDOP,
    RC_GPSQual,
    RC_AccelX,
    RC_AccelY,
    RC_AccelZ,
    RC_Yaw,
    // ── CAN channels ────────────────────────────────────────────────────────
    RC_RPM,
    RC_ThrottlePos,
    RC_PedalRequested,
    RC_TgtTorque,
    RC_ActTorque,
    RC_VehicleSpd,
    RC_CoolantTmp,
    RC_OilTemp,
    RC_OilPressure,
    RC_Brake,
    RC_BrakeSw,
    RC_BrakeSw2,
    RC_WheelSpdLF,
    RC_WheelSpdRF,
    RC_WheelSpdLR,
    RC_WheelSpdRR,
    RC_Steering,
    RC_SteerRate,
    RC_FuelLevel,
    RC_Kickdown,
    RC_BaroPress,
    RC_FuelTemp,
    RC_EngineTemp,
    RC_Gear,
    RC_PdkGearRaw,
    RC_PdkSelRaw,
    // ── 718 supplemental channels (append-only: preserve existing indices) ──
    RC_IAT,
    RC_MAP,
    RC_MAF,
    RC_CANLatG,
    RC_CANLongG,
    RC_CANYaw,
    RC_ClutchPos,
    RC_OutsideTmp,
    RC_Odometer,
    RC_DriveMode,
    RC_PSMMode,
    RC_PDKState,
    RC_PDKFault,
    RC_TPMSLF,
    RC_TPMSRF,
    RC_TPMSLR,
    RC_TPMSRR,
    RC_TPMSTmpLF,
    RC_TPMSTmpRF,
    RC_TPMSTmpLR,
    RC_TPMSTmpRR,
    RC_CHANNEL_COUNT
};

static const RcChannel kChannels[RC_CHANNEL_COUNT] = {
    // name               unit   prec    min       max     Hz
    // ── System / timing (RaceCapture standard names) ─────────────────────────
    {"Interval",          "ms",    0,      0,    99999,    1},
    {"Utc",               "ms",    0,      0, 99999999.0f,  1},
    {"LapCount",          "",      0,      0,      999,    1},
    {"LapTime",           "Min",   4,      0,   9999.9,    5},
    {"Distance",          "mi",    3,      0,    999.9,    5},
    {"ElapsedTime",       "Min",   4,      0,  99999.9,    1},
    {"CurrentLap",        "",      0,      0,      999,    1},
    // ── GPS (RaceCapture standard names — GPSSats/GPSDOP/GPSQual, NOT
    //    Satellites/DOP/GPSQuality — and ENGLISH units: mph, ft) ──────────────
    {"Latitude",          "Degrees", 9,    -90,       90,   20},
    {"Longitude",         "Degrees", 9,   -180,      180,   20},
    {"Speed",             "mph",   2,      0,      200,   20},
    {"Heading",           "Deg",   1,      0,      360,   20},
    {"Altitude",          "ft",    1,      0,    13000,   20},   // was "m" — now ft
    {"GPSSats",           "",      0,      0,       20,    1},   // was "Satellites"
    {"GPSDOP",            "",      1,      0,       20,    1},   // was "DOP"
    {"GPSQual",           "",      0,      0,        5,    1},   // was "GPSQuality"
    // ── IMU (RaceCapture standard names) ─────────────────────────────────────
    {"AccelX",            "G",     2,     -3,        3,   20},
    {"AccelY",            "G",     2,     -3,        3,   20},
    {"AccelZ",            "G",     2,     -3,        3,   20},
    {"Yaw",               "Deg/Sec", 0, -120,      120,   20},
    // ── CAN — ENGLISH units, names ≤11 chars, IDENTICAL to the BLE table in
    //    ble_racecapture.cpp and the SD CAN header in sd_log.cpp. Values are
    //    converted metric→English in fillSample() via the helpers in types.h.
    //    Renames to fit 11 chars: TargetTorque→TgtTorque, ActualTorque→
    //    ActTorque, VehicleSpeed→VehicleSpd, BrakeSwitch→BrakeSw, BrakeSwitch2→
    //    BrakeSw2, SteeringAngle→Steering, SteeringRate→SteerRate,
    //    PdkSelectorRaw→PdkSelRaw. Wheel speeds use L/R-then-F/R order.
    {"RPM",               "RPM",   0,      0,    10000,   20},
    {"TPS",               "%",     1,      0,      100,   20},  // 718: pedal-position alias for SoloStorm
    {"PedalPos",          "%",     1,      0,      100,   20},
    {"TgtTorque",         "%",     1,      0,      100,   20},
    {"ActTorque",         "%",     1,      0,      100,   20},
    {"VehicleSpd",        "mph",   1,      0,      200,   20},
    {"CoolantTmp",        "F",     1,    -40,      300,    1},
    {"OilTemp",           "F",     1,    -40,      300,    1},
    {"OilPress",          "psi",   2,      0,      145,    5},
    {"Brake",             "psi",   1,      0,     2900,   20},
    {"BrakeSw",           "",      0,      0,        1,   20},
    {"BrakeSw2",          "",      0,      0,        1,   20},
    {"WheelSpdLF",        "mph",   1,      0,      200,   20},
    {"WheelSpdRF",        "mph",   1,      0,      200,   20},
    {"WheelSpdLR",        "mph",   1,      0,      200,   20},
    {"WheelSpdRR",        "mph",   1,      0,      200,   20},
    {"Steering",          "Deg",   1,   -600,      600,   20},
    {"SteerRate",         "Deg/Sec", 1, -2000,     2000,   20},
    {"FuelLevel",         "L",     0,      0,      127,    1},  // 718 LITERS (0-127); 987 unresolved
    {"Kickdown",          "",      0,      0,        1,   20},
    {"BaroPress",         "psi",   2,      0,       18,    1},
    {"FuelTemp",          "F",     1,    -40,      300,    1},
    {"EngineTemp",        "F",     1,    -40,      300,    1},
    {"Gear",              "",      0,      0,        8,   20},
    {"PdkGearRaw",        "",      0,      0,      255,   20},
    {"PdkSelRaw",         "",      0,      0,      255,   20},
    // 718 supplemental channels. Names are short enough for SoloStorm and are
    // appended so every pre-existing channel keeps its original index.
    {"IAT",               "F",     1,    -40,      350,    1},
    {"MAP",               "psi",   2,      0,       75,   20},  // absolute manifold pressure
    {"MAF",               "g/s",   1,      0,     1000,   20},  // metadata only until passive ID is identified
    {"CANLatG",           "G",     3,     -3,        3,   20},
    {"CANLongG",          "G",     3,     -3,        3,   20},
    {"CANYaw",            "Deg/Sec", 1, -250,      250,   20},
    {"ClutchPos",         "%",     1,      0,      100,   20},
    {"OutsideTmp",        "F",     1,    -60,      160,    1},
    {"Odometer",          "mi",    1,      0,   700000,    1},
    {"DriveMode",         "",      0,      0,       15,    5},
    {"PSMMode",           "",      0,      0,       15,    5},
    {"PDKState",          "",      0,      0,        3,   20},
    {"PDKFault",          "",      0,      0,        1,   20},
    {"TPMSLF",            "psi",   1,      0,      100,    1},
    {"TPMSRF",            "psi",   1,      0,      100,    1},
    {"TPMSLR",            "psi",   1,      0,      100,    1},
    {"TPMSRR",            "psi",   1,      0,      100,    1},
    {"TPMSTmpLF",         "F",     1,    -40,      300,    1},
    {"TPMSTmpRF",         "F",     1,    -40,      300,    1},
    {"TPMSTmpLR",         "F",     1,    -40,      300,    1},
    {"TPMSTmpRR",         "F",     1,    -40,      300,    1},
};

// ── Server state ──────────────────────────────────────────────────────────────
static WiFiServer rcServer(RACECAPTURE_PORT);
static WiFiClient rcClient;
static bool       serverBegun  = false;
static bool       streaming    = true;   // stream by default after connect

// JSON RX buffer — command from app can arrive in fragments
static char rxBuf[256];
static int  rxPos = 0;

// ── JSON helpers ──────────────────────────────────────────────────────────────

// Append a float or "null" if NaN.
// Takes DOUBLE (not float): casting lat/lon to float32 quantises longitude to
// ~2.4 m, which would throw away the RTK precision this whole project exists to
// capture. Also guards pos against running past the buffer — snprintf returns the
// length it WOULD have written, so blindly doing pos += snprintf can push pos beyond
// size; then size-pos goes negative, converts to a huge size_t, and the next write
// smashes the stack. We clamp pos to stay <= size on every call.
static int appendVal(char* buf, int pos, int size, double v, int prec) {
    if (pos >= size - 24) return pos;                 // no room — stop cleanly
    if (isnan(v)) {
        int n = snprintf(buf + pos, size - pos, "null");
        return (n > 0 && pos + n < size) ? pos + n : pos;
    }
    if (!isfinite(v))  v = 0.0;                        // neutralise inf
    else if (v >  1e9) v =  1e9;                        // clamp garbage (dtostrf-style safety)
    else if (v < -1e9) v = -1e9;
    char fmt[12]; snprintf(fmt, sizeof(fmt), "%%.%df", prec);
    int n = snprintf(buf + pos, size - pos, fmt, v);
    return (n > 0 && pos + n < size) ? pos + n : pos;  // never let pos exceed size
}

// Emit the full getCapabilities response.
//
// ⚠️  WIRE IDENTITY — these two strings are what a connected app records as this
//     device. The protocol shape around them is RaceCapture's and must not
//     change; only the display name does. An app that has already paired may
//     have cached the previous identity, so if it behaves oddly after this
//     rename, remove the saved device in the app and re-pair. The version
//     triplet is RaceCapture's protocol version and is NOT this project's.
static void sendCapabilities() {
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"getCapabilities\":{\"major\":2,\"minor\":14,\"bugfix\":0,"
        "\"serial\":\"RCX_Datalogger_01\","
        "\"friendlyName\":\"RCX Datalogger\"}}\r\n");
    rcClient.print(buf);
}

// Emit the full getChannels response — one JSON object per channel
static void sendChannelList() {
    // Send header
    rcClient.print("{\"getChannels\":{\"success\":1,\"channels\":[");
    for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        const RcChannel& ch = kChannels[i];
        char entry[180];
        snprintf(entry, sizeof(entry),
            "%s{\"nm\":\"%s\",\"ut\":\"%s\",\"sr\":%d,\"prec\":%d,"
            "\"min\":%.0f,\"max\":%.0f}",
            (i == 0) ? "" : ",",
            ch.name, ch.unit, ch.sampleRate, ch.prec, ch.minVal, ch.maxVal);
        rcClient.print(entry);
    }
    rcClient.print("]}}\r\n");
}

// Build and send one data sample: {"s":{"t":ms,"d":[v0,v1,...]}}
static void sendSample(uint32_t ms, const GnssData& g, const ImuData& m, const CanData& c) {
    // Extra 718 channels increase the full-array JSON size.
    static char buf[3072];
    int pos = 0;
    int sz  = (int)sizeof(buf);

    pos += snprintf(buf + pos, sz - pos, "{\"s\":{\"t\":%lu,\"d\":[", (unsigned long)ms);

    // ── Burst-encode all channels in index order ──────────────────────────
    auto ap = [&](double v, int prec) { pos = appendVal(buf, pos, sz, v, prec); };
    auto comma = [&]() { if (pos < sz-1) buf[pos++] = ','; };

    // RC_Interval
    ap((double)(ms % 1000), 0); comma();
    // RC_Utc (ms within day)
    uint32_t utcMs = (uint32_t)g.hour*3600000UL + g.minute*60000UL + g.second*1000UL + g.millisecond;
    ap(g.valid ? (double)utcMs : NAN, 0); comma();
    // Lap / timing fields — not implemented; emit null
    for (int i = RC_LapCount; i <= RC_CurrentLap; i++) { ap(NAN, 0); comma(); }
    // GPS — lat/lon kept as double so RTK-grade precision survives to the app
    ap(g.valid ? g.latitude  : NAN, 9); comma();
    ap(g.valid ? g.longitude : NAN, 9); comma();
    ap(g.valid ? (g.speedKnots * 1.15078) : NAN, 2); comma();  // knots→mph
    ap(g.valid ? g.headingDeg : NAN, 1); comma();
    ap(g.valid ? (g.altMSL * 3.28084) : NAN, 1); comma();  // m→ft (meta unit "ft")
    ap(g.valid ? (double)g.numSV : NAN, 0); comma();
    ap(g.valid ? g.hAccM      : NAN, 1); comma();  // GPSDOP proxy (hAcc metres)
    ap(g.valid ? (double)g.rtkType : NAN, 0); comma();  // GPSQual
    // IMU
    ap(m.ax, 2); comma();
    ap(m.ay, 2); comma();
    ap(m.az, 2); comma();
    ap(m.gz, 0);  // Yaw rate (prec 0 to match meta)
    // ── CAN — converted METRIC → ENGLISH to match the meta (helpers in types.h).
    //    CanData stays metric; convert here so BLE/WiFi/SD all agree.
    //    The 718 broadcast field is accelerator-pedal position, not confirmed
    //    throttle-plate angle. SoloStorm uses the standard TPS channel for its
    //    throttle trace, so alias the 718 pedal value into TPS at transport only.
    //    Keep PedalPos populated too, and leave the internal/SD semantics honest.
    Porsche718ExtraData x718;
    const bool is718 = can_getPorsche718Extra(x718);
    const float soloThrottlePct = is718 ? c.pedalRequestedPct : c.throttleActualPct;

    comma(); ap(c.rpm,                       0); comma();
    ap(soloThrottlePct,              1); comma();
    ap(c.pedalRequestedPct,          1); comma();
    ap(c.targetTorquePct,            1); comma();
    ap(c.actualTorquePct,            1); comma();
    ap(kphToMph(c.vehicleSpeedKph),  1); comma();  // kph→mph
    ap(cToF(c.coolantTempC),         1); comma();  // °C→°F
    ap(cToF(c.oilTempC),             1); comma();  // °C→°F
    ap(barToPsi(c.oilPressBar),      2); comma();  // bar→psi
    ap(barToPsi(c.brakePressBar),    1); comma();  // bar→psi
    ap(c.brakeSwitch,                0); comma();
    ap(c.brakeSwitch2,               0); comma();
    ap(kphToMph(c.wsFL_kph),         1); comma();  // kph→mph
    ap(kphToMph(c.wsFR_kph),         1); comma();  // kph→mph
    ap(kphToMph(c.wsRL_kph),         1); comma();  // kph→mph
    ap(kphToMph(c.wsRR_kph),         1); comma();  // kph→mph
    ap(c.steerAngleDeg,              1); comma();
    ap(c.steerRateDegPerSec,         1); comma();
    ap(c.fuelLevel,                  0); comma();  // 718 liters (integer)
    ap(c.kickdown,                   0); comma();
    ap(kpaToPsi(c.atmosphericKpa),   2); comma();  // kPa→psi
    ap(cToF(c.fuelTempC),            1); comma();  // °C→°F
    ap(cToF(c.engineTempC),          1); comma();  // °C→°F
    ap((float)c.gear,                0); comma();
    ap(c.pdkGearRaw,                 0); comma();
    ap(c.pdkSelectorRaw,             0);

    comma(); ap(is718 ? cToF(x718.intakeAirTempC) : NAN,              1);
    comma(); ap(is718 ? barToPsi(x718.manifoldAbsPressureBar) : NAN,  2);
    comma(); ap(is718 ? x718.massAirFlowGps : NAN,                    1);
    comma(); ap(is718 ? x718.canLateralAccelG : NAN,                  3);
    comma(); ap(is718 ? x718.canLongitudinalAccelG : NAN,             3);
    comma(); ap(is718 ? x718.canYawRateDegPerSec : NAN,               1);
    comma(); ap(is718 ? x718.clutchPositionPercent : NAN,             1);
    comma(); ap(is718 ? cToF(x718.outsideTempC) : NAN,                 1);
    comma(); ap(is718 ? x718.odometerKm * 0.621371f : NAN,             1);
    comma(); ap(is718 ? x718.driveMode : NAN,                          0);
    comma(); ap(is718 ? x718.psmMode : NAN,                            0);
    comma(); ap(is718 ? x718.pdkState : NAN,                           0);
    comma(); ap(is718 ? x718.pdkNoDriveOrFault : NAN,                  0);
    comma(); ap(is718 ? x718.tpmsFrontLeftPsi : NAN,                   1);
    comma(); ap(is718 ? x718.tpmsFrontRightPsi : NAN,                  1);
    comma(); ap(is718 ? x718.tpmsRearLeftPsi : NAN,                    1);
    comma(); ap(is718 ? x718.tpmsRearRightPsi : NAN,                   1);
    comma(); ap(is718 ? cToF(x718.tpmsTempFrontLeftC) : NAN,           1);
    comma(); ap(is718 ? cToF(x718.tpmsTempFrontRightC) : NAN,          1);
    comma(); ap(is718 ? cToF(x718.tpmsTempRearLeftC) : NAN,            1);
    comma(); ap(is718 ? cToF(x718.tpmsTempRearRightC) : NAN,           1);

    // Close
    if (pos < sz - 5)
        pos += snprintf(buf + pos, sz - pos, "]}}\r\n");

    rcClient.write((const uint8_t*)buf, pos);
}

// ── Handle one line of JSON from the app ──────────────────────────────────────
static void handleCommand(const char* json) {
    // Simple keyword dispatch — no JSON parser needed for these short commands
    if (strstr(json, "getCapabilities")) { sendCapabilities(); return; }
    if (strstr(json, "getChannels"))     { sendChannelList(); return; }
    if (strstr(json, "startStreaming"))  { streaming = true;  return; }
    if (strstr(json, "stopStreaming"))   { streaming = false; return; }
    // Unknown command — silently ignore
}

// ── Public API ────────────────────────────────────────────────────────────────
void racecapture_beginServer() {
    if (serverBegun) { rcServer.stop(); rcClient.stop(); }
    rcServer.begin();
    serverBegun = true;
    Serial.printf("📡 RaceCapture server listening on port %d\n", RACECAPTURE_PORT);
}

bool racecapture_clientConnected() {
    return rcClient.connected();
}

void racecapture_loop(const GnssData& g, const ImuData& m, const CanData& c) {
    if (!serverBegun) return;

    // Accept new client (only one at a time)
    if (!rcClient || !rcClient.connected()) {
        if (rcClient) {
            rcClient.stop();
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
                status.rcConnected = false;
                xSemaphoreGive(dataMutex);
            }
            Serial.println("📡 RaceCapture client disconnected");
        }
        rcClient = rcServer.available();
        if (rcClient) {
            streaming = true;
            rxPos = 0;
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
                status.rcConnected = true;
                xSemaphoreGive(dataMutex);
            }
            Serial.printf("📡 RaceCapture app connected from %s\n",
                          rcClient.remoteIP().toString().c_str());
        }
    }

    if (!rcClient || !rcClient.connected()) return;

    // ── Drain inbound commands ────────────────────────────────────────────
    while (rcClient.available()) {
        char ch = (char)rcClient.read();
        if (ch == '\r') continue;
        if (ch == '\n') {
            if (rxPos > 0) {
                rxBuf[rxPos] = '\0';
                handleCommand(rxBuf);
                rxPos = 0;
            }
        } else if (rxPos < (int)sizeof(rxBuf) - 1) {
            rxBuf[rxPos++] = ch;
        }
    }

    // ── Stream sample at 20 Hz ────────────────────────────────────────────
    if (!streaming) return;
    static uint32_t lastSend = 0;
    uint32_t now = millis();
    if (now - lastSend < (1000u / 20u)) return;   // 20 Hz
    lastSend = now;

    sendSample(now, g, m, c);
}

#else  // RACECAPTURE_ENABLE not set

void racecapture_beginServer()                                              {}
void racecapture_loop(const GnssData&, const ImuData&, const CanData&)     {}
bool racecapture_clientConnected()                                          { return false; }

#endif // RACECAPTURE_ENABLE
