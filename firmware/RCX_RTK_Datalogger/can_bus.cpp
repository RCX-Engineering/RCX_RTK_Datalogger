/*
 * can_bus.cpp — ESP32-S3 TWAI (built-in CAN) driver + decode task
 * ================================================================
 * Hardware: SN65HVD230 transceiver, listen-only.
 *   SN65HVD230 RXD → GPIO8 (CAN_RX_PIN)   green wire
 *   SN65HVD230 TXD → GPIO9 (CAN_TX_PIN)   blue wire (never driven)
 *
 * Reliability design:
 *   - RX queue is large (64) and the task BLOCKS on twai_receive, so frames
 *     are pulled the instant they arrive rather than batched every 50 ms.
 *     This prevents queue overflow / frame loss on a busy 500 kbps bus.
 *   - Decoded values accumulate in a local struct; the global 'can' struct is
 *     updated ONCE per 20 Hz tick under the mutex (not once per frame), which
 *     minimises lock contention with the BLE / display / loop tasks.
 *   - Bus-off and stopped states are detected every cycle and automatically
 *     recovered, so electrical noise can't permanently kill CAN reception.
 *   - 5-second silence timeout resets all channels to NAN.
 *
 * ── Multi-vehicle support (987.2 + 718) ──────────────────────────────────────
 * The two Porsche maps are loaded from can_porsche987.h and can_porsche718.h.
 * Their CAN-ID sets are disjoint EXCEPT for 0x102 (both decode a PDK gear from
 * it), so the active vehicle is identified by PASSIVE ID FINGERPRINTING — we
 * simply watch which platform-exclusive IDs appear on the bus. No frames are
 * ever transmitted (listen-only), and value heuristics are deliberately NOT
 * used (they are fragile before the engine is running). 0x102 is ignored for
 * scoring because it is ambiguous.
 *
 * Selection is cached in NVS (namespace "rcx_veh") so a known car starts in the
 * correct profile INSTANTLY on the next boot with no detection delay. At runtime
 * the task keeps watching: if the OTHER platform's exclusive IDs appear (you
 * physically swapped cars), it switches profile and re-caches. Whichever profile
 * is active, its decoder fills the SAME generic CanData struct, so BLE,
 * RaceCapture, SD logging, and the display are all profile-agnostic and need no
 * changes.
 *
 *   *** PROTECTIVE NOTE (for future AI edits) ***
 *   Do NOT "simplify" detection by keying on 0x102 — it exists on BOTH cars and
 *   will never discriminate them. Do NOT replace ID fingerprinting with value
 *   heuristics (RPM ranges etc.); the bus is alive before the engine is, so a
 *   stopped 718 and a stopped 987 look identical by value. The disjoint-ID
 *   approach is correct and intentional.
 */

#include "can_bus.h"
#include "can_porsche987.h"
#include "can_porsche718.h"
#include "can_porsche718_extra.h"
#include "types.h"
#include "config.h"

#if defined(CAN_ENABLE) && CAN_ENABLE

#include "driver/twai.h"
#include <Preferences.h>
#include "sd_log.h"          // sdlog_push_can_raw() — raw frame logging in sniffer mode

// ── CAN sniffer / diagnostic state ────────────────────────────────────────────
// All of this is inert unless canSniffEnabled is true. The snapshot table holds
// the last payload per standard ID; sniffIndex maps ID→slot for O(1) updates so
// per-frame work stays bounded even on a busy bus.
#define CAN_SNIFF_MAX_IDS 128
static volatile bool     canSniffEnabled = false;
static SemaphoreHandle_t sniffMux        = nullptr;
static CanSniffEntry     sniffTable[CAN_SNIFF_MAX_IDS];
static uint16_t          sniffIndex[2048];          // ID → slot (0xFFFF = unseen)
static int               sniffCount      = 0;
static volatile uint32_t sniffOverflow   = 0;

static void sniffReset() {
    for (int i = 0; i < 2048; i++) sniffIndex[i] = 0xFFFF;
    sniffCount = 0; sniffOverflow = 0;
    memset(sniffTable, 0, sizeof(sniffTable));
}

// O(1) per-frame update. MUST be called with sniffMux held.
static inline void sniffRecord(uint16_t id, uint8_t dlc, const uint8_t* d, uint32_t ms) {
    if (id >= 2048) return;                          // standard 11-bit only
    uint16_t slot = sniffIndex[id];
    if (slot == 0xFFFF) {
        if (sniffCount >= CAN_SNIFF_MAX_IDS) { sniffOverflow++; return; }
        slot = (uint16_t)sniffCount++;
        sniffIndex[id] = slot;
        sniffTable[slot].id = id;
    }
    CanSniffEntry& e = sniffTable[slot];
    if (dlc > 8) dlc = 8;
    e.dlc = dlc; e.lastMs = ms; e.count++;
    for (int i = 0; i < 8; i++) e.data[i] = (i < dlc) ? d[i] : 0;
}

void can_setSniffer(bool on) {
    if (on && !canSniffEnabled && sniffMux &&
        xSemaphoreTake(sniffMux, portMAX_DELAY) == pdTRUE) {
        sniffReset();                                // fresh table each enable
        xSemaphoreGive(sniffMux);
    }
    canSniffEnabled = on;
    Serial.printf("🔎 CAN sniffer %s\n",
                  on ? "ENABLED — capturing ALL IDs" : "disabled");
}
bool can_getSniffer() { return canSniffEnabled; }
uint32_t can_getSniffOverflow() { return sniffOverflow; }

int can_getSniffSnapshot(CanSniffEntry* out, int cap) {
    if (!sniffMux || cap <= 0) return 0;
    int n = 0;
    if (xSemaphoreTake(sniffMux, pdMS_TO_TICKS(50)) == pdTRUE) {
        n = (sniffCount < cap) ? sniffCount : cap;
        memcpy(out, sniffTable, (size_t)n * sizeof(CanSniffEntry));
        xSemaphoreGive(sniffMux);
    }
    // Sort the local copy by ID (small n — insertion sort is plenty).
    for (int i = 1; i < n; i++) {
        CanSniffEntry t = out[i]; int j = i - 1;
        while (j >= 0 && out[j].id > t.id) { out[j + 1] = out[j]; j--; }
        out[j + 1] = t;
    }
    return n;
}

// ── Vehicle profile ───────────────────────────────────────────────────────────
enum VehicleProfile : uint8_t { VEH_UNKNOWN = 0, VEH_987 = 1, VEH_718 = 2 };

static VehicleProfile activeProfile = VEH_UNKNOWN;

// 718-only channels that do not exist in the shared CanData structure. Keeping
// these separate prevents any change to the validated 987.2 mapping. Access is
// mutex-protected through can_getPorsche718Extra().
static Porsche718ExtraData g718Extra;

static inline void clearPorsche718ExtraLocked() {
    g718Extra = Porsche718ExtraData();
}

static const char* vehName(VehicleProfile v) {
    switch (v) { case VEH_987: return "987.2"; case VEH_718: return "718"; default: return "unknown"; }
}

// ── NVS cache of the last detected vehicle ───────────────────────────────────
// One small read at boot, one write per change. Survives power cycles so a known
// car is in the right profile from the very first frame.
static VehicleProfile loadCachedProfile() {
    Preferences p; p.begin("rcx_veh", true);
    VehicleProfile v = (VehicleProfile)p.getUChar("prof", VEH_UNKNOWN);
    p.end();
    if (v != VEH_987 && v != VEH_718) v = VEH_UNKNOWN;
    return v;
}
static void saveCachedProfile(VehicleProfile v) {
    Preferences p; p.begin("rcx_veh", false);
    p.putUChar("prof", (uint8_t)v);
    p.end();
}

// ── Exclusive-ID fingerprinting ───────────────────────────────────────────────
// Each platform-exclusive ID maps to one bit. 0x102 (shared) is deliberately
// absent from both tables. We track which exclusive IDs have been seen as a
// bitmask; popcount(mask) = number of DISTINCT exclusive IDs observed, which is
// a far more robust signal than a raw frame count (one chatty ID can't fake a
// platform on its own).
static const uint16_t kExcl987[] = {
    PORSCHE_CAN_STEERING_ANGLE_ID,        // 0x0C2
    PORSCHE_CAN_VEHICLE_SPEED_ID,         // 0x14A
    PORSCHE_CAN_ENGINE_1_ID,              // 0x242
    PORSCHE_CAN_COOLANT_TEMP_ID,          // 0x245
    PORSCHE_CAN_DME_2_ID,                 // 0x246
    PORSCHE_CAN_WHEEL_SPEEDS_ID,          // 0x24A
    PORSCHE_CAN_PDK_STATUS_CANDIDATE_ID,  // 0x440
    PORSCHE_CAN_OIL_TEMP_PRESS_CAND_ID,   // 0x441
    PORSCHE_CAN_BRAKE_PRESS_CAND_ID,      // 0x44B
};
static const uint16_t kExcl718[] = {
    PORSCHE718_CAN_MANUAL_GEAR_ID,        // 0x081
    PORSCHE718_CAN_STEERING_ID,           // 0x086
    PORSCHE718_CAN_ESP_02_ID,             // 0x101
    PORSCHE718_CAN_WHEEL_SPEEDS_ID,       // 0x103
    PORSCHE718_CAN_TRANSMISSION_ID,       // 0x104
    PORSCHE718_CAN_THROTTLE_ID,           // 0x105
    PORSCHE718_CAN_BRAKES_ID,             // 0x106
    PORSCHE718_CAN_MOTOR_ID,              // 0x107
    PORSCHE718_CAN_DISPLAY_SPEED_ID,      // 0x30B
    PORSCHE718_CAN_TEMPS_ID,              // 0x640
    PORSCHE718_CAN_DIAGNOSE_01_ID,        // 0x6B2
    PORSCHE718_CAN_FUEL_ID,               // 0x6B7
};
static constexpr int kN987 = sizeof(kExcl987) / sizeof(kExcl987[0]);
static constexpr int kN718 = sizeof(kExcl718) / sizeof(kExcl718[0]);

// Number of DISTINCT exclusive IDs needed to commit when starting UNKNOWN, and
// (higher) to override an already-active profile mid-run (a genuine car swap).
#define VEH_DETECT_MIN_DISTINCT   2
#define VEH_SWAP_MIN_DISTINCT     3

static inline void markExclusive(uint16_t id, uint16_t& mask987, uint16_t& mask718) {
    for (int i = 0; i < kN987; i++) if (id == kExcl987[i]) { mask987 |= (uint16_t)(1U << i); return; }
    for (int i = 0; i < kN718; i++) if (id == kExcl718[i]) { mask718 |= (uint16_t)(1U << i); return; }
    // 0x102 and any unknown ID fall through — ignored for scoring.
}
static inline int popcount16(uint16_t m) {
    int c = 0; while (m) { m &= (uint16_t)(m - 1); c++; } return c;
}

// ── can_init ──────────────────────────────────────────────────────────────────
bool can_init() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    g.rx_queue_len = 64;   // deep queue: both Porsche maps broadcast many IDs at 500kbps
    g.tx_queue_len = 0;    // listen-only — no TX
    g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_QUEUE_FULL
                     | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_PASS;

    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();   // both 987.2 and 718 PT/DRIVE CAN are 500k
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("❌ CAN: TWAI driver install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("❌ CAN: TWAI start failed");
        twai_driver_uninstall();
        return false;
    }

    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        status.canBusOk = true;
        xSemaphoreGive(dataMutex);
    }
    sniffMux = xSemaphoreCreateMutex();   // guards the sniffer snapshot table
    sniffReset();
    Serial.println("✅ CAN: TWAI listening at 500kbps (listen-only)");
    return true;
}

// ── Map either profile's decode state into the generic CanData (under mutex) ──
// The generic CanData carries shared stream/log fields. Unmapped channels stay
// NAN so downstream isnan() gating treats them as absent.
static void pushToGlobal987(const Porsche9872CanData& s) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
        can.rpm                = s.engineRpm;
        can.throttleActualPct  = s.throttleActualPercent;
        can.pedalRequestedPct  = s.pedalRequestedPercent;
        can.targetTorquePct    = s.targetTorquePercent;
        can.actualTorquePct    = s.actualTorquePercent;
        can.vehicleSpeedKph    = s.vehicleSpeedKph;
        can.coolantTempC       = s.coolantTempC;
        can.oilTempC           = s.engineOilTempC;
        can.oilPressBar        = s.engineOilPressureBar;
        can.brakePressBar      = s.brakePressureBarCandidate;
        can.brakeSwitch        = NAN;  // 987.2 raw brake-switch bit is still unresolved
        can.brakeSwitch2       = NAN;  // 987.2 raw brake-switch-2 bit is still unresolved
        can.wsFL_kph           = s.wheelSpeedFrontLeftKph;
        can.wsFR_kph           = s.wheelSpeedFrontRightKph;
        can.wsRL_kph           = s.wheelSpeedRearLeftKph;
        can.wsRR_kph           = s.wheelSpeedRearRightKph;
        can.steerAngleDeg      = s.steeringAngleDeg;
        can.steerRateDegPerSec = s.steeringRateDegPerSec;
        can.fuelLevel          = NAN;  // 987.2 raw fuel-level mapping unresolved
        can.kickdown           = s.kickdownValid ? (s.kickdownActive ? 1.0f : 0.0f) : NAN;
        can.atmosphericKpa     = s.atmosphericPressureKpa;
        can.fuelTempC          = NAN;  // 987.2 raw fuel-temperature mapping unresolved
        can.engineTempC        = s.engineCompartmentTempC;
        can.gear               = s.gearCandidate;
        can.pdkGearRaw         = s.pdkStatusCandidateValid ? (float)s.pdkGearRaw : NAN;
        can.pdkSelectorRaw     = s.pdkStatusCandidateValid ? (float)s.pdkSelectorRaw : NAN;
        xSemaphoreGive(dataMutex);
    }
}
static void pushToGlobal718(const Porsche718CanData& s) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
        can.rpm                = s.rpmValid ? s.engineRpm : NAN;

        // 0x105 bits 48..55 are accelerator-pedal raw position, not confirmed
        // throttle-plate angle. Preserve that distinction in the shared data and
        // SD log. The RaceCapture/SoloStorm transports alias PedalPos into TPS
        // for the 718 so SoloStorm receives the expected throttle trace.
        can.throttleActualPct  = NAN;
        can.pedalRequestedPct  = s.acceleratorPedalValid ? s.acceleratorPedalPercent : NAN;
        can.targetTorquePct    = NAN;
        can.actualTorquePct    = NAN;

        // 0x30B is not present on every bus tap. Use it when available; otherwise
        // derive a robust median vehicle speed from the available non-sentinel
        // wheel speeds. This preserves a useful VehicleSpd channel on the chassis bus.
        if (s.displayedSpeedValid && !isnan(s.displayedSpeedKph)) {
            can.vehicleSpeedKph = s.displayedSpeedKph;
        } else {
            float validWheel[4];
            uint8_t count = 0;
            const float wheel[4] = {
                s.wheelSpeedFrontLeftKph, s.wheelSpeedFrontRightKph,
                s.wheelSpeedRearLeftKph, s.wheelSpeedRearRightKph
            };
            for (uint8_t i = 0; i < 4; ++i) {
                if (!isnan(wheel[i])) validWheel[count++] = wheel[i];
            }
            for (uint8_t i = 1; i < count; ++i) {
                const float value = validWheel[i];
                int8_t j = (int8_t)i - 1;
                while (j >= 0 && validWheel[j] > value) {
                    validWheel[j + 1] = validWheel[j];
                    --j;
                }
                validWheel[j + 1] = value;
            }
            if (count == 0U) can.vehicleSpeedKph = NAN;
            else if ((count & 1U) != 0U) can.vehicleSpeedKph = validWheel[count / 2U];
            else can.vehicleSpeedKph = 0.5f * (validWheel[count / 2U - 1U] + validWheel[count / 2U]);
        }

        can.coolantTempC       = s.coolantTempValid ? s.coolantTempC : NAN;
        can.oilTempC           = s.oilTempValid ? s.engineOilTempC : NAN;
        can.oilPressBar        = s.oilPressureValid ? s.engineOilPressureBar : NAN;
        can.brakePressBar      = s.brakePressureValid ? s.brakePressureBar : NAN;
        can.brakeSwitch        = s.brakeLightValid ? (s.brakeLightOn ? 1.0f : 0.0f) : NAN;
        can.brakeSwitch2       = NAN;
        can.wsFL_kph           = s.wheelSpeedsValid ? s.wheelSpeedFrontLeftKph : NAN;
        can.wsFR_kph           = s.wheelSpeedsValid ? s.wheelSpeedFrontRightKph : NAN;
        can.wsRL_kph           = s.wheelSpeedsValid ? s.wheelSpeedRearLeftKph : NAN;
        can.wsRR_kph           = s.wheelSpeedsValid ? s.wheelSpeedRearRightKph : NAN;
        can.steerAngleDeg      = s.steeringAngleValid ? s.steeringAngleDeg : NAN;
        can.steerRateDegPerSec = s.steeringRateValid ? s.steeringRateDegPerSec : NAN;
        can.fuelLevel          = s.fuelLevelValid ? s.fuelLevelLiters : NAN;
        can.kickdown           = NAN;
        can.atmosphericKpa     = NAN;
        can.fuelTempC          = NAN;
        can.engineTempC        = NAN;

        uint8_t selectedGear = 0U;
        bool gearValid = false;
        if (s.pdkGearValid && s.pdkGear >= 1U && s.pdkGear <= 8U) {
            selectedGear = s.pdkGear;
            gearValid = true;
        } else if (s.manualGearValid) {
            if (s.manualGear >= 1U && s.manualGear <= 6U) {
                selectedGear = s.manualGear;
                gearValid = true;
            } else if (s.manualGear == 13U) {
                selectedGear = 8U;  // shared convention: 8 = reverse
                gearValid = true;
            }
        }
        can.gear               = gearValid ? selectedGear : 0U;
        can.pdkGearRaw         = NAN;
        can.pdkSelectorRaw     = NAN;

        // Supplemental 718 channels. All values remain metric here; each output
        // converts to its advertised SoloStorm/RaceCapture unit.
        g718Extra.intakeAirTempC         = s.intakeTempValid ? s.intakeAirTempC : NAN;
        g718Extra.manifoldAbsPressureBar = s.boostPressureValid ? s.boostPressureBar : NAN;
        g718Extra.massAirFlowGps         = NAN;  // passive broadcast mapping not identified
        g718Extra.canLateralAccelG       = s.vehicleDynamicsValid ? s.lateralAccelG : NAN;
        g718Extra.canLongitudinalAccelG  = s.vehicleDynamicsValid
                                            ? (s.longitudinalAccelMps2 / PORSCHE718_STANDARD_GRAVITY_MPS2)
                                            : NAN;
        g718Extra.canYawRateDegPerSec    = s.vehicleDynamicsValid ? s.yawRateDegPerSec : NAN;
        g718Extra.clutchPositionPercent  = s.clutchPositionValid ? s.clutchPositionPercent : NAN;
        g718Extra.outsideTempC           = s.outsideTempValid ? s.outsideTempC : NAN;
        g718Extra.odometerKm             = s.odometerValid ? (float)s.odometerKm : NAN;
        g718Extra.driveMode              = s.driveModeValid ? (float)s.driveMode : NAN;
        g718Extra.psmMode                = s.psmModeValid ? (float)s.psmMode : NAN;
        g718Extra.pdkState               = s.pdkTransmissionStateValid ? (float)s.pdkTransmissionState : NAN;
        g718Extra.pdkNoDriveOrFault      = s.pdkTransmissionStateValid
                                            ? (s.pdkNoDriveOrFault ? 1.0f : 0.0f)
                                            : NAN;
        g718Extra.gearValid              = gearValid ? 1.0f : 0.0f;
        g718Extra.tpmsFrontLeftPsi       = s.tpmsPressureValid ? s.tpmsPressureFrontLeftPsi : NAN;
        g718Extra.tpmsFrontRightPsi      = s.tpmsPressureValid ? s.tpmsPressureFrontRightPsi : NAN;
        g718Extra.tpmsRearLeftPsi        = s.tpmsPressureValid ? s.tpmsPressureRearLeftPsi : NAN;
        g718Extra.tpmsRearRightPsi       = s.tpmsPressureValid ? s.tpmsPressureRearRightPsi : NAN;
        g718Extra.tpmsTempFrontLeftC     = s.tpmsTemperatureValid ? s.tpmsTempFrontLeftC : NAN;
        g718Extra.tpmsTempFrontRightC    = s.tpmsTemperatureValid ? s.tpmsTempFrontRightC : NAN;
        g718Extra.tpmsTempRearLeftC      = s.tpmsTemperatureValid ? s.tpmsTempRearLeftC : NAN;
        g718Extra.tpmsTempRearRightC     = s.tpmsTemperatureValid ? s.tpmsTempRearRightC : NAN;

        xSemaphoreGive(dataMutex);
    }
}

// ── Check/recover bus health. Returns true if bus is healthy. ─────────────────
static bool serviceBusHealth() {
    uint32_t alerts = 0;
    twai_read_alerts(&alerts, 0);   // non-blocking poll of pending alerts

    if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
        Serial.println("⚠️  CAN: RX queue full — frames dropped (bus very busy)");
    if (alerts & TWAI_ALERT_ERR_PASS)
        Serial.println("⚠️  CAN: controller error-passive");

    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) return true;

    // Keep the reported CAN health in sync with the real controller state so the
    // web/TFT "CAN" indicator means "controller is on the bus", not just
    // "driver installed once at boot". (Frame flow is shown separately as Hz.)
    bool running = (st.state == TWAI_STATE_RUNNING);
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
        status.canBusOk = running;
        xSemaphoreGive(dataMutex);
    }

    if (st.state == TWAI_STATE_BUS_OFF) {
        Serial.println("❌ CAN: bus-off — initiating recovery");
        twai_initiate_recovery();
        return false;
    }
    if (st.state == TWAI_STATE_STOPPED) {
        if (twai_start() == ESP_OK)
            Serial.println("🔄 CAN: restarted after recovery");
        return false;
    }
    return running;
}

// ── canBusTask ────────────────────────────────────────────────────────────────
void canBusTask(void*) {
    // Decode state for both profiles. Only the active one is filled, but both
    // exist so a runtime car-swap doesn't carry stale values across the switch.
    Porsche9872CanData local987;
    Porsche718CanData  local718;

    uint32_t frameCount   = 0;
    uint32_t lastFrameMs  = 0;
    uint32_t lastRateCalc = millis();
    uint32_t lastPush     = 0;
    uint32_t lastHealth   = 0;
    bool     dirty        = false;

    // ── Vehicle detection state ──────────────────────────────────────────────
#ifdef VEHICLE_FORCE
    // Optional hard override from config.h: VEH_987 or VEH_718. Skips detection
    // entirely (use only for bench testing on a known bus / replay).
    activeProfile = (VehicleProfile)(VEHICLE_FORCE);
    Serial.printf("🚗 CAN: vehicle FORCED to %s (config.h VEHICLE_FORCE)\n", vehName(activeProfile));
#else
    activeProfile = loadCachedProfile();
    if (activeProfile != VEH_UNKNOWN)
        Serial.printf("🚗 CAN: starting in cached profile %s (will auto-switch if the other car is detected)\n",
                      vehName(activeProfile));
    else
        Serial.println("🚗 CAN: vehicle unknown — fingerprinting bus (need ≥2 distinct platform IDs)…");
#endif

    uint16_t mask987 = 0, mask718 = 0;   // distinct exclusive IDs seen since last reset
    uint32_t lastDetectLog = 0;

    for (;;) {
        twai_message_t msg;
        esp_err_t r = twai_receive(&msg, pdMS_TO_TICKS(10));
        if (r == ESP_OK) {
            // Sniffer hooks (all inert when disabled). We take sniffMux ONCE for
            // the whole drain batch (≤rx_queue_len frames, O(1) each) with a 2 ms
            // cap, so the snapshot update can never stall real-time CAN draining;
            // if the web reader holds the lock we simply skip recording this
            // batch (best-effort) — decoding and raw-logging still happen.
            const bool sniff  = canSniffEnabled;
            const bool rawLog = sniff;   // sniffer ON ⇒ capture raw frames, independent
                                         // of the normal CAN-log toggle and of GPS fix
            bool snifLocked = sniff && sniffMux &&
                              (xSemaphoreTake(sniffMux, pdMS_TO_TICKS(2)) == pdTRUE);
            do {
                if (!msg.rtr && !msg.extd) {
                    const uint16_t id = (uint16_t)msg.identifier;

                    // Fingerprint every standard frame, regardless of profile.
                    markExclusive(id, mask987, mask718);

                    // Decode with the ACTIVE profile's decoder only.
                    bool decoded = false;
                    if (activeProfile == VEH_987)
                        decoded = decodePorsche9872CanFrame(id, msg.data, msg.data_length_code, local987);
                    else if (activeProfile == VEH_718)
                        decoded = decodePorsche718CanFrame(id, msg.data, msg.data_length_code, local718);

                    if (decoded) { dirty = true; }
                    frameCount++;
                    lastFrameMs = millis();

                    // Diagnostic/sniffer capture — ALL IDs, not just decoded ones.
                    if (snifLocked) sniffRecord(id, msg.data_length_code, msg.data, lastFrameMs);
                    if (rawLog)     sdlog_push_can_raw(lastFrameMs, id, msg.data_length_code, msg.data);
                }
            } while (twai_receive(&msg, 0) == ESP_OK);
            if (snifLocked) xSemaphoreGive(sniffMux);
        }

        uint32_t now = millis();

        // ── Detection / car-swap logic ───────────────────────────────────────
        const int n987 = popcount16(mask987);
        const int n718 = popcount16(mask718);

        if (activeProfile == VEH_UNKNOWN) {
            VehicleProfile decided = VEH_UNKNOWN;
            if (n987 >= VEH_DETECT_MIN_DISTINCT && n987 > n718)      decided = VEH_987;
            else if (n718 >= VEH_DETECT_MIN_DISTINCT && n718 > n987) decided = VEH_718;
            if (decided != VEH_UNKNOWN) {
                activeProfile = decided;
                saveCachedProfile(decided);
                local987 = Porsche9872CanData();   // start clean in the chosen profile
                local718 = Porsche718CanData();
                Serial.printf("✅ CAN: detected %s (987 ids=%d, 718 ids=%d) — cached\n",
                              vehName(decided), n987, n718);
            } else if (now - lastDetectLog > 2000) {
                lastDetectLog = now;
                Serial.printf("🚗 CAN: fingerprinting… 987 ids=%d / 718 ids=%d\n", n987, n718);
            }
        } else {
            // Already running. Detect a genuine swap: the OTHER profile shows a
            // strong, distinct fingerprint. Higher threshold than first-detect so
            // a single mis-decoded frame can never flip a live, correct profile.
            VehicleProfile other = (activeProfile == VEH_987) ? VEH_718 : VEH_987;
            int otherCount = (other == VEH_987) ? n987 : n718;
            if (otherCount >= VEH_SWAP_MIN_DISTINCT) {
                Serial.printf("🔁 CAN: %s exclusive IDs appeared — switching profile %s → %s\n",
                              vehName(other), vehName(activeProfile), vehName(other));
                activeProfile = other;
                saveCachedProfile(other);
                local987 = Porsche9872CanData();
                local718 = Porsche718CanData();
                mask987 = mask718 = 0;             // reset so we don't oscillate
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
                    can = CanData();
                    clearPorsche718ExtraLocked();
                    xSemaphoreGive(dataMutex);
                }
            }
        }

        // Push accumulated values to the shared struct at 20 Hz.
        if (dirty && now - lastPush >= 50) {
            lastPush = now;
            if (activeProfile == VEH_987)      pushToGlobal987(local987);
            else if (activeProfile == VEH_718) pushToGlobal718(local718);
            dirty = false;
        }

        // Stale-data timeout — bus silent for 5 s. Channels reset, but the
        // detected profile is RETAINED (engine-off ≠ car swap). A real swap is
        // caught by the other-profile fingerprint above, not by silence.
        if (lastFrameMs != 0 && now - lastFrameMs > 5000) {
            local987 = Porsche9872CanData();
            local718 = Porsche718CanData();
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
                can = CanData();
                clearPorsche718ExtraLocked();
                xSemaphoreGive(dataMutex);
            }
            lastFrameMs = 0;
            // Decay the fingerprint so a swap performed while powered is still
            // re-evaluated from a clean slate next time the bus wakes.
            mask987 = mask718 = 0;
            Serial.println("⚠️  CAN: bus silent 5s — channels reset (profile retained)");
        }

        // Bus health / recovery — every 200 ms.
        if (now - lastHealth >= 200) {
            lastHealth = now;
            serviceBusHealth();
        }

        // Frame-rate measurement + periodic diagnostic — every 2 s.
        if (now - lastRateCalc >= 2000) {
            float hz = frameCount / ((now - lastRateCalc) / 1000.0f);
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
                status.canHz = hz;
                xSemaphoreGive(dataMutex);
            }

            // When NO frames are arriving, dump the TWAI controller state + error
            // counters every cycle so the cause is visible at a glance:
            //   • bus_error / rx_error climbing → wrong bit rate, or wiring/noise
            //   • state BUS_OFF / RECOVERING    → severe errors (baud/termination)
            //   • RUNNING, counters flat, 0 fr  → no traffic on the pair you tapped:
            //       wrong bus, CANH/CANL swapped, no 120Ω termination, or
            //       listen-only against a lone sender that bus-offed for lack of ACK
            if (hz == 0.0f) {
#if 0  // CAN heartbeat (no-frames bus diagnostic) — re-enable to troubleshoot CAN
                twai_status_info_t st;
                if (twai_get_status_info(&st) == ESP_OK) {
                    const char* sname = st.state == TWAI_STATE_RUNNING    ? "RUNNING"
                                      : st.state == TWAI_STATE_BUS_OFF     ? "BUS_OFF"
                                      : st.state == TWAI_STATE_RECOVERING  ? "RECOVERING"
                                                                           : "STOPPED";
                    Serial.printf("🔌 CAN bus: 0 fr/s | state=%s rxErr=%lu txErr=%lu busErr=%lu "
                                  "rxMissed=%lu rxOverrun=%lu rxQ=%lu\n",
                                  sname,
                                  (unsigned long)st.rx_error_counter, (unsigned long)st.tx_error_counter,
                                  (unsigned long)st.bus_error_count,   (unsigned long)st.rx_missed_count,
                                  (unsigned long)st.rx_overrun_count,  (unsigned long)st.msgs_to_rx);
                }
#endif
            }
            static uint32_t lastDiag = 0;
            if (now - lastDiag > 10000) {
                lastDiag = now;
#if 0  // CAN telemetry heartbeat (every 10s) — re-enable to troubleshoot CAN
                float rpm = NAN, tps = NAN, pedal = NAN, oil = NAN, ect = NAN; uint8_t gear = 0;
                if (activeProfile == VEH_987) {
                    rpm = local987.engineRpm; tps = local987.throttleActualPercent; pedal = local987.pedalRequestedPercent;
                    oil = local987.engineOilTempC; ect = local987.coolantTempC; gear = local987.gearCandidate;
                } else if (activeProfile == VEH_718) {
                    rpm = local718.engineRpm; tps = NAN; pedal = local718.acceleratorPedalPercent;
                    oil = local718.engineOilTempC; ect = local718.coolantTempC;
                    gear = local718.pdkGearValid ? local718.pdkGear : local718.manualGear;
                }
                Serial.printf("🚗 CAN[%s]: %.0f fr/s | RPM=%.0f TPS=%.1f%% Pedal=%.1f%% ECT=%.0f°C Oil=%.0f°C Gear=%d\n",
                              vehName(activeProfile), hz,
                              isnan(rpm) ? 0.0f : rpm,
                              isnan(tps) ? 0.0f : tps,
                              isnan(pedal) ? 0.0f : pedal,
                              isnan(ect) ? 0.0f : ect,
                              isnan(oil) ? 0.0f : oil, gear);
#endif
            }
            frameCount   = 0;
            lastRateCalc = now;
        }
    }
}

// ── Diagnostic accessor (optional; safe to call from any task) ───────────────
const char* can_getVehicleName() { return vehName(activeProfile); }

bool can_getPorsche718Extra(Porsche718ExtraData& out) {
    out = Porsche718ExtraData();
    if (activeProfile != VEH_718 || !dataMutex) return false;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
    out = g718Extra;
    xSemaphoreGive(dataMutex);
    return true;
}

#else  // CAN disabled — stubs so the project links without hardware

bool can_init() { Serial.println("ℹ️  CAN: disabled in config.h"); return false; }
void canBusTask(void*) {}
const char* can_getVehicleName() { return "disabled"; }
void can_setSniffer(bool) {}
bool can_getSniffer() { return false; }
int  can_getSniffSnapshot(CanSniffEntry*, int) { return 0; }
uint32_t can_getSniffOverflow() { return 0; }
bool can_getPorsche718Extra(Porsche718ExtraData& out) {
    out = Porsche718ExtraData();
    return false;
}

#endif  // CAN_ENABLE
