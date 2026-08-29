/*
    Porsche Boxster/Cayman 987.2 S CAN Decode Specification
    Target vehicle:
      - Porsche Boxster S / Cayman S / Cayman R 987.2, model years 2009-2012
      - Porsche 997.2 Carrera/Turbo Siemens EMS SDI 3.1 cars where the same
        direct engine-CAN protocol is present
      - Direct CAN broadcast frames, not generic SAE OBD-II PID requests

    CAN bus:
      - Standard 11-bit CAN identifiers
      - Expected bus speed: 500000 bps
      - Byte numbering: data[0] through data[7]
      - CAN IDs below are written in hexadecimal, with decimal source IDs noted.

    Signal confidence:
      - Signals marked "candidate" have mostly been confirmed, but not changed yet.
      - TPS and PPS are intentionally separate: TPS is actual throttle plate / ECU
        torque-managed throttle, while PPS/PedalPos is the driver's accelerator
        pedal request.

    Identified, but not yet mapped/logged:
    Sport           0x308 byte0 bit5        Button-press
    Sport Plus      0x308 byte1 bit7        Button-press
    Sport Exhaust   0x502 possibly byte3, but noisy with a lot of stuff going on. Need to do more.
    Spoiler Up      0x600 byte 6            Button-press
    Spoiler Down    0x600 byte 6            Button-hold
    PSM on/off      0x440 byte4 bit0 low pulses  Still trying to puzzle this one one
*/


#pragma once
#include <Arduino.h>
#include <math.h>

static constexpr uint16_t PORSCHE_CAN_STEERING_ANGLE_ID        = 0x0C2;  // decimal 194
static constexpr uint16_t PORSCHE_CAN_VEHICLE_SPEED_ID         = 0x14A;  // decimal 330
static constexpr uint16_t PORSCHE_CAN_ENGINE_1_ID              = 0x242;  // decimal 578
static constexpr uint16_t PORSCHE_CAN_COOLANT_TEMP_ID          = 0x245;  // decimal 581
static constexpr uint16_t PORSCHE_CAN_DME_2_ID                 = 0x246;  // decimal 582
static constexpr uint16_t PORSCHE_CAN_PEDAL_GEAR_CANDIDATE_ID  = PORSCHE_CAN_DME_2_ID; // legacy name
static constexpr uint16_t PORSCHE_CAN_PDK_GEAR_CANDIDATE_ID    = 0x102;  // candidate fallback
static constexpr uint16_t PORSCHE_CAN_PDK_STATUS_CANDIDATE_ID  = 0x440;  // decimal 1088, candidate
static constexpr uint16_t PORSCHE_CAN_WHEEL_SPEEDS_ID          = 0x24A;  // decimal 586
static constexpr uint16_t PORSCHE_CAN_OIL_TEMP_PRESS_CAND_ID   = 0x441;  // decimal 1089
static constexpr uint16_t PORSCHE_CAN_BRAKE_PRESS_CAND_ID      = 0x44B;  // decimal 1099, candidate

struct Porsche9872CanData {
    bool rpmValid                    = false;
    bool throttleActualValid         = false;
    bool pedalRequestedValid         = false;
    bool targetTorqueValid           = false;
    bool actualTorqueValid           = false;
    bool vehicleSpeedValid           = false;
    bool coolantTempValid            = false;
    bool steeringAngleValid          = false;
    bool steeringRateValid           = false;
    bool wheelSpeedsValid            = false;
    bool oilTempValid                = false;
    bool oilPressureValid            = false;
    bool brakePressureCandidateValid = false;
    bool gearCandidateValid          = false;
    bool pdkStatusCandidateValid     = false;
    bool kickdownValid               = false;
    bool atmosphericPressureValid    = false;
    bool engineCompartmentTempValid  = false;
    bool radiatorFanRequestValid     = false;

    float   engineRpm                     = NAN;
    float   throttleActualPercent         = NAN;  // TPS actual, ECU/torque-managed throttle
    float   pedalRequestedPercent         = NAN;  // PPS/PedalPos, driver accelerator request
    float   targetTorquePercent           = NAN;  // candidate from DME_2
    float   actualTorquePercent           = NAN;  // candidate from DME_2
    float   vehicleSpeedKph               = NAN;
    float   coolantTempC                  = NAN;
    float   steeringAngleDeg              = NAN;
    float   steeringRateDegPerSec         = NAN;
    float   wheelSpeedFrontLeftKph        = NAN;
    float   wheelSpeedFrontRightKph       = NAN;
    float   wheelSpeedRearLeftKph         = NAN;
    float   wheelSpeedRearRightKph        = NAN;
    float   engineOilTempC                = NAN;
    float   engineOilPressureBar          = NAN;
    float   brakePressureBarCandidate     = NAN;
    float   atmosphericPressureKpa        = NAN;
    float   engineCompartmentTempC        = NAN;  // candidate, not ECT
    float   radiatorFanRequestPercent     = NAN;  // useful DME status, not requested for RaceCapture by default
    uint8_t gearCandidate                 = 0;  // 1-7 forward, 8=reverse, 0=unknown/transition
    uint8_t pdkGearRaw                    = 0;  // raw 0x440 data[1], logged to identify 7th
    uint8_t pdkSelectorRaw                = 0;  // raw 0x440 data[2]
    bool    kickdownActive                = false;
};

// ── Byte helpers ──────────────────────────────────────────────────────────────
static inline uint16_t u16le(const uint8_t lo, const uint8_t hi) {
    return ((uint16_t)hi << 8) | lo;
}
static inline int16_t s16le(const uint8_t lo, const uint8_t hi) {
    return (int16_t)(((uint16_t)hi << 8) | lo);
}
static inline float clampPercent(const float value) {
    if (!isfinite(value)) return NAN;
    if (value < 0.0f) return 0.0f;
    if (value > 100.0f) return 100.0f;
    return value;
}
static inline float decodePorscheWheelSpeedKph_15bit(const uint8_t lo, const uint8_t hi) {
    return (float)(u16le(lo, hi) >> 1) * 0.02f;
}

// ── Per-ID decoders ───────────────────────────────────────────────────────────
// Sign-MAGNITUDE 16-bit decode: bit 15 is a DIRECTION flag, the low 15 bits are
// the unsigned magnitude. This is NOT two's-complement. 0.045 deg/bit.
static inline float sm16le_0045(const uint8_t lo, const uint8_t hi) {
    uint16_t u = u16le(lo, hi);
    float mag = (float)(u & 0x7FFFU) * 0.045f;
    return (u & 0x8000U) ? -mag : mag;
}
static inline void decodePorsche9872_0x0C2_Steering(const uint8_t data[8], Porsche9872CanData& out) {
    // FIELD-CORRECTED 2026-06-29 from can_20260629_140311.csv (987.2 S).
    // The previous decode used two's-complement s16le * 0.045, which read the
    // centered (parked) wheel as ~-1470 deg — physically impossible (4+ turns)
    // and "all wonky" in SoloStorm. In the raw log the at-rest samples sit at
    // 0x80xx (bit 15 SET, tiny low bits), i.e. the encoding is SIGN-MAGNITUDE,
    // not two's-complement: bit 15 = steer direction, low 15 bits = magnitude.
    // Re-decoded that way the wheel centers at ~-1..-15 deg (sensor zero point),
    // turning sweeps smoothly to +/-400 deg, and steer-rate centers at 0 deg/s.
    // (This is effectively the "unsigned + offset" family the old comment warned
    //  against — but the field data is unambiguous, so the warning was wrong for
    //  THIS bus. Do not revert to plain s16le without re-checking a fresh log.)
    out.steeringAngleDeg      = sm16le_0045(data[0], data[1]);
    out.steeringRateDegPerSec = sm16le_0045(data[2], data[3]);
    out.steeringAngleValid    = true;
    out.steeringRateValid     = true;
}

static inline void decodePorsche9872_0x14A_VehicleSpeed(const uint8_t data[8], Porsche9872CanData& out) {
    // Validated against canraw_b28545.csv: data[2..3] / 100 matches the average
    // of the four 0x24A wheel speeds and GPS-speed scale. The older 0.02 scale is
    // exactly 2x high on this 987.2S log.
    out.vehicleSpeedKph     = (float)u16le(data[2], data[3]) * 0.01f;
    out.vehicleSpeedValid   = true;
}

static inline void decodePorsche9872_0x242_Engine1(const uint8_t data[8], Porsche9872CanData& out) {
    out.engineRpm             = (float)u16le(data[2], data[3]) * 0.25f;
    out.throttleActualPercent = clampPercent((float)data[5] * (100.0f / 255.0f));
    out.rpmValid              = true;
    out.throttleActualValid   = true;
}

static inline void decodePorsche9872_0x245_Coolant(const uint8_t data[8], Porsche9872CanData& out) {
    // data[1] - 93 gives °C. data[1]==0 is a not-yet-populated/garbage startup
    // frame that would emit -93°C (impossible); field log can_20260629_140311.csv
    // showed 3 such frames. Treat data[1]==0 as "no reading" rather than -93°C.
    if (data[1] == 0) return;   // leave coolantTempC = NAN, valid stays false
    out.coolantTempC      = (float)data[1] - 93.0f;
    out.coolantTempValid  = true;
}

static inline void decodePorsche9872_0x246_Dme2(const uint8_t data[8], Porsche9872CanData& out) {
    out.kickdownActive           = (data[0] & 0x08U) != 0U;
    out.targetTorquePercent      = (float)data[2] * 0.39f;
    out.pedalRequestedPercent    = clampPercent((float)data[3] * 0.4f);
    out.actualTorquePercent      = (float)data[4] * 0.39f;
    out.atmosphericPressureKpa   = (float)data[5] * 0.5f;
    out.kickdownValid            = true;
    out.targetTorqueValid        = true;
    out.pedalRequestedValid      = true;
    out.actualTorqueValid        = true;
    out.atmosphericPressureValid = true;
}

static inline void decodePorsche9872_0x102_PdkGearCandidate(const uint8_t data[8], Porsche9872CanData& out) {
    // Fallback only. 0x440 is the validated 987.2S PDK status frame in the
    // field log. Do not let this older candidate overwrite an already-seen
    // 0x440 gear, because 0x102 can produce misleading values on this bus.
    if (out.pdkStatusCandidateValid) return;

    const uint8_t gear = (uint8_t)((data[3] >> 4) & 0x0FU);
    if (gear >= 1U && gear <= 8U) {
        out.gearCandidate      = gear;
        out.gearCandidateValid = true;
    }
}

static inline uint8_t decodePorsche9872PdkGearFrom0x440Byte1(const uint8_t raw) {
    // Validated from canraw_b28545.csv against the user's sequence:
    // reverse at the start, then 1→6 shift progression on the road.
    // External 997/987 swap notes support 0x440 as a TCM/PDK status frame, with
    // 0x50-family values corresponding to drive and 0x70-family to reverse.
    // 0xE_ was witnessed over a 60ms period when transitioning from M back to D (0xD9 to 0xE9 to 0X59 when I went D-M-D).
    switch (raw) {
        case 0x51U: return 1U; // Automatic
        case 0x52U: return 2U; // Automatic
        case 0x53U: return 3U; // Automatic
        case 0x54U: return 4U; // Automatic
        case 0x55U: return 5U; // Automatic
        case 0x58U: return 6U; // Automatic
        case 0x59U: return 7U; // Automatic
        case 0xD1U: return 1U; // shifter/paddle commanded
        case 0xD2U: return 2U; // shifter/paddle commanded
        case 0xD3U: return 3U; // shifter/paddle commanded
        case 0xD4U: return 4U; // shifter/paddle commanded
        case 0xD5U: return 5U; // shifter/paddle commanded
        case 0xD8U: return 6U; // shifter/paddle commanded
        case 0xD9U: return 7U; // shifter/paddle commanded
        case 0x77U: return 8U;  // reverse engaged; project convention uses 8=R
        default:    return 0U;  // unknown, transition, neutral, park, drive-range only
    }
}

static inline void decodePorsche9872_0x440_PdkStatusCandidate(const uint8_t data[8], Porsche9872CanData& out) {
    out.pdkGearRaw               = data[1];
    out.pdkSelectorRaw           = data[2];
    out.pdkStatusCandidateValid  = true;

    const uint8_t gear = decodePorsche9872PdkGearFrom0x440Byte1(data[1]);
    if (gear != 0U) {
        out.gearCandidate      = gear;
        out.gearCandidateValid = true;
    }
}

static inline void decodePorsche9872_0x24A_WheelSpeeds(const uint8_t data[8], Porsche9872CanData& out) {
    out.wheelSpeedFrontLeftKph  = decodePorscheWheelSpeedKph_15bit(data[0], data[1]);
    out.wheelSpeedFrontRightKph = decodePorscheWheelSpeedKph_15bit(data[2], data[3]);
    out.wheelSpeedRearLeftKph   = decodePorscheWheelSpeedKph_15bit(data[4], data[5]);
    out.wheelSpeedRearRightKph  = decodePorscheWheelSpeedKph_15bit(data[6], data[7]);
    out.wheelSpeedsValid        = true;
}

static inline void decodePorsche9872_0x441_OilTempPressureCandidate(const uint8_t data[8], Porsche9872CanData& out) {
    out.radiatorFanRequestPercent = clampPercent((float)data[1]);
    out.engineOilTempC            = (float)data[5] - 60.0f;
    out.engineOilPressureBar      = (float)data[6] * 0.04f;

    // Candidate engine-compartment temperature from public 9x7 DME notes. This
    // is not engine coolant temperature and is kept separate from ECT. A zero
    // field decodes to -48°C (impossible startup artifact — 3 such frames in
    // can_20260629_140311.csv); only mark valid when the raw field is nonzero.
    {
        uint8_t ectRaw = (data[7] >> 1) & 0x3FU;
        if (ectRaw != 0) {
            out.engineCompartmentTempC     = (float)ectRaw * 3.0f - 48.0f;
            out.engineCompartmentTempValid = true;
        }
    }

    out.radiatorFanRequestValid    = true;
    out.oilTempValid               = true;
    out.oilPressureValid           = true;
}

static inline void decodePorsche9872_0x44B_BrakePressureCandidate(const uint8_t data[8], Porsche9872CanData& out) {
    out.brakePressureBarCandidate    = (float)data[0];
    out.brakePressureCandidateValid  = true;
}

// ── Main dispatcher — returns true if id was recognised ──────────────────────
static inline bool decodePorsche9872CanFrame(const uint16_t canId,
                                             const uint8_t* data,
                                             const uint8_t  len,
                                             Porsche9872CanData& out) {
    if (data == nullptr || len < 8) return false;
    switch (canId) {
        case PORSCHE_CAN_STEERING_ANGLE_ID:
            decodePorsche9872_0x0C2_Steering(data, out);                 return true;
        case PORSCHE_CAN_VEHICLE_SPEED_ID:
            decodePorsche9872_0x14A_VehicleSpeed(data, out);             return true;
        case PORSCHE_CAN_ENGINE_1_ID:
            decodePorsche9872_0x242_Engine1(data, out);                  return true;
        case PORSCHE_CAN_COOLANT_TEMP_ID:
            decodePorsche9872_0x245_Coolant(data, out);                  return true;
        case PORSCHE_CAN_DME_2_ID:
            decodePorsche9872_0x246_Dme2(data, out);                     return true;
        case PORSCHE_CAN_PDK_GEAR_CANDIDATE_ID:
            decodePorsche9872_0x102_PdkGearCandidate(data, out);         return true;
        case PORSCHE_CAN_PDK_STATUS_CANDIDATE_ID:
            decodePorsche9872_0x440_PdkStatusCandidate(data, out);       return true;
        case PORSCHE_CAN_WHEEL_SPEEDS_ID:
            decodePorsche9872_0x24A_WheelSpeeds(data, out);              return true;
        case PORSCHE_CAN_OIL_TEMP_PRESS_CAND_ID:
            decodePorsche9872_0x441_OilTempPressureCandidate(data, out); return true;
        case PORSCHE_CAN_BRAKE_PRESS_CAND_ID:
            decodePorsche9872_0x44B_BrakePressureCandidate(data, out);   return true;
        default:
            return false;
    }
}

static inline void printPorsche9872CanData(const Porsche9872CanData& d, Stream& port = Serial) {
    if (d.rpmValid)                    { port.print(F("RPM="));       port.print(d.engineRpm, 0);                  port.print(F("  ")); }
    if (d.throttleActualValid)         { port.print(F("TPS="));       port.print(d.throttleActualPercent, 1);      port.print(F("%  ")); }
    if (d.pedalRequestedValid)         { port.print(F("Pedal="));     port.print(d.pedalRequestedPercent, 1);      port.print(F("%  ")); }
    if (d.vehicleSpeedValid)           { port.print(F("Veh="));       port.print(d.vehicleSpeedKph, 1);            port.print(F("kph  ")); }
    if (d.coolantTempValid)            { port.print(F("ECT="));       port.print(d.coolantTempC, 0);               port.print(F("°C  ")); }
    if (d.gearCandidateValid) {
        port.print(F("Gear="));
        if (d.gearCandidate == 8U) port.print(F("R"));
        else port.print(d.gearCandidate);
        port.print(F("  "));
    }
    if (d.pdkStatusCandidateValid)     { port.print(F("PdkRaw=0x"));  if (d.pdkGearRaw < 16) port.print('0'); port.print(d.pdkGearRaw, HEX); port.print(F("  ")); }
    if (d.steeringAngleValid)          { port.print(F("Steer="));     port.print(d.steeringAngleDeg, 1);           port.print(F("°  ")); }
    if (d.steeringRateValid)           { port.print(F("Rate="));      port.print(d.steeringRateDegPerSec, 1);      port.print(F("°/s  ")); }
    if (d.wheelSpeedsValid) {
        port.print(F("WS FL=")); port.print(d.wheelSpeedFrontLeftKph, 1);
        port.print(F(" FR="));   port.print(d.wheelSpeedFrontRightKph, 1);
        port.print(F(" RL="));   port.print(d.wheelSpeedRearLeftKph, 1);
        port.print(F(" RR="));   port.print(d.wheelSpeedRearRightKph, 1);
        port.print(F(" kph  "));
    }
    if (d.oilTempValid)                { port.print(F("OilT="));      port.print(d.engineOilTempC, 0);             port.print(F("°C  ")); }
    if (d.oilPressureValid)            { port.print(F("OilP="));      port.print(d.engineOilPressureBar, 2);       port.print(F("bar  ")); }
    if (d.brakePressureCandidateValid) { port.print(F("Brake*="));    port.print(d.brakePressureBarCandidate, 1);  port.print(F("bar  ")); }
    if (d.kickdownValid)               { port.print(F("Kick="));      port.print(d.kickdownActive ? F("on") : F("off")); port.print(F("  ")); }
    if (d.atmosphericPressureValid)    { port.print(F("Baro="));      port.print(d.atmosphericPressureKpa, 1);     port.print(F("kPa  ")); }
    port.println();
}
