/*
    Porsche Boxster/Cayman 718 / 982 CAN Decode Specification
    Target vehicle:
      - Porsche 718 Boxster / Cayman 982, including 2017-2019 four-cylinder cars
      - Direct DRIVE CAN broadcast frames, not generic SAE OBD-II PID requests
      - Optional TPMS decoder for MMI CAN frame 0x673 when that bus is logged

    CAN bus:
      - Standard 11-bit CAN identifiers
      - Expected bus speed: 500000 bps
      - Byte numbering: data[0] through data[7]
      - Bit extraction follows RaceChrono/DBC Intel little-endian bit numbering:
        bit 0 is the least-significant bit of data[0].
      - CAN IDs below are written in hexadecimal, with decimal source IDs noted.

    Validation notes:
      - These decoders are based on public 718 / 982 reverse-engineered DRIVE CAN data,
        VAG DBC definitions for the shared 0x086/0x101/0x105 message layouts, and the
        user's 2018 718 captures.
      - 0x105 bits 48..55 are accelerator-pedal raw position (MO_Fahrpedalrohwert_01),
        not throttle-plate position. This is confirmed by the field dropping to zero
        during cruise control while engine load remains present.
      - Steering direction is carried separately at bit 29; steering-rate direction is
        bit 30. Yaw-rate direction is bit 54. Those sign bits are applied here.
      - The public 718 coolant formula used -75 C. The user's vehicle capture produced an
        implausible 96-129 C range; -100 C yields the plausible observed 71-104 C range.
        Keep PORSCHE718_COOLANT_OFFSET_C isolated below for easy revalidation.
      - 0x107 (oil pressure/manifold pressure) and 0x30B (display speed) are DRIVE-bus
        messages and may be absent on a chassis/body-bus tap. 0x673 TPMS is on MMI CAN.
*/

#pragma once
#include <Arduino.h>
#include <math.h>

static constexpr uint16_t PORSCHE718_CAN_MANUAL_GEAR_ID      = 0x081;  // decimal 129
static constexpr uint16_t PORSCHE718_CAN_STEERING_ID         = 0x086;  // decimal 134
static constexpr uint16_t PORSCHE718_CAN_PSM_MODE_ID          = 0x100;  // decimal 256
static constexpr uint16_t PORSCHE718_CAN_ESP_02_ID           = 0x101;  // decimal 257
static constexpr uint16_t PORSCHE718_CAN_PDK_GEAR_ID         = 0x102;  // decimal 258
static constexpr uint16_t PORSCHE718_CAN_WHEEL_SPEEDS_ID     = 0x103;  // decimal 259
static constexpr uint16_t PORSCHE718_CAN_TRANSMISSION_ID     = 0x104;  // decimal 260
static constexpr uint16_t PORSCHE718_CAN_THROTTLE_ID         = 0x105;  // decimal 261
static constexpr uint16_t PORSCHE718_CAN_BRAKES_ID           = 0x106;  // decimal 262
static constexpr uint16_t PORSCHE718_CAN_MOTOR_ID            = 0x107;  // decimal 263
static constexpr uint16_t PORSCHE718_CAN_DISPLAY_SPEED_ID    = 0x30B;  // decimal 779
static constexpr uint16_t PORSCHE718_CAN_DRIVE_MODE_ID        = 0x30E;  // decimal 782
static constexpr uint16_t PORSCHE718_CAN_TEMPS_ID            = 0x640;  // decimal 1600
static constexpr uint16_t PORSCHE718_CAN_DIAGNOSE_01_ID      = 0x6B2;  // decimal 1714
static constexpr uint16_t PORSCHE718_CAN_FUEL_ID             = 0x6B7;  // decimal 1719
static constexpr uint16_t PORSCHE718_CAN_TPMS_ID             = 0x673;  // decimal 1651, MMI CAN TPMS

static constexpr uint8_t PORSCHE718_DRIVE_MODE_NORMAL = 0U;
static constexpr uint8_t PORSCHE718_DRIVE_MODE_SPORT  = 2U;

static constexpr uint8_t PORSCHE718_PSM_MODE_NORMAL    = 0U;
static constexpr uint8_t PORSCHE718_PSM_MODE_SPORT     = 1U;
static constexpr uint8_t PORSCHE718_PSM_MODE_OFF       = 3U;

static constexpr uint8_t PORSCHE718_PDK_STATE_STAGED_OR_OPEN = 0U;
static constexpr uint8_t PORSCHE718_PDK_STATE_DRIVE_ENGAGED  = 1U;
static constexpr uint8_t PORSCHE718_PDK_STATE_NO_DRIVE       = 3U;

static constexpr float PORSCHE718_TPMS_PSI_PER_0P1_BAR       = 1.45038f;
static constexpr float PORSCHE718_TPMS_PSI_HALF_STEP_OFFSET  = 0.72654f;
static constexpr float PORSCHE718_TPMS_TEMP_OFFSET_C         = -60.0f;
static constexpr float PORSCHE718_COOLANT_OFFSET_C          = -100.0f;
static constexpr float PORSCHE718_BRAKE_ZERO_CLAMP_BAR      = 0.35f;
static constexpr float PORSCHE718_STANDARD_GRAVITY_MPS2     = 9.80665f;

struct Porsche718CanData {
    bool rpmValid                    = false;
    bool acceleratorPedalValid       = false;
    bool steeringAngleValid          = false;
    bool steeringRateValid           = false;
    bool vehicleDynamicsValid        = false;
    bool wheelSpeedsValid            = false;
    bool brakePressureValid          = false;
    bool brakeLightValid             = false;
    bool clutchPositionValid         = false;
    bool oilPressureValid            = false;
    bool motorSpeedCandidateValid    = false;
    bool boostPressureValid          = false;
    bool displayedSpeedValid         = false;
    bool oilTempValid                = false;
    bool coolantTempValid            = false;
    bool intakeTempValid             = false;
    bool odometerValid               = false;
    bool clockValid                  = false;
    bool fuelLevelValid              = false;
    bool outsideTempValid            = false;
    bool pdkGearValid                = false;
    bool pdkTransmissionStateValid   = false;
    bool manualGearValid             = false;
    bool driveModeValid              = false;
    bool psmModeValid                = false;
    bool tpmsPressureValid           = false;
    bool tpmsTemperatureValid        = false;

    float   engineRpm                    = NAN;
    float   acceleratorPedalPercent        = NAN;
    float   steeringAngleDeg              = NAN;
    float   steeringRateDegPerSec         = NAN;
    float   yawRateDegPerSec              = NAN;
    float   lateralAccelG                 = NAN;
    float   longitudinalAccelMps2         = NAN;
    float   wheelSpeedFrontLeftKph        = NAN;
    float   wheelSpeedFrontRightKph       = NAN;
    float   wheelSpeedRearLeftKph         = NAN;
    float   wheelSpeedRearRightKph        = NAN;
    float   brakePressureBar              = NAN;
    bool    brakeLightOn                  = false;
    float   clutchPositionPercent         = NAN;
    float   engineOilPressureBar          = NAN;
    float   motorSpeedRpmCandidate        = NAN;
    float   boostPressureBar              = NAN;
    float   displayedSpeedKph             = NAN;
    float   engineOilTempC                = NAN;
    float   coolantTempC                  = NAN;
    float   intakeAirTempC                = NAN;
    uint32_t odometerKm                   = 0;
    uint16_t clockYear                    = 0;
    uint8_t clockMonth                    = 0;
    uint8_t clockDay                      = 0;
    uint8_t clockHour                     = 0;
    uint8_t clockMinute                   = 0;
    uint8_t clockSecond                   = 0;
    float   fuelLevelLiters               = NAN;
    float   outsideTempC                  = NAN;
    float   tpmsPressureFrontLeftPsi      = NAN;
    float   tpmsPressureFrontRightPsi     = NAN;
    float   tpmsPressureRearLeftPsi       = NAN;
    float   tpmsPressureRearRightPsi      = NAN;
    float   tpmsTempFrontLeftC            = NAN;
    float   tpmsTempFrontRightC           = NAN;
    float   tpmsTempRearLeftC             = NAN;
    float   tpmsTempRearRightC            = NAN;
    uint8_t pdkGear                       = 0;
    uint8_t pdkTransmissionState          = PORSCHE718_PDK_STATE_STAGED_OR_OPEN;
    bool    pdkDriveEngaged               = false;
    bool    pdkNoDriveOrFault             = false;
    uint8_t manualGear                    = 0;
    uint8_t driveMode                     = PORSCHE718_DRIVE_MODE_NORMAL;
    uint8_t psmMode                       = PORSCHE718_PSM_MODE_NORMAL;
    bool    sportModeOn                   = false;
    bool    psmSportModeOn                = false;
    bool    psmOff                        = false;
};

// ── Bit helpers ───────────────────────────────────────────────────────────────
static inline uint32_t porsche718BitsToUintLe(const uint8_t data[8],
                                              const uint8_t startBit,
                                              const uint8_t length) {
    uint32_t value = 0;
    for (uint8_t i = 0; i < length; ++i) {
        const uint8_t absoluteBit = startBit + i;
        const uint8_t byteIndex = absoluteBit >> 3;
        const uint8_t bitIndex = absoluteBit & 0x07;
        if ((data[byteIndex] & (uint8_t)(1U << bitIndex)) != 0U) {
            value |= (uint32_t)1U << i;
        }
    }
    return value;
}

static inline bool porsche718BitToBool(const uint8_t data[8], const uint8_t bitIndex) {
    return porsche718BitsToUintLe(data, bitIndex, 1) != 0U;
}

// ── Per-ID decoders ───────────────────────────────────────────────────────────
static inline void decodePorsche718_0x081_ManualGear(const uint8_t data[8], Porsche718CanData& out) {
    /*
        CAN ID 0x081 / decimal 129
        Manual gear values from the public map:
          0 = stopped, 1-6 = forward gears, 13 = reverse, 14 = not engaged.
    */
    out.manualGear = (uint8_t)porsche718BitsToUintLe(data, 49, 4);
    out.manualGearValid = true;
}

static inline void decodePorsche718_0x086_Steering(const uint8_t data[8], Porsche718CanData& out) {
    const float angleMagnitude = (float)porsche718BitsToUintLe(data, 16, 13) * 0.1f;
    const float rateMagnitude  = (float)porsche718BitsToUintLe(data, 31, 9) * 5.0f;
    const bool angleNegative   = porsche718BitToBool(data, 29);
    const bool rateNegative    = porsche718BitToBool(data, 30);

    out.steeringAngleDeg      = angleNegative ? -angleMagnitude : angleMagnitude;
    out.steeringRateDegPerSec = rateNegative  ? -rateMagnitude  : rateMagnitude;
    out.steeringAngleValid    = true;
    out.steeringRateValid     = true;
}

static inline void decodePorsche718_0x100_PsmMode(const uint8_t data[8], Porsche718CanData& out) {
    /*
        CAN ID 0x100 / decimal 256
        PSM mode field identified from the user's controlled 2018 718 Boxster DRIVE-CAN log.
        Observed sequence while cycling PSM modes:
          0 = PSM normal
          1 = PSM Sport
          3 = PSM off
        If later testing shows a different label for a raw value, update the constants above
        while keeping this raw nibble extraction unchanged.
    */
    out.psmMode        = (uint8_t)(data[6] & 0x0FU);
    out.psmSportModeOn = (out.psmMode == PORSCHE718_PSM_MODE_SPORT);
    out.psmOff         = (out.psmMode == PORSCHE718_PSM_MODE_OFF);
    out.psmModeValid   = true;
}

static inline void decodePorsche718_0x101_Esp02(const uint8_t data[8], Porsche718CanData& out) {
    out.lateralAccelG         = (float)porsche718BitsToUintLe(data, 16, 8) * 0.01f - 1.27f;
    out.longitudinalAccelMps2 = (float)porsche718BitsToUintLe(data, 24, 10) * 0.03125f - 16.0f;

    const float yawMagnitude = (float)porsche718BitsToUintLe(data, 40, 14) * 0.01f;
    const bool yawNegative   = porsche718BitToBool(data, 54);
    out.yawRateDegPerSec     = yawNegative ? -yawMagnitude : yawMagnitude;
    out.vehicleDynamicsValid = true;
}

static inline void decodePorsche718_0x102_PdkGear(const uint8_t data[8], Porsche718CanData& out) {
    /*
        CAN ID 0x102 / decimal 258
        PDK gear values from the public map:
          1-7 = forward gears, 8 = reverse.

        Public RaceChrono formula: bitsToUInt(raw,28,4). In this C++ byte layout,
        the observed equivalent is the low nibble of data[3], not the Intel/little-
        endian helper porsche718BitsToUintLe(data,28,4). In the captured 718 PDK
        log, data[3] values 0x11, 0x12, and 0x13 corresponded to gears 1, 2,
        and 3. The high nibble of data[3] behaved as a PDK drive/clutch state.

        Observed PDK state labels from the transmission-fault log:
          0 = staged/stopped/clutch-open candidate
          1 = normal drive-engaged candidate
          3 = no-drive/fault/disengaged candidate
    */
    out.pdkGear = (uint8_t)(data[3] & 0x0FU);
    out.pdkTransmissionState = (uint8_t)(data[3] >> 4);
    out.pdkDriveEngaged = (out.pdkTransmissionState == PORSCHE718_PDK_STATE_DRIVE_ENGAGED);
    out.pdkNoDriveOrFault = ((data[5] & 0x01U) != 0U) ||
                            (out.pdkTransmissionState == PORSCHE718_PDK_STATE_NO_DRIVE);
    out.pdkGearValid = (out.pdkGear >= 1U && out.pdkGear <= 8U);
    out.pdkTransmissionStateValid = true;
}

static inline float decodePorsche718WheelSpeedKph(const uint8_t data[8], const uint8_t startBit) {
    const uint16_t raw = (uint16_t)porsche718BitsToUintLe(data, startBit, 12);
    return (raw == 0x0FFFU) ? NAN : (float)raw * 0.103f;
}

static inline void decodePorsche718_0x103_WheelSpeeds(const uint8_t data[8], Porsche718CanData& out) {
    out.wheelSpeedFrontLeftKph  = decodePorsche718WheelSpeedKph(data, 16);
    out.wheelSpeedFrontRightKph = decodePorsche718WheelSpeedKph(data, 28);
    out.wheelSpeedRearLeftKph   = decodePorsche718WheelSpeedKph(data, 40);
    out.wheelSpeedRearRightKph  = decodePorsche718WheelSpeedKph(data, 52);
    out.wheelSpeedsValid        = !isnan(out.wheelSpeedFrontLeftKph)  ||
                                  !isnan(out.wheelSpeedFrontRightKph) ||
                                  !isnan(out.wheelSpeedRearLeftKph)   ||
                                  !isnan(out.wheelSpeedRearRightKph);
}

static inline void decodePorsche718_0x104_Transmission(const uint8_t data[8], Porsche718CanData& out) {
    out.clutchPositionPercent = (float)porsche718BitsToUintLe(data, 32, 8) * 0.4f - 10.0f;
    out.clutchPositionValid   = true;
}

static inline void decodePorsche718_0x105_EnginePedal(const uint8_t data[8], Porsche718CanData& out) {
    const uint16_t rpmRaw = (uint16_t)porsche718BitsToUintLe(data, 16, 16);
    const uint8_t pedalRaw = (uint8_t)porsche718BitsToUintLe(data, 48, 8);

    out.engineRpm = (rpmRaw == 0xFFFFU) ? NAN : (float)rpmRaw * 0.25f;
    out.acceleratorPedalPercent = (pedalRaw == 0xFFU) ? NAN : (float)pedalRaw * 0.4f;
    out.rpmValid = !isnan(out.engineRpm);
    out.acceleratorPedalValid = !isnan(out.acceleratorPedalPercent);
}

static inline void decodePorsche718_0x106_Brakes(const uint8_t data[8], Porsche718CanData& out) {
    const uint16_t raw = (uint16_t)porsche718BitsToUintLe(data, 16, 10);
    if (raw == 0x03FFU) {
        out.brakePressureBar = NAN;
        out.brakePressureValid = false;
    } else {
        float pressure = (float)raw * 0.3f - 30.0f;
        if (fabsf(pressure) <= PORSCHE718_BRAKE_ZERO_CLAMP_BAR) pressure = 0.0f;
        out.brakePressureBar = pressure;
        out.brakePressureValid = true;
    }
    out.brakeLightOn    = porsche718BitToBool(data, 26);
    out.brakeLightValid = true;
}

static inline void decodePorsche718_0x107_Motor(const uint8_t data[8], Porsche718CanData& out) {
    out.engineOilPressureBar    = (float)porsche718BitsToUintLe(data, 16, 8) * 0.04f;
    out.motorSpeedRpmCandidate  = (float)porsche718BitsToUintLe(data, 24, 12) * 3.0f;
    out.boostPressureBar        = (float)porsche718BitsToUintLe(data, 39, 9) * 0.01f;
    out.oilPressureValid        = true;
    out.motorSpeedCandidateValid = true;
    out.boostPressureValid      = true;
}

static inline void decodePorsche718_0x30B_DisplaySpeed(const uint8_t data[8], Porsche718CanData& out) {
    const uint16_t raw = (uint16_t)porsche718BitsToUintLe(data, 48, 10);
    out.displayedSpeedKph   = (raw == 0x03FFU) ? NAN : (float)raw * 0.32f;
    out.displayedSpeedValid = !isnan(out.displayedSpeedKph);
}

static inline void decodePorsche718_0x30E_DriveMode(const uint8_t data[8], Porsche718CanData& out) {
    /*
        CAN ID 0x30E / decimal 782
        Drive mode field identified from the user's controlled 2018 718 Boxster DRIVE-CAN log.
        Observed sequence while toggling Sport mode three times:
          0 = normal drive mode
          2 = Sport mode active
        Sport Plus, Individual, and other equipment-dependent states were not exercised in the
        reference log, so additional raw values should be preserved if observed later.
    */
    out.driveMode    = (uint8_t)(data[5] >> 4);
    out.sportModeOn  = (out.driveMode == PORSCHE718_DRIVE_MODE_SPORT);
    out.driveModeValid = true;
}

static inline void decodePorsche718_0x640_Temps(const uint8_t data[8], Porsche718CanData& out) {
    const uint8_t iatRaw = (uint8_t)porsche718BitsToUintLe(data, 8, 8);
    const uint8_t oilRaw = (uint8_t)porsche718BitsToUintLe(data, 16, 8);
    const uint8_t ectRaw = (uint8_t)porsche718BitsToUintLe(data, 24, 8);

    out.intakeAirTempC = (iatRaw == 0xFFU) ? NAN : (float)iatRaw * 0.75f - 48.0f;
    out.engineOilTempC = (oilRaw == 0xFFU) ? NAN : (float)oilRaw - 60.0f;
    out.coolantTempC   = (ectRaw == 0xFFU) ? NAN : (float)ectRaw + PORSCHE718_COOLANT_OFFSET_C;
    out.intakeTempValid  = !isnan(out.intakeAirTempC);
    out.oilTempValid     = !isnan(out.engineOilTempC);
    out.coolantTempValid = !isnan(out.coolantTempC);
}

static inline void decodePorsche718_0x6B2_Diagnose01(const uint8_t data[8], Porsche718CanData& out) {
    out.odometerKm   = porsche718BitsToUintLe(data, 8, 20);
    out.clockYear    = (uint16_t)(porsche718BitsToUintLe(data, 28, 7) + 2000U);
    out.clockMonth   = (uint8_t)porsche718BitsToUintLe(data, 35, 4);
    out.clockDay     = (uint8_t)porsche718BitsToUintLe(data, 39, 5);
    out.clockHour    = (uint8_t)porsche718BitsToUintLe(data, 44, 5);
    out.clockMinute  = (uint8_t)porsche718BitsToUintLe(data, 49, 6);
    out.clockSecond  = (uint8_t)porsche718BitsToUintLe(data, 55, 6);
    out.odometerValid = true;
    out.clockValid    = true;
}

static inline void decodePorsche718_0x6B7_Fuel(const uint8_t data[8], Porsche718CanData& out) {
    out.fuelLevelLiters = (float)porsche718BitsToUintLe(data, 40, 7);
    out.outsideTempC    = (float)porsche718BitsToUintLe(data, 56, 8) * 0.5f - 50.0f;
    out.fuelLevelValid  = true;
    out.outsideTempValid = true;
}

static inline void decodePorsche718_0x673_Tpms(const uint8_t data[8], Porsche718CanData& out) {
    /*
        CAN ID 0x673 / decimal 1651
        TPMS frame reported on the MMI CAN bus, not the DRIVE CAN bus. This ID was
        not present in the captured 718 DRIVE-CAN log, so this decoder is intended
        for a logger connected to the MMI bus or to a gateway feed that forwards it.

        Layout reported by community testing:
          data[0..3] = pressure FL, FR, RL, RR in 0.1 bar units
          data[4..7] = temperature FL, FR, RL, RR with -60 C offset

        Pressure compensation keeps the poster's half-step correction because the
        TPMS sensors round down.
    */
    out.tpmsPressureFrontLeftPsi  = (float)data[0] * PORSCHE718_TPMS_PSI_PER_0P1_BAR + PORSCHE718_TPMS_PSI_HALF_STEP_OFFSET;
    out.tpmsPressureFrontRightPsi = (float)data[1] * PORSCHE718_TPMS_PSI_PER_0P1_BAR + PORSCHE718_TPMS_PSI_HALF_STEP_OFFSET;
    out.tpmsPressureRearLeftPsi   = (float)data[2] * PORSCHE718_TPMS_PSI_PER_0P1_BAR + PORSCHE718_TPMS_PSI_HALF_STEP_OFFSET;
    out.tpmsPressureRearRightPsi  = (float)data[3] * PORSCHE718_TPMS_PSI_PER_0P1_BAR + PORSCHE718_TPMS_PSI_HALF_STEP_OFFSET;

    out.tpmsTempFrontLeftC        = (float)data[4] + PORSCHE718_TPMS_TEMP_OFFSET_C;
    out.tpmsTempFrontRightC       = (float)data[5] + PORSCHE718_TPMS_TEMP_OFFSET_C;
    out.tpmsTempRearLeftC         = (float)data[6] + PORSCHE718_TPMS_TEMP_OFFSET_C;
    out.tpmsTempRearRightC        = (float)data[7] + PORSCHE718_TPMS_TEMP_OFFSET_C;

    out.tpmsPressureValid         = true;
    out.tpmsTemperatureValid      = true;
}

// ── Main dispatcher — returns true if id was recognised ──────────────────────
static inline bool decodePorsche718CanFrame(const uint16_t canId,
                                            const uint8_t* data,
                                            const uint8_t  len,
                                            Porsche718CanData& out) {
    if (data == nullptr || len < 8) return false;

    switch (canId) {
        case PORSCHE718_CAN_MANUAL_GEAR_ID:
            decodePorsche718_0x081_ManualGear(data, out);       return true;
        case PORSCHE718_CAN_STEERING_ID:
            decodePorsche718_0x086_Steering(data, out);         return true;
        case PORSCHE718_CAN_PSM_MODE_ID:
            decodePorsche718_0x100_PsmMode(data, out);          return true;
        case PORSCHE718_CAN_ESP_02_ID:
            decodePorsche718_0x101_Esp02(data, out);            return true;
        case PORSCHE718_CAN_PDK_GEAR_ID:
            decodePorsche718_0x102_PdkGear(data, out);          return true;
        case PORSCHE718_CAN_WHEEL_SPEEDS_ID:
            decodePorsche718_0x103_WheelSpeeds(data, out);      return true;
        case PORSCHE718_CAN_TRANSMISSION_ID:
            decodePorsche718_0x104_Transmission(data, out);     return true;
        case PORSCHE718_CAN_THROTTLE_ID:
            decodePorsche718_0x105_EnginePedal(data, out);      return true;
        case PORSCHE718_CAN_BRAKES_ID:
            decodePorsche718_0x106_Brakes(data, out);           return true;
        case PORSCHE718_CAN_MOTOR_ID:
            decodePorsche718_0x107_Motor(data, out);            return true;
        case PORSCHE718_CAN_DISPLAY_SPEED_ID:
            decodePorsche718_0x30B_DisplaySpeed(data, out);     return true;
        case PORSCHE718_CAN_DRIVE_MODE_ID:
            decodePorsche718_0x30E_DriveMode(data, out);        return true;
        case PORSCHE718_CAN_TEMPS_ID:
            decodePorsche718_0x640_Temps(data, out);            return true;
        case PORSCHE718_CAN_DIAGNOSE_01_ID:
            decodePorsche718_0x6B2_Diagnose01(data, out);       return true;
        case PORSCHE718_CAN_FUEL_ID:
            decodePorsche718_0x6B7_Fuel(data, out);             return true;
        case PORSCHE718_CAN_TPMS_ID:
            decodePorsche718_0x673_Tpms(data, out);             return true;
        default:
            return false;
    }
}

static inline void printPorsche718TwoDigit(const uint8_t value, Stream& port) {
    if (value < 10U) {
        port.print(F("0"));
    }
    port.print(value);
}

static inline void printPorsche718CanData(const Porsche718CanData& d, Stream& port = Serial) {
    if (d.rpmValid)               { port.print(F("RPM="));      port.print(d.engineRpm, 0);                  port.print(F("  ")); }
    if (d.acceleratorPedalValid)  { port.print(F("Pedal="));    port.print(d.acceleratorPedalPercent, 1);     port.print(F("%  ")); }
    if (d.pdkGearValid)           { port.print(F("PDK="));      port.print(d.pdkGear);                       port.print(F("  ")); }
    if (d.pdkTransmissionStateValid) {
        port.print(F("PDKState=")); port.print(d.pdkTransmissionState);
        port.print(d.pdkNoDriveOrFault ? F("(NoDrive/Fault)  ") : (d.pdkDriveEngaged ? F("(Engaged)  ") : F("  ")));
    }
    if (d.manualGearValid)        { port.print(F("ManGear="));  port.print(d.manualGear);                    port.print(F("  ")); }
    if (d.driveModeValid) {
        port.print(F("DriveMode=")); port.print(d.driveMode);
        port.print(d.sportModeOn ? F("(Sport)  ") : (d.driveMode == PORSCHE718_DRIVE_MODE_NORMAL ? F("(Normal)  ") : F("(Unknown)  ")));
    }
    if (d.psmModeValid) {
        port.print(F("PSM=")); port.print(d.psmMode);
        port.print(d.psmOff ? F("(Off)  ") : (d.psmSportModeOn ? F("(Sport)  ") : (d.psmMode == PORSCHE718_PSM_MODE_NORMAL ? F("(Normal)  ") : F("(Unknown)  "))));
    }
    if (d.steeringAngleValid)     { port.print(F("Steer="));    port.print(d.steeringAngleDeg, 1);           port.print(F("°  ")); }
    if (d.steeringRateValid)      { port.print(F("Rate="));     port.print(d.steeringRateDegPerSec, 0);      port.print(F("°/s  ")); }
    if (d.displayedSpeedValid)    { port.print(F("V="));        port.print(d.displayedSpeedKph, 1);          port.print(F("kph  ")); }
    if (d.wheelSpeedsValid) {
        port.print(F("WS FL=")); port.print(d.wheelSpeedFrontLeftKph, 1);
        port.print(F(" FR="));   port.print(d.wheelSpeedFrontRightKph, 1);
        port.print(F(" RL="));   port.print(d.wheelSpeedRearLeftKph, 1);
        port.print(F(" RR="));   port.print(d.wheelSpeedRearRightKph, 1);
        port.print(F(" kph  "));
    }
    if (d.brakePressureValid)     { port.print(F("Brake="));    port.print(d.brakePressureBar, 1);           port.print(F("bar  ")); }
    if (d.brakeLightValid)        { port.print(F("BrakeLt="));  port.print(d.brakeLightOn ? F("on") : F("off")); port.print(F("  ")); }
    if (d.oilPressureValid)       { port.print(F("OilP="));     port.print(d.engineOilPressureBar, 2);       port.print(F("bar  ")); }
    if (d.boostPressureValid)     { port.print(F("Boost="));    port.print(d.boostPressureBar, 2);           port.print(F("bar  ")); }
    if (d.motorSpeedCandidateValid) { port.print(F("MO_RPM*=")); port.print(d.motorSpeedRpmCandidate, 0);    port.print(F("  ")); }
    if (d.oilTempValid)           { port.print(F("OilT="));     port.print(d.engineOilTempC, 0);             port.print(F("°C  ")); }
    if (d.coolantTempValid)       { port.print(F("ECT="));      port.print(d.coolantTempC, 0);               port.print(F("°C  ")); }
    if (d.intakeTempValid)        { port.print(F("IAT="));      port.print(d.intakeAirTempC, 0);             port.print(F("°C  ")); }
    if (d.vehicleDynamicsValid) {
        port.print(F("LatG="));   port.print(d.lateralAccelG, 2);
        port.print(F(" Long="));  port.print(d.longitudinalAccelMps2, 2);
        port.print(F("m/s2 Yaw=")); port.print(d.yawRateDegPerSec, 2);
        port.print(F("°/s  "));
    }
    if (d.clutchPositionValid)    { port.print(F("Clutch="));   port.print(d.clutchPositionPercent, 1);      port.print(F("%  ")); }
    if (d.fuelLevelValid)         { port.print(F("Fuel="));     port.print(d.fuelLevelLiters, 0);            port.print(F("L  ")); }
    if (d.outsideTempValid)       { port.print(F("OAT="));      port.print(d.outsideTempC, 1);               port.print(F("°C  ")); }
    if (d.tpmsPressureValid) {
        port.print(F("TPMSpsi FL=")); port.print(d.tpmsPressureFrontLeftPsi, 1);
        port.print(F(" FR="));        port.print(d.tpmsPressureFrontRightPsi, 1);
        port.print(F(" RL="));        port.print(d.tpmsPressureRearLeftPsi, 1);
        port.print(F(" RR="));        port.print(d.tpmsPressureRearRightPsi, 1);
        port.print(F("  "));
    }
    if (d.tpmsTemperatureValid) {
        port.print(F("TPMSTc FL=")); port.print(d.tpmsTempFrontLeftC, 0);
        port.print(F(" FR="));       port.print(d.tpmsTempFrontRightC, 0);
        port.print(F(" RL="));       port.print(d.tpmsTempRearLeftC, 0);
        port.print(F(" RR="));       port.print(d.tpmsTempRearRightC, 0);
        port.print(F("  "));
    }
    if (d.odometerValid)          { port.print(F("Odo="));      port.print(d.odometerKm);                    port.print(F("km  ")); }
    if (d.clockValid) {
        port.print(F("Clock="));
        port.print(d.clockYear);
        port.print(F("-"));
        printPorsche718TwoDigit(d.clockMonth, port);
        port.print(F("-"));
        printPorsche718TwoDigit(d.clockDay, port);
        port.print(F(" "));
        printPorsche718TwoDigit(d.clockHour, port);
        port.print(F(":"));
        printPorsche718TwoDigit(d.clockMinute, port);
        port.print(F(":"));
        printPorsche718TwoDigit(d.clockSecond, port);
        port.print(F("  "));
    }
    port.println();
}
