#pragma once
/*
 * can_porsche718_extra.h — 718-only supplemental telemetry snapshot
 * =================================================================
 * Keeps 718 channels that do not exist in the shared CanData structure out of
 * the 987.2 decoder path. Consumers may request one coherent snapshot; when the
 * active vehicle is not a 718, the accessor returns false and every field stays
 * NAN.
 *
 * Units are metric at this boundary. BLE/WiFi/SD convert at the output site so
 * all three outputs advertise values consistent with their metadata.
 */

#include <Arduino.h>
#include <math.h>

struct Porsche718ExtraData {
    float intakeAirTempC          = NAN;
    float manifoldAbsPressureBar  = NAN;  // public 0x107 "manifold/boost pressure" field
    float massAirFlowGps          = NAN;  // no passive 718 broadcast mapping documented yet
    float canLateralAccelG        = NAN;
    float canLongitudinalAccelG   = NAN;
    float canYawRateDegPerSec     = NAN;
    float clutchPositionPercent   = NAN;
    float outsideTempC            = NAN;
    float odometerKm              = NAN;
    float driveMode               = NAN;
    float psmMode                 = NAN;
    float pdkState                = NAN;
    float pdkNoDriveOrFault       = NAN;
    float gearValid               = NAN;  // 1 = selected gear valid, 0 = unavailable
    float tpmsFrontLeftPsi        = NAN;
    float tpmsFrontRightPsi       = NAN;
    float tpmsRearLeftPsi         = NAN;
    float tpmsRearRightPsi        = NAN;
    float tpmsTempFrontLeftC      = NAN;
    float tpmsTempFrontRightC     = NAN;
    float tpmsTempRearLeftC       = NAN;
    float tpmsTempRearRightC      = NAN;
};

// Copies a mutex-protected snapshot. Returns true only while the active profile
// is the Porsche 718 profile; false means all returned fields should be treated
// as unavailable.
bool can_getPorsche718Extra(Porsche718ExtraData& out);
