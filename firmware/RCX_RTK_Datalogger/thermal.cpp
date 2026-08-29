#include "thermal.h"

// One Schmitt-trigger gate per threshold: engages at onC, clears at offC
// (offC < onC gives the hysteresis band). NAN holds the current state rather
// than transitioning either way — see thermal.h for why.
//
// active is volatile: thermal_update() writes it from the main loop task, but
// three of the five getters below are read from sdLogTask, a separate
// FreeRTOS task — the same reason sd_log.cpp marks its own cross-task flags
// (sdReady, dirScanReq, etc.) volatile. A plain bool would risk the compiler
// caching a stale value in a register across loop iterations on the reading
// task; volatile forces every read to actually happen.
struct ThermalGate {
    float onC, offC;
    volatile bool active = false;
    bool update(float t) {
        if (isnan(t)) return active;
        if (!active && t >= onC)      active = true;
        else if (active && t < offC)  active = false;
        return active;
    }
};

static ThermalGate gBacklightDim {100.0f,  95.0f};
static ThermalGate gLcdOff       {105.0f, 100.0f};
static ThermalGate gCanSniffInh  {105.0f, 100.0f};
static ThermalGate gSatCanImuInh {112.0f, 107.0f};
static ThermalGate gGpsReduce    {115.0f, 110.0f};

void thermal_update(float espTempC) {
    gBacklightDim.update(espTempC);
    gLcdOff.update(espTempC);
    gCanSniffInh.update(espTempC);
    gSatCanImuInh.update(espTempC);
    gGpsReduce.update(espTempC);
}

bool thermal_backlightDim()     { return gBacklightDim.active; }
bool thermal_lcdOff()           { return gLcdOff.active; }
bool thermal_canSniffInhibit()  { return gCanSniffInh.active; }
bool thermal_satCanImuInhibit() { return gSatCanImuInh.active; }
bool thermal_gpsReduce1Hz()     { return gGpsReduce.active; }
