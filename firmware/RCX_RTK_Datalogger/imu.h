#pragma once
/*
 * imu.h — QMI8658 IMU driver (direct I2C, no external library)
 * =============================================================
 * Provides init and read functions for the onboard QMI8658
 * 6-axis IMU on the Waveshare ESP32-S3-LCD-1.47B.
 *
 * Configuration: ±8g accelerometer, ±512dps gyro, 1024 Hz ODR.
 *
 * ── IMU CALIBRATION ──────────────────────────────────────────────────────────
 * MEMS IMUs have a constant zero offset (bias) on every axis. The gyro bias is
 * the important one: an uncalibrated gyro reads a few deg/s while perfectly
 * still, which integrates into large heading/yaw drift. The accel bias shows
 * up as a small steady g-force error.
 *
 * Calibration here captures the bias while the unit is STATIONARY and (for the
 * accelerometer) sitting so that exactly one axis is aligned with gravity
 * (i.e. resting flat / on a level surface). It then stores the bias in NVS and
 * subtracts it from every future reading.
 *
 *   • Gyro  : bias = mean(reading) while still   → expected 0 on all axes.
 *   • Accel : bias = mean(reading) − gravity, where gravity (±1 g) is assigned
 *             to whichever axis is dominant during the capture and 0 to the
 *             other two. So it works for any flat orientation (right-side-up,
 *             on its edge, etc.) as long as ONE axis points along gravity.
 *
 * Motion rejection: if the readings move too much during the capture window the
 * calibration is rejected (IMU_CAL_FAILED) so a bad bias is never stored.
 *
 * Calibration is NON-BLOCKING: imu_startCalibration() arms it, and the samples
 * are gathered inside the normal imu_read() calls over ~2 s. Nothing blocks the
 * BLE/GNSS loop. Default (no stored calibration) = zero bias = identical to the
 * pre-calibration behaviour, so this is safe to ship without breaking anything.
 *
 * HOW TO TRIGGER:
 *   • Serial   : send "imucal" in the Serial Monitor (handled by imu_pollSerial)
 *   • Web/BLE  : call imu_requestCalibration()/imu_requestClearCalibration() —
 *                NOT imu_startCalibration()/imu_clearCalibration() directly.
 *                Those two write calState/calSum/calCount/calSumSq, which
 *                imu_read() also writes every sample from loop()/Core 1 with no
 *                lock (same reason imu_readTempC() below is same-context-only) —
 *                a web/BLE handler runs on a different task. The request
 *                functions just set a flag; imu_serviceCalRequests(), called
 *                from the same place as imu_pollSerial(), does the real work
 *                from the right context.
 *   • 1st flash: call imu_startCalibration() once in setup() guarded by
 *                !imu_isCalibrated(), but only if you can guarantee the unit is
 *                still & flat at power-on — setup() runs single-threaded before
 *                loop() starts, so this one case is exempt from the rule above.
 */

#include <Arduino.h>
#include "types.h"

enum ImuCalState : uint8_t {
    IMU_CAL_IDLE = 0,   // not calibrating
    IMU_CAL_RUNNING,    // capturing stationary samples
    IMU_CAL_DONE,       // finished, bias stored
    IMU_CAL_FAILED      // too much motion — bias NOT changed
};

// Initialise I2C and configure QMI8658.  Loads any stored calibration.
// Returns true on success.
bool imu_init();

// Read one sample into 'out'.  Applies stored bias.  Blocks ~0.35ms (I2C burst).
// If a calibration is armed, this call also feeds the capture and finalises it
// once enough samples have been gathered.
void imu_read(ImuData& out);

// Read the QMI8658 internal temperature (°C). Tracks board/enclosure heat-soak,
// NOT ambient air. MUST be called from the same context as imu_read() (shared
// Wire bus). Returns NAN on I2C failure or out-of-range (-40..150 °C) reading.
float imu_readTempC();

// ── Calibration control ───────────────────────────────────────────────────────
// Arm a calibration. Unit must be stationary and resting flat (one axis along
// gravity) for ~2 s. Result lands in imu_calibrationState(). MUST be called
// from the same context as imu_read() (loop(), Core 1) — see the file header.
// From any other task (a web or BLE handler), use imu_requestCalibration() /
// imu_requestClearCalibration() instead.
void        imu_startCalibration();
ImuCalState imu_calibrationState();      // poll for progress/result — safe from any task
bool        imu_isCalibrated();          // true if a valid bias is loaded/stored — safe from any task
void        imu_clearCalibration();      // erase stored bias (back to raw) — same-context-only, see above

// Safe to call from any task (web/BLE handlers, etc.) — just sets a flag.
// imu_serviceCalRequests() (called alongside imu_pollSerial(), same context as
// imu_read()) does the actual work.
void        imu_requestCalibration();
void        imu_requestClearCalibration();
void        imu_serviceCalRequests();

// Optional convenience: poll Serial for the "imucal" command and arm a
// calibration when seen. Call once per loop() iteration if you want the serial
// trigger; harmless to omit.
void        imu_pollSerial();
