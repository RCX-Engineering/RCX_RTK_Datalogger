#pragma once
/*
 * thermal.h — die-temperature throttling
 * =======================================
 * The ESP32-S3 has been seen to restart spontaneously above ~117°C. This
 * module steps down non-essential work well before that as the die heats up,
 * so the unit keeps running rather than resetting mid-session.
 *
 * Every measure below is independent and gated by its own Schmitt-trigger
 * hysteresis band: it engages at its threshold and only clears once the
 * temperature has fallen 5°C below that SAME threshold, so hovering right at
 * a threshold can't chatter the output on and off every reading.
 *
 * Scope, deliberately narrow: this only ever inhibits or slows down LCD/
 * backlight output and SD-card logging. It never touches GNSS, WiFi/NTRIP,
 * BLE, or the per-channel logging ENABLE flags the user sets from the web
 * dashboard — those flags are read as-is; throttling only gates whether the
 * work they'd otherwise cause actually runs right now. RTK positioning data
 * keeps flowing to BLE/SoloStorm at full rate through every threshold here,
 * all the way up to physical shutdown.
 *
 * A NAN reading (sensor fault) holds the last known state rather than
 * guessing — it neither engages nor clears a threshold — so a transient
 * temperature-read failure can't itself trigger a throttle step.
 */

#include <Arduino.h>

// Feed a fresh die-temperature reading (°C) into the throttling state
// machine. Call once per reading; the hysteresis bands are wide enough that
// the ~0.5 Hz cadence this is already read at (see the .ino) is ample.
void thermal_update(float espTempC);

// ── Threshold 1: ≥100°C engages, <95°C clears ──────────────────────────────
// Dim the LCD backlight to 20%.
bool thermal_backlightDim();

// ── Threshold 2: ≥105°C engages, <100°C clears ─────────────────────────────
// Blank the LCD and turn the backlight fully off.
bool thermal_lcdOff();

// ── Threshold 2b: ≥105°C engages, <100°C clears ────────────────────────────
// Pause the CAN sniffer's raw-frame SD dump. The sniffer's live web snapshot
// (can_bus.cpp) is untouched — only its SD write is paused.
bool thermal_canSniffInhibit();

// ── Threshold 3: ≥112°C engages, <107°C clears ─────────────────────────────
// Pause SAT, CAN, and IMU SD logging entirely (GPS is unaffected here).
bool thermal_satCanImuInhibit();

// ── Threshold 4: ≥115°C engages, <110°C clears ─────────────────────────────
// Cap GPS SD logging at 1 Hz. BLE/SoloStorm still gets the full-rate stream —
// this only slows the SD write.
bool thermal_gpsReduce1Hz();
