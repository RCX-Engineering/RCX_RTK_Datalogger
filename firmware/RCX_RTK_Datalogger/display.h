#pragma once
/*
 * display.h — TFT_eSPI display driver (172×320 portrait)
 * =======================================================
 * Manages the 1.47" LCD on the Waveshare ESP32-S3-LCD-1.47B.
 * Layout sections (top→bottom):
 *   y=  0-17:  title bar (health color)
 *   y= 20-39:  WiFi SSID
 *   y= 40-59:  IP address
 *   y= 60-79:  NTRIP mountpoint + distance
 *   y= 80-99:  NTRIP band + VRS
 *   y=100-119: RTK status
 *   y=120-139: Sat count + accuracy
 *   y=140-159: [divider]
 *   y=144-183: Lat / Lon
 *   y=184-203: Speed + heading
 *   y=204-227: BLE status + Hz
 *   y=228-247: GPS Hz + CAN Hz
 *   y=248-319: G-force circle + IMU readouts
 */

#include <Arduino.h>
#include "types.h"

void display_init();
void display_update(const GnssData& g, const ImuData& m, const SystemStatus& s);

// ── LCD/backlight power (web-controlled, RAM only) ─────────────────────────
// Always defaults to ON at boot — never persisted, so the unit never boots
// into a blank screen from a state left over the last time it was on. This
// is the user's own on/off wish, independent of thermal throttling (thermal.h
// dims/blanks the same hardware on its own schedule; the two compose — either
// one wanting it off is enough to turn it off).
void display_setEnabled(bool on);
bool display_isEnabled();
