#pragma once
/*
 * gnss.h — LG290P GNSS module driver
 * ====================================
 * Handles:
 *   - LG290P UART configuration (PQTM commands)
 *   - NMEA parsing via TinyGPSPlus
 *   - PQTMEPE proprietary accuracy message parsing
 *   - GSA/GSV satellite tracking (when SAT_LOG_ENABLE)
 *   - GGA sentence builder for VRS NTRIP casters
 *   - Periodic update of the global 'gps' struct
 *
 * Call gnss_init() once from setup(), then gnss_loop() from loop().
 */

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "types.h"

// Initialise UART1, send LG290P configuration commands.
void gnss_init(HardwareSerial& serial);

// Must be called every loop(). Drains the UART RX buffer, updates gps
// under dataMutex, and fires SD log records if SD_LOG_ENABLE is set.
void gnss_loop(HardwareSerial& serial);

// ── Tunable multipath/occlusion masks (defaults in config.h) ──────────────────
// Current live values (from NVS, or config.h defaults on a fresh module).
float gnss_getEleMask();     // elevation cutoff, degrees
float gnss_getCnrMask();     // C/N0 cutoff, dB-Hz
// Request new masks (called from the web task). Validates against the LG290P's
// documented ranges; the GNSS task pushes them to the module + NVS on its next
// loop. Returns false if either value is out of range.
bool  gnss_requestMasks(float eleDeg, float cnrDbHz);

// ── PPP (fallback below RTK, above vanilla 3D GPS) ────────────────────────────
// Mode = $PQTMCFGPPP <Mode>: 0 = off, 1 = BeiDou B2b, 2 = Galileo E6 HAS.
// Enabling PPP never costs an RTK fix — the module still prefers RTK whenever RTCM is
// flowing, and falls back to PPP (and then to plain 3D) on its own.
uint8_t gnss_getPppMode();
// Request a new PPP mode (called from the web task). The GNSS task pushes it to the module
// + NVS on its next loop. May briefly disturb the fix — see the note in gnss.cpp. Returns
// false if mode > 2.
bool    gnss_requestPppMode(uint8_t mode);

// Build a valid NMEA GGA sentence from current GPS state for VRS casters.
// Returns empty String if fix is not valid.
String gnss_buildGGA(const GnssData& g);

// NMEA checksum helper (XOR of bytes between $ and *)
void gnss_checksum(const char* sentence, char* out2);

// Rate tracking — updated every 2 s in gnss_loop()
extern float gnss_updateRate_Hz;

// Shared TinyGPS++ instance (also used by racecapture.cpp for satellite count)
extern TinyGPSPlus tinygps;
