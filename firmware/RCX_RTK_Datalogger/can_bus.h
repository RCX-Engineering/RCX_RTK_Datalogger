#pragma once
/*
 * can_bus.h — ESP32-S3 TWAI (built-in CAN) driver + 20Hz polling task
 * =====================================================================
 * Uses the ESP32-S3's hardware TWAI peripheral in TWAI_MODE_LISTEN_ONLY.
 * Hardware: SN65HVD230 transceiver on GPIO8 (RX) / GPIO9 (TX).
 * No external SPI module or additional library required —
 * driver/twai.h is part of the ESP-IDF shipped with arduino-esp32.
 *
 * Decodes Porsche 987.2 PT-CAN *or* Porsche 718 DRIVE-CAN frames into the
 * global CanData struct. The active vehicle is auto-detected by passive CAN-ID
 * fingerprinting (see can_bus.cpp) and cached in NVS; it can also be pinned at
 * compile time via VEHICLE_FORCE in config.h.
 */

#include <Arduino.h>
#include "config.h"
#include "types.h"

// Called once from setup(). Installs the TWAI driver and starts canBusTask.
// Returns true on success. On failure status.canBusOk stays false and
// the firmware continues without CAN data (all channels emit null/NaN).
bool can_init();

// FreeRTOS task — internal, created by can_init(), runs on Core 0.
void canBusTask(void* pvParam);

// Returns the currently-active vehicle profile name ("987.2", "718",
// "unknown", or "disabled"). Safe to call from any task — returns a pointer to
// a static string literal. Useful for the display/status row.
const char* can_getVehicleName();

// ── CAN diagnostic / sniffer mode ─────────────────────────────────────────────
// When enabled (via the web UI), canBusTask captures the last payload of EVERY
// standard 11-bit ID into a snapshot table for a live web view, and — if CAN
// logging is on — streams every frame to a separate canraw_<stamp>.csv. When
// OFF (default), it adds zero overhead beyond one flag check and only the
// configured channels are decoded/logged, exactly as in normal operation.
struct CanSniffEntry {
    uint16_t id;        // 11-bit standard arbitration ID
    uint8_t  dlc;       // data length (0..8)
    uint8_t  data[8];   // last payload seen
    uint32_t lastMs;    // millis() of the last frame for this ID
    uint32_t count;     // total frames seen for this ID since sniffer enabled
};
void can_setSniffer(bool on);   // toggle diagnostic/sniffer mode (RAM only, default off)
bool can_getSniffer();          // current sniffer state
// Copies the snapshot (sorted by ID) into out[]; returns the number of IDs.
int  can_getSniffSnapshot(CanSniffEntry* out, int cap);
uint32_t can_getSniffOverflow();  // distinct IDs seen beyond the table cap (rare)
