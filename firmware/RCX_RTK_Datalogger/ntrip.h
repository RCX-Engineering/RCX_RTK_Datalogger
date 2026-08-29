#pragma once
/*
 * ntrip.h — NTRIP client (RTK correction delivery)
 * =================================================
 * Fetches RTCM3 corrections from the nearest CORS station and pipes
 * them to the LG290P UART to enable RTK-level GPS accuracy.
 *
 * Two-phase station selection:
 *   Phase 1: connects immediately to the first station within
 *            NTRIP_QUICK_THRESHOLD_DEG (background scan catches closer ones).
 *   Phase 2: rescans every NTRIP_RESCAN_INTERVAL_MS; switches if a station
 *            >NTRIP_IMPROVE_THRESHOLD improvement is found.
 *
 * Call ntrip_init() once, then ntrip_loop() from wifiNtripTask.
 */

#include <Arduino.h>
#include <WiFiClient.h>
#include "types.h"

enum NtripConnResult { NTRIP_OK, NTRIP_FAIL, NTRIP_STALE };

// One-time setup — resets state
void ntrip_init();

// Persist a new NTRIP caster to NVS (namespace "rcx_ntrip") and rebuild the runtime
// caster table immediately, so it is counted and scanned without a reboot. Returns
// false if the host is empty or the table is already full (NTRIP_MAX_CASTERS).
// host/port/user/pass/prefMpt are copied; prefMpt "" = auto-select nearest mountpoint.
bool ntrip_saveCaster(const char* host, const char* port,
                      const char* user, const char* pass, const char* prefMpt);

// ── Caster list management ───────────────────────────────────────────────────
// Backs the dashboard's caster page. The list is the compile-time defaults from
// config.cpp followed by any casters added at runtime, and that is the order the
// indices below refer to.
//
// Defaults can be disabled but not removed — they are compiled in and would
// return on the next boot. Added casters can be both. Enable state and added
// casters persist in NVS (namespace "rcx_ntrip").
//
// These are called from the web server's task. They write NVS and schedule the
// runtime table rebuild; the rebuild itself happens inside ntrip_loop(), which
// restarts caster selection when it does.
struct NtripCasterInfo {
    const char* host;
    const char* port;
    const char* mount;      // preferred mountpoint, "" = auto-select nearest
    bool        enabled;
    bool        isDefault;  // compile-time row: disableable, not removable
    bool        active;     // currently feeding corrections
};

int  ntrip_casterCount();
bool ntrip_casterInfo(int i, NtripCasterInfo* out);
bool ntrip_setCasterEnabled(int i, bool enabled);

// Remove an added caster. Returns false for a compile-time default.
bool ntrip_removeCaster(int i);

// Service the NTRIP state machine from wifiNtripTask.
// Pass the GPS serial so RTCM bytes can be piped directly.
// Must be called frequently (every ~10 ms) to maintain data flow.
void ntrip_loop(HardwareSerial& gpsSerial, const GnssData& g);

// Returns true if the TCP connection to the caster is active
bool ntrip_connected();

// Returns current active mountpoint name (or empty string)
const char* ntrip_mountpoint();

// Hard-reset the NTRIP client (clears active mountpoint, closes TCP,
// resets backoff). Forces a fresh station scan on the next ntrip_loop().
// Use only when the mountpoint is known-bad or position changed drastically.
void ntripClient_reset();

// Operator reset (web dashboard "Reset caster timeouts" button): clears the
// per-caster source-table rate limits, cooldowns and backoff, closes the
// session and forces immediate reselection. Thread-safe — raises a flag
// consumed inside ntrip_loop().
void ntrip_requestReset();

// Soft handler for a WiFi drop. Closes the TCP connection and marks NTRIP
// disconnected, but RETAINS the active mountpoint so that when WiFi returns
// the client reconnects with a single GET (no source-table fetch — avoids
// hammering the caster). Triggers one prompt reconnect attempt on restore.
void ntrip_onWifiLost();

// Running total of RTCM bytes forwarded to the LG290P. Zero = no corrections
// flowing (caster not sending, or header drain bug). Non-zero = bytes reaching GPS.
extern volatile uint32_t rtcmBytesTotal;
