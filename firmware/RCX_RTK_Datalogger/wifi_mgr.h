#pragma once
/*
 * wifi_mgr.h — WiFi station list (NVS) + always-on configuration AP
 * ==================================================================
 * There are no compile-time WiFi credentials anywhere in this project. The
 * station list lives entirely in NVS (namespace "rcx_wifi") and is managed from
 * the web dashboard: add, remove, and reorder. Connection order IS priority —
 * index 0 is tried first.
 *
 * The device also runs a permanent SoftAP. That AP is the only way to reach the
 * dashboard on a freshly-flashed unit (which by definition knows no networks),
 * and it is what makes a base-only, no-internet field setup possible: join the
 * AP, add the base's SSID, done.
 *
 * Call order matters — see wifi_init().
 */

#include <Arduino.h>
#include "types.h"

// Maximum stored station entries. Bounds the NVS scan and the runtime table.
#ifndef WIFI_MAX_NETWORKS
#define WIFI_MAX_NETWORKS 12
#endif

// Load the station list from NVS and derive this unit's AP SSID from its eFuse
// MAC. Touches ONLY NVS and the eFuse — it does NOT bring up the radio, so it is
// safe to call at the very top of setup(), before the GNSS module is probed.
//
// ORDERING CONTRACT: nothing in this module may call WiFi.mode() before the
// LG290P configuration/probe in gnss_init() has completed. Bringing the radio
// up alongside that timing-sensitive UART exchange corrupts it. wifi_apBegin()
// and wifi_tryConnect() are therefore called from the WiFi task, which is only
// created after gnss_init() returns.
void wifi_init();

// Bring up the SoftAP. First call also sets the radio mode (AP+STA). Must run
// after gnss_init() — see the ordering contract above. Idempotent.
void wifi_apBegin();

// Retire the AP once a station link exists. The AP is up from boot and stays up
// alongside the station search — effectively the last entry in the connection
// chain — so the unit is always reachable while it is looking for a network. On
// the FIRST successful association it is shut down for the rest of this boot and
// is never restarted, including after a later dropout. Call once per WiFi-task
// pass; cheap and idempotent.
void wifi_apService();

// One full priority sweep. Surveys the air first and attempts the stored
// networks that are actually present, in list order, so the operator's priority
// decides which of several available networks is used. Networks the survey could
// not see (hidden SSIDs, or a failed scan) are attempted afterwards rather than
// skipped. Blocking; returns true on success, and false without doing anything
// when the list is empty (AP-only operation).
bool wifi_tryConnect();

// Drive the station link. Call once per WiFi-task pass; it owns all reconnect
// timing and is a cheap no-op returning true while the link is up.
//
// After a link drops, the network that was connected gets exclusive retries for
// WIFI_RECONNECT_WINDOW_MS — the one that just worked is the one most likely to
// work again, and this keeps a brief dropout from costing a full sweep or
// letting a lower-priority network take over during a blip. When that window
// expires the full priority search restarts from the top.
bool wifi_service();

// ── Station list accessors (safe from the web task) ──────────────────────────
int  wifi_count();
// Copies entry i's SSID into out. Passwords are never read back out of this
// module — they go to the radio and nowhere else.
bool wifi_getSsid(int i, char* out, size_t outLen);

// Mutators. All persist to NVS immediately and return false on a full list or a
// bad index. None of these touch the radio: an existing association is left
// alone so that editing the list from the dashboard cannot drop the connection
// the dashboard is being served over.
bool wifi_add(const char* ssid, const char* pass);

// Enable/disable an entry without losing it. A disabled network keeps its
// passphrase and its place in the priority order but is never attempted, which
// is what you want for a network that is temporarily wrong (a venue hotspot that
// has gone captive-portal, a hotspot you don't want to burn data on today)
// rather than one you want gone. Disabling does not drop a live link.
bool wifi_getEnabled(int i);
bool wifi_setEnabled(int i, bool enabled);
bool wifi_remove(int i);
bool wifi_move(int i, int delta);      // delta -1 = higher priority, +1 = lower

// ── Device identity ──────────────────────────────────────────────────────────
// The device name IS the configuration AP's SSID — one identity, so the name
// shown on the LCD is the name that appears in a phone's WiFi picker. It
// defaults to DEVICE_NAME_PREFIX plus the last two eFuse MAC bytes and is
// persisted in NVS once renamed.
const char* wifi_deviceName();

// Rename the device. Trims surrounding blanks and rejects an empty name or one
// longer than the 32-byte 802.11 SSID limit (the name is also the AP's SSID).
// The new name is stored and reported immediately, but a running AP keeps the
// SSID it was started with — the rename reaches the air on the next boot. That
// is deliberate: republishing would kick every client off the AP, usually
// including the browser that requested the rename.
bool wifi_setDeviceName(const char* name);

const char* wifi_apIp();               // dotted-quad, valid once wifi_apBegin() ran
