/*
 * wifi_mgr.cpp — WiFi station list (NVS) + always-on configuration AP
 * ====================================================================
 * Storage layout, NVS namespace "rcx_wifi":
 *   "n"      int    number of stored entries
 *   "s<i>"   string SSID of entry i
 *   "p<i>"   string passphrase of entry i ("" = open network)
 * Entry order is priority order; index 0 is tried first.
 *
 * ── Radio discipline (every rule here was paid for) ──────────────────────────
 *  1. Set the mode ONCE. Re-entering WiFi.mode()/WiFi.disconnect() between
 *     attempts tears down and rebuilds the LwIP netif; the AsyncWebServer listen
 *     socket is bound to that netif and is orphaned when it changes — the device
 *     then associates and gets an IP while nothing answers on port 80.
 *  2. No per-attempt WiFi.disconnect(). WiFi.begin() supersedes a pending
 *     association on its own, and the explicit disconnect is what triggers rule
 *     1's netif churn.
 *  3. WiFi.persistent(false). The driver otherwise writes its own copy of the
 *     credentials to flash on EVERY WiFi.begin(). A flash commit disables the
 *     instruction cache on both cores, which stalls the 460800-baud GNSS UART
 *     drain long enough to lose NMEA/RTCM bytes.
 *  4. WiFi.setAutoReconnect(false). The SDK's own reconnect races this module's
 *     rotation through the list, producing ESP_ERR_WIFI_STATE / "cannot set
 *     config" errors when both try to drive the association at once.
 */

#include "wifi_mgr.h"
#include "config.h"
#include "types.h"
#include <WiFi.h>
#include <Preferences.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include "debug_log.h"   // LAST include: serial-to-SD tee macro must not touch library headers above

// ── Runtime station table ────────────────────────────────────────────────────
struct WifiEntry {
    char ssid[33];       // 32 chars + NUL (802.11 maximum)
    char pass[65];       // 64 chars + NUL (WPA2 PSK maximum); "" = open network
    bool enabled;        // false = kept with its passphrase, but never attempted
};
static WifiEntry s_nets[WIFI_MAX_NETWORKS];
static int       s_count = 0;

// The device name IS the AP SSID — one identity, so the name on the screen is
// the name in the WiFi picker. Defaults to DEVICE_NAME_PREFIX + the last two
// eFuse MAC bytes so two units are never confusable out of the box; renameable
// from the dashboard and persisted in NVS.
static char s_deviceName[33] = "";
static char s_apIp[16]   = "0.0.0.0";
static bool s_apUp       = false;
// SSID of the network most recently associated. Held by name, not index — see
// indexOfSsid(). Empty until the first successful connect of this boot.
static char     s_lastGoodSsid[33] = "";
// Link-state edge tracking and reconnect scheduling for wifi_service().
static bool     s_linkWasUp        = false;
static bool     s_reconnecting     = false;
static uint32_t s_reconnectUntilMs = 0;
static uint32_t s_nextAttemptMs    = 0;

// Latches true when THIS module brings a station link up. Distinct from
// WiFi.status(): only a connection this module made counts as proof that the
// stored list works, and only that may retire the configuration AP.
static bool s_ownedLink = false;

// Latches true the first time a station link is established. The configuration
// AP is retired permanently at that point and never comes back for the rest of
// this boot — see wifi_apService().
static bool s_apRetired = false;
static bool s_modeSet    = false;

static const char* NVS_NS = "rcx_wifi";

// ── NVS ──────────────────────────────────────────────────────────────────────
static void loadNetworks() {
    s_count = 0;
    Preferences p;
    if (!p.begin(NVS_NS, true)) return;          // namespace absent on a fresh unit
    const int n = p.getInt("n", 0);
    for (int i = 0; i < n && s_count < WIFI_MAX_NETWORKS; i++) {
        char key[8];
        snprintf(key, sizeof(key), "s%d", i);
        String ssid = p.getString(key, "");
        if (ssid.length() == 0) continue;        // blank slot — skip, never count
        snprintf(key, sizeof(key), "p%d", i);
        String pass = p.getString(key, "");
        strlcpy(s_nets[s_count].ssid, ssid.c_str(), sizeof(s_nets[0].ssid));
        strlcpy(s_nets[s_count].pass, pass.c_str(), sizeof(s_nets[0].pass));
        // Absent flag means enabled: entries stored before this field existed,
        // and every newly added network, are active by default.
        snprintf(key, sizeof(key), "e%d", i);
        s_nets[s_count].enabled = p.getBool(key, true);
        s_count++;
    }
    p.end();
}

// Rewrite the whole list. Called after any add/remove/reorder so the stored
// order always matches the runtime order — priority is the array index, and a
// partial update would silently scramble it.
//
// COST NOTE: this commits to flash, which disables the instruction cache on both
// cores for the duration and briefly stalls the GNSS UART drain. That is
// acceptable here because every caller is a deliberate operator action from the
// dashboard, not something that happens on a run.
static void saveNetworks() {
    Preferences p;
    if (!p.begin(NVS_NS, false)) return;
    p.clear();                                    // drop stale higher-index slots
    p.putInt("n", s_count);
    for (int i = 0; i < s_count; i++) {
        char key[8];
        snprintf(key, sizeof(key), "s%d", i); p.putString(key, s_nets[i].ssid);
        snprintf(key, sizeof(key), "p%d", i); p.putString(key, s_nets[i].pass);
        snprintf(key, sizeof(key), "e%d", i); p.putBool(key, s_nets[i].enabled);
    }
    p.end();
}

// ── Init ─────────────────────────────────────────────────────────────────────
void wifi_init() {
    loadNetworks();

    // Factory name from the eFuse MAC via esp_read_mac() — deliberately NOT
    // WiFi.macAddress(), which requires the WiFi driver to be started and would
    // drag WiFi.mode() ahead of the GNSS probe (see the ordering contract in the
    // header). esp_read_mac() reads the eFuse directly with the radio down.
    uint8_t mac[6] = {0};
    char factory[sizeof(s_deviceName)];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(factory, sizeof(factory), "%s%02X%02X", DEVICE_NAME_PREFIX, mac[4], mac[5]);
    } else {
        snprintf(factory, sizeof(factory), "%s0000", DEVICE_NAME_PREFIX);
    }

    Preferences p;
    if (p.begin(NVS_NS, true)) {
        String stored = p.getString("name", "");
        p.end();
        strlcpy(s_deviceName, stored.length() ? stored.c_str() : factory, sizeof(s_deviceName));
    } else {
        strlcpy(s_deviceName, factory, sizeof(s_deviceName));
    }

    Serial.printf("📶 Device \"%s\" — %d stored network(s)\n", s_deviceName, s_count);
    if (s_count == 0) {
        Serial.println("📶 WiFi: no networks stored — join the AP and add one from the dashboard");
    }
}

const char* wifi_deviceName() { return s_deviceName; }

bool wifi_setDeviceName(const char* name) {
    if (!name) return false;
    // Trim surrounding blanks: a name that is all spaces, or one that picked up a
    // trailing space from a form field, would produce an SSID nobody can read or
    // reliably type back in.
    while (*name == ' ') name++;
    size_t len = strlen(name);
    while (len && name[len - 1] == ' ') len--;
    // 32 bytes is the 802.11 SSID maximum, and the name IS the SSID.
    if (len == 0 || len > 32) return false;

    char clean[sizeof(s_deviceName)];
    memcpy(clean, name, len);
    clean[len] = '\0';
    if (strcmp(clean, s_deviceName) == 0) return true;   // nothing to do

    strlcpy(s_deviceName, clean, sizeof(s_deviceName));
    Preferences p;
    if (p.begin(NVS_NS, false)) { p.putString("name", s_deviceName); p.end(); }
    Serial.printf("📶 Device renamed to \"%s\" — AP SSID follows on restart\n", s_deviceName);

    // The running AP is deliberately left alone. Its SSID is read at bring-up, so
    // the new name takes effect on the next boot. Republishing here would drop
    // every client on the AP — including, almost always, the browser that just
    // submitted the rename — which is a hostile response to a rename and leaves
    // the operator unsure whether it even worked.
    return true;
}

// Set the radio mode exactly once (rule 1). AP+STA so the configuration AP stays
// reachable while the station side is associated or rotating through the list.
static void ensureMode() {
    if (s_modeSet) return;
    WiFi.persistent(false);          // rule 3 — no driver-side flash writes
    WiFi.mode(WIFI_AP_STA);
    WiFi.setAutoReconnect(false);    // rule 4 — this module owns the rotation
    WiFi.setSleep(false);

    // Discard any station credentials the SDK persisted under previously-flashed
    // firmware. Those live in the driver's own NVS, independent of this module's
    // list, and the driver will happily associate with one of them on its own —
    // which both contradicts the stored priority order and, because it looks
    // like a successful link, would retire the configuration AP before anyone
    // could use it. NVS is the single source of truth for credentials, so the
    // driver's private copy is cleared.
    //
    // This does NOT violate rule 2: that rule forbids a disconnect between
    // connection ATTEMPTS. This runs exactly once, at init, before any attempt
    // and before the web server binds its listen socket.
    WiFi.disconnect(false, true);

    s_modeSet = true;
}

// ── Configuration AP lifecycle ───────────────────────────────────────────────
// The AP exists for exactly one situation: the unit cannot reach any network, so
// there is no other way to talk to it. It therefore runs from boot until the
// FIRST successful station association, and is then retired for the rest of this
// boot — permanently, not conditionally.
//
// The AP is effectively the last entry in the connection chain: while the WiFi
// task keeps cycling the stored station list looking for something in range, the
// AP stays up alongside it, so an operator always has a way in during that
// search. The moment anything associates, the search is over and so is the AP.
//
// ONE-WAY BY DESIGN: it is NOT restarted when a station link later drops.
// Intermittent WiFi is common in the field, and cycling the AP on every dropout
// would repeatedly bring an interface up and down on the same radio that carries
// the 20 Hz BLE stream, for no benefit — a link that flaps is a link that is
// coming back, and the stored credentials that worked once will work again.
// Recovering from a genuinely permanent loss is a reboot, which is the right
// cost for how rarely it happens.
//
// The radio MODE is never changed after ensureMode() — only softAP() and
// softAPdisconnect() are used. Re-entering WiFi.mode() rebuilds the LwIP netif
// and orphans the web server's listen socket (rule 1 at the top of this file);
// bringing just the AP interface up and down does not.
static void apStart() {
    if (s_apUp || s_apRetired) return;
    ensureMode();

    // The SSID is taken from the device name at this moment and is fixed for the
    // life of the AP — a rename lands on the next boot, by design (see
    // wifi_setDeviceName).
    //
    // Pin the AP subnet before bringing the interface up. Without this the
    // address depends on whatever the driver last had configured, and the LCD
    // ends up advertising a URL that does not match where the dashboard actually
    // answers — which looks exactly like the AP being broken.
    if (!WiFi.softAPConfig(IPAddress(192, 168, 5, 1),
                           IPAddress(192, 168, 5, 1),
                           IPAddress(255, 255, 255, 0))) {
        Serial.println("⚠️  WiFi: SoftAP IP configuration failed");
    }

    // Open AP by default: a unit that has never been configured must be joinable
    // without a credential the operator has no way to look up. Set
    // WIFI_AP_PASSWORD in config.h to require a passphrase. A passphrase shorter
    // than WPA2's 8-character minimum is treated as "no passphrase" rather than
    // being handed to the driver, which would reject it and leave no AP at all.
    const char* apPass = (strlen(WIFI_AP_PASSWORD) >= 8) ? WIFI_AP_PASSWORD : nullptr;
    if (!WiFi.softAP(s_deviceName, apPass, WIFI_AP_CHANNEL, false, WIFI_AP_MAX_CLIENTS)) {
        Serial.println("⚠️  WiFi: SoftAP failed to start");
        return;
    }

    // Force the advertised security to match intent. A unit that previously ran
    // different firmware carries that firmware's AP record in the driver's own
    // NVS, and it is that record — not this call — that decides what the AP
    // beacons if anything about this configuration is rejected. The symptom is a
    // phone demanding a password for an AP the code believes is open, so the
    // authentication mode is set explicitly and then read back and reported.
    wifi_config_t apCfg;
    if (esp_wifi_get_config(WIFI_IF_AP, &apCfg) == ESP_OK) {
        const wifi_auth_mode_t want = apPass ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        if (apCfg.ap.authmode != want) {
            apCfg.ap.authmode = want;
            if (!apPass) apCfg.ap.password[0] = '\0';
            esp_wifi_set_config(WIFI_IF_AP, &apCfg);
            esp_wifi_get_config(WIFI_IF_AP, &apCfg);
        }
        Serial.printf("📶 AP beacon: ssid=\"%s\" authmode=%d (0 = open)\n",
                      (const char*)apCfg.ap.ssid, (int)apCfg.ap.authmode);
    }

    s_apUp = true;
    strlcpy(s_apIp, WiFi.softAPIP().toString().c_str(), sizeof(s_apIp));
    Serial.printf("📶 WiFi AP \"%s\" up at http://%s%s\n",
                  s_deviceName, s_apIp, apPass ? " (protected)" : " (open)");
}

static void apStop() {
    if (!s_apUp) return;
    WiFi.softAPdisconnect(false);   // drop the AP interface, leave the mode alone
    s_apUp = false;
    Serial.println("📶 WiFi AP retired — station link established, AP not needed again this boot");
}

void wifi_apBegin() {
    // Start in AP-on. At the point this first runs there is by definition no
    // station link yet, and on a unit with no stored networks there never will
    // be one until somebody uses this AP to add one.
    apStart();
}

void wifi_apService() {
    if (s_apRetired) return;      // one-way: never comes back
    // Gated on THIS module having made the connection, not on WiFi.status().
    // A link the driver raised on its own does not prove the stored list is
    // usable, and retiring the AP on it would strand the operator on a unit
    // whose configuration they can no longer reach.
    if (!s_ownedLink) return;     // still searching — AP stays up
    s_apRetired = true;
    apStop();
}

// ── Station connect ──────────────────────────────────────────────────────────
// Priority is list order: index 0 is the operator's first choice. When several
// stored networks are in range at once the highest-priority one wins — signal
// strength is deliberately NOT a tiebreaker, because the ordering is a statement
// about which network the operator wants (the one carrying the base's caster,
// the one that isn't metered), not about which one is loudest.
//
// Priority applies at connect time only. A live link is never abandoned because
// a higher-priority network has come into range: roaming mid-session would drop
// the correction stream and re-fix the receiver at an arbitrary moment, which on
// a timed run is precisely the thing worth avoiding.

// One association attempt against stored entry i. Shared by the priority sweep
// and the fast reconnect so both behave identically on the radio.
static bool attemptEntry(int i, uint32_t timeoutMs) {
    Serial.printf("📶 Trying WiFi: %s\n", s_nets[i].ssid);
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        status.wifiAttempting = true;
        strlcpy(status.wifiSSID, s_nets[i].ssid, sizeof(status.wifiSSID));
        xSemaphoreGive(dataMutex);
    }

    // An empty passphrase must be passed as nullptr, not "" — the latter is
    // treated as a zero-length PSK and fails against an open access point.
    const char* pass = (s_nets[i].pass[0] != '\0') ? s_nets[i].pass : nullptr;
    WiFi.begin(s_nets[i].ssid, pass);

    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("\n   Failed (status %d)\n", WiFi.status());
        // No disconnect between attempts — see rules 1 and 2 at the top of this
        // file. The next WiFi.begin() supersedes this pending association.
        return false;
    }

    delay(2000);   // let DHCP and the TCP stack settle before servers bind
    Serial.printf("\n✅ WiFi: %s (%s)\n",
                  s_nets[i].ssid, WiFi.localIP().toString().c_str());
    strlcpy(s_lastGoodSsid, s_nets[i].ssid, sizeof(s_lastGoodSsid));
    // Latch the link-up edge HERE, not only at the top of wifi_service(): the
    // WiFi task stops calling wifi_service() once the link is up, so a flag set
    // only on that path would never latch, and the loyalty window below would
    // never open on the subsequent drop.
    s_linkWasUp = true;
    s_ownedLink = true;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        status.wifiConnected  = true;
        status.wifiAttempting = false;
        strlcpy(status.wifiSSID, s_nets[i].ssid, sizeof(status.wifiSSID));
        strlcpy(status.ipAddress, WiFi.localIP().toString().c_str(),
                sizeof(status.ipAddress));
        xSemaphoreGive(dataMutex);
    }
    return true;
}

// Look up a stored entry by SSID. The last-good network is remembered by NAME
// rather than by index on purpose: add, remove and reorder all shift indices, so
// a cached index would quietly come to mean a DIFFERENT network the moment the
// operator edits the list from the dashboard.
static int indexOfSsid(const char* ssid) {
    if (!ssid || ssid[0] == '\0') return -1;
    for (int i = 0; i < s_count; i++) {
        if (strcmp(s_nets[i].ssid, ssid) == 0) return i;
    }
    return -1;
}

// Report the AP-only steady state: no stored networks means nothing to attempt.
// This is normal operation on an unconfigured unit, not a fault.
static void markIdle() {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        status.wifiAttempting = false;
        status.wifiConnected  = false;
        status.wifiSSID[0]    = '\0';
        xSemaphoreGive(dataMutex);
    }
}

bool wifi_tryConnect() {
    ensureMode();
    if (s_count == 0) { markIdle(); return false; }

    // Survey first, then attempt only what is actually on the air, in priority
    // order. Without this a sweep burns WIFI_CONNECT_TIMEOUT_MS on every absent
    // network before it reaches a present one; across a full list that is
    // minutes, which would make the reconnect timing in wifi_service() and the
    // operator's priority ordering both meaningless in practice.
    //
    // The scan hops channels, so a client sitting on the configuration AP will
    // see it stall for the scan's duration. That is accepted: a unit still
    // looking for a network has nothing more important to do, and the
    // alternative is never finding one.
    const int found = WiFi.scanNetworks(false, true);
    bool present[WIFI_MAX_NETWORKS];
    memset(present, 0, sizeof(present));
    if (found > 0) {
        for (int r = 0; r < found; r++) {
            const int i = indexOfSsid(WiFi.SSID(r).c_str());
            if (i >= 0) present[i] = true;
        }
    }
    WiFi.scanDelete();   // scan results are heap-allocated until this is called

    // Pass 0 takes the networks the scan actually saw, in priority order.
    // Pass 1 takes everything else, which covers the two cases where absence
    // from the scan proves nothing: a hidden SSID broadcasts no name, and a scan
    // that failed outright (found <= 0) saw nothing at all. Skipping those would
    // turn a recoverable situation into a unit that never connects.
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < s_count; i++) {
            if (!s_nets[i].enabled) continue;   // kept, but excluded from the search
            const bool seen = (found > 0) && present[i];
            if ((pass == 0) != seen) continue;
            if (attemptEntry(i, WIFI_CONNECT_TIMEOUT_MS)) return true;
        }
    }

    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        status.wifiAttempting = false;
        xSemaphoreGive(dataMutex);
    }
    Serial.println("⚠️  No stored WiFi networks in range");
    return false;
}

bool wifi_service() {
    if (WiFi.status() == WL_CONNECTED) {
        s_linkWasUp = true;
        return true;
    }
    ensureMode();
    if (s_count == 0) { markIdle(); return false; }

    // Down-edge: open the loyalty window. A network that just worked is the one
    // most likely to work again, so it gets exclusive retries for a while before
    // the search widens — that keeps a brief dropout from costing a full sweep,
    // and keeps a lower-priority network from stealing the link during a blip.
    if (s_linkWasUp) {
        s_linkWasUp        = false;
        s_reconnectUntilMs = millis() + WIFI_RECONNECT_WINDOW_MS;
        const int lastIdx  = indexOfSsid(s_lastGoodSsid);
        s_reconnecting     = (lastIdx >= 0) && s_nets[lastIdx].enabled;
        s_nextAttemptMs    = millis();   // act on the drop now, don't wait out a gate
        if (s_reconnecting) {
            Serial.printf("📶 WiFi: link lost — retrying %s for %lus\n",
                          s_lastGoodSsid, (unsigned long)(WIFI_RECONNECT_WINDOW_MS / 1000));
        }
    }

    // Rate gate. Comparisons are signed so they stay correct across the millis()
    // rollover at ~49 days, which a naive `millis() < deadline` does not.
    if ((int32_t)(millis() - s_nextAttemptMs) < 0) return false;

    if (s_reconnecting) {
        const int i = indexOfSsid(s_lastGoodSsid);
        if (i < 0 || !s_nets[i].enabled) {
            // The operator removed or disabled that entry mid-window. Abandon the
            // loyalty window immediately rather than spending it on a network
            // they have just said not to use.
            s_reconnecting = false;
        } else if ((int32_t)(millis() - s_reconnectUntilMs) >= 0) {
            s_reconnecting = false;
            Serial.println("📶 WiFi: reconnect window expired — restarting the priority search");
        } else {
            // Short per-attempt timeout so several attempts fit inside the
            // window rather than one long one consuming the whole budget.
            if (attemptEntry(i, WIFI_RECONNECT_ATTEMPT_MS)) return true;
            s_nextAttemptMs = millis();   // keep retrying back-to-back until the window closes
            return false;
        }
    }

    if (wifi_tryConnect()) return true;
    s_nextAttemptMs = millis() + WIFI_RETRY_INTERVAL_MS;
    return false;
}


// ── List accessors / mutators ────────────────────────────────────────────────
int wifi_count() { return s_count; }

bool wifi_getSsid(int i, char* out, size_t outLen) {
    if (i < 0 || i >= s_count || !out || outLen == 0) return false;
    strlcpy(out, s_nets[i].ssid, outLen);
    return true;
}

bool wifi_getEnabled(int i) {
    return (i >= 0 && i < s_count) ? s_nets[i].enabled : false;
}

bool wifi_setEnabled(int i, bool enabled) {
    if (i < 0 || i >= s_count) return false;
    if (s_nets[i].enabled == enabled) return true;
    s_nets[i].enabled = enabled;
    saveNetworks();
    Serial.printf("📶 WiFi: %s %s\n", enabled ? "enabled" : "disabled", s_nets[i].ssid);
    // A live link is deliberately left alone. Disabling a network means "stop
    // choosing this one", not "drop me right now" — cutting the connection the
    // operator is using to make the change would be a surprising way to obey it.
    return true;
}

bool wifi_add(const char* ssid, const char* pass) {
    if (!ssid || ssid[0] == '\0') return false;
    if (strlen(ssid) >= sizeof(s_nets[0].ssid)) return false;
    if (pass && strlen(pass) >= sizeof(s_nets[0].pass)) return false;

    // Re-adding a known SSID updates its passphrase in place rather than creating
    // a duplicate that would be tried twice with different credentials.
    for (int i = 0; i < s_count; i++) {
        if (strcmp(s_nets[i].ssid, ssid) == 0) {
            strlcpy(s_nets[i].pass, pass ? pass : "", sizeof(s_nets[0].pass));
            saveNetworks();
            Serial.printf("📶 WiFi: updated credentials for %s\n", ssid);
            return true;
        }
    }
    if (s_count >= WIFI_MAX_NETWORKS) return false;

    strlcpy(s_nets[s_count].ssid, ssid,             sizeof(s_nets[0].ssid));
    strlcpy(s_nets[s_count].pass, pass ? pass : "", sizeof(s_nets[0].pass));
    s_nets[s_count].enabled = true;
    s_count++;
    saveNetworks();
    Serial.printf("📶 WiFi: added %s (%d stored)\n", ssid, s_count);
    return true;
}

bool wifi_remove(int i) {
    if (i < 0 || i >= s_count) return false;
    Serial.printf("📶 WiFi: removed %s\n", s_nets[i].ssid);
    for (int j = i; j < s_count - 1; j++) s_nets[j] = s_nets[j + 1];
    s_count--;
    saveNetworks();
    return true;
}

bool wifi_move(int i, int delta) {
    const int j = i + delta;
    if (i < 0 || i >= s_count || j < 0 || j >= s_count) return false;
    WifiEntry tmp = s_nets[i];
    s_nets[i] = s_nets[j];
    s_nets[j] = tmp;
    saveNetworks();
    return true;
}

const char* wifi_apIp()   { return s_apUp ? s_apIp : "0.0.0.0"; }
