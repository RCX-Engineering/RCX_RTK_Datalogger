// debug_log — see debug_log.h for the full design rationale.
#include "config.h"

#if defined(DEBUG_SERIAL_TO_SD) && DEBUG_SERIAL_TO_SD

// The `Serial` macro from debug_log.h would rewrite our own Serial.* calls in
// this file too — we need the REAL Serial here (the tee mirrors to it). Include
// the header for the class decl, then undo the macro for this translation unit.
//
// A bare #undef is NOT enough on its own, and this was a latent bug until now:
// on USB-CDC-on-boot builds (this board's config), the ESP32 core itself binds
// `Serial` via ITS OWN macro (Serial -> USBSerial) — it's not a plain global in
// that mode. #undef only removes debug_log.h's override; it can't restore
// whatever `Serial` meant before that, because #undef has no memory of a prior
// definition, it just deletes the current one. Left as a bare #undef, every
// Serial.* call below (including the real hardware write in DebugTee::write())
// would fail to compile with "not declared in this scope" the moment this file
// is actually built with DEBUG_SERIAL_TO_SD on — so re-bind it explicitly here,
// using the same ARDUINO_USB_CDC_ON_BOOT check debug_log.h already uses for
// setTxTimeoutMs() above.
#include "debug_log.h"
#undef Serial
#if ARDUINO_USB_CDC_ON_BOOT
#define Serial USBSerial
#endif

#include "sd_log.h"      // sdMutex ownership lives in sd_log; we borrow the accessor
#include <SD_MMC.h>
#include <Preferences.h> // NVS-backed enable flag, same pattern as gnss.cpp/sd_log.cpp

DebugTee Debug;

// ── Lock-free-ish byte ring ───────────────────────────────────────────────────
// Single logical consumer (sdLogTask). Producers are many tasks calling write();
// they only advance `head` and store a byte. A torn head under contention can at
// worst duplicate/drop a byte in the FILE — acceptable for a debug log, and the
// real Serial mirror is never affected. Sized for ~2 s of heavy logging at 115200.
static constexpr size_t RING_SZ = 8192;          // power of two → cheap masking
static uint8_t   s_ring[RING_SZ];
static volatile size_t s_head = 0;               // next write index (producers)
static size_t          s_tail = 0;               // next read index (consumer only)
static volatile uint32_t s_dropped = 0;          // bytes lost to a full ring

static File     s_file;
static bool     s_open   = false;
static bool     s_armed  = false;
static uint32_t s_lastFlush = 0;
static uint32_t s_reportedDrops = 0;
// Name of the currently-open debug file, bare (no leading '/'), for the web
// download link. Empty until the file is opened (which waits on a session
// stamp). Written only by drainToFile on sdLogTask; read by the web task —
// a torn read would at worst render one stale link, so no lock is warranted.
static char     s_name[40] = "";

static inline void ringPut(uint8_t b) {
    size_t h = s_head;
    size_t next = (h + 1) & (RING_SZ - 1);
    // Full if advancing head would collide with tail. Drop the byte (don't block).
    if (next == (s_tail & (RING_SZ - 1))) { s_dropped++; return; }
    s_ring[h & (RING_SZ - 1)] = b;
    s_head = next;
}

void DebugTee::begin() { /* nothing yet — file opens lazily on first drain */ }

size_t DebugTee::write(uint8_t b) {
    size_t n = Serial.write(b);                  // live stream FIRST, always pristine
    if (s_armed) ringPut(b);
    return n;
}

size_t DebugTee::write(const uint8_t* buf, size_t len) {
    size_t n = Serial.write(buf, len);
    if (s_armed) for (size_t i = 0; i < len; i++) ringPut(buf[i]);
    return n;
}

namespace DebugLog {

// NVS namespace is private to this file — sd_log.cpp owns "rcx_log" for the
// GPS/IMU/CAN/SAT channel flags; this is a separate concern (serial mirror,
// not a data-log channel) so it gets its own small namespace.
static const char* NVS_NS = "rcx_dbg";

void begin() {
#if defined(DEBUG_SERIAL_TO_SD_FORCE_ON) && DEBUG_SERIAL_TO_SD_FORCE_ON
    // config.h override wins outright — don't even read NVS, so there is no
    // stored state that could ever disagree with "armed".
    s_armed = true;
#else
    Preferences p;
    p.begin(NVS_NS, true);
    // Default true: matches this feature's original behavior (compiling
    // DEBUG_SERIAL_TO_SD in meant always-armed). First boot after this change
    // — before the web page has ever written a preference — behaves exactly
    // as it did before; the web toggle is what lets that change from here on.
    s_armed = p.getBool("en", true);
    p.end();
#endif
}

bool isEnabled() { return s_armed; }

bool isForced() {
#if defined(DEBUG_SERIAL_TO_SD_FORCE_ON) && DEBUG_SERIAL_TO_SD_FORCE_ON
    return true;
#else
    return false;
#endif
}

// Web-facing setter. Deliberately a no-op when forced: config.h is the one
// that made this unconditional, so a web click silently "succeeding" while
// changing nothing would be exactly the kind of ambiguous state this codebase
// keeps trying to design out. The caller (webserver.cpp) checks isForced()
// after calling this and reports the real state back to the page.
void setEnabled(bool on) {
    if (isForced()) {
        Serial.println("📝 SD DEBUG: web toggle ignored — DEBUG_SERIAL_TO_SD_FORCE_ON "
                       "in config.h forces this on");
        return;
    }
    s_armed = on;
    Preferences p;
    p.begin(NVS_NS, false);
    p.putBool("en", on);
    p.end();
    Serial.printf("📝 SD DEBUG: serial-to-SD mirror %s (web)\n", on ? "ENABLED" : "disabled");
}

void closeFile() {
    if (s_open && sdlog_takeMutex(200)) {
        s_file.flush(); s_file.close(); s_open = false;
        s_name[0] = '\0';
        sdlog_giveMutex();
    }
}

// Bare name of the open debug file ("" when none). The web layer offers this
// for download; the file stays OPEN while served, which is safe — the download
// path never takes sdMutex (see the /log route) and reads only up to the size
// captured when it opened its own handle. Content lags live serial by up to
// the ~1 Hz flush interval below.
const char* currentFileName() { return s_name; }

// Called by sdLogTask each pass. Opens debug_<stamp>.txt lazily (once a session
// stamp exists), drains the ring under sdMutex, and flushes at most ~1 Hz so the
// debug file never becomes its own source of flush-stall jitter.
void drainToFile() {
    if (!s_armed) return;

    // Lazy open: reuse the logger's session stamp so the debug file sorts next to
    // its run's data files. Before a GPS fix, sdlog_sessionStamp() returns "".
    if (!s_open) {
        const char* stamp = sdlog_sessionStamp();
        if (!stamp || !stamp[0]) return;         // wait for a stamp; ring keeps filling
        char name[40];
        snprintf(name, sizeof(name), "/debug_%s.txt", stamp);
        if (!sdlog_takeMutex(200)) return;
        s_file = SD_MMC.open(name, FILE_WRITE);
        if (s_file) { s_open = true; }
        sdlog_giveMutex();
        if (!s_open) return;
        strncpy(s_name, name + 1, sizeof(s_name) - 1);   // store bare, no leading '/'
        s_name[sizeof(s_name) - 1] = '\0';
        Serial.printf("📝 SD DEBUG: serial mirror → %s\n", name);
    }

    // Snapshot head once; drain [tail, head) in a bounded batch so a burst can't
    // monopolise sdLogTask (same discipline as the CAN-raw drain).
    size_t h = s_head;
    if (h == (s_tail & (RING_SZ - 1)) && s_dropped == s_reportedDrops) return;

    if (!sdlog_takeMutex(50)) return;            // brief; skip this pass if busy
    int budget = 2048;
    while ((s_tail & (RING_SZ - 1)) != h && budget-- > 0) {
        s_file.write(s_ring[s_tail & (RING_SZ - 1)]);
        s_tail++;
    }
    // Note any bytes lost to ring overflow, so gaps in the file are explained.
    if (s_dropped != s_reportedDrops) {
        char note[64];
        int m = snprintf(note, sizeof(note),
                         "\n[debug_log: %lu bytes dropped — ring overflow]\n",
                         (unsigned long)(s_dropped - s_reportedDrops));
        s_file.write((const uint8_t*)note, m);
        s_reportedDrops = s_dropped;
    }
    if (millis() - s_lastFlush >= 1000) { s_file.flush(); s_lastFlush = millis(); }
    sdlog_giveMutex();
}

} // namespace DebugLog

#endif // DEBUG_SERIAL_TO_SD
