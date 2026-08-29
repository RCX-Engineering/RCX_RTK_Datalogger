#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// debug_log — mirror everything printed to Serial into a debug_*.txt file on SD.
//
// WHY: the field failures we chase (NTRIP selection, reboots, thermal) only show
// in the serial monitor, but you can't run a serial monitor from a phone in the
// car. This tees Serial output to the SD card so you drive, then read the log
// afterward. OFF by default (DEBUG_SERIAL_TO_SD in config.h) — zero cost when off.
//
// DESIGN CONSTRAINTS (all forced by hard-won lessons in this codebase):
//   • NEVER block the control loop. Serial already runs setTxTimeoutMs(0) because
//     a full CDC buffer blocking Serial.print() collapsed BLE to 3-4 Hz. The tee
//     must be equally non-blocking: print sites only copy bytes into a RAM ring
//     and return. The SD write happens later, on sdLogTask.
//   • Respect sdMutex. The actual file write is done by sdLogTask under sdMutex,
//     never inline from whatever task called Serial.print (could be any core).
//   • Lock-free producer. The ring is written from many tasks; it uses a single
//     producer-side head with byte-wise stores. Worst case under contention is a
//     garbled byte in the DEBUG FILE ONLY — never a crash, never a stall, and the
//     live Serial stream is always pristine.
//   • Bounded. Ring overflow drops oldest debug bytes (a dropped-byte counter is
//     printed), so a burst can never grow memory or block a producer.
//
// USAGE: DebugLog::begin() once in setup() after Serial.begin(); sdLogTask calls
// DebugLog::drainToFile() each pass. The `Serial` macro below redirects existing
// Serial.print* calls through the tee with NO call-site changes.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include "config.h"

#if defined(DEBUG_SERIAL_TO_SD) && DEBUG_SERIAL_TO_SD

class DebugTee : public Print {
public:
    void begin();
    // Print interface: mirror to real Serial, then copy into the ring.
    size_t write(uint8_t b) override;
    size_t write(const uint8_t* buf, size_t n) override;
    // Pass-throughs so `Serial.begin(...)`, `Serial.setTxTimeoutMs(...)`, and
    // truthiness (`if (Serial)`) keep working after the macro redirect.
    void begin(unsigned long baud) { Serial.begin(baud); }
    void setTxTimeoutMs(uint32_t ms) {
    #if ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(ms);
    #endif
    }
    operator bool() { return (bool)Serial; }
    int availableForWrite() { return Serial.availableForWrite(); }
    void flush() { Serial.flush(); }
};

extern DebugTee Debug;

namespace DebugLog {
    void begin();          // arm the tee (call once, after Serial.begin) — loads the
                            // NVS-persisted enable flag, or forces on if
                            // DEBUG_SERIAL_TO_SD_FORCE_ON is set in config.h
    void drainToFile();    // called by sdLogTask each pass: ring → SD (under sdMutex)
    void closeFile();      // flush + close (e.g. on log-session rotation)

    bool isEnabled();      // current runtime armed state
    bool isForced();       // true if DEBUG_SERIAL_TO_SD_FORCE_ON overrides the web/NVS toggle
    void setEnabled(bool on); // web-facing setter; persists to NVS. No-op when isForced().
    const char* currentFileName(); // bare name of the open debug file, "" if none —
                                    // the dashboard offers this for download via /log
}

// Redirect all existing `Serial.print*` call sites through the tee. Because
// DebugTee inherits Print, every print/println/printf/write overload is
// inherited unchanged — no call site is edited. Placed AFTER any Serial.begin
// in setup() would double-init, so the macro deliberately maps Serial→Debug and
// DebugTee::begin(baud) forwards to the real Serial.begin.
#define Serial Debug

#endif // DEBUG_SERIAL_TO_SD
