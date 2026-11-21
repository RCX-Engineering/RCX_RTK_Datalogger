// breadcrumb.h — where each task was when the chip last reset.
//
// A watchdog reset leaves no record of what the code was doing: the serial log ends mid-line and
// the reset reason names the watchdog, not the task or the operation. Each task here writes the
// phase it is entering into a small record kept in RTC memory, which survives watchdog, panic and
// software resets. At the next boot the record is printed (it lands in the debug log) and cleared.
//
// Reading it: every entry carries the millis() of its last update. Ages are taken back from the
// newest entry, which is the last moment any task ran. A task that kept updating until the end
// shows an age near zero. A task whose age is large and whose phase is not a waiting phase was
// inside that operation, and not returning from it, while the others carried on. If every task's
// age is small, nothing in the instrumented code was stuck: the whole chip stopped together, as
// it does for a flash write with the cache off, or something stopped interrupts on both cores.
//
// Cost: three stores to RTC memory per mark. No lock, no allocation, no pin, no partition and no
// flash write. A mark is never made from an interrupt handler. Stores by different tasks go to
// different entries, so no task can overwrite another's; an entry read at the instant of reset may
// carry a timestamp one mark older than its phase, which shifts an age by one phase at most.
#pragma once

#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>

// X(id, printed text, waiting). waiting = 1 for phases where the task is idle or blocked on purpose
// (queue wait, delay, mutex-free idle); a long age in such a phase is normal and is not reported as
// a stall.
#define CRUMB_PHASES(X)                                                                   \
    X(P_NONE,        "not started",                     1)                                \
    /* main loop (core 1) */                                                              \
    X(L_GNSS,        "loop: GNSS parse",                0)                                \
    X(L_IMU,         "loop: IMU I2C read",              0)                                \
    X(L_IMU_PUB,     "loop: IMU publish (data mutex)",  0)                                \
    X(L_TEMP,        "loop: die temperature",           0)                                \
    X(L_BLE_SNAP,    "loop: BLE snapshot (data mutex)", 0)                                \
    X(L_BLE,         "loop: BLE tick",                  0)                                \
    X(L_SDPUSH,      "loop: SD queue push",             0)                                \
    X(L_HEAP_FREE,   "loop: heap free size",            0)                                \
    X(L_HEAP_LARGEST,"loop: heap largest block",        0)                                \
    X(L_DIAG,        "loop: diagnostics",               0)                                \
    /* SD logging task */                                                                 \
    X(S_PUBLISH,     "sd: publish open-file sizes",     0)                                \
    X(S_CAPACITY,    "sd: capacity refresh",            0)                                \
    X(S_DIRSCAN,     "sd: directory scan",              0)                                \
    X(S_EXPORT,      "sd: archive build",               0)                                \
    X(S_RAWCAN,      "sd: raw CAN log",                 0)                                \
    X(S_IMU,         "sd: IMU log",                     0)                                \
    X(S_SAT,         "sd: satellite log",               0)                                \
    X(S_NMEA,        "sd: receiver capture log",        0)                                \
    X(S_CAN,         "sd: decoded CAN log",             0)                                \
    X(S_WAIT,        "sd: waiting for a GPS record",    1)                                \
    X(S_OPEN,        "sd: opening or recovering files", 0)                                \
    X(S_GPS,         "sd: GPS log",                     0)                                \
    /* CAN task */                                                                        \
    X(C_WAIT,        "can: waiting for a frame",        1)                                \
    X(C_DECODE,      "can: decode and queue frames",    0)                                \
    X(C_PUBLISH,     "can: publish (data mutex)",       0)                                \
    X(C_HEALTH,      "can: bus health and recovery",    0)                                \
    /* WiFi / NTRIP / RaceCapture task */                                                 \
    X(N_AP,          "net: access point service",       0)                                \
    X(N_WIFI,        "net: WiFi reconnect service",     0)                                \
    X(N_SNAP,        "net: snapshot (data mutex)",      0)                                \
    X(N_NTRIP,       "net: NTRIP",                      0)                                \
    X(N_RC,          "net: RaceCapture server",         0)                                \
    X(N_NVS,         "net: nearby-mount cache write to flash", 0)                         \
    X(N_IDLE,        "net: idle between passes",        1)                                \
    /* display task */                                                                    \
    X(D_SNAP,        "display: snapshot (data mutex)",  0)                                \
    X(D_DRAW,        "display: redraw",                 0)                                \
    X(D_IDLE,        "display: idle between passes",    1)                                \
    /* DBC task */                                                                        \
    X(B_IDLE,        "dbc: idle",                       1)                                \
    X(B_WORK,        "dbc: SD work",                    0)                                \
    /* web server (async_tcp) */                                                          \
    X(W_IDLE,        "web: idle",                       1)                                \
    X(W_LIST,        "web: file list",                  0)                                \
    X(W_READ,        "web: file download read",         0)                                \
    X(W_DELETE,      "web: delete files",               0)                                \
    X(W_EXPORT,      "web: archive request",            0)

namespace crumb {

enum Phase : uint8_t {
#define CRUMB_X(id, text, waiting) id,
    CRUMB_PHASES(CRUMB_X)
#undef CRUMB_X
    PHASE_COUNT
};

// One slot per task. The async_tcp task runs every web handler, so it has one slot of its own.
enum Slot : uint8_t { SLOT_LOOP, SLOT_SDLOG, SLOT_CAN, SLOT_NET, SLOT_DISPLAY, SLOT_DBC, SLOT_WEB, SLOT_COUNT };

struct Entry {
    volatile uint32_t ms;       // millis() at the last update
    volatile uint8_t  phase;    // Phase
    volatile uint8_t  core;     // core the task was running on at the update
    uint16_t          reserved;
};
struct Record {
    uint32_t magic;
    Entry    e[SLOT_COUNT];
};

extern Record g_crumbs;   // in RTC memory (breadcrumb.cpp)

// Record that task slot `s` is entering phase `p`. Phase first, timestamp last: an entry whose
// timestamp is current always carries the phase that goes with it.
static inline void mark(Slot s, Phase p) {
    Entry& en = g_crumbs.e[s];
    en.phase = (uint8_t)p;
    en.core  = (uint8_t)xPortGetCoreID();
    en.ms    = millis();
}

// Marks a phase for the length of a scope and sets `after` on every way out (return, early return).
struct Scope {
    Slot s; Phase after;
    Scope(Slot slot, Phase p, Phase done) : s(slot), after(done) { mark(s, p); }
    ~Scope() { mark(s, after); }
};

// Call once in setup(), before any task starts. If the previous run left a valid record and the
// chip did not just power up, prints it (reasonText is the printed reset reason). Then clears the
// record and arms it for this run.
void begin(esp_reset_reason_t reason, const char* reasonText);

}  // namespace crumb
