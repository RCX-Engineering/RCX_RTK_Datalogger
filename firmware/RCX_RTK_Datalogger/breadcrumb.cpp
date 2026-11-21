// breadcrumb.cpp — storage and boot-time report for the per-task breadcrumbs (see breadcrumb.h).
#include "breadcrumb.h"
#include <string.h>

namespace crumb {

// RTC memory that is not initialised at start-up: it keeps its contents through watchdog, panic
// and software resets. After a power-up it holds noise, which the magic value rejects.
RTC_NOINIT_ATTR Record g_crumbs;

static const uint32_t kMagic = 0x42555243u;      // "CRUB"

// A phase that is not a waiting phase and has not been updated for this long, while other tasks
// kept running, is reported as where the task was held.
static const uint32_t kStuckMs = 250;

static const char* const kText[PHASE_COUNT] = {
#define CRUMB_X(id, text, waiting) text,
    CRUMB_PHASES(CRUMB_X)
#undef CRUMB_X
};
static const bool kWaiting[PHASE_COUNT] = {
#define CRUMB_X(id, text, waiting) (waiting) != 0,
    CRUMB_PHASES(CRUMB_X)
#undef CRUMB_X
};
static const char* const kSlotName[SLOT_COUNT] = {
    "loop", "SDLog", "CAN", "WiFiNTRIP", "Display", "DBC", "async_tcp"
};

static void printReport(const char* reasonText) {
    // Copy first: tasks are not running yet, but the printing below must not depend on that.
    Record r;
    memcpy(&r, (const void*)&g_crumbs, sizeof(r));

    bool have = false;
    uint32_t newest = 0;
    for (int i = 0; i < SLOT_COUNT; i++) {
        if (r.e[i].phase == P_NONE || r.e[i].phase >= PHASE_COUNT) continue;
        if (!have || (int32_t)(r.e[i].ms - newest) > 0) newest = r.e[i].ms;
        have = true;
    }
    Serial.printf("🧭 Breadcrumbs from the run before this reset (%s)\n",
                  reasonText ? reasonText : "unknown");
    if (!have) {
        Serial.println("   no task had recorded a phase");
        return;
    }
    Serial.println("   age = time before the newest entry; a long age outside a waiting phase means "
                   "that task was inside the phase and not returning from it");

    int worst = -1;
    uint32_t worstAge = 0;
    for (int i = 0; i < SLOT_COUNT; i++) {
        const uint8_t ph = r.e[i].phase;
        if (ph >= PHASE_COUNT) {
            Serial.printf("   %-10s (entry unreadable)\n", kSlotName[i]);
            continue;
        }
        if (ph == P_NONE) {
            Serial.printf("   %-10s %s\n", kSlotName[i], kText[P_NONE]);
            continue;
        }
        const uint32_t age = (uint32_t)(int32_t)(newest - r.e[i].ms);
        const bool stuck = !kWaiting[ph] && age >= kStuckMs;
        if (stuck && age >= worstAge) { worst = i; worstAge = age; }
        Serial.printf("   %-10s core %u  %-42s age %7lu ms%s\n", kSlotName[i], (unsigned)r.e[i].core,
                      kText[ph], (unsigned long)age, stuck ? "  <== held here" : "");
    }
    if (worst >= 0) {
        Serial.printf("🧭 Suspect: %s, in \"%s\", for %lu ms while the newest entry was still being written\n",
                      kSlotName[worst], kText[r.e[worst].phase], (unsigned long)worstAge);
    } else {
        Serial.printf("🧭 No task was held in one phase for %lu ms or more: the chip stopped as a whole "
                      "(a flash write with the cache off, or interrupts held off on both cores) or inside "
                      "code that carries no marks\n", (unsigned long)kStuckMs);
    }
}

void begin(esp_reset_reason_t reason, const char* reasonText) {
    if (g_crumbs.magic == kMagic && reason != ESP_RST_POWERON) printReport(reasonText);
    memset((void*)&g_crumbs, 0, sizeof(g_crumbs));
    g_crumbs.magic = kMagic;
}

}  // namespace crumb
