/*
 * dbc_parse.cpp — DBC reader and import audit
 * ===========================================
 * The file is read a line at a time rather than loaded whole, so import cost
 * does not scale with file size. Only the signal table is retained.
 */

#include <Arduino.h>
#include <SD_MMC.h>
#include <esp_heap_caps.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include "config.h"
#include "dbc_parse.h"

static DbcSignal* s_sig      = nullptr;
static int        s_sigCount = 0;

static char*  s_report    = nullptr;
static size_t s_reportLen = 0;

static DbcAudit s_audit = {};

const char*     dbc_auditText()    { return s_report ? s_report : ""; }
const DbcAudit& dbc_auditSummary() { return s_audit; }
int             dbc_signalCount()  { return s_sigCount; }

const DbcSignal* dbc_signalAt(int i) {
    return (i >= 0 && i < s_sigCount) ? &s_sig[i] : nullptr;
}

// Append to the report, stopping cleanly at the buffer end. A truncated report
// is acceptable; the summary counts remain accurate because they are tallied
// independently of the text.
static void rep(const char* fmt, ...) {
    if (!s_report || s_reportLen >= DBC_REPORT_BYTES - 1) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s_report + s_reportLen, DBC_REPORT_BYTES - s_reportLen, fmt, ap);
    va_end(ap);
    if (n > 0) {
        s_reportLen += (size_t)n;
        if (s_reportLen >= DBC_REPORT_BYTES) s_reportLen = DBC_REPORT_BYTES - 1;
    }
}

static void finding(const char* fmt, ...) {
    s_audit.findings++;
    if (!s_report || s_reportLen >= DBC_REPORT_BYTES - 1) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s_report + s_reportLen, DBC_REPORT_BYTES - s_reportLen, fmt, ap);
    va_end(ap);
    if (n > 0) {
        s_reportLen += (size_t)n;
        if (s_reportLen >= DBC_REPORT_BYTES) s_reportLen = DBC_REPORT_BYTES - 1;
    }
}

// Widest text a value can occupy in the telemetry sample, including its
// separating comma. Derived from the signal's own declared range and the
// decimal places implied by its scale factor, so the estimate reflects this
// file rather than an assumed average.
static uint32_t worstFieldBytes(const DbcSignal& s) {
    float mag = fmaxf(fabsf(s.minVal), fabsf(s.maxVal));
    if (!(mag > 0.0f) || isnan(mag) || isinf(mag)) mag = 1.0f;

    int intDigits = (int)floorf(log10f(mag)) + 1;
    if (intDigits < 1) intDigits = 1;

    // Decimal places needed to represent the scale factor without loss, capped
    // at the precision the telemetry encoder emits.
    int dec = 0;
    float f = fabsf(s.factor);
    while (f > 0.0f && f < 1.0f && dec < 4) { f *= 10.0f; dec++; }

    uint32_t n = (uint32_t)intDigits;
    if (dec > 0) n += 1 + (uint32_t)dec;          // decimal point + places
    if (s.minVal < 0.0f || s.offset < 0.0f) n++;  // sign
    return n + 1;                                  // comma
}

// Extract the signal name from an SG_ line and report whether it carries a
// multiplex marker. `p` points just past "SG_".
static const char* readSignalName(const char* p, char* out, size_t outSz, bool* muxed) {
    while (*p == ' ' || *p == '\t') p++;
    size_t n = 0;
    while (*p && *p != ' ' && *p != ':' && n < outSz - 1) out[n++] = *p++;
    out[n] = '\0';

    *muxed = false;
    while (*p == ' ') p++;
    if (*p == 'M' || (*p == 'm' && p[1] >= '0' && p[1] <= '9')) {
        *muxed = true;
        while (*p && *p != ':') p++;
    }
    while (*p && *p != ':') p++;
    return (*p == ':') ? p + 1 : p;
}

static void resetState() {
    s_sigCount  = 0;
    s_reportLen = 0;
    memset(&s_audit, 0, sizeof(s_audit));
    if (s_report) s_report[0] = '\0';
}

bool dbc_parseAndAudit(const char* bareName) {
    if (!s_sig) {
        size_t bytes = sizeof(DbcSignal) * (size_t)DBC_MAX_CHANNELS;
        s_sig = (DbcSignal*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
        if (!s_sig) s_sig = (DbcSignal*)malloc(bytes);
    }
    if (!s_report) {
        s_report = (char*)heap_caps_malloc(DBC_REPORT_BYTES, MALLOC_CAP_SPIRAM);
        if (!s_report) s_report = (char*)malloc(DBC_REPORT_BYTES);
    }
    if (!s_sig || !s_report) return false;

    resetState();

    char path[64];
    snprintf(path, sizeof(path), "%s/%s", DBC_DIR, bareName);
    File f = SD_MMC.open(path);
    if (!f) { rep("Could not open %s.\n", bareName); return false; }

    rep("Import audit — %s\n", bareName);
    rep("================================================\n\n");

    uint32_t curId = 0;
    uint8_t  curDlc = 0;
    bool     haveMsg = false;
    int      extendedIds = 0, muxSkipped = 0, floatSkipped = 0, overrun = 0, tooLong = 0;
    bool     capped = false;

    char line[256];
    while (f.available()) {
        size_t n = f.readBytesUntil('\n', line, sizeof(line) - 1);
        line[n] = '\0';
        char* p = line;
        while (*p == ' ' || *p == '\t' || *p == '\r') p++;

        if (strncmp(p, "BO_ ", 4) == 0) {
            unsigned long id = 0; int dlc = 0;
            char nm[40];
            if (sscanf(p + 4, "%lu %39[^:]: %d", &id, nm, &dlc) == 3) {
                // Bit 31 marks an extended (29-bit) identifier in DBC. The TWAI
                // receive path filters those out, so such a message can never
                // match live traffic no matter how well it parses.
                if (id & 0x80000000UL) { extendedIds++; id &= 0x1FFFFFFFUL; }
                curId   = (uint32_t)id;
                curDlc  = (uint8_t)dlc;
                haveMsg = true;
                s_audit.messages++;
            }
            continue;
        }

        if (strncmp(p, "SG_ ", 4) != 0) {
            if (strncmp(p, "SIG_VALTYPE_ ", 13) == 0) floatSkipped++;
            continue;
        }

        s_audit.signalsFound++;
        if (!haveMsg) continue;

        char nm[32]; bool muxed = false;
        const char* rest = readSignalName(p + 4, nm, sizeof(nm), &muxed);
        if (muxed) { muxSkipped++; continue; }

        int start = 0, len = 0, order = 1;
        char sgn = '+';
        float fac = 1, off = 0, mn = 0, mx = 0;
        char unit[12] = {0};

        int got = sscanf(rest, " %d|%d@%d%c (%f,%f) [%f|%f] \"%11[^\"]\"",
                         &start, &len, &order, &sgn, &fac, &off, &mn, &mx, unit);
        if (got < 8) continue;   // unreadable definition; counted, not accepted

        if (s_sigCount >= DBC_MAX_CHANNELS) { capped = true; continue; }
        if (len < 1 || len > 64) continue;
        if (curDlc > 0 && (start + len) > (int)curDlc * 8) { overrun++; continue; }

        DbcSignal& s = s_sig[s_sigCount];
        memset(&s, 0, sizeof(s));
        strncpy(s.name, nm, sizeof(s.name) - 1);
        strncpy(s.unit, unit, sizeof(s.unit) - 1);
        s.msgId        = curId;
        s.msgDlc       = curDlc;
        s.startBit     = (uint16_t)start;
        s.bitLen       = (uint8_t)len;
        s.littleEndian = (order == 1);
        s.isSigned     = (sgn == '-');
        s.factor       = fac;
        s.offset       = off;
        s.minVal       = mn;
        s.maxVal       = mx;
        s_sigCount++;

        if (strlen(nm) > 11) tooLong++;
    }
    f.close();

    s_audit.signalsAccepted = s_sigCount;

    // ── Telemetry budget ─────────────────────────────────────────────────────
    // Every sample carries the full channel array, so cost scales with channel
    // count regardless of how often a channel actually changes.
    uint32_t bytes = 24;   // envelope and timestamp
    for (int i = 0; i < s_sigCount; i++) bytes += worstFieldBytes(s_sig[i]);
    s_audit.worstSampleBytes = bytes;
    s_audit.worstChunks      = (bytes + BLE_CHUNK_BYTES - 1) / BLE_CHUNK_BYTES;

    rep("Messages          %d\n", s_audit.messages);
    rep("Signals found     %d\n", s_audit.signalsFound);
    rep("Signals usable    %d\n", s_audit.signalsAccepted);
    rep("Worst sample      %lu bytes\n", (unsigned long)bytes);
    rep("BLE notifications %lu per sample at %d Hz\n\n",
        (unsigned long)s_audit.worstChunks, CAN_TELEMETRY_HZ);

    // ── Findings ─────────────────────────────────────────────────────────────
    rep("Findings\n--------\n");

    if (s_sigCount == 0)
        finding("No usable signals. Nothing would be logged from this file.\n");

    if (capped)
        finding("Signal count exceeds the %d-channel limit. Signals beyond that "
                "were dropped in file order — split the database or remove "
                "channels you do not need.\n", DBC_MAX_CHANNELS);

    if (s_audit.worstChunks > 1)
        finding("A sample spans %lu notifications. Multi-notification transfer "
                "works, but each one costs time inside the %d ms sample window "
                "and the receiving app has its own ceiling.\n",
                (unsigned long)s_audit.worstChunks, 1000 / CAN_TELEMETRY_HZ);

    if (extendedIds)
        finding("%d message(s) use 29-bit extended identifiers. The receive path "
                "accepts standard 11-bit frames only, so these will never "
                "match live traffic.\n", extendedIds);

    if (muxSkipped)
        finding("%d multiplexed signal(s) skipped. The same bits carry different "
                "meanings depending on a mux selector, which is not read here.\n",
                muxSkipped);

    if (floatSkipped)
        finding("%d SIG_VALTYPE_ declaration(s) found. Signals declared as IEEE "
                "floats are not read as such and would decode incorrectly.\n",
                floatSkipped);

    if (overrun)
        finding("%d signal(s) extend past the end of their message and were "
                "dropped. Check start bit, length, and DLC.\n", overrun);

    if (tooLong)
        finding("%d signal name(s) exceed 11 characters. They log to CSV in full "
                "but are truncated in the app, where two names sharing their "
                "first 11 characters become indistinguishable.\n", tooLong);

    // Duplicate names collide silently downstream, so they are worth naming
    // individually rather than counting.
    for (int i = 0; i < s_sigCount; i++)
        for (int j = i + 1; j < s_sigCount; j++)
            if (strncmp(s_sig[i].name, s_sig[j].name, 11) == 0) {
                finding("Duplicate channel name within first 11 characters: "
                        "%s / %s.\n", s_sig[i].name, s_sig[j].name);
                j = s_sigCount;
            }

    // Unit strings that indicate display units rather than the logger's native
    // ones. The conversion to display units happens downstream, so a database
    // that has already converted produces values wrong by a fixed ratio —
    // plausible-looking, and easy to miss in a log.
    for (int i = 0; i < s_sigCount; i++) {
        const char* u = s_sig[i].unit;
        if (!u[0]) continue;
        if (!strcasecmp(u, "mph") || !strcasecmp(u, "psi") ||
            !strcasecmp(u, "F")   || !strcasecmp(u, "degF"))
            finding("%s is declared in %s. Scale and offset must produce km/h, "
                    "bar, or degrees C; display conversion happens afterwards.\n",
                    s_sig[i].name, u);
    }

    for (int i = 0; i < s_sigCount; i++)
        if (s_sig[i].factor == 0.0f)
            finding("%s has a scale factor of zero and would read as a constant.\n",
                    s_sig[i].name);

    if (s_audit.findings == 0) rep("None.\n");

    s_audit.valid = (s_sigCount > 0);
    Serial.printf("🗂️  DBC audit %s: %d signals, %lu B/sample, %d finding(s)\n",
                  bareName, s_sigCount, (unsigned long)bytes, s_audit.findings);
    return s_audit.valid;
}
