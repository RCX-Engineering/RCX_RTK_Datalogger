#pragma once
/*
 * dbc_parse.h — DBC reader and import audit
 * =========================================
 * Reads a CAN database file from the SD card, extracts its message and signal
 * definitions, and produces a plain-text audit describing what the logger would
 * do with it. The audit runs at import so that a file which cannot be used, or
 * can only be used partially, says so while the operator is still at the
 * dashboard rather than at an event.
 *
 * ── Supported subset ─────────────────────────────────────────────────────────
 * DBC has no published standard; it is Vector's de facto format. The subset
 * read here is the part every tool writes identically:
 *
 *   BO_ <id> <name>: <dlc> <transmitter>
 *    SG_ <name> : <start>|<len>@<order><sign> (<factor>,<offset>) [<min>|<max>] "<unit>" <rx>
 *
 * Conversion is linear throughout — physical = raw * factor + offset — so no
 * expression evaluation is involved anywhere in this module.
 *
 * Constructs outside that subset are reported as findings rather than guessed
 * at. A signal the logger would misread is worse than a signal it declines to
 * read, because the misread one still produces plausible numbers.
 *
 * ── Threading ────────────────────────────────────────────────────────────────
 * Parsing reads the SD card, so it runs only on dbcTask, behind the SD mutex.
 * Web handlers read the finished audit text out of RAM.
 */

#include <stdint.h>
#include <stddef.h>

// One decoded signal definition. Retained for the audit and, in a later stage,
// for decode; nothing outside this module interprets it yet.
struct DbcSignal {
    char     name[32];
    uint32_t msgId;
    uint8_t  msgDlc;
    uint16_t startBit;
    uint8_t  bitLen;
    bool     littleEndian;   // DBC @1 = Intel, @0 = Motorola
    bool     isSigned;
    float    factor;
    float    offset;
    float    minVal;
    float    maxVal;
    char     unit[12];
};

// Summary of one import. Counts are of what was found in the file, not of what
// survived; rejected items appear in the audit text.
struct DbcAudit {
    bool     valid;            // false when the file cannot be used at all
    int      messages;         // BO_ definitions seen
    int      signalsFound;     // SG_ definitions seen
    int      signalsAccepted;  // usable within the supported subset
    int      findings;         // warning lines in the report
    uint32_t worstSampleBytes; // largest possible telemetry sample, in bytes
    uint32_t worstChunks;      // BLE notifications that sample would need
};

// Parse `bareName` from the DBC directory and build its audit. Runs on dbcTask
// only. Returns false if the file could not be opened or contains no signals;
// the audit text explains either way.
bool dbc_parseAndAudit(const char* bareName);

// The most recent audit report, as plain text, and its summary. Never null.
const char* dbc_auditText();
const DbcAudit& dbc_auditSummary();

// Signals retained from the most recent successful parse.
int              dbc_signalCount();
const DbcSignal* dbc_signalAt(int i);
