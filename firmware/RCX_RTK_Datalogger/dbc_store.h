#pragma once
/*
 * dbc_store.h — SD-backed store for user-supplied DBC files
 * =========================================================
 * Holds CAN database (.dbc) files uploaded from the dashboard so that vehicles
 * other than the built-in Porsche profiles can be decoded without a firmware
 * build. This module is storage only: it moves bytes and remembers which file
 * the operator selected. It does not parse DBC content and has no connection to
 * the CAN decode path, so nothing here can disturb a built-in profile.
 *
 * ── Threading ────────────────────────────────────────────────────────────────
 * The web handlers run in the async_tcp task, which must never touch the SD
 * card — enumerating the card from a web handler overran the Task-WDT (see the
 * directory-scan note in sd_log.cpp). The same rule applies here, so the split
 * is:
 *
 *   async_tcp task  →  fills a PSRAM staging buffer, sets a request flag
 *   dbcTask         →  performs every SD operation, holding the SD mutex
 *
 * Requests are single-slot and are consumed by dbcTask on its next pass. A
 * request issued while one is already in flight is rejected rather than queued;
 * the dashboard serialises operations, and dropping a duplicate is safer than
 * growing an unbounded queue in an interrupt-adjacent context.
 */

#include <stdint.h>
#include <stddef.h>

// One entry in the directory snapshot the dashboard reads.
struct DbcFileEntry {
    char     name[32];   // bare filename, no leading slash
    uint32_t size;       // bytes on card
};

// ── Lifecycle ────────────────────────────────────────────────────────────────
// Loads the selected filename from NVS and starts dbcTask. Safe to call before
// the SD card has mounted; the task waits for it.
void dbc_init();

// ── Upload (called from the async_tcp task only) ─────────────────────────────
// Streams one uploaded file into the PSRAM staging buffer. begin() validates and
// sanitises the filename and reserves the buffer; chunk() appends; end() hands
// the staged file to dbcTask to be committed to the card. Each returns false on
// rejection, after which the remaining chunks of that upload are discarded.
bool dbc_uploadBegin(const char* filename);
bool dbc_uploadChunk(const uint8_t* data, size_t len);
bool dbc_uploadEnd();

// Human-readable result of the most recent store operation, for the dashboard.
// Never null; empty before the first operation.
const char* dbc_lastStatus();

// ── Directory snapshot ───────────────────────────────────────────────────────
// The dashboard asks for a scan, then reads the RAM snapshot once the scan has
// completed. It must not enumerate the card itself.
void dbc_requestScan();
bool dbc_scanPending();
int  dbc_getSnapshot(DbcFileEntry* out, int cap);   // returns entry count

// Queues removal of one file. The active selection is cleared if it names the
// file being removed.
bool dbc_requestDelete(const char* name);

// ── Active selection ─────────────────────────────────────────────────────────
// The selected file is remembered by name in NVS and survives a reboot. Passing
// an empty string clears the selection. Selecting a file records the choice
// only — no CAN decode behaviour is attached to it yet.
bool        dbc_setActive(const char* name);
const char* dbc_getActive();   // "" when nothing is selected

// Filename the retained audit report describes, or "" if none has run.
const char* dbc_getAudited();

// Internal task entry point.
void dbcTask(void*);
