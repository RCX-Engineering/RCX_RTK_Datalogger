/*
 * dbc_store.cpp — SD-backed store for user-supplied DBC files
 * ===========================================================
 * See dbc_store.h for the threading contract. In short: web handlers only ever
 * write to the staging buffer and set a request flag; dbcTask owns the card.
 */

#include <Arduino.h>
#include <SD_MMC.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <esp_heap_caps.h>

#include "config.h"
#include "dbc_store.h"
#include "sd_log.h"
#include "dbc_parse.h"

static const char* NVS_NS = "rcx_dbc";

// ── Staging buffer ───────────────────────────────────────────────────────────
// Allocated from PSRAM on first use and kept for the life of the run. Uploads
// are small and infrequent, and holding one buffer avoids repeated large
// allocations that would fragment the heap over a long session.
static uint8_t*  s_stage     = nullptr;
static size_t    s_stageLen  = 0;
static bool      s_stageOpen = false;      // an upload is streaming in
static char      s_stageName[32]  = {0};

// ── Request slots (async_tcp → dbcTask) ──────────────────────────────────────
static volatile bool s_commitReq = false;
static volatile bool s_deleteReq = false;
static volatile bool s_scanReq   = false;
static char          s_deleteName[32] = {0};

// ── Directory snapshot (dbcTask → async_tcp) ─────────────────────────────────
static DbcFileEntry  s_snap[DBC_MAX_FILES];
static volatile int  s_snapCount = 0;

static char s_active[32] = {0};
static char s_audited[32] = {0};   // file the retained audit report describes
static char s_status[96] = {0};

static void setStatus(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_status, sizeof(s_status), fmt, ap);
    va_end(ap);
}

// ── Filename handling ────────────────────────────────────────────────────────
// Browsers may submit a full client-side path, and a name reaching the card
// unchecked could escape the DBC directory or produce a name FatFs cannot
// represent. Everything outside a conservative set is rejected rather than
// silently rewritten, so what the operator sees listed is what they uploaded.
static bool sanitiseName(const char* in, char* out, size_t outSz) {
    if (!in || !out || outSz < 8) return false;

    // Keep only the final path component, whichever separator the client used.
    const char* base = in;
    for (const char* p = in; *p; ++p)
        if (*p == '/' || *p == '\\') base = p + 1;

    size_t n = strlen(base);
    if (n == 0 || n >= outSz) return false;

    // Case-insensitive .dbc extension check.
    if (n < 5) return false;
    const char* ext = base + n - 4;
    if (!(ext[0] == '.' &&
          tolower((unsigned char)ext[1]) == 'd' &&
          tolower((unsigned char)ext[2]) == 'b' &&
          tolower((unsigned char)ext[3]) == 'c')) return false;

    for (size_t i = 0; i < n; i++) {
        char c = base[i];
        bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                  (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-';
        if (!ok) return false;
    }
    memcpy(out, base, n);
    out[n] = '\0';
    return true;
}

// Build the on-card path for a bare filename.
static void pathFor(const char* bare, char* out, size_t outSz) {
    snprintf(out, outSz, "%s/%s", DBC_DIR, bare);
}

// ── Upload path (async_tcp task) ─────────────────────────────────────────────
bool dbc_uploadBegin(const char* filename) {
    s_stageOpen = false;
    s_stageLen  = 0;

    if (s_commitReq) { setStatus("Busy — a previous upload is still saving."); return false; }

    char clean[32];
    if (!sanitiseName(filename, clean, sizeof(clean))) {
        setStatus("Rejected: name must end in .dbc and use letters, digits, . _ - only.");
        return false;
    }

    if (!s_stage) {
        s_stage = (uint8_t*)heap_caps_malloc(DBC_MAX_FILE_BYTES, MALLOC_CAP_SPIRAM);
        if (!s_stage) s_stage = (uint8_t*)malloc(DBC_MAX_FILE_BYTES);
        if (!s_stage) { setStatus("Rejected: no memory for upload buffer."); return false; }
    }

    strncpy(s_stageName, clean, sizeof(s_stageName) - 1);
    s_stageName[sizeof(s_stageName) - 1] = '\0';
    s_stageOpen = true;
    setStatus("Receiving %s…", s_stageName);
    return true;
}

bool dbc_uploadChunk(const uint8_t* data, size_t len) {
    if (!s_stageOpen || !s_stage) return false;
    if (s_stageLen + len > DBC_MAX_FILE_BYTES) {
        s_stageOpen = false;
        s_stageLen  = 0;
        setStatus("Rejected: file exceeds %u KB.", (unsigned)(DBC_MAX_FILE_BYTES / 1024));
        return false;
    }
    memcpy(s_stage + s_stageLen, data, len);
    s_stageLen += len;
    return true;
}

bool dbc_uploadEnd() {
    if (!s_stageOpen) return false;
    s_stageOpen = false;
    if (s_stageLen == 0) { setStatus("Rejected: empty file."); return false; }
    s_commitReq = true;                    // dbcTask writes it to the card
    setStatus("Saving %s…", s_stageName);
    return true;
}

const char* dbc_lastStatus() { return s_status; }

// ── Snapshot + requests (async_tcp task) ─────────────────────────────────────
void dbc_requestScan() { s_scanReq = true; }
bool dbc_scanPending() { return s_scanReq; }

int dbc_getSnapshot(DbcFileEntry* out, int cap) {
    int n = s_snapCount;
    if (n > cap) n = cap;
    for (int i = 0; i < n; i++) out[i] = s_snap[i];
    return n;
}

bool dbc_requestDelete(const char* name) {
    if (s_deleteReq) return false;
    char clean[32];
    if (!sanitiseName(name, clean, sizeof(clean))) return false;
    strncpy(s_deleteName, clean, sizeof(s_deleteName) - 1);
    s_deleteName[sizeof(s_deleteName) - 1] = '\0';
    s_deleteReq = true;
    return true;
}

const char* dbc_getActive()  { return s_active; }
const char* dbc_getAudited() { return s_audited; }

// Re-audit request, used when the operator selects an already-stored file so
// the report on screen always describes the file named beside it.
static volatile bool s_auditReq = false;
static char          s_auditName[32] = {0};

bool dbc_setActive(const char* name) {
    char clean[32];
    if (name && name[0]) {
        if (!sanitiseName(name, clean, sizeof(clean))) return false;
    } else {
        clean[0] = '\0';                   // empty selects nothing
    }

    strncpy(s_active, clean, sizeof(s_active) - 1);
    s_active[sizeof(s_active) - 1] = '\0';

    Preferences p;
    if (p.begin(NVS_NS, false)) {
        if (s_active[0]) p.putString("active", s_active);
        else             p.remove("active");
        p.end();
    }
    if (s_active[0]) {
        strncpy(s_auditName, s_active, sizeof(s_auditName) - 1);
        s_auditName[sizeof(s_auditName) - 1] = '\0';
        s_auditReq = true;
    }
    setStatus(s_active[0] ? "Selected %s — auditing…" : "Selection cleared.", s_active);
    Serial.printf("🗂️  DBC: active file %s\n", s_active[0] ? s_active : "(none)");
    return true;
}

// ── SD operations (dbcTask only) ─────────────────────────────────────────────
static void doCommit() {
    char path[64];
    pathFor(s_stageName, path, sizeof(path));

    SD_MMC.mkdir(DBC_DIR);                 // harmless if it already exists
    File f = SD_MMC.open(path, FILE_WRITE);
    if (!f) { setStatus("Save failed: could not open %s.", s_stageName); return; }

    size_t written = f.write(s_stage, s_stageLen);
    f.close();

    if (written != s_stageLen) {
        SD_MMC.remove(path);               // a truncated DBC is worse than none
        setStatus("Save failed: short write on %s.", s_stageName);
        return;
    }
    Serial.printf("🗂️  DBC: saved %s (%u bytes)\n", s_stageName, (unsigned)s_stageLen);

    // Audit immediately, while the operator is still at the dashboard. A file
    // the logger cannot fully use is far cheaper to discover here than at an
    // event, so the result is reported even when the file is unusable — the
    // file is kept either way so it can be corrected and re-uploaded.
    strncpy(s_audited, s_stageName, sizeof(s_audited) - 1);
    s_audited[sizeof(s_audited) - 1] = '\0';

    if (dbc_parseAndAudit(s_stageName)) {
        const DbcAudit& a = dbc_auditSummary();
        setStatus("Saved %s — %d channels, %d finding(s). See the audit below.",
                  s_stageName, a.signalsAccepted, a.findings);
    } else {
        setStatus("Saved %s, but no usable channels were found. See the audit below.",
                  s_stageName);
    }
    s_scanReq = true;                      // refresh the listing for the dashboard
}

static void doDelete() {
    char path[64];
    pathFor(s_deleteName, path, sizeof(path));

    if (SD_MMC.remove(path)) {
        setStatus("Deleted %s.", s_deleteName);
        // Leaving a selection pointing at a file that no longer exists would
        // read as though a database were still loaded.
        if (strcmp(s_active, s_deleteName) == 0) dbc_setActive("");
    } else {
        setStatus("Delete failed: %s.", s_deleteName);
    }
    s_scanReq = true;
}

static void doScan() {
    int n = 0;
    File dir = SD_MMC.open(DBC_DIR);
    if (dir && dir.isDirectory()) {
        for (File e = dir.openNextFile(); e && n < DBC_MAX_FILES; e = dir.openNextFile()) {
            if (!e.isDirectory()) {
                const char* nm = e.name();
                for (const char* p = nm; *p; ++p) if (*p == '/') nm = p + 1;
                strncpy(s_snap[n].name, nm, sizeof(s_snap[n].name) - 1);
                s_snap[n].name[sizeof(s_snap[n].name) - 1] = '\0';
                s_snap[n].size = (uint32_t)e.size();
                n++;
            }
            e.close();
        }
    }
    if (dir) dir.close();
    s_snapCount = n;
}

void dbcTask(void*) {
    // The card is mounted by sdLogTask; nothing here may touch SD_MMC until it
    // reports ready, and the SD mutex is held for every operation so a scan can
    // never interleave with a log write.
    while (!sdlog_isReady()) vTaskDelay(pdMS_TO_TICKS(250));

    s_scanReq = true;                      // prime the listing at boot

    for (;;) {
        if (s_commitReq || s_deleteReq || s_scanReq) {
            SemaphoreHandle_t mtx = sdlog_getMutex();
            if (!mtx || xSemaphoreTake(mtx, pdMS_TO_TICKS(2000)) == pdTRUE) {
                if (s_commitReq) { doCommit(); s_commitReq = false; }
                if (s_auditReq) {
                    strncpy(s_audited, s_auditName, sizeof(s_audited) - 1);
                    s_audited[sizeof(s_audited) - 1] = '\0';
                    dbc_parseAndAudit(s_auditName);
                    const DbcAudit& a = dbc_auditSummary();
                    setStatus("%s — %d channels, %d finding(s).",
                              s_auditName, a.signalsAccepted, a.findings);
                    s_auditReq = false;
                }
                if (s_deleteReq) { doDelete(); s_deleteReq = false; }
                if (s_scanReq)   { doScan();   s_scanReq   = false; }
                if (mtx) xSemaphoreGive(mtx);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void dbc_init() {
    Preferences p;
    if (p.begin(NVS_NS, true)) {
        String a = p.getString("active", "");
        strncpy(s_active, a.c_str(), sizeof(s_active) - 1);
        s_active[sizeof(s_active) - 1] = '\0';
        p.end();
    }
    Serial.printf("🗂️  DBC store: active file %s\n", s_active[0] ? s_active : "(none)");

    xTaskCreatePinnedToCore(dbcTask, "dbcTask", 4096, nullptr, 1, nullptr, 0);
}
