/*
 * gnss.cpp — LG290P GNSS driver + NMEA parser
 * =============================================
 * Ported from RaceBox_LG290P.ino.  Now exports gnss_init() and gnss_loop()
 * so setup()/loop() stay clean.  All shared state is written under dataMutex.
 */

#include "gnss.h"
#include "config.h"
#include "types.h"
#include "sd_log.h"   // sdlog_push_sat — gnss.cpp is the sat-log GSV producer
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include "debug_log.h"   // LAST include: serial-to-SD tee macro must not touch library headers above

// ── Module-level state ────────────────────────────────────────────────────────
TinyGPSPlus tinygps;
float gnss_updateRate_Hz = 0.0f;

// TinyGPSCustom objects for fields TinyGPSPlus doesn't parse natively
static TinyGPSCustom fixQuality (tinygps, "GNGGA", 6);
static TinyGPSCustom fixQuality2(tinygps, "GPGGA", 6);
static TinyGPSCustom geoidSep   (tinygps, "GNGGA", 11);
static TinyGPSCustom geoidSep2  (tinygps, "GPGGA", 11);
static TinyGPSCustom diffAge    (tinygps, "GNGGA", 13);   // age (s) of differential/RTK corrections
static TinyGPSCustom diffAge2   (tinygps, "GPGGA", 13);
// GGA field 14 <StationID> — the ONLY way to tell a PPP solution from an RTK FLOAT one.
// Quectel PPP App Note V1.0.0 §3.1: a PPP fix reports GGA quality 5 (identical to RTK
// FLOAT) and is distinguished solely by the differential reference station ID:
//   9001 = PPP from B2b corrections, 9002 = PPP from Galileo E6 HAS corrections.
// Any other value is a real RTCM base (our NTRIP caster). Without this parse, PPP would
// be silently mislabelled "FLOAT" on the TFT, in the SD log, and over BLE.
static TinyGPSCustom staId      (tinygps, "GNGGA", 14);
static TinyGPSCustom staId2     (tinygps, "GPGGA", 14);

// ── Tunable multipath/occlusion masks (persisted in NVS namespace "rcx_gnss") ──
// Loaded from NVS in gnss_init(); pushed to the LG290P by the GNSS task (never the
// web task) when s_maskApplyPending is set, or written in configureLG290P() on a
// config-version bump. See config.h for the defaults and rationale.
static float          s_eleMask = GNSS_ELE_MASK_DEG_DEFAULT;   // ° elevation cutoff
static float          s_cnrMask = GNSS_CNR_MASK_DBHZ_DEFAULT;  // dB-Hz C/N0 cutoff
static volatile bool  s_maskApplyPending = false;              // web → GNSS-task handoff

// ── PPP (Precise Point Positioning) — RTK's understudy, never its replacement ──
// Persisted in the same NVS namespace ("rcx_gnss": ppp). Mode is the <Mode> field of
// $PQTMCFGPPP: 0 = off, 1 = BeiDou B2b, 2 = Galileo E6 HAS. Written by configureLG290P()
// on a config-version bump, or pushed live by the GNSS task when s_pppApplyPending is set.
//
//   *** WHY THIS IS SAFE TO LEAVE ON (and where it is NOT) ***
//   PPP is a CORRECTION SOURCE, not a mode. The nav engine still solves RTK whenever RTCM
//   is flowing, and RTK FIXED outranks a PPP solution (PQTMPPPNAV <SolType>: 12 = RTK fixed
//   > 7 = PPP converged). So the ladder is exactly the intended one:
//        RTK FIXED (incl. RTK HOLD coasting) → RTK FLOAT → PPP → 3D GPS
//   Nothing is locked out: with PPP off, or un-converged, the module falls back to the
//   ordinary single-point solution on its own. No extra host logic is needed for fallback.
//
//   *** WHAT PPP IS NOT ***
//   It is NOT a fast fallback. Convergence is ~10-20 min in open sky, and LG290P fw v2.01
//   has no fast re-acquisition: a cycle slip (canopy, trailer, hard occlusion) costs the
//   FULL convergence again. It is also more multipath-vulnerable than RTK while reporting
//   optimistic accuracy. Treat a PPP solution as decimetre-class at best, and never trust
//   its self-reported accuracy the way you would an RTK FIXED one.
static uint8_t        s_pppMode = GNSS_PPP_MODE_DEFAULT;       // 0=off 1=B2b 2=E6 HAS
static volatile bool  s_pppApplyPending = false;               // web → GNSS-task handoff

// Rate tracking
static unsigned long gnssUpdateCount = 0;
static unsigned long lastRateCalc    = 0;

// ── NMEA raw-line buffer ──────────────────────────────────────────────────────
static char   nmeaLine[200];
static uint8_t nmeaIdx = 0;

// ── PQTM helpers ─────────────────────────────────────────────────────────────
void gnss_checksum(const char* sentence, char* out2) {
    uint8_t ck = 0;
    for (const char* p = sentence + 1; *p && *p != '*'; p++) ck ^= *p;
    sprintf(out2, "%02X", ck);
}

static void sendPQTM(HardwareSerial& port, const char* cmd) {
    char checksum[4];
    gnss_checksum(cmd, checksum);
    port.printf("%s*%s\r\n", cmd, checksum);
    delay(150);
}

// ── GSA message-rate read-back (READ-only) ────────────────────────────────────
// Confirms $PQTMCFGMSGRATE,W,GNGSA,1 actually stuck. Added because it DIDN'T:
// real flown data (2026-07-22) showed GSA still cadenced at ~1 Hz despite this
// exact write, because CFGSYS — sent AFTER it at the time — reverted it via an
// internal restart before PQTMSAVEPAR captured the state. Reordering (above)
// is the fix; this read-back is the reason we don't have to trust that blindly
// a second time.
//   Reply: $PQTMCFGMSGRATE,OK,GNGSA,<rate>[,...]
static int gnss_readAndLogGsaRate(HardwareSerial& port) {
    while (port.available()) port.read();
    sendPQTM(port, "$PQTMCFGMSGRATE,R,GNGSA");
    char line[96]; int idx = 0; uint32_t t0 = millis();
    while (millis() - t0 < 700) {
        while (port.available()) {
            char c = port.read();
            if (c == '$') idx = 0;
            if (c == '\n' || c == '\r') {
                line[idx] = '\0';
                if (strncmp(line, "$PQTMCFGMSGRATE,OK,GNGSA,", 25) == 0) {
                    int rate = atoi(line + 25);
                    Serial.printf("📡 LG290P GSA rate = %d%s\n", rate,
                                  rate == 1 ? "  ✅ 20 Hz (matches config)"
                                            : "  ⚠️ NOT 20 Hz — reverted (order/restart bug)");
                    return rate;
                }
                // Matches the ERROR-branch convention in gnss_readAndLogRtkHold: report
                // a rejection immediately and distinctly, don't let it masquerade as a
                // generic timeout after burning the rest of the 700 ms window.
                if (strncmp(line, "$PQTMCFGMSGRATE,ERROR", 21) == 0) {
                    Serial.printf("⚠️ LG290P rejected GSA-rate query (%s)\n", line);
                    return -1;
                }
                idx = 0;
            } else if (idx < (int)sizeof(line) - 1) {
                line[idx++] = c;
            }
        }
        delay(5);
    }
    Serial.println("📡 LG290P GSA-rate readback: no response");
    return -1;
}

// ── Constellation read-back (READ-only) ───────────────────────────────────────
// Confirms $PQTMCFGSYS,W,1,1,1,1,1,1 actually stuck. Same rationale as the GSA
// check above — this setting sits right next to the one that silently reverted,
// so it gets the same "never trust a write blindly" treatment.
//   Reply: $PQTMCFGSYS,OK,<GPS>,<GLONASS>,<Galileo>,<BDS>,<QZSS>,<NavIC>
static void gnss_readAndLogConstellations(HardwareSerial& port) {
    while (port.available()) port.read();
    sendPQTM(port, "$PQTMCFGSYS,R");
    char line[96]; int idx = 0; uint32_t t0 = millis();
    while (millis() - t0 < 700) {
        while (port.available()) {
            char c = port.read();
            if (c == '$') idx = 0;
            if (c == '\n' || c == '\r') {
                line[idx] = '\0';
                if (strncmp(line, "$PQTMCFGSYS,OK,", 15) == 0) {
                    // Stop at '*': the reply ends "...,1,1*CS" like every PQTM
                    // sentence, and a checksum hex digit can itself be '1' —
                    // counting through it would misreport the total roughly 1
                    // time in 8 (whichever hex digit lands there). Truncating
                    // at the checksum delimiter first is what keeps this count
                    // honest; a bare '1'-scan over the untrimmed line is not.
                    char* star = strchr(line + 15, '*');
                    if (star) *star = '\0';
                    int on = 0;
                    for (const char* p = line + 15; *p; p++) if (*p == '1') on++;
                    Serial.printf("📡 LG290P constellations = %s%s\n", line + 15,
                                  on == 6 ? "  ✅ all 6" : "  ⚠️ NOT all 6 — reverted");
                    return;
                }
                if (strncmp(line, "$PQTMCFGSYS,ERROR", 17) == 0) {
                    Serial.printf("⚠️ LG290P rejected constellation query (%s)\n", line);
                    return;
                }
                idx = 0;
            } else if (idx < (int)sizeof(line) - 1) {
                line[idx++] = c;
            }
        }
        delay(5);
    }
    Serial.println("📡 LG290P constellation readback: no response");
}

// ── Fix-rate readback (READ-only — safe per the hot-start contract) ───────────
// Asks the receiver what fix interval it is ACTUALLY running and prints it, so
// "GPS stuck at 10 Hz" is never a guess again. If this ever prints an interval
// that does NOT match GNSS_RATE_MS, the rate write did not stick (escalate);
// if it matches but the counted gnssHz is still low, the problem is downstream
// (NMEA timestamp resolution / epoch counting), not the receiver. Returns the
// interval in ms, or -1 on no response.
static int gnss_readAndLogFixRate(HardwareSerial& port) {
    while (port.available()) port.read();
    sendPQTM(port, "$PQTMCFGFIXRATE,R");
    char line[96]; int idx = 0; uint32_t t0 = millis();
    while (millis() - t0 < 700) {
        while (port.available()) {
            char c = port.read();
            if (c == '$') idx = 0;
            if (c == '\n' || c == '\r') {
                line[idx] = '\0';
                if (strncmp(line, "$PQTMCFGFIXRATE,OK,", 19) == 0) {
                    int ms = atoi(line + 19);
                    Serial.printf("📡 LG290P actual fix rate = %d ms (%.0f Hz)%s\n",
                                  ms, ms > 0 ? 1000.0f / ms : 0.0f,
                                  ms == GNSS_RATE_MS ? "  ✅ matches config"
                                                     : "  ⚠️ does NOT match config");
                    return ms;
                }
                idx = 0;
            } else if (idx < (int)sizeof(line) - 1) {
                line[idx++] = c;
            }
        }
        delay(5);
    }
    Serial.println("📡 LG290P fix-rate readback: no response");
    return -1;
}

// ── RTK HOLD read-back (READ-only — safe per the hot-start contract) ──────────
// Confirms the LG290P accepted the RTK HOLD <Timeout> written in configureLG290P.
//   Reply: $PQTMCFGRTK,OK,<DiffMode>,<RelMode>,<Timeout>*CS
// A firmware too old for the 3rd field either answers with only two fields
// ($PQTMCFGRTK,OK,1,1) or rejects the write with $PQTMCFGRTK,ERROR — both are
// surfaced here so an unsupported build can never silently no-op. Returns the
// reported hold seconds, or -1 on error / missing field / no reply.
static int gnss_readAndLogRtkHold(HardwareSerial& port) {
    while (port.available()) port.read();
    sendPQTM(port, "$PQTMCFGRTK,R");
    char line[96]; int idx = 0; uint32_t t0 = millis();
    while (millis() - t0 < 700) {
        while (port.available()) {
            char c = port.read();
            if (c == '$') idx = 0;
            if (c == '\n' || c == '\r') {
                line[idx] = '\0';
                if (strncmp(line, "$PQTMCFGRTK,OK,", 15) == 0) {
                    int diff = -1, rel = -1, to = -1;             // <DiffMode>,<RelMode>,<Timeout>
                    sscanf(line + 15, "%d,%d,%d", &diff, &rel, &to);
                    if (to >= 0) {
                        Serial.printf("📡 LG290P RTK HOLD = %d s (DiffMode=%d RelMode=%d)%s\n",
                                      to, diff, rel,
                                      to == RTK_HOLD_TIMEOUT_S ? "  ✅ matches config"
                                                               : "  ⚠️ does NOT match config");
                        return to;
                    }
                    Serial.printf("⚠️ LG290P RTK read-back has no <Timeout> field (%s) — the "
                                  "flashed LG290P firmware predates RTK HOLD; update it to enable.\n", line);
                    return -1;
                }
                if (strncmp(line, "$PQTMCFGRTK,ERROR", 17) == 0) {
                    Serial.printf("⚠️ LG290P rejected RTK HOLD (%s) — firmware likely predates the "
                                  "$PQTMCFGRTK <Timeout> field; update the LG290P to enable it.\n", line);
                    return -1;
                }
                idx = 0;
            } else if (idx < (int)sizeof(line) - 1) {
                line[idx++] = c;
            }
        }
        delay(5);
    }
    Serial.println("📡 LG290P RTK HOLD read-back: no response");
    return -1;
}

// ── PPP write + read-back ─────────────────────────────────────────────────────
// Write:  $PQTMCFGPPP,W,<Mode>,<Datum>,<Timeout>,<HorStd>,<VerStd>
//   <Mode> 0=disable 1=B2b 2=E6 HAS | <Datum> 1=WGS84 2=PPP original 3=CGCS2000
//   <Timeout> [90,180] s (default 120) | <HorStd>/<VerStd> convergence thresholds, m
// (Quectel LG290P(03)/LGx80P(03) PPP Application Note V1.0.0, §2.1.1 — the same command
// and field order already field-proven on the RCX1 caster.)
// This is a PQTMCFG WRITE, so per the hot-start contract it may restart the nav engine.
// It is therefore only issued from configureLG290P() (which already pays that cost) or
// from the deferred web-toggle path in gnss_loop() — NEVER speculatively at boot.
static void sendPppConfig(HardwareSerial& port, uint8_t mode) {
    char cmd[64];
    if (mode == 0) {
        snprintf(cmd, sizeof(cmd), "$PQTMCFGPPP,W,0");   // disable — other fields ignored
    } else {
        snprintf(cmd, sizeof(cmd), "$PQTMCFGPPP,W,%u,%u,%u,%.2f,%.2f",
                 (unsigned)mode, (unsigned)GNSS_PPP_DATUM, (unsigned)GNSS_PPP_TIMEOUT_S,
                 (double)GNSS_PPP_HOR_STD_M, (double)GNSS_PPP_VER_STD_M);
    }
    sendPQTM(port, cmd);
}

// Read-back (READ-only — safe per the hot-start contract).
//   Reply: $PQTMCFGPPP,OK,<Mode>,<Datum>,<Timeout>,<HorStd>,<VerStd>   (<Mode> is HEX, e.g. 02)
//   Error: $PQTMCFGPPP,ERROR,<ErrCode>   1=invalid params 2=exec failed 3=UNSUPPORTED COMMAND
// ERROR,3 is the decisive one: it means the flashed LG290P firmware predates PPP (this is
// exactly what the caster reported on fw ...A06S before the v2.01 upgrade). Surfacing it
// here means an unsupported build can never silently no-op. Returns the reported mode, or
// -1 on error / no reply.
static int gnss_readAndLogPpp(HardwareSerial& port) {
    while (port.available()) port.read();
    sendPQTM(port, "$PQTMCFGPPP,R");
    char line[96]; int idx = 0; uint32_t t0 = millis();
    while (millis() - t0 < 700) {
        while (port.available()) {
            char c = port.read();
            if (c == '$') idx = 0;
            if (c == '\n' || c == '\r') {
                line[idx] = '\0';
                if (strncmp(line, "$PQTMCFGPPP,OK,", 15) == 0) {
                    // <Mode> comes back hex-formatted ("01"/"02") per the app note example.
                    int mode = (int)strtol(line + 15, nullptr, 16);
                    Serial.printf("📡 LG290P PPP = %s%s\n",
                                  mode == 2 ? "E6 HAS (Galileo)" :
                                  mode == 1 ? "B2b (BeiDou)"     : "OFF",
                                  mode == (int)s_pppMode ? "  ✅ matches config"
                                                         : "  ⚠️ does NOT match config");
                    return mode;
                }
                if (strncmp(line, "$PQTMCFGPPP,ERROR", 17) == 0) {
                    Serial.printf("⚠️ LG290P rejected PPP (%s) — ERROR,3 means the flashed "
                                  "LG290P firmware has no PPP command set; update it to v2.01+.\n", line);
                    return -1;
                }
                idx = 0;
            } else if (idx < (int)sizeof(line) - 1) {
                line[idx++] = c;
            }
        }
        delay(5);
    }
    Serial.println("📡 LG290P PPP read-back: no response");
    return -1;
}

// ── Generic config read-back (READ-only — safe under the hot-start contract) ──
// Sends one $PQTM...,R query and prints the raw reply line, so the boot log shows exactly
// what the module is running rather than what we assume it defaults to. Used to diagnose
// the standalone-accuracy question: which SBAS systems are active (PQTMCFGSBAS), which
// signal bands feed the position engine (PQTMCFGSIGNAL), and the constellation set
// (PQTMCFGCNST). Per the protocol spec these are all Set/Get commands and the R form
// restarts nothing. `okPrefix` is the expected "$PQTM...,OK" so we can label the echo.
static void gnss_readAndLogCfg(HardwareSerial& port, const char* query,
                               const char* okPrefix, const char* label) {
    while (port.available()) port.read();
    sendPQTM(port, query);
    char line[96]; int idx = 0; uint32_t t0 = millis();
    size_t okLen = strlen(okPrefix);
    while (millis() - t0 < 600) {
        while (port.available()) {
            char c = port.read();
            if (c == '$') idx = 0;
            if (c == '\n' || c == '\r') {
                line[idx] = '\0';
                if (strncmp(line, okPrefix, okLen) == 0) {
                    Serial.printf("📡 %s: %s\n", label, line);
                    return;
                }
                if (strstr(line, ",ERROR")) {           // any PQTM error reply
                    Serial.printf("⚠️ %s read-back error: %s\n", label, line);
                    return;
                }
                idx = 0;
            } else if (idx < (int)sizeof(line) - 1) {
                line[idx++] = c;
            }
        }
        delay(5);
    }
    Serial.printf("📡 %s read-back: no response\n", label);
}

/* ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║  HOT-START CONTRACT — READ THIS BEFORE TOUCHING ANY GNSS BOOT CODE         ║
 * ╠═══════════════════════════════════════════════════════════════════════════╣
 * ║  The LG290P hot-starts (fix in ~1-2 s) ONLY when its navigation context    ║
 * ║  (RTC time + last position + fresh ephemeris, kept alive by the board's    ║
 * ║  backup supply) survives AND nothing restarts the nav engine at boot.      ║
 * ║  This has regressed multiple times. The rules, learned the hard way:       ║
 * ║                                                                            ║
 * ║   1. NEVER send PQTMSRR / PQTMCOLD / PQTMWARM / PQTMHOT /                  ║
 * ║      PQTMRESTOREPAR / PQTMGNSSSTOP at boot. All of them destroy or         ║
 * ║      degrade the retained fix context.                                     ║
 * ║   2. NEVER send any "PQTMCFG...,W" WRITE unless the configuration has      ║
 * ║      genuinely changed. Writes like PQTMCFGSYS / PQTMCFGRCVRMODE restart   ║
 * ║      the nav engine internally → the retained fix is discarded → TTFF      ║
 * ║      degrades from ~2 s (hot) to ~30 s (warm). One stray write = broken    ║
 * ║      hot start, and it looks "random" because it depends on NVS state.     ║
 * ║   3. "PQTMCFG...,R" READ queries are SAFE — they do not restart anything.  ║
 * ║      That is why read-only rate checks (below) may run at boot.     ║
 * ║   4. The skip path in gnss_init() must send ZERO config bytes. Do not add  ║
 * ║      "harmless" writes, "just in case" saves, or unconditional restarts.   ║
 * ║   5. configureLG290P() may only be reached when (a) the receiver probe     ║
 * ║      failed, (b) GNSS_CONFIG_VERSION was deliberately bumped in config.h,  ║
 * ║      or (c) the boot rate measurement PROVED the module running slower     ║
 * ║      than configured — evidence something external rewrote it (the known   ║
 * ║      case: an RCX1 base-firmware cross-flash leaves base mode, capping     ║
 * ║      output at 1 Hz). It ends with PQTMSAVEPAR and costs one warm start —  ║
 * ║      expected on a config change or a proven-wrong module, never on a      ║
 * ║      routine boot.                                                         ║
 * ║                                                                            ║
 * ║  If the symptom "warm starts but not immediate" ever returns: someone      ║
 * ║  violated one of these rules. Diff THIS file and config.h first.           ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝ */

// ── LG290P one-time configuration ────────────────────────────────────────────
// ⚠️  Sends PQTMCFG writes + PQTMSAVEPAR → restarts the nav engine → warm start.
//     Only call when configuration has actually changed (see contract above).
static void configureLG290P(HardwareSerial& gpsSerial) {
    Serial.println("⚙️  Configuring LG290P...");

    // ── ORDER-CRITICAL: restart-triggering commands go FIRST ──────────────────
    // PQTMCFGRCVRMODE and PQTMCFGSYS each trigger an internal nav-engine restart
    // (see the fix-rate note further down — that restart is why fix rate STILL
    // needs a full PQTMSRR on top of it). sendPQTM() is fire-and-forget: it
    // writes, waits a flat 150 ms, and checks neither an ack nor whether a
    // restart has settled. So any command issued BEFORE a restart-triggering one
    // can be reverted by that restart before PQTMSAVEPAR captures the intended
    // state, and the revert is silent — the command was accepted, it just isn't
    // what ends up saved.
    //   Measured, 2026-07-22 commute: with CFGSYS sent after the message-rate
    //   commands, GSA was requested at 20 Hz (rate 1) yet held one value for
    //   15-20 consecutive rows — ~1 Hz on the wire. CFGPROT (RTCM+NMEA accept)
    //   is the same class of setting, and a silent revert there would stop the
    //   module accepting RTCM even on a session where NTRIP forwards it
    //   perfectly, which is far harder to notice than a wrong message rate.
    // Therefore: restart-triggering commands go first and are allowed to settle,
    // and everything else is configured after, so it is the last word before
    // PQTMSAVEPAR. Read-back verification of the GSA rate and the constellation
    // set follows below so this class of silent revert cannot hide again.
    sendPQTM(gpsSerial, "$PQTMCFGRCVRMODE,W,1");           // rover mode
    // All six constellations the LG290P supports. QZSS (field 5) and NavIC
    // (field 6) are regional — QZSS over ~135°E, NavIC over ~55-110°E — so from
    // most of the Americas/Europe they are below the horizon and contribute zero
    // observables at zero cost (no sats tracked → no GSV entries, no CPU). They
    // are enabled unconditionally rather than gated on locale because this is
    // open source: a user in Japan, Australia, or India gets the accuracy benefit
    // with no config change, and everyone else pays nothing. Never disable a
    // constellation for being small — the engine already ignores what it can't see.
    sendPQTM(gpsSerial, "$PQTMCFGSYS,W,1,1,1,1,1,1");     // GPS+GLO+GAL+BDS+QZSS+NavIC
    // Extra settle time on top of sendPQTM's normal 150 ms: this is the one
    // spot we KNOW triggers an internal restart, and we have no ack to wait on.
    // 400 ms is a conservative bench estimate, not a datasheet figure — if any
    // later command in this function still turns out not to have stuck (check
    // the read-backs below and the NEW $PQTMCFGSYS,R / $PQTMCFGMSGRATE,R
    // verification), lengthen this before adding more settle delays elsewhere.
    delay(400);

    // Navigation dynamics model — module ships with mode 11 (mower). 0 = Normal:
    // no application-specific motion constraints on the nav filter. A mower model
    // assumes low speed and gentle dynamics — exactly wrong for autocross, and a
    // plausible contributor to velocity-channel lag under high dynamics. Adopted
    // (like the fix rate) only after PQTMSAVEPAR + the engine restart below.
    sendPQTM(gpsSerial, "$PQTMCFGNAVMODE,W,0");
    char navCmd[64];
    snprintf(navCmd, sizeof(navCmd), "$PQTMCFGFIXRATE,W,%d", GNSS_RATE_MS);
    sendPQTM(gpsSerial, navCmd);
    sendPQTM(gpsSerial, "$PQTMCFGPROT,W,1,1,00000005,00000005"); // RTCM+NMEA
    sendPQTM(gpsSerial, "$PQTMCFGMSGRATE,W,PQTMEPE,1,2");        // EPE v2
    sendPQTM(gpsSerial, "$PQTMCFGMSGRATE,W,GNGSV,20");            // GSV 1 Hz
    sendPQTM(gpsSerial, "$PQTMCFGMSGRATE,W,GPGSV,20");
    sendPQTM(gpsSerial, "$PQTMCFGMSGRATE,W,GLGSV,20");
    sendPQTM(gpsSerial, "$PQTMCFGMSGRATE,W,GAGSV,20");
    sendPQTM(gpsSerial, "$PQTMCFGMSGRATE,W,GBGSV,20");
    // ── Message-set trim (bandwidth lives or dies inside the 50 ms epoch) ────
    // VTG off: nothing consumed it — TinyGPS++ has no VTG handler, and the only
    //   value it carries (ground speed/course) already comes from RMC; the mph
    //   conversion at the SD/BLE write sites is the "math" that replaces it.
    // GLL off: lat/lon/time only — a strict subset of GGA. Pure dead weight.
    // GSA → 20 Hz: DOPs + used-sat IDs per-epoch, so an occultation/multipath
    //   event can be correlated to the exact epoch whose geometry degraded.
    //   Costs ~420 B/epoch (~8.4 kB/s), the largest single per-epoch consumer.
    sendPQTM(gpsSerial, "$PQTMCFGMSGRATE,W,GNVTG,0");
    sendPQTM(gpsSerial, "$PQTMCFGMSGRATE,W,GNGLL,0");
    sendPQTM(gpsSerial, "$PQTMCFGMSGRATE,W,GNGSA,1");            // GSA 20 Hz
    sendPQTM(gpsSerial, "$PQTMCFGNMEADP,W,3,8,3,2,3,2");         // 8 decimal places
    // RTK HOLD — extend <Timeout> (max differential age) so a held, model-predicted
    // RTK fix survives brief RTCM/NTRIP dropouts instead of dropping to FLOAT/3D at
    // the 120 s default. DiffMode=1 (Auto) RelMode=1 (Absolute) match the module
    // defaults, so only the hold time changes. See RTK_HOLD_TIMEOUT_S in config.h.
    { char rtkCmd[48];
      snprintf(rtkCmd, sizeof(rtkCmd), "$PQTMCFGRTK,W,1,1,%d", RTK_HOLD_TIMEOUT_S);
      sendPQTM(gpsSerial, rtkCmd); }
    // Multipath/occlusion masks — elevation cutoff + C/N0 floor exclude the low, weak,
    // and reflected signals that dominate at fenced/grandstand venues. Values come from
    // NVS (loaded in gnss_init) or the config.h defaults; tunable live via the web page.
    { char mb[48];
      snprintf(mb, sizeof(mb), "$PQTMCFGELETHD,W,%.1f", s_eleMask); sendPQTM(gpsSerial, mb);
      snprintf(mb, sizeof(mb), "$PQTMCFGCNRTHD,W,%.1f", s_cnrMask); sendPQTM(gpsSerial, mb); }
    // PPP — the fallback BELOW RTK, above vanilla 3D GPS. Enabling it never costs an RTK
    // fix: the engine still prefers RTK whenever RTCM is flowing. Written unconditionally
    // (including mode 0) so the module's saved PPP state is always deterministic after a
    // config change, exactly like the caster's base config does it.
    sendPppConfig(gpsSerial, s_pppMode);
#if defined(PPP_NAV_DEBUG) && PPP_NAV_DEBUG
    // BENCH DIAGNOSTIC ONLY (fix rate UNCHANGED — still GNSS_RATE_MS / 20 Hz). Turns on the
    // module's PPP navigation output so the serial log can show whether the PPP engine is
    // making ANY progress at 20 Hz. Field reports (SparkFun, Feb 2026) only ever converged
    // HAS at 1 Hz — this test asks the one question those reports leave open: does the engine
    // produce a rising SolType at 20 Hz, or sit at 0? handlePqtmPppNav() prints the answer.
    // MsgVer 1, rate 1 (every fix). Rate 1 at 20 Hz = 20 msgs/s; fine for a bench USB session.
    sendPQTM(gpsSerial, "$PQTMCFGMSGRATE,W,PQTMPPPNAV,1,1");
    Serial.println("🔬 PPP_NAV_DEBUG: PQTMPPPNAV output ENABLED (bench diagnostic, 20 Hz "
                   "UNCHANGED). Watch for 'PPPNAV SolType' climbing 0→…→7.");
#endif
    sendPQTM(gpsSerial, "$PQTMSAVEPAR");
    delay(500);
    Serial.printf("   Fix rate: %dHz | NAV Normal | RTCM+NMEA | PQTMEPE | VTG/GLL off | GSA 20Hz "
                  "| 6 constellations | RTK HOLD %ds "
                  "| ele %.1f° | C/N0 %.1f dB-Hz | PPP %s\n",
                  1000 / GNSS_RATE_MS, RTK_HOLD_TIMEOUT_S, s_eleMask, s_cnrMask,
                  s_pppMode == 2 ? "E6 HAS" : s_pppMode == 1 ? "B2b" : "off");

    // Confirm the writes actually saved BEFORE we reset (reads are harmless).
    gnss_readAndLogFixRate(gpsSerial);
    gnss_readAndLogRtkHold(gpsSerial);   // confirms RTK HOLD took — flags old fw if not
    gnss_readAndLogPpp(gpsSerial);       // confirms PPP took — flags pre-v2.01 fw if not
    gnss_readAndLogGsaRate(gpsSerial);        // confirms the reordering fix actually worked
    gnss_readAndLogConstellations(gpsSerial); // confirms all 6 constellations actually stuck

    // ── APPLY the new fix rate by rebooting the module ───────────────────────
    //   *** PROTECTIVE NOTE — why a reset is REQUIRED here (and only here) ***
    //   On the LG290P the fix-rate change is NOT adopted by the running engine
    //   when written — not even after PQTMSAVEPAR or the internal nav-engine
    //   restart that PQTMCFGSYS/PQTMCFGRCVRMODE trigger. It is only adopted
    //   after a FULL module reboot (field/forum-confirmed: 50 ms written but
    //   the receiver kept emitting 10 Hz until a power cycle). That is exactly
    //   the symptom we hit: BLE ran at 20 Hz but GPS stayed at 10 Hz.
    //   PQTMSRR is the software power-cycle. It is forbidden on the BOOT/skip
    //   path (rule #1 of the hot-start contract — it discards the retained fix
    //   context), but configureLG290P already costs a (re)start by design, so
    //   issuing it HERE, once per config change, is correct and necessary.
    //   The module re-reads the just-saved flash on reboot → comes back at 20 Hz.
    sendPQTM(gpsSerial, "$PQTMSRR");
    delay(1500);   // allow the module to reboot before NMEA parsing resumes
    Serial.println("   Module reset (PQTMSRR) to apply fix rate — warm-starting once.");
}

// ── CSV field extractor ───────────────────────────────────────────────────────
static bool csvField(const char* body, int target, char* out, size_t outLen) {
    int field = 0;
    const char* start = body;
    for (const char* p = body; ; p++) {
        if (*p == ',' || *p == '\0' || *p == '*') {
            if (field == target) {
                size_t n = (size_t)(p - start);
                if (n >= outLen) n = outLen - 1;
                memcpy(out, start, n);
                out[n] = '\0';
                return true;
            }
            if (*p == '\0' || *p == '*') return false;
            field++;
            start = p + 1;
        }
    }
}

// ── NMEA checksum validator ───────────────────────────────────────────────────
static bool nmeaChecksumValid(const char* line) {
    if (!line || line[0] != '$') return false;
    const char* star = strchr(line, '*');
    if (!star || star - line < 2) return false;
    uint8_t ck = 0;
    for (const char* p = line + 1; p < star; p++) ck ^= (uint8_t)*p;
    char got[3] = { star[1], star[2], 0 };
    return ck == (uint8_t)strtoul(got, nullptr, 16);
}

// ── PQTMEPE proprietary accuracy message ─────────────────────────────────────
// LG290P PQTMEPE sentence formats (fields 0-indexed after the leading '$'):
//   v1: $PQTMEPE,<time>,<hError>,<vError>*CS
//       field 0=PQTMEPE  1=time  2=hError  3=vError
//   v2: $PQTMEPE,<time>,<hPL>,<vPL>,<hError>,<vError>,<hVelErr>*CS
//       field 0=PQTMEPE  1=time  2=hPL  3=vPL  4=hError  5=vError  6=hVelErr
//   isV2 detected by presence of field[6] (hVelErr, only in v2).
//   We configure "PQTMEPE,1,2" → v2 always, but handle v1 for robustness.
//
// PAST BUG (June 2026): fHoriz and fVert were swapped for v2 (used 5,4 instead
// of 4,5), AND v1 used field 4 (empty/checksum) for hAcc — so hAcc was never
// set in v1, and in v2 vError was stored as hAccM and hError as vAccM. This
// caused RTK float to display ~15 m hAcc (the vError value) instead of ~0.3 m.
static void handlePQTMEPE(const char* line) {
    if (!nmeaChecksumValid(line)) return;

    // Detect v1 vs v2 by probing field 6 (hVelErr, only present in v2)
    char probe[4] = "";
    bool isV2 = csvField(line + 1, 6, probe, sizeof(probe)) && probe[0] != '\0';

    // Correct field indices per format above
    int fHoriz = isV2 ? 4 : 2;   // hError
    int fVert  = isV2 ? 5 : 3;   // vError

    char eH[16]="", eV[16]="";
    if (csvField(line+1, fHoriz, eH, sizeof(eH)) &&
        csvField(line+1, fVert,  eV, sizeof(eV))) {
        double h = atof(eH);
        double v = atof(eV);
        if (h > 0.0 && h < 200.0 && v > 0.0 && v < 200.0) {
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
                gps.hAccM    = h;
                gps.vAccM    = v;
                gps.epeValid  = true;
                gps.epeMillis = millis();
                xSemaphoreGive(dataMutex);
            }
            static uint32_t lastLog = 0;
            if (millis() - lastLog > 5000) {
                lastLog = millis();
#if 0  // GPS accuracy heartbeat (every 5s) — re-enable to troubleshoot GPS
                Serial.printf("📍 PQTMEPE(%s): hAcc=%.3fm vAcc=%.3fm\n",
                              isV2 ? "v2" : "v1", h, v);
#endif
            }
        }
    }
}

// ── PQTMPPPNAV — PPP navigation output (BENCH DIAGNOSTIC) ─────────────────────
// Field layout, verified against Quectel LG290P(03)/LGx80P(03) PPP Application Note V1.0.0
// §2.1.2 (comma-indexed after the leading '$', so field 0 = "PQTMPPPNAV"):
//   1 MsgVer  2 TimeStatus  3 TimeRef  4 UTC  5 Date  6 TOW  7 WN  8 LeapSec  9 DatumId
//   10 Reserved  11 SolType  12 Reserved  13 Lat  14 Lon  15 Alt  16 Sep ...
//   ... 24 DiffID  25 DiffAge ... 27 SatView  28 SatUsed ...
// The ONLY field this diagnostic cares about is <SolType> (field 11) — it is the direct
// readout of the PPP engine's state. Per the app note's operation guide the PPP flow climbs
// SolType until a converged PPP solution (the same 0→…→converged progression rftop/bamarcant
// watched). If SolType sits at 0 for 20 min with E6 tracked strong, the engine is idle at
// 20 Hz — the answer we're after. This parser is compiled out entirely unless PPP_NAV_DEBUG.
#if defined(PPP_NAV_DEBUG) && PPP_NAV_DEBUG
static void handlePqtmPppNav(const char* line) {
    // Split on commas into up to 12 fields — we only need through SolType (field 11).
    char buf[160];
    strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[13] = {0}; int nf = 0;
    for (char* p = strtok(buf, ","); p && nf < 13; p = strtok(nullptr, ",")) f[nf++] = p;
    if (nf < 12) return;                       // not enough fields to hold SolType
    int solType = atoi(f[11]);
    int satUsed = (nf > 28 && f[28]) ? atoi(f[28]) : -1;  // best-effort, may be truncated
    static int lastSol = -999;
    static uint32_t lastPrint = 0;
    // Print on every SolType change, and at most once/2 s otherwise so 20 Hz doesn't flood.
    if (solType != lastSol || millis() - lastPrint > 2000) {
        Serial.printf("🔬 PPPNAV SolType=%d%s (satUsed=%d)\n", solType,
                      solType >= 7 ? "  ← CONVERGED PPP" :
                      solType >  0 ? "  (converging…)"   : "  (engine idle / no PPP)",
                      satUsed);
        lastSol = solType; lastPrint = millis();
    }
}
#endif

// ── $GNGSA — DOPs + used-satellite IDs (1 Hz; one sentence per system) ───────
// $GNGSA,<Mode>,<FixStatus>,<ID1>..<ID12>,<PDOP>,<HDOP>,<VDOP>,<SystemID>*CS
// The module emits one GSA per GNSS system in each epoch's group (spec §2.2.4:
// only the first 12 used sats per system appear), and the three DOPs are
// identical across the group. The IDs are accumulated across the group into a
// single CSV-safe string — "G:05 09 15/R:68 79" — no commas, so it drops into
// the GPS log as one column. System letters per NMEA 4.11 <SystemID>:
// 1=GPS(G) 2=GLONASS(R) 3=Galileo(E) 4=BeiDou(B) 5=QZSS(Q) 6=NavIC(I).
// Group detection is by <SystemID> ORDER, not by arrival gap: systems emit in
// ascending ID order within one epoch's group, so an ID that does not exceed the
// previous one means a new group started. This is rate-agnostic — a gap-based
// test would break at 20 Hz, where groups arrive every 50 ms and would never be
// seen as separate. (Spec §2.2.4 truncates at 12 IDs per system rather than
// emitting a second sentence, so IDs never repeat inside one group.) A stale-gap
// fallback still clears the stage if GSA stops and later resumes.
// Raw GSA does NOT go to the sat log — mixing GSA rows into the
// GSV file broke its uniform schema (2026-07-20: "sat log looks like garbage").
static void handleGNGSA(const char* line) {
    if (!nmeaChecksumValid(line)) return;

    char f[10];
    int  sysId = 0;
    if (csvField(line + 1, 18, f, sizeof(f)) && f[0]) sysId = atoi(f);  // atoi stops at '*'

    static char     stage[sizeof(gps.gsaSats)] = "";   // sized to the struct field
    static uint32_t lastGsaMs   = 0;
    static int      lastSysId   = 0;
    const uint32_t  now = millis();
    if (sysId <= lastSysId || (now - lastGsaMs) > 500) stage[0] = '\0';   // new group
    lastGsaMs = now;
    lastSysId = sysId;

    static const char SYS[] = "?GREBQI";
    const char sysC = (sysId >= 1 && sysId <= 6) ? SYS[sysId] : '?';

    size_t n = strlen(stage);
    int    w = snprintf(stage + n, sizeof(stage) - n, "%s%c:", n ? "/" : "", sysC);
    if (w > 0) n = (n + (size_t)w < sizeof(stage)) ? n + (size_t)w : sizeof(stage) - 1;
    bool first = true;
    for (int i = 3; i <= 14; i++) {                    // the 12 SatID fields
        if (!(csvField(line + 1, i, f, sizeof(f)) && f[0])) continue;
        if (n >= sizeof(stage) - 4) break;             // truncate cleanly, never overflow
        w = snprintf(stage + n, sizeof(stage) - n, "%s%s", first ? "" : " ", f);
        if (w > 0) n = (n + (size_t)w < sizeof(stage)) ? n + (size_t)w : sizeof(stage) - 1;
        first = false;
    }

    char pd[10] = "", hd[10] = "", vd[10] = "";
    csvField(line + 1, 15, pd, sizeof(pd));
    csvField(line + 1, 16, hd, sizeof(hd));
    csvField(line + 1, 17, vd, sizeof(vd));

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
        if (pd[0]) gps.pdop = atof(pd);
        if (hd[0]) gps.hdop = atof(hd);
        if (vd[0]) gps.vdop = atof(vd);
        strlcpy(gps.gsaSats, stage, sizeof(gps.gsaSats));
        gps.gsaMillis = now;
        xSemaphoreGive(dataMutex);
    }
}

// Set by handleRawNmeaLine when a CHECKSUM-VALID RMC line has just completed;
// consumed in gnss_loop's mutex block right after TinyGPS++ commits that same
// sentence (raw handler runs on the terminator char BEFORE encode() sees it, so
// the ordering is guaranteed). A corrupt RMC never sets it → the epoch watchdog
// publishes that row flagged instead of a wrong seq-bump crediting stale speed.
static bool s_rmcLineDone = false;

// ── Raw NMEA line dispatcher ──────────────────────────────────────────────────
static void handleRawNmeaLine(const char* line) {
    if (!line || line[0] != '$') return;

    // ── WIRE-TRUTH COUNTERS ──────────────────────────────────────────────────
    // PROTECTIVE NOTE: these count sentences as they arrive off the UART ring,
    // BEFORE TinyGPS++ and BEFORE any epoch/snapshot logic. This is the ONLY
    // measurement that separates "the RMC never arrived" (module/UART loss)
    // from "the RMC arrived but nobody sampled it" (the snapshot race). Do not
    // move this below the tinygps.encode() feed, and do not gate it on a mutex.
    // Expected at 20 Hz over the 5 s window: GGA=100 RMC=100 GSA=~30 GSV=~95
    // ckfail=0. "oth" counts PQTM traffic (EPE at 20 Hz ⇒ ~100) — that's normal.
    {
        static uint32_t nGGA=0, nRMC=0, nGSA=0, nGSV=0, nOth=0, nCk=0, lastPr=0;
        // ── SKY HEALTH ───────────────────────────────────────────────────────
        // Counters alone cannot tell "no fix because indoors" from "no fix
        // because the antenna is dead" — both give GGA=100 ckfail=0. C/N0 does:
        //   nothing tracked          → antenna/LNA/cable fault (or no power to an active antenna)
        //   a few sats at 20-30 dB-Hz → real signal, indoor attenuation
        //   many at 40+              → open sky; a no-fix is then a config problem
        // Also prints the live masks, because a high C/N0 gate rejects most indoor
        // signal by design — a bench "failure" that is correct behaviour.
        static uint8_t  maxCno=0, nCnoUse=0, nCnoAny=0, lastQual=0, lastSats=0;
        static uint8_t  gsvMaxCno=0, gsvUse=0, gsvAny=0;
        const bool ckOk = nmeaChecksumValid(line);   // ckfail separates CORRUPTED
        if (!ckOk) nCk++;                            // sentences from MISSING ones
                                                     // (RMC count < GGA count)
        const char* typ = (line[1] && line[2]) ? line + 3 : "";
        if      (strncmp(typ, "GGA", 3) == 0) {
            nGGA++;
            char q[6]="", s[6]="";
            if (ckOk && csvField(line+1, 6, q, sizeof(q)) && q[0]) lastQual = (uint8_t)atoi(q);
            if (ckOk && csvField(line+1, 7, s, sizeof(s)) && s[0]) lastSats = (uint8_t)atoi(s);
        }
        else if (strncmp(typ, "RMC", 3) == 0) { nRMC++; if (ckOk) s_rmcLineDone = true; }
        else if (strncmp(typ, "GSA", 3) == 0) nGSA++;
        else if (strncmp(typ, "GSV", 3) == 0) {
            nGSV++;
            if (ckOk) {
                // GSV repeats {PRN,elev,azim,C/N0} — C/N0 at fields 7, 11, 15, 19.
                char c[6];
                for (int i = 7; i <= 19; i += 4) {
                    if (!(csvField(line+1, i, c, sizeof(c)) && c[0])) continue;
                    int v = atoi(c);                    // atoi stops at '*' on the last field
                    if (v <= 0) continue;
                    gsvAny++;
                    if (v > gsvMaxCno) gsvMaxCno = (uint8_t)v;
                    if (v >= (int)s_cnrMask) gsvUse++;   // would survive the C/N0 mask
                }
            }
        }
        else nOth++;
        if (millis() - lastPr >= 5000) {
            lastPr = millis();
            maxCno = gsvMaxCno; nCnoUse = gsvUse; nCnoAny = gsvAny;
            Serial.printf("📶 wire/5s: GGA=%lu RMC=%lu GSA=%lu GSV=%lu oth=%lu ckfail=%lu\n"
                          "   🛰️  sky: qual=%u sats=%u | C/N0 max=%u dB-Hz, tracked=%u, "
                          "≥%.0f dB-Hz mask=%u\n",
                          (unsigned long)nGGA, (unsigned long)nRMC, (unsigned long)nGSA,
                          (unsigned long)nGSV, (unsigned long)nOth, (unsigned long)nCk,
                          lastQual, lastSats, maxCno, nCnoAny, s_cnrMask, nCnoUse);
            nGGA = nRMC = nGSA = nGSV = nOth = nCk = 0;
            gsvMaxCno = gsvUse = gsvAny = 0;
        }
    }
    if (strncmp(line, "$PQTMEPE,", 9) == 0) {
        handlePQTMEPE(line);
        return;
    }
#if defined(PPP_NAV_DEBUG) && PPP_NAV_DEBUG
    if (strncmp(line, "$PQTMPPPNAV,", 12) == 0) {
        handlePqtmPppNav(line);
        return;
    }
#endif
    // $xxGSV (satellites-in-view, any talker: GP/GL/GA/GB/GN) → SD sat log.
    // This call IS the sat-log producer — sd_log.cpp only consumes its queue.
    // It was missing entirely once (sat_*.csv files were created but stayed
    // zero bytes forever); if sat logs go empty again, check this line first.
    // sdlog_push_sat() is cheap and self-gates on the logSat toggle + queue.
    if (line[1] && line[2] && strncmp(line + 3, "GSV", 3) == 0) {
        sdlog_push_sat(line);
    }
    // $GNGSA (1 Hz, one sentence per system) → parsed into gps.pdop/hdop/vdop +
    // gps.gsaSats for the GPS log columns. NOT pushed to the sat log: raw GSA
    // rows interleaved with GSV broke that file's uniform schema (2026-07-20).
    if (line[1] && line[2] && strncmp(line + 3, "GSA", 3) == 0) {
        handleGNGSA(line);
    }
}

// ── Measure the ACTUAL output rate (READ-only — preserves hot start) ─────────
// Listens to the NMEA stream and counts RMC sentences (one per fix epoch) to
// compute the rate the nav engine is REALLY running. This is the ground truth,
// and crucially it differs from PQTMCFGFIXRATE,R: a fix-rate write that has been
// saved but not yet ADOPTED reads back as "correct" while the engine still emits
// the old rate (adoption needs a reboot). We must measure to tell them apart.
// Same-epoch GN/GP duplicates are de-duped by ignoring RMCs <20 ms apart.
// Returns measured Hz, or 0 if too few sentences were seen (treated as unknown).
static float gnss_measureOutputRateHz(HardwareSerial& port) {
    while (port.available()) port.read();           // flush stale bytes
    // The base window resolves rates down to 2 Hz (three RMC epochs are needed
    // to compute a figure). If it closes with a stream present but too slow to
    // resolve — one or two epochs seen — the deadline extends once, so
    // genuinely slow output (a base-moded module is capped at 1 Hz) measures
    // as its true rate instead of collapsing to 0.0 "unknown". That matters
    // because 0.0 deliberately triggers NO repair (a module also reads 0.0
    // when it is merely still booting), so a slow module that always read as
    // unknown could never be repaired. A healthy 20 Hz stream exits at ten
    // epochs in well under the base window either way, and a module that
    // starts mid-window floods the extension with epochs and measures
    // accurately — the extension can never misread a fast module as slow.
    const uint32_t WINDOW_MS      = 1500;
    const uint32_t WINDOW_SLOW_MS = 6000;   // ≥3 epochs at 1 Hz, with margin
    uint32_t deadline = WINDOW_MS;
    uint32_t t0 = millis();
    uint32_t firstMs = 0, lastMs = 0, lastCountMs = 0;
    int count = 0;
    char hdr[6]; int hidx = 0; bool cap = false;

    while (millis() - t0 < deadline) {
        while (port.available()) {
            char c = (char)port.read();
            if (c == '$') { hidx = 0; cap = true; continue; }
            if (cap && hidx < 5) {
                hdr[hidx++] = c;
                if (hidx == 5) {
                    cap = false; hdr[5] = '\0';
                    if (strncmp(hdr + 2, "RMC", 3) == 0) {     // xxRMC = 1 per epoch
                        uint32_t now = millis();
                        if (count == 0 || now - lastCountMs >= 20) {   // de-dup
                            if (count == 0) firstMs = now;
                            lastMs = now; lastCountMs = now; count++;
                        }
                    }
                }
            }
        }
        if (count >= 10) break;          // enough to measure — stop early
        if (count >= 1 && count < 3) deadline = WINDOW_SLOW_MS;   // slow stream: extend
        delay(2);
    }

    float hz = (count >= 3 && lastMs > firstMs)
             ? (float)(count - 1) * 1000.0f / (float)(lastMs - firstMs) : 0.0f;
    Serial.printf("📡 LG290P measured output rate ≈ %.1f Hz (%d RMC epochs in window)\n",
                  hz, count);
    return hz;
}

// ── Public init ───────────────────────────────────────────────────────────────
void gnss_init(HardwareSerial& serial) {
    // Enlarge the UART RX buffer BEFORE begin(). Default is 256 bytes, which at
    // 460800 baud fills in ~5.5 ms — far too small if loop() ever stalls (e.g.
    // a display redraw or BLE meta burst). 4096 bytes buffers ~89 ms of data,
    // so transient stalls no longer drop NMEA/RTCM bytes.
    serial.setRxBufferSize(4096);
    serial.begin(460800, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(100);

    // ⚠️  HOT-START CRITICAL — see the contract banner at the top of this file.
    // Goal: send ZERO bytes and never reset when nothing needs changing, so the
    // LG290P hot-starts. The ONLY things that may disturb it are (a) a genuine
    // config-version change, or (b) the module MEASURABLY running at the wrong
    // fix rate. We decide using the ACTUAL output rate (measured by listening —
    // a read-only operation that does not disturb hot start), NOT a saved-rate
    // read-back: a rate that was written+saved but not yet adopted reads back as
    // "correct" while the engine still emits the old rate, so only measurement
    // can catch it.
    const float targetHz = 1000.0f / (float)GNSS_RATE_MS;          // 50 ms → 20 Hz
    const float minOkHz  = targetHz * 0.75f;                       // 20 → 15 Hz floor
    float measuredHz = gnss_measureOutputRateHz(serial);
    bool  rateOk     = (measuredHz >= minOkHz);
    (void)rateOk;   // no longer gates needFullConfig — see the savedVer==0 note below.
                    // Kept because rateWrong's meaning is defined against this floor.
    bool  rateWrong  = (measuredHz > 0.0f && measuredHz < minOkHz); // 0 = unknown

    Preferences prefs;
    prefs.begin("rcx_gnss", true);
    uint32_t savedVer     = prefs.getUInt("cfg_ver", 0);
    uint32_t savedMaskVer = prefs.getUInt("mask_ver", 0);
    s_eleMask = prefs.getFloat("ele", GNSS_ELE_MASK_DEG_DEFAULT);   // live mask values
    s_cnrMask = prefs.getFloat("cnr", GNSS_CNR_MASK_DBHZ_DEFAULT);
    s_pppMode = (uint8_t)prefs.getUChar("ppp", GNSS_PPP_MODE_DEFAULT);
    if (s_pppMode > 2) s_pppMode = GNSS_PPP_MODE_DEFAULT;           // corrupt NVS guard
    prefs.end();

    // ── Mask-defaults migration ──────────────────────────────────────────────
    // NVS is authoritative for the live masks, so new #define defaults would otherwise
    // never reach a board that has already booted (see GNSS_MASK_DEFAULTS_VERSION in
    // config.h). On a version bump we adopt the new defaults once, overwriting whatever
    // NVS held. Field-tuned values set from the web page survive every boot EXCEPT the
    // one after a deliberate defaults bump — that is the intended contract.
    // The values are pushed to the module via the normal deferred path (s_maskApplyPending
    // → gnss_loop), which costs no reset. If a full reconfigure is also due, skip the
    // pending flag: configureLG290P() writes the same two commands itself.
    const bool maskDefaultsChanged = (savedMaskVer != GNSS_MASK_DEFAULTS_VERSION);
    if (maskDefaultsChanged) {
        Serial.printf("🛰️  Mask defaults v%u→v%u — adopting %.1f° / %.1f dB-Hz "
                      "(overwrites saved %.1f° / %.1f dB-Hz).\n",
                      (unsigned)savedMaskVer, (unsigned)GNSS_MASK_DEFAULTS_VERSION,
                      GNSS_ELE_MASK_DEG_DEFAULT, GNSS_CNR_MASK_DBHZ_DEFAULT,
                      s_eleMask, s_cnrMask);
        s_eleMask = GNSS_ELE_MASK_DEG_DEFAULT;
        s_cnrMask = GNSS_CNR_MASK_DBHZ_DEFAULT;
    }

    // Does the FULL config (constellations / message rates / protocols) need
    // (re)writing? That is version-gated.
    //
    // savedVer == 0 (fresh/erased NVS) means WE HAVE NO RECORD OF EVER CONFIGURING
    // THIS MODULE — so we must. This used to be treated as "already configured if
    // the measured rate looks right", to spare a warm start after a flash erase.
    // That was wrong twice over: fix rate is ONE of ~15 settings (message set,
    // constellations, NAVMODE, protocols), so a correct rate says nothing about
    // the rest; and because the version was stamped below regardless of whether
    // anything was written, the skip became PERMANENT — every later boot saw
    // savedVer == GNSS_CONFIG_VERSION and skipped again. Observed 2026-07-20:
    // a v7 build ran for boots on a v6 module (GSA still 1 Hz, VTG/GLL still on)
    // while printing "hot start preserved". Cost of the fix is one warm start
    // after an NVS erase — rare, expected, and vastly cheaper than silently
    // running an unknown config forever.
    bool needFullConfig;
    if (savedVer == GNSS_CONFIG_VERSION) needFullConfig = false;
    else                                 needFullConfig = true;   // incl. savedVer == 0

    bool configWritten = false;   // ONLY a true write may stamp the version below

    if (needFullConfig) {
        // Settings changed or fresh/unknown module → rewrite everything.
        // configureLG290P() ends with a reset to adopt the fix rate. One warm
        // start, by design, only on a real config change.
        configureLG290P(serial);
        configWritten = true;
        Serial.println("   Full reconfigure — THIS boot warm-starts (expected, one time).");
    } else if (rateWrong) {
        // Config version is current in NVS, yet the module is MEASURABLY slower
        // than configured. NVS being current means this firmware wrote the full
        // config once — so a wrong rate now proves something outside this
        // firmware has rewritten the module since. The known case: this same
        // hardware flashed with the RCX1 base firmware leaves base mode in the
        // module's battery-backed config, and base mode caps output at 1 Hz — a
        // state a rate-only write cannot undo, because only the full config
        // writes PQTMCFGRCVRMODE back to rover. One setting proven wrong makes
        // every setting suspect, so all of them are rewritten. The cost is the
        // same single warm start a rate-only repair would spend, and it is paid
        // ONLY on a measured wrong rate — never on a routine boot.
        Serial.printf("⚙️  LG290P running %.1f Hz but config wants %.0f Hz — "
                      "full reconfigure (external rewrite suspected) + one reset.\n",
                      measuredHz, targetHz);
        configureLG290P(serial);
        configWritten = true;
    } else {
        // Already at target (or rate unverifiable). Send ZERO bytes — hot start
        // preserved. (If unverifiable, we deliberately do NOT reset: a false
        // reset would break hot start, which is worse than briefly staying put;
        // the next boot re-measures.)
        Serial.printf("✅ LG290P at ~%.1f Hz (config v%u) — no writes, hot start preserved.\n",
                      measuredHz, (unsigned)GNSS_CONFIG_VERSION);
    }

    // Stamp the version ONLY if configureLG290P() actually ran. Stamping on a
    // skip is what made the old savedVer==0 shortcut permanent — NVS claimed a
    // version the module had never been given. If a write did not happen, leave
    // savedVer alone so the next boot tries again.
    const bool stampVer = configWritten && (savedVer != GNSS_CONFIG_VERSION);
    if (stampVer || maskDefaultsChanged) {
        prefs.begin("rcx_gnss", false);
        if (stampVer) {
            prefs.putUInt("cfg_ver", GNSS_CONFIG_VERSION);
            prefs.putUChar("ppp", s_pppMode);   // seed the key on first run at this version
        }
        if (maskDefaultsChanged) {
            prefs.putUInt("mask_ver", GNSS_MASK_DEFAULTS_VERSION);
            prefs.putFloat("ele", s_eleMask);
            prefs.putFloat("cnr", s_cnrMask);
        }
        prefs.end();
    }
    // Push the adopted defaults to the module on the first gnss_loop() pass — but only if
    // configureLG290P() did not already write them this boot (it writes the same two
    // commands from the same s_eleMask/s_cnrMask). Deferred so the GNSS task stays the
    // only UART writer; no reset, so hot start is untouched.
    if (maskDefaultsChanged && !needFullConfig) s_maskApplyPending = true;

    // PPP state is NOT re-asserted on a hot-start boot. It lives in the module's own flash
    // (PQTMSAVEPAR), so a write here would be exactly the "harmless just-in-case write" the
    // hot-start contract forbids. If PPP ever reads back wrong, bump GNSS_CONFIG_VERSION —
    // that is the sanctioned way to re-push it.
    //
    //   *** WHY THIS ASKS THE MODULE INSTEAD OF PRINTING NVS ***
    //   A previous version of this printed s_pppMode straight from NVS. That is a LIE
    //   detector with the battery removed: if the module ever rejected the PPP write (old
    //   firmware → $PQTMCFGPPP,ERROR,3), NVS would still cheerfully report "E6 HAS" on every
    //   boot forever, and the boot log would confirm a feature that is not running. PQTMCFG
    //   READS are explicitly safe (contract rule #3 — they restart nothing), so on the skip
    //   path we ask the receiver what it is ACTUALLY doing. Same reason gnss_measureOutputRateHz
    //   exists instead of trusting PQTMCFGFIXRATE,R.
    if (!needFullConfig) {
        Serial.printf("🛰️  Masks (NVS): %.1f° / %.1f dB-Hz\n", s_eleMask, s_cnrMask);
        gnss_readAndLogPpp(serial);   // module truth — flags NVS/module divergence + old fw
    }

#if defined(PPP_NAV_DEBUG) && PPP_NAV_DEBUG
    // Standalone-accuracy diagnosis (bench only, all READ-only — hot start untouched). Prints
    // what the module is ACTUALLY running so we can compare against the spec defaults rather
    // than assume. Expected on an unmodified module (per protocol spec v1.1):
    //   SBAS   0x003F  = WAAS+SDCM+EGNOS+BDSBAS+MSAS+GAGAN (so WAAS IS on by default)
    //   SIGNAL 07,03,0F,3F,07,01 = GPS L1+L2C+L5 / GLO / GAL incl. E6 / BDS / QZSS / NavIC
    //   CNST   1,1,1,1,x,x = GPS+GLO+GAL+BDS (+QZSS/NavIC per our config)
    // Anything diverging from these is a lead on why standalone accuracy isn't better.
    gnss_readAndLogCfg(serial, "$PQTMCFGSBAS,R",   "$PQTMCFGSBAS,OK",   "SBAS (WAAS bit0)");
    gnss_readAndLogCfg(serial, "$PQTMCFGSIGNAL,R", "$PQTMCFGSIGNAL,OK", "Signal bands");
    gnss_readAndLogCfg(serial, "$PQTMCFGCNST,R",   "$PQTMCFGCNST,OK",   "Constellations");
#endif

    lastRateCalc = millis();
}

// ── Tunable-mask public API ───────────────────────────────────────────────────
float gnss_getEleMask() { return s_eleMask; }
float gnss_getCnrMask() { return s_cnrMask; }

// ── PPP public API ────────────────────────────────────────────────────────────
uint8_t gnss_getPppMode() { return s_pppMode; }

// Called from the web task. Stores the mode and flags the GNSS task to push it to the
// LG290P + NVS on its next loop (keeping all UART + NVS access on one task).
//
//   *** COST OF TOGGLING PPP AT RUNTIME (do not make this a casual switch) ***
//   $PQTMCFGPPP is a PQTMCFG WRITE. Quectel does not document whether it restarts the nav
//   engine, and the hot-start contract's rule #2 says assume it can. Worst case you lose
//   the current fix for a few seconds and re-acquire — acceptable in the paddock, NOT in
//   the middle of a run. The web page warns accordingly. Returns false on an invalid mode.
bool gnss_requestPppMode(uint8_t mode) {
    if (mode > 2) return false;              // $PQTMCFGPPP <Mode>: 0=off 1=B2b 2=E6 HAS
    s_pppMode = mode;
    s_pppApplyPending = true;
    return true;
}

// Called from the web task. Validates against the module's documented ranges, stores
// the values, and flags the GNSS task to push them to the LG290P + NVS on its next
// loop (keeping all UART + NVS access on one task). Returns false if out of range.
bool gnss_requestMasks(float eleDeg, float cnrDbHz) {
    if (eleDeg < -90.0f || eleDeg > 90.0f)  return false;   // $PQTMCFGELETHD range
    if (cnrDbHz <  0.0f || cnrDbHz > 99.0f) return false;   // $PQTMCFGCNRTHD range
    s_eleMask = eleDeg;
    s_cnrMask = cnrDbHz;
    s_maskApplyPending = true;
    return true;
}

// ── GGA sentence builder ──────────────────────────────────────────────────────
String gnss_buildGGA(const GnssData& g) {
    if (!g.valid) return "";

    double latAbs = fabs(g.latitude);
    int latDeg = (int)latAbs;
    double latMin = (latAbs - latDeg) * 60.0;

    double lonAbs = fabs(g.longitude);
    int lonDeg = (int)lonAbs;
    double lonMin = (lonAbs - lonDeg) * 60.0;

    char gg[128];
    snprintf(gg, sizeof(gg),
        "$GPGGA,%02d%02d%02d.00,%02d%08.5f,%c,%03d%08.5f,%c,%d,%02d,1.0,%.1f,M,%.1f,M,,",
        g.hour, g.minute, g.second,
        latDeg, latMin, (g.latitude >= 0) ? 'N' : 'S',
        lonDeg, lonMin, (g.longitude >= 0) ? 'E' : 'W',
        (g.rtkType == 2) ? 4 : (g.rtkType == 1) ? 5 : (g.valid ? 1 : 0),
        g.numSV, g.altMSL, g.geoidSepM);

    char ck[4];
    gnss_checksum(gg, ck);
    char out[140];
    snprintf(out, sizeof(out), "%s*%s\r\n", gg, ck);
    return String(out);
}

// ── Main loop call ────────────────────────────────────────────────────────────
static void gnssProcessTinyGps();   // applies a parsed line's fields to gps (defined below)
static void gnssUpdateRate();       // 2 s rate calc (defined below)

void gnss_loop(HardwareSerial& serial) {
    // Deferred mask apply requested by the web task. Handled HERE so the GNSS task is
    // the only writer to the UART and to NVS. No module reset: the position-engine
    // elevation/C-N0 thresholds take effect on the next epoch, and PQTMSAVEPAR persists
    // them across reboots. At most once per web "Apply".
    if (s_maskApplyPending) {
        s_maskApplyPending = false;
        char mb[48];
        snprintf(mb, sizeof(mb), "$PQTMCFGELETHD,W,%.1f", s_eleMask); sendPQTM(serial, mb);
        snprintf(mb, sizeof(mb), "$PQTMCFGCNRTHD,W,%.1f", s_cnrMask); sendPQTM(serial, mb);
        sendPQTM(serial, "$PQTMSAVEPAR");
        Preferences p;
        if (p.begin("rcx_gnss", false)) {
            p.putFloat("ele", s_eleMask);
            p.putFloat("cnr", s_cnrMask);
            p.end();
        }
        Serial.printf("🛰️  GNSS masks applied: ele=%.1f° cnr=%.1f dB-Hz (saved)\n",
                      s_eleMask, s_cnrMask);
    }

    // Deferred PPP mode change requested by the web task. Same one-writer discipline as the
    // masks. Unlike the masks this MAY briefly disturb the nav engine (see
    // gnss_requestPppMode) — the read-back afterwards tells us what the module actually did,
    // and flags a pre-v2.01 firmware that can't do PPP at all.
    if (s_pppApplyPending) {
        s_pppApplyPending = false;
        sendPppConfig(serial, s_pppMode);
        sendPQTM(serial, "$PQTMSAVEPAR");
        Preferences p;
        if (p.begin("rcx_gnss", false)) {
            p.putUChar("ppp", s_pppMode);
            p.end();
        }
        Serial.printf("🛰️  PPP set to %s (saved)\n",
                      s_pppMode == 2 ? "E6 HAS" : s_pppMode == 1 ? "B2b" : "off");
        gnss_readAndLogPpp(serial);
    }

    // ── Bounded, LINE-AT-A-TIME drain ────────────────────────────────────────
    // Accumulate a whole NMEA line, THEN dispatch it. Two reasons this must be
    // line-based, not per-byte:
    //   1) TinyGPS++ integrity. encode() is a byte-wise state machine that only
    //      resolves a sentence on its terminator. Feeding it the first few bytes
    //      of a GSV and then skipping the rest (the old per-byte bypass) left the
    //      parser hung mid-sentence; because $G_GSV and $G_RMC/$G_GGA share the
    //      "$G" prefix, that corrupted the parse of the FOLLOWING sentence and
    //      dropped fixes (regression found 2026-07-20 — a working fix stopped
    //      working after the bypass landed). Feeding only COMPLETE non-GSV lines
    //      means encode() always sees clean sentence boundaries.
    //   2) GSV cost avoidance still holds: a classified-GSV line is never fed to
    //      encode() at all (TinyGPS++ has no GSV handler), so the per-byte waste
    //      the bypass targeted is still avoided — just at line granularity.
    // Budget bounds bytes consumed per pass so the 1 Hz GSV block spreads across
    // passes instead of eating a BLE slot; the remainder waits in the RX ring.
    int drainBudget = GNSS_DRAIN_BUDGET_BYTES;

    while (serial.available() && drainBudget-- > 0) {
        char c = serial.read();

        if (c == '$') {                        // start of a new sentence
            nmeaIdx = 0;
            nmeaLine[nmeaIdx++] = c;
        } else if (nmeaIdx == 0) {
            continue;                          // stray byte between sentences — ignore
        } else if (c == '\n' || c == '\r') {   // sentence complete
            nmeaLine[nmeaIdx] = '\0';
            handleRawNmeaLine(nmeaLine);       // counters, GSA/GSV routing, RMC flag

            // Feed the WHOLE line to TinyGPS++ unless it is GSV (no handler for
            // it, and the sat/GSA data is already captured in handleRawNmeaLine).
            const bool isGSV = (nmeaIdx >= 6 && strncmp(nmeaLine + 3, "GSV", 3) == 0);
            if (!isGSV) {
                for (uint16_t i = 0; i < nmeaIdx; i++) tinygps.encode(nmeaLine[i]);
                tinygps.encode('\n');          // give the parser a clean terminator
                gnssProcessTinyGps();          // apply whatever this line updated
            }
            nmeaIdx = 0;
        } else if (nmeaIdx < sizeof(nmeaLine) - 1) {
            nmeaLine[nmeaIdx++] = c;
        } else {
            nmeaIdx = 0;                        // overlong/garbage — drop the line
        }
    }

    gnssUpdateRate();
}

// ── Apply a just-parsed TinyGPS++ update to shared gps state ──────────────────
// Called once per complete non-GSV line, right after that line is fed to
// tinygps.encode(). Pulls whatever fields that line freshened into `gps` under
// the data mutex. (Formerly the body of the per-byte drain loop; hoisted out so
// the drain can be clean line-at-a-time — see gnss_loop.)
static void gnssProcessTinyGps() {
        // Could not get the mutex this pass — skip applying THIS line. The
        // TinyGPS++ isUpdated() flags stay set and are applied on the next line
        // (was 'continue' when this was a per-byte loop body; now a plain return).
        if (!xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) return;

        // Time MUST be read before anything else touches the time object:
        // TinyGPSTime::second()/centisecond()/hour()/minute() all clear the
        // object's isUpdated() flag as a side effect. If the epoch-counter in
        // the location block below reads them first, this block's isUpdated()
        // returns false and the time freezes at 00:00:00. (Verified in
        // TinyGPS++ source: each getter sets updated=false.)
        if (tinygps.time.isUpdated()) {
            gps.hour        = tinygps.time.hour();
            gps.minute      = tinygps.time.minute();
            gps.second      = tinygps.time.second();
            gps.millisecond = tinygps.time.centisecond() * 10;
            gps.timeUpdateMillis = millis();
        }

        if (tinygps.location.isUpdated()) {
            gps.latitude  = tinygps.location.lat();
            gps.longitude = tinygps.location.lng();
            gps.valid     = tinygps.location.isValid();

            // Count unique epochs to avoid double-counting GNGGA+GNRMC of the
            // same timestamp. Use the STORED time (set above), not tinygps.time
            // getters, which would clear the updated flag.
            static uint32_t lastEpochCs = 0xFFFFFFFF;
            uint32_t cs = (uint32_t)gps.second * 100 + gps.millisecond / 10;
            if (cs != lastEpochCs) {
                lastEpochCs = cs;
                gnssUpdateCount++;
            }
        }
        if (tinygps.altitude.isUpdated()) {
            gps.altMSL   = tinygps.altitude.meters();
            gps.altWGS84 = gps.altMSL + gps.geoidSepM;
        }
        if (geoidSep.isUpdated() || geoidSep2.isUpdated()) {
            double sep = atof(geoidSep.isUpdated() ? geoidSep.value() : geoidSep2.value());
            if (sep > -200.0 && sep < 200.0) {
                gps.geoidSepM = sep;
                gps.altWGS84  = gps.altMSL + sep;
            }
        }
        // GGA field 13 <DiffAge> — age (s) of the differential/RTK corrections the
        // fix is based on. It climbs when RTCM/NTRIP drops and the module coasts on
        // RTK HOLD, and is empty (→ -1) when no corrections are in use at all. This
        // is the honest "how stale is my fix" number, independent of the FIXED/FLOAT
        // quality flag — a held FIXED reports quality 4 while this keeps rising.
        if (diffAge.isUpdated() || diffAge2.isUpdated()) {
            const char* v = diffAge.isUpdated() ? diffAge.value() : diffAge2.value();
            if (v && v[0] != '\0') {
                double a = atof(v);
                gps.diffAgeS = (a >= 0.0 && a < 100000.0) ? a : -1.0;
            } else {
                gps.diffAgeS = -1.0;   // empty field → no differential corrections in use
            }
        }
        bool spdUpd = tinygps.speed.isUpdated();
        bool crsUpd = tinygps.course.isUpdated();
        if (spdUpd)  gps.speedKnots = tinygps.speed.knots();
        if (crsUpd)  gps.headingDeg = tinygps.course.deg();
        // ── Epoch-complete marker ────────────────────────────────────────────
        // s_rmcLineDone was set by handleRawNmeaLine for THIS sentence (checksum
        // already verified), and we are past the speed/course application above,
        // so speed+heading in `gps` now belong to the current epoch. Publishing
        // (SD snapshot, BLE freshness) keys on epochSeq — never on the timestamp,
        // which GGA advances a sentence too early. That gap WAS the Langley
        // stale-speed bug; keying on the timestamp again reintroduces it.
        if (s_rmcLineDone) {
            s_rmcLineDone = false;
            // velUpdateMillis only advances when speed/course actually committed a
            // NEW value this epoch — not on every epoch regardless. RMC's SOG/COG
            // fields can legitimately arrive blank (receiver momentarily not
            // confident in a velocity solution) while position stays valid; when
            // that happens speedKnots/headingDeg correctly hold their last value
            // per the isUpdated() gate above, and this timestamp must say so, not
            // claim they're fresh. It was doing the latter — the ONLY signal
            // spd_age_ms (sd_log.cpp) and BLE have for "how old is this speed/
            // heading" was silently lying whenever RMC omitted SOG/COG.
            if (spdUpd || crsUpd) gps.velUpdateMillis = millis();
            gps.epochSeq++;
        }
        if (tinygps.satellites.isUpdated()) gps.numSV      = tinygps.satellites.value();
        if (tinygps.date.isUpdated()) {
            // Reject GPS-epoch reacquisition glitches. During reacquisition the
            // receiver recovers time-of-week before the week number, so it
            // momentarily reports the GPS epoch (1980-01-06); TinyGPS++ adds 2000
            // → year 2080. Only accept a sane current-era window so a transient
            // bad date never reaches the SD filename, BLE Utc, or display.
            // (Keeps the last good date until a valid one arrives.)
            uint16_t y = tinygps.date.year();
            if (y >= 2024 && y <= 2050) {
                gps.year  = y;
                gps.month = tinygps.date.month();
                gps.day   = tinygps.date.day();
            }
        }

        // Fix and RTK type
        if (!gps.valid) {
            gps.fixType   = 0;
            gps.rtkType   = 0;
            gps.pppActive = false;    // no fix ⇒ no PPP solution
        } else {
            gps.fixType = (gps.numSV >= 4) ? 3 : 2;
        }

        // GGA field 14 <StationID> — parsed BEFORE the quality block below, which needs it.
        // NOTE ON ORDERING: TinyGPSCustom::value() clears the object's isUpdated() flag as a
        // side effect (same trap as the time getters above), so this must read once and stash.
        // 9001/9002 = the module's own PPP solution; anything else = a real RTCM base.
        if (staId.isUpdated() || staId2.isUpdated()) {
            const char* v = staId.isUpdated() ? staId.value() : staId2.value();
            gps.diffStationId = (v && v[0]) ? (uint16_t)atoi(v) : 0;
        }

        if (fixQuality.isUpdated() || fixQuality2.isUpdated()) {
            int qi = atoi(fixQuality.isUpdated() ? fixQuality.value() : fixQuality2.value());
            bool recentEpe = gps.epeValid && (millis() - gps.epeMillis < 3000);
            uint8_t prevRtk = gps.rtkType;
            bool    prevPpp = gps.pppActive;
            // A PPP solution reports GGA quality 5 — IDENTICAL to RTK FLOAT — and is told
            // apart ONLY by the station ID (Quectel PPP App Note §3.1). Without this test
            // the dashboard, SD log and BLE stream would all call PPP "FLOAT".
            bool isPpp = (qi == 5) && (gps.diffStationId == 9001 || gps.diffStationId == 9002);
            gps.pppActive = isPpp;
            if      (qi == 4) { gps.rtkType = 2; if (!recentEpe) { gps.hAccM = 0.015; gps.vAccM = 0.015; } }
            else if (qi == 5) { gps.rtkType = 1;
                                if (!recentEpe) {
                                    // Stale-EPE fallback only. PPP is decimetre-class at BEST
                                    // (Galileo claims ~20 cm; field results are worse under any
                                    // multipath), so it must not inherit RTK FLOAT's 0.30 m
                                    // estimate — that would understate the error. PQTMEPE at
                                    // 20 Hz is the real source; this is the honest guess when
                                    // EPE has gone quiet.
                                    double est = isPpp ? 0.50 : 0.30;
                                    gps.hAccM = est; gps.vAccM = est;
                                } }
            else if (qi >= 1) { gps.rtkType = 0; }
            else /* qi == 0 — receiver reports NO FIX */ {
                // Un-latch validity. TinyGPS++'s location.isValid() is STICKY —
                // it means "has ever held a good value", and a quality-0 GGA has
                // EMPTY lat/lon fields, so tinygps.location never updates again
                // and gps.valid keeps vouching for the LAST fix indefinitely.
                // Observed 2026-07-20 bench: a brief warm-start fix latched
                // valid=true, the fix dropped, and BLE then streamed frames
                // marked "2D" with numSV=0 / hAcc=99.9 that SoloStorm displayed.
                // Quality 0 is the receiver saying "I have nothing" — believe it.
                // (Re-validation is automatic: the first quality>=1 GGA carries
                // real lat/lon, tinygps.location updates, and the location block
                // above restores gps.valid=true in that same pass.)
                gps.valid     = false;
                gps.rtkType   = 0;
                gps.pppActive = false;
            }
            // RTK engagement visibility — one line per state change. This is the
            // ground truth for "is the receiver actually using corrections":
            // RTCM bytes flowing + this never firing = receiver-side problem
            // (baseline too long, incompatible mount, RTK mode); RTCM at zero =
            // caster-side problem. Keep this log.
            if (gps.rtkType != prevRtk || gps.pppActive != prevPpp) {
                Serial.printf("🛰️  %s (GGA quality=%d, station=%u)\n",
                              gps.rtkType == 2   ? "RTK FIXED ✅" :
                              gps.pppActive      ? (gps.diffStationId == 9002
                                                      ? "PPP (E6 HAS) — no RTK"
                                                      : "PPP (B2b) — no RTK") :
                              gps.rtkType == 1   ? "RTK FLOAT"    : "no corrections — 3D GPS",
                              qi, (unsigned)gps.diffStationId);
            }
        }

        // EPE timeout fallback to HDOP
        if (gps.epeValid && millis() - gps.epeMillis > 3000) gps.epeValid = false;
        if (!gps.epeValid && gps.rtkType == 0 && tinygps.hdop.isUpdated()) {
            double hdop = tinygps.hdop.hdop();
            gps.hAccM = hdop * 1.0;
            gps.vAccM = hdop * 1.5;
        }

        xSemaphoreGive(dataMutex);
}

// ── Rate calculation, called once per gnss_loop pass ──────────────────────────
static void gnssUpdateRate() {
    uint32_t now = millis();
    if (now - lastRateCalc >= 2000) {
        float elapsed = (now - lastRateCalc) / 1000.0f;
        gnss_updateRate_Hz = gnssUpdateCount / elapsed;
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10))) {
            status.gnssHz = gnss_updateRate_Hz;
            xSemaphoreGive(dataMutex);
        }
        gnssUpdateCount = 0;
        lastRateCalc = now;
    }
}
