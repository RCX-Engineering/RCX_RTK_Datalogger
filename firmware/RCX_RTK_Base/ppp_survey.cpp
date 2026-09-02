/*
 * ppp_survey.cpp — see ppp_survey.h for the design rationale.
 *
 * State flow:
 *   IDLE → CONFIGURING → SURVEYING → LOCKING → DONE
 *                                          └──→ FAILED (not enough samples)
 *
 * All position math is double precision (ECEF ~6.4e6 m; cm precision needs it).
 */

#include "ppp_survey.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ── Module state ─────────────────────────────────────────────────────────────
static Stream*       gGnss        = nullptr;
static PppSurveyState gState      = PPP_IDLE;

static uint32_t      gDurationS   = PPP_SURVEY_DEFAULT_DURATION_S;
// LIVE-ADJUSTABLE ACCEPTANCE CRITERIA. These start at the compile-time defaults and can
// be changed at any time, including mid-survey, via ppp_survey_set_criteria(). Because
// the survey is ours rather than the module's, retargeting costs nothing: already-
// accumulated samples stay, and the new limits simply govern which fixes are accepted
// from this point on. Nothing is sent to the receiver, so there is no restart and no
// loss of convergence.
static float         gEpe2dLimit  = PPP_SURVEY_EPE_2D_LIMIT_M;
static float         gEpe3dLimit  = PPP_SURVEY_EPE_3D_LIMIT_M;
static uint32_t      gMinSamples  = PPP_SURVEY_MIN_SAMPLES;
static uint32_t      gSurveyStartMs = 0;

// Running accumulators (double — do not down-cast)
// ── Short-window position scatter, the convergence test that actually works ──
// PQTMEPE is CLAMPED. Measured across three sessions and 8,850 fixes it never once
// reported below 1.300 m, and it reported EXACTLY 1.300 for 46%, 22% and 10% of the
// fixes respectively — a hard wall, not a noise floor. Meanwhile the positions those
// same fixes describe scatter at sigma 0.28-0.49 m overall and 0.055-0.18 m within any
// 60-second window. The solution is several times better than the receiver's own
// accuracy field is able to express, so a convergence gate keyed on that field can
// never pass: 0.30 m is unreachable on a number whose floor is 1.300 m, by construction
// and regardless of how well PPP is actually doing.
//
// This measures convergence from the fixes themselves. Scatter over a short window is
// the thing an EPE figure is trying to estimate, it is computed here rather than
// reported by the module, and it has no floor. Welford accumulators over a sliding
// window of the most recent PPP_SURVEY_SCATTER_WINDOW positions.
static double        gScatN[PPP_SURVEY_SCATTER_WINDOW];
static double        gScatE[PPP_SURVEY_SCATTER_WINDOW];
static uint32_t      gScatCount = 0;   // total pushed; index = gScatCount % WINDOW
static double        gScatRefLat = 0.0, gScatRefLon = 0.0;
static bool          gScatHaveRef = false;
static float         gScatSigma = -1.0f;   // -1 = not enough samples yet

// Population sigma of the radial offsets in the window, in metres. -1 until the window
// is full — a scatter figure from a handful of fixes would read tight simply because
// consecutive GNSS fixes are correlated.
static float scatterSigmaM()
{
    if (gScatCount < PPP_SURVEY_SCATTER_WINDOW) return -1.0f;
    double mn = 0.0, me = 0.0;
    for (uint32_t i = 0; i < PPP_SURVEY_SCATTER_WINDOW; ++i) { mn += gScatN[i]; me += gScatE[i]; }
    mn /= PPP_SURVEY_SCATTER_WINDOW; me /= PPP_SURVEY_SCATTER_WINDOW;
    double acc = 0.0;
    for (uint32_t i = 0; i < PPP_SURVEY_SCATTER_WINDOW; ++i) {
        const double dn = gScatN[i] - mn, de = gScatE[i] - me;
        acc += dn * dn + de * de;
    }
    return (float)sqrt(acc / PPP_SURVEY_SCATTER_WINDOW);
}

static double        gSumLat = 0.0, gSumLon = 0.0, gSumAltEll = 0.0;
static uint32_t      gSamples = 0;
// SECOND, UNFILTERED ACCUMULATOR — every valid fix, regardless of EPE.
// PPP is not a mode the receiver can fail into: with PPP enabled the rover always
// produces the best solution the available corrections allow, degrading to an ordinary
// autonomous 3D fix when HAS data is weak or absent. Nothing needs to be switched over.
// What CAN come up empty is our own EPE-gated average, since a survey that never
// converges accepts no samples. So we average everything in parallel and, if the gated
// set is too thin at the end of the window, lock the unfiltered mean instead. The survey
// then always yields a coordinate — just an honestly less accurate one, recorded as such.
// Echo detector. A receiver still in FIXED base mode does not solve — it replays the
// coordinate it was configured with, so consecutive fixes are BIT-IDENTICAL and carry a
// tiny EPE. Those sail through the convergence gate, so a survey started against a
// fixed module would "converge" in seconds and re-lock the very coordinate the operator
// was trying to replace. A real GNSS solution always jitters in the low decimals, so
// identical fixes are proof we are not measuring anything.
static double        gEchoFirstLat = 0.0, gEchoFirstLon = 0.0;
static bool          gEchoAllSame  = false;
static bool          gEchoFault    = false;   // last survey failed because fixes were echoed
static double        gAllSumLat = 0.0, gAllSumLon = 0.0, gAllSumAltEll = 0.0;
static uint32_t      gAllSamples = 0;
static double        gAllSumEpe2d = 0.0;   // for an honest accuracy on the unfiltered path
static uint32_t      gAllEpeCount = 0;
static bool          gLockUsedConverged = false;
static uint32_t      gValidFixes = 0;

// Latest EPE (metres); negative = none yet
static float         gEpe2d = -1.0f, gEpe3d = -1.0f;
static bool          gHaveEpe = false;
static float         gBestEpe2d = NAN, gBestEpe3d = NAN;  // best (lowest) seen this survey

// E6 / HAS reception, parsed live from $GAGSV signal-ID 5.
static uint32_t      gE6Sats = 0;          // published (committed per GSV cycle)
static float         gE6AvgCnr = 0.0f;
static uint32_t      gE6TmpCount = 0;       // temp accumulator within one GSV group
static float         gE6TmpCnrSum = 0.0f;
static uint32_t      gE6LastMs = 0;         // for staleness (E6 group stopped arriving)

// Latest good GGA (deg / m ellipsoidal); valid flag set per-epoch
static bool          gHaveGga = false;
static double        gGgaLat = NAN, gGgaLon = NAN, gGgaAltEll = NAN;

// Captured result
static double        gEcefX = 0.0, gEcefY = 0.0, gEcefZ = 0.0;

// Command-sequencing (space commands so the module can process each)
static uint8_t       gStep = 0;
static uint32_t      gStepMs = 0;
static const uint32_t STEP_INTERVAL_MS = 250;
// Start time of the PPP_LOCKING post-reset settle (case 4→5 in ppp_survey_tick()).
// Deliberately separate from gStepMs — see the comment at case 4 for why sharing it
// stalled the state machine at gStep 5 indefinitely.
static uint32_t      gSettleStartMs = 0;
// The LG290P takes several seconds to come back after a restart — matches the main
// sketch's LG290P_POST_RESET_WAIT_MS.
static const uint32_t PPP_POST_RESET_SETTLE_MS = 5000;
// Delay before the NEXT configure step. Normally STEP_INTERVAL_MS; raised to the
// post-restart settle after a step that restarts the receiver, so the commands that
// follow are not fired at a module that is still booting and would silently drop them.
static uint32_t      gStepDelayMs = STEP_INTERVAL_MS;

// ── NMEA helpers ─────────────────────────────────────────────────────────────

// Extract the idx-th comma-delimited field (0 = "$xxGGA"). Handles empty fields.
static bool nmeaField(const char* s, int idx, char* out, size_t outLen)
{
    int cur = 0;
    const char* p = s;
    while (cur < idx && *p) {
        if (*p == ',') cur++;
        else if (*p == '*') { out[0] = 0; return false; }
        p++;
    }
    if (cur != idx) { out[0] = 0; return false; }
    size_t n = 0;
    while (*p && *p != ',' && *p != '*' && n < outLen - 1) out[n++] = *p++;
    out[n] = 0;
    return true;
}

// Extract the LAST comma-field before '*' (used for the GSV trailing SignalID).
static bool nmeaLastField(const char* s, char* out, size_t outLen)
{
    const char* star = strchr(s, '*');
    const char* end  = star ? star : s + strlen(s);
    const char* p    = end;
    while (p > s && *(p - 1) != ',') p--;
    size_t n = 0;
    while (p < end && *p && n < outLen - 1) out[n++] = *p++;
    out[n] = 0;
    return n > 0;
}

// Parse a Galileo GSV line; if it's the E6 signal (SignalID 5), tally tracked
// satellites and their C/N0. GSV blocks are {PRN,Elev,Azim,CNR} at fields
// 4..7, 8..11, 12..15, 16..19; CNR is the 4th of each. The group spans <numMsg>
// sentences — reset on msgNum==1, commit on the last one.
static void parseGalileoE6Gsv(const char* s)
{
    char sig[8];
    if (!nmeaLastField(s, sig, sizeof sig) || strcmp(sig, "5") != 0) return;  // E6 only

    char num[4], msg[4];
    nmeaField(s, 1, num, sizeof num);   // numMsg (total in group)
    nmeaField(s, 2, msg, sizeof msg);   // msgNum (this one)
    int total  = num[0] ? atoi(num) : 1;
    int msgNum = msg[0] ? atoi(msg) : 0;

    gE6LastMs = millis();
    if (msgNum == 1) { gE6TmpCount = 0; gE6TmpCnrSum = 0.0f; }

    for (int cnrField = 7; cnrField <= 19; cnrField += 4) {   // CNR slots
        char cnr[8];
        if (nmeaField(s, cnrField, cnr, sizeof cnr) && cnr[0]) {
            float c = atof(cnr);
            if (c > 0.0f) { gE6TmpCount++; gE6TmpCnrSum += c; }
        }
    }

    if (msgNum >= total) {   // group complete → publish
        gE6Sats   = gE6TmpCount;
        gE6AvgCnr = gE6TmpCount ? gE6TmpCnrSum / (float)gE6TmpCount : 0.0f;
    }
}

// ddmm.mmmm (+ hemisphere) → signed decimal degrees. Empty → NAN.
static double nmeaDegrees(const char* val, const char* hemi)
{
    if (!val[0]) return NAN;
    double v = atof(val);
    int deg = (int)(v / 100.0);
    double minutes = v - (double)deg * 100.0;
    double dd = (double)deg + minutes / 60.0;
    if (hemi[0] == 'S' || hemi[0] == 'W') dd = -dd;
    return dd;
}

// WGS84 geodetic (lat/lon deg, ellipsoidal height m) → ECEF metres.
static void geodeticToEcef(double latDeg, double lonDeg, double hEll,
                           double& X, double& Y, double& Z)
{
    const double a  = 6378137.0;                 // semi-major axis
    const double f  = 1.0 / 298.257223563;       // flattening
    const double e2 = f * (2.0 - f);             // first eccentricity squared

    double lat = latDeg * (M_PI / 180.0);
    double lon = lonDeg * (M_PI / 180.0);
    double sLat = sin(lat), cLat = cos(lat);
    double sLon = sin(lon), cLon = cos(lon);

    double N = a / sqrt(1.0 - e2 * sLat * sLat); // prime-vertical radius
    X = (N + hEll) * cLat * cLon;
    Y = (N + hEll) * cLat * sLon;
    Z = (N * (1.0 - e2) + hEll) * sLat;
}

// ── Command sender (appends NMEA checksum + CRLF) ────────────────────────────
// body is WITHOUT leading '$' and WITHOUT '*cs', e.g. "PQTMCFGPPP,W,0"
static void sendPqtm(const char* body)
{
    if (!gGnss) return;
    uint8_t cs = 0;
    for (const char* p = body; *p; p++) cs ^= (uint8_t)*p;
    gGnss->print('$');
    gGnss->print(body);
    gGnss->print('*');
    if (cs < 0x10) gGnss->print('0');
    gGnss->print(cs, HEX);
    gGnss->print("\r\n");
}

// ── Public API ───────────────────────────────────────────────────────────────

void ppp_survey_begin(Stream& gnssPort, uint32_t durationSeconds)
{
    gGnss       = &gnssPort;
    gDurationS  = durationSeconds ? durationSeconds : PPP_SURVEY_DEFAULT_DURATION_S;
    // gEpe2dLimit/gEpe3dLimit/gMinSamples deliberately persist across begin() so an
    // operator-tuned criterion is not silently reverted by the next survey.

    gStepDelayMs = STEP_INTERVAL_MS;
    gSumLat = gSumLon = gSumAltEll = 0.0;
    gScatCount = 0; gScatHaveRef = false; gScatSigma = -1.0f;
    gAllSumLat = gAllSumLon = gAllSumAltEll = 0.0;
    gAllSamples = 0; gAllSumEpe2d = 0.0; gAllEpeCount = 0;
    gEchoFirstLat = gEchoFirstLon = 0.0; gEchoAllSame = false; gEchoFault = false;
    gLockUsedConverged = false;
    gSamples = gValidFixes = 0;
    gEpe2d = gEpe3d = -1.0f;
    gBestEpe2d = gBestEpe3d = NAN;
    gHaveEpe = gHaveGga = false;
    gGgaLat = gGgaLon = gGgaAltEll = NAN;
    gE6Sats = gE6TmpCount = 0;
    gE6AvgCnr = gE6TmpCnrSum = 0.0f;
    gE6LastMs = 0;
    gEcefX = gEcefY = gEcefZ = 0.0;

    gStep = 0;
    gStepMs = millis();
    gSettleStartMs = 0;
    gState = PPP_CONFIGURING;
}

void ppp_survey_abort()
{
    if (gState == PPP_CONFIGURING || gState == PPP_SURVEYING || gState == PPP_LOCKING)
        gState = PPP_FAILED;
}

bool ppp_survey_active()
{
    return gState == PPP_CONFIGURING || gState == PPP_SURVEYING || gState == PPP_LOCKING;
}

void ppp_survey_feed_nmea(const char* sentence)
{
    if (!sentence) return;
    // Active during CONFIGURING (so E6 reception shows on the dashboard while the
    // module warms up) and SURVEYING. Idle/done/failed: ignore.
    if (gState != PPP_CONFIGURING && gState != PPP_SURVEYING) return;

    // ---- E6/HAS reception monitoring (both active states) ----
    if (strncmp(sentence, "$GAGSV", 6) == 0) { parseGalileoE6Gsv(sentence); return; }

    if (gState != PPP_SURVEYING) return;  // averaging only happens during the window

    // ---- PQTMEPE: $PQTMEPE,<ver>,<N>,<E>,<D>,<2D>,<3D>*cs ----
    if (strncmp(sentence, "$PQTMEPE", 8) == 0) {
        char f[16];
        if (nmeaField(sentence, 5, f, sizeof f) && f[0]) gEpe2d = atof(f);
        if (nmeaField(sentence, 6, f, sizeof f) && f[0]) gEpe3d = atof(f);
        gHaveEpe = (gEpe2d >= 0.0f && gEpe3d >= 0.0f);
        if (gHaveEpe) {  // track best-so-far for the dashboard progress read
            if (isnan(gBestEpe2d) || gEpe2d < gBestEpe2d) gBestEpe2d = gEpe2d;
            if (isnan(gBestEpe3d) || gEpe3d < gBestEpe3d) gBestEpe3d = gEpe3d;
        }
        return;
    }

    // ---- GGA (any talker): $xxGGA,time,lat,NS,lon,EW,q,nsat,hdop,alt,M,geoidSep,M,... ----
    if (strlen(sentence) > 6 && strncmp(sentence + 3, "GGA", 3) == 0) {
        char lat[16], ns[4], lon[16], ew[4], q[4], alt[16], sep[16];
        nmeaField(sentence, 2, lat, sizeof lat);
        nmeaField(sentence, 3, ns,  sizeof ns);
        nmeaField(sentence, 4, lon, sizeof lon);
        nmeaField(sentence, 5, ew,  sizeof ew);
        nmeaField(sentence, 6, q,   sizeof q);
        nmeaField(sentence, 9, alt, sizeof alt);
        nmeaField(sentence, 11, sep, sizeof sep);

        int quality = q[0] ? atoi(q) : 0;
        if (quality <= 0) { gHaveGga = false; return; }  // no fix this epoch

        double dLat = nmeaDegrees(lat, ns);
        double dLon = nmeaDegrees(lon, ew);
        if (isnan(dLat) || isnan(dLon) || !alt[0]) { gHaveGga = false; return; }

        double mslAlt   = atof(alt);
        double geoidSep = sep[0] ? atof(sep) : 0.0;
        // GGA altitude is orthometric (MSL); ECEF needs ELLIPSOIDAL height.
        double hEll = mslAlt + geoidSep;

        gGgaLat = dLat; gGgaLon = dLon; gGgaAltEll = hEll;
        gHaveGga = true;
        gValidFixes++;

        // Unfiltered running mean — the always-available fallback coordinate.
        if (gAllSamples == 0) {
            gEchoFirstLat = dLat; gEchoFirstLon = dLon; gEchoAllSame = true;
        } else if (dLat != gEchoFirstLat || dLon != gEchoFirstLon) {
            gEchoAllSame = false;
        }
        gAllSumLat += dLat; gAllSumLon += dLon; gAllSumAltEll += hEll;
        gAllSamples++;
        if (gHaveEpe && gEpe2d > 0.0f) { gAllSumEpe2d += gEpe2d; gAllEpeCount++; }

        // Feed the scatter window. Offsets are metres from the first accepted fix, which
        // only has to be a stable local origin — the sigma is about spread, not position.
        if (!gScatHaveRef) { gScatRefLat = dLat; gScatRefLon = dLon; gScatHaveRef = true; }
        {
            const uint32_t i = gScatCount % PPP_SURVEY_SCATTER_WINDOW;
            gScatN[i] = (dLat - gScatRefLat) * 111320.0;
            gScatE[i] = (dLon - gScatRefLon) * 111320.0 * cos(gScatRefLat * M_PI / 180.0);
            gScatCount++;
            gScatSigma = scatterSigmaM();
        }

        // Accept this fix into the converged set if EITHER convergence test passes.
        //
        // Scatter is the primary test: it is measured here, it has no floor, and it is
        // what an accuracy estimate is trying to approximate. The EPE test is kept as a
        // second route so a receiver that DOES report honest sub-limit accuracy is still
        // believed — but it can no longer be the only way in, which is what made every
        // survey fall back to the unfiltered mean.
        const bool epeConverged = gHaveEpe &&
                                  gEpe2d <= gEpe2dLimit &&
                                  gEpe3d <= gEpe3dLimit;
        const bool scatterConverged = gScatSigma >= 0.0f &&
                                      gScatSigma <= PPP_SURVEY_SCATTER_LIMIT_M;
        if (epeConverged || scatterConverged) {
            gSumLat    += dLat;
            gSumLon    += dLon;
            gSumAltEll += hEll;
            gSamples++;
        }
        return;
    }
}

void ppp_survey_tick()
{
    uint32_t now = millis();

    switch (gState) {

    case PPP_CONFIGURING: {
        // ┌─ DO NOT "CORRECT" THIS SEQUENCE AGAINST THE PUBLISHED SPEC ──────────────┐
        // │ The commands below are FIELD-PROVEN on this hardware and firmware. The   │
        // │ online LG290P documentation is NOT fully accurate, and two places in     │
        // │ particular contradict observed behaviour:                                │
        // │                                                                          │
        // │  1. The spec says a PQTMCFGRCVRMODE change requires PQTMSAVEPAR followed │
        // │     by a reset or "it will continue to operate in the original mode".    │
        // │     On this module the mode change applies LIVE — the survey runs, PPP   │
        // │     converges, and RTCM output stops, all without a save or a reset.     │
        // │     Adding save+reset here broke a working survey.                       │
        // │                                                                          │
        // │  2. PQTMEPE is documented as needing a trailing <MsgVer> in MSGRATE, and │
        // │     it does in the BASE-mode configuration — but NOT here. The plain     │
        // │     "PQTMCFGMSGRATE,W,PQTMEPE,1" form below is what actually produces    │
        // │     usable EPE in rover/PPP mode. Adding ",2" silenced the convergence   │
        // │     gate and the survey collected zero converged samples.                │
        // │                                                                          │
        // │ Verify any change here against a real survey log before keeping it.      │
        // └──────────────────────────────────────────────────────────────────────────┘
        //
        // Pace the setup commands. Runtime-only (no SAVEPAR) — we do NOT want
        // rover+PPP persisted as the boot state; only the final base config is saved.
        if (now - gStepMs < STEP_INTERVAL_MS) return;
        gStepMs = now;

        char cmd[64];
        switch (gStep) {
        case 0:  // ensure ROVER mode (PPP is a rover function)
            sendPqtm("PQTMCFGRCVRMODE,W,1");
            break;
        case 1:  // enable PPP: mode,datum,timeout,hAcc,vAcc
            // Parameter list confirmed against Quectel's PPP application note: Mode is
            // hexadecimal (0x02 = E6 HAS), Datum 1 = WGS84, Timeout range [90,180] s, and
            // both convergence thresholds are metres in range [0,5]. Every value sent here
            // sits inside those ranges, so a rejection is a capability or firmware answer
            // rather than a syntax one.
            snprintf(cmd, sizeof cmd, "PQTMCFGPPP,W,%d,%d,%d,%.2f,%.2f",
                     PPP_SURVEY_PPP_MODE, PPP_SURVEY_PPP_DATUM, PPP_SURVEY_PPP_TIMEOUT_S,
                     (double)PPP_SURVEY_PPP_HACC_M, (double)PPP_SURVEY_PPP_VACC_M);
            sendPqtm(cmd);
            break;
        case 2:  // SAVE the PPP setting — the vendor procedure requires it.
            // Quectel's PPP application note gives the enabling sequence as PQTMCFGPPP
            // FOLLOWED BY PQTMSAVEPAR, and this module has a long history of writes that
            // are accepted and then quietly do nothing until they are saved. Every survey
            // so far has run without this step, which is the simplest available
            // explanation for a survey that sees E6 satellites yet never produces a PPP
            // solution.
            //
            // The runtime-only rule above stands for the REST of this sequence, and this
            // step does not breach its purpose. The concern behind it was not persisting
            // PPP but persisting ROVER mode as the boot state — and rover is already what
            // the escape sequence saved before this survey began, on firmware that
            // deliberately boots into rover every time. What persists here is a rover that
            // also has PPP enabled, which is what the next survey wants anyway, and the
            // lock sequence disables PPP and saves again before the base goes live.
            sendPqtm("PQTMSAVEPAR");
            break;
        case 3:  // make sure PQTMEPE is output (our convergence gate) — 1/fix
            sendPqtm("PQTMCFGMSGRATE,W,PQTMEPE,1");
            break;
        case 4:  // make sure GGA is output (our position source) — 1/fix
            sendPqtm("PQTMCFGMSGRATE,W,GGA,1");
            break;
        case 5:  // make sure GSV is output (our E6 reception monitor) — 1/fix
            sendPqtm("PQTMCFGMSGRATE,W,GSV,1");
            break;
        case 6:  // Permissive elevation mask — see PPP_SURVEY_ELE_MASK_DEG.
            // Added after the proven steps above, not woven into them: these masks live
            // in module NVM and a tight rover mask left over from other use would starve
            // this survey. Nothing above this line is modified.
            snprintf(cmd, sizeof cmd, "PQTMCFGELETHD,W,%.1f", (double)PPP_SURVEY_ELE_MASK_DEG);
            sendPqtm(cmd);
            break;
        case 7:  // Permissive C/N0 mask — see PPP_SURVEY_CNR_MASK_DBHZ.
            snprintf(cmd, sizeof cmd, "PQTMCFGCNRTHD,W,%.1f", (double)PPP_SURVEY_CNR_MASK_DBHZ);
            sendPqtm(cmd);
            break;
        case 8:  // PQTMPPPNAV — the module's own report of its PPP solution.
            // Appended after the proven steps, like the two masks above, and read-only in
            // effect: nothing in the survey's averaging or gating consumes it. It exists
            // because every signal this survey currently watches — GGA position and
            // PQTMEPE accuracy — describes the receiver's ORDINARY solution. If the module
            // publishes its PPP fix separately, a converged PPP solution and a completely
            // stalled one look identical from here, which is exactly the ambiguity two
            // full sessions of 1.300 m EPE have been sitting in. The sentence is captured
            // verbatim by the sketch and shown on the dashboard rather than parsed, so no
            // assumption about its field layout can go wrong.
            sendPqtm("PQTMCFGMSGRATE,W,PQTMPPPNAV,1,1");
            break;
        default:
            gSurveyStartMs = now;
            gState = PPP_SURVEYING;
            return;
        }
        gStep++;
        return;
    }

    case PPP_SURVEYING: {
        if (now - gSurveyStartMs >= gDurationS * 1000UL) {
            gStep = 0;
            gStepMs = now;
            gState = PPP_LOCKING;
        }
        return;
    }

    case PPP_LOCKING: {
        // First entry (gStep==0): decide pass/fail and compute the coordinate.
        if (gStep == 0) {
            // Prefer the EPE-gated (PPP-converged) average. If it is too thin, fall back
            // to the unfiltered mean rather than failing — see the accumulator note above.
            // Refuse to lock an echoed coordinate — see the echo detector above.
            if (gEchoAllSame && gAllSamples >= 2) {
                gState    = PPP_FAILED;
                gEchoFault = true;   // surfaced via ppp_survey_status().echoFault
                return;
            }
            gLockUsedConverged = (gSamples >= gMinSamples);
            if (!gLockUsedConverged && gAllSamples == 0) {
                gState = PPP_FAILED;   // no valid fix at all: no sky, nothing to lock
                return;
            }
            const uint32_t n  = gLockUsedConverged ? gSamples    : gAllSamples;
            const double sLat = gLockUsedConverged ? gSumLat     : gAllSumLat;
            const double sLon = gLockUsedConverged ? gSumLon     : gAllSumLon;
            const double sAlt = gLockUsedConverged ? gSumAltEll  : gAllSumAltEll;
            double meanLat = sLat / (double)n;
            double meanLon = sLon / (double)n;
            double meanH   = sAlt / (double)n;
            geodeticToEcef(meanLat, meanLon, meanH, gEcefX, gEcefY, gEcefZ);
        }

        if (now - gStepMs < STEP_INTERVAL_MS) return;
        gStepMs = now;

        char cmd[96];
        switch (gStep) {
        case 0:  // stop PPP cleanly before becoming a base
            sendPqtm("PQTMCFGPPP,W,0");
            break;
        case 1:  // base-station working mode
            sendPqtm("PQTMCFGRCVRMODE,W,2");
            break;
        case 2:  // fixed base at our captured ECEF (mode 2; count/acc = 0)
            snprintf(cmd, sizeof cmd, "PQTMCFGSVIN,W,2,0,0,%.4f,%.4f,%.4f",
                     gEcefX, gEcefY, gEcefZ);
            sendPqtm(cmd);
            break;
        case 3:  // persist to the module's NVS
            sendPqtm("PQTMSAVEPAR");
            break;
        case 4:  // reset → reboots as a fixed base broadcasting RTCM
            // The reset IS needed at THIS step (unlike the rover switch above) because
            // it is what brings the module back up publishing from the fixed coordinate.
            // Field-proven; do not swap it for a hot start to save reacquisition time.
            // PQTMSRR, not PQTMHOT. This is the sequence that was field-proven to bring
            // the module back up as a working fixed base. A hot start was tried to
            // preserve tracking state and is NOT worth the risk here — the mode change
            // taking effect matters more than the reacquisition time, which is paid once
            // at the end of a survey that already ran for its full window.
            sendPqtm("PQTMSRR");
            // Mark the settle window's own start. MUST be a separate variable from
            // gStepMs: gStepMs is reassigned to `now` on every tick by the outer
            // STEP_INTERVAL_MS gate above, including the tick that first evaluates
            // case 5 — so a case 5 that measured elapsed time against gStepMs was
            // always comparing `now` against a timestamp taken the same instant,
            // never accumulating past ~0 ms. That stalled gStep at 5 forever: LOCKING
            // never reached PPP_DONE, checkPppSurveyCompletion() never fired, the
            // converged position was never saved/applied, and enableLg290pBaseOutputs()
            // was never called to reassert RTCM/PQTMSVINSTATUS rates after the reset —
            // leaving the module on whatever output the reset defaulted to.
            gSettleStartMs = now;
            break;
        case 5:
            // Settle after the reset before declaring DONE, so the caller does not
            // reconfigure or start casting into a module that is still booting.
            if (now - gSettleStartMs < PPP_POST_RESET_SETTLE_MS) return;
            break;
        default:
            gState = PPP_DONE;
            return;
        }
        gStep++;
        return;
    }

    default:
        return;  // IDLE / DONE / FAILED — nothing to do
    }
}

void ppp_survey_set_criteria(float epe2dLimitM, float epe3dLimitM,
                             uint32_t minSamples, uint32_t durationSeconds)
{
    // Applies immediately, mid-survey included. Only positive values change anything, so
    // a caller can retarget one criterion and leave the rest alone.
    //
    // A fix must clear BOTH gates, so the tighter one decides. A caller that moves only
    // the 2D limit would otherwise leave the 3D limit at its previous value and silently
    // hand it control: with the 2D gate opened to 3.20 m and the 3D gate still at 0.50 m,
    // a session whose 3D EPE never fell below 2.32 m accepted zero samples and fell back
    // to the unfiltered autonomous mean, while the dashboard reported the 2D limit the
    // operator had set. Deriving the partner keeps the pair honest.
    if (epe2dLimitM > 0.0f)  gEpe2dLimit = epe2dLimitM;
    if (epe3dLimitM > 0.0f)  gEpe3dLimit = epe3dLimitM;
    else if (epe2dLimitM > 0.0f) gEpe3dLimit = epe2dLimitM * PPP_SURVEY_EPE_3D_RATIO;
    if (minSamples  > 0)     gMinSamples = minSamples;
    if (durationSeconds > 0) gDurationS  = durationSeconds;   // window is start + duration
}

void ppp_survey_get_criteria(float& epe2dLimitM, float& epe3dLimitM,
                             uint32_t& minSamples, uint32_t& durationSeconds)
{
    epe2dLimitM = gEpe2dLimit;
    epe3dLimitM = gEpe3dLimit;
    minSamples  = gMinSamples;
    durationSeconds = gDurationS;
}

PppSurveyStatus ppp_survey_status()
{
    PppSurveyStatus s;
    s.state       = gState;
    s.durationS   = gDurationS;
    s.elapsedS    = (gState == PPP_SURVEYING && gSurveyStartMs)
                        ? (millis() - gSurveyStartMs) / 1000UL
                        : (gState >= PPP_LOCKING ? gDurationS : 0);
    s.samples     = gSamples;
    s.validFixes  = gValidFixes;
    s.haveEpe     = gHaveEpe;
    s.lastEpe2d   = gEpe2d;
    s.lastEpe3d   = gEpe3d;
    s.bestEpe2d   = gBestEpe2d;
    s.bestEpe3d   = gBestEpe3d;
    s.converged   = gHaveEpe && gEpe2d >= 0.0f && gEpe3d >= 0.0f &&
                    gEpe2d <= gEpe2dLimit &&
                    gEpe3d <= gEpe3dLimit;

    // E6 reception, with a staleness guard: if the E6 GSV group hasn't arrived in
    // ~3 s, treat E6 as not being received (0 sats) rather than showing stale data.
    bool e6Stale = (gE6LastMs == 0) || (millis() - gE6LastMs > 3000);
    s.e6Sats   = e6Stale ? 0 : gE6Sats;
    s.e6AvgCnr = (e6Stale || gE6Sats == 0) ? NAN : gE6AvgCnr;
    s.e6Ok     = (s.e6Sats > 0) && !isnan(s.e6AvgCnr) &&
                 (s.e6AvgCnr >= PPP_SURVEY_E6_CNR_FLOOR);

    s.samplesAll        = gAllSamples;
    s.echoFault         = gEchoFault;
    s.lockUsedConverged = gLockUsedConverged;
    s.meanEpe2dAll      = gAllEpeCount ? (float)(gAllSumEpe2d / (double)gAllEpeCount) : NAN;
    s.meanLat     = gSamples ? gSumLat / (double)gSamples : NAN;
    s.meanLon     = gSamples ? gSumLon / (double)gSamples : NAN;
    s.meanAltEll  = gSamples ? gSumAltEll / (double)gSamples : NAN;
    s.ecefX = gEcefX; s.ecefY = gEcefY; s.ecefZ = gEcefZ;
    return s;
}

const char* ppp_survey_state_name(PppSurveyState st)
{
    switch (st) {
    case PPP_IDLE:        return "IDLE";
    case PPP_CONFIGURING: return "CONFIGURING";
    case PPP_SURVEYING:   return "SURVEYING";
    case PPP_LOCKING:     return "LOCKING";
    case PPP_DONE:        return "DONE";
    default:              return "FAILED";
    }
}

// Compact JSON for GET /api/ppp. NAN/absent values are emitted as JSON null so
// the dashboard can render "–". Field names here MUST match the dashboard JS.
void ppp_survey_status_json(char* buf, size_t len)
{
    PppSurveyStatus s = ppp_survey_status();
    char* w   = buf;
    char* end = buf + len;
    #define PPP_APPEND(...) do { \
        int _n = snprintf(w, (size_t)(end - w), __VA_ARGS__); \
        if (_n < 0 || _n >= end - w) { if (len) buf[0] = 0; return; } \
        w += _n; } while (0)

    PPP_APPEND("{\"state\":\"%s\"", ppp_survey_state_name(s.state));
    PPP_APPEND(",\"elapsed\":%lu,\"dur\":%lu",
               (unsigned long)s.elapsedS, (unsigned long)s.durationS);
    PPP_APPEND(",\"samples\":%lu,\"valid\":%lu",
               (unsigned long)s.samples, (unsigned long)s.validFixes);

    PPP_APPEND(",\"lim2d\":%.2f,\"lim3d\":%.2f,\"need\":%lu,\"scatlim\":%.2f",
               (double)gEpe2dLimit, (double)gEpe3dLimit, (unsigned long)gMinSamples,
               (double)PPP_SURVEY_SCATTER_LIMIT_M);
    if (gScatSigma >= 0.0f) PPP_APPEND(",\"scat\":%.3f", (double)gScatSigma);
    else                    PPP_APPEND(",\"scat\":null");
    if (s.lastEpe2d >= 0.0f) PPP_APPEND(",\"epe2d\":%.2f", s.lastEpe2d); else PPP_APPEND(",\"epe2d\":null");
    if (s.lastEpe3d >= 0.0f) PPP_APPEND(",\"epe3d\":%.2f", s.lastEpe3d); else PPP_APPEND(",\"epe3d\":null");
    if (!isnan(s.bestEpe2d)) PPP_APPEND(",\"best2d\":%.2f", s.bestEpe2d); else PPP_APPEND(",\"best2d\":null");
    if (!isnan(s.bestEpe3d)) PPP_APPEND(",\"best3d\":%.2f", s.bestEpe3d); else PPP_APPEND(",\"best3d\":null");
    PPP_APPEND(",\"conv\":%s", s.converged ? "true" : "false");

    PPP_APPEND(",\"e6sats\":%lu", (unsigned long)s.e6Sats);
    if (!isnan(s.e6AvgCnr)) PPP_APPEND(",\"e6cnr\":%.1f", s.e6AvgCnr); else PPP_APPEND(",\"e6cnr\":null");
    PPP_APPEND(",\"e6ok\":%s", s.e6Ok ? "true" : "false");

    if (!isnan(s.meanLat))
        PPP_APPEND(",\"lat\":%.8f,\"lon\":%.8f,\"alt\":%.3f", s.meanLat, s.meanLon, s.meanAltEll);
    else
        PPP_APPEND(",\"lat\":null,\"lon\":null,\"alt\":null");

    if (s.ecefX != 0.0)
        PPP_APPEND(",\"x\":%.4f,\"y\":%.4f,\"z\":%.4f", s.ecefX, s.ecefY, s.ecefZ);
    else
        PPP_APPEND(",\"x\":null,\"y\":null,\"z\":null");

    PPP_APPEND("}");
    #undef PPP_APPEND
}
