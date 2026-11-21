/*
 * ppp_survey.cpp — see ppp_survey.h for the design rationale.
 *
 * State flow:
 *   IDLE → CONFIGURING → SURVEYING → LOCKING → DONE
 *                            │           └──→ FAILED (echoed coordinate, or no valid fix at all)
 *                            └──→ FAILED (no fresh fix by the end of the window's grace,
 *                                         or base echoes at its end)
 *
 * Too few converged fixes never fails a survey: the lock then uses the average of all fixes
 * (AVERAGING POLICY below). ppp_survey_abort() ends any active state in FAILED.
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
static uint32_t      gSurveyStartMs = 0;
// Seconds the averaging window actually ran, frozen when it ends (expiry, operator finish
// or abort). Reported as elapsedS from LOCKING on, so a window the operator finished early
// is not described as the full configured duration.
static uint32_t      gWindowS = 0;
// The operator ended the window early (ppp_survey_finish_now()); the coordinate is the
// average of what was collected up to that moment.
static bool          gFinishedEarly = false;
// When the last fix was averaged in (0 = none yet), and whether the survey failed because
// fixes had stopped when the window ended. See PPP_SURVEY_FIX_STALE_MS.
static uint32_t      gLastFixMs = 0;
static bool          gSignalLost = false;
// Window-end grace (PPP_SURVEY_END_GRACE_MS): when the operator asked for the early end that
// is waiting for a fresh fix (0 = none), whether the survey is waiting, and how many early
// ends lapsed.
static uint32_t      gFinishReqMs = 0;
static bool          gAwaitingFix = false;
static uint32_t      gFinishLapses = 0;

// Measured fix interval (PPP_SURVEY_RATE_PROBE_MS): GGA sentences seen in the first stretch of
// the averaging window, and when the first and last of them arrived.
static uint32_t      gRateGgaCount = 0, gRateFirstMs = 0, gRateLastMs = 0;

// Running accumulators (double — do not down-cast)
// ── Short-window position scatter, the convergence test that actually works ──
// Earlier sessions (8,850 fixes across three) never recorded a PQTMEPE value below
// 1.300 m, with exactly 1.300 on 46%, 22% and 10% of fixes, while the positions those
// fixes describe scattered at sigma 0.28-0.49 m overall and 0.055-0.18 m within any
// 60-second window. A later session (0107) recorded 2D EPE as low as 0.052 m, so that
// floor is not a fixed property of the field in every configuration. The reported
// figure has not been shown to track the achieved scatter, so the convergence gate
// does not depend on it.
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

// ── AVERAGING POLICY: two sets, one coordinate ──────────────────────────────
// CONVERGED SET. A fix counts as converged only if it is a PPP solution (GGA quality 5, or
// differential station 9001/9002 in GGA field 14, which is how the receiver marks a B2b or
// E6 HAS solution) AND passes the convergence test (the EPE route or the scatter route; the
// scatter window holds PPP-solution fixes only). Autonomous (1) and SBAS/DGPS (2) fixes are
// never converged, however tight they look, and base echoes (7) are not averaged at all.
//
// ALL-FIX SET. PPP is not a mode the receiver can fail into: with PPP enabled the rover
// always produces the best solution the available corrections allow, degrading to an
// ordinary autonomous fix when HAS data is weak or absent. Every non-echo fix is averaged
// in parallel so the survey always yields a coordinate.
//
// AT LOCK. The coordinate is the mean of the all-fix set (source "ppp-auto", saved and shown
// as UNCONVERGED, accuracy the larger of that set's mean EPE and its position scatter) until
// converged PPP has run long enough to replace it. It is then the mean of the converged set
// only (source "ppp", accuracy the mean EPE of that set). Converged PPP replaces the
// all-fix mean when BOTH hold:
//   1. it holds at least one fix for every PPP_SURVEY_PPP_TIME_RATIO fixes that did not
//      converge, never fewer than PPP_SURVEY_CONVERGED_FLOOR_FIXES and never more than
//      PPP_SURVEY_CONVERGED_CAP_FIXES required (ppp_survey.h gives the reasoning for each);
//   2. its first-half and second-half means agree (the stability gate in ppp_survey.h).
// Otherwise the lock keeps the all-fix mean, and the rule and its numbers are reported
// (PppSurveyStatus.rule, ppp_survey_rule_text()). The same decision serves the window's end
// and "Lock survey now".
//
// Why switch outright rather than mix: an autonomous average is bias-limited (ionosphere,
// orbit and multipath errors stay correlated over hours) at roughly 0.5-1 m however long it
// runs, while converged PPP is ~5 cm. Inverse-variance weighting would give the autonomous
// fixes about 1% of the weight, which is not worth the complexity. Until the switch, the
// all-fix mean includes the PPP fixes too, so it improves as PPP runs.
static double        gSumLat = 0.0, gSumLon = 0.0, gSumAltEll = 0.0;
static uint32_t      gSamples = 0;                 // converged set size
static double        gConvSumEpe2d = 0.0;          // EPE of converged fixes that carried one
static uint32_t      gConvEpeCount = 0;
static double        gConvSumScat = 0.0;           // scatter sigma at admission, for converged
static uint32_t      gConvScatCount = 0;           // fixes admitted with no fresh EPE
static bool          gLastConverged = false;       // the most recent fix joined the converged set
static bool          gLastPppSolution = false;     // the most recent fix was a PPP solution
static float         gLockAccM = NAN;              // accuracy decided at lock (see AT LOCK)
static PppLockRule   gLockRule = PPP_RULE_NONE;    // the rule the lock applied
// Converged-set halves for the stability gate: north/east/up offsets (m) from the set's first
// fix, summed over consecutive blocks of gHalfBlockSize fixes. When every block is full the
// blocks merge in pairs and the block size doubles, so the blocks always cover the whole set
// in order and the split nearest its middle is within half a block of it.
struct PppHalfBlock { float n, e, u; uint32_t k; };
static PppHalfBlock  gHalf[PPP_SURVEY_HALF_BLOCKS];
static uint32_t      gHalfUsed = 0, gHalfBlockSize = 1;
static double        gConvRefLat = 0.0, gConvRefLon = 0.0, gConvRefH = 0.0;
// Echo detector. A receiver still in FIXED base mode does not solve — it replays the
// coordinate it was configured with, so consecutive fixes are BIT-IDENTICAL and carry a
// tiny EPE. Those sail through the convergence gate, so a survey started against a
// fixed module would "converge" in seconds and re-lock the very coordinate the operator
// was trying to replace. A real GNSS solution always jitters in the low decimals, so
// identical fixes are proof we are not measuring anything.
static double        gEchoFirstLat = 0.0, gEchoFirstLon = 0.0;
static bool          gEchoAllSame  = false;
static bool          gEchoFault    = false;   // last survey failed because fixes were echoed
// GGA quality 7 is the base-station code: the receiver is reporting the reference coordinate
// it was configured with, not a solution. A rover never reports it, so a quality-7 fix is
// proof the mode switch did not take, and it is never averaged (a run of identical echoes
// has zero scatter and would otherwise pass the scatter gate as perfectly converged). These
// count the echoes seen this survey and when the last one arrived.
static uint32_t      gEchoQ7Fixes  = 0;
static uint32_t      gLastQ7Ms     = 0;
static double        gAllSumLat = 0.0, gAllSumLon = 0.0, gAllSumAltEll = 0.0;
static uint32_t      gAllSamples = 0;
static double        gAllSumEpe2d = 0.0;   // for an honest accuracy on the unfiltered path
static uint32_t      gAllEpeCount = 0;
// Position scatter of the all-fix set: offsets (m) from its first fix, summed.
static double        gAllRefLat = 0.0, gAllRefLon = 0.0;
static double        gAllSumN = 0.0, gAllSumE = 0.0, gAllSumN2 = 0.0, gAllSumE2 = 0.0;
static bool          gLockUsedConverged = false;
// The coordinate the lock decided (deg / m ellipsoidal), whichever set it came from. Reported
// from LOCKING on, so a page showing the survey never shows a thin or empty converged-set mean
// while a different, locked coordinate exists.
static bool          gLockHaveCoord = false;
static double        gLockLat = 0.0, gLockLon = 0.0, gLockAltEll = 0.0;
static uint32_t      gValidFixes = 0;

// Latest EPE (metres); negative = none yet
static float         gEpe2d = -1.0f, gEpe3d = -1.0f;
static bool          gHaveEpe = false;
// When the latest PQTMEPE arrived. An EPE older than PPP_SURVEY_EPE_FRESH_MS is not used: a
// receiver that stops sending PQTMEPE would otherwise leave its last figure standing for every
// later fix — admitting all of them through the EPE route if that figure was low.
static uint32_t      gEpeMs = 0;
static bool epeFresh()
{
    return gHaveEpe && gEpeMs != 0 && millis() - gEpeMs <= PPP_SURVEY_EPE_FRESH_MS;
}
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

// ── Converged-set halves (stability gate) ────────────────────────────────────
static void halfAdd(float n, float e, float u)
{
    if (gHalfUsed == 0 || gHalf[gHalfUsed - 1].k >= gHalfBlockSize) {
        if (gHalfUsed == PPP_SURVEY_HALF_BLOCKS) {   // all full: merge pairs, double the size
            for (uint32_t i = 0; i < PPP_SURVEY_HALF_BLOCKS / 2; ++i) {
                const PppHalfBlock& a = gHalf[2 * i];
                const PppHalfBlock& b = gHalf[2 * i + 1];
                gHalf[i] = { a.n + b.n, a.e + b.e, a.u + b.u, a.k + b.k };
            }
            gHalfUsed = PPP_SURVEY_HALF_BLOCKS / 2;
            gHalfBlockSize *= 2;
        }
        gHalf[gHalfUsed++] = { 0.0f, 0.0f, 0.0f, 0 };
    }
    PppHalfBlock& b = gHalf[gHalfUsed - 1];
    b.n += n; b.e += e; b.u += u; b.k++;
}

// Horizontal and vertical distance (m) between the means of the converged set's first and
// second halves, split at the block boundary nearest its middle. False with fewer than two
// blocks.
static bool halfSplit(float& h, float& v)
{
    if (gHalfUsed < 2) return false;
    uint32_t total = 0;
    for (uint32_t i = 0; i < gHalfUsed; ++i) total += gHalf[i].k;
    uint32_t cum = 0, bestB = 1, bestOff = UINT32_MAX;
    for (uint32_t b = 1; b < gHalfUsed; ++b) {   // boundary before block b
        cum += gHalf[b - 1].k;
        const uint32_t off = 2 * cum > total ? 2 * cum - total : total - 2 * cum;
        if (off < bestOff) { bestOff = off; bestB = b; }
    }
    double s1[3] = {0, 0, 0}, s2[3] = {0, 0, 0};
    uint32_t k1 = 0, k2 = 0;
    for (uint32_t i = 0; i < gHalfUsed; ++i) {
        double* s = i < bestB ? s1 : s2;
        s[0] += gHalf[i].n; s[1] += gHalf[i].e; s[2] += gHalf[i].u;
        (i < bestB ? k1 : k2) += gHalf[i].k;
    }
    const double dn = s2[0] / k2 - s1[0] / k1, de = s2[1] / k2 - s1[1] / k1, du = s2[2] / k2 - s1[2] / k1;
    h = (float)sqrt(dn * dn + de * de);
    v = (float)fabs(du);
    return true;
}

// The AVERAGING POLICY decision and its numbers, from the accumulators as they stand. The
// lock takes it once at PPP_LOCKING; the status reports it live while averaging.
struct PppRuleEval {
    PppLockRule rule;
    uint32_t nonConverged, needed;
    float convMeanEpe, halfH, halfV, limH, limV, convToAllH, convToAllV;
};
static PppRuleEval evaluateRule()
{
    PppRuleEval r;
    r.nonConverged = gAllSamples > gSamples ? gAllSamples - gSamples : 0;
    uint32_t need = (r.nonConverged + PPP_SURVEY_PPP_TIME_RATIO - 1) / PPP_SURVEY_PPP_TIME_RATIO;
    if (need < PPP_SURVEY_CONVERGED_FLOOR_FIXES) need = PPP_SURVEY_CONVERGED_FLOOR_FIXES;
    if (need > PPP_SURVEY_CONVERGED_CAP_FIXES)   need = PPP_SURVEY_CONVERGED_CAP_FIXES;
    r.needed = need;
    // Mean EPE of the converged set; fixes admitted on scatter with no EPE in hand contribute
    // their window sigma instead.
    const uint32_t k = gConvEpeCount + gConvScatCount;
    r.convMeanEpe = k ? (float)((gConvSumEpe2d + gConvSumScat) / (double)k) : NAN;
    const float base = (r.convMeanEpe > 0.0f) ? PPP_SURVEY_STABLE_EPE_MULT * r.convMeanEpe : 0.0f;
    r.limH = base > PPP_SURVEY_STABLE_MIN_M ? base : PPP_SURVEY_STABLE_MIN_M;
    r.limV = r.limH * PPP_SURVEY_STABLE_V_RATIO;
    if (!halfSplit(r.halfH, r.halfV)) r.halfH = r.halfV = NAN;
    r.convToAllH = r.convToAllV = NAN;
    if (gSamples && gAllSamples) {
        const double cLat = gSumLat / gSamples, cLon = gSumLon / gSamples, cH = gSumAltEll / gSamples;
        const double aLat = gAllSumLat / gAllSamples, aLon = gAllSumLon / gAllSamples, aH = gAllSumAltEll / gAllSamples;
        const double dn = (cLat - aLat) * 111320.0, de = (cLon - aLon) * 111320.0 * cos(aLat * M_PI / 180.0);
        r.convToAllH = (float)sqrt(dn * dn + de * de);
        r.convToAllV = (float)fabs(cH - aH);
    }
    if (gAllSamples == 0)            r.rule = PPP_RULE_NONE;
    else if (gSamples < r.needed)    r.rule = PPP_RULE_ALL_TOO_FEW;
    else if (!(r.halfH <= r.limH && r.halfV <= r.limV)) r.rule = PPP_RULE_ALL_UNSTABLE;
    else                             r.rule = PPP_RULE_CONVERGED;
    return r;
}

// RULE CACHE (keep). evaluateRule() is a few hundred soft-float double operations on the
// ESP32-S3 (its FPU is single precision), and ppp_survey_status() is called several times
// per loop pass, all day, while the accumulators stay as the last survey left them. The rule
// depends only on the accumulators, which change only when a fix is averaged in, so it is
// evaluated once per averaged fix (end of the GGA accumulation) and in ppp_survey_begin(),
// and ppp_survey_status() copies the result. gRuleAtFixes is gAllSamples at that evaluation
// (every averaged fix increments it); currentRule() evaluates again if it is not current, so
// the lock always decides on a result that includes the latest fix.
static PppRuleEval   gRuleNow;
static uint32_t      gRuleAtFixes = UINT32_MAX;
static void refreshRule()
{
    gRuleNow = evaluateRule();
    gRuleAtFixes = gAllSamples;
}
static const PppRuleEval& currentRule()
{
    if (gRuleAtFixes != gAllSamples) refreshRule();
    return gRuleNow;
}

// ── Public API ───────────────────────────────────────────────────────────────

void ppp_survey_begin(Stream& gnssPort, uint32_t durationSeconds)
{
    gGnss       = &gnssPort;
    gDurationS  = durationSeconds ? durationSeconds : PPP_SURVEY_DEFAULT_DURATION_S;
    // gEpe2dLimit/gEpe3dLimit deliberately persist across begin() so an
    // operator-tuned criterion is not silently reverted by the next survey.

    gStepDelayMs = STEP_INTERVAL_MS;
    gSumLat = gSumLon = gSumAltEll = 0.0;
    gScatCount = 0; gScatHaveRef = false; gScatSigma = -1.0f;
    gAllSumLat = gAllSumLon = gAllSumAltEll = 0.0;
    gAllSamples = 0; gAllSumEpe2d = 0.0; gAllEpeCount = 0;
    gAllRefLat = gAllRefLon = 0.0;
    gAllSumN = gAllSumE = gAllSumN2 = gAllSumE2 = 0.0;
    gConvSumEpe2d = 0.0; gConvEpeCount = 0; gConvSumScat = 0.0; gConvScatCount = 0;
    gLastConverged = false; gLastPppSolution = false; gLockAccM = NAN;
    gLockRule = PPP_RULE_NONE;
    gHalfUsed = 0; gHalfBlockSize = 1;
    gConvRefLat = gConvRefLon = gConvRefH = 0.0;
    gEchoFirstLat = gEchoFirstLon = 0.0; gEchoAllSame = false; gEchoFault = false;
    gEchoQ7Fixes = 0; gLastQ7Ms = 0;
    gLockUsedConverged = false;
    gLockHaveCoord = false; gLockLat = gLockLon = gLockAltEll = 0.0;
    gWindowS = 0; gFinishedEarly = false;
    gLastFixMs = 0; gSignalLost = false;
    gFinishReqMs = 0; gAwaitingFix = false; gFinishLapses = 0;
    gRateGgaCount = gRateFirstMs = gRateLastMs = 0;
    gSamples = gValidFixes = 0;
    gEpe2d = gEpe3d = -1.0f;
    gBestEpe2d = gBestEpe3d = NAN;
    gHaveEpe = gHaveGga = false;
    gEpeMs = 0;
    gGgaLat = gGgaLon = gGgaAltEll = NAN;
    gE6Sats = gE6TmpCount = 0;
    gE6AvgCnr = gE6TmpCnrSum = 0.0f;
    gE6LastMs = 0;
    gEcefX = gEcefY = gEcefZ = 0.0;

    refreshRule();   // RULE CACHE: the empty accumulators' rule

    gStep = 0;
    gStepMs = millis();
    gSettleStartMs = 0;
    gState = PPP_CONFIGURING;
}

void ppp_survey_abort()
{
    if (gState == PPP_SURVEYING && gSurveyStartMs) gWindowS = (millis() - gSurveyStartMs) / 1000UL;
    if (gState == PPP_CONFIGURING || gState == PPP_SURVEYING || gState == PPP_LOCKING)
        gState = PPP_FAILED;
    gFinishReqMs = 0;
    gAwaitingFix = false;
}

bool ppp_survey_finish_now()
{
    // Only from the averaging window: before it (CONFIGURING) nothing has been collected,
    // and from LOCKING on the lock sequence is already writing the module. Too few fixes
    // averaged is refused rather than locked, and the survey keeps running.
    if (gState != PPP_SURVEYING || gAllSamples < PPP_SURVEY_MIN_LOCK_FIXES) return false;
    const uint32_t now = millis();
    // WINDOW ALREADY OVER: its end is already being handled (PPP_SURVEYING case in
    // ppp_survey_tick()) — locked at the next tick with a fresh fix, otherwise inside the
    // window-end grace, which locks at the first fresh fix within it. An early end adds
    // nothing to that: it does not restart the grace and it cannot lapse.
    if (now - gSurveyStartMs >= gDurationS * 1000UL) return true;
    if (gLastFixMs == 0 || now - gLastFixMs >= PPP_SURVEY_FIX_STALE_MS) {
        // No fresh fix: the same grace the window's end gets (PPP_SURVEYING case in
        // ppp_survey_tick()), which locks at the first fresh fix within it.
        if (gFinishReqMs == 0) gFinishReqMs = now ? now : 1;
        return true;
    }
    gWindowS       = gSurveyStartMs ? (now - gSurveyStartMs) / 1000UL : 0;
    gFinishedEarly = true;
    gFinishReqMs   = 0;       // an earlier request still waiting is answered by this lock
    gAwaitingFix   = false;
    // Same transition the window's expiry makes (PPP_SURVEYING case in ppp_survey_tick()),
    // so the echo check, the converged-or-all mean and the verified lock sequence all apply
    // unchanged.
    gStep   = 0;
    gStepMs = now;
    gState  = PPP_LOCKING;
    return true;
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
        gEpeMs = millis();
        if (gHaveEpe) {  // track best-so-far for the dashboard progress read
            if (isnan(gBestEpe2d) || gEpe2d < gBestEpe2d) gBestEpe2d = gEpe2d;
            if (isnan(gBestEpe3d) || gEpe3d < gBestEpe3d) gBestEpe3d = gEpe3d;
        }
        return;
    }

    // ---- GGA (any talker): $xxGGA,time,lat,NS,lon,EW,q,nsat,hdop,alt,M,geoidSep,M,... ----
    if (strlen(sentence) > 6 && strncmp(sentence + 3, "GGA", 3) == 0) {
        char lat[16], ns[4], lon[16], ew[4], q[4], alt[16], sep[16], sta[8];
        nmeaField(sentence, 2, lat, sizeof lat);
        nmeaField(sentence, 3, ns,  sizeof ns);
        nmeaField(sentence, 4, lon, sizeof lon);
        nmeaField(sentence, 5, ew,  sizeof ew);
        nmeaField(sentence, 6, q,   sizeof q);
        nmeaField(sentence, 9, alt, sizeof alt);
        nmeaField(sentence, 11, sep, sizeof sep);
        nmeaField(sentence, 14, sta, sizeof sta);   // differential station ID; empty on most fixes

        // Every GGA counts toward the measured interval, fix or not: the receiver emits one
        // per epoch whatever its fix quality.
        {
            const uint32_t t = millis();
            if (t - gSurveyStartMs < PPP_SURVEY_RATE_PROBE_MS) {
                if (gRateGgaCount == 0) gRateFirstMs = t;
                gRateLastMs = t;
                gRateGgaCount++;
            }
        }

        int quality = q[0] ? atoi(q) : 0;
        if (quality <= 0) { gHaveGga = false; return; }  // no fix this epoch
        // BASE ECHO, NEVER A SAMPLE (keep). See gEchoQ7Fixes. Not counted as a fix either:
        // it says nothing about where the antenna is.
        if (quality == 7) {
            gEchoQ7Fixes++;
            gLastQ7Ms = millis();
            gHaveGga = false;
            return;
        }

        double dLat = nmeaDegrees(lat, ns);
        double dLon = nmeaDegrees(lon, ew);
        // isfinite, not just isnan: atof() reads "inf" as infinity, and one infinite fix would
        // make every later mean infinite.
        if (!isfinite(dLat) || !isfinite(dLon) || !alt[0]) { gHaveGga = false; return; }

        // NO SEPARATION, NO SAMPLE (keep). Without GGA field 11 the ellipsoidal height
        // cannot be formed; treating MSL as ellipsoidal put the locked reference ~36 m
        // below the antenna here (sim: first boot with the field missing). With nothing
        // averaged, "Lock survey now" refuses in the same case. The survey then ends without fixes and takes the
        // no-fix retry path rather than locking a wrong height.
        if (!sep[0]) {
            static bool warned = false;
            if (!warned) { warned = true; Serial.println("⚠️  PPP: GGA has no geoid separation - fixes not used for the survey"); }
            gHaveGga = false;
            return;
        }
        double mslAlt   = atof(alt);
        double geoidSep = atof(sep);
        // GGA altitude is orthometric (MSL); ECEF needs ELLIPSOIDAL height.
        double hEll = mslAlt + geoidSep;
        if (!isfinite(hEll)) { gHaveGga = false; return; }

        gGgaLat = dLat; gGgaLon = dLon; gGgaAltEll = hEll;
        gHaveGga = true;
        gValidFixes++;
        gLastFixMs = millis();

        // Unfiltered running mean — the always-available fallback coordinate.
        if (gAllSamples == 0) {
            gEchoFirstLat = dLat; gEchoFirstLon = dLon; gEchoAllSame = true;
        } else if (dLat != gEchoFirstLat || dLon != gEchoFirstLon) {
            gEchoAllSame = false;
        }
        gAllSumLat += dLat; gAllSumLon += dLon; gAllSumAltEll += hEll;
        if (gAllSamples == 0) { gAllRefLat = dLat; gAllRefLon = dLon; }
        {   // position scatter of the all-fix set (see the AT LOCK note above)
            const double n = (dLat - gAllRefLat) * 111320.0;
            const double e = (dLon - gAllRefLon) * 111320.0 * cos(gAllRefLat * M_PI / 180.0);
            gAllSumN += n; gAllSumE += e; gAllSumN2 += n * n; gAllSumE2 += e * e;
        }
        gAllSamples++;
        const bool epeNow = epeFresh();   // a stale EPE is not used (see gEpeMs)
        if (epeNow && gEpe2d > 0.0f) { gAllSumEpe2d += gEpe2d; gAllEpeCount++; }

        // PPP SOLUTION OR NOT. Quality 5 (the code this receiver reports for a converged PPP
        // float), or the PPP service's station ID in field 14 (9002 = Galileo E6 HAS, 9001 =
        // BDS B2b), which the vendor note gives as the way to confirm a PPP solution.
        const int station = sta[0] ? atoi(sta) : -1;
        const bool pppSolution = (quality == 5) || station == 9001 || station == 9002;
        gLastPppSolution = pppSolution;

        // Feed the scatter window — PPP-solution fixes only, and only consecutive ones: a
        // fix that is not a PPP solution empties the window, so its sigma always describes
        // an unbroken run of the PPP solution (a run of autonomous fixes can be just as tight
        // over a minute and say nothing about convergence). Offsets are metres from the first
        // fix pushed, which only has to be a stable local origin — the sigma is about spread.
        if (!pppSolution) {
            gScatCount = 0;
            gScatSigma = -1.0f;
        } else {
            if (!gScatHaveRef) { gScatRefLat = dLat; gScatRefLon = dLon; gScatHaveRef = true; }
            const uint32_t i = gScatCount % PPP_SURVEY_SCATTER_WINDOW;
            gScatN[i] = (dLat - gScatRefLat) * 111320.0;
            gScatE[i] = (dLon - gScatRefLon) * 111320.0 * cos(gScatRefLat * M_PI / 180.0);
            gScatCount++;
            gScatSigma = scatterSigmaM();
        }

        // Accept this fix into the converged set if it is a PPP solution AND either
        // convergence test passes.
        //
        // Scatter is measured here, has no floor, and is what an accuracy estimate is trying
        // to approximate. The EPE test is the second route, so a receiver that DOES report
        // honest sub-limit accuracy is believed without waiting for a full scatter window.
        const bool epeConverged = epeNow &&
                                  gEpe2d <= gEpe2dLimit &&
                                  gEpe3d <= gEpe3dLimit;
        const bool scatterConverged = gScatSigma >= 0.0f &&
                                      gScatSigma <= PPP_SURVEY_SCATTER_LIMIT_M;
        gLastConverged = pppSolution && (epeConverged || scatterConverged);
        if (gLastConverged) {
            if (gSamples == 0) { gConvRefLat = dLat; gConvRefLon = dLon; gConvRefH = hEll; }
            halfAdd((float)((dLat - gConvRefLat) * 111320.0),
                    (float)((dLon - gConvRefLon) * 111320.0 * cos(gConvRefLat * M_PI / 180.0)),
                    (float)(hEll - gConvRefH));
            gSumLat    += dLat;
            gSumLon    += dLon;
            gSumAltEll += hEll;
            gSamples++;
            if (epeNow && gEpe2d > 0.0f) { gConvSumEpe2d += gEpe2d; gConvEpeCount++; }
            else if (gScatSigma >= 0.0f)   { gConvSumScat  += gScatSigma; gConvScatCount++; }
        }
        refreshRule();   // RULE CACHE: once per averaged fix
        return;
    }
}

void ppp_survey_tick()
{
    uint32_t now = millis();

    switch (gState) {

    case PPP_CONFIGURING: {
        // ┌─ NOTES ON THIS COMMAND SEQUENCE ─────────────────────────────────────────┐
        // │ The commands below are used as field-proven on this hardware and         │
        // │ firmware. Two points differ from a plain reading of the published spec:  │
        // │                                                                          │
        // │  1. The spec says a PQTMCFGRCVRMODE change requires PQTMSAVEPAR followed │
        // │     by a reset or "it will continue to operate in the original mode".    │
        // │     No reset is issued here. The caller establishes rover mode first     │
        // │     (servicePppSurveyStart's escape, checked by GGA quality and MSM      │
        // │     observation), so this sequence does not depend on the mode write     │
        // │     taking effect by itself. The one save (step 3) follows PQTMCFGPPP,   │
        // │     as the PPP application note requires. An earlier build that added a  │
        // │     save and reset to this sequence was recorded as breaking a working   │
        // │     survey.                                                              │
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
        case 1:  // 1 Hz fix rate — see PPP_SURVEY_FIX_INTERVAL_MS in ppp_survey.h.
            // Directly after the rover-mode write and before the save below, so the saved
            // rover configuration carries 1 Hz as well as the running one.
            snprintf(cmd, sizeof cmd, "PQTMCFGFIXRATE,W,%d", PPP_SURVEY_FIX_INTERVAL_MS);
            sendPqtm(cmd);
            break;
        case 2:  // enable PPP: mode,datum,timeout,hAcc,vAcc
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
        case 3:  // SAVE the PPP setting — the vendor procedure requires it.
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
        case 4:  // make sure PQTMEPE is output (our convergence gate) — 1/fix
            sendPqtm("PQTMCFGMSGRATE,W,PQTMEPE,1");
            break;
        case 5:  // make sure GGA is output (our position source) — 1/fix
            sendPqtm("PQTMCFGMSGRATE,W,GGA,1");
            break;
        case 6:  // make sure GSV is output (our E6 reception monitor) — 1/fix
            sendPqtm("PQTMCFGMSGRATE,W,GSV,1");
            break;
        case 7:  // Permissive elevation mask — see PPP_SURVEY_ELE_MASK_DEG.
            // Added after the proven steps above, not woven into them: these masks live
            // in module NVM and a tight rover mask left over from other use would starve
            // this survey. Nothing above this line is modified.
            snprintf(cmd, sizeof cmd, "PQTMCFGELETHD,W,%.1f", (double)PPP_SURVEY_ELE_MASK_DEG);
            sendPqtm(cmd);
            break;
        case 8:  // Permissive C/N0 mask — see PPP_SURVEY_CNR_MASK_DBHZ.
            snprintf(cmd, sizeof cmd, "PQTMCFGCNRTHD,W,%.1f", (double)PPP_SURVEY_CNR_MASK_DBHZ);
            sendPqtm(cmd);
            break;
        case 9:  // PQTMPPPNAV — the module's own report of its PPP solution.
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
        case 10: // Read the fix interval back. The reply lands in the sketch's
            // PQTMCFGFIXRATE parser (g_probedFixRateMs), which logs it when the window opens.
            sendPqtm("PQTMCFGFIXRATE,R");
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
        const uint32_t durMs = gDurationS * 1000UL;
        const bool windowOver = (now - gSurveyStartMs >= durMs);
        if (!windowOver && gFinishReqMs == 0) return;
        // SIGNAL GUARD (keep). The lock sequence writes base mode and a coordinate and
        // restarts the receiver, and none of its writes is acknowledged; a receiver that has
        // stopped producing fixes (link or antenna lost mid-window, as in field session 0087)
        // is not listening either, and the survey would report DONE on a module that took
        // none of it. So the lock starts only with a fix newer than PPP_SURVEY_FIX_STALE_MS.
        // Fixes that stop partway and RESUME before the end leave the average intact, so only
        // the state at the window's end decides — with a grace: a gap that straddles the end
        // waits up to PPP_SURVEY_END_GRACE_MS for a fresh fix (still averaging) rather than
        // discarding the window.
        const bool fresh = gLastFixMs != 0 && now - gLastFixMs < PPP_SURVEY_FIX_STALE_MS;
        if (fresh) {
            gAwaitingFix   = false;
            gFinishedEarly = !windowOver;
            const uint32_t ranS = (now - gSurveyStartMs) / 1000UL;
            gWindowS       = (windowOver && ranS < gDurationS) ? gDurationS : ranS;
            gFinishReqMs   = 0;
            gStep = 0;
            gStepMs = now;
            gState = PPP_LOCKING;
            return;
        }
        // Once the window is over, its own end governs: an operator's early end still waiting
        // is superseded by it, so it neither shortens the window's grace nor lapses into a
        // window that has already ended.
        if (windowOver) gFinishReqMs = 0;
        // A receiver answering with base echoes (quality 7) is not a lost signal: it is a base
        // that did not leave base mode. That is an echo fault, which the sketch reports as
        // such and retries through the forced mode escape; no grace, the receiver is talking.
        // The same holds for a survey that saw nothing BUT echoes.
        const bool echoing = (gLastQ7Ms != 0 && now - gLastQ7Ms < PPP_SURVEY_FIX_STALE_MS) ||
                             (gAllSamples == 0 && gEchoQ7Fixes > 0);
        // The moment this window's end was reached: the operator's early end while one is
        // pending, otherwise the configured end.
        const uint32_t endMs = gFinishReqMs ? gFinishReqMs : gSurveyStartMs + durMs;
        if (!echoing && now - endMs < PPP_SURVEY_END_GRACE_MS) { gAwaitingFix = true; return; }
        gAwaitingFix = false;
        if (gFinishReqMs != 0 && !echoing) {
            // The operator's early end found no fix within the grace: it lapses and the
            // survey runs on to its configured end (which applies the same rule).
            gFinishReqMs = 0;
            gFinishLapses++;
            return;
        }
        gFinishReqMs = 0;
        gWindowS = windowOver ? gDurationS : (now - gSurveyStartMs) / 1000UL;
        if (echoing) gEchoFault = true;
        else         gSignalLost = (gAllSamples > 0);   // zero fixes is the no-sky failure, reported as such
        gState = PPP_FAILED;
        return;
    }

    case PPP_LOCKING: {
        // First entry (gStep==0): decide pass/fail and compute the coordinate.
        if (gStep == 0) {
            // An echoed coordinate is refused first (echo detector above). The coordinate is
            // then decided by the AVERAGING POLICY at the accumulators (evaluateRule()): the
            // converged set once it has run long enough and agrees with itself, otherwise
            // the mean of every non-echo fix. Only a survey with no valid fix at all fails
            // here.
            if ((gEchoAllSame && gAllSamples >= 2) || (gAllSamples == 0 && gEchoQ7Fixes > 0)) {
                gState    = PPP_FAILED;
                gEchoFault = true;   // surfaced via ppp_survey_status().echoFault
                return;
            }
            if (gAllSamples == 0) {
                gState = PPP_FAILED;   // no valid fix at all: no sky, nothing to lock
                return;
            }
            const PppRuleEval& ev = currentRule();   // RULE CACHE: includes the latest fix
            gLockRule = ev.rule;
            gLockUsedConverged = (ev.rule == PPP_RULE_CONVERGED);
            if (gLockUsedConverged) {
                gLockAccM = ev.convMeanEpe;
            } else {
                const double nA = (double)gAllSamples;
                const double varN = gAllSumN2 / nA - (gAllSumN / nA) * (gAllSumN / nA);
                const double varE = gAllSumE2 / nA - (gAllSumE / nA) * (gAllSumE / nA);
                const double v = varN + varE;
                const double scat = (gAllSamples >= 2 && v > 0.0) ? sqrt(v) : 0.0;
                const double epe  = gAllEpeCount ? gAllSumEpe2d / (double)gAllEpeCount : 0.0;
                const double acc  = scat > epe ? scat : epe;
                gLockAccM = acc > 0.0 ? (float)acc : NAN;
            }
            const uint32_t n  = gLockUsedConverged ? gSamples    : gAllSamples;
            const double sLat = gLockUsedConverged ? gSumLat     : gAllSumLat;
            const double sLon = gLockUsedConverged ? gSumLon     : gAllSumLon;
            const double sAlt = gLockUsedConverged ? gSumAltEll  : gAllSumAltEll;
            double meanLat = sLat / (double)n;
            double meanLon = sLon / (double)n;
            double meanH   = sAlt / (double)n;
            geodeticToEcef(meanLat, meanLon, meanH, gEcefX, gEcefY, gEcefZ);
            gLockLat = meanLat; gLockLon = meanLon; gLockAltEll = meanH; gLockHaveCoord = true;
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
            rcxReceiverRestartCommanded();   // a planned restart: see ppp_survey.h
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

void ppp_survey_set_criteria(float epe2dLimitM, float epe3dLimitM, uint32_t durationSeconds)
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
    // A non-finite limit is ignored like a zero one (the status JSON prints these as numbers).
    if (!isfinite(epe2dLimitM)) epe2dLimitM = 0.0f;
    if (!isfinite(epe3dLimitM)) epe3dLimitM = 0.0f;
    if (epe2dLimitM > 0.0f)  gEpe2dLimit = epe2dLimitM;
    if (epe3dLimitM > 0.0f)  gEpe3dLimit = epe3dLimitM;
    else if (epe2dLimitM > 0.0f) gEpe3dLimit = epe2dLimitM * PPP_SURVEY_EPE_3D_RATIO;
    if (durationSeconds > 0) {
        // LENGTHENED DURING A GRACE. With the window's end moved past now, the window is open
        // again: the wait for a fresh fix ends, an operator's early end still waiting is
        // withdrawn with it (lengthening is the later instruction), and the survey averages on
        // to the new end, which applies the same end-of-window rule.
        if (gState == PPP_SURVEYING && durationSeconds > gDurationS &&
            (gAwaitingFix || gFinishReqMs != 0) &&
            millis() - gSurveyStartMs < durationSeconds * 1000UL) {
            gAwaitingFix = false;
            gFinishReqMs = 0;
        }
        gDurationS = durationSeconds;   // window is start + duration
    }
}

void ppp_survey_get_criteria(float& epe2dLimitM, float& epe3dLimitM, uint32_t& durationSeconds)
{
    epe2dLimitM = gEpe2dLimit;
    epe3dLimitM = gEpe3dLimit;
    durationSeconds = gDurationS;
}

PppSurveyStatus ppp_survey_status()
{
    PppSurveyStatus s;
    s.state       = gState;
    s.durationS   = gDurationS;
    s.elapsedS    = (gState == PPP_SURVEYING && gSurveyStartMs)
                        ? (millis() - gSurveyStartMs) / 1000UL
                        : (gState >= PPP_LOCKING ? gWindowS : 0);
    s.finishedEarly = gFinishedEarly;
    s.signalLost    = gSignalLost;
    s.awaitingFix   = gAwaitingFix;
    s.finishLapses  = gFinishLapses;
    s.samples     = gSamples;
    s.validFixes  = gValidFixes;
    // A stale EPE is reported as none (see gEpeMs).
    s.haveEpe     = epeFresh();
    s.lastEpe2d   = s.haveEpe ? gEpe2d : -1.0f;
    s.lastEpe3d   = s.haveEpe ? gEpe3d : -1.0f;
    s.bestEpe2d   = gBestEpe2d;
    s.bestEpe3d   = gBestEpe3d;
    s.converged   = gLastConverged;
    s.pppSolution = gLastPppSolution;
    s.rateGgaCount   = gRateGgaCount;
    s.rateIntervalMs = gRateGgaCount >= 2 ? (int32_t)((gRateLastMs - gRateFirstMs) / (gRateGgaCount - 1)) : -1;
    s.rateProbeDone  = gState != PPP_SURVEYING || millis() - gSurveyStartMs >= PPP_SURVEY_RATE_PROBE_MS;

    // E6 reception, with a staleness guard: if the E6 GSV group hasn't arrived in
    // ~3 s, treat E6 as not being received (0 sats) rather than showing stale data.
    bool e6Stale = (gE6LastMs == 0) || (millis() - gE6LastMs > 3000);
    s.e6Sats   = e6Stale ? 0 : gE6Sats;
    s.e6AvgCnr = (e6Stale || gE6Sats == 0) ? NAN : gE6AvgCnr;
    s.e6Ok     = (s.e6Sats > 0) && !isnan(s.e6AvgCnr) &&
                 (s.e6AvgCnr >= PPP_SURVEY_E6_CNR_FLOOR);

    s.samplesAll        = gAllSamples;
    s.echoFault         = gEchoFault;
    s.echoFixes         = gEchoQ7Fixes;
    s.lockUsedConverged = gLockUsedConverged;
    s.lockAccM          = gLockAccM;
    {   // The rule: decided at the lock, live before it (see PppSurveyStatus.rule), copied
        // from the RULE CACHE (evaluated here only before the first survey of a boot).
        const PppRuleEval& ev = currentRule();
        s.rule         = gState >= PPP_LOCKING && gLockHaveCoord ? gLockRule : ev.rule;
        s.nonConverged = ev.nonConverged;
        s.convNeeded   = ev.needed;
        s.convMeanEpe  = ev.convMeanEpe;
        s.halfH = ev.halfH; s.halfV = ev.halfV; s.halfLimH = ev.limH; s.halfLimV = ev.limV;
        s.convToAllH = ev.convToAllH; s.convToAllV = ev.convToAllV;
    }
    s.meanEpe2dAll      = gAllEpeCount ? (float)(gAllSumEpe2d / (double)gAllEpeCount) : NAN;
    s.meanLat     = gSamples ? gSumLat / (double)gSamples : NAN;
    s.meanLon     = gSamples ? gSumLon / (double)gSamples : NAN;
    s.meanAltEll  = gSamples ? gSumAltEll / (double)gSamples : NAN;
    // Only while locking or locked: a survey aborted during LOCKING has decided a
    // coordinate but never locked it, and must not be shown as "locked".
    const bool lockShown = gLockHaveCoord && (gState == PPP_LOCKING || gState == PPP_DONE);
    s.haveLockCoord = lockShown;
    s.lockLat     = lockShown ? gLockLat : NAN;
    s.lockLon     = lockShown ? gLockLon : NAN;
    s.lockAltEll  = lockShown ? gLockAltEll : NAN;
    s.ecefX = gEcefX; s.ecefY = gEcefY; s.ecefZ = gEcefZ;
    return s;
}

// Fix count as survey time (1 Hz: one fix per second), "9:40" or "1:24:40".
static void fixTime(uint32_t fixes, char* buf, size_t len)
{
    const uint32_t h = fixes / 3600UL, m = (fixes % 3600UL) / 60UL, sec = fixes % 60UL;
    if (h) snprintf(buf, len, "%lu:%02lu:%02lu", (unsigned long)h, (unsigned long)m, (unsigned long)sec);
    else   snprintf(buf, len, "%lu:%02lu", (unsigned long)m, (unsigned long)sec);
}

void ppp_survey_rule_text(const PppSurveyStatus& st, char* buf, size_t len)
{
    if (!buf || !len) return;
    char have[16], need[16];
    fixTime(st.samples, have, sizeof have);
    fixTime(st.convNeeded, need, sizeof need);
    // The component that failed the stability gate: horizontal unless only vertical did.
    const bool vert = !(st.halfV <= st.halfLimV) && (st.halfH <= st.halfLimH);
    switch (st.rule) {
    case PPP_RULE_CONVERGED:
        snprintf(buf, len, "converged PPP mean: PPP converged for %s (%s needed), halves %.2f m apart (limit %.2f m)",
                 have, need, (double)st.halfH, (double)st.halfLimH);
        break;
    case PPP_RULE_ALL_TOO_FEW:
        snprintf(buf, len, "all-fix mean: converged PPP %s of %s needed", have, need);
        break;
    case PPP_RULE_ALL_UNSTABLE:
        snprintf(buf, len, "all-fix mean: converged PPP halves %.2f m apart%s (limit %.2f m)",
                 (double)(vert ? st.halfV : st.halfH), vert ? " vertically" : "",
                 (double)(vert ? st.halfLimV : st.halfLimH));
        break;
    default:
        snprintf(buf, len, "no fixes averaged yet");
        break;
    }
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
               (double)gEpe2dLimit, (double)gEpe3dLimit, (unsigned long)s.convNeeded,
               (double)PPP_SURVEY_SCATTER_LIMIT_M);
    // Which mean the lock uses and why (AVERAGING POLICY): rule is "ppp", "all-few",
    // "all-unstable" or "none"; nonconv the fixes that did not converge; halfh/halfv the
    // converged halves' disagreement against halflimh/halflimv; cvallh/cvallv the converged
    // mean's distance from the all-fix mean; why the same in one sentence.
    {
        static const char* const kRule[] = { "none", "ppp", "all-few", "all-unstable" };
        char why[160];
        ppp_survey_rule_text(s, why, sizeof why);
        PPP_APPEND(",\"rule\":\"%s\",\"nonconv\":%lu,\"why\":\"%s\"",
                   kRule[(int)s.rule], (unsigned long)s.nonConverged, why);
        const float nums[6] = { s.halfH, s.halfV, s.halfLimH, s.halfLimV, s.convToAllH, s.convToAllV };
        static const char* const kNum[6] = { "halfh", "halfv", "halflimh", "halflimv", "cvallh", "cvallv" };
        for (int i = 0; i < 6; ++i) {
            if (isfinite(nums[i])) PPP_APPEND(",\"%s\":%.3f", kNum[i], (double)nums[i]);
            else                   PPP_APPEND(",\"%s\":null", kNum[i]);
        }
    }
    // Every number below is guarded with isfinite(): printf writes NaN and infinity as nan /
    // inf, which is not JSON, and one such token leaves the /ppp page with nothing to show.
    if (isfinite(gScatSigma) && gScatSigma >= 0.0f) PPP_APPEND(",\"scat\":%.3f", (double)gScatSigma);
    else                    PPP_APPEND(",\"scat\":null");
    if (isfinite(s.lastEpe2d) && s.lastEpe2d >= 0.0f) PPP_APPEND(",\"epe2d\":%.2f", s.lastEpe2d); else PPP_APPEND(",\"epe2d\":null");
    if (isfinite(s.lastEpe3d) && s.lastEpe3d >= 0.0f) PPP_APPEND(",\"epe3d\":%.2f", s.lastEpe3d); else PPP_APPEND(",\"epe3d\":null");
    if (isfinite(s.bestEpe2d)) PPP_APPEND(",\"best2d\":%.2f", s.bestEpe2d); else PPP_APPEND(",\"best2d\":null");
    if (isfinite(s.bestEpe3d)) PPP_APPEND(",\"best3d\":%.2f", s.bestEpe3d); else PPP_APPEND(",\"best3d\":null");
    PPP_APPEND(",\"conv\":%s,\"pppsol\":%s", s.converged ? "true" : "false", s.pppSolution ? "true" : "false");

    PPP_APPEND(",\"e6sats\":%lu", (unsigned long)s.e6Sats);
    if (isfinite(s.e6AvgCnr)) PPP_APPEND(",\"e6cnr\":%.1f", s.e6AvgCnr); else PPP_APPEND(",\"e6cnr\":null");
    PPP_APPEND(",\"e6ok\":%s", s.e6Ok ? "true" : "false");

    if (isfinite(s.meanLat) && isfinite(s.meanLon) && isfinite(s.meanAltEll))
        PPP_APPEND(",\"lat\":%.9f,\"lon\":%.9f,\"alt\":%.3f", s.meanLat, s.meanLon, s.meanAltEll);   // 9 decimals, as the /ppp page shows
    else
        PPP_APPEND(",\"lat\":null,\"lon\":null,\"alt\":null");

    // The locked coordinate and which set it came from (lkconv false = the unconverged
    // all-fix mean); null until the lock decides.
    if (s.haveLockCoord)
        PPP_APPEND(",\"lklat\":%.9f,\"lklon\":%.9f,\"lkalt\":%.3f,\"lkconv\":%s",
                   s.lockLat, s.lockLon, s.lockAltEll, s.lockUsedConverged ? "true" : "false");
    else
        PPP_APPEND(",\"lklat\":null,\"lklon\":null,\"lkalt\":null,\"lkconv\":null");

    if (s.ecefX != 0.0 && isfinite(s.ecefX) && isfinite(s.ecefY) && isfinite(s.ecefZ))
        PPP_APPEND(",\"x\":%.4f,\"y\":%.4f,\"z\":%.4f", s.ecefX, s.ecefY, s.ecefZ);
    else
        PPP_APPEND(",\"x\":null,\"y\":null,\"z\":null");

    PPP_APPEND("}");
    #undef PPP_APPEND
}
