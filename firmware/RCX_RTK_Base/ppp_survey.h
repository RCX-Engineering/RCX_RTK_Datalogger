#pragma once
/*
 * ppp_survey.h — Manual PPP "survey-in" for the RCX1 base station
 * ==============================================================
 * PURPOSE
 *   An alternative to the LG290P's *internal* survey-in (PQTMCFGSVIN mode 1).
 *   Instead of letting the module average autonomous single-point fixes, this
 *   runs the receiver as a ROVER with PPP (Galileo E6 HAS) enabled, sits for a
 *   user-set duration (e.g. 15 min), averages the PPP-converged position, then
 *   LOCKS that coordinate into the module as a FIXED base (PQTMCFGSVIN mode 2)
 *   and switches to base mode. Result: a base that sits at its true absolute
 *   position (~decimetre) instead of survey-in's ~1–2 m.
 *
 * WHY PPP AND BASE MODE ARE SEPARATE PHASES (do not merge them)
 *   Base mode (PQTMCFGRCVRMODE,W,2) auto-disables NMEA and only emits RTCM — it
 *   does NOT report a position. PPP is a rover-mode positioning function. So the
 *   only way to "survey on PPP" is: converge in rover mode, capture, THEN switch
 *   to base. This module sequences exactly that.
 *
 * WHY WE READ POSITION FROM GGA, NOT PQTMPPPNAV
 *   The PPP position also comes out in standard NMEA (GGA), whose field layout is
 *   documented and stable. PQTMPPPNAV's field layout is only in the newer (v1.2.0)
 *   Quectel spec that isn't public, so we deliberately avoid parsing it. GGA gives
 *   us lat/lon + MSL altitude + geoid separation, which is all we need.
 *
 * WHY WE GATE ON PQTMEPE
 *   "Good position" = converged accuracy, not a fix-quality code (PPP's GGA quality
 *   digit is ambiguous). PQTMEPE reports estimated 2D/3D error in metres every
 *   epoch; a fix is only averaged in once its EPE is inside the limits below.
 *
 * INTEGRATION (mirrors bridge_sd_log's decoupled pattern)
 *   1. ppp_survey_begin(gnssSerial, 900);      // start a 15-minute PPP survey
 *   2. In your NMEA handler, alongside bridge_sdlog_feed_nmea():
 *          ppp_survey_feed_nmea(sentence);     // feed EVERY NMEA line
 *   3. In loop():  ppp_survey_tick();          // drives the state machine
 *   4. Poll ppp_survey_status() for the LCD / web dashboard.
 *
 *   On success the module reboots as a persistent fixed base (coordinate saved to
 *   its NVS via PQTMSAVEPAR) and your existing RTCM/NTRIP pipeline takes over.
 *
 * ⚠ BOOT-CONFIG INTERACTION: once locked, the fixed position lives in the LG290P's
 *   NVS. Make sure your boot-time configure path does NOT re-issue a survey-in
 *   (PQTMCFGSVIN,W,1,...) on warm boots, or it will overwrite the locked fix. Your
 *   existing hot-start/skip-reconfigure logic already handles this — just don't
 *   force a survey-in reconfigure after a successful PPP lock.
 *
 * ⚠ DOUBLE PRECISION REQUIRED: ECEF values are ~6.4e6 m; cm precision needs 64-bit
 *   doubles for the accumulate + geodetic→ECEF math. This module uses double
 *   throughout. Do not down-cast to float.
 *
 * ⚠ VERIFY the PPP enable command against YOUR firmware once. The $PQTMCFGPPP form
 *   used here (mode/datum/timeout/hAcc/vAcc) is taken from working field firmware;
 *   confirm the module answers "$PQTMCFGPPP,OK" on your v2.01 build.
 */

#include <Arduino.h>

// ── Tunables (here, not config.h, so config.h stays verbatim) ────────────────
#ifndef PPP_SURVEY_DEFAULT_DURATION_S
#define PPP_SURVEY_DEFAULT_DURATION_S   900     // 15 min default window
#endif
#ifndef PPP_SURVEY_EPE_2D_LIMIT_M
#define PPP_SURVEY_EPE_2D_LIMIT_M       0.30f   // only average fixes at/under this 2D EPE
#endif
#ifndef PPP_SURVEY_EPE_3D_LIMIT_M
#define PPP_SURVEY_EPE_3D_LIMIT_M       0.50f   // ...and this 3D EPE
#endif
#ifndef PPP_SURVEY_SCATTER_WINDOW
// Fixes in the sliding window used to measure convergence from position scatter. At the
// base's 1 Hz rate this is a one-minute window — long enough that correlated consecutive
// fixes cannot make it read artificially tight, short enough to follow a survey that is
// still converging.
#define PPP_SURVEY_SCATTER_WINDOW       60u
#endif
#ifndef PPP_SURVEY_SCATTER_LIMIT_M
// Convergence threshold on that window's sigma. Measured 60-second scatter on this
// hardware runs 0.055-0.18 m once settled and 0.28-0.49 m over a whole session, so this
// admits the settled state and excludes the drifting one.
#define PPP_SURVEY_SCATTER_LIMIT_M      0.25f
#endif
#ifndef PPP_SURVEY_EPE_3D_RATIO
// A 2D limit supplied on its own is scaled by this to give the matching 3D limit, so the
// two gates can never drift apart and quietly disagree about what counts as converged.
// Vertical error runs consistently larger than horizontal: measured across a 45-minute
// session (2,750 fixes) the 3D/2D ratio held between 1.75 and 2.30, median 1.94. 2.0 sits
// on that median, so a 2D limit set here admits the same fixes its 3D partner does rather
// than being overruled by it.
#define PPP_SURVEY_EPE_3D_RATIO         2.0f
#endif
#ifndef PPP_SURVEY_MIN_SAMPLES
#define PPP_SURVEY_MIN_SAMPLES          60      // need this many converged fixes to lock
#endif
#ifndef PPP_SURVEY_E6_CNR_FLOOR
// Average E6 C/N0 below this is reported as "E6 weak". This is a REPORTING threshold
// only — it colours the dashboard indicator and sets e6Ok; it never filters satellites
// out of the solution and never rejects a fix. 30 dB-Hz was far too stringent: E6 is a
// pilot/data signal routinely tracked in the teens, so a perfectly usable HAS stream was
// being flagged weak. 10 dB-Hz marks genuinely marginal reception instead.
#define PPP_SURVEY_E6_CNR_FLOOR         10.0f
#endif

// Satellite gating pushed to the module while this survey runs. These are the module's own
// documented defaults (elevation 5.0 deg, CNR 10.0 dB-Hz) and match BASE_ELEVATION_MASK_DEG
// / BASE_CNR_MASK_DBHZ, which carry the full rationale. They are written explicitly rather
// than assumed because these thresholds live in the module's NVM and a tight mask left over
// from other use would starve the survey.
#ifndef PPP_SURVEY_ELE_MASK_DEG
#define PPP_SURVEY_ELE_MASK_DEG         5.0f
#endif
#ifndef PPP_SURVEY_CNR_MASK_DBHZ
#define PPP_SURVEY_CNR_MASK_DBHZ        10.0f
#endif

// FIX INTERVAL FORCED FOR THE SURVEY — this is the one that made PPP work.
//
// The PPP engine needs a 1 Hz fix rate. Base station mode forces 1 Hz and does not allow it
// to be changed, which is why the only session where PPP has ever converged on this hardware
// (0081: 1.5 m to 5 cm in ~19 minutes) was running in BASE mode. A PPP survey runs the
// receiver as a ROVER, and the documented rover default is 10 Hz — so every survey since has
// been asking the PPP engine to converge at ten times the rate it supports, and none has.
// Nothing in this firmware had ever set the fix rate at all.
//
// Setting it here also restores the meaning of the scatter window below. That window is sized
// in FIXES and its length in seconds was reasoned about at 1 Hz; at the rover's 10 Hz default
// a 60-fix window spans six seconds of heavily correlated fixes, which reads far tighter than
// the true minute-scale scatter and would let a survey call itself converged on six seconds of
// noise. Both problems have the same one-line cause.
//
// No restore is needed after the survey: entering base mode returns the module to 1 Hz.
#ifndef PPP_SURVEY_FIX_INTERVAL_MS
#define PPP_SURVEY_FIX_INTERVAL_MS      1000
#endif

// PPP enable parameters → $PQTMCFGPPP,W,<mode>,<datum>,<timeout>,<hAcc>,<vAcc>
#ifndef PPP_SURVEY_PPP_MODE
#define PPP_SURVEY_PPP_MODE             2       // 2 = Galileo E6 HAS (the usable one in the US)
#endif
#ifndef PPP_SURVEY_PPP_DATUM
#define PPP_SURVEY_PPP_DATUM            1       // 1 = WGS84 (must match the base's ECEF datum)
#endif
#ifndef PPP_SURVEY_PPP_TIMEOUT_S
#define PPP_SURVEY_PPP_TIMEOUT_S        120     // module's own PPP fallback timeout
#endif
#ifndef PPP_SURVEY_PPP_HACC_M
#define PPP_SURVEY_PPP_HACC_M           0.10f   // module's horizontal convergence target
#endif
#ifndef PPP_SURVEY_PPP_VACC_M
#define PPP_SURVEY_PPP_VACC_M           0.15f   // module's vertical convergence target
#endif

enum PppSurveyState {
    PPP_IDLE = 0,
    PPP_CONFIGURING,   // sending rover+PPP+EPE setup commands
    PPP_SURVEYING,     // collecting converged fixes for the window
    PPP_LOCKING,       // averaging, converting, writing fixed base + save + reset
    PPP_DONE,          // locked; module rebooting as fixed base
    PPP_FAILED         // window ended without enough converged samples
};

struct PppSurveyStatus {
    PppSurveyState state;
    uint32_t elapsedS;        // seconds since survey window started
    uint32_t durationS;       // configured window length
    uint32_t samples;         // EPE-gated (PPP-converged) fixes accumulated
    uint32_t samplesAll;      // ALL valid fixes accumulated, gated or not
    bool     echoFault;       // survey failed because every fix was bit-identical — the
                              // receiver is echoing a stored coordinate (still in FIXED
                              // mode), not solving. Means the mode switch did not take.
    bool     lockUsedConverged;  // true if the locked coordinate came from the gated set;
                                 // false means PPP never converged and the unfiltered
                                 // autonomous mean was used instead (valid once PPP_DONE)
    float    meanEpe2dAll;    // mean 2D EPE across all valid fixes (m), NAN if none
    uint32_t validFixes;      // any valid GGA fix seen (for diagnostics)
    bool     haveEpe;
    float    lastEpe2d;       // most recent 2D EPE (m), -1 if none yet
    float    lastEpe3d;       // most recent 3D EPE (m), -1 if none yet
    float    bestEpe2d;       // best (lowest) 2D EPE seen this survey (m), NAN if none
    float    bestEpe3d;       // best (lowest) 3D EPE seen this survey (m), NAN if none
    bool     converged;       // latest EPE is within the averaging limits (PPP is helping)
    // ── E6 / HAS reception (the "is PPP data actually arriving" indicator) ──
    // Parsed live from $GAGSV signal-ID 5 (Galileo E6). If E6 is weak/absent at a
    // site, e6Sats drops and e6Ok goes false — your cue to abort and survey-in instead.
    uint32_t e6Sats;          // Galileo E6 satellites currently tracked
    float    e6AvgCnr;        // average C/N0 of those E6 signals (dB-Hz), NAN if none
    bool     e6Ok;            // heuristic: E6 reception healthy (sats>0 and C/N0 decent)
    double   meanLat;         // running mean latitude  (deg), NAN until first sample
    double   meanLon;         // running mean longitude (deg)
    double   meanAltEll;      // running mean ellipsoidal height (m) = MSL alt + geoid sep
    double   ecefX, ecefY, ecefZ;  // captured fixed-base coordinate (valid once PPP_DONE)
};

// Start a manual PPP survey. Pass the same serial stream the LG290P is on.
// durationSeconds == 0 → uses PPP_SURVEY_DEFAULT_DURATION_S.
void ppp_survey_begin(Stream& gnssPort, uint32_t durationSeconds);

// Feed EVERY NMEA sentence (GGA and PQTMEPE are consumed; others ignored).
// Cheap; safe to call for all lines like bridge_sdlog_feed_nmea().
void ppp_survey_feed_nmea(const char* sentence);

// Call from loop(). Drives the state machine and paces command sends.
void ppp_survey_tick();

// Abort an in-progress survey (leaves the module in rover+PPP; does not lock).
void ppp_survey_abort();

// True while configuring / surveying / locking.
bool ppp_survey_active();

// Change the acceptance criteria at ANY time, including while a survey is running.
// Zero for a parameter leaves it unchanged, EXCEPT that supplying a 2D limit with a zero
// 3D limit derives the 3D limit from it (see PPP_SURVEY_EPE_3D_RATIO) — a caller that
// knows only one of the two must not be able to leave the other contradicting it. Takes effect on the next epoch; accumulated
// samples are kept and the receiver is never touched, so this never restarts anything.
void ppp_survey_set_criteria(float epe2dLimitM, float epe3dLimitM,
                             uint32_t minSamples, uint32_t durationSeconds);

// Read back the criteria currently in force.
void ppp_survey_get_criteria(float& epe2dLimitM, float& epe3dLimitM,
                             uint32_t& minSamples, uint32_t& durationSeconds);

// Snapshot for LCD / web dashboard.
PppSurveyStatus ppp_survey_status();

// Human-readable state name (for the dashboard / logs).
const char* ppp_survey_state_name(PppSurveyState st);

// Serialize the current status as a compact JSON object into buf. Use this to
// answer a GET /api/ppp poll from the web dashboard (see the integration snippet).
void ppp_survey_status_json(char* buf, size_t len);
