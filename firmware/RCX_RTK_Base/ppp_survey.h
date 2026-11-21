#pragma once
/*
 * ppp_survey.h — Manual PPP "survey-in" for the RCX1 base station
 * ==============================================================
 * PURPOSE
 *   An alternative to the LG290P's *internal* survey-in (PQTMCFGSVIN mode 1).
 *   Instead of letting the module average autonomous single-point fixes, this
 *   runs the receiver as a ROVER with PPP (Galileo E6 HAS) enabled, sits for a
 *   user-set duration (default 60 min), averages the PPP-converged position, then
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
 * WHAT COUNTS AS CONVERGED
 *   A fix joins the converged set only if it is a PPP solution (GGA quality 5, or the
 *   PPP service's station ID 9001/9002 in GGA field 14) AND it passes a convergence test:
 *   PQTMEPE (estimated 2D/3D error, every epoch) inside the limits below, or the measured
 *   position scatter of the last minute of PPP-solution fixes inside its limit. Every
 *   other non-echo fix is averaged separately as the fallback. ppp_survey.cpp (AVERAGING
 *   POLICY) says which set the lock uses and what accuracy it records.
 *
 * INTEGRATION (mirrors bridge_sd_log's decoupled pattern)
 *   1. ppp_survey_begin(gnssSerial, 3600);     // start a 60-minute PPP survey
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
#define PPP_SURVEY_DEFAULT_DURATION_S   3600    // 60 min default window (DEFAULT_SURVEY_IN_SEC)
#endif
#ifndef PPP_SURVEY_EPE_2D_LIMIT_M
#define PPP_SURVEY_EPE_2D_LIMIT_M       0.30f   // only average fixes at/under this 2D EPE
#endif
#ifndef PPP_SURVEY_EPE_3D_LIMIT_M
#define PPP_SURVEY_EPE_3D_LIMIT_M       0.50f   // ...and this 3D EPE
#endif
#ifndef PPP_SURVEY_EPE_FRESH_MS
// A PQTMEPE older than this is not used. It arrives once per 1 Hz epoch, after that epoch's
// GGA, so the EPE in hand when a fix is averaged is normally about one second old.
#define PPP_SURVEY_EPE_FRESH_MS         2000u
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
// ── WHICH MEAN THE LOCK USES (AVERAGING POLICY in ppp_survey.cpp) ────────────
// The coordinate is the mean of every non-echo fix until converged PPP has run long enough
// to replace it; then it is the mean of the converged set alone. Counts are fixes, which
// at the survey's 1 Hz rate (PPP_SURVEY_FIX_INTERVAL_MS) are seconds.
#ifndef PPP_SURVEY_PPP_TIME_RATIO
// Converged PPP takes over once it holds at least one fix for every this-many fixes that
// were not converged (75 min of autonomous averaging: 15 min of converged PPP). The owner's
// estimate is that an autonomous average needs roughly ten times as long as converged PPP
// to be as good; a fifth keeps a factor-of-two margin on that, so PPP is at least as good
// as the average it replaces when it takes over.
#define PPP_SURVEY_PPP_TIME_RATIO       5u
#endif
#ifndef PPP_SURVEY_CONVERGED_FLOOR_FIXES
// Never fewer converged fixes than the convergence test itself is judged on (the scatter
// window), so the switch is never decided on less evidence than convergence was.
#define PPP_SURVEY_CONVERGED_FLOOR_FIXES PPP_SURVEY_SCATTER_WINDOW
#endif
#ifndef PPP_SURVEY_CONVERGED_CAP_FIXES
// Thirty minutes of converged PPP always suffices, however long the autonomous part ran.
// An autonomous average stops improving after an hour or two: its orbit, clock and
// atmosphere errors are correlated over hours, so it settles on a bias floor (roughly
// 0.5-1 m) rather than converging further. This is that bias-floor behaviour stated as a
// limit, not a precisely measured figure: past it, a longer autonomous run would only
// delay a coordinate that is already the better one.
#define PPP_SURVEY_CONVERGED_CAP_FIXES  1800u
#endif
// STABILITY GATE. Even with enough converged fixes, the converged set takes over only if it
// agrees with itself: the mean of its first half and the mean of its second half must lie
// within max(PPP_SURVEY_STABLE_EPE_MULT x the set's mean EPE, PPP_SURVEY_STABLE_MIN_M)
// horizontally, and PPP_SURVEY_STABLE_V_RATIO times that vertically. A set that still
// drifts (a convergence flagged early, a solution walking off) fails it, and the lock keeps
// the mean of all fixes and says why.
#ifndef PPP_SURVEY_STABLE_EPE_MULT
// Twice the set's own accuracy claim: two half-means of a settled solution sit well inside
// it, a drift of more than twice what the receiver claims does not.
#define PPP_SURVEY_STABLE_EPE_MULT      2.0f
#endif
#ifndef PPP_SURVEY_STABLE_MIN_M
// The horizontal limit never falls below 10 cm, the receiver's own PPP convergence target
// (PPP_SURVEY_PPP_HACC_M), so a set claiming 2 cm is not failed for the few centimetres of
// slow wander between halves that converged PPP shows.
#define PPP_SURVEY_STABLE_MIN_M         0.10f
#endif
#ifndef PPP_SURVEY_STABLE_V_RATIO
// Vertical limit as a multiple of the horizontal one. Vertical error runs larger than
// horizontal on this hardware: the 3D/2D EPE ratio measured 1.75-2.30 (median 1.94) over
// 2,750 fixes (PPP_SURVEY_EPE_3D_RATIO), which puts the vertical component at 1.4-2.1
// times the horizontal (1.7 at the median); the receiver's own PPP targets are 0.10 m
// horizontal and 0.15 m vertical. 2.0 covers the upper part of that range, so ordinary
// vertical wander does not fail a set whose horizontal agrees, while a vertical drift
// several times it does.
#define PPP_SURVEY_STABLE_V_RATIO       2.0f
#endif
#ifndef PPP_SURVEY_HALF_BLOCKS
// The halves are measured from the converged set's fixes summed in this many consecutive
// blocks (pairs merge when they fill), so the split falls within 1/32 of the set's size of
// its middle on any survey length, in 512 bytes.
#define PPP_SURVEY_HALF_BLOCKS          32u
#endif
#ifndef PPP_SURVEY_MIN_LOCK_FIXES
// Fewest fixes ppp_survey_finish_now() will lock. Thirty seconds of averaging at 1 Hz, and
// enough samples for the echo check (which needs two) to mean something.
#define PPP_SURVEY_MIN_LOCK_FIXES       30u
#endif
#ifndef PPP_SURVEY_FIX_STALE_MS
// The lock needs a fix newer than this (signal guard in ppp_survey_tick()). Ten 1 Hz epochs:
// long enough that one dropped sentence or a short UART stall is not mistaken for a lost
// receiver.
#define PPP_SURVEY_FIX_STALE_MS         10000u
#endif
#ifndef PPP_SURVEY_END_GRACE_MS
// When the window ends (or the operator ends it) with no fresh fix, the survey keeps
// averaging and waits this long for one before giving up, so a short gap in the receiver's
// output across the window's end does not discard the whole window. The window's end fails
// the survey after it; an operator's early end lapses instead and the survey runs on.
#define PPP_SURVEY_END_GRACE_MS         60000u
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

// FIX INTERVAL FORCED FOR THE SURVEY (keep).
//
// The PPP engine needs a 1 Hz fix rate. Per the LG290P protocol specification (V1.3,
// PQTMCFGFIXRATE), base station mode runs at 1 Hz and does not allow it to be changed, while
// rover mode runs at 10 Hz unless the default is changed. Sessions in which PPP converged on
// this hardware (0078, 0081) ran with the receiver held at 1 Hz; a PPP survey runs the
// receiver as a ROVER, where the rate is whatever was last configured.
//
// The survey's own configure sequence therefore writes PQTMCFGFIXRATE,W,<this> directly after
// its rover-mode write and before its PQTMSAVEPAR, and reads the rate back (PQTMCFGFIXRATE,R)
// as its last step; the sketch logs the answer when the averaging window opens. Boot sets the
// same rate, but a survey is not always preceded by boot: a re-survey started from base mode
// restarts the receiver into rover mode with whatever rate its saved configuration holds.
//
// The rate also gives the scatter window its meaning. That window is sized in FIXES and its
// length in seconds was reasoned about at 1 Hz; at 10 Hz a 60-fix window spans six seconds of
// heavily correlated fixes, which reads far tighter than the true minute-scale scatter.
//
// No restore is needed after the survey: base mode runs at 1 Hz.
#ifndef PPP_SURVEY_FIX_INTERVAL_MS
#define PPP_SURVEY_FIX_INTERVAL_MS      1000
#endif
#ifndef PPP_SURVEY_RATE_PROBE_MS
// The fix interval is also MEASURED: GGA sentences are counted over this first stretch of the
// averaging window, so the field log shows the rate the receiver runs at as well as the rate
// its read-back reports configured (status rateIntervalMs; the sketch logs both in one row).
#define PPP_SURVEY_RATE_PROBE_MS        10000u
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
    PPP_FAILED         // no usable fix, echoed fixes, no fix at the window's end, or aborted
};

// Which mean the lock uses, and why (AVERAGING POLICY in ppp_survey.cpp).
enum PppLockRule {
    PPP_RULE_NONE = 0,       // no fix averaged yet
    PPP_RULE_CONVERGED,      // converged PPP: enough converged fixes, and its halves agree
    PPP_RULE_ALL_TOO_FEW,    // all-fix mean: fewer converged fixes than convNeeded
    PPP_RULE_ALL_UNSTABLE    // all-fix mean: enough converged fixes, but its halves disagree
};

struct PppSurveyStatus {
    PppSurveyState state;
    uint32_t elapsedS;        // seconds since survey window started
    uint32_t durationS;       // configured window length
    uint32_t samples;         // fixes in the converged set (PPP solution + convergence test)
    uint32_t samplesAll;      // all non-echo fixes accumulated, converged or not
    bool     echoFault;       // survey failed because the receiver is echoing a stored
                              // coordinate (still in base mode), not solving: every fix was
                              // bit-identical, or the window ended on GGA quality-7 fixes,
                              // or quality 7 was all it reported. The mode switch did not take.
    uint32_t echoFixes;       // GGA quality-7 (base echo) fixes seen this survey; never averaged
    bool     finishedEarly;   // the operator ended the window early (ppp_survey_finish_now())
    bool     signalLost;      // FAILED because fixes had stopped when the window ended and
                              // none came within PPP_SURVEY_END_GRACE_MS (some had been
                              // averaged; zero fixes is the no-sky case)
    bool     awaitingFix;     // the window has ended (or the operator ended it) with no fresh
                              // fix; still SURVEYING, waiting up to PPP_SURVEY_END_GRACE_MS
    uint32_t finishLapses;    // operator early ends that lapsed for want of a fix this survey
    bool     lockUsedConverged;  // true if the locked coordinate came from the converged
                                 // set; false means the all-fix mean was used (rule says
                                 // why). Valid from PPP_LOCKING on.
    float    lockAccM;        // accuracy recorded for the locked coordinate (m): converged →
                              // mean EPE of the converged set; all-fix → the larger of that
                              // set's mean EPE and its position scatter. NAN until the lock
                              // decides, or when nothing was measurable.
    // WHICH MEAN (see PPP_SURVEY_PPP_TIME_RATIO). While averaging: what a lock now would
    // choose; from PPP_LOCKING on: what the lock chose (the accumulators stop at the lock).
    PppLockRule rule;
    uint32_t nonConverged;    // non-echo fixes that did not join the converged set
    uint32_t convNeeded;      // converged fixes needed: nonConverged / RATIO, floor and cap applied
    float    convMeanEpe;     // mean EPE of the converged set (m), NAN if none measurable
    float    halfH, halfV;    // converged first-half mean to second-half mean (m), NAN if < 2 fixes
    float    halfLimH, halfLimV;  // the stability gate's limits for them (m)
    float    convToAllH, convToAllV;  // converged mean to all-fix mean (m), NAN if no converged fix
    float    meanEpe2dAll;    // mean 2D EPE across all valid fixes (m), NAN if none
    uint32_t validFixes;      // any valid GGA fix seen (for diagnostics)
    bool     haveEpe;
    float    lastEpe2d;       // most recent 2D EPE (m), -1 if none or older than PPP_SURVEY_EPE_FRESH_MS
    float    lastEpe3d;       // most recent 3D EPE (m), -1 if none or older than PPP_SURVEY_EPE_FRESH_MS
    float    bestEpe2d;       // best (lowest) 2D EPE seen this survey (m), NAN if none
    float    bestEpe3d;       // best (lowest) 3D EPE seen this survey (m), NAN if none
    bool     converged;       // the most recent fix joined the converged set
    bool     pppSolution;     // the most recent fix was a PPP solution (quality 5 / station 9001, 9002)
    // Fix interval measured over the first PPP_SURVEY_RATE_PROBE_MS of the averaging window:
    // mean spacing of the GGA sentences received in it (any quality), -1 with fewer than two.
    int32_t  rateIntervalMs;
    uint32_t rateGgaCount;    // GGA sentences counted in that stretch
    bool     rateProbeDone;   // that stretch is over (or the window ended before it was)
    // ── E6 / HAS reception (the "is PPP data actually arriving" indicator) ──
    // Parsed live from $GAGSV signal-ID 5 (Galileo E6). If E6 is weak/absent at a
    // site, e6Sats drops and e6Ok goes false — your cue to abort and survey-in instead.
    uint32_t e6Sats;          // Galileo E6 satellites currently tracked
    float    e6AvgCnr;        // average C/N0 of those E6 signals (dB-Hz), NAN if none
    bool     e6Ok;            // heuristic: E6 reception healthy (sats>0 and C/N0 decent)
    double   meanLat;         // converged-set mean latitude (deg), NAN until its first fix
    double   meanLon;         // running mean longitude (deg)
    double   meanAltEll;      // running mean ellipsoidal height (m) = MSL alt + geoid sep
    bool     haveLockCoord;   // the lock has decided its coordinate (from PPP_LOCKING on)
    double   lockLat, lockLon, lockAltEll;  // that coordinate (deg / m ellipsoidal), NAN until then;
                              // from the converged or the all-fix set as lockUsedConverged says
    double   ecefX, ecefY, ecefZ;  // captured fixed-base coordinate (valid once PPP_DONE)
};

// Implemented in RCX_RTK_Base.ino: the lock sequence has just commanded a receiver restart
// (PQTMSRR). The sketch records it as a planned restart, so its GGA watch does not report
// the restart's silence as a receiver that stopped.
void rcxReceiverRestartCommanded();

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

// End the averaging window now and lock what has been averaged so far, through the same
// echo check, mean and verified lock sequence as a window that runs its full length. This
// is the dashboard's "Lock survey now". Returns false (and changes nothing) unless the
// survey is in PPP_SURVEYING with at least PPP_SURVEY_MIN_LOCK_FIXES fixes averaged. With
// a fix within PPP_SURVEY_FIX_STALE_MS the lock starts at once; without one the survey
// waits up to PPP_SURVEY_END_GRACE_MS for a fresh fix (status awaitingFix) and locks then,
// or the request lapses and the survey runs on (status finishLapses). Once the window itself
// has ended it returns true and changes nothing: the window's own grace already applies.
bool ppp_survey_finish_now();

// True while configuring / surveying / locking.
bool ppp_survey_active();

// Change the acceptance criteria at ANY time, including while a survey is running.
// Zero for a parameter leaves it unchanged, EXCEPT that supplying a 2D limit with a zero
// 3D limit derives the 3D limit from it (see PPP_SURVEY_EPE_3D_RATIO) — a caller that
// knows only one of the two must not be able to leave the other contradicting it. Takes effect on the next epoch; accumulated
// samples are kept and the receiver is never touched, so this never restarts anything.
void ppp_survey_set_criteria(float epe2dLimitM, float epe3dLimitM, uint32_t durationSeconds);

// Read back the criteria currently in force.
void ppp_survey_get_criteria(float& epe2dLimitM, float& epe3dLimitM, uint32_t& durationSeconds);

// One plain sentence naming the mean st.rule chooses and the numbers that chose it, e.g.
// "all-fix mean: converged PPP 9:40 of 15:00 needed". Shared by /api/ppp, /api/status, the
// lock's event row and the saved note, so every place says the same thing.
void ppp_survey_rule_text(const PppSurveyStatus& st, char* buf, size_t len);

// Snapshot for LCD / web dashboard.
PppSurveyStatus ppp_survey_status();

// Human-readable state name (for the dashboard / logs).
const char* ppp_survey_state_name(PppSurveyState st);

// Serialize the current status as a compact JSON object into buf. Use this to
// answer a GET /api/ppp poll from the web dashboard (see the integration snippet).
void ppp_survey_status_json(char* buf, size_t len);
