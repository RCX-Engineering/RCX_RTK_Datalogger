#pragma once
/*
 * ppp_web.h — self-contained web UI + control endpoints for the PPP survey
 * =======================================================================
 * Registers everything on your existing WebServer with ONE call. No edits to
 * bridge_web_page.h are required — the PPP status page is served standalone.
 *
 * Endpoints added:
 *   GET  /ppp          → standalone auto-refreshing status page (link it from your
 *                        dashboard with:  <a href="/ppp">PPP Survey</a>)
 *   GET  /api/ppp      → status JSON (same object the page polls)
 *   POST /api/pppstart → body "sec=<seconds>" (0 = the configured window; otherwise
 *                        rcxMinSurveySec() .. 86400, e.g. 900 = 15 min)
 *   POST /api/pppabort → abort an in-progress survey (leaves module in rover)
 *
 * Integration — four lines total in your sketch:
 *   1) #include "ppp_web.h"
 *   2) setup():               ppp_web_register(server, GnssSerial);
 *   3) handleNmeaSentence():  ppp_survey_feed_nmea(nmeaLine);
 *   4) loop():                ppp_survey_tick();
 *
 * The feed + tick calls are irreducible: NMEA flows through your parser (single
 * UART reader), and the state machine advances in loop(). Everything else —
 * endpoints, HTML, JS — lives in this module.
 */

#include <WebServer.h>
#include <HardwareSerial.h>
#include "ppp_survey.h"

// Wire the PPP dashboard + control endpoints onto an existing server.
// Call once from setup(). `gnss` is the same UART the LG290P is on (GnssSerial).
void ppp_web_register(WebServer& server, HardwareSerial& gnss);

// Implemented in RCX_RTK_Base.ino: sends a static page on the current request in
// bounded pieces, draining the GNSS UART between them and giving up after a few
// seconds without progress. Every static page goes through it.
void rcxSendStaticPage(const char* contentType, const char* body, size_t len);

// Implemented in RCX_RTK_Base.ino: one event-log row for a dashboard action, with the
// client's address. Never pass a password in `detail`.
void rcxLogWebAction(const char* route, const char* detail);

// Implemented in RCX_RTK_Base.ino: queue a fresh PPP survey through the same gate as the
// dashboard's re-survey (escape to rover, verify, bookkeeping reset), run from loop().
// durationSeconds 0 keeps the configured survey window.
void rcxRequestPppSurvey(uint32_t durationSeconds);

// Implemented in RCX_RTK_Base.ino: the shortest survey time the device accepts, s — the
// venue check's window plus its post-restart settle (SURVEY TIME LIMITS in the sketch).
uint32_t rcxMinSurveySec();

// Implemented in RCX_RTK_Base.ino (STRICT NUMBERS FROM THE WEB): the whole argument must be
// a finite decimal number / a decimal integer within [lo, hi]; false refuses it.
bool rcxParseWebNumber(const char* s, double& out);
bool rcxParseWebInt(const char* s, long lo, long hi, long& out);

// Implemented in RCX_RTK_Base.ino: an operator stopped the survey. Cancels a queued one
// and, from loop(), re-checks the saved position against the live fix as a reboot would,
// without starting another survey.
void rcxSurveyAbortedByOperator();
