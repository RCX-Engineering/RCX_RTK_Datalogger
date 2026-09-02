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
 *   POST /api/pppstart → body "sec=<seconds>" (0 = module default, 900 = 15 min)
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
