#pragma once
/*
 * webserver.h — ESPAsyncWebServer UI
 * ==================================
 * Three pages, each a separate document so the dashboard loads and refreshes
 * without carrying the whole configuration surface with it:
 *   GET /          Dashboard — live status, SD capacity, pause, log channels
 *   GET /setup     Device name, WiFi, NTRIP casters, CAN sniffer, DBC, GNSS, IMU
 *   GET /logs      Pause, paged file listing, multi-select delete, bulk export
 *   GET /app.css   Stylesheet shared by all three
 *   GET /app.js    Helpers shared by all three
 *
 * JSON/data routes are listed at the top of webserver.cpp.
 *
 * Call webserver_init() from setup() (registers routes).
 * Call webserver_begin() after WiFi connects (starts listening).
 */

#include <Arduino.h>
#include "types.h"

void webserver_init();
void webserver_begin();
