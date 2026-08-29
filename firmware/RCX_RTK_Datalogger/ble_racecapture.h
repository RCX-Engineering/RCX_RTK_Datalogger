#pragma once
/*
 * ble_racecapture.h — RaceCapture/Pro JSON protocol over BLE NUS
 * ===============================================================
 * Advertises as "RCX Datalogger" using the Nordic UART Service.
 * Implements the same JSON API as racecapture.cpp (WiFi TCP), so the
 * RaceCapture app and SoloStorm (as "RaceCapture BLE Logger") can both
 * connect and receive GPS + IMU + shared 987.2/718 CAN data plus append-only
 * 718 supplemental channels. Supplemental channels are sparse and omitted when
 * the active vehicle is a 987.2 or the source frame is absent on the tapped bus.
 *
 * Protocol (same as WiFi version):
 *   App → device:  {"getCapabilities":{}}
 *                  {"getChannels":{}}
 *                  {"startStreaming":{}}
 *                  {"stopStreaming":{}}
 *   Device → app:  {"getCapabilities":{...}}\r\n
 *                  {"getChannels":{"success":1,"channels":[...]}}\r\n
 *                  {"s":{"t":ms,"d":[v0,v1,...]}}\r\n  ← 20 Hz stream
 *
 * Library: NimBLE-Arduino
 */

#include <Arduino.h>
#include "types.h"

// Initialise NimBLE, create services, start advertising.
// Call once from setup() after dataMutex is created.
void ble_racecapture_init();

// Call at ~25 Hz from loop(). Sends pending command responses and,
// when streaming is active, emits one JSON sample per 50 ms tick.
void ble_racecapture_tick(const GnssData& g, const ImuData& m, const CanData& c);

// Returns true when a BLE client is connected.
bool ble_racecapture_connected();

// Updated by ble_racecapture_tick() — shown on the display.
extern float ble_packetHz;

// BLE TX instrumentation (lock-free reads; safe from any task). Consumed by
// sd_log so link health is a log column instead of a bench session:
//   ble_linkState()    0 = no central, 1 = connected, 2 = connected+subscribed
//   ble_txDelivered()  cumulative samples fully notified to the central
//   ble_txSuperseded() cumulative samples produced but never sent (replaced by
//                      a fresher sample, or blocked behind an in-flight message)
uint8_t  ble_linkState();
uint32_t ble_txDelivered();
uint32_t ble_txSuperseded();
