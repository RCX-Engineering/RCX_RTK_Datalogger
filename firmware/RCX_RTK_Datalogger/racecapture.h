#pragma once
/*
 * racecapture.h — RaceCapture/Pro JSON WiFi TCP emulator
 * =======================================================
 * Implements the RaceCapture/Pro JSON streaming API on a TCP socket
 * (default port 7223) so the RaceCapture app (Android/iOS/desktop)
 * can connect and display live GPS, IMU, and CAN channel data.
 *
 * Protocol overview (reference: autosportlabs/RaceCapture-Pro_firmware,
 * and RacePi by Donour Sizemore — github.com/donour/racepi):
 *
 *   All messages are newline-terminated JSON (\r\n).
 *   The device listens for JSON command objects from the app, and
 *   continuously streams sample data objects at 20 Hz.
 *
 *   App → Device queries:
 *     {"getCapabilities":{}}
 *     {"getChannels":{}}
 *     {"startStreaming":{}}
 *     {"stopStreaming":{}}
 *
 *   Device → App responses + stream:
 *     {"getCapabilities":{"major":2,"minor":14,"bugfix":0,...}}
 *     {"getChannels":{"success":1,"channels":[{channel def},{...}]}}
 *     {"s":{"t":ms,"d":[v0,v1,v2,...]}}   ← 20 Hz data stream
 *
 *   Channel index order matches the channel list returned by getChannels.
 *   null is emitted for channels with no valid data.
 *
 * Shared Porsche 987.2/718 CAN channels follow the built-in GPS/IMU channels.
 * 718-only channels are appended after the existing list so legacy channel
 * indices remain stable; they emit null when the 987.2 profile is active or a
 * required frame is absent on the tapped bus.
 *
 * WiFi must be connected before calling racecapture_beginServer().
 * The TCP server survives WiFi reconnects — call again after reconnect.
 */

#include <Arduino.h>
#include "types.h"

// Call once after WiFi connects to start the TCP listener.
void racecapture_beginServer();

// Call from the WiFi/NTRIP task loop to accept new connections and
// service the active client.  Lightweight — no blocking I/O.
void racecapture_loop(const GnssData& g, const ImuData& m, const CanData& c);

// True if a RaceCapture app client is currently connected.
bool racecapture_clientConnected();
