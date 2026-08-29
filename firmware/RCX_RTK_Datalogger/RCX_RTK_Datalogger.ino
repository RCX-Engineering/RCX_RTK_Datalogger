/*
 * RCX_RTK_Datalogger.ino — Main sketch
 * ===================================
 * Hardware: Waveshare ESP32-S3-LCD-1.47B
 *           + Waveshare LG290P GNSS RTK module
 *           + SN65HVD230 CAN transceiver (built-in TWAI, see config.h for pins)
 *
 * What this does:
 *   - Reads GPS/GNSS from LG290P via UART (NMEA + PQTMEPE)
 *   - Reads IMU from onboard QMI8658 (accel + gyro at 1kHz)
 *   - Reads CAN via built-in TWAI (listen-only) at 20Hz
 *   - Connects to WiFi, pulls NTRIP RTK corrections, pipes to LG290P
 *   - Streams GPS+IMU+CAN channels in RaceCapture format over BLE
 *   - Logs everything to SD card (gps_*.csv + sat_*.csv)
 *   - Serves a web dashboard at http://<device-ip>/
 *
 * Libraries (install via Arduino Library Manager):
 *   NimBLE-Arduino, TinyGPSPlus, TFT_eSPI,
 *   ESP Async WebServer, AsyncTCP
 *   (driver/twai.h is built into arduino-esp32 — no extra install needed)
 *
 * Before compiling:
 *   1. Copy TFT_UserSetup.h into your TFT_eSPI library as User_Setup.h
 *   2. Edit config.h for your WiFi, NTRIP, and CAN pin assignments
 *   3. Wire the SN65HVD230 to CAN_RX_PIN/CAN_TX_PIN listed in config.h
 *   4. Remove the 120 Ohm surface-mount resistor from SN65HVD230 CAN transciever
 *
 * Task layout:
 *   Core 1 (main): setup() + loop() — BLE TX, IMU read, display, GNSS parse
 *   Core 0:        wifiNtripTask    — WiFi, NTRIP, RaceCapture TCP server
 *   Core 0:        canBusTask       — TWAI listen-only at 500kbps, 20 Hz drain
 *   Core 0:        sdLogTask        — SD card writes (optional)
 *
 * ── Forward declarations (Arduino IDE needs these before the .cpp files) ──────
 */

struct GnssData;
struct ImuData;
struct SystemStatus;
struct CanData;
struct LogRecord;
// NtripConnResult is declared in ntrip.h — no forward-decl needed here

// ── Module includes ────────────────────────────────────────────────────────────
#include "config.h"
#include "types.h"
#include "gnss.h"
#include "imu.h"
#include "can_bus.h"
#include "ble_racecapture.h"
#include "racecapture.h"
#include "ntrip.h"
#include "wifi_mgr.h"
#include "display.h"
#include "thermal.h"
#include "sd_log.h"
#include "dbc_store.h"
#include "webserver.h"
#include <WiFi.h>
#include <esp_heap_caps.h>   // internal-heap diagnostics (heapTrace + 30s watermark)
#include <esp_system.h>      // esp_reset_reason() — boot cause for thermal-restart diagnosis
#include <driver/temperature_sensor.h>   // see dieTempReadWideC() — Arduino's own temperatureRead()
                                          // requests too narrow a range for this project's use
// debug_log.h LAST on purpose: when DEBUG_SERIAL_TO_SD is on it defines
// `#define Serial Debug`, and that must not rewrite the token inside any system
// or library header. Placed after every other include, the macro only affects
// this .ino's own code below — which is exactly what we want to tee.
#include "debug_log.h"

// ── Global shared state — declared extern in types.h ──────────────────────────
GnssData      gps;
ImuData       imu;
CanData       can;
SystemStatus  status;
SemaphoreHandle_t dataMutex = nullptr;

// rtcmBytesTotal declared extern in types.h, defined in ntrip.cpp

// ── Internal-RAM low-water tracking ───────────────────────────────────────────
// Sampled fast in loop() (catches dips a 5 s status poll misses). g_minFreeInternal
// = lowest total internal SRAM free ever; g_minLargestBlock = lowest largest
// contiguous block (fragmentation guard — TCP/SD allocs need contiguous memory).
// /status reports both (extern'd in webserver.cpp) so you get a hours-long record.
volatile uint32_t g_minFreeInternal = 0xFFFFFFFF;
volatile uint32_t g_minLargestBlock = 0xFFFFFFFF;

// DMA-capable is a narrower pool than general internal SRAM (MALLOC_CAP_DMA vs
// MALLOC_CAP_INTERNAL) — it's specifically what the SD driver's per-sector
// bounce buffer and WiFi's own buffers draw from (see printlnRetry() in
// sd_log.cpp). Tracked the same way, separately, because the general figures
// above have been observed reading a normal ~28-30 KB at the exact moment a
// write failed — this is the pool that actually explains that.
volatile uint32_t g_minFreeDma    = 0xFFFFFFFF;
volatile uint32_t g_minLargestDma = 0xFFFFFFFF;

// Last reset reason as text — captured once in setup(), declared extern in types.h.
// Read by sd_log (boot_events.csv) and webserver (/status). See types.h for why.
char g_resetReasonStr[32] = "unknown";

// ── GPS UART ──────────────────────────────────────────────────────────────────
HardwareSerial gpsSerial(1);   // UART1

// ── Display task (Core 0) ────────────────────────────────────────────────────
// Display SPI is slow (~15-30ms per redraw). Running it here instead of in
// loop() keeps that blocking off Core 1, so GNSS UART draining and BLE
// streaming are never stalled by a screen refresh. The task snapshots shared
// state under the mutex, then does all SPI work WITHOUT holding the lock.
// Only this task ever touches the TFT, so there is no display-bus contention.
static void displayTask(void*) {
    for (;;) {
        GnssData g_snap; ImuData m_snap; SystemStatus s_snap;
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10))) {
            g_snap = gps; m_snap = imu; s_snap = status;
            xSemaphoreGive(dataMutex);
        }
        display_update(g_snap, m_snap, s_snap);   // internally rate-limited to 5 Hz
        vTaskDelay(pdMS_TO_TICKS(50));            // poll at 20 Hz; redraw gate is inside
    }
}

// Per-step internal-heap trace. Internal SRAM (not PSRAM) is what SDMMC DMA
// buffers and TCP sockets need. File-scope (not setup()-local) because the real
// startup sequence — WiFi AP/STA bring-up, the webserver's actual listener bind,
// NTRIP init, the RaceCapture server — all run inside wifiNtripTask, not setup(),
// and none of it was ever traced before. See that function for why this mattered:
// every heap sample this project has ever logged already reflects whatever these
// four calls cost, with no visibility into which one actually costs it.
static void heapTrace(const char* step) {
    Serial.printf("🔍 [heap] after %-12s internal=%6u B (largest block %6u B)\n",
        step,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

// ── WiFi/NTRIP/RaceCapture task (Core 0) ─────────────────────────────────────
static void wifiNtripTask(void*) {
    heapTrace("task start");
    ntrip_init();
    heapTrace("ntrip_init");

    // Configuration AP first. This task is created after gnss_init() has fully
    // completed, which is the earliest point the radio may legally come up (see
    // the ordering note in setup()). The AP is the only route to the dashboard
    // on a unit with no stored networks, and it stays up for as long as the
    // station search is still running.
    wifi_apBegin();
    heapTrace("wifi_apBegin");

    // Initial WiFi connect. Returns false with an empty station list — that is
    // normal AP-only operation, not a failure, so the servers below still start.
    bool connected = wifi_tryConnect();
    heapTrace("wifi_tryConnect");

    // The dashboard must come up whether or not a station link exists: on a unit
    // with no stored networks the AP is the ONLY way to reach it, and adding the
    // first network is done from that dashboard. Binding the listen socket while
    // AP-only is safe: the AP netif is already up from wifi_apBegin() above, and
    // the radio mode is never re-entered afterwards, so the socket is never
    // orphaned by an interface rebuild (see wifi_mgr.cpp).
#if defined(WEBSERVER_ENABLE) && WEBSERVER_ENABLE
    webserver_begin();
    heapTrace("webserver_begin");
#endif
    // The RaceCapture telemetry server is only meaningful to an app on the same
    // network, but the AP is a network — start it in both cases too.
#if defined(RACECAPTURE_ENABLE) && RACECAPTURE_ENABLE
    racecapture_beginServer();
    heapTrace("racecapture_begin");
#endif
    (void)connected;

    static bool wifiWasUp = false;

    for (;;) {
        // Track the AP against station state: up while disconnected, down once a
        // station link has held (see wifi_apService).
        wifi_apService();

        // ── WiFi watchdog ─────────────────────────────────────────────────
        if (WiFi.status() != WL_CONNECTED) {
            if (wifiWasUp) {
                // Down-edge: run cleanup exactly once, not every retry cycle.
                wifiWasUp = false;
                if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
                    status.wifiConnected  = false;
                    status.ntripConnected = false;
                    status.rcConnected    = false;
                    xSemaphoreGive(dataMutex);
                }
                ntrip_onWifiLost();   // soft: keep mountpoint, prompt retry on return
            }

            // wifi_service() owns all reconnect policy and timing: the loyalty
            // window on the network that just dropped, the restart of the full
            // priority search when that window expires, and the spacing between
            // sweeps. It is a cheap no-op when there is nothing to attempt, so
            // it is safe to call on every pass of this loop.
            if (wifi_service()) {
                wifiWasUp = true;     // up-edge after a drop
#if defined(RACECAPTURE_ENABLE) && RACECAPTURE_ENABLE
                racecapture_beginServer();
#endif
            }
        } else {
            wifiWasUp = true;
        }

        if (WiFi.status() == WL_CONNECTED) {
            // Snapshot GPS for NTRIP and RaceCapture (mutex held briefly).
            // On a failed take, SKIP the consumers this pass rather than feed
            // them default-constructed (all-zero) snapshots: a zeroed GnssData
            // reads as epochSeq=0 to the BLE epoch detector (a phantom "new
            // epoch" carrying garbage data) and as a 0,0 position to any GGA
            // the NTRIP session sends. The next pass is 10 ms away; every
            // consumer tolerates that far better than fabricated data.
            GnssData g_snap; ImuData m_snap; CanData c_snap;
            bool snapOk = false;
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
                g_snap = gps; m_snap = imu; c_snap = can;
                xSemaphoreGive(dataMutex);
                snapOk = true;
            }

            if (snapOk) {
                ntrip_loop(gpsSerial, g_snap);

#if defined(RACECAPTURE_ENABLE) && RACECAPTURE_ENABLE
                racecapture_loop(g_snap, m_snap, c_snap);
#endif
            }
        }

        delay(10);
    }
}

// ── setup() ───────────────────────────────────────────────────────────────────
void setup() {
    // ════════════════════════════════════════════════════════════════════════
    //  CAN SAFETY — MUST BE THE FIRST THING IN setup(), BEFORE ANYTHING ELSE.
    //  GPIO9 drives the SN65HVD230's TXD input. The transceiver drives the bus
    //  DOMINANT whenever TXD is LOW. Until can_init() hands GPIO9 to the TWAI
    //  peripheral (which idles it recessive in listen-only mode), the pin is in
    //  its power-on default — and can_init() runs ~2.5 s into boot, AFTER
    //  display/IMU/GNSS/BLE init. If GPIO9 floats or sits low through that
    //  window, the transceiver holds the live vehicle CAN bus dominant for
    //  seconds on EVERY boot/reset, which reads as a bus fault / lost comms on
    //  the real ECUs (engine limp mode).
    //
    //  Driving it HIGH (recessive) here, before the delay() and all init, closes
    //  the app-side window. The remaining power-on→app gap (bootloader, a few
    //  hundred ms) is covered by a HARDWARE pull-up on the TXD line — fit one.
    //  The handoff to TWAI later is glitch-free: both states are recessive/high.
    //  DO NOT move this below any other init, and DO NOT remove it.
    pinMode(CAN_TX_PIN, OUTPUT);
    digitalWrite(CAN_TX_PIN, HIGH);   // recessive — transceiver does NOT drive bus
    // ════════════════════════════════════════════════════════════════════════

    Serial.begin(115200);
    delay(1000);
#if ARDUINO_USB_CDC_ON_BOOT
    // CRITICAL for sustained 20 Hz BLE in the field. With USB-CDC Serial, if no
    // host is draining the port (i.e. running in the car), the CDC TX buffer
    // fills and Serial.print()/printf() BLOCK loop() up to the CDC timeout on
    // every call. That alone collapsed the BLE stream to ~3-4 Hz. Timeout 0 =
    // never block: excess serial bytes are dropped instead of stalling the
    // control loop. (Per-send BLE logging is also gated off via BLE_TX_DEBUG;
    // this is the belt-and-suspenders guard for ALL Serial output everywhere.)
    // Guarded so it still compiles if USB-CDC-On-Boot is disabled (plain UART0).
    Serial.setTxTimeoutMs(0);
#endif
#if defined(DEBUG_SERIAL_TO_SD) && DEBUG_SERIAL_TO_SD
    // Arm the serial-to-SD mirror now that Serial is up. Everything printed from
    // here on is teed into the RAM ring and later written to debug_<stamp>.txt by
    // sdLogTask. Pre-arm output (the banner above) is not captured — that's fine,
    // the interesting NTRIP/reboot lines all come later.
    DebugLog::begin();
#endif
    Serial.println("\n\n=== RCX Datalogger v1.0 ===");

    // ── Capture WHY we just (re)booted ────────────────────────────────────────
    // The field restarts we're chasing may be thermal. esp_reset_reason() tells
    // us the CLASS of reset so the temp trail in the GPS log can be lined up
    // against it on the next boot. Captured once, surfaced on /status, and
    // written to boot_events.csv on the SD when the card mounts.
    //   BROWNOUT   → supply sagged (regulator hot / current spike) — thermal-ish
    //   TASK_WDT / INT_WDT / WDT → a task starved the watchdog (hang)
    //   PANIC      → exception/abort (firmware fault, e.g. stack overflow)
    //   POWERON / EXT → clean power-up or reset button (NOT a crash)
    {
        esp_reset_reason_t rr = esp_reset_reason();
        const char* s;
        switch (rr) {
            case ESP_RST_POWERON:   s = "POWERON";   break;
            case ESP_RST_EXT:       s = "EXT";       break;
            case ESP_RST_SW:        s = "SW";        break;
            case ESP_RST_PANIC:     s = "PANIC";     break;
            case ESP_RST_INT_WDT:   s = "INT_WDT";   break;
            case ESP_RST_TASK_WDT:  s = "TASK_WDT";  break;
            case ESP_RST_WDT:       s = "WDT";       break;
            case ESP_RST_DEEPSLEEP: s = "DEEPSLEEP"; break;
            case ESP_RST_BROWNOUT:  s = "BROWNOUT";  break;
            case ESP_RST_SDIO:      s = "SDIO";      break;
            default:                s = "UNKNOWN";   break;
        }
        strncpy(g_resetReasonStr, s, sizeof(g_resetReasonStr) - 1);
        g_resetReasonStr[sizeof(g_resetReasonStr) - 1] = '\0';
        Serial.printf("↻ Reset reason: %s (code %d)\n", g_resetReasonStr, (int)rr);
    }

    dataMutex = xSemaphoreCreateMutex();
    if (!dataMutex) {
        Serial.println("❌ FATAL: mutex alloc failed");
        for (;;) delay(1000);
    }

    // Load the stored WiFi list and derive this unit's AP name. Deliberately
    // placed here, near the top: it reads NVS and the eFuse MAC only and leaves
    // the radio DOWN. The radio itself is not brought up until wifiNtripTask
    // runs, which is created after gnss_init() below — bringing WiFi up during
    // the LG290P configuration probe corrupts that timing-sensitive UART
    // exchange. Do not move any WiFi.mode()/softAP() call above gnss_init().
    wifi_init();

    heapTrace("boot");

    display_init();
    Serial.println("✅ Display initialized");
    heapTrace("display");

    if (!imu_init()) {
        // Non-fatal: continue without IMU
    }
    heapTrace("imu");

    gnss_init(gpsSerial);
    heapTrace("gnss");

    ble_racecapture_init();
    heapTrace("ble");

    if (!can_init()) {
        Serial.println("⚠️  CAN disabled — continuing without vehicle data");
    } else {
        xTaskCreatePinnedToCore(
            canBusTask, "CAN",
            // 6 KB. The 30 s diagnostic has this task's high-water mark sitting
            // at ~5990 B free of 8192 across long runs — a peak use near 2200 B.
            // Task stacks cannot live in PSRAM, so every kilobyte over what the
            // task actually needs is taken directly out of the internal heap the
            // SDMMC DMA buffers, lwIP and the NTRIP source-table scan all draw
            // on. 6 KB still leaves well over the measured peak again in reserve.
            // Watch the CAN watermark in the 30 s print; below ~2000 B, raise it.
            6144, nullptr, 2, nullptr, 0);
    }
    heapTrace("can");

#if defined(SD_LOG_ENABLE) && SD_LOG_ENABLE
    sdlog_init();
    heapTrace("sdlog");
#endif

    // Storage only — reads the selected filename and starts the task that owns
    // DBC files on the card. Nothing here touches the CAN decode path.
    dbc_init();

#if defined(WEBSERVER_ENABLE) && WEBSERVER_ENABLE
    webserver_init();
    heapTrace("webserver");
#endif

    // WiFi/NTRIP/RaceCapture task. Stack was 28 KB, sized defensively for the
    // String-heavy source-table parse; that parse is now zero-heap/zero-String,
    // and the measured high-water mark showed <3 KB of 28 KB actually used.
    // Task stacks MUST live in internal SRAM (cannot go to PSRAM), and internal
    // RAM is our scarcest resource — 12 KB still leaves >9 KB of headroom over
    // the worst observed usage. Watch the WiFiNTRIP watermark in the 30 s diag
    // print; if it ever drops below ~2000, raise this again.
    xTaskCreatePinnedToCore(
        wifiNtripTask, "WiFiNTRIP",
        12288, nullptr, 1, nullptr, 0);
    heapTrace("wifintrip task");

    // Display task on Core 0 — keeps slow SPI redraws off the GNSS/BLE path.
    // TFT_eSPI pushSprite + font rendering + snprintf("%.9f",...) each consume
    // 300-500 bytes of stack above the local GnssData/SystemStatus snapshots. At
    // 4 KB this task was stack-overflowing (confirmed via watermark monitoring),
    // causing rst:0xc / no guru meditation (USB CDC drops before backtrace
    // flushes) — so 4 KB is a known-bad floor, not a theoretical one.
    // 6 KB: the 30 s diagnostic reports this task's high-water mark at ~5620 B
    // free of 8192, a peak use near 2570 B — comfortably clear of the overflow
    // point with headroom larger than the peak itself. The kilobytes returned
    // go to the internal heap, which is the resource this board runs out of
    // first. Watch the Display watermark; below ~2000 B, raise it.
    xTaskCreatePinnedToCore(
        displayTask, "Display",
        6144, nullptr, 1, nullptr, 0);
    heapTrace("display task");

    Serial.println("✅ Setup complete — waiting for GPS fix...");
}

// ── ESP32-S3 die temperature, full high-temperature range ──────────────────────
// Arduino's own temperatureRead() (esp32-hal-misc.c) installs the sensor with
// TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50) — a room-temperature range picked
// for general-purpose IoT use, nowhere near what a dual-core chip running WiFi
// under sustained load actually reaches. The ESP-IDF driver only offers a small
// set of PREDEFINED measurement bands (it picks whichever covers the requested
// range with the least error) — the widest is 50-125°C, and Espressif's own
// forum confirms a reading taken outside the installed band does not reliably
// fail, it can come back as a plausible-looking but WRONG value. That is almost
// certainly why this chip was reading a plateau well under its real temperature:
// Arduino's 10-50°C request maps to the "-10 to 80°C" band, and everything
// hotter than that was silently wrong, not merely capped.
//
// 50-125°C is exactly the window this project cares about — thermal.h's
// thresholds start at 100°C — so trading away accurate cold-boot readings for
// an honest signal in the range that actually matters is the right call, and
// matches how this die sensor is already documented and used everywhere else
// in this project: a threshold/trend signal, never a precision thermometer.
static float dieTempReadWideC() {
    static temperature_sensor_handle_t tsens = nullptr;
    static bool failed = false;
    if (!tsens && !failed) {
        temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(50, 125);
        if (temperature_sensor_install(&cfg, &tsens) != ESP_OK ||
            temperature_sensor_enable(tsens) != ESP_OK) {
            Serial.println("❌ Die temp sensor (50-125°C) install failed — thermal throttling disabled");
            tsens = nullptr; failed = true;
        }
    }
    float c = NAN;
    if (tsens) temperature_sensor_get_celsius(tsens, &c);   // leaves c untouched (NAN) on failure
    return c;
}

// ── loop() phase timing ───────────────────────────────────────────────────────
// Worst-case wall time of each loop() phase, in microseconds, kept as a running
// maximum and reported once per interval.
//
// Why this exists: the UART ISR that empties the GNSS 128-byte hardware RX FIFO
// runs on this core. At 460800 baud that FIFO holds 2.8 ms of line time, so any
// phase that defers interrupts past that — a long critical section, a suspended
// cache, a blocking driver call — costs whole NMEA sentences, and a loss of one
// epoch's sentence group (~257 B, 5.6 ms) removes that epoch from every consumer
// at once. Field logs show exactly that shape: 62% of losses are a single 100 ms
// gap, loop() itself never stalls (IMU holds 50 Hz throughout), and the software
// RX ring is far too large to be the victim. That leaves the FIFO, and the
// question these counters answer is WHICH phase holds the core long enough.
//
// The counters read a free-running microsecond timer and change no behavior, no
// ordering and no timing of the work they measure.
namespace loopPhase {
enum : uint8_t { GNSS, IMU, TEMP, BLE, SDPUSH, HEAP_FREE, HEAP_LARGEST, WHOLE, COUNT };
static const char* const kName[COUNT] = {
    "gnss", "imu", "temp", "ble", "sdpush", "heapFree", "heapLargest", "loop"
};
static uint32_t maxUs[COUNT] = {0};
static inline void note(uint8_t i, uint32_t us) { if (us > maxUs[i]) maxUs[i] = us; }

// Report and reset. Anything at or above the FIFO depth is flagged, because that
// is the threshold at which a phase can destroy an epoch.
static void report() {
    Serial.print("⏱️  loop phase max µs:");
    for (uint8_t i = 0; i < COUNT; i++) {
        Serial.printf(" %s=%lu%s", kName[i], (unsigned long)maxUs[i],
                      maxUs[i] >= 2800 ? "!" : "");
        maxUs[i] = 0;
    }
    Serial.println();
}
}  // namespace loopPhase

// Time one statement into a phase slot.
#define LOOP_PHASE(slot, stmt)                                   \
    do {                                                         \
        uint32_t _phaseT0 = micros();                            \
        stmt;                                                    \
        loopPhase::note(loopPhase::slot, micros() - _phaseT0);   \
    } while (0)

// ── loop() ────────────────────────────────────────────────────────────────────
void loop() {
    uint32_t loopT0 = micros();

    // ── GNSS: parse UART bytes, update gps struct ─────────────────────────
    LOOP_PHASE(GNSS, gnss_loop(gpsSerial));

    // ── IMU: read at 50 Hz into a local, then publish under mutex ─────────
    {
        static uint32_t lastImu = 0;
        if (millis() - lastImu >= 20) {
            lastImu = millis();
            imu_pollSerial();                        // "imucal" → start calibration (no-op otherwise)
            imu_serviceCalRequests();                // web /imu/cal → start/clear, same context as imu_read()
            ImuData m_local;
            LOOP_PHASE(IMU, imu_read(m_local));      // blocking I2C — done WITHOUT the mutex
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
                imu = m_local;                        // publish atomically
                xSemaphoreGive(dataMutex);
            }
            // Push IMU record at 50 Hz to its own SD log channel
            sdlog_push_imu(m_local.ax, m_local.ay, m_local.az,
                           m_local.gx, m_local.gy, m_local.gz);
        }
    }

    // ── Device temperature at 0.5 Hz ──────────────────────────────────────
    // The ESP32-S3 die sensor (dieTempReadWideC, defined above loop()) is a quick
    // internal-ADC read with no bus involved. 0.5 Hz is ample for thermal work.
    // Published under the mutex so the GPS log, /status and the display all read a
    // consistent snapshot.
    {
        static uint32_t lastTemp = 0;
        if (millis() - lastTemp >= 2000) {
            lastTemp = millis();
            uint32_t tempT0 = micros();
            float espC = dieTempReadWideC();         // ESP32-S3 silicon die, °C
            thermal_update(espC);                    // see thermal.h — LCD/SD throttling only
            loopPhase::note(loopPhase::TEMP, micros() - tempT0);
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
                status.espTempC = espC;
                xSemaphoreGive(dataMutex);
            }
        }
    }

    // ── BLE: RaceCapture command replies every pass; samples self-limited ──
    // The 40 ms window that used to wrap this gated command replies too. But
    // SoloStorm fires {"getMeta":null} repeatedly and impatiently right after
    // subscribe and will spin/disconnect if the {"meta":[...]} reply lags. So
    // tick() now runs EVERY loop pass: it drains ACKs + pendingMeta promptly,
    // while sendSample() inside tick() keeps its own 50 ms (20 Hz) rate gate —
    // emitting samples no faster than before. Only the reply latency improves.
    {
        // Snapshot shared state under the mutex ONLY (see note below). On a
        // timed-out take, reuse the last good snapshot rather than skipping.
        //
        // ⚠️ ble_racecapture_tick() MUST run unconditionally, NOT gated on the
        //    mutex AND NOT gated on a sample-rate window. It drains command
        //    ACKs and pendingMeta/pendingVersion. If it is skipped (5 ms take
        //    failed, or a coarse rate window), pendingMeta is set but never
        //    sent — SoloStorm re-requests getMeta forever and spins.
        //    (Field log: repeated "BLE RX: getMeta" with no TX, then disconnect.)
        //    tick() does BLE notify()s with vTaskDelays, so it must run OUTSIDE
        //    the critical section. Snapshot in, tick out.
        static GnssData g_snap; static ImuData m_snap; static CanData c_snap;
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))) {
            g_snap = gps; m_snap = imu; c_snap = can;
            xSemaphoreGive(dataMutex);
        }
        LOOP_PHASE(BLE, ble_racecapture_tick(g_snap, m_snap, c_snap));
    }

    // ── SD log ────────────────────────────────────────────────────────────
#if defined(SD_LOG_ENABLE) && SD_LOG_ENABLE
    LOOP_PHASE(SDPUSH, sdlog_push_if_new());   // queues when gnss_loop flags a new epoch
#endif

    // ── Internal-RAM low-water sampling (~20 Hz) ──────────────────────────
    {
        static uint32_t lastHeapSample = 0;
        if (millis() - lastHeapSample >= 50) {
            lastHeapSample = millis();
            // get_free_size reads maintained per-region counters;
            // get_largest_free_block walks the free-block list of every matching
            // heap under the heap lock. The two are timed separately because
            // only the second scales with fragmentation, and fragmentation
            // climbs all session as the WiFi and BLE stacks cycle buffers.
            uint32_t f = 0, lb = 0;
            LOOP_PHASE(HEAP_FREE,
                       f = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            LOOP_PHASE(HEAP_LARGEST,
                       lb = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            if (f  < g_minFreeInternal) g_minFreeInternal = f;
            if (lb < g_minLargestBlock) g_minLargestBlock = lb;
            // DMA-capable is a NARROWER pool than general internal SRAM — it's
            // what the SD driver's per-sector bounce buffer actually competes
            // for (see printlnRetry() in sd_log.cpp), and WiFi's own buffers
            // draw from the same pool while a page response streams out. A
            // write has been observed failing while the figures above still
            // read a normal ~28-30 KB, which only makes sense if this specific
            // pool was tighter at that moment than the general one — tracking
            // it separately is what would show that directly next time,
            // instead of inferring it after the fact.
            uint32_t fd = 0, ld = 0;
            LOOP_PHASE(HEAP_FREE,    fd = heap_caps_get_free_size(MALLOC_CAP_DMA));
            LOOP_PHASE(HEAP_LARGEST, ld = heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            if (fd < g_minFreeDma)    g_minFreeDma    = fd;
            if (ld < g_minLargestDma) g_minLargestDma = ld;
        }
    }

    // ── Loop phase timing report ──────────────────────────────────────────
    // WHOLE captures the entire iteration, so any cost NOT attributed to a named
    // phase above shows up as the difference. Reported on its own 5 s cadence so
    // the numbers stay readable next to the existing wire counters.
    loopPhase::note(loopPhase::WHOLE, micros() - loopT0);
    {
        static uint32_t lastPhaseReport = 0;
        if (millis() - lastPhaseReport >= 5000) {
            lastPhaseReport = millis();
            loopPhase::report();
        }
    }

    // ── Stack + heap diagnostics every 30 s ──────────────────────────────
    // Prints the minimum free stack (high-water mark) for each task, and free
    // heap. A value near 0 means that task nearly overflowed. Remove or gate
    // with DEBUG_WATERMARKS once the firmware is stable.
#if defined(DEBUG_WATERMARKS) && DEBUG_WATERMARKS
    {
        static uint32_t lastWM = 0;
        if (millis() - lastWM >= 30000) {
            lastWM = millis();
            Serial.printf("🔍 Heap: %u B free (internal: %u B, largest block %u B) | Stacks: loop=%u WiFiNTRIP=%u\n",
                (unsigned)esp_get_free_heap_size(),                       // includes PSRAM — misleading alone
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                (unsigned)uxTaskGetStackHighWaterMark(nullptr),   // loop() task
                (unsigned)uxTaskGetStackHighWaterMark(
                    xTaskGetHandle("WiFiNTRIP")));
            Serial.printf("   ⬇ internal-RAM lows since boot: free=%u B, largest block=%u B\n",
                (unsigned)g_minFreeInternal, (unsigned)g_minLargestBlock);
            Serial.printf("   ⬇ DMA-capable lows since boot:  free=%u B, largest block=%u B\n",
                (unsigned)g_minFreeDma, (unsigned)g_minLargestDma);
            Serial.printf("   Display=%u CAN=%u SDLog=%u\n",
                (unsigned)uxTaskGetStackHighWaterMark(
                    xTaskGetHandle("Display")),
                (unsigned)uxTaskGetStackHighWaterMark(
                    xTaskGetHandle("CAN")),
                (unsigned)uxTaskGetStackHighWaterMark(
                    xTaskGetHandle("SDLog")));
        }
    }
#endif
    // Display is handled by its own task (displayTask) — see setup().
}
