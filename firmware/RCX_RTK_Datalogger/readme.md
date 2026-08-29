This is the firmware for the rover. It's designed for motorsports data logging of 20Hz GNSS, IMU, and CAN data using the RaceCapture protocol published by Autosports Labs.

## Hardware 
### Waveshare ESP32-S3-LCD-1.47B
Other variants may work but are untested. Note: IO/GPIO vary greatly betweeen ESP32 variants, even similar ones, and must be remapped

### Waveshare LG290P RTK GNSS Dev Board
LG290P must be on v2.02 firmware or newer. (LG290P03AANR02A02S.pkg)


## Building the firmware
Built with the Arduino IDE http://arduino.cc/en/software

### Board package

Add to *File → Preferences → Additional Boards Manager URLs*: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
Then install **esp32 by Espressif Systems, version 3.3.8**. Other 3.x versions may work; earlier versions will not — the firmware relies on core 3.x APIs and memory layout.

### Board settings

Select **ESP32S3 Dev Module** (not the Waveshare board entry — it hides the PSRAM option, which the firmware requires).

|     Setting      |           Value                  |
|------------------|----------------------------------|
| Flash Size       | 16MB                             |
| Flash Mode       | QIO 80MHz                        |
| PSRAM            | OPI PSRAM                        |
| Partition Scheme | Huge APP (3MB No OTA/1MB SPIFFS) |
| USB CDC On Boot  | Enabled                          |
| USB DFU On Boot  | Disabled                         |
| CPU Frequency    | 240MHz                           |
| Upload Speed     | 921600                           |

**Change the partition scheme.**
- Go to *Tools → Partition Scheme* and select **Huge APP (3MB No OTA/1MB SPIFFS)**.
- If you need OTA, *Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)* retains it, but has not been tested and may not support future upgrades.

### Libraries

Install from the Library Manager:

|        Library       |   Author   |
|----------------------|------------|
| TFT_eSPI             | Bodmer     |
| NimBLE-Arduino (2.x) | h2zero     |
| TinyGPSPlus          | Mikal Hart |
| ESP Async WebServer  | ESP32Async |
| AsyncTCP             | ESP32Async |

The web server libraries must be the **ESP32Async** forks. The original me-no-dev versions do not compile against core 3.x. If both are present the IDE picks the wrong one — delete `Arduino/libraries/ESPAsyncWebServer` and `Arduino/libraries/AsyncTCP`, keeping `ESP_Async_WebServer` and `Async_TCP`.

WiFi, Wire, SD_MMC, Preferences, TWAI and the task watchdog come from the core. The QMI8658 IMU is driven directly over I2C and needs no library.

### Display configuration

Copy `TFT_UserSetup.h` from this repository over `Arduino/libraries/TFT_eSPI/User_Setup.h`, and confirm that `User_Setup_Select.h` has only `#include <User_Setup.h>` uncommented.
`TFT_BL` is intentionally left undefined. The backlight is on GPIO46 and is driven by the sketch through LEDC; declaring it to TFT_eSPI as well produces a boot-time IO 46 conflict.

### Hardware notes

The LG290P GNSS module connects to GPIO4 (RX) and GPIO5 (TX) at 460800 baud. If the COM port does not appear, install the Silicon Labs CP2102 driver. If upload stalls at "Connecting…", hold BOOT while connecting USB.
