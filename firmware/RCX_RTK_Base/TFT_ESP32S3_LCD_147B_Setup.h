#pragma once

// Project-local TFT_eSPI setup for Waveshare ESP32-S3-LCD-1.47B.
// This lets the sketch use TFT_eSPI without editing the installed library's User_Setup.h.

#define USER_SETUP_LOADED
#define USER_SETUP_ID 147320

#define ST7789_DRIVER
#define TFT_WIDTH  172
#define TFT_HEIGHT 320

// 172x320 ST7789 panels usually need the ST7789 controller RAM offset handling.
#define CGRAM_OFFSET
#define TFT_RGB_ORDER TFT_RGB
#define TFT_INVERSION_ON

#define TFT_MOSI 45
#define TFT_SCLK 40
#define TFT_CS   42
#define TFT_DC   41
#define TFT_RST  39
// NOTE: backlight (GPIO46) is intentionally NOT declared to TFT_eSPI here.
// The sketch owns it via LEDC PWM (ledcAttach(PIN_LCD_BL) in initDisplay) for
// brightness control. If TFT_BL/TFT_BACKLIGHT_ON were also defined, tft.init()
// would digitalWrite() the same pin that LEDC already attached, producing the
// boot-time "IO 46 is not set as GPIO" error. One owner only — leave these out.
// #define TFT_BL   46
// #define TFT_BACKLIGHT_ON HIGH

#define USE_HSPI_PORT
#define SUPPORT_TRANSACTIONS

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define SMOOTH_FONT

#define SPI_FREQUENCY       40000000
#define SPI_READ_FREQUENCY  16000000
#define SPI_TOUCH_FREQUENCY 2500000
