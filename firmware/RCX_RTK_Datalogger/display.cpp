/*
 * display.cpp — TFT_eSPI display driver
 * ======================================
 * Refactored from RaceBox_LG290P.ino.  Added CAN Hz to the status row.
 */

#include "display.h"
#include "types.h"
#include "sd_log.h"        // sdlog_getActiveGpsSize() for the recording-size readout
#include "wifi_mgr.h"      // wifi_deviceName()/wifi_apIp() for the no-station-link rows
#include "thermal.h"       // thermal_lcdOff()/thermal_backlightDim() — see below
#include "debug_log.h"     // routes the status-LED report through the SD debug tee
#include <TFT_eSPI.h>
// Cone survey mode draws its large figures in Font 4. Without it TFT_eSPI would
// silently draw nothing, so a setup that lacks the font fails here instead.
#ifndef LOAD_FONT4
#error "display.cpp needs TFT_eSPI Font 4: add #define LOAD_FONT4 to TFT_eSPI's User_Setup.h"
#endif
#include <esp_heap_caps.h>  // cone-mode survey state lives in PSRAM
#include <algorithm>       // std::nth_element for the cone-mode median

static TFT_eSPI    tft;
static TFT_eSprite gcSprite(&tft);

// LCD backlight on IO46 — active-HIGH, confirmed against real hardware (the
// identical RCX1 base board): duty 0 = off, duty (1<<LCD_BL_RES_BITS)-1 = full
// brightness, no inversion. PWM'd via LEDC so it can be dimmed, not just
// switched. Ordering matters: LEDC must attach AFTER tft.init() — see
// display_init() for why.
#define LCD_BL_PIN      46
#define LCD_BL_FREQ_HZ  5000
#define LCD_BL_RES_BITS 8

// ── Daylight palette ─────────────────────────────────────────────────────────
// Dark content on a white field (positive polarity). On a transmissive LCD in
// daylight, reflected ambient light adds equally to every pixel, so what stays
// visible is the difference in EMITTED luminance between text and background.
// On black, a coloured glyph can only emit its own luminance — pure red is 21%
// of white and pure blue 7%, so the "bad" state was the first thing to vanish in
// sun. On white, every dark colour below keeps ~85% of the panel's full swing,
// whatever its hue. Dark-on-light is also the MIL-STD-1472H day-mode rule
// (>= 540 lx), gives the documented positive-polarity legibility advantage for
// small characters (Piepenbrock, Mayr & Buchner, Human Factors 56(5), 2014),
// and a bright field masks screen reflections that a black field shows as a
// mirror.
//
// Stoplight colours are the web dashboard's own (webserver.cpp :root CSS):
//   C_GOOD      #0f7a37  --ok-bg       text contrast on white 5.5:1
//   C_BAD       #c62330  --bad-bg      text contrast on white 5.7:1
//   C_WARN_FILL #e6a100  --warn-bg     title-bar fill only (dark title text)
//   C_WARN      #946500  --warn-bg darkened at the same hue so amber TEXT
//                        still reaches 5.1:1 on white (#e6a100 as text: 2.2:1)
//   C_DIM       #5a626d  --muted       6.3:1
//   C_TEXT      #14181d  --text
// Every text colour in this file, including each point of the accuracy and
// satellite gradients, clears the 4.5:1 small-text minimum against C_BG.
// The title-bar text stays black on all three stoplight fills.
// Values are RGB565 as packed by TFT_eSPI::color565().
#define C_BG        TFT_WHITE
#define C_GOOD      0x0BC6
#define C_WARN      0x9320
#define C_WARN_FILL 0xE500
#define C_BAD       0xC106
#define C_TEXT      0x10C3
#define C_DIM       0x5B0D

#define GC_CX   86
#define GC_CY   284
#define GC_R    36
#define GC_SCALE 24
#define GC_TRAIL 24
#define GC_Y0   248
#define GC_SP_H (320 - GC_Y0)

static uint16_t lerpColor(float t,
    uint8_t r1,uint8_t g1,uint8_t b1,
    uint8_t r2,uint8_t g2,uint8_t b2) {
    t=constrain(t,0.0f,1.0f);
    return tft.color565((uint8_t)(r1+(r2-r1)*t),(uint8_t)(g1+(g2-g1)*t),(uint8_t)(b1+(b2-b1)*t));
}

// Gradient nodes, best to worst: teal-green (0,110,90) → dashboard green
// (15,122,55) → dashboard accent blue (11,96,201) → dark amber (148,101,0) →
// dashboard red (198,35,48). All nodes are dark, so every interpolated shade
// between two of them also stays at or above 4.5:1 on the white field.
static uint16_t accuracyColor(float m) {
    if (m>=1.5f) return tft.color565(198,35,48);
    if (m>=0.8f) return lerpColor((m-0.8f)/0.70f,148,101,0,198,35,48);
    if (m>=0.4f) return lerpColor((m-0.4f)/0.40f,11,96,201,148,101,0);
    if (m>=0.05f)return lerpColor((m-0.05f)/0.35f,15,122,55,11,96,201);
    return               lerpColor(m/0.05f,       0,110,90,15,122,55);
}
static uint16_t satColor(int n) {
    if (n>=24) return lerpColor((n-24)/8.0f,15,122,55,0,110,90);
    if (n>=16) return lerpColor((n-16)/8.0f,11,96,201,15,122,55);
    if (n>=10) return lerpColor((n-10)/6.0f,148,101,0,11,96,201);
    if (n>= 6) return lerpColor((n- 6)/4.0f,198,35,48,148,101,0);
    return tft.color565(198,35,48);
}
static uint16_t healthColor(const GnssData& g, const SystemStatus& s) {
    if (!s.wifiConnected||!s.bleConnected||!g.valid) return C_BAD;
    if (!s.ntripConnected||g.rtkType==0||g.hAccM>1.5f) return C_BAD;
    if (g.rtkType==1||g.hAccM>0.30f) return C_WARN_FILL;
    return C_GOOD;
}

// ── Status LED ───────────────────────────────────────────────────────────────
// The onboard RGB bead (IO38, RGB_IO) shows the same condition as the title
// banner, so the unit reads as one indicator from any side of the car rather
// than only from in front of the panel. It is keyed off the banner colour that
// display_update() has just painted, in the same pass, so the two can never
// report different conditions. Keep healthColor() returning exactly C_GOOD,
// C_WARN_FILL or C_BAD; the static_assert below holds them distinct.
//
// A plain stoplight: pure green, yellow and red. These deliberately differ from
// the banner fills — the banner's dark green and amber exist so black text can
// sit on them; the LED carries no text and is read as a bare hue at a distance.
//
// ONE bead, not a chain (Waveshare ESP32-S3-LCD-1.47B, RGB Light Beads table).
// rgbLedWriteOrdered() latches after 24 bits and cannot address a second pixel;
// a strip added to the header pins would need an RMT symbol encoder instead.
struct LedColour { uint8_t r, g, b; };
static constexpr LedColour LED_OK   = {   0, 255,   0 };   // green
static constexpr LedColour LED_WARN = { 255, 255,   0 };   // yellow
static constexpr LedColour LED_BAD  = { 255,   0,   0 };   // red
static constexpr LedColour LED_DARK = {   0,   0,   0 };   // off: panel off, and boot
static constexpr LedColour LED_INHIBIT = { 255,   0, 255 };   // magenta: RTK held back by low RAM (not a stoplight state)

static constexpr uint8_t LED_PIN = 38;   // RGB_IO; not used by the LCD, SD, GNSS UART,
                                         // CAN or IMU I2C, and not a strapping pin
// COLOUR ORDER. This bead takes its bytes as R,G,B. The core's rgbLedWrite()
// assumes G,R,B, which turns a red request into green on the wire — a unit in its
// worst state glowing the all-clear colour. Every write names the order. If the
// colours ever come out swapped, this is the constant to change; the core enum
// covers all six permutations.
static constexpr rgb_led_color_order_t LED_ORDER = LED_COLOR_ORDER_RGB;
// Output scale, 0..255, applied to every channel at write time. A quarter duty
// is visible across a paddock in daylight without wrecking the night vision of
// whoever is leaning over the car to read the panel. Linear: at this level the
// LED's own response dominates and gamma correction buys nothing visible.
static constexpr uint16_t LED_BRIGHTNESS = 64;
// Idempotent re-send. The bead latches and holds with no further traffic; this
// repaints a pixel that latched a corrupted value (a line glitch, supply sag
// during a WiFi transmit burst) instead of leaving the wrong condition showing
// until the next state change, which on a healthy run can be a long time.
static constexpr uint32_t LED_REFRESH_MS = 5000;

static_assert(C_GOOD != C_WARN_FILL && C_GOOD != C_BAD && C_WARN_FILL != C_BAD,
              "status LED is keyed off the banner colour; the three fills must differ");

static void statusLedWrite(const LedColour& c) {
    rgbLedWriteOrdered(LED_PIN, LED_ORDER,
                       (uint8_t)(c.r * LED_BRIGHTNESS / 255u),
                       (uint8_t)(c.g * LED_BRIGHTNESS / 255u),
                       (uint8_t)(c.b * LED_BRIGHTNESS / 255u));
}

// Called every display_update() pass: with the banner colour when the banner is
// painted, and with panelOff while the panel is off, so the LED goes dark with
// the screen rather than freezing on its last colour. Touches the wire only on a
// change of condition or every LED_REFRESH_MS; otherwise it is a compare.
static void serviceStatusLed(bool panelOff, uint16_t banner) {
    static uint8_t  lastKey     = 0xFF;   // 0xFF: not yet written by this function
    static uint32_t lastWriteMs = 0;
    uint8_t key; LedColour c;
    if      (panelOff)         { key = 0; c = LED_DARK; }
    // RTK is held back by internal RAM: a colour no stoplight state uses, so it cannot be
    // read as the ordinary red of "no corrections yet" from across the paddock.
    else if (status.rtkInhibit != RTK_INH_NONE) { key = 4; c = LED_INHIBIT; }
    else if (banner == C_GOOD) { key = 1; c = LED_OK;   }
    else if (banner == C_BAD)  { key = 3; c = LED_BAD;  }
    else                       { key = 2; c = LED_WARN; }
    const uint32_t now = millis();
    if (key == lastKey && (now - lastWriteMs) < LED_REFRESH_MS) return;
    const bool first = (lastKey == 0xFF);
    lastKey = key; lastWriteMs = now;
    statusLedWrite(c);
    // Reported once. A bead showing the wrong colour and a bead the firmware never
    // reached look identical on the bench; printing the REQUESTED colour and the
    // byte order next to what is actually glowing tells them apart, and asking for
    // red but seeing green is a byte-order mismatch fixed by LED_ORDER.
    if (first) Serial.printf("💡 Status LED: pin %u, order %d, first colour requested %u,%u,%u\n",
                             (unsigned)LED_PIN, (int)LED_ORDER, (unsigned)c.r, (unsigned)c.g, (unsigned)c.b);
}

static void txt(const char* s, int x, int y, uint16_t fg, uint8_t font=2, int padTo=0) {
    char padded[36];
    if (padTo > (int)strlen(s)) { snprintf(padded,sizeof(padded),"%-*s",padTo,s); s=padded; }
    tft.setTextColor(fg, C_BG);
    tft.drawString(s, x, y, font);
}
static void txtr(const char* s, int x, int y, uint16_t fg, uint8_t font=2) {
    tft.setTextColor(fg, C_BG);
    tft.drawRightString(s, x, y, font);
}

struct GFPoint { int16_t x,y; };
static GFPoint gfTrail[GC_TRAIL];
static int gfHead=0; static bool gfInit=false;

static void drawGForceBg() {
    const int cy=GC_CY-GC_Y0;
    gcSprite.fillSprite(C_BG);
    gcSprite.drawCircle(GC_CX,cy,GC_R,TFT_DARKGREY);
    gcSprite.drawCircle(GC_CX,cy,GC_SCALE,0xA514);
    gcSprite.drawCircle(GC_CX,cy,GC_SCALE/2,TFT_LIGHTGREY);
    gcSprite.drawFastHLine(GC_CX-GC_R,cy,GC_R*2,0xA514);
    gcSprite.drawFastVLine(GC_CX,cy-GC_R,GC_R*2,0xA514);
    gcSprite.setTextColor(C_DIM);
    gcSprite.drawString("1g",GC_CX+GC_SCALE+2,cy-4,1);
}

static void updateGForceCircle(float ax, float ay) {
    drawGForceBg();
    for (int i=0;i<GC_TRAIL;i++) {
        int idx=(gfHead+i)%GC_TRAIL;
        if (!gfInit && i>gfHead) continue;
        uint8_t age=GC_TRAIL-((gfHead-idx+GC_TRAIL)%GC_TRAIL);
        if (age<2) continue;
        gcSprite.drawPixel(gfTrail[idx].x,gfTrail[idx].y-GC_Y0,gcSprite.color565(255-age*10,255-age*11/2,255-age*8));
    }
    int16_t px=GC_CX+(int16_t)(ay*GC_SCALE);
    int16_t py=GC_CY-(int16_t)(ax*GC_SCALE);
    float dx=px-GC_CX,dy=py-GC_CY,dist=sqrt(dx*dx+dy*dy);
    if (dist>GC_R) { px=GC_CX+dx/dist*GC_R; py=GC_CY+dy/dist*GC_R; }
    gcSprite.fillCircle(px,py-GC_Y0,3,C_TEXT);
    gfTrail[gfHead]={px,py}; gfHead=(gfHead+1)%GC_TRAIL; gfInit=true;
}

void display_init() {
    // Backlight LAST, after tft.init() — attaching LEDC before tft.init() leaves
    // IO46 uncontrolled (tft.init() reconfigures it out from under the PWM
    // attach), so ledcWrite() has no effect and the backlight sticks at its
    // default-on state. Already found and fixed once on the RCX1 base's
    // identical hardware; keep this ordering. Pin, polarity (active-HIGH), and
    // duty scale (255 = full, 0 = off) are all confirmed against real hardware.
    tft.init(); tft.setRotation(0); tft.fillScreen(C_BG);
    ledcAttach(LCD_BL_PIN, LCD_BL_FREQ_HZ, LCD_BL_RES_BITS);
    ledcWrite(LCD_BL_PIN, (1 << LCD_BL_RES_BITS) - 1);   // 100%: identical to the HIGH this replaces
    gcSprite.createSprite(172, GC_SP_H);
    tft.fillRect(0,0,172,18,C_DIM);
    tft.setTextColor(TFT_BLACK); tft.drawCentreString("RCX Engineering",86,2,2);
    tft.setTextColor(C_DIM,C_BG);
    tft.drawString("Lat",2,144,2); tft.drawString("Lon",2,164,2);
    tft.drawFastHLine(0,140,172,TFT_DARKGREY);
    tft.drawFastHLine(0,204,172,TFT_DARKGREY);
    tft.drawFastHLine(0,248,172,TFT_DARKGREY);
    drawGForceBg(); gcSprite.pushSprite(0,GC_Y0);
    // Dark until the first banner paint. An undriven WS2812 powers up showing
    // whatever its register holds, commonly full white, and the status is not
    // meaningful until the first display_update() has real data behind it.
    statusLedWrite(LED_DARK);
}

static unsigned long lastDisplayUpdate = 0;
// WiFi rows are redrawn only on change (see display_update); -1 forces the next
// pass to draw them. File scope so a full-screen repaint can reset it.
static int lastWifiMode = -1;

// ── LCD/backlight power ─────────────────────────────────────────────────────
// Two independent inputs, either one enough to turn things down: the user's
// own web toggle (lcdUserEnabled, default on, never persisted — see display.h)
// and thermal throttling (thermal.h). Combined here into one small state
// machine so display_update() has one cheap check to make each pass, and
// hardware (the panel command, the backlight duty) is only touched on an
// actual transition rather than every ~200ms refresh.
static bool lcdUserEnabled = true;
static int  lastPanelState = -1;   // -1 unset, 0 off, 1 dim, 2 full — forces first apply

void display_setEnabled(bool on) { lcdUserEnabled = on; }
bool display_isEnabled()         { return lcdUserEnabled; }

// Returns true if the panel is currently OFF (caller should skip drawing).
static bool applyLcdPower() {
    bool off = !lcdUserEnabled || thermal_lcdOff();
    bool dim = !off && thermal_backlightDim();
    int  state = off ? 0 : dim ? 1 : 2;
    if (state != lastPanelState) {
        if (state == 0) {
            ledcWrite(LCD_BL_PIN, 0);
            tft.writecommand(TFT_DISPOFF);
        } else {
            if (lastPanelState == 0) tft.writecommand(TFT_DISPON);   // was off: wake the panel first
            uint32_t duty = (state == 1) ? (uint32_t)(((1 << LCD_BL_RES_BITS) - 1) * 0.20f)
                                          : (1 << LCD_BL_RES_BITS) - 1;
            ledcWrite(LCD_BL_PIN, duty);
        }
        lastPanelState = state;
    }
    return state == 0;
}

// ═════════════════════════════════════════════════════════════════════════════
// CONE SURVEY DISPLAY MODE
// ═════════════════════════════════════════════════════════════════════════════
// A second screen layout for surveying autocross cones with the unit sitting on
// the cone-positioning lid. It is purely a way of LOOKING at data the logger
// already has: it reads the same GNSS / IMU / status snapshot display_update()
// is handed, writes nothing to any shared state, logs nothing new, and changes
// nothing about GNSS, BLE, NTRIP, CAN or SD. Everything here runs inside the
// display task, the only task that touches the panel.
//
// ENTRY AND EXIT are automatic, from the IMU:
//   • held in the cone-scan orientation (Y axis up), mostly still and not
//     travelling, for CONE_MODE_ENTER_S → cone mode;
//   • horizontal (Z axis up or down) for CONE_MODE_EXIT_S → datalogger mode.
//   Both use a leaky timer rather than a strict continuous window, so a single
//   odd sample — a bump, or a default-valued snapshot on a missed mutex take —
//   costs a little time instead of restarting the count.
//   Two guards stop a vehicle-mounted unit ever showing this layout while it
//   matters: entry is refused while BLE is connected or the GNSS reports speed
//   above CONE_MODE_MAX_SPEED_MPS, and the mode drops out IMMEDIATELY (not
//   after the exit timer) when BLE connects or speed passes
//   CONE_MODE_ABORT_SPEED_MPS. The cone-mode banner deliberately ignores BLE,
//   so it must never be on screen in a car that is about to run.
//
// IN CONE MODE the panel is rotated 180° to read upright on the lid, and:
//   • the banner (and so the status LED) is RTK-only — green FIXED with
//     HAcc <= CONE_BANNER_HACC_M, amber FLOAT/PPP or a looser fix, red no RTK;
//   • the BLE and GPS/CAN rows become an accuracy band: the reported HAcc large
//     on the right; on the left the scatter-based estimate — live scatter while
//     stationary, and the dwell's surveyed accuracy while sitting on a cone;
//   • VAcc takes the small accuracy slot beside the satellite count;
//   • the g-meter area becomes a status block meant to be read from a step
//     back, in direct sun. States are told apart by BRIGHTNESS and MOTION, not
//     by hue: in sunlight the reflection off the glass adds the same light to
//     every pixel, so dark fills of different hues all read as the same near-
//     black and only large changes in emitted light survive (see the palette
//     notes at the top of this file). So:
//       READY  light grey, dark text        — nothing on a cone
//       HOLD   black, white text + progress — on a cone, surveying: keep clear
//       DONE   blinking bright green / black — surveyed: pick it up
//       FLOAT  yellow/black hazard stripes   — no RTK fix: stop and wait
//       REDO / DRIFT  full red, white text  — lifted early / fix wandering
//     DONE is the only state that moves and FLOAT the only one with a
//     pattern, so both are recognisable at a glance even where the colours
//     themselves have washed out. FLOAT is shown whenever the receiver is not
//     RTK FIXED — on a cone that is not yet surveyed, and between cones — so a
//     lost fix can never look like an ordinary HOLD or READY. DONE blinks only
//     while the unit rests on the cone it surveyed — including if the fix
//     drops afterwards, since that cone was surveyed fixed. Once lifted, the
//     block shows READY with that cone's result (or FLOAT), so the next cone
//     can never inherit a DONE while it is still being recognised.
//
// CONE DETECTION follows the West Course Cone Detection Algorithm Revision:
//   §1  pointwise cone-resting pose gate (thresholds below, unchanged from the
//       spec), evaluated as a ~1.0 s rolling pose fraction >= 0.65 rather than
//       a strict every-sample window;
//   §2  no hard inclusion cliff — a dwell starts, and HOLD is shown, as soon
//       as the pose fraction holds; the dwell's clock is backdated to the
//       first sample of the in-pose run that triggered it, so the ~0.65 s the
//       rolling fraction needs to recognise a placement is not lost from the
//       CONE_DONE_FIXED_MS budget;
//   §3  a dwell interrupted for up to CONE_MERGE_GAP_MS that resumes within
//       CONE_MERGE_DIST_M of its own position is the SAME placement (this also
//       covers the spec's 0.30 s gap bridging);
//   §5  RTK-fixed and non-fixed samples are kept apart, the dwell position is
//       the fixed-only median, and fixed-sample scatter while the IMU holds the
//       pose still is reported as DRIFT, not movement;
//   §6  only a fixed subset can produce DONE; a dwell with no fixed samples
//       stays at HOLD and says it is waiting for a fix.
//   The IMU bridge reconstruction, drift-validated dead reckoning and
//   upright/laydown classification (§4, §7–§9) need the whole session and stay
//   in post-processing; the logs this unit writes remain the source of truth.
//
// SURVEYED ACCURACY is the RMS horizontal distance of the dwell's fixed samples
// from their median. It is deliberately NOT divided by sqrt(N): RTK errors are
// correlated over seconds, so 20 Hz samples are far from independent and that
// division would report millimetres the data cannot support.
//
// COST: about 24 B of static RAM for mode detection, which runs in either mode.
// Everything the survey needs (sample buffer, median scratch, field caches,
// ~20 KB) is one PSRAM block allocated on entering cone mode and freed on
// leaving it; internal RAM and the display task stack are not touched.

// Mode switching
static constexpr float CONE_MODE_ENTER_S         = 10.0f;
static constexpr float CONE_MODE_EXIT_S          = 10.0f;
static constexpr float CONE_MODE_MAX_SPEED_MPS   = 0.5f;   // entry refused above this
static constexpr float CONE_MODE_ABORT_SPEED_MPS = 3.0f;   // immediate exit above this
// Mode-entry orientation: the cone-scan pose, loosened because it is hand-held
static constexpr float CONE_ENTER_AY_MIN    = 0.80f;
static constexpr float CONE_ENTER_AXZ_MAX   = 0.35f;
static constexpr float CONE_ENTER_GYRO_MAX  = 20.0f;       // deg/s
// Mode-exit orientation
static constexpr float CONE_EXIT_AZ_MIN     = 0.80f;       // |az|, either face up
static constexpr float CONE_EXIT_AY_MAX     = 0.50f;
// Cone-resting pose gate — spec §1, unchanged
static constexpr float CONE_AY_MIN          = 0.85f;
static constexpr float CONE_AY_MAX          = 1.18f;
static constexpr float CONE_AXZ_MAX         = 0.22f;
static constexpr float CONE_GYRO_MAX_DPS    = 6.0f;
static constexpr float CONE_ACC_MIN         = 0.88f;
static constexpr float CONE_ACC_MAX         = 1.18f;
static constexpr uint32_t CONE_POSE_WINDOW_MS  = 1000;     // spec §1 rolling window
static constexpr float    CONE_POSE_FRACTION   = 0.65f;    // spec §1
static constexpr uint8_t  CONE_POSE_MIN_SAMPLES = 5;       // before a fraction is trusted
// Dwell handling — spec §3 and the post-processing reference implementation
static constexpr uint32_t CONE_MERGE_GAP_MS    = 3000;
static constexpr float    CONE_MERGE_DIST_M    = 1.5f;
static constexpr uint32_t CONE_MIN_DWELL_MS    = 800;      // shorter: handling transient, ignored
// User cues
static constexpr uint32_t CONE_DONE_FIXED_MS   = 3000;     // RTK-fixed dwell, counted from placement, for DONE
static constexpr float    CONE_DONE_RMS_M      = 0.020f;   // surveyed scatter needed for DONE
static constexpr float    CONE_DRIFT_RMS_M     = 0.050f;   // fixed scatter this large = DRIFT
static constexpr uint32_t CONE_DRIFT_MIN_MS    = 1000;     // fixed data before DRIFT can be called
static constexpr float    CONE_BANNER_HACC_M   = 0.05f;    // green banner needs FIXED and this
// Sampling
static constexpr uint32_t CONE_EPOCH_FRESH_MS  = 500;      // a fixed epoch older than this stops the clock
static constexpr uint16_t CONE_BUF_SAMPLES     = 1200;     // 60 s of fixed epochs at 20 Hz
static constexpr uint16_t CONE_STATS_MIN       = 10;       // fixed samples before stats are shown
static constexpr float    CONE_SCATTER_TAU_S   = 2.0f;     // live scatter averaging time constant
static constexpr float    CONE_STILL_SPEED_MPS = 0.2f;     // live scatter only below this
// Status block colours — chosen for emitted brightness, see the header comment
static constexpr uint16_t CONE_READY_BG        = 0xD6DC;   // (214,219,226) light grey, READY
static constexpr uint16_t CONE_HOLD_BG         = TFT_BLACK;
static constexpr uint16_t CONE_DONE_LIT        = TFT_GREEN; // (0,255,0): the brightest green the panel has
static constexpr uint16_t CONE_ALERT_BG        = TFT_RED;   // (255,0,0): REDO, DRIFT
static constexpr uint16_t CONE_PROGRESS_TRACK  = TFT_DARKGREY;
static constexpr uint16_t CONE_FLOAT_BG        = TFT_YELLOW; // (255,255,0): the brightest colour the panel has
static constexpr int      CONE_STRIPE_PERIOD   = 16;         // hazard stripe pitch, px (half black, half yellow)

enum ConeResult : uint8_t { CONE_RES_NONE, CONE_RES_DONE, CONE_RES_REDO };
enum ConeField  : uint8_t { CF_L1, CF_L2, CF_NTRIP, CF_BAND, CF_FIX, CF_REC, CF_SATS, CF_VACC,
                            CF_LAT, CF_LON, CF_CPU, CF_LEFT_LBL, CF_LEFT, CF_RIGHT, CF_COUNT };

struct ConeSample { float e, n, u; };   // metres from the dwell origin

struct ConeState {
    // Rolling pose fraction (spec §1)
    uint32_t poseT[32];
    uint8_t  poseOk[32];
    uint8_t  poseHead, poseCount;
    // Current dwell
    bool     dwellOpen;          // a placement exists: in pose now, or inside the merge gap
    bool     inPose;
    uint32_t gapStartMs;
    uint32_t activeMs;           // time in pose
    uint32_t fixedMs;            // time in pose with a fresh RTK-fixed epoch
    bool     done;
    bool     originSet;
    double   lat0, lon0; float alt0;
    uint16_t nFixed, nOther;
    ConeSample buf[CONE_BUF_SAMPLES];
    float    scratch[CONE_BUF_SAMPLES];
    bool     statsValid;
    float    medE, medN, medU, rmsH, rmsU;
    uint32_t statsMs;
    // GNSS sampling
    uint32_t lastEpoch;
    uint32_t lastFixedEpochMs;
    uint32_t lastNotFixedMs;     // last epoch that was NOT RTK fixed (limits backdated fixed time)
    // Previous dwell
    ConeResult lastResult;
    float    lastRmsH;
    // Live scatter (EWMA while stationary)
    bool     ewInit;
    uint16_t ewCount;
    double   ewLat0, ewLon0;
    float    ewE, ewN, ewVar;
    uint32_t ewLastMs;
    // Drawn-field caches: a field is repainted only when its text or colour changes
    char     fText[CF_COUNT][40];   // holds the longest field: a 32-char SSID or 31-char device name
    uint16_t fFg[CF_COUNT];
    uint16_t bannerDrawn;
};

static bool       coneModeActive  = false;   // mode as decided by the IMU
static bool       coneLayoutDrawn = false;   // layout currently on the panel
static float      coneEnterS = 0.0f, coneExitS = 0.0f;
static uint32_t   coneLastMs = 0;
static ConeState* cs = nullptr;              // PSRAM, cone mode only

// Written only by the display task; a single-byte read from another core sees
// either the old or the new value, and one pass of lag is harmless to callers.
bool display_coneModeActive() { return coneModeActive; }

static constexpr double CONE_EARTH_R = 6378137.0;

static void coneToLocal(double lat, double lon, double lat0, double lon0, float& e, float& n) {
    e = (float)((lon - lon0) * DEG_TO_RAD * CONE_EARTH_R * cos(lat0 * DEG_TO_RAD));
    n = (float)((lat - lat0) * DEG_TO_RAD * CONE_EARTH_R);
}

static float coneMedian(const float* src, uint16_t n, float* scratch) {
    memcpy(scratch, src, n * sizeof(float));
    std::nth_element(scratch, scratch + n / 2, scratch + n);
    return scratch[n / 2];
}

// Fixed-only median position and RMS scatter about it (spec §5).
static void coneUpdateStats() {
    const uint16_t n = cs->nFixed;
    if (n < CONE_STATS_MIN) { cs->statsValid = false; return; }
    for (uint16_t i = 0; i < n; i++) cs->scratch[i] = cs->buf[i].e;
    cs->medE = coneMedian(cs->scratch, n, cs->scratch);
    for (uint16_t i = 0; i < n; i++) cs->scratch[i] = cs->buf[i].n;
    cs->medN = coneMedian(cs->scratch, n, cs->scratch);
    for (uint16_t i = 0; i < n; i++) cs->scratch[i] = cs->buf[i].u;
    cs->medU = coneMedian(cs->scratch, n, cs->scratch);
    double sh = 0.0, su = 0.0;
    for (uint16_t i = 0; i < n; i++) {
        const float de = cs->buf[i].e - cs->medE, dn = cs->buf[i].n - cs->medN, du = cs->buf[i].u - cs->medU;
        sh += (double)de * de + (double)dn * dn;
        su += (double)du * du;
    }
    cs->rmsH = (float)sqrt(sh / n);
    cs->rmsU = (float)sqrt(su / n);
    cs->statsValid = true;
}

static void coneResetDwell() {
    cs->dwellOpen = false; cs->inPose = false;
    cs->activeMs = 0; cs->fixedMs = 0;
    cs->done = false;
    cs->originSet = false; cs->nFixed = 0; cs->nOther = 0;
    cs->statsValid = false;
}

static void coneCloseDwell() {
    if (cs->done) {
        cs->lastResult = CONE_RES_DONE;
        cs->lastRmsH   = cs->statsValid ? cs->rmsH : NAN;
    } else if (cs->activeMs >= CONE_MIN_DWELL_MS) {
        cs->lastResult = CONE_RES_REDO;
    }
    // One line per placement: the on-device verdict, next to the logs that
    // post-processing will judge it against.
    if (cs->activeMs >= CONE_MIN_DWELL_MS) {
        Serial.printf("📍 Cone %s: dwell %.1f s, fixed %u of %u epochs, scatter H %.0f mm V %.0f mm\n",
                      cs->done ? "DONE" : "REDO", cs->activeMs / 1000.0f, (unsigned)cs->nFixed,
                      (unsigned)(cs->nFixed + cs->nOther),
                      cs->statsValid ? cs->rmsH * 1000.0f : NAN, cs->statsValid ? cs->rmsU * 1000.0f : NAN);
    }
    coneResetDwell();
}

// One call per display_update() pass (~20 Hz), whether or not the panel is on.
static void coneSurveyTrack(const GnssData& g, const ImuData& m, uint32_t now, uint32_t dtMs) {
    // Pose sample and rolling fraction (spec §1)
    const float gyro = sqrtf(m.gx*m.gx + m.gy*m.gy + m.gz*m.gz);
    const float acc  = sqrtf(m.ax*m.ax + m.ay*m.ay + m.az*m.az);
    const bool  ok   = m.ay >= CONE_AY_MIN && m.ay <= CONE_AY_MAX &&
                       fabsf(m.ax) < CONE_AXZ_MAX && fabsf(m.az) < CONE_AXZ_MAX &&
                       gyro < CONE_GYRO_MAX_DPS && acc >= CONE_ACC_MIN && acc <= CONE_ACC_MAX;
    cs->poseT[cs->poseHead] = now; cs->poseOk[cs->poseHead] = ok;
    cs->poseHead = (cs->poseHead + 1) & 31;
    if (cs->poseCount < 32) cs->poseCount++;
    uint8_t inWin = 0, okWin = 0;
    for (uint8_t i = 0; i < cs->poseCount; i++) {
        const uint8_t k = (cs->poseHead + 31 - i) & 31;
        if (now - cs->poseT[k] > CONE_POSE_WINDOW_MS) break;
        inWin++; okWin += cs->poseOk[k];
    }
    const bool pose = inWin >= CONE_POSE_MIN_SAMPLES && okWin >= CONE_POSE_FRACTION * inWin;

    // New GNSS epoch? (a default snapshot from a missed mutex take is invalid and skipped)
    const bool newEpoch = g.valid && g.epochSeq != 0 && g.epochSeq != cs->lastEpoch;
    if (newEpoch) cs->lastEpoch = g.epochSeq;
    const bool fixedNow = g.valid && g.rtkType == 2;
    if (newEpoch) { if (fixedNow) cs->lastFixedEpochMs = now; else cs->lastNotFixedMs = now; }
    const float speed = g.valid ? (float)(g.speedKnots * 0.514444) : 0.0f;

    // Live scatter: exponentially weighted spread while stationary
    if (newEpoch) {
        const bool still = gyro < CONE_GYRO_MAX_DPS && speed < CONE_STILL_SPEED_MPS;
        if (!still) { cs->ewInit = false; }
        else {
            if (!cs->ewInit) {
                cs->ewInit = true; cs->ewCount = 0; cs->ewLat0 = g.latitude; cs->ewLon0 = g.longitude;
                cs->ewE = 0; cs->ewN = 0; cs->ewVar = 0; cs->ewLastMs = now;
            }
            float e, n; coneToLocal(g.latitude, g.longitude, cs->ewLat0, cs->ewLon0, e, n);
            const float a = fminf(1.0f, ((now - cs->ewLastMs) / 1000.0f) / CONE_SCATTER_TAU_S);
            cs->ewLastMs = now;
            const float de = e - cs->ewE, dn = n - cs->ewN;
            cs->ewE += a * de; cs->ewN += a * dn;
            cs->ewVar += a * ((de*de + dn*dn) - cs->ewVar);
            if (cs->ewCount < 0xFFFF) cs->ewCount++;
        }
    }

    // Dwell open / merge / close (spec §3)
    if (pose) {
        if (cs->dwellOpen && !cs->inPose) {
            // A placement already DONE is never extended: resting again — on the
            // next cone, or after a knock — starts a new survey, so a finished
            // cone cannot lend its result to one set down beside it.
            bool same = !cs->done;
            if (same && cs->statsValid && g.valid) {
                float e, n; coneToLocal(g.latitude, g.longitude, cs->lat0, cs->lon0, e, n);
                same = hypotf(e - cs->medE, n - cs->medN) <= CONE_MERGE_DIST_M;
            }
            if (!same) coneCloseDwell();
        }
        uint32_t credit = dtMs, fixedCredit = dtMs;
        if (!cs->dwellOpen) {
            coneResetDwell(); cs->dwellOpen = true;
            // Backdate to placement: the unit has been resting since the oldest
            // sample of the unbroken in-pose run that ends at this detection.
            uint32_t first = now;
            for (uint8_t i = 0; i < cs->poseCount; i++) {
                const uint8_t k = (cs->poseHead + 31 - i) & 31;
                if (!cs->poseOk[k] || now - cs->poseT[k] > CONE_POSE_WINDOW_MS) break;
                first = cs->poseT[k];
            }
            credit = now - first;
            // Fixed time is backdated only as far as the last epoch that was not
            // fixed, so a float-to-fixed change during placement is not counted.
            const uint32_t sinceNotFixed = now - cs->lastNotFixedMs;
            fixedCredit = credit < sinceNotFixed ? credit : sinceNotFixed;
        }
        cs->inPose = true;
        cs->activeMs += credit;
        const bool fixedFresh = fixedNow && (now - cs->lastFixedEpochMs) <= CONE_EPOCH_FRESH_MS;
        if (fixedFresh) cs->fixedMs += fixedCredit;
        // Positions are taken only from samples that pass the pointwise pose
        // gate themselves, and only until the cone is DONE. The rolling
        // fraction that holds the dwell open lags a lift by ~0.35 s; positions
        // from that moment are the antenna already leaving the cone, and a DONE
        // survey is final — later samples cannot improve it, only corrupt it.
        if (newEpoch && ok && !cs->done) {
            if (fixedNow) {
                if (!cs->originSet) {
                    cs->originSet = true; cs->lat0 = g.latitude; cs->lon0 = g.longitude; cs->alt0 = (float)g.altMSL;
                }
                if (cs->nFixed < CONE_BUF_SAMPLES) {
                    ConeSample& sm = cs->buf[cs->nFixed++];
                    coneToLocal(g.latitude, g.longitude, cs->lat0, cs->lon0, sm.e, sm.n);
                    sm.u = (float)g.altMSL - cs->alt0;
                }
            } else if (cs->nOther < 0xFFFF) {
                cs->nOther++;
            }
        }
    } else if (cs->dwellOpen) {
        if (cs->inPose) { cs->inPose = false; cs->gapStartMs = now; }
        else if (now - cs->gapStartMs > CONE_MERGE_GAP_MS) coneCloseDwell();
    }

    // Statistics and cues, at the panel's own 5 Hz pace
    if (cs->dwellOpen && now - cs->statsMs >= 200) {
        cs->statsMs = now;
        coneUpdateStats();
        if (!cs->done && cs->statsValid &&
            cs->fixedMs >= CONE_DONE_FIXED_MS && cs->rmsH <= CONE_DONE_RMS_M) {
            cs->done = true;
        }
    }
}

// Mode decision. Runs every display_update() pass in both modes; the survey
// itself only runs while the mode is active.
static void coneTrack(const GnssData& g, const ImuData& m, const SystemStatus& s) {
    const uint32_t now = millis();
    uint32_t dtMs = coneLastMs ? now - coneLastMs : 0;
    coneLastMs = now;
    if (dtMs > 500) dtMs = 500;                  // a stalled pass is not evidence either way
    const float dt = dtMs / 1000.0f;

    const float speed = g.valid ? (float)(g.speedKnots * 0.514444) : 0.0f;
    const float gyro  = sqrtf(m.gx*m.gx + m.gy*m.gy + m.gz*m.gz);

    if (!coneModeActive) {
        const bool vertical = m.ay >= CONE_ENTER_AY_MIN && fabsf(m.ax) < CONE_ENTER_AXZ_MAX &&
                              fabsf(m.az) < CONE_ENTER_AXZ_MAX && gyro < CONE_ENTER_GYRO_MAX &&
                              !s.bleConnected && speed <= CONE_MODE_MAX_SPEED_MPS;
        coneEnterS = vertical ? coneEnterS + dt : fmaxf(0.0f, coneEnterS - 2.0f * dt);
        if (coneEnterS >= CONE_MODE_ENTER_S) {
            coneModeActive = true; coneEnterS = 0.0f; coneExitS = 0.0f;
            cs = (ConeState*)heap_caps_calloc(1, sizeof(ConeState), MALLOC_CAP_SPIRAM);
            Serial.printf("📍 LCD: cone survey mode ON%s\n",
                          cs ? "" : " — PSRAM allocation failed, survey figures disabled");
        }
        return;
    }

    const bool abort      = s.bleConnected || speed > CONE_MODE_ABORT_SPEED_MPS;
    const bool horizontal = fabsf(m.az) >= CONE_EXIT_AZ_MIN && fabsf(m.ay) < CONE_EXIT_AY_MAX;
    coneExitS = horizontal ? coneExitS + dt : fmaxf(0.0f, coneExitS - 2.0f * dt);
    if (abort || coneExitS >= CONE_MODE_EXIT_S) {
        coneModeActive = false; coneEnterS = 0.0f; coneExitS = 0.0f;
        if (cs && cs->dwellOpen) coneCloseDwell();
        heap_caps_free(cs); cs = nullptr;
        Serial.printf("📍 LCD: cone survey mode OFF (%s)\n",
                      s.bleConnected ? "BLE connected" : abort ? "moving" : "horizontal");
        return;
    }
    if (cs) coneSurveyTrack(g, m, now, dtMs);
}

// ── Cone-mode drawing ────────────────────────────────────────────────────────
// Fields are painted only when their text or colour changes, and never blanked
// first: the new text is drawn opaque over the old, then only the strip beyond
// its end is cleared. There is no instant at which a field is empty, so there
// is nothing to blink, and nothing of a longer old string is left behind.
static void coneField(ConeField f, int x, int y, int w, const char* s, uint16_t fg,
                      uint8_t font, uint8_t size = 1, bool right = false) {
    if (cs) {
        if (cs->fFg[f] == fg && strncmp(cs->fText[f], s, sizeof(cs->fText[f])) == 0) return;
        strncpy(cs->fText[f], s, sizeof(cs->fText[f]) - 1); cs->fText[f][sizeof(cs->fText[f]) - 1] = '\0';
        cs->fFg[f] = fg;
    }
    const int h  = (font == 1 ? 8 : font == 2 ? 16 : 26) * size;   // GLCD / Font 2 / Font 4
    tft.setTextSize(size);
    tft.setTextColor(fg, C_BG);
    const int tw = tft.textWidth(s, font);
    if (right) { tft.drawRightString(s, x + w, y, font); if (tw < w) tft.fillRect(x, y, w - tw, h, C_BG); }
    else       { tft.drawString(s, x, y, font);           if (tw < w) tft.fillRect(x + tw, y, w - tw, h, C_BG); }
    tft.setTextSize(1);
}

static void coneFormatM(char* out, size_t n, float m) {
    if (isnan(m))       snprintf(out, n, "--");
    else if (m < 0.1f)  snprintf(out, n, "%.0fmm", m * 1000.0f);
    else if (m < 10.0f) snprintf(out, n, "%.2fm", m);
    else                snprintf(out, n, "%.1fm", m);
}

// RTK-only stoplight for the cone-mode banner and status LED.
static uint16_t coneHealthColor(const GnssData& g) {
    if (!g.valid) return C_BAD;
    if (g.rtkType == 2) return g.hAccM <= CONE_BANNER_HACC_M ? C_GOOD : C_WARN_FILL;
    if (g.rtkType == 1 || g.pppActive) return C_WARN_FILL;
    return C_BAD;
}

static void paintConeStatic() {
    tft.setRotation(2);
    tft.fillScreen(C_BG);
    tft.setTextColor(C_DIM, C_BG);
    tft.drawString("Lat", 2, 144, 2); tft.drawString("Lon", 2, 164, 2); tft.drawString("CPU", 2, 184, 2);
    tft.drawRightString("H ACC", 170, 207, 1);
    tft.drawFastHLine(0, 140, 172, TFT_DARKGREY);
    tft.drawFastHLine(0, 204, 172, TFT_DARKGREY);
    if (cs) { memset(cs->fText, 0, sizeof(cs->fText)); memset(cs->fFg, 0, sizeof(cs->fFg)); cs->bannerDrawn = 0xFFFF; }
    coneLayoutDrawn = true;
}

// The datalogger layout drawn by display_init(), repainted on leaving cone mode.
// Everything else on that screen is redrawn by the same display_update() pass,
// and lastWifiMode = -1 makes the change-gated WiFi rows repaint too.
static void restoreNormalLayout() {
    tft.setRotation(0);
    tft.fillScreen(C_BG);
    tft.setTextColor(C_DIM, C_BG);
    tft.drawString("Lat", 2, 144, 2); tft.drawString("Lon", 2, 164, 2);
    tft.drawFastHLine(0, 140, 172, TFT_DARKGREY);
    tft.drawFastHLine(0, 204, 172, TFT_DARKGREY);
    tft.drawFastHLine(0, 248, 172, TFT_DARKGREY);
    lastWifiMode = -1;
    coneLayoutDrawn = false;
}

static void drawConeStatusBlock(const char* word, const char* sub, uint16_t bg, uint16_t fg, float progress) {
    gcSprite.fillSprite(bg);
    gcSprite.setTextColor(fg, bg);
    gcSprite.setTextSize(2);
    gcSprite.drawCentreString(word, 86, 1, 4);    // Font 4 doubled: 52 px, heavy strokes for distance
    gcSprite.setTextSize(1);
    if (progress >= 0.0f) {
        const int w = (int)(156 * fminf(progress, 1.0f));
        gcSprite.fillRect(8, 58, 156, 10, CONE_PROGRESS_TRACK);
        gcSprite.fillRect(8, 58, w, 10, fg);
    } else {
        gcSprite.drawCentreString(sub, 86, 55, 2);
    }
    gcSprite.pushSprite(0, GC_Y0);
}

// No-fix block: diagonal yellow/black hazard stripes with the fix state in a
// clean yellow panel across the middle. The word is drawn opaque, so its glyph
// cells paint that panel and the stripes remain as a frame around it.
static void drawConeFloatBlock(const GnssData& g) {
    const char* word = !g.valid ? "NO FIX" : g.pppActive ? "PPP" : (g.rtkType == 1) ? "FLOAT" : "no RTK";
    gcSprite.fillSprite(CONE_FLOAT_BG);
    for (int y = 0; y < GC_SP_H; y++)
        for (int x = -(y % CONE_STRIPE_PERIOD); x < 172; x += CONE_STRIPE_PERIOD)
            gcSprite.drawFastHLine(x, y, CONE_STRIPE_PERIOD / 2, TFT_BLACK);
    gcSprite.setTextColor(TFT_BLACK, CONE_FLOAT_BG);
    gcSprite.setTextSize(2);
    gcSprite.drawCentreString(word, 86, 10, 4);
    gcSprite.setTextSize(1);
    gcSprite.pushSprite(0, GC_Y0);
}

static void drawConeScreen(const GnssData& g, const SystemStatus& s) {
    if (!coneLayoutDrawn) paintConeStatic();
    char buf[40];

    const uint16_t banner = coneHealthColor(g);
    if (!cs || cs->bannerDrawn != banner) {
        tft.fillRect(0, 0, 172, 18, banner);
        tft.setTextColor(TFT_BLACK); tft.drawCentreString("CONE SURVEY", 86, 2, 2);
        if (cs) cs->bannerDrawn = banner;
    }
    serviceStatusLed(false, banner);

    // Connection rows — same content and colours as the datalogger layout
    if (s.wifiConnected) {
        coneField(CF_L1, 2, 20, 168, s.wifiSSID, C_GOOD, 2);
        coneField(CF_L2, 2, 40, 168, s.ipAddress, C_GOOD, 2);
    } else if (s.wifiAttempting) {
        coneField(CF_L1, 2, 20, 168, s.wifiSSID, C_WARN, 2);
        coneField(CF_L2, 2, 40, 168, "connecting...", C_DIM, 2);
    } else {
        coneField(CF_L1, 2, 20, 168, wifi_deviceName(), C_WARN, 2);
        snprintf(buf, sizeof(buf), "http://%s", wifi_apIp());
        coneField(CF_L2, 2, 40, 168, buf, C_WARN, 2);
    }
    if (s.ntripConnected) {
        char mpt[12]; strncpy(mpt, s.mountpoint, 11); mpt[11] = '\0';
        if (s.ntripDistanceKm < 10.0f) snprintf(buf, sizeof(buf), "%s  %.1fkm", mpt, s.ntripDistanceKm);
        else                           snprintf(buf, sizeof(buf), "%s  %.0fkm", mpt, s.ntripDistanceKm);
        coneField(CF_NTRIP, 2, 60, 168, buf, C_GOOD, 2);
        const char* bands = (s.ntripCarrier == 2) ? "L1+L2" : (s.ntripCarrier == 1) ? "L1" : "?";
        snprintf(buf, sizeof(buf), "%s%s", bands, s.ntripVRS ? "  VRS" : "");
        if (s.rtkInhibit == RTK_INH_HEAP_CRIT) coneField(CF_BAND, 2, 80, 168, rtkInhibitLcd(s.rtkInhibit), C_BAD, 2);
        else coneField(CF_BAND, 2, 80, 168, buf, s.ntripCarrier == 2 ? C_GOOD : C_WARN, 2);
    } else {
        if (s.rtkInhibit != RTK_INH_NONE) coneField(CF_NTRIP, 2, 60, 168, rtkInhibitLcd(s.rtkInhibit), C_BAD, 2);
        else                              coneField(CF_NTRIP, 2, 60, 168, "---", C_WARN, 2);
        coneField(CF_BAND, 2, 80, 168, "", C_DIM, 2);
    }

    { const char* str; uint16_t col;
      if      (g.rtkType == 2) { str = "FIXED OK"; col = C_GOOD; }
      else if (g.pppActive)    { str = (g.diffStationId == 9002) ? "PPP E6" : "PPP B2b"; col = C_WARN; }
      else if (g.rtkType == 1) { str = "FLOAT";    col = C_WARN; }
      else if (g.valid)        { str = "3D GPS";   col = C_WARN; }
      else                     { str = "NO FIX";   col = C_BAD;  }
      coneField(CF_FIX, 2, 100, 104, str, col, 2); }
    { uint32_t lb = sdlog_getActiveGpsSize();
      if      (lb == 0)      snprintf(buf, sizeof(buf), "REC --");
      else if (lb < 1024)    snprintf(buf, sizeof(buf), "REC %uB",   (unsigned)lb);
      else if (lb < 1048576) snprintf(buf, sizeof(buf), "REC %.1fK", lb / 1024.0f);
      else                   snprintf(buf, sizeof(buf), "REC %.1fM", lb / 1048576.0f);
      coneField(CF_REC, 108, 104, 62, buf, lb ? C_GOOD : C_DIM, 1); }

    snprintf(buf, sizeof(buf), "%d sats", g.numSV);
    coneField(CF_SATS, 2, 120, 84, buf, satColor(g.numSV), 2);
    { char v[16]; coneFormatM(v, sizeof(v), (float)g.vAccM);
      snprintf(buf, sizeof(buf), "V %s", v);
      coneField(CF_VACC, 86, 120, 84, buf, accuracyColor((float)g.vAccM), 2, 1, true); }

    { const uint16_t c = accuracyColor((float)g.hAccM);
      snprintf(buf, sizeof(buf), "%.9f", g.latitude);  coneField(CF_LAT, 38, 144, 132, buf, c, 2);
      snprintf(buf, sizeof(buf), "%.9f", g.longitude); coneField(CF_LON, 38, 164, 132, buf, c, 2); }

    if (isnan(s.espTempC)) {
        coneField(CF_CPU, 70, 184, 100, "-- \xB0""F", C_DIM, 2, 1, true);
    } else {
        const uint16_t tc = (s.espTempC >= 100.0f) ? C_BAD : (s.espTempC >= 85.0f) ? C_WARN : C_GOOD;
        snprintf(buf, sizeof(buf), "%.0f\xB0""F", cToF(s.espTempC));
        coneField(CF_CPU, 70, 184, 100, buf, tc, 2, 1, true);
    }

    // Accuracy band: scatter estimate left, reported HAcc right, in Font 4 (26 px)
    { const bool surveying = cs && cs->dwellOpen && cs->statsValid;
      float left = NAN;
      if (surveying)                                          left = cs->rmsH;
      else if (cs && cs->ewInit && cs->ewCount >= CONE_STATS_MIN) left = sqrtf(fmaxf(cs->ewVar, 0.0f));
      coneField(CF_LEFT_LBL, 2, 207, 84, surveying ? "SURVEYED" : "SCATTER", C_DIM, 1);
      char v[16]; coneFormatM(v, sizeof(v), left);
      coneField(CF_LEFT, 2, 217, 84, v, isnan(left) ? C_DIM : C_TEXT, 4);
      coneFormatM(v, sizeof(v), (float)g.hAccM);
      coneField(CF_RIGHT, 86, 217, 84, v, accuracyColor((float)g.hAccM), 4, 1, true); }

    // Status block — the cue meant to be read from a step back
    if (!cs) { drawConeStatusBlock("READY", "no survey memory", CONE_READY_BG, C_TEXT, -1.0f); return; }
    // DONE alternates lit and dark on every panel refresh (200-250 ms), i.e.
    // 2-2.5 flashes a second: fast enough to read as blinking from a step back,
    // and under the 3-per-second general-flash limit (WCAG 2.3.1).
    static bool doneLit = false;
    const bool rtkFixed = g.valid && g.rtkType == 2;
    if (!rtkFixed && !(cs->dwellOpen && cs->inPose && cs->done)) {
        drawConeFloatBlock(g);
    } else if (cs->dwellOpen && !(cs->done && !cs->inPose)) {
        if (cs->statsValid && cs->fixedMs >= CONE_DRIFT_MIN_MS && cs->rmsH > CONE_DRIFT_RMS_M) {
            drawConeStatusBlock("DRIFT", "fix wandering", CONE_ALERT_BG, TFT_WHITE, -1.0f);
        } else if (cs->done) {
            char v[16]; coneFormatM(v, sizeof(v), cs->rmsH);
            snprintf(buf, sizeof(buf), "%s  lift cone", v);
            doneLit = !doneLit;
            if (doneLit) drawConeStatusBlock("DONE", buf, CONE_DONE_LIT, TFT_BLACK, -1.0f);
            else         drawConeStatusBlock("DONE", buf, TFT_BLACK, CONE_DONE_LIT, -1.0f);
        } else if (cs->nFixed == 0) {
            drawConeStatusBlock("HOLD", "waiting for FIX", CONE_HOLD_BG, TFT_WHITE, -1.0f);
        } else if (cs->statsValid && cs->fixedMs >= CONE_DONE_FIXED_MS) {
            // Enough fixed data, but not yet tight enough to call DONE
            char v[16]; coneFormatM(v, sizeof(v), cs->rmsH);
            snprintf(buf, sizeof(buf), "settling %s", v);
            drawConeStatusBlock("HOLD", buf, CONE_HOLD_BG, TFT_WHITE, -1.0f);
        } else {
            drawConeStatusBlock("HOLD", nullptr, CONE_HOLD_BG, TFT_WHITE,
                                cs->fixedMs / (float)CONE_DONE_FIXED_MS);
        }
    } else if (!cs->dwellOpen && cs->lastResult == CONE_RES_REDO) {
        drawConeStatusBlock("REDO", "lifted early", CONE_ALERT_BG, TFT_WHITE, -1.0f);
    } else if ((cs->dwellOpen && cs->done) ||
               (!cs->dwellOpen && cs->lastResult == CONE_RES_DONE && !isnan(cs->lastRmsH))) {
        // A DONE cone that has been lifted: its result, never a DONE that could
        // be mistaken for the next cone's while that one is being recognised.
        char v[16]; coneFormatM(v, sizeof(v), cs->dwellOpen ? cs->rmsH : cs->lastRmsH);
        snprintf(buf, sizeof(buf), "last cone %s", v);
        drawConeStatusBlock("READY", buf, CONE_READY_BG, C_TEXT, -1.0f);
    } else {
        drawConeStatusBlock("READY", "set on cone", CONE_READY_BG, C_TEXT, -1.0f);
    }
}

void display_update(const GnssData& g, const ImuData& m, const SystemStatus& s) {
    coneTrack(g, m, s);             // mode decision + survey, panel on or off
    if (applyLcdPower()) {          // panel off: nothing further to draw
        serviceStatusLed(true, 0);  // and the LED goes dark with it
        return;
    }
    if (millis()-lastDisplayUpdate < 200) return;
    lastDisplayUpdate=millis();
    if (coneModeActive) { drawConeScreen(g, s); return; }
    if (coneLayoutDrawn) restoreNormalLayout();
    char buf[40];

    const uint16_t banner = healthColor(g,s);
    tft.fillRect(0,0,172,18,banner);
    serviceStatusLed(false, banner);
    tft.setTextColor(TFT_BLACK); tft.drawCentreString("RCX Engineering",86,2,2);

    // WiFi rows. With no station link these show the configuration AP instead of
    // a blank SSID and "---": on a unit with no stored networks that AP is the
    // only way in, so the screen has to say what to join and where to browse.
    // A live station link takes precedence — it is the more useful address.
    //
    // Drawn only when the content actually changes (new SSID, new IP, or a
    // transition between connected/attempting/AP-fallback) — not on every
    // ~200ms display refresh. The block is wiped before drawing rather than
    // relying on txt()'s space padding to self-clear: the AP line pads to 21
    // characters but the connected SSID/IP lines pad to only 14-15, and font 2
    // is proportional-width besides, so neither the character count nor the
    // padding matches pixel-for-pixel across a state change. Gating the whole
    // block on a real change is also what stops it from blinking — wiping and
    // redrawing unchanged text every cycle produced a visible flicker with no
    // benefit, since text that hasn't changed needs no re-clear.
    {
        static char lastLine1[40] = "", lastLine2[48] = "";
        char line1[40], line2[48];
        int  mode;
        if (s.wifiConnected) {
            mode = 0;
            snprintf(line1,sizeof(line1),"%s",s.wifiSSID);
            snprintf(line2,sizeof(line2),"%s",s.ipAddress);
        } else if (s.wifiAttempting) {
            mode = 1;
            snprintf(line1,sizeof(line1),"%s",s.wifiSSID);
            snprintf(line2,sizeof(line2),"connecting...");
        } else {
            // Buffer is generously sized against the 32-character device name; the
            // panel is only ~21 characters wide at this font, so the name is shown
            // bare rather than behind an "AP" label that would cost three of them.
            mode = 2;
            snprintf(line1,sizeof(line1),"%s",wifi_deviceName());
            snprintf(line2,sizeof(line2),"http://%s",wifi_apIp());
        }
        if (mode != lastWifiMode || strcmp(line1,lastLine1) || strcmp(line2,lastLine2)) {
            tft.fillRect(0,18,172,40,C_BG);
            if (mode == 0) {
                txt(line1,2,20,C_GOOD,2,14);
                txt(line2,2,40,C_GOOD,2,15);
            } else if (mode == 1) {
                txt(line1,2,20,C_WARN,2,14);
                txt(line2,2,40,C_DIM,2,15);
            } else {
                txt(line1,2,20,C_WARN,2,21);
                txt(line2,2,40,C_WARN,2,21);
            }
            lastWifiMode = mode;
            strncpy(lastLine1,line1,sizeof(lastLine1)-1); lastLine1[sizeof(lastLine1)-1]='\0';
            strncpy(lastLine2,line2,sizeof(lastLine2)-1); lastLine2[sizeof(lastLine2)-1]='\0';
        }
    }

    if (s.ntripConnected) {
        char mpt[12]; strncpy(mpt,s.mountpoint,11); mpt[11]='\0';
        if (s.ntripDistanceKm<10.0f) snprintf(buf,sizeof(buf),"%s  %.1fkm",mpt,s.ntripDistanceKm);
        else                          snprintf(buf,sizeof(buf),"%s  %.0fkm",mpt,s.ntripDistanceKm);
        txt(buf,2,60,C_GOOD,2,15);
        const char* bands=(s.ntripCarrier==2)?"L1+L2":(s.ntripCarrier==1)?"L1":"?";
        snprintf(buf,sizeof(buf),"%s%s",bands,s.ntripVRS?"  VRS":"");
        if (s.rtkInhibit==RTK_INH_HEAP_CRIT) txt(rtkInhibitLcd(s.rtkInhibit),2,80,C_BAD,2,12);
        else txt(buf,2,80,s.ntripCarrier==2?C_GOOD:C_WARN,2,12);
    } else {
        if (s.rtkInhibit!=RTK_INH_NONE) txt(rtkInhibitLcd(s.rtkInhibit),2,60,C_BAD,2,15);
        else                            txt("---",2,60,C_WARN,2,15);
        txt("",2,80,C_DIM,2,12);
    }

    // Fix state. PPP shares GGA quality 5 with RTK FLOAT, so it is checked FIRST — a PPP
    // solution is base-less and decimetre-class, and calling it "FLOAT" on the dash would
    // imply corrections are flowing from our base when none are. Padded to 9 chars so the
    // longer "PPP E6" / "PPP B2b" strings self-clear.
    { const char* str; uint16_t col;
      if      (g.rtkType==2) { str="FIXED OK"; col=C_GOOD; }
      else if (g.pppActive)  { str=(g.diffStationId==9002)?"PPP E6":"PPP B2b"; col=C_WARN; }
      else if (g.rtkType==1) { str="FLOAT";    col=C_WARN; }
      else if (g.valid)      { str="3D GPS";   col=C_WARN; }
      else                   { str="NO FIX";   col=C_BAD;  }
      txt(str,2,100,col,2,9); }

    // Active recording size (GPS log, on-disk) on the right of the RTK row.
    // Small font + padded field so it can't collide with the RTK text on the
    // left and self-clears as the number grows/shrinks. "REC --" when idle.
    { uint32_t lb = sdlog_getActiveGpsSize();
      if      (lb == 0)      snprintf(buf,sizeof(buf),"REC --");
      else if (lb < 1024)    snprintf(buf,sizeof(buf),"REC %uB",  (unsigned)lb);
      else if (lb < 1048576) snprintf(buf,sizeof(buf),"REC %.1fK", lb/1024.0f);
      else                   snprintf(buf,sizeof(buf),"REC %.1fM", lb/1048576.0f);
      txt(buf,108,104,lb?C_GOOD:C_DIM,1,12); }

    snprintf(buf,sizeof(buf),"%d sats",g.numSV);
    txt(buf,2,120,satColor(g.numSV),2,7);
    if (g.hAccM<0.1f) snprintf(buf,sizeof(buf),"%.0fmm%s",g.hAccM*1000.0f,g.epeValid?" EPE":" est");
    else               snprintf(buf,sizeof(buf),"%.2fm%s", g.hAccM,        g.epeValid?" EPE":" est");
    txt(buf,88,120,accuracyColor(g.hAccM),2,10);

    { uint16_t c=accuracyColor(g.hAccM);
      snprintf(buf,sizeof(buf),"%.9f",g.latitude);  txt(buf,38,164,c,2,13);
      snprintf(buf,sizeof(buf),"%.9f",g.longitude); txt(buf,38,144,c,2,13); }

    // CPU (ESP32-S3 silicon die) temperature. Speed + heading used to live on this
    // row but were removed — the LCD is only read while stopped, so they carried no
    // value. Color-coded on the same stoplight thresholds as the web dashboard:
    // <85°C good (green), 85–100°C warm (amber), ≥100°C hot (red) — ~25°C of margin
    // below the S3's 125°C junction max. Whole row is wiped first so the old
    // speed/heading glyphs can't ghost through.
    tft.fillRect(0,184,172,18,C_BG);
    txt("CPU",2,184,C_DIM,2);
    if (isnan(s.espTempC)) {
        txtr("-- \xB0""F",170,184,C_DIM);
    } else {
        uint16_t tc = (s.espTempC>=100.0f) ? C_BAD : (s.espTempC>=85.0f) ? C_WARN : C_GOOD;
        snprintf(buf,sizeof(buf),"%.0f\xB0""F",cToF(s.espTempC));
        txtr(buf,170,184,tc);
    }

    if (s.bleConnected) snprintf(buf,sizeof(buf),"BLE  %.0fHz",s.blePacketHz);
    else                snprintf(buf,sizeof(buf),"BLE WAITING");
    txt(buf,2,208,s.bleConnected?C_GOOD:C_WARN,2,16);

    // GPS Hz + CAN Hz
    snprintf(buf,sizeof(buf),"GPS %.0fHz",s.gnssHz);    txt(buf, 2,228,C_DIM,2,9);
    if (s.canBusOk) {
        snprintf(buf,sizeof(buf),"CAN %.0f/s",s.canHz); txt(buf,90,228,C_GOOD,2,9);
    } else {
        txt("CAN OFF",90,228,C_DIM,2);
    }

    updateGForceCircle(m.ay,m.ax);
    gcSprite.setTextColor(C_TEXT,C_BG);
    snprintf(buf,sizeof(buf),"%+.2fg",m.ay); gcSprite.drawString(buf,2, 4,2);
    snprintf(buf,sizeof(buf),"%+.2fg",m.ax); gcSprite.drawString(buf,2,24,2);
    snprintf(buf,sizeof(buf),"%+.2fg",m.az); gcSprite.drawString(buf,2,44,2);
    gcSprite.setTextColor(0x0B19,C_BG);
    snprintf(buf,sizeof(buf),"%+.0fd/s",m.gy); gcSprite.drawRightString(buf,170, 4,2);
    snprintf(buf,sizeof(buf),"%+.0fd/s",m.gx); gcSprite.drawRightString(buf,170,24,2);
    snprintf(buf,sizeof(buf),"%+.0fd/s",m.gz); gcSprite.drawRightString(buf,170,44,2);
    gcSprite.pushSprite(0,GC_Y0);
}
