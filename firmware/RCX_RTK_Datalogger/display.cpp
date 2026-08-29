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
#include <TFT_eSPI.h>

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

#define C_BG    TFT_BLACK
#define C_GOOD  TFT_GREEN
#define C_WARN  TFT_YELLOW
#define C_BAD   TFT_RED
#define C_TEXT  TFT_WHITE
#define C_DIM   0x528A

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

static uint16_t accuracyColor(float m) {
    if (m>=1.5f) return tft.color565(255,0,0);
    if (m>=0.8f) return lerpColor((m-0.8f)/0.70f,255,220,0,255,0,0);
    if (m>=0.4f) return lerpColor((m-0.4f)/0.40f,0,64,255,255,220,0);
    if (m>=0.05f)return lerpColor((m-0.05f)/0.35f,0,200,0,0,64,255);
    return               lerpColor(m/0.05f,       0,255,128,0,200,0);
}
static uint16_t satColor(int n) {
    if (n>=24) return lerpColor((n-24)/8.0f,0,200,0,0,255,128);
    if (n>=16) return lerpColor((n-16)/8.0f,0,100,255,0,200,0);
    if (n>=10) return lerpColor((n-10)/6.0f,255,220,0,0,100,255);
    if (n>= 6) return lerpColor((n- 6)/4.0f,255,0,0,255,220,0);
    return tft.color565(255,0,0);
}
static uint16_t healthColor(const GnssData& g, const SystemStatus& s) {
    if (!s.wifiConnected||!s.bleConnected||!g.valid) return C_BAD;
    if (!s.ntripConnected||g.rtkType==0||g.hAccM>1.5f) return C_BAD;
    if (g.rtkType==1||g.hAccM>0.30f) return C_WARN;
    return C_GOOD;
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
    gcSprite.fillSprite(TFT_BLACK);
    gcSprite.drawCircle(GC_CX,cy,GC_R,TFT_DARKGREY);
    gcSprite.drawCircle(GC_CX,cy,GC_SCALE,0x2945);
    gcSprite.drawCircle(GC_CX,cy,GC_SCALE/2,0x2104);
    gcSprite.drawFastHLine(GC_CX-GC_R,cy,GC_R*2,0x2945);
    gcSprite.drawFastVLine(GC_CX,cy-GC_R,GC_R*2,0x2945);
    gcSprite.setTextColor(0x528A);
    gcSprite.drawString("1g",GC_CX+GC_SCALE+2,cy-4,1);
}

static void updateGForceCircle(float ax, float ay) {
    drawGForceBg();
    for (int i=0;i<GC_TRAIL;i++) {
        int idx=(gfHead+i)%GC_TRAIL;
        if (!gfInit && i>gfHead) continue;
        uint8_t age=GC_TRAIL-((gfHead-idx+GC_TRAIL)%GC_TRAIL);
        if (age<2) continue;
        gcSprite.drawPixel(gfTrail[idx].x,gfTrail[idx].y-GC_Y0,gcSprite.color565(0,age*6,0));
    }
    int16_t px=GC_CX+(int16_t)(ay*GC_SCALE);
    int16_t py=GC_CY-(int16_t)(ax*GC_SCALE);
    float dx=px-GC_CX,dy=py-GC_CY,dist=sqrt(dx*dx+dy*dy);
    if (dist>GC_R) { px=GC_CX+dx/dist*GC_R; py=GC_CY+dy/dist*GC_R; }
    gcSprite.fillCircle(px,py-GC_Y0,3,gcSprite.color565(0,255,80));
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
}

static unsigned long lastDisplayUpdate = 0;

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

void display_update(const GnssData& g, const ImuData& m, const SystemStatus& s) {
    if (applyLcdPower()) return;   // panel off: nothing further to draw
    if (millis()-lastDisplayUpdate < 200) return;
    lastDisplayUpdate=millis();
    char buf[40];

    tft.fillRect(0,0,172,18,healthColor(g,s));
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
        static int  lastWifiMode = -1;              // -1: nothing drawn yet
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
        txt(buf,2,80,s.ntripCarrier==2?C_GOOD:C_WARN,2,12);
    } else { txt("---",2,60,C_WARN,2,15); txt("",2,80,C_DIM,2,12); }

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
    // <85°C good (green), 85–100°C warm (yellow), ≥100°C hot (red) — ~25°C of margin
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
    gcSprite.setTextColor(0xAD75,TFT_BLACK);
    snprintf(buf,sizeof(buf),"%+.2fg",m.ay); gcSprite.drawString(buf,2, 4,2);
    snprintf(buf,sizeof(buf),"%+.2fg",m.ax); gcSprite.drawString(buf,2,24,2);
    snprintf(buf,sizeof(buf),"%+.2fg",m.az); gcSprite.drawString(buf,2,44,2);
    gcSprite.setTextColor(0xFD20,TFT_BLACK);
    snprintf(buf,sizeof(buf),"%+.0fd/s",m.gy); gcSprite.drawRightString(buf,170, 4,2);
    snprintf(buf,sizeof(buf),"%+.0fd/s",m.gx); gcSprite.drawRightString(buf,170,24,2);
    snprintf(buf,sizeof(buf),"%+.0fd/s",m.gz); gcSprite.drawRightString(buf,170,44,2);
    gcSprite.pushSprite(0,GC_Y0);
}
