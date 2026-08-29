/*
 * imu.cpp — QMI8658 IMU driver + non-blocking bias calibration
 * =============================================================
 * Direct I2C register access — no external library required.
 */

#include "imu.h"
#include "config.h"
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// ── QMI8658 register map ─────────────────────────────────────────────────────
#define QMI8658_WHO_AM_I  0x00
#define QMI8658_CTRL1     0x02
#define QMI8658_CTRL2     0x03   // accel config
#define QMI8658_CTRL3     0x04   // gyro config
#define QMI8658_CTRL7     0x08   // enable sensors
#define QMI8658_TEMP_L    0x33   // temperature low byte  (high byte at 0x34)
#define QMI8658_AX_L      0x35   // 6 bytes accel, 6 bytes gyro

// Sensitivity for configured ranges
static const float ACCEL_SCALE = 8.0f    / 32768.0f;  // g/LSB   (±8g)
static const float GYRO_SCALE  = 512.0f  / 32768.0f;  // dps/LSB (±512dps)

// ── Calibration state ─────────────────────────────────────────────────────────
// Stored bias subtracted from every reading. Default 0 = no correction.
static float accelBias[3] = {0, 0, 0};
static float gyroBias[3]  = {0, 0, 0};
static bool  calLoaded    = false;     // a valid bias is in effect

// Non-blocking capture machine
static volatile ImuCalState calState = IMU_CAL_IDLE;
static const int   CAL_TARGET = 100;   // samples (~2 s at 50 Hz read rate)
static int         calCount   = 0;
static double      calSum[6]   = {0,0,0,0,0,0};   // ax,ay,az,gx,gy,gz (raw, bias-free)
static double      calSumSq[6] = {0,0,0,0,0,0};   // for motion (variance) check

// Motion-rejection thresholds (std-dev over the capture window)
static const float CAL_MAX_GYRO_STD  = 2.0f;    // dps — still hand-held is < this
static const float CAL_MAX_ACCEL_STD = 0.08f;   // g

static const char* NVS_NS = "rcx_imu";

// ── Low-level I2C helpers ─────────────────────────────────────────────────────
static void reg_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t reg_read(uint8_t reg) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)1);
    return Wire.read();
}

// ── NVS persistence ───────────────────────────────────────────────────────────
static void loadCalibration() {
    Preferences p;
    p.begin(NVS_NS, true);                 // read-only
    bool valid = p.getBool("valid", false);
    if (valid) {
        accelBias[0] = p.getFloat("abx", 0);
        accelBias[1] = p.getFloat("aby", 0);
        accelBias[2] = p.getFloat("abz", 0);
        gyroBias[0]  = p.getFloat("gbx", 0);
        gyroBias[1]  = p.getFloat("gby", 0);
        gyroBias[2]  = p.getFloat("gbz", 0);
        calLoaded = true;
    }
    p.end();
    if (calLoaded) {
        Serial.printf("✅ IMU calibration loaded: aBias=[%.3f %.3f %.3f]g gBias=[%.2f %.2f %.2f]dps\n",
                      accelBias[0], accelBias[1], accelBias[2],
                      gyroBias[0],  gyroBias[1],  gyroBias[2]);
    } else {
        Serial.println("ℹ️  IMU not calibrated — send \"imucal\" while still & flat to calibrate.");
    }
}

static void saveCalibration() {
    Preferences p;
    p.begin(NVS_NS, false);                // read-write
    p.putBool("valid", true);
    p.putFloat("abx", accelBias[0]);
    p.putFloat("aby", accelBias[1]);
    p.putFloat("abz", accelBias[2]);
    p.putFloat("gbx", gyroBias[0]);
    p.putFloat("gby", gyroBias[1]);
    p.putFloat("gbz", gyroBias[2]);
    p.end();
}

// ── Public API ────────────────────────────────────────────────────────────────
bool imu_init() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);   // 400 kHz — cuts the 12-byte read from ~1.3ms to ~0.35ms
    delay(50);

    uint8_t whoAmI = reg_read(QMI8658_WHO_AM_I);
    Serial.printf("QMI8658 WHO_AM_I = 0x%02X\n", whoAmI);

    if (whoAmI == 0xFF) {
        Serial.printf("❌ QMI8658 not responding at 0x%02X. Check pins.\n", IMU_ADDR);
        return false;
    }

    reg_write(QMI8658_CTRL1, 0x60);  // auto-increment address enabled
    reg_write(QMI8658_CTRL2, 0xAC);  // Accel: 1024 Hz ODR, ±8g, LPF on
    // Gyro CTRL3: bits[6:4]=FSR, bits[3:0]=ODR.  *** SCALE BUG FIX ***
    // This was 0xAA, whose FSR bits [6:4]=010 select ±64 dps — but GYRO_SCALE below assumes
    // ±512 dps, so every gyro reading came out 8× too large (512/64). Confirmed against the
    // commute log: integrated yaw was a constant 7.95× the GPS heading change on every turn.
    // ±64 dps also clips an autocross car (slaloms/spins exceed 64 dps), so the right fix is
    // to program the ±512 the code already expects, not to down-scale to ±64.
    //   ±512 dps = bits[6:4]=101, 1024 Hz ODR = bits[3:0]=1010  →  0xDA
    reg_write(QMI8658_CTRL3, 0xDA);  // Gyro:  1024 Hz ODR, ±512 dps (was 0xAA = ±64, mislabeled)
    reg_write(QMI8658_CTRL7, 0x03);  // Enable accel + gyro

    delay(50);
    Serial.println("✅ QMI8658 IMU initialized (±8g, ±512dps, 1kHz)");
    // gz sign note: raw gz follows the right-hand rule (Z-up ⇒ positive = counterclockwise /
    // left turn). GPS compass heading is the opposite sense (positive = clockwise / right
    // turn). That is why integrated raw gz is the NEGATIVE of GPS heading change — expected,
    // not a fault. Any dead-reckoning consumer must negate gz to get compass-sense heading;
    // the driver leaves the raw sign untouched so other consumers see true sensor axes.

    // ── One-shot temperature diagnostic ───────────────────────────────────────
    // The loop read returns 0x7FFF (invalid rail). This dumps chip ID/config and
    // reads the temp register HERE — at boot, before the 50 Hz accel/gyro burst
    // starts — both as single-byte reads and as the same 2-byte burst the loop
    // uses. If temp is VALID here but 0x7FFF in the loop → the burst is the cause
    // (fix: read temp inside the sensor burst). If it's 0x7FFF here too → it's a
    // config/enable difference vs the caster. Either way the boot log decides.
#if defined(IMU_TEMP_DEBUG) && IMU_TEMP_DEBUG
    {
        Serial.printf("🌡  QMI id: WHO_AM_I=0x%02X REV=0x%02X CTRL1=0x%02X CTRL2=0x%02X CTRL3=0x%02X CTRL7=0x%02X\n",
                      reg_read(QMI8658_WHO_AM_I), reg_read(0x01),
                      reg_read(QMI8658_CTRL1), reg_read(QMI8658_CTRL2),
                      reg_read(QMI8658_CTRL3), reg_read(QMI8658_CTRL7));
        Serial.printf("🌡  QMI status: STATUSINT(0x2E)=0x%02X STATUS0(0x2F)=0x%02X\n",
                      reg_read(0x2E), reg_read(0x2F));
        for (int k = 0; k < 3; k++) {
            delay(60);
            // (a) two single-byte reads
            uint8_t sl = reg_read(QMI8658_TEMP_L);
            uint8_t sh = reg_read(0x34);
            // (b) the SAME 2-byte burst imu_readTempC() uses
            uint8_t bl = 0xEE, bh = 0xEE;
            Wire.beginTransmission(IMU_ADDR);
            Wire.write(QMI8658_TEMP_L);
            Wire.endTransmission(false);
            if (Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)2) >= 2) { bl = Wire.read(); bh = Wire.read(); }
            int16_t sw = (int16_t)((uint16_t)sl | ((uint16_t)sh << 8));
            int16_t bw = (int16_t)((uint16_t)bl | ((uint16_t)bh << 8));
            Serial.printf("🌡  boot temp[%d]: single L=0x%02X H=0x%02X (%.2f°C) | burst L=0x%02X H=0x%02X (%.2f°C)\n",
                          k, sl, sh, sw / 256.0f, bl, bh, bw / 256.0f);
        }
    }
#endif

    loadCalibration();
    return true;
}

// Finalise a calibration once CAL_TARGET raw samples are gathered.
static void finishCalibration() {
    float mean[6], var[6];
    for (int i = 0; i < 6; i++) {
        mean[i] = (float)(calSum[i] / calCount);
        // population variance = E[x^2] - E[x]^2, clamped to >=0
        float v = (float)(calSumSq[i] / calCount) - mean[i] * mean[i];
        var[i] = v > 0 ? v : 0;
    }

    // Motion rejection — std-dev over the window must be small on every axis.
    for (int i = 0; i < 3; i++) {
        if (sqrtf(var[i])     > CAL_MAX_ACCEL_STD) { calState = IMU_CAL_FAILED; break; }
        if (sqrtf(var[i + 3]) > CAL_MAX_GYRO_STD)  { calState = IMU_CAL_FAILED; break; }
    }
    if (calState == IMU_CAL_FAILED) {
        Serial.println("❌ IMU calibration FAILED — motion detected. Keep it still & flat, retry.");
        return;
    }

    // Gyro: expected 0 on every axis.
    gyroBias[0] = mean[3];
    gyroBias[1] = mean[4];
    gyroBias[2] = mean[5];

    // Accel: assign ±1 g to the dominant axis, 0 to the others, then bias = mean - expected.
    int   domAxis = 0;
    float domMag  = fabsf(mean[0]);
    for (int i = 1; i < 3; i++) {
        if (fabsf(mean[i]) > domMag) { domMag = fabsf(mean[i]); domAxis = i; }
    }
    for (int i = 0; i < 3; i++) {
        float expected = (i == domAxis) ? copysignf(1.0f, mean[i]) : 0.0f;
        accelBias[i] = mean[i] - expected;
    }

    calLoaded = true;
    calState  = IMU_CAL_DONE;
    saveCalibration();
    Serial.printf("✅ IMU calibrated: aBias=[%.3f %.3f %.3f]g gBias=[%.2f %.2f %.2f]dps (gravity axis=%c)\n",
                  accelBias[0], accelBias[1], accelBias[2],
                  gyroBias[0],  gyroBias[1],  gyroBias[2],
                  "XYZ"[domAxis]);
}

void imu_read(ImuData& out) {
    out = {};   // always start clean — if I2C fails below, caller gets 0s not stack garbage

    Wire.beginTransmission(IMU_ADDR);
    Wire.write(QMI8658_AX_L);
    Wire.endTransmission(false);

    if (Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)12) < 12) return;

    int16_t raw[6];
    for (int i = 0; i < 6; i++) {
        uint8_t lo = Wire.read();
        uint8_t hi = Wire.read();
        raw[i] = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
    }

    // Raw, bias-free physical values.
    float a[3] = { raw[0] * ACCEL_SCALE, raw[1] * ACCEL_SCALE, raw[2] * ACCEL_SCALE };
    float g[3] = { raw[3] * GYRO_SCALE,  raw[4] * GYRO_SCALE,  raw[5] * GYRO_SCALE  };

    // Feed an in-progress calibration with the RAW (bias-free) values.
    if (calState == IMU_CAL_RUNNING) {
        const float s[6] = { a[0], a[1], a[2], g[0], g[1], g[2] };
        for (int i = 0; i < 6; i++) { calSum[i] += s[i]; calSumSq[i] += (double)s[i] * s[i]; }
        if (++calCount >= CAL_TARGET) finishCalibration();
    }

    // Apply stored bias (zero until calibrated).
    out.ax = a[0] - accelBias[0];
    out.ay = a[1] - accelBias[1];
    out.az = a[2] - accelBias[2];
    out.gx = g[0] - gyroBias[0];
    out.gy = g[1] - gyroBias[1];
    out.gz = g[2] - gyroBias[2];
}

// ── Board heat-soak temperature (QMI8658 internal sensor) ─────────────────────
// The QMI8658 exposes a 16-bit signed die temperature at 0x33 (L) / 0x34 (H),
// scaled 1/256 °C/LSB. The IMU barely self-heats, so on this board it tracks the
// enclosure/board heat-soak rather than its own dissipation — a far better proxy
// for the physical limits (PETG Tg, LCD bond) than the ESP32-S3 silicon die.
//
// MUST be called from the SAME context as imu_read() (loop(), Core 1): both share
// the single Wire bus, and there is no second lock guarding it. Call it at a low
// rate (0.5 Hz is plenty for thermal) — it is one extra 2-byte I2C burst.
//
// Returns °C on success, or NAN on an I2C failure or an out-of-range reading
// (sanity-gated to -40..150 °C) so a glitch shows as "no data" rather than a
// fake temperature. NAN propagates cleanly through the log/web/display layers.
float imu_readTempC() {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(QMI8658_TEMP_L);
    Wire.endTransmission(false);

    if (Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)2) < 2) return NAN;

    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    int16_t raw = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
    float c = raw / 256.0f;

    // ── DIAGNOSTIC (bench only) ───────────────────────────────────────────────
    // 262.4 °F (=128 °C) on the dashboard = raw pinned at ~0x7FFF, the signed-16
    // positive rail = an INVALID register read, not a real temp. Print the raw
    // bytes so a USB-connected bench session shows exactly what the register
    // returns. Serial TX timeout is 0 (see setup), so this drops silently in the
    // field and never stalls the loop. Remove once the read is trusted.
#if defined(IMU_TEMP_DEBUG) && IMU_TEMP_DEBUG
    Serial.printf("🌡  QMI temp raw: L=0x%02X H=0x%02X  word=0x%04X (%d)  → %.2f °C\n",
                  lo, hi, (uint16_t)raw, raw, c);
#endif

    // Reject the rail and anything non-physical. The QMI die in this enclosure
    // cannot really sit at 128 °C; a value at/over ~125 °C (or the exact 0x7FFF /
    // 0x8000 rails) means the register isn't giving valid data → report "no data"
    // (NAN) rather than a fake reading. Lower bound guards a stuck-cold rail too.
    if (raw == 0x7FFF || raw == (int16_t)0x8000) return NAN;
    if (c > -20.0f && c < 125.0f) return c;
    return NAN;
}

// ── Calibration control ───────────────────────────────────────────────────────
void imu_startCalibration() {
    for (int i = 0; i < 6; i++) { calSum[i] = 0; calSumSq[i] = 0; }
    calCount = 0;
    calState = IMU_CAL_RUNNING;
    Serial.println("⏳ IMU calibration started — hold STILL and FLAT for ~2 s...");
}

ImuCalState imu_calibrationState() { return calState; }
bool        imu_isCalibrated()     { return calLoaded; }

void imu_clearCalibration() {
    for (int i = 0; i < 3; i++) { accelBias[i] = 0; gyroBias[i] = 0; }
    calLoaded = false;
    calState  = IMU_CAL_IDLE;
    Preferences p;
    p.begin(NVS_NS, false);
    p.clear();
    p.end();
    Serial.println("🧹 IMU calibration cleared — using raw values.");
}

// Web-safe triggers (imu.h): imu_startCalibration()/imu_clearCalibration() touch
// calState/calSum/calCount/calSumSq, which imu_read() also writes every 50 Hz
// sample from loop()/Core 1 with no lock — same reason imu_readTempC() above is
// documented same-context-only. These two just set a flag; imu_serviceCalRequests()
// (called from the same place as imu_pollSerial(), so the same context as
// imu_read()) does the real work. Same deferred-request pattern sd_log.cpp
// already uses for web-triggered work (dirScanReq, exportReq).
static volatile bool calStartReq = false;
static volatile bool calClearReq = false;
void imu_requestCalibration()      { calStartReq = true; }
void imu_requestClearCalibration() { calClearReq = true; }
void imu_serviceCalRequests() {
    if (calStartReq) { calStartReq = false; imu_startCalibration(); }
    if (calClearReq) { calClearReq = false; imu_clearCalibration(); }
}

void imu_pollSerial() {
    // Minimal line reader: trigger on "imucal", clear on "imucalclear".
    static char line[16];
    static uint8_t n = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            line[n] = '\0';
            if      (!strcmp(line, "imucal"))      imu_startCalibration();
            else if (!strcmp(line, "imucalclear")) imu_clearCalibration();
            n = 0;
        } else if (n < sizeof(line) - 1) {
            line[n++] = c;
        } else {
            n = 0;   // overflow — reset
        }
    }
}
