#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <Wire.h>

// ============================================================
//  LGFX — Waveshare ESP32-S3-Touch-LCD-1.28
// ============================================================
class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_GC9A01 _panel;
    lgfx::Bus_SPI      _bus;
    lgfx::Light_PWM    _light;

public:
    LGFX() {
        {
            auto cfg = _bus.config();
            #if defined(SPI2_HOST)
                cfg.spi_host = SPI2_HOST;
            #else
                cfg.spi_host = SPI3_HOST;
            #endif
            cfg.spi_mode    = 0;
            cfg.freq_write  = 20000000;
            cfg.freq_read   = 0;
            cfg.spi_3wire   = false;
            cfg.use_lock    = true;
            cfg.dma_channel = 1;
            cfg.pin_sclk    = 10;
            cfg.pin_mosi    = 11;
            cfg.pin_miso    = 12;
            cfg.pin_dc      = 8;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {
            auto cfg = _panel.config();
            cfg.panel_width  = 240;
            cfg.panel_height = 240;
            cfg.offset_x     = 0;
            cfg.offset_y     = 0;
            cfg.pin_cs       = 9;
            cfg.pin_rst      = 14;
            cfg.readable     = false;
            cfg.rgb_order    = false;
            cfg.invert       = true;
            cfg.dlen_16bit   = false;
            cfg.bus_shared   = true;
            _panel.config(cfg);
        }
        {
            auto cfg = _light.config();
            cfg.pin_bl      = 2;
            cfg.invert      = false;
            cfg.freq        = 44100;
            cfg.pwm_channel = 7;
            _light.config(cfg);
            _panel.setLight(&_light);
        }
        setPanel(&_panel);
    }
};

LGFX lcd;
lgfx::LGFX_Sprite spr(&lcd);

// ============================================================
//  TOUCH — interrupt-driven
// ============================================================
#define CST816S_ADDR  0x15
#define TOUCH_SDA     6
#define TOUCH_SCL     7
#define TOUCH_INT     5
#define TOUCH_RST     13

#define GEST_NONE       0x00
#define GEST_SINGLE_TAP 0x05
#define GEST_DOUBLE_TAP 0x0B

volatile bool touchFlag = false;

void IRAM_ATTR onTouchInt() {
    touchFlag = true;
}

void touch_reset() {
    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW);
    delay(5);
    digitalWrite(TOUCH_RST, HIGH);
    delay(50);
}

bool touch_init() {
    Wire.begin(TOUCH_SDA, TOUCH_SCL, 400000);
    pinMode(TOUCH_INT, INPUT_PULLUP);
    touch_reset();
    attachInterrupt(digitalPinToInterrupt(TOUCH_INT), onTouchInt, FALLING);
    Wire.beginTransmission(CST816S_ADDR);
    return (Wire.endTransmission() == 0);
}

uint8_t touch_get_gesture() {
    if (!touchFlag) return GEST_NONE;
    touchFlag = false;
    delay(10);
    Wire.beginTransmission(CST816S_ADDR);
    Wire.write(0x01);
    if (Wire.endTransmission(false) != 0) return GEST_NONE;
    if (Wire.requestFrom(CST816S_ADDR, (uint8_t)1) != 1) return GEST_NONE;
    return Wire.read();
}

// ============================================================
//  DETECTION
// ============================================================
#define RX_PIN            15
#define SIGNAL_THRESHOLD  120
#define BURST_MIN_US      500
#define BURST_MAX_US      5000
#define LISTEN_WINDOW_MS  3000
#define RESULT_HOLD_MS    4000

// ============================================================
//  COLOURS
// ============================================================
#define C_BG        0x0861
#define C_GREEN     0x07E0
#define C_GREEN_DIM 0x03E0
#define C_GREEN_BG  0x0420
#define C_RED       0xF800
#define C_RED_DIM   0x7800
#define C_RED_BG    0x2000
#define C_AMBER     0xFD20
#define C_WHITE     0xFFFF
#define C_DIM       0x4A69
#define C_BLUE      0x051F
#define C_ARC_BG    0x2104

// ============================================================
//  STATE
// ============================================================
enum AppState { STATE_IDLE, STATE_TESTING, STATE_PASS, STATE_FAIL };

AppState      appState       = STATE_IDLE;
int           baseline       = 2048;
int           signalPct      = 0;
float         estFreqKhz     = 0.0f;
unsigned long stateEnteredAt = 0;
unsigned long listenStart    = 0;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void drawIdle();
void drawTesting(int pct);
void drawPass(int pct, float freqKhz);
void drawFail();
void drawArc(int cx, int cy, int r, int thickness,
             float startDeg, float endDeg,
             uint16_t color, float fillFraction);
void calibrateBaseline();
bool runDetection(int &outPct, float &outFreq);

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);

    lcd.init();
    lcd.setColorDepth(16);
    lcd.setSwapBytes(true);
    lcd.setBrightness(255);
    lcd.setRotation(0);

    spr.createSprite(240, 240);
    spr.setColorDepth(16);

    if (touch_init()) {
        Serial.println("Touch init OK");
    } else {
        Serial.println("Touch init FAILED");
    }

    calibrateBaseline();
    drawIdle();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
    uint8_t g = touch_get_gesture();

   if (g != GEST_NONE) {
    if (appState == STATE_IDLE || appState == STATE_PASS || appState == STATE_FAIL) {
        if (g == GEST_SINGLE_TAP) {
            appState       = STATE_TESTING;
            stateEnteredAt = millis();
            listenStart    = millis();
            drawTesting(0);
        } else if (g == GEST_DOUBLE_TAP) {
            calibrateBaseline();
            appState = STATE_IDLE;
            drawIdle();
        }
    }
    // no delay here
}

    switch (appState) {
        case STATE_IDLE:
            delay(20);
            break;

        case STATE_TESTING: {
            int   pct  = 0;
            float freq = 0.0f;
            bool  hit  = runDetection(pct, freq);

            if (hit) {
                signalPct  = pct;
                estFreqKhz = freq;
                appState   = STATE_PASS;
                stateEnteredAt = millis();
                drawPass(signalPct, estFreqKhz);
            } else {
                int rawDev = abs(analogRead(RX_PIN) - baseline);
                int livePct = constrain(map(rawDev, 0, 400, 0, 100), 0, 100);
                drawTesting(livePct);
                if (millis() - listenStart > LISTEN_WINDOW_MS) {
                    appState       = STATE_FAIL;
                    stateEnteredAt = millis();
                    drawFail();
                }
            }
            break;
        }

        case STATE_PASS:
        case STATE_FAIL:
            if (millis() - stateEnteredAt > RESULT_HOLD_MS) {
                appState = STATE_IDLE;
                drawIdle();
            }
            delay(50);
            break;
    }
}

// ============================================================
//  CALIBRATION & DETECTION
// ============================================================
void calibrateBaseline() {
    long sum = 0;
    for (int i = 0; i < 256; i++) {
        sum += analogRead(RX_PIN);
        delayMicroseconds(400);
    }
    baseline = (int)(sum / 256);
    Serial.printf("Baseline: %d\n", baseline);
}

bool runDetection(int &outPct, float &outFreq) {
    int raw = analogRead(RX_PIN);
    int dev = abs(raw - baseline);
    if (dev < SIGNAL_THRESHOLD) return false;

    unsigned long burstStart = micros();
    int peakDev   = dev;
    int crossings = 0;
    int prevRaw   = raw;

    while (true) {
        int s    = analogRead(RX_PIN);
        int sdev = abs(s - baseline);
        if ((s > baseline) != (prevRaw > baseline)) crossings++;
        prevRaw = s;
        if (sdev > peakDev) peakDev = sdev;

        unsigned long elapsed = micros() - burstStart;

        if (sdev < (SIGNAL_THRESHOLD / 2)) {
            if (elapsed >= BURST_MIN_US && elapsed <= BURST_MAX_US) {
                float dur = elapsed / 1000000.0f;
                outFreq = (crossings / 2.0f) / dur / 1000.0f;
                outPct  = (int)constrain(
                    map(peakDev, SIGNAL_THRESHOLD, 800, 10, 100), 10, 100);
                Serial.printf("BURST: %luus peak=%d ~%.1fkHz str=%d%%\n",
                    elapsed, peakDev, outFreq, outPct);
                return true;
            }
            return false;
        }
        if (elapsed > BURST_MAX_US) return false;
    }
}

// ============================================================
//  DRAWING
// ============================================================
void drawArc(int cx, int cy, int r, int thickness,
             float startDeg, float endDeg,
             uint16_t color, float fillFraction) {
    float fillEnd = startDeg + (endDeg - startDeg) * fillFraction;
    for (float deg = startDeg; deg <= endDeg; deg += 0.8f) {
        float rad  = (deg - 90.0f) * DEG_TO_RAD;
        uint16_t c = (deg <= fillEnd) ? color : C_ARC_BG;
        for (int t = 0; t < thickness; t++) {
            int rr = r - t;
            int x  = cx + (int)(rr * cosf(rad));
            int y  = cy + (int)(rr * sinf(rad));
            spr.drawPixel(x, y, c);
        }
    }
}

// ============================================================
//  IDLE
// ============================================================
void drawIdle() {
    spr.fillSprite(C_BG);

    spr.drawCircle(120, 120, 118, C_DIM);

    // Large tap icon — circle with dot
    spr.drawCircle(120, 95, 28, C_DIM);
    spr.fillCircle(120, 95, 6, C_DIM);

    // TAP TO TEST
    spr.setTextSize(3);
    spr.setTextColor(C_WHITE);
    const char* t1 = "TAP TO";
    spr.setCursor(120 - spr.textWidth(t1) / 2, 140);
    spr.print(t1);

    const char* t2 = "TEST";
    spr.setTextColor(C_AMBER);
    spr.setCursor(120 - spr.textWidth(t2) / 2, 168);
    spr.print(t2);

    // Double tap hint
    spr.setTextSize(1);
    spr.setTextColor(C_DIM);
    const char* h = "double tap to recalibrate";
    spr.setCursor(120 - spr.textWidth(h) / 2, 208);
    spr.print(h);

    spr.pushSprite(0, 0);
}

// ============================================================
//  TESTING
// ============================================================
void drawTesting(int pct) {
    spr.fillSprite(C_BG);

    drawArc(120, 120, 108, 14, 150.0f, 390.0f, C_BLUE, pct / 100.0f);
    spr.drawCircle(120, 120, 118, C_DIM);

    static bool pulse = false;
    pulse = !pulse;
    spr.fillCircle(120, 120, pulse ? 8 : 5, C_BLUE);

    spr.setTextSize(2);
    spr.setTextColor(C_DIM);
    const char* tl = "LISTENING";
    spr.setCursor(120 - spr.textWidth(tl) / 2, 100);
    spr.print(tl);

    // Countdown bar
    unsigned long elapsed = millis() - listenStart;
    int remaining = constrain((int)(LISTEN_WINDOW_MS - elapsed), 0, LISTEN_WINDOW_MS);
    int barW = (int)(160 * (remaining / (float)LISTEN_WINDOW_MS));
    spr.drawRect(40, 148, 160, 8, C_DIM);
    if (barW > 0) spr.fillRect(40, 148, barW, 8, C_BLUE);

    spr.setTextSize(1);
    spr.setTextColor(C_DIM);
    const char* h = "hold near sensor face";
    spr.setCursor(120 - spr.textWidth(h) / 2, 168);
    spr.print(h);

    spr.pushSprite(0, 0);
}

// ============================================================
//  PASS
// ============================================================
void drawPass(int pct, float freqKhz) {
    spr.fillSprite(C_BG);

    drawArc(120, 120, 108, 14, 150.0f, 390.0f, C_GREEN, 1.0f);
    spr.fillCircle(120, 120, 92, C_GREEN_BG);
    spr.drawCircle(120, 120, 118, C_GREEN_DIM);

    // PASS — very large
    spr.setTextSize(4);
    spr.setTextColor(C_GREEN);
    const char* ps = "PASS";
    spr.setCursor(120 - spr.textWidth(ps) / 2, 82);
    spr.print(ps);

    // Signal strength %
    char pBuf[8];
    snprintf(pBuf, sizeof(pBuf), "%d%%", pct);
    spr.setTextSize(4);
    spr.setTextColor(C_WHITE);
    spr.setCursor(120 - spr.textWidth(pBuf) / 2, 128);
    spr.print(pBuf);

    // Frequency
    char fBuf[20];
    if (freqKhz > 5.0f && freqKhz < 80.0f)
        snprintf(fBuf, sizeof(fBuf), "~%.1f kHz", freqKhz);
    else
        snprintf(fBuf, sizeof(fBuf), "~40 kHz");
    spr.setTextSize(1);
    spr.setTextColor(C_GREEN);
    spr.setCursor(120 - spr.textWidth(fBuf) / 2, 174);
    spr.print(fBuf);

    spr.pushSprite(0, 0);
}

// ============================================================
//  FAIL
// ============================================================
void drawFail() {
    spr.fillSprite(C_BG);

    drawArc(120, 120, 108, 14, 150.0f, 390.0f, C_RED_DIM, 1.0f);
    spr.fillCircle(120, 120, 92, C_RED_BG);
    spr.drawCircle(120, 120, 118, C_RED_DIM);

    // FAIL — very large
    spr.setTextSize(4);
    spr.setTextColor(C_RED);
    const char* fs = "FAIL";
    spr.setCursor(120 - spr.textWidth(fs) / 2, 82);
    spr.print(fs);

    // No signal
    spr.setTextSize(2);
    spr.setTextColor(C_RED_DIM);
    const char* ns = "NO SIGNAL";
    spr.setCursor(120 - spr.textWidth(ns) / 2, 130);
    spr.print(ns);

    // Tap to retry
    spr.setTextSize(1);
    spr.setTextColor(C_DIM);
    const char* r = "tap to test again";
    spr.setCursor(120 - spr.textWidth(r) / 2, 174);
    spr.print(r);

    spr.pushSprite(0, 0);
}