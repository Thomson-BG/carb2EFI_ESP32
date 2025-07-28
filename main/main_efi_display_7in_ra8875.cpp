/**
 * ESP32 EFI 7" TFT Dashboard (RA8875 Controller)
 *
 * - Displays: RPM, MAP, Injector Pulse Width, AFR/O2
 * - Warnings for lean/rich, high RPM, sensor error
 * - Demo for 7" 800x480 TFT with RA8875 controller (SPI)
 * - Framework: Arduino-ESP32 (platformio.ini: board = esp32dev)
 *
 * Wiring (example, VSPI):
 *   MOSI: GPIO23, MISO: GPIO19, SCK: GPIO18
 *   CS:   GPIO5, RST: GPIO4
 *   (Touch not implemented in this sample)
 */

#include <Adafruit_GFX.h>
#include <Adafruit_RA8875.h>

// --- Display Pinout ---
#define RA8875_CS   5
#define RA8875_RST  4

Adafruit_RA8875 tft(RA8875_CS, RA8875_RST);

#define TFT_WIDTH   800
#define TFT_HEIGHT  480

// --- Demo EFI Data Inputs (replace with your EFI variables) ---
extern volatile float engine_rpm;
extern float map_kpa;
extern float pw_ms;
extern float afr; // Or lambda

// --- Demo values if not linked to EFI controller
float demo_rpm = 850.0;
float demo_map = 35.0;
float demo_pw  = 3.1;
float demo_afr = 14.7;

// --- Color Palette ---
#define COLOR_BG      RA8875_BLACK
#define COLOR_LABEL   RA8875_WHITE
#define COLOR_DATA    RA8875_CYAN
#define COLOR_WARN    RA8875_YELLOW
#define COLOR_ALERT   RA8875_RED
#define COLOR_OK      RA8875_GREEN

// --- Thresholds ---
#define RPM_MAX       6000
#define RPM_WARN      5000
#define AFR_LEAN      16.0
#define AFR_RICH      12.0
#define MAP_MAX       110

// --- Utility ---
void drawGauge(int x, int y, const char* label, float value, const char* units, uint16_t color, int size = 3) {
    tft.textEnlarge(size);
    tft.textColor(color, COLOR_BG);
    tft.textSetCursor(x, y);
    tft.print(label);
    tft.textEnlarge(size+1);
    tft.textSetCursor(x, y + 36);
    tft.print(value, 1);
    tft.print(" ");
    tft.print(units);
    tft.textEnlarge(1);
}

// --- Main Display Update ---
void drawDashboard(float rpm, float map, float pw, float afr) {
    tft.fillScreen(COLOR_BG);

    // RPM Gauge
    uint16_t rpm_color = (rpm > RPM_WARN) ? COLOR_WARN : COLOR_OK;
    drawGauge(30, 30, "RPM", rpm, "", rpm_color, 3);

    // MAP Gauge
    drawGauge(30, 180, "MAP", map, "kPa", COLOR_DATA, 2);

    // PW Gauge
    drawGauge(30, 300, "INJ PW", pw, "ms", COLOR_DATA, 2);

    // AFR
    uint16_t afr_color = (afr >= AFR_LEAN) ? COLOR_ALERT : (afr <= AFR_RICH ? COLOR_ALERT : COLOR_OK);
    drawGauge(400, 30, "AFR", afr, "", afr_color, 3);

    // Warnings
    int ywarn = 250;
    tft.textEnlarge(2);
    tft.textColor(COLOR_ALERT, COLOR_BG);
    if (afr >= AFR_LEAN) {
        tft.textSetCursor(400, ywarn);
        tft.print("WARNING: LEAN!");
        ywarn += 40;
    } else if (afr <= AFR_RICH) {
        tft.textSetCursor(400, ywarn);
        tft.print("WARNING: RICH!");
        ywarn += 40;
    }
    if (rpm > RPM_WARN) {
        tft.textSetCursor(400, ywarn);
        tft.print("WARNING: HIGH RPM!");
        ywarn += 40;
    }
    if (map > MAP_MAX) {
        tft.textSetCursor(400, ywarn);
        tft.print("SENSOR ERROR: MAP");
        ywarn += 40;
    }

    // Demo: Connection Status (always OK here)
    tft.textColor(COLOR_OK, COLOR_BG);
    tft.textEnlarge(1);
    tft.textSetCursor(30, TFT_HEIGHT - 40);
    tft.print("ESP32 EFI DASH - v1.0  |  Status: OK");
}

// --- Arduino Setup ---
void setup() {
    Serial.begin(115200);
    delay(200);
    if (!tft.begin(RA8875_800x480)) {
        Serial.println("RA8875 Not found!");
        while (1) delay(10);
    }
    tft.displayOn(true);
    tft.GPIOX(true);      // Enable TFT - display enable pin
    tft.PWM1config(true, 255); // Backlight on full
    tft.fillScreen(COLOR_BG);
    tft.textMode();
    tft.textTransparent(RA8875_WHITE);
    tft.textEnlarge(1);
}

// --- Arduino Loop ---
void loop() {
    // --- Get live data here or use demo values ---
    float rpm = engine_rpm ? engine_rpm : demo_rpm;
    float map = map_kpa ? map_kpa : demo_map;
    float pw  = pw_ms ? pw_ms : demo_pw;
    float afrval = afr ? afr : demo_afr;

    // For demo, animate:
    static int tick = 0;
    tick++;
    demo_rpm = 650 + 2000 * fabs(sin(tick * 0.01));
    demo_map = 30 + 65 * fabs(sin(tick * 0.008));
    demo_pw  = 2.5 + 3.0 * fabs(sin(tick * 0.012));
    demo_afr = 13.5 + 2.0 * sin(tick * 0.007);

    drawDashboard(rpm, map, pw, afrval);
    delay(250);
}
