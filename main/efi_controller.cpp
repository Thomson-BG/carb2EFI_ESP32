/**
 * ESP32 EFI Controller with 7" TFT Dashboard (RA8875) + GPS Speedometer
 * 
 * Updated: Adds dedicated speedometer gauge and needle, plus digital speed readout.
 * 
 * Features:
 * - EFI logic: MAP sensor, O2/lambda sensor, RPM from ignition points (8cyl), single injector PWM
 * - Hardwired 7" RA8875 TFT (SPI) dashboard: analog/digital gauges, warnings, GPS speedometer
 * - GPS speedometer gauge (dedicated, needle + digital)
 * - (Expandable: Wi-Fi/Bluetooth, OTA, data logging, remote tuning)
 */

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_RA8875.h>
#include <TinyGPSPlus.h>
#include <math.h>

// --- Display Pinout ---
#define RA8875_CS   5
#define RA8875_RST  4

Adafruit_RA8875 tft(RA8875_CS, RA8875_RST);

#define TFT_WIDTH   800
#define TFT_HEIGHT  480

// --- EFI Pins ---
#define MAP_SENSOR_ADC_CH   36    // GPIO36 (VP) - analog in
#define O2_SENSOR_ADC_CH    39    // GPIO39 (VN) - analog in (optional)
#define INJECTOR_GPIO       18    // PWM output pin for injector driver
#define IGNITION_PULSE_GPIO 4     // Input from coil negative/points

// --- GPS UART Pins ---
#define GPS_RX_PIN  16
#define GPS_TX_PIN  17
#define GPS_BAUD    9600

HardwareSerial SerialGPS(2); // UART2 for GPS
TinyGPSPlus gps;

// --- Engine Config ---
#define ENGINE_DISPLACEMENT_L   5.7f
#define NUM_CYLINDERS           8
#define INJECTOR_FLOW_RATE_CC   170
#define AFR_STOICH              14.7f

// --- Injector PWM ---
#define INJECTOR_PWM_FREQ   100   // Hz (low freq for pulse control)
#define INJECTOR_PWM_CH     0     // PWM channel

// --- Dashboard Color Palette ---
#define COLOR_BG      RA8875_BLACK
#define COLOR_LABEL   RA8875_WHITE
#define COLOR_DATA    RA8875_CYAN
#define COLOR_WARN    RA8875_YELLOW
#define COLOR_ALERT   RA8875_RED
#define COLOR_OK      RA8875_GREEN
#define COLOR_NEEDLE  RA8875_RED

// --- Gauge Thresholds ---
#define RPM_MAX       6000
#define RPM_WARN      5000
#define AFR_LEAN      16.0
#define AFR_RICH      12.0
#define MAP_MAX       110
#define SPDOMETER_MAX 200   // 200 km/h for gauge face

// --- EFI State Variables ---
volatile uint32_t ign_pulse_count = 0;
volatile float engine_rpm = 0;
uint64_t last_calc_time_us = 0;

float map_kpa = 35.0;
float o2_lambda = 1.0;
float afr = AFR_STOICH;
float pw_ms = 3.1;

// --- GPS State ---
float gps_speed_kph = 0.0;
bool gps_fix = false;

// --- Demo Animation Fallback ---
float demo_rpm = 850.0;
float demo_map = 35.0;
float demo_pw  = 3.1;
float demo_afr = 14.7;
float demo_spd = 0.0;
int tick = 0;

// --- RPM Interrupt Handler (Points/Coil - Rising Edge) ---
void IRAM_ATTR ign_pulse_isr_handler() {
    ign_pulse_count++;
    uint64_t now = micros();
    static uint64_t last_time = 0;
    if (now - last_time > 200000) {
        float seconds = (now - last_time) / 1e6f;
        engine_rpm = ((float)ign_pulse_count / 4.0f) * (60.0f / seconds);
        ign_pulse_count = 0;
        last_time = now;
    }
}

// --- Sensor Functions ---
float read_map_kpa() {
    int adc = analogRead(MAP_SENSOR_ADC_CH);
    // Assume 0-4095 = 20-110kPa for typical MAP sensor
    return 20.0f + (adc / 4095.0f) * (110.0f - 20.0f);
}

float read_o2_lambda() {
    int adc = analogRead(O2_SENSOR_ADC_CH);
    float voltage = (adc / 4095.0f) * 3.3f;
    // For narrowband, rough lambda estimation
    if (voltage > 0.75f) return 0.9f;
    if (voltage < 0.2f) return 1.1f;
    return 1.0f;
}

// --- Injector Pulse Calculation (speed-density, batch fire) ---
float calculate_injector_pw_ms(float map_kpa, float afr, float rpm) {
    float ve = 0.85f; // Volumetric efficiency
    float imap = (rpm * map_kpa * ve * ENGINE_DISPLACEMENT_L) / (120.0f * 100.0f); // L/min
    float fuel_cc_per_min = imap / afr * 1000.0f;
    float pw = (fuel_cc_per_min / INJECTOR_FLOW_RATE_CC) * 60000.0f / rpm;
    if (pw < 2.0f) pw = 2.0f;
    if (pw > 20.0f) pw = 20.0f;
    return pw;
}

// --- Draw Analog Gauge Helper ---
void drawAnalogGaugeFace(int cx, int cy, int radius, int minVal, int maxVal, int step, const char* label, uint16_t color, int textsize = 2) {
    tft.drawCircle(cx, cy, radius, color);
    tft.setTextColor(color, COLOR_BG);
    tft.setTextSize(textsize);
    for (int v = minVal; v <= maxVal; v += step) {
        float angle = 3.14f * (1.0f - ((float)(v - minVal) / (maxVal - minVal)));
        int x1 = cx + (radius - 10) * cos(angle);
        int y1 = cy + (radius - 10) * sin(angle);
        int x2 = cx + radius * cos(angle);
        int y2 = cy + radius * sin(angle);
        tft.drawLine(x1, y1, x2, y2, color);
        char buf[8];
        sprintf(buf, "%d", v);
        int xt = cx + (radius - 25) * cos(angle) - 10;
        int yt = cy + (radius - 25) * sin(angle) - 8;
        tft.setCursor(xt, yt);
        tft.print(buf);
    }
    tft.setTextSize(textsize+1);
    tft.setCursor(cx - 45, cy + radius / 2);
    tft.print(label);
}

void drawGaugeNeedle(int cx, int cy, int radius, float value, int minVal, int maxVal, uint16_t color, int width = 3) {
    float angle = 3.14f * (1.0f - ((float)(value - minVal) / (maxVal - minVal)));
    int x = cx + (radius - 20) * cos(angle);
    int y = cy + (radius - 20) * sin(angle);
    tft.drawLine(cx, cy, x, y, color);
    tft.drawCircle(cx, cy, 5, color);
}

// --- Main Dashboard UI ---
void drawDashboard(float rpm, float map, float pw, float afr, float speed, bool gps_has_fix) {
    tft.fillScreen(COLOR_BG);

    // --- Speedometer Gauge (center, large) ---
    int spd_cx = 600, spd_cy = 260, spd_r = 170;
    drawAnalogGaugeFace(spd_cx, spd_cy, spd_r, 0, SPDOMETER_MAX, 20, "km/h", COLOR_DATA, 2);
    drawGaugeNeedle(spd_cx, spd_cy, spd_r, speed, 0, SPDOMETER_MAX, COLOR_NEEDLE, 5);

    // Digital speed display
    tft.setTextColor(COLOR_OK, COLOR_BG);
    tft.setTextSize(6);
    tft.setCursor(spd_cx - 65, spd_cy + 80);
    if (gps_has_fix) {
        char s[8];
        sprintf(s, "%03d", (int)(speed + 0.5));
        tft.print(s);
    } else {
        tft.print("---");
    }

    tft.setTextSize(2);
    tft.setCursor(spd_cx + 60, spd_cy + 110);
    tft.print(gps_has_fix ? "GPS" : "NO FIX");

    // --- RPM Gauge (left, mid) ---
    int rpm_cx = 170, rpm_cy = 140, rpm_r = 100;
    drawAnalogGaugeFace(rpm_cx, rpm_cy, rpm_r, 0, RPM_MAX, 1000, "RPM", COLOR_OK, 2);
    drawGaugeNeedle(rpm_cx, rpm_cy, rpm_r, rpm, 0, RPM_MAX, COLOR_NEEDLE, 2);

    // --- MAP Gauge (left, lower) ---
    int map_cx = 170, map_cy = 340, map_r = 70;
    drawAnalogGaugeFace(map_cx, map_cy, map_r, 20, 110, 30, "MAP", COLOR_DATA, 1);
    drawGaugeNeedle(map_cx, map_cy, map_r, map, 20, 110, COLOR_NEEDLE, 1);

    // --- AFR Digital Value ---
    int afr_x = 450, afr_y = 70;
    tft.setTextColor(COLOR_OK, COLOR_BG);
    tft.setTextSize(3);
    tft.setCursor(afr_x, afr_y);
    tft.print("AFR: ");
    tft.setTextColor((afr > AFR_LEAN || afr < AFR_RICH) ? COLOR_ALERT : COLOR_OK, COLOR_BG);
    tft.print(afr, 1);

    // --- Injector Pulse Width Digital Value ---
    tft.setTextColor(COLOR_DATA, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(afr_x, afr_y + 60);
    tft.print("INJ PW: ");
    tft.print(pw, 2);
    tft.print(" ms");

    // --- Warnings ---
    int ywarn = 400;
    tft.setTextSize(2);
    tft.setTextColor(COLOR_ALERT, COLOR_BG);
    if (afr >= AFR_LEAN) {
        tft.setCursor(afr_x, ywarn);
        tft.print("WARNING: LEAN!");
        ywarn += 28;
    } else if (afr <= AFR_RICH) {
        tft.setCursor(afr_x, ywarn);
        tft.print("WARNING: RICH!");
        ywarn += 28;
    }
    if (rpm > RPM_WARN) {
        tft.setCursor(afr_x, ywarn);
        tft.print("WARNING: HIGH RPM!");
        ywarn += 28;
    }
    if (map > MAP_MAX) {
        tft.setCursor(afr_x, ywarn);
        tft.print("SENSOR ERROR: MAP");
        ywarn += 28;
    }

    // --- Status ---
    tft.setTextSize(1);
    tft.setTextColor(COLOR_LABEL, COLOR_BG);
    tft.setCursor(24, TFT_HEIGHT - 28);
    tft.print("ESP32 EFI DASH | Status: OK | GPS: ");
    tft.print(gps_has_fix ? "FIX" : "NO FIX");
}

// --- Arduino Setup ---
void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    pinMode(MAP_SENSOR_ADC_CH, INPUT);
    pinMode(O2_SENSOR_ADC_CH, INPUT);
    pinMode(IGNITION_PULSE_GPIO, INPUT_PULLUP);

    // Setup injector PWM
    ledcSetup(INJECTOR_PWM_CH, INJECTOR_PWM_FREQ, 13);
    ledcAttachPin(INJECTOR_GPIO, INJECTOR_PWM_CH);

    // Setup ignition interrupt
    attachInterrupt(digitalPinToInterrupt(IGNITION_PULSE_GPIO), ign_pulse_isr_handler, RISING);

    // Setup TFT
    delay(200);
    if (!tft.begin(RA8875_800x480)) {
        Serial.println("RA8875 Display not found!");
        while (1) delay(10);
    }
    tft.displayOn(true);
    tft.GPIOX(true);
    tft.PWM1config(true, 255);
    tft.fillScreen(COLOR_BG);
    tft.textMode();
    tft.textTransparent(RA8875_WHITE);

    // Setup GPS UART
    SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

// --- Main Loop ---
void loop() {
    // --- GPS Processing ---
    while (SerialGPS.available()) {
        char c = SerialGPS.read();
        gps.encode(c);
    }
    if (gps.location.isValid() && gps.speed.isValid() && gps.hdop.isValid() && gps.hdop.value() < 200) {
        gps_speed_kph = gps.speed.kmph();
        gps_fix = true;
    } else {
        gps_speed_kph = 0.0;
        gps_fix = false;
    }

    // Read sensors (replace demo with real readings if sensors present)
    float rpm = engine_rpm > 20 ? engine_rpm : demo_rpm;
    float map = read_map_kpa();
    float lambda = read_o2_lambda();
    afr = AFR_STOICH * lambda;

    // Calculate injector pulse width
    pw_ms = calculate_injector_pw_ms(map, afr, rpm);

    // Drive injector via PWM (100Hz, 13-bit, duty = pw_ms / 10ms)
    uint32_t max_duty = (1 << 13) - 1;
    float period_ms = 1000.0f / INJECTOR_PWM_FREQ;
    uint32_t duty = (uint32_t)((pw_ms / period_ms) * max_duty);
    if (duty > max_duty) duty = max_duty;
    ledcWrite(INJECTOR_PWM_CH, duty);

    // Animate demo values if no sensor input
    tick++;
    demo_rpm = 650 + 2000 * fabs(sin(tick * 0.01));
    demo_map = 30 + 65 * fabs(sin(tick * 0.008));
    demo_pw  = 2.5 + 3.0 * fabs(sin(tick * 0.012));
    demo_afr = 13.5 + 2.0 * sin(tick * 0.007);
    demo_spd = 60 + 80 * fabs(sin(tick * 0.015));

    // Draw Dashboard (with GPS speed)
    drawDashboard(rpm, map, pw_ms, afr, gps_fix ? gps_speed_kph : demo_spd, gps_fix);
    delay(250);
}
