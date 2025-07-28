/**
 * ESP32 EFI Controller - MAP and O2 Sensor Only, with RPM from Ignition Points (8 Cyl)
 * Framework: ESP-IDF (C)
 * Author: Thomson-BG (MIT License)
 * 
 * Description:
 * Simplified EFI controller for carb-to-EFI retrofit using MAP and (optional) O2 sensor.
 * RPM is measured from ignition coil negative (points ignition, 8 cylinders).
 * Fueling is based on speed-density with MAP and measured RPM.
 * O2 sensor can optionally provide closed-loop correction.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

// --- CONFIGURABLE PINS AND CONSTANTS ---
#define MAP_SENSOR_ADC_CH   ADC1_CHANNEL_0   // GPIO36 (VP)
#define O2_SENSOR_ADC_CH    ADC1_CHANNEL_7   // GPIO35 (optional)

#define INJECTOR_GPIO       18               // PWM output pin for injector driver

#define INJECTOR_PWM_FREQ   100              // Hz (low freq for pulse control)
#define INJECTOR_PWM_TIMER  LEDC_TIMER_0
#define INJECTOR_PWM_MODE   LEDC_LOW_SPEED_MODE
#define INJECTOR_PWM_CH     LEDC_CHANNEL_0

#define IGNITION_PULSE_GPIO GPIO_NUM_4       // Input from coil negative/points

// Engine configuration
#define ENGINE_DISPLACEMENT_L   5.7f         // Example: 350ci ~ 5.7L
#define NUM_CYLINDERS           8
#define INJECTOR_FLOW_RATE_CC   170          // injector size in cc/min

// AFR targets
#define AFR_STOICH              14.7f

static const char *TAG = "EFI";

// --- RPM from points ignition ---
volatile uint64_t last_calc_time_us = 0;
volatile uint32_t ign_pulse_count = 0;
volatile float engine_rpm = 0;

// Interrupt handler for ignition pulse (points opening or coil negative)
static void IRAM_ATTR ign_pulse_isr_handler(void* arg) {
    ign_pulse_count++;
    uint64_t now = esp_timer_get_time();
    // Calculate RPM every 200 ms to reduce noise
    if (now - last_calc_time_us > 200000) {
        // For 8-cyl, 4-stroke, points fire 4 times per revolution
        // RPM = (pulses / 4) * (60 / seconds)
        float seconds = (now - last_calc_time_us) / 1000000.0f;
        engine_rpm = ((float)ign_pulse_count / 4.0f) * (60.0f / seconds);
        ign_pulse_count = 0;
        last_calc_time_us = now;
    }
}

void ign_pulse_input_init() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // points open = coil negative rises
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << IGNITION_PULSE_GPIO),
        .pull_up_en = 1,
        .pull_down_en = 0,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(IGNITION_PULSE_GPIO, ign_pulse_isr_handler, NULL);
}

// --- Sensor Functions ---

float read_map_kpa()
{
    int adc = adc1_get_raw(MAP_SENSOR_ADC_CH);
    // Assume 0-4095 = 20-110kPa for typical MAP sensor
    return 20.0f + (adc / 4095.0f) * (110.0f - 20.0f);
}

// Simulated function: Reads O2 sensor and returns lambda (1.0 = stoich)
float read_o2_lambda()
{
    int adc = adc1_get_raw(O2_SENSOR_ADC_CH);
    float voltage = (adc / 4095.0f) * 3.3f; // ESP32 ADC reference is 3.3V
    // Wideband example:
    // float lambda = 0.7f + (voltage / 3.3f) * (1.3f - 0.7f);
    // return lambda;

    // Narrowband example (simple):
    // return (voltage > 0.45f) ? 0.95f : 1.05f;

    // For demonstration, return 1.0 (stoich)
    return 1.0f;
}

// Injector pulse width calculation (speed-density, batch fire)
float calculate_injector_pw_ms(float map_kpa, float afr, float rpm)
{
    float ve = 0.85f;           // volumetric efficiency (tune!)
    // Intake air mass per cycle (simplified speed-density)
    float imap = (rpm * map_kpa * ve * ENGINE_DISPLACEMENT_L) / (120.0f * 100.0f); // L/min

    // Convert to injector pulse width (ms)
    float fuel_cc_per_min = imap / afr * 1000.0f; // cc/min
    float pw_ms = (fuel_cc_per_min / INJECTOR_FLOW_RATE_CC) * 60000.0f / rpm; // ms

    // Clamp
    if (pw_ms < 2.0f) pw_ms = 2.0f;
    if (pw_ms > 20.0f) pw_ms = 20.0f;
    return pw_ms;
}

// --- Main EFI Loop Task ---
void efi_task(void *pvParameter)
{
    ESP_LOGI(TAG, "EFI task started");

    // Initialize injector PWM channel (LEDC)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = INJECTOR_PWM_MODE,
        .timer_num        = INJECTOR_PWM_TIMER,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = INJECTOR_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = INJECTOR_PWM_MODE,
        .channel        = INJECTOR_PWM_CH,
        .timer_sel      = INJECTOR_PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = INJECTOR_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    // Setup ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MAP_SENSOR_ADC_CH, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(O2_SENSOR_ADC_CH, ADC_ATTEN_DB_11);

    // Initialize ignition pulse (RPM) input
    ign_pulse_input_init();

    while (1) {
        // --- Read sensors ---
        float map_kpa = read_map_kpa();

        // Default target AFR
        float afr_target = AFR_STOICH;

        // Optional: Closed-loop correction if O2 sensor available
        #ifdef USE_O2_SENSOR
        float lambda = read_o2_lambda();
        // Simple proportional closed-loop: adjust AFR by lambda deviation
        afr_target *= lambda;
        #endif

        // Use measured RPM from ignition/points
        float rpm = engine_rpm;
        if (rpm < 250.0f) rpm = 250.0f; // prevent divide-by-zero/very low idle

        // --- Calculate injector pulse width ---
        float pw_ms = calculate_injector_pw_ms(map_kpa, afr_target, rpm);

        ESP_LOGI(TAG, "MAP: %.1fkPa RPM: %.1f AFR: %.2f PW: %.2fms", map_kpa, rpm, afr_target, pw_ms);

        // --- Drive injector: set PWM duty cycle to achieve pw_ms at INJECTOR_PWM_FREQ ---
        // At 100Hz, each cycle = 10ms. Duty = pw_ms / 10ms
        uint32_t max_duty = (1 << 13) - 1; // 13-bit
        float period_ms = 1000.0f / INJECTOR_PWM_FREQ;
        uint32_t duty = (uint32_t)((pw_ms / period_ms) * max_duty);
        if (duty > max_duty) duty = max_duty;
        ledc_set_duty(INJECTOR_PWM_MODE, INJECTOR_PWM_CH, duty);
        ledc_update_duty(INJECTOR_PWM_MODE, INJECTOR_PWM_CH);

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz update cycle
    }
}

// --- Main app entry ---
void app_main()
{
    // Initialize EFI task
    xTaskCreate(&efi_task, "efi_task", 4096, NULL, 5, NULL);
}
