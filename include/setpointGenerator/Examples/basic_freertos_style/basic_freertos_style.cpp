#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// If setpointGenerator.h is in "include/", this will find it:
#include "setpointGenerator.h"

// Optional: LEDC example (simple PWM on a GPIO)
// Comment out these includes if you don't want PWM output.
#include "driver/ledc.h"
#include "esp_err.h"

static const char* TAG = "wave_demo";

// ─── Demo generators ─────────────────────────────────────────────────────────
// 10-bit range (0..1023), 2.0 s period:
// - sine: base 2.0 s
// - square: 30% "low" time at MIN
// - triangle: +90° phase offset
// - saw: standard ramp
static setpointGenerator s_sine  (0, 1023, 2.0f, WaveformType::SINE);
static setpointGenerator s_square(0, 1023, 2.0f, WaveformType::SQUARE);
static setpointGenerator s_triang(0, 1023, 2.0f, WaveformType::TRIANGLE);
static setpointGenerator s_saw   (0, 1023, 2.0f, WaveformType::SAWTOOTH);

// ─── Optional: LEDC PWM setup to visualize one generator on a pin ───────────
// Change to a valid GPIO on your board (LED pin if you have one).
static constexpr gpio_num_t PWM_GPIO = GPIO_NUM_2;

static esp_err_t init_ledc_10bit()
{
    // Timer: 10-bit resolution, ~20 kHz is fine for LED / light loads.
    ledc_timer_config_t tcfg = {};
    tcfg.speed_mode       = LEDC_LOW_SPEED_MODE;
    tcfg.timer_num        = LEDC_TIMER_0;
    tcfg.duty_resolution  = LEDC_TIMER_10_BIT;
    tcfg.freq_hz          = 20000;
    tcfg.clk_cfg          = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    // Channel on PWM_GPIO, initial duty 0
    ledc_channel_config_t ccfg = {};
    ccfg.speed_mode     = LEDC_LOW_SPEED_MODE;
    ccfg.channel        = LEDC_CHANNEL_0;
    ccfg.timer_sel      = LEDC_TIMER_0;
    ccfg.intr_type      = LEDC_INTR_DISABLE;
    ccfg.gpio_num       = PWM_GPIO;
    ccfg.duty           = 0;
    ccfg.hpoint         = 0;

    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
    return ESP_OK;
}

static void wave_task(void* arg)
{
    // Configure demo parameters
    s_square.setSquareDutycycle(0.30f);         // 30% MIN (LOW) time
    s_triang.setPhaseOffsetDegrees(90.0f);      // +90° phase lead

    // Optional: uncomment to set exact 10-bit range
    // s_sine.setDutycycleResolution(10);
    // s_square.setDutycycleResolution(10);
    // s_triang.setDutycycleResolution(10);
    // s_saw.setDutycycleResolution(10);

    // Optional: LEDC init — comment out if not using PWM output
    if (init_ledc_10bit() == ESP_OK)
    {
        ESP_LOGI(TAG, "LEDC initialized on GPIO %d (10-bit, ~20 kHz)", PWM_GPIO);
    }

    const TickType_t periodTicks = pdMS_TO_TICKS(20); // 50 Hz sampling
    TickType_t lastWake = xTaskGetTickCount();

    while (true)
    {
        // Get the current integer duty from each generator
        const int d_sine   = s_sine.get();
        const int d_square = s_square.get();
        const int d_triang = s_triang.get();
        const int d_saw    = s_saw.get();

        // Log a Serial Plotter-friendly line:
        // Labels with values separated by spaces
        // (If you prefer CSV: use commas and no labels.)
        ESP_LOGI(TAG,
                 "SINE:%d SQUARE:%d TRIANGLE:%d SAW:%d",
                 d_sine, d_square, d_triang, d_saw);

        // Optional: drive PWM with one of the waveforms (e.g., sine)
        // Note: 10-bit LEDC → duty is 0..1023
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, d_sine);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        vTaskDelayUntil(&lastWake, periodTicks);
    }
}

extern "C" void app_main(void)
{
    // Create the demonstration task
    xTaskCreate(wave_task, "wave_task", 4096, nullptr, 5, nullptr);
}
