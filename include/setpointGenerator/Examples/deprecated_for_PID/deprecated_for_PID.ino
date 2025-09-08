#include "setpointGenerator.h"
#include "esp_log.h"

// Example PID variables
float setpointRPS = 0.0f;
float measuredRPS = 0.0f;
float controlOutput = 0.0f;

// Setpoint generator: SINE wave from 5 to 15 RPS over 6 seconds
SetpointGenerator setpointGen(5.0f, 15.0f, 6.0f, WaveformType::SINE);

extern "C" void app_main()
{
    while (true)
    {
        // ─ Update setpoint from generator ─
        setpointRPS = setpointGen.get();

        // ─ Example placeholder: you would measure actual RPS here ─
        measuredRPS = readMotorRPS();  // your encoder logic

        // ─ PID computation (simplified) ─
        float error = setpointRPS - measuredRPS;
        controlOutput = computePID(error);  // your PID logic

        // ─ Apply to motor driver ─
        setMotorPWM(controlOutput);  // e.g. MCPWM duty set

        // ─ Logging ─
        ESP_LOGI("SETPOINT", "SP: %.2f RPS, FB: %.2f, OUT: %.2f", setpointRPS, measuredRPS, controlOutput);

        // ─ Control loop delay (100ms) ─
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
