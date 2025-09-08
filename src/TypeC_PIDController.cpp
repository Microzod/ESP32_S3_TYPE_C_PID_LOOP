#include "TypeC_PIDController.h"


TypeC_PIDController::TypeC_PIDController(const controlSettings& config)
    : settings(config), integral(0.0f), prevMeasured(0.0f), prevFilteredD(0.0f)
{}

void TypeC_PIDController::reset()
{
    integral = 0.0f;
    prevMeasured = 0.0f;
    prevFilteredD = 0.0f;
}

float TypeC_PIDController::lowPassFilter(float value)
{
    prevFilteredD = settings.LPF_ALPHA * value + (1.0f - settings.LPF_ALPHA) * prevFilteredD;
    return prevFilteredD;
}

int TypeC_PIDController::update(int setpointTicks50ms, int measuredTicks50ms)
{
    float error = static_cast<float>(setpointTicks50ms - measuredTicks50ms);

    // Integral term (I acts on error)
    integral += error;

    // Derivative on measurement (D acts on -measured)
    float dInput = measuredTicks50ms - prevMeasured;
    prevMeasured = measuredTicks50ms;
    float dTerm = -lowPassFilter(dInput);

    float output =
        (settings.Kp * measuredTicks50ms) +
        (settings.Ki * integral) +
        (settings.Kd * dTerm);

    return static_cast<int>(output);
}
