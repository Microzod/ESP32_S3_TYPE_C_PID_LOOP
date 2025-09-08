#ifndef TYPEC_PID_CONTROLLER_H
#define TYPEC_PID_CONTROLLER_H

#include "nvsSettings_definition.h"  // Ensure this includes `struct controlSettings`
#include <GptimerBeat.h>
struct controlSettings;

class TypeC_PIDController
{
public:
    TypeC_PIDController(const controlSettings& config);

    void reset();
    int update(int setpointTicks50ms, int measuredTicks50ms);

private:
    const controlSettings& settings;
    float integral;
    float prevMeasured;
    float prevFilteredD;

    float lowPassFilter(float value);
};

#endif
