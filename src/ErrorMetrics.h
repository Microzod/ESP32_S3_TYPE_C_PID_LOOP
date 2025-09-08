#pragma once
#include <math.h>

class ErrorMetrics
{
public:
    void update(float setpoint, float feedback, float dtSec)
    {
        float error = setpoint - feedback;

        totalSamples++;
        absErrorSum += fabsf(error);
        squaredErrorSum += error * error;
        integralError += error * dtSec;

        // Track overshoot
        if (feedback > setpoint)
        {
            float overshootNow = feedback - setpoint;
            if (overshootNow > maxOvershoot) {
                maxOvershoot = overshootNow;
            }
        }

        // Settling logic
        if (fabsf(error) < settlingThreshold) {
            timeInTolerance += dtSec;
            if (timeInTolerance >= minStableTime && settlingTime < 0)
            {
                settlingTime = totalElapsed;
            }
        } else {
            timeInTolerance = 0;  // reset tolerance window
        }

        totalElapsed += dtSec;
    }

    void reset()
    {
        totalSamples = 0;
        absErrorSum = 0;
        squaredErrorSum = 0;
        integralError = 0;

        maxOvershoot = 0;
        totalElapsed = 0;
        timeInTolerance = 0;
        settlingTime = -1;
    }

    float getMAE() const   { return (totalSamples > 0) ? absErrorSum / totalSamples : 0; }
    float getRMSE() const  { return (totalSamples > 0) ? sqrtf(squaredErrorSum / totalSamples) : 0; }
    float getIAE() const   { return integralError; }
    float getOvershoot() const { return maxOvershoot; }
    float getSettlingTime() const { return settlingTime; }

    void print()
    {
        printf("MAE=%.3f, RMSE=%.3f, Overshoot=%.2f, Settled=%.2fs, IAE=%.3f\n",
            getMAE(), getRMSE(), getOvershoot(), getSettlingTime(), getIAE());
    }

    void setSettlingThreshold(float errorTolerance, float timeRequired = 0.5f)
    {
        settlingThreshold = errorTolerance;
        minStableTime = timeRequired;
    }

private:
    int totalSamples = 0;
    float absErrorSum = 0;
    float squaredErrorSum = 0;
    float integralError = 0;

    float maxOvershoot = 0;

    float totalElapsed = 0;
    float timeInTolerance = 0;
    float settlingTime = -1;

    float settlingThreshold = 0.1f;
    float minStableTime = 0.5f;
};
