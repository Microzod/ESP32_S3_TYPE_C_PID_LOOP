#ifndef DYNAMIC_LUT_BLENDER_H
#define DYNAMIC_LUT_BLENDER_H

#include <stddef.h>
#include <math.h>

class DynamicLUTBlender
{
public:
    DynamicLUTBlender(const float* slow, const float* fast, size_t size)
        : slowLUT(slow), fastLUT(fast), length(size)
    {
        blendAlpha = 0.0f;
        prevSpeed = 0.0f;
        smoothedAccel = 0.0f;
    }

    void setBlendFactor(float alpha)
    {
        blendAlpha = _clamp(alpha, 0.0f, 1.0f);
    }

    float getBlendFactor() const
    {
        return blendAlpha;
    }

    void setAsymmetricBlending(bool enabled)
    {
        asymmetric = enabled;
    }

    void setSmoothingFactor(float alpha)
    {
        smoothingAlpha = _clamp(alpha, 0.01f, 0.99f);
    }

    void setAccelerationThresholds(float posThreshold, float negThreshold)
    {
        accelPosThresh = posThreshold;
        accelNegThresh = negThreshold;
    }

    // Provide current speed every cycle (ticks/50ms, for example)
    void updateBlendFactorFromAccel(float currentSpeed)
    {
        float accel = currentSpeed - prevSpeed;
        prevSpeed = currentSpeed;

        // Smooth the acceleration
        smoothedAccel = smoothingAlpha * accel + (1.0f - smoothingAlpha) * smoothedAccel;

        // Adjust blendAlpha dynamically
        if (asymmetric)
        {
            if (smoothedAccel > accelPosThresh)
                blendAlpha = 1.0f;  // Fully fast
            else if (smoothedAccel < accelNegThresh)
                blendAlpha = 0.0f;  // Fully slow
            else
                blendAlpha = 0.5f;  // Blend in between
        }
        else
        {
            float range = accelPosThresh - accelNegThresh;
            float centered = (smoothedAccel - accelNegThresh) / range;
            blendAlpha = _clamp(centered, 0.0f, 1.0f);
        }
    }

    float getBlendedValueFromIndex(size_t index) const
    {
        if (index >= length) return 0.0f;
        return (1.0f - blendAlpha) * slowLUT[index] + blendAlpha * fastLUT[index];
    }

    // Lookup by floating-point position (e.g. speed)
    float getBlendedValueFromInput(float input) const
    {
        if (length == 0) return 0.0f;
        if (input <= 0.0f) return getBlendedValueFromIndex(0);
        if (input >= (float)(length - 1)) return getBlendedValueFromIndex(length - 1);

        size_t i = (size_t)input;
        float frac = input - i;
        float v0 = getBlendedValueFromIndex(i);
        float v1 = getBlendedValueFromIndex(i + 1);
        return v0 + frac * (v1 - v0);
    }

private:
    const float* slowLUT;
    const float* fastLUT;
    size_t length;
    float blendAlpha;
    float prevSpeed = 0.0f;
    float smoothedAccel = 0.0f;
    float accelPosThresh = 2.0f;
    float accelNegThresh = -2.0f;
    float smoothingAlpha = 0.1f;
    bool asymmetric = false;

    static float _clamp(float val, float minVal, float maxVal)
    {
        if (val < minVal) return minVal;
        if (val > maxVal) return maxVal;
        return val;
    }
};

#endif  // DYNAMIC_LUT_BLENDER_H
