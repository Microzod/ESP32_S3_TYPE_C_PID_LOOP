#pragma once

#include "motorLUT.h"

// Clamp index to valid range
inline int clampIndex(int index, int max)
{
    if (index < 0) return 0;
    if (index >= max) return max - 1;
    return index;
}

// === Motor B (Ticks per 50ms LUT) ===
inline float getTicksFromDuty_B_fast(int index)
{
    index = clampIndex(index, LUT_SIZE);
    return motorB_ticksPer50ms_fast_LUT[index];
}

inline float getTicksFromDuty_B_slow(int index)
{
    index = clampIndex(index, LUT_SIZE);
    return motorB_ticksPer50ms_slow_LUT[index];
}

// Lookup index by target ticks/50ms (linear search)
inline int getDutyFromTicks_B_fast(float ticks)
{
    for (int i = 0; i < LUT_SIZE; ++i)
    {
        if (motorB_ticksPer50ms_fast_LUT[i] >= ticks)
            return i;
    }
    return LUT_SIZE - 1;
}

inline int getDutyFromTicks_B_slow(float ticks)
{
    for (int i = 0; i < LUT_SIZE; ++i)
    {
        if (motorB_ticksPer50ms_slow_LUT[i] >= ticks)
            return i;
    }
    return LUT_SIZE - 1;
}

// Optional: Blended lookup using DynamicLUTBlender instance
// (Only needed if you use DynamicLUTBlender instead of fixed fast/slow)
#ifdef USE_DYNAMIC_LUT_BLENDER
#include "DynamicLUTBlender.h"

inline float getTicksFromDuty_B_blended(int index, DynamicLUTBlender& blender)
{
    return blender.getBlendedValueFromIndex(index);
}
#endif
