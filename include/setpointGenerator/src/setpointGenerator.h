// ─────────────────────────────────────────────────────────────────────────────
// setpointGenerator.h — written by me, Dave. chatGPT?... never heard of it...
//
// Purpose:
//   Generates integer dutycycle setpoints over time in common waveform shapes
//   (sine, square, triangle, sawtooth) using the ESP32 high-resolution timer.
//
// Features:
//   • Configurable duty range (min/max or full N-bit resolution).
//   • Configurable period (seconds) or frequency (Hz).
//   • Variable-duty square wave (not just 50%).
//   • Phase offset control in cycles, radians, or degrees.
//   • Live phase nudging without reset (cycles, radians, degrees).
//
// Typical uses:
//   • Drive PWM duty as a test stimulus.
//   • Feed lookup-table sampling routines with smooth sweeps.
//   • Generate two signals with fixed phase relationship.
//   • Create repeatable patterns for motor, LED, or DAC experiments.
//
// Return value is always an integer duty within the specified range.
// ─────────────────────────────────────────────────────────────────────────────

#pragma once                                // Ensure the header is included only once per translation unit

#include <cstdint>                           // Fixed-width integer types (e.g., int64_t)
#include <cmath>                             // Math functions like sinf, floor
#include "esp_timer.h"                       // ESP-IDF high-resolution timer for microsecond timestamps

#ifndef M_PI                                  // Some environments don’t define M_PI unless a macro is set
#define M_PI 3.14159265358979323846           // Define π if missing so radians helpers work consistently
#endif

// Enumerates the waveform shapes this generator can produce
enum class WaveformType
{
    SINE,                                     // Smooth 0→1→0 sinusoid mapped to duty range
    SQUARE,                                   // Two-level output with configurable low/high segment ratio
    TRIANGLE,                                 // Linear up then linear down each period
    SAWTOOTH                                  // Linear 0→1 ramp then wrap to 0
};

// Generates integer dutycycle setpoints over time using the ESP timer as a timebase
class setpointGenerator
{
public:
    // Construct with integer duty range [minDuty, maxDuty] and period in seconds.
    // Example: bits=10 → setDutyResolution(10) → range 0..1023
    setpointGenerator(  int minDuty,
                        int maxDuty,
                        float periodSec,
                        WaveformType type = WaveformType::SINE)
      : minDuty_(minDuty),                              // Store minimum duty (clamped via setRange-style swap below)
        maxDuty_(maxDuty),                              // Store maximum duty
        period_(periodSec > 0.0f ? periodSec : 1.0f),   // Enforce a positive period (fallback to 1 s)
        type_(type),                                    // Initial waveform type
        phaseOffsetCycles_(0.0f),                       // Start with zero phase offset (can be changed later)
        squareDuty01_(0.5f)                             // Default square-wave duty: 50% time at MIN, 50% at MAX
    {
        if (minDuty_ > maxDuty_)              // If caller provided inverted range, swap for safety
        {
            int tmp = minDuty_;
            minDuty_ = maxDuty_;
            maxDuty_ = tmp;
        }

        reset();                              // Capture current time as phase zero (plus any phase offset)
    }

    // Convenience: set the dutycycle "resolution" to full scale of an N-bit PWM.
    // For example, bits=8 → range 0..255, bits=10 → 0..1023, bits=12 → 0..4095.
    // You can call setRange() after this if you want a narrower sub-range.
    void setDutycycleResolution(uint8_t bits)
    {
        if (bits == 0 || bits > 16)           // Guard against weird inputs; ESP timers typically ≤16 bits
        {
            bits = 10;                        // Reasonable default to 10-bit (0..1023)
        }

        const int fullScale = (1 << bits) - 1;// Compute max code for N-bit resolution
        setRange(minDuty_, fullScale);               // Apply as the working [min,max] range
    }

    void setWaveform(WaveformType newType)    // Change the waveform shape at runtime
    {
        type_ = newType;
    }

    void setPeriod(float seconds)             // Set period (seconds) directly
    {
        period_ = (seconds > 0.0f) ? seconds : 1.0f; // Keep positive with 1 s fallback
    }

    // Optional helpers if you prefer frequency semantics instead of period
    void setFrequencyHz(float hz)             // Set frequency and derive period
    {
        period_ = (hz > 0.0f) ? (1.0f / hz) : 1.0f; // Avoid divide-by-zero; fallback to 1 Hz
    }

    // Set absolute integer duty range (order-agnostic; will sort if inverted)
    void setRange(int minDuty, int maxDuty)
    {
        if (minDuty <= maxDuty)
        {
            minDuty_ = minDuty;
            maxDuty_ = maxDuty;
        }
        else
        {
            minDuty_ = maxDuty;
            maxDuty_ = minDuty;
        }
    }

    // Square-wave duty as fraction of period spent at MIN (0..1).
    // 0.0 → always max; 1.0 → always min; 0.5 → classic 50% square.
    void setSquareDutycycle(float duty01)
    {
        if (duty01 < 0.0f) duty01 = 0.0f;     // Clamp lower bound
        if (duty01 > 1.0f) duty01 = 1.0f;     // Clamp upper bound
        squareDuty01_ = duty01;               // Store fraction of cycle to output the MIN level
    }

    // Phase offset in "cycles" (0..1 means 0–100% of period). Negative values also allowed.
    void setPhaseOffset(float offsetCycles)
    {
        phaseOffsetCycles_ = normalizeUnitInterval(offsetCycles); // Normalize to [0,1)
    }

    // Phase offset in radians (2π rad = 1 cycle) for sine/triangle/saw alignment by angle
    void setPhaseOffsetRadians(float radians)
    {
        setPhaseOffset(radians / (2.0f * static_cast<float>(M_PI))); // Convert to cycles
    }

    // Phase offset in degrees (360° = 1 cycle)
    void setPhaseOffsetDegrees(float degrees)
    {
        setPhaseOffset(degrees / 360.0f);     // Convert to cycles
    }

    // Live nudge of phase by Δcycles (positive or negative) without resetting time origin
    void advancePhase(float deltaCycles)
    {
        phaseOffsetCycles_ = normalizeUnitInterval(phaseOffsetCycles_ + deltaCycles); // Wrap to [0,1)
    }

    // Live nudge in radians
    void advancePhaseRadians(float deltaRadians)
    {
        advancePhase(deltaRadians / (2.0f * static_cast<float>(M_PI))); // Convert and nudge
    }

    // Live nudge in degrees
    void advancePhaseDegrees(float deltaDegrees)
    {
        advancePhase(deltaDegrees / 360.0f);  // Convert and nudge
    }

    // Reset the time origin (phase = phaseOffset at t=now). Useful when starting tests.
    void reset()
    {
        startTimeUs_ = esp_timer_get_time();  // Capture current microsecond timestamp as new t0
    }

    // Get current integer duty. Clamped to [minDuty_, maxDuty_].
    int get() const
    {
        const float phase = currentPhase();                 // Normalized phase in [0,1) including phase offset
        const float minF = static_cast<float>(minDuty_);    // Work in float for intermediate math
        const float maxF = static_cast<float>(maxDuty_);
        const float range = maxF - minF;                    // Span of the duty interval

        float valueF = minF;                                // Default to min; we’ll compute true value per waveform

        switch (type_)      // Dispatch per selected waveform
        {
            case WaveformType::SINE:
            {
                // Map sin to [0,1]: 0.5 * (1 + sin(2π·phase))
                constexpr float twoPi = 6.28318530718f;                         // Faster than computing each call
                valueF = minF + range * 0.5f * (1.0f + sinf(twoPi * phase));    // Smooth oscillation
                break;
            }

            case WaveformType::SQUARE:
            {
                // For phase in [0, squareDuty01_) → min; else → max
                valueF = (phase < squareDuty01_) ? minF : maxF; // Variable “low time” square wave
                break;
            }

            case WaveformType::TRIANGLE:
            {
                // Linear rise over first half-cycle, linear fall over second half-cycle.
                // The 2.0f factor ensures full range is covered in half the cycle.
                if (phase < 0.5f)
                {
                    valueF = minF + 2.0f * range * phase;           // Up-slope
                }
                else
                {
                    valueF = maxF - 2.0f * range * (phase - 0.5f);  // Down-slope
                }
                break;
            }

            case WaveformType::SAWTOOTH:
            {
                valueF = minF + range * phase; // Linear 0→1 mapping per cycle
                break;
            }

            default:
            {
                valueF = minF;                 // Fallback: output min
                break;
            }
        }

        // Round to nearest integer and clamp to the configured range
        int value = static_cast<int>(lroundf(valueF)); // Convert float result to int with rounding
        if (value < minDuty_)                          // Lower clamp
        {
            value = minDuty_;
        }
        else if (value > maxDuty_)                     // Upper clamp
        {
            value = maxDuty_;
        }
        return value;                                  // Final integer duty setpoint
    }

    // Expose normalized phase in [0,1) after applying phase offset (useful for diagnostics)
    float currentPhase() const
    {
        const int64_t nowUs = esp_timer_get_time();                         // Current microseconds since boot
        const float t = static_cast<float>(nowUs - startTimeUs_) / 1e6f;    // Elapsed seconds since reset()
        const float raw = (t / period_) + phaseOffsetCycles_;               // Base phase + offset (unwrapped)
        return raw - std::floor(raw);                                       // Normalize to [0,1) via fractional part
    }

private:
    // Normalize any real value to the half-open interval [0,1)
    static float normalizeUnitInterval(float x)
    {
        float y = x - std::floor(x);                            // Remove integer part (wrap around)
        return (y >= 1.0f) ? 0.0f : (y < 0.0f ? y + 1.0f : y);  // Ensure 1 maps to 0 and negatives wrap
    }

private:
    int   minDuty_;                                    // Lower bound of integer duty output
    int   maxDuty_;                                    // Upper bound of integer duty output
    float period_;                                     // Waveform period in seconds
    WaveformType type_;                                // Current waveform selection
    float phaseOffsetCycles_;                          // Phase offset as fraction of a cycle (0..1)
    float squareDuty01_;                               // Square “low time” fraction (0..1) spent at MIN
    int64_t startTimeUs_;                              // Time origin captured by reset() (microseconds)
};
