#pragma once
#include <esp_timer.h>
#include <cmath>
#include <cstdio>  // for optional debug logging

class IntervalTimer
{
public:
    IntervalTimer() = default;

    IntervalTimer(float value, bool useHz = false)
    {
        if (useHz)
            setFrequencyHz(value);
        else
            setPeriodMs(value);

        last_us = esp_timer_get_time();
    }

    void setPeriodMs(float periodMs)
    {
        if (periodMs <= 0.0f)
        {
            printf("⚠️ IntervalTimer: Invalid period %.3f ms\n", periodMs);
            periodMs = 1.0f;
        }

        interval_us = static_cast<int64_t>(periodMs * 1000.0f);
        expected_ms = periodMs;
    }

    void setFrequencyHz(float freqHz)
    {
        if (freqHz <= 0.0f)
        {
            printf("⚠️ IntervalTimer: Invalid frequency %.3f Hz\n", freqHz);
            freqHz = 1.0f;
        }

        float periodMs = 1000.0f / freqHz;
        setPeriodMs(periodMs);
    }

    bool ready()
    {
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - last_us;

        if (elapsed >= interval_us)
        {
            float elapsed_sec = elapsed / 1e6f;
            smoothedElapsed = 0.9f * smoothedElapsed + 0.1f * elapsed_sec;

            float elapsed_ms = elapsed / 1000.0f;
            jitter = 0.9f * jitter + 0.1f * fabsf(elapsed_ms - expected_ms);

            last_us = now;
            return true;
        }

        return false;
    }

    float getElapsedSec() const { return (esp_timer_get_time() - last_us) / 1e6f; }
    float getSmoothedElapsedSec() const { return smoothedElapsed; }
    float getSmoothedJitterMs() const { return jitter; }
    float getExpectedPeriodMs() const { return expected_ms; }
    float getCurrentFrequencyHz() const { return 1000.0f / expected_ms; }

private:
    int64_t last_us = 0;
    int64_t interval_us = 50000;     // default to 50ms
    float expected_ms = 50.0f;
    float smoothedElapsed = 1.0f;
    float jitter = 0.0f;
};
