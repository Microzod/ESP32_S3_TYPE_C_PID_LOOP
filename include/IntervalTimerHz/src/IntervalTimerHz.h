#pragma once
#include <esp_timer.h>
#include <cmath>

class IntervalTimerHz
{
public:
    explicit IntervalTimerHz(float frequencyHz = 1.0f)
    {
        setFrequency(frequencyHz);
        last_us = esp_timer_get_time();
    }

    void setFrequency(float frequencyHz)
    {
        interval_us = static_cast<int64_t>(1e6f / frequencyHz);
        expected_ms = 1000.0f / frequencyHz;
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
    float getSmoothedElapsedSec(float alpha = 0.1f) const { return smoothedElapsed; }
    float getSmoothedJitterMs() const { return jitter; }

private:
    int64_t last_us = 0;
    int64_t interval_us = 1000000;
    float expected_ms = 1000.0f;
    float smoothedElapsed = 1.0f;
    float jitter = 0.0f;
};
