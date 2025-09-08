#include <IntervalTimerHz.h>

IntervalTimerHz timer(5.0f);  // 5 Hz

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    if (timer.ready())
    {
        Serial.printf("Smoothed Interval = %.3f sec, Jitter = %.3f ms\n",
                      timer.getSmoothedElapsedSec(), timer.getSmoothedJitterMs());
    }
}
