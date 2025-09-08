#include "IntervalTimer.h"

IntervalTimer pidTimer(20.0f, true);  // 20Hz = 50ms interval

void loop()
{
    if (pidTimer.ready())
    {
        // PID update or sync job
    }

    // Runtime change example
    if (userChangedSetting)
    {
        pidTimer.setFrequencyHz(10.0f);  // Switch to 10Hz = 100ms
        // or:
        // pidTimer.setPeriodMs(25.0f);
    }
}
