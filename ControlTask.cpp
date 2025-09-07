#include "ControlTask.h"
#include <esp_timer.h>
#include <Arduino.h>   // for Serial.printf / optional

// ====== Configuration knobs ======
#define CTRL_TASK_STACK   4096
#define CTRL_TASK_PRIO    (configMAX_PRIORITIES - 2)
#define ENABLE_CTRL_PRINT 0  // set to 1 to print "ticksA_50,ticksB_50" each tick
// =================================

static TaskHandle_t     s_ctrlTask    = nullptr;
static GptimerBeat*     s_beat        = nullptr;
static volatile int32_t s_ticksA      = 0;
static volatile int32_t s_ticksB      = 0;
static volatile float   s_ticksA_50   = 0.0f;
static volatile float   s_ticksB_50   = 0.0f;
static volatile uint64_t s_sample_us  = 0;

extern encoderEncoderPCNT encoder;

static void controlTask(void* arg)
{
    // Attach to receive task notifications from the timer ISR
    s_beat->attachTaskNotify(xTaskGetCurrentTaskHandle());

    // Prime the timing
    uint64_t last_us = esp_timer_get_time();

#if ENABLE_CTRL_PRINT
    Serial.println("ticksA_50,ticksB_50");
#endif

    for (;;)
    {
        // Block until next timer tick (counting semantics; usually returns 1)
        (void)s_beat->waitForTick(UINT32_MAX);

        // Timestamp at (or very near) this sample
        uint64_t now_us = esp_timer_get_time();
        uint64_t dt_us  = now_us - last_us;
        if (dt_us == 0) dt_us = 1; // guard

        // Read + reset encoder counts for this window
        // These are the raw ticks during ~dt_us
        int32_t ta = encoder.getCountA(true);
        int32_t tb = encoder.getCountB(true);

        // Normalize to exactly 50 ms for stable units (ticks per 50 ms)
        const float scale = 50000.0f / (float)dt_us;
        float ta_50 = (float)ta * scale;
        float tb_50 = (float)tb * scale;

        // Publish
        s_ticksA     = ta;
        s_ticksB     = tb;
        s_ticksA_50  = ta_50;
        s_ticksB_50  = tb_50;
        s_sample_us  = now_us;

#if ENABLE_CTRL_PRINT
        // Friendly for Arduino Serial Plotter or CSV
        Serial.printf("%.3f,%.3f\n", (double)ta_50, (double)tb_50);
#endif

        last_us = now_us;
        // (loop)
    }
}

bool startControlTask(GptimerBeat& beat, uint32_t period_us, int core_id)
{
    if (s_ctrlTask) return true; // already running

    s_beat = &beat;

    // Start (or reconfigure) the timer beat
    // If already started externally, this is a no-op except period/core check.
    if (!s_beat->start(period_us, core_id))
    {
        return false;
    }
    // If you need to change period at runtime:
    // s_beat->setPeriodUs(period_us);

    BaseType_t ok = xTaskCreatePinnedToCore(
        controlTask, "ctrl", CTRL_TASK_STACK, nullptr, CTRL_TASK_PRIO, &s_ctrlTask, core_id
    );
    return ok == pdPASS;
}

void stopControlTask()
{
    if (!s_ctrlTask) return;
    TaskHandle_t t = s_ctrlTask;
    s_ctrlTask = nullptr;
    vTaskDelete(t);

    // Optionally stop the beat too (only if you own it here)
    // s_beat->stop();
    s_beat = nullptr;
}

// --- Getters (simple volatile reads) ---

int32_t getLastTicksA()         { return s_ticksA; }
int32_t getLastTicksB()         { return s_ticksB; }
float   getLastTicksA_50ms()    { return s_ticksA_50; }
float   getLastTicksB_50ms()    { return s_ticksB_50; }
uint64_t getLastSampleUs()      { return s_sample_us; }
