#pragma once
#include <stdint.h>
#include <GptimerBeat.h>
#include <encoderEncoderPCNT.h>

// Forward declaration of your encoder object.
//class encoderEncoderPCNT;
extern encoderEncoderPCNT encoder;

// Start/stop the control task that samples encoders each timer tick.
bool startControlTask(GptimerBeat& beat, uint32_t period_us = 50000, int core_id = 1);
void stopControlTask();

// Latest readings (atomic-enough for simple use; read-only from other tasks)
int32_t getLastTicksA();          // raw ticks since last sample window
int32_t getLastTicksB();
float   getLastTicksA_50ms();     // normalized to exactly 50 ms
float   getLastTicksB_50ms();
uint64_t getLastSampleUs();       // esp_timer timestamp at sample moment
