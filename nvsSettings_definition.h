#pragma once

#include <stdint.h>
#include <inttypes.h>
#include "motorLUT.h"

//const int LUT_SIZE;
//extern const float motorB_ticksPer50ms_fast_LUT[];
//extern const float motorB_ticksPer50ms_slow_LUT[];

struct controlSettings {
    float Kp;
    float Ki;
    float Kd;
    float LPF_ALPHA;

    float scaleA;
    float scaleB;
    float vA_full;
    float vB_full;
    int needToCalibrate;

    uint32_t Q = 1000000;

    unsigned int restart_counter;
    
    float reservedForFutureUse_1;
    float reservedForFutureUse_2;
    int reservedForFutureUse_3;
    int reservedForFutureUse_4;
    bool reservedForFutureUse_5;
    bool reservedForFutureUse_6;
};