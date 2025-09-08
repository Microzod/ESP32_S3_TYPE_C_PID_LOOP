#pragma once

constexpr int NN = 1024;

struct single_motor
{
    int dutycycleArray[NN] = {};
    float resultFromStallArray[NN] = {};
    float resultDownArray[NN] = {};
    float resultUpArray[NN] = {};
    float resultAvgArray[NN] = {};
};

struct both_motors
{
    struct single_motor A;
    struct single_motor B;
};

//both_motors data;

#include "ESP32_L298N.h"
#include "MCPWM_DUAL_PWM.h"
#include "encoderEncoderPCNT.h"

/*
void kickStart(
    mcpwm_cmpr_handle_t cmprA,
    pcnt_unit_handle_t encA,
    int targetDutycycleA,
    mcpwm_cmpr_handle_t cmprB,
    pcnt_unit_handle_t encB,
    int targetDutycycleB
)
void testWithKickUniversalDual(
    mcpwm_cmpr_handle_t pwmA,
    pcnt_unit_handle_t encA,
    const int (&dutyA)[N],
    int (&resultsA)[N],
    mcpwm_cmpr_handle_t pwmB,
    pcnt_unit_handle_t encB,
    const int (&dutyB)[N],
    int (&resultsB)[N])
{
    for (int i = 0; i < N; i++)
    {
        int da = dutyA[i];
        

        encoder.clearTarget(encA);
        

        // Kick-start both (or just A)
        l298n.pwm.setDutyCycle(pwmA, l298n.pwm.getResolution());
        

        while (encoder.getCountFrom(encA, false) < 56) { __asm__ __volatile__("nop"); }
        

        l298n.pwm.setDutyCycle(pwmA, da);
        
        auto waitForSteady = [](pcnt_unit_handle_t enc) -> bool
        {
            int last[STEADY_COUNT] = {0};
            int idx = 0, stable = 0;
            int64_t t0 = esp_timer_get_time();

            while (true)
            {
                vTaskDelay(pdMS_TO_TICKS(STEADY_WINDOW_MS));
                int count = encoder.getCountFrom(enc, true);
                last[idx] = count;

                if (idx > 0)
                {
                    int delta = abs(last[idx] - last[idx - 1]);
                    stable = (delta < STEADY_THRESHOLD) ? stable + 1 : 0;
                }

                idx = (idx + 1) % STEADY_COUNT;

                if (stable >= (STEADY_COUNT - 1)) return true;
                if ((esp_timer_get_time() - t0) > (STALL_TIMEOUT_MS * 1000)) return false;
            }
        };

        bool stableA = waitForSteady(encA);
        

        if (!stableA)
        {
            printf("[WARN] i=%d → Unstable (A:%d, B:%d)\n", i, da, db);
            resultsA[i] = 0;
            if (dualMode) resultsB[i] = 0;
            continue;
        }

        encoder.clearTarget(encA);
        if (dualMode) encoder.clearTarget(encB);
        int64_t t0 = esp_timer_get_time();
        vTaskDelay(pdMS_TO_TICKS(MEASURE_DURATION_MS));
        int64_t t1 = esp_timer_get_time();
        float dt = (t1 - t0) / 1e6;

        resultsA[i] = encoder.getCountFrom(encA, true) / dt;
        if (dualMode)
        {
            resultsB[i] = encoder.getCountFrom(encB, true) / dt;
            printf("[RESULT] i=%d → A: %d (%d), B: %d (%d)\n", i, da, resultsA[i], db, resultsB[i]);
        }
        else
        {
            printf("[RESULT] i=%d → A: %d (%d)\n", i, da, resultsA[i]);
        }

        // Reset
        l298n.pwm.setDutyCycle(pwmA, 0);
        if (dualMode) l298n.pwm.setDutyCycle(pwmB, 0);
        while (encoder.getCountFrom(encA, true) || (dualMode && encoder.getCountFrom(encB, true)))
        {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void testWithKickUniversalSingle(
    pcnt_unit_handle_t encA,
    mcpwm_cmpr_handle_t pwmA,
    const int (&dutyA)[N],
    int (&resultsA)[N])
{
    testWithKickUniversalDual(
        encA,
        pwmA,
        dutyA,
        resultsA,
        false,         // dualMode = false
        nullptr,       // unused
        nullptr,       // unused
        dummyDuty,
        dummyResults
    );
}

void printResultsWithKickDual(const int (&dutyA)[N],
                          int (&resultsA)[N],
                          bool dualMode,
                          const int (&dutyB)[N],
                          int (&resultsB)[N])
{
    if (dualMode)
    {
        printf("Print results:\nint resultA[%u] =\n{\n    ", N);
        for(int i = 0; i < N; i++)
        {
            printf("%u,%u,", dutyA[i], resultsA[i]);
        }
        printf("\n");
        printf("}\n\n\n");

        printf("Print results:\nint resultB[%u] =\n{\n    ", N);
        for(int i = 0; i < N; i++)
        {
            printf("%u,%u,", dutyB[i], resultsB[i]);
        }
        printf("\n");
        printf("}\n\n\n");
    }
    else
    {
        printf("Print results:\nint resultA[%u] =\n{\n    ", N);
        for(int i = 0; i < N; i++)
        {
            printf("%u,%u,", dutyA[i], resultsA[i]);
        }
        printf("\n");
        printf("}\n\n\n");
    }
}

void printResultsWithKickSingle(const int (&dutyA)[N],
                                int (&resultsA)[N])
{
    printResultsWithKickDual(
        dutyA,
        resultsA,
        false,
        dummyDuty,
        dummyResults
    );
}
*/
