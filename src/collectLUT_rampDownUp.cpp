#include "collectLUT_rampDownUp.h"
#include <esp_timer.h>
#include <math.h>
#include <cstdio>

void collectLUT_rampDownUp(
    pcnt_unit_handle_t enc,
    mcpwm_cmpr_handle_t pwm,
    MCPWM_DUAL_PWM& pwmDriver,
    encoderEncoderPCNT& encoderAccess,
    both_motors& data)
{
    const int  numSamples = 5;
    const int  interval_ms = 50;
    constexpr int settleTime_ms = 250;
    constexpr int kickSettleTime_ms = 150;
    constexpr int checkForTicksAfterKickTime_ms = 50;
    bool kicksAreNeeded = true;

    // Decide which motor once
    std::string_view label;
    single_motor* m = nullptr;

    if (pwm == pwmDriver.cmprA)
    {
        label = "A";
        m = &data.A;
    }
    else if (pwm == pwmDriver.cmprB)
    {
        label = "B";
        m = &data.B;
    }
    else
    {
        // Unknown PWM -> nothing to do
        return;
    }

    // Use the full array length unless you track an active count elsewhere
    const size_t N = NN;

    // --- Down measurement: set duty, settle, sample numSamples windows
    auto measureTicksDown = [&](int duty) -> int
    {
        int total = 0;
        pwmDriver.setDutyCycle(pwm, duty);
        vTaskDelay(pdMS_TO_TICKS(settleTime_ms));

        for (int i = 0; i < numSamples; ++i)
        {
            pcnt_unit_clear_count(enc);                 // reset PCNT for this window
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
            int ticks = encoderAccess.getCountFrom(enc, false); // read-only (avoid double reset)
            total += ticks;
        }
        return total / numSamples;
    };

    // --- Up measurement with kick-start when needed
    auto measureTicksUp = [&](int duty) -> int
    {
        int total = 0;

        if (kicksAreNeeded)
        {
            // Check if currently stopped
            int isTicksZero = encoderAccess.getCountFrom(enc, false);
            if (isTicksZero == 0)
            {
                encoderAccess.clearTarget(enc);
                pwmDriver.setDutyCycle(pwm, pwmDriver.getResolution());   // Kick-start (assumes max compare)
                while (encoderAccess.getCountFrom(enc, false) < 56)
                {
                    __asm__ __volatile__("nop");
                }
                pwmDriver.setDutyCycle(pwm, duty);    // Apply target duty cycle
                vTaskDelay(pdMS_TO_TICKS(kickSettleTime_ms));
                encoderAccess.clearTarget(enc);
                vTaskDelay(pdMS_TO_TICKS(checkForTicksAfterKickTime_ms));
                if (encoderAccess.getCountFrom(enc, true) > 0)
                {
                    kicksAreNeeded = false;
                }
            }
            else
            {
                kicksAreNeeded = false;
                pwmDriver.setDutyCycle(pwm, duty);
                vTaskDelay(pdMS_TO_TICKS(settleTime_ms));
            }
        }
        else
        {
            pwmDriver.setDutyCycle(pwm, duty);
            vTaskDelay(pdMS_TO_TICKS(settleTime_ms));
        }

        for (int i = 0; i < numSamples; ++i)
        {
            pcnt_unit_clear_count(enc);
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
            int ticks = encoderAccess.getCountFrom(enc, false);  // read-only (avoid double reset)
            total += ticks;
        }
        return total / numSamples;
    };

    auto preformAndPrint = [&]()
    {
        std::string header;
        header.reserve(96);

        // ----- rampDown header -----
        header.clear();
        header.append("const float rampDownLUT_");
        header.append(label.data(), label.size());
        header.append("[] =\n{\n\t");
        std::printf("%s", header.c_str());

        // Ramp down: N-1 -> 0
        for (size_t i = N; i-- > 0; )
        {
            m->resultDownArray[i] = static_cast<float>(measureTicksDown(m->dutycycleArray[i]));
            std::printf("%f,", m->resultDownArray[i]);
        }
        std::printf("\n}\n\n");

        // ----- rampUp header -----
        header.clear();
        header.append("const float rampUpLUT_");
        header.append(label.data(), label.size());
        header.append("[] =\n{\n\t");
        std::printf("%s", header.c_str());

        // Ramp up: 0 -> N-1
        for (size_t i = 0; i < N; ++i)
        {
            m->resultUpArray[i] = static_cast<float>(measureTicksUp(m->dutycycleArray[i]));
            std::printf("%f,", m->resultUpArray[i]);
        }
        std::printf("\n}\n\n");

        // ----- average header -----
        header.clear();
        header.append("const float averageLUT_");
        header.append(label.data(), label.size());
        header.append("[] =\n{\n\t");
        std::printf("%s", header.c_str());

        for (size_t i = 0; i < N; ++i)
        {
            m->resultAvgArray[i] = 0.5f * (m->resultDownArray[i] + m->resultUpArray[i]);
            std::printf("%f,", m->resultAvgArray[i]);
        }
        std::printf("\n}\n");

        // stop motor
        pwmDriver.setDutyCycle(pwm, 0);
    };

    preformAndPrint();
}
