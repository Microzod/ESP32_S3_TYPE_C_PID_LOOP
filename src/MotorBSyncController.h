#ifndef MOTOR_B_SYNC_CONTROLLER_H
#define MOTOR_B_SYNC_CONTROLLER_H

#include <stdint.h>
#include <cmath>
#include <stdio.h>
#include "nvsSettings_definition.h"  // For `controlSettings`

class MotorBSyncController
{
public:
    enum State
    {
        IDLE,
        KICKING,
        SYNCING,
        PID_ACTIVE
    };

    MotorBSyncController()
        : state(IDLE),
          prevError(0),
          integral(0),
          twistErrorTurns(0.0f),
          lastUpdateTimeUs(0),
          Kp(0), Ki(0), Kd(0), LPF_ALPHA(0), CPR(56)
    {}

    void configureFromSettings(const controlSettings& s, uint32_t cpr)
    {
        Kp = s.Kp;
        Ki = s.Ki;
        Kd = s.Kd;
        LPF_ALPHA = s.LPF_ALPHA;
        CPR = cpr;
    }

    void reset()
    {
        prevError = 0;
        integral = 0;
        twistErrorTurns = 0.0f;
        lastUpdateTimeUs = 0;
        state = IDLE;
    }

    void start()
    {
        reset();
        state = KICKING;
    }

    void update(int motorA_ticks_50ms, int motorB_ticks_50ms)
    {
        uint64_t now = esp_timer_get_time();
        if (lastUpdateTimeUs == 0)
        {
            lastUpdateTimeUs = now;
            return;
        }

        float dt = (now - lastUpdateTimeUs) / 1e6f;
        lastUpdateTimeUs = now;

        int error = motorA_ticks_50ms - motorB_ticks_50ms;

        // Exponential smoothing for derivative
        float rawD = (error - prevError) / dt;
        dFiltered = LPF_ALPHA * rawD + (1.0f - LPF_ALPHA) * dFiltered;

        integral += error * dt;
        twistErrorTurns += (error * dt) / CPR;
        prevError = error;

        output = Kp * error + Ki * integral + Kd * dFiltered;

        // State transition
        if (state == KICKING && std::abs(error) < 5)
        {
            state = SYNCING;
        }
        else if (state == SYNCING && std::abs(error) < 3)
        {
            state = PID_ACTIVE;
        }
    }

    float getOutput() const { return output; }
    float getTwistErrorTurns() const { return twistErrorTurns; }
    int getError() const { return prevError; }
    float getD() const { return dFiltered; }
    float getI() const { return integral; }
    State getState() const { return state; }

    void printDebug()
    {
        printf("E:%d I:%.2f D:%.2f PID:%.2f Twist:%.2f State:%d\n",
               prevError, integral, dFiltered, output, twistErrorTurns, (int)state);
    }

private:
    State state;
    float prevError;
    float integral;
    float dFiltered = 0;
    float output;
    float twistErrorTurns;
    uint64_t lastUpdateTimeUs;

    float Kp, Ki, Kd, LPF_ALPHA;
    uint32_t CPR;
};

#endif // MOTOR_B_SYNC_CONTROLLER_H
