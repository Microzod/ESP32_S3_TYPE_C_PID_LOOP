#ifndef _ESP32_L298N_H_
#define _ESP32_L298N_H_

#include "MCPWM_DUAL_PWM.h"
#include "encoderEncoderPCNT.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_timer.h>  // esp_timer_get_time()
#include <math.h>       // NAN, isnan()

extern "C"
{
    #include "driver/gpio.h"
    #include "esp_err.h"
}

enum KickFlags : uint8_t
{
    KICK_ACT_ON_A       = 1u << 0,
    KICK_A_ATTEMPTED    = 1u << 1,
    KICK_A_IS_STALLED   = 1u << 2,
    KICK_A_FAILURE      = 1u << 3,
    KICK_ACT_ON_B       = 1u << 4,
    KICK_B_ATTEMPTED    = 1u << 5,
    KICK_B_IS_STALLED   = 1u << 6,
    KICK_B_FAILURE      = 1u << 7
};

class ESP32_L298N
{
public:
    // \param pwmPinA       MCPWM pin A
    // \param pwmPinB       MCPWM pin B
    // \param pwm_freq      PWM frequency (Hz)
    // \param pwm_resolution  bit-depth (so resolution = 1<<pwm_resolution)
    // \param in1…in4       direction GPIOs
    ESP32_L298N(int                 pwmPinA,
                int                 pwmPinB,
                uint32_t            pwm_freq,
                uint8_t             pwm_resolution,
                int                 in1,
                int                 in2,
                int                 in3,
                int                 in4,
                encoderEncoderPCNT& encoderAccess);

    
    esp_err_t init();
    
    MCPWM_DUAL_PWM     pwm;
    encoderEncoderPCNT encoder;
    
    mcpwm_cmpr_handle_t     motor_A;
    mcpwm_cmpr_handle_t     motor_B;
    pcnt_unit_handle_t      encoder_A;
	pcnt_unit_handle_t      encoder_B;
    
    void setSpoolDirection();
    void setUnspoolDirection();
    void switchSpoolDirection();

    bool getDirectionFlag() const;
    int getSpeedTicksA();
    int getSpeedTicksB();
    
    void set

    void setDutyMotorA(uint32_t duty);
    void setDutyMotorB(uint32_t duty);

    void stop();
    
    void kickStart(int targetDutycycleA, int targetDutycycleB);
    void kickStartA(int targetDutycycleA);
    void kickStartB(int targetDutycycleB);
    void kickStartAsNeeded(int dutyRequest, int noKickNeededAbove, int positi);

private:

    void setMotorDirection(uint8_t motor, uint8_t direction);
    int waitForSteady(pcnt_unit_handle_t enc);

    int dutycycle_A;
    int dutycycle_B;
    int last50msTicks_A;
    int last50msTicks_B;
    
    int STEADY_CYCLES_TO_PASS = 5;
    int STEADY_WINDOW_DELAY_MS = 100;
    int STEADY_ACCEPTABLE_THRESHOLD = 10;
    int STEADY_STALLED_TIMEOUT_MS = 3000;
    
    // Target window and acceptable bounds (µs)
    const int64_t TARGET_US = 50000;   // 50 ms
    const int64_t MIN_US    = 45000;   // -5 ms guard
    const int64_t MAX_US    = 55000;   // +5 ms guard
    
    int             _in1, _in2, _in3, _in4;
    gpio_num_t      _pins[2][2];
    bool            directionFlag;

    // dir_map[ direction ][ channel ] → 0=low / 1=high
    //   STOP    = {0,0}
    //   FORWARD = {0,1}
    //   BACK    = {1,0}
    static constexpr uint8_t dir_map[3][2] = {
        { 0, 0 },   // STOP
        { 0, 1 },   // FORWARD
        { 1, 0 }    // BACKWARD
    };

    enum { MOTOR_A = 0, MOTOR_B = 1 };
    enum { STOP = 0, FORWARD = 1, BACKWARD = 2 };
};


#endif // _ESP32_L298N_H_