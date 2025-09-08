#include <MCPWM_DUAL_PWM.h>

extern "C"
{
    #include "driver/mcpwm_prelude.h"
}

static const int        pwm_pin_A                   = 6;
static const int        pwm_pin_B                   = 7;
static const uint32_t   pwm_hertz                   = 21000;
static const uint8_t    pwm_number_of_bits          = 10;
static const int        pwm_max_counts              = (1u << pwm_number_of_bits);
static const int        pwm_min_counts              = (-pwm_max_counts);


MCPWM_DUAL_PWM pwm(pwm_pin_A, pwm_pin_B, pwm_hertz, pwm_number_of_bits);


void setup()
{
    pwm.initPWM(0);

    pwm.setDutyCycle(pwm.cmprA, 0);	// the first channel is accessed with pwm.cmprA
    pwm.setDutyCycle(pwm.cmprB, 0);	// the second channel is accessed with pwm.cmprB
}

void loop()
{
	
}