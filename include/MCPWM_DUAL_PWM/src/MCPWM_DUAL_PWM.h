#ifndef _MCPWM_DUAL_PWM_H_
#define _MCPWM_DUAL_PWM_H_

extern "C"
{
	#include "driver/gpio.h"
    #include "driver/mcpwm_prelude.h"
}

class MCPWM_DUAL_PWM
{
	
public:
	MCPWM_DUAL_PWM(int pwmPinA = 17, int pwmPinB = 18, uint32_t pwm_freq = 21000, uint8_t pwm_resolution = 10);
	
	void initPWM(int startAtDutyCycle = 0);
    
    //int32_t getEncCountA(bool resetAfterRead = false);
    //int32_t getEncCountB(bool resetAfterRead = false);

    void setDutyCycle(mcpwm_cmpr_handle_t cmpr, uint32_t level);
    void setDeadTime(uint32_t red_ticks, uint32_t fed_ticks);
	
	uint32_t getResolution();
	
	//mcpwm_cmpr_handle_t getCmprHandles();
    // This is a static “thunk” that the ISR subsystem can call.
    //static void IRAM_ATTR _onEncA(void* ctx);
    //static void IRAM_ATTR _onEncB(void* ctx);

    // Member functions that actually get called from _onEncA/_onEncB:
    //void handleEncA();
    //void handleEncB();
	
    mcpwm_cmpr_handle_t  cmprA, cmprB;

private:
    void setupChannel(int gpio_num, mcpwm_cmpr_handle_t &cmpr, mcpwm_gen_handle_t &gen);
    
	gpio_num_t  _pwm_pin_A;		// = 17;
	gpio_num_t  _pwm_pin_B;		// = 18;
	uint32_t 	_pwm_freq;			// = 5000;
	uint8_t  	_pwm_bit_depth;	// = 10;
	uint32_t 	_resolution;		// = (1u << RES_BITS);
	uint32_t 	_pwm_period_ticks; // = RESOLUTION;
	
	uint32_t _deadTimerRedTicks      = 0;
	uint32_t _deadTimerFedTicks      = 0;

	mcpwm_timer_handle_t _pwm_timer;
	mcpwm_oper_handle_t  _pwm_oper;
    mcpwm_gen_handle_t   _genA, _genB;
	
	//int    				_enc_pin_A;//     = 15;
	//int    				_enc_pin_B;//     = 16;
	//volatile int32_t	_encCountA       = 0;
	//volatile int32_t	_encCountB       = 0;
	//volatile uint32_t	_lastEncATime    = 0;
	//volatile uint32_t	_lastEncBTime    = 0;
	
};

#endif  // _MCPWM_DUAL_PWM_H_