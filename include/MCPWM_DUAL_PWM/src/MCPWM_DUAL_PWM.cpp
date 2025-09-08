#include <MCPWM_DUAL_PWM.h>


MCPWM_DUAL_PWM::MCPWM_DUAL_PWM(int pwmPinA, int pwmPinB, uint32_t pwm_freq, uint8_t pwm_resolution)
{
    _pwm_pin_A = static_cast<gpio_num_t>(pwmPinA);		
    _pwm_pin_B = static_cast<gpio_num_t>(pwmPinB);	
    //_enc_pin_A = encPinA;
    //_enc_pin_B = encPinB; 
    _pwm_freq = pwm_freq;			
    _pwm_bit_depth = pwm_resolution;
    _resolution = (1u << _pwm_bit_depth);		
    _pwm_period_ticks = _resolution;
    
}

void MCPWM_DUAL_PWM::initPWM(int startAtDutyCycle)
{
    // 1) MCPWM Timer
    mcpwm_timer_config_t _tcfg = {};
    _tcfg.group_id      = 0;
    _tcfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
    _tcfg.resolution_hz = _pwm_freq * _resolution;
    _tcfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
    _tcfg.period_ticks  = _pwm_period_ticks;
    ESP_ERROR_CHECK(mcpwm_new_timer(&_tcfg, &_pwm_timer));

    // 2) Operator
    mcpwm_operator_config_t _ocfg = {};
    _ocfg.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&_ocfg, &_pwm_oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(_pwm_oper, _pwm_timer));

    // 3) PWM Channels
    setupChannel(_pwm_pin_A, cmprA, _genA);
    setupChannel(_pwm_pin_B, cmprB, _genB);

    // 4) Start timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(_pwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop( _pwm_timer, MCPWM_TIMER_START_NO_STOP ));


    // 7) Initial duties (0%)
    setDutyCycle(cmprA, startAtDutyCycle);
    setDutyCycle(cmprB, startAtDutyCycle);
	
}

void MCPWM_DUAL_PWM::setupChannel(int gpio_num, mcpwm_cmpr_handle_t &cmpr, mcpwm_gen_handle_t &gen)
{
    mcpwm_comparator_config_t _ccfg = {};
    //_ccfg.intr_priority = 0;
    _ccfg.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(_pwm_oper, &_ccfg, &cmpr));
    
    mcpwm_generator_config_t _gcfg = {};
    _gcfg.gen_gpio_num = gpio_num;
    ESP_ERROR_CHECK(mcpwm_new_generator(_pwm_oper, &_gcfg, &gen));


    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH
        )
    ));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            cmpr,
            MCPWM_GEN_ACTION_LOW
        )
    ));
}

void MCPWM_DUAL_PWM::setDutyCycle(mcpwm_cmpr_handle_t cmpr, uint32_t level)
{
    if (level > _resolution)
    {
        level = _resolution;
    }
    ESP_ERROR_CHECK( mcpwm_comparator_set_compare_value(cmpr, level) );
}

void MCPWM_DUAL_PWM::setDeadTime(uint32_t red_ticks, uint32_t fed_ticks)
{
    // clamp into a sane range
    //red_ticks = min(red_ticks, _pwm_period_ticks / 2);
    //fed_ticks = min(fed_ticks, _pwm_period_ticks / 2);
    red_ticks = (red_ticks < _pwm_period_ticks/2) ? red_ticks : (_pwm_period_ticks/2);
    fed_ticks = (fed_ticks < _pwm_period_ticks/2) ? fed_ticks : (_pwm_period_ticks/2);

    // 1) Rising-edge dead-time: delay the turn-on of the low-side (_genB)
    //    until 'red_ticks' after the high-side (_genA) has switched off.
    mcpwm_dead_time_config_t dt_cfg = {};
    dt_cfg.posedge_delay_ticks = red_ticks;
    dt_cfg.negedge_delay_ticks = 0;
    dt_cfg.flags.invert_output = false;
    // leading = _genA (high-side), trailing = _genB (low-side)
    ESP_ERROR_CHECK(    // leading = _genA (high-side), trailing = _genB (low-side)
        mcpwm_generator_set_dead_time(_genA, _genB, &dt_cfg)
    );

    // 2) Falling-edge dead-time: delay the turn-on of the high-side (_genA)
    //    until 'fed_ticks' after the low-side (_genB) has switched off.
    dt_cfg.posedge_delay_ticks = 0;
    dt_cfg.negedge_delay_ticks = fed_ticks;
    dt_cfg.flags.invert_output = false;  // still driving the true output
    // leading = _genB (low-side), trailing = _genA (high-side)
    ESP_ERROR_CHECK(
        mcpwm_generator_set_dead_time(_genB, _genA, &dt_cfg)
    );

    // save for runtime inspection if you like
    _deadTimerRedTicks = red_ticks;
    _deadTimerFedTicks = fed_ticks;
}

uint32_t MCPWM_DUAL_PWM::getResolution()
{
    return _resolution;
}


/*
mcpwm_timer_config_t timer_config = {};
timer_config.group_id = 0;                      // Specify from which group to allocate the MCPWM timer
timer_config.clk_src = 0;                       // MCPWM timer clock source
timer_config.resolution_hz = 0;                 // Counter resolution in Hz, The step size of each count tick equals to (1 / resolution_hz) seconds
timer_config.count_mode = 0;                    // Count mode
timer_config.period_ticks = 0;                  // Number of count ticks within a period
timer_config.intr_priority = 0;                 // MCPWM timer interrupt priority, if set to 0, the driver will try to allocate an interrupt with a relative low priority (1,2,3)
timer_config.flags.update_period_on_empty = 0;  // Whether to update period when timer counts to zero
timer_config.flags.update_period_on_sync = 0;   // Whether to update period on sync event
timer_config.flags.allow_pd = 0;                // Set to allow power down. When this flag set, the driver will backup/restore the MCPWM registers before/after entering/exist sleep mode.

mcpwm_operator_config_t operator_config = {};
operator_config.group_id = 0;                           // Specify from which group to allocate the MCPWM operator
operator_config.intr_priority = 0;                      // MCPWM operator interrupt priority,
operator_config.flags.update_gen_action_on_tez = 0;     // Whether to update generator action when timer counts to zero
operator_config.flags.update_gen_action_on_tep = 0;     // Whether to update generator action when timer counts to peak
operator_config.flags.update_gen_action_on_sync = 0;    // Whether to update generator action on sync event
operator_config.flags.update_dead_time_on_tez = 0;      // Whether to update dead time when timer counts to zero
operator_config.flags.update_dead_time_on_tep = 0;      // Whether to update dead time when timer counts to peak
operator_config.flags.update_dead_time_on_sync = 0;     // Whether to update dead time on sync event

mcpwm_comparator_config_t compr_config = {};
compr_config.intr_priority = 0;             // MCPWM comparator interrupt priority,
compr_config.flags.update_cmp_on_tez = 0;   // Whether to update compare value when timer count equals to zero (tez)
compr_config.flags.update_cmp_on_tep = 0;   // Whether to update compare value when timer count equals to peak (tep)
compr_config.flags.update_cmp_on_sync = 0;  // Whether to update compare value on sync event

mcpwm_generator_config_t gen_config = {};
gen_config.gen_gpio_num = 0;        // The GPIO number used to output the PWM signal
gen_config.flags.invert_pwm = 0;    // Whether to invert the PWM signal (done by GPIO matrix)
gen_config.flags.io_loop_back = 0;  // For debug/test, the signal output from the GPIO will be fed to the input path as well
gen_config.flags.io_od_mode = 0;    // Configure the GPIO as open-drain mode
gen_config.flags.pull_up = 0;       // Whether to pull up internally
gen_config.flags.pull_down = 0;     // Whether to pull down internally
================================== For posterity, class member in c interrupt: ============================
// static ISR wrappers:
void IRAM_ATTR MCPWM_DUAL_PWM::_onEncA(void* ctx)
{
    auto self = reinterpret_cast<MCPWM_DUAL_PWM*>(ctx);
    self->handleEncA();
}

void IRAM_ATTR MCPWM_DUAL_PWM::_onEncB(void* ctx)
{
    auto self = reinterpret_cast<MCPWM_DUAL_PWM*>(ctx);
    self->handleEncB();
}

// actual member functions:
void MCPWM_DUAL_PWM::handleEncA()
{
    // note: _encCountA is volatile
    _encCountA++;
}

void MCPWM_DUAL_PWM::handleEncB()
{
    _encCountB++;
}

int32_t MCPWM_DUAL_PWM::getEncCountA(bool resetAfterRead)
{
    noInterrupts();
    int32_t c = _encCountA;
    if (resetAfterRead)
    {
        _encCountA = 0;
    }
    interrupts();
    return c;
}

int32_t MCPWM_DUAL_PWM::getEncCountB(bool resetAfterRead)
{
    noInterrupts();
    int32_t c = _encCountB;
    if (resetAfterRead)
    {
        _encCountB = 0;
    }
    interrupts();
    return c;
}
*/
