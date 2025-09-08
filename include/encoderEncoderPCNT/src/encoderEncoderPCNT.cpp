#include "encoderEncoderPCNT.h"

encoderEncoderPCNT::encoderEncoderPCNT(int a, int b, int enc1, int enc2, int start_pos, uint16_t glitch_ns)
{
	glitch_time = glitch_ns;
	offset = start_pos;
	pulsePinA = enc1;
	pulsePinB = enc2;
	pin_a = a;
	pin_b = b;
	init();
}

encoderEncoderPCNT::encoderEncoderPCNT(int a, int b, int enc1, int enc2, int start_pos)
{
	offset = start_pos;
	pulsePinA = enc1;
	pulsePinB = enc2;
	pin_a = a;
	pin_b = b;
	init();
}

encoderEncoderPCNT::encoderEncoderPCNT(int a, int b, int enc1, int enc2 )
{
	pulsePinA = enc1;
	pulsePinB = enc2;
	pin_a = a;
	pin_b = b;
	init();
}

encoderEncoderPCNT::encoderEncoderPCNT()
{
	//init();
}

encoderEncoderPCNT::~encoderEncoderPCNT()
{
  deinit();
}

void encoderEncoderPCNT::init()
{
	encoderPosition = 0;
	oldPosition = 0;
	delta = 0;
	deltaDirection = 0;
	positionSignValue = 0;
	absPosition = 0;
	
	
	// Unit config - Quadrature Encoder:
	pcnt_unit_config_t unit_config = {};
	unit_config.low_limit = low_limit;
	unit_config.high_limit = high_limit;
	unit_config.flags.accum_count = false;
	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));
  
    // Unit config - Motor Encoder:
    pcnt_unit_config_t ucfg1 = {};
	ucfg1.low_limit = low_limit;
	ucfg1.high_limit = high_limit;
	ucfg1.flags.accum_count = true;
	ESP_ERROR_CHECK(pcnt_new_unit(&ucfg1, &pcntA));
	
	// Unit config - Motor Encoder:
	pcnt_unit_config_t ucfg2 = {};
	ucfg2.low_limit = low_limit;
	ucfg2.high_limit = high_limit;
	ucfg2.flags.accum_count = true;
	ESP_ERROR_CHECK(pcnt_new_unit(&ucfg2, &pcntB));
	// Create unit
	
	// Quadrature Encoder, Set watch points at low and high limits to auto-accumulate overflows:
	pcnt_unit_add_watch_point(unit, low_limit);
	pcnt_unit_add_watch_point(unit, high_limit);
	
	// Quadrature Encoder, Glitch filter setup:
	pcnt_glitch_filter_config_t filter_config = {};
	filter_config.max_glitch_ns = glitch_time;
	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_config));
	
	// Motor Encoder, Glitch filter setup:
	pcnt_glitch_filter_config_t filter_cfg = {};
	filter_cfg.max_glitch_ns = glitch_time;
	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcntA, &filter_cfg));
	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcntB, &filter_cfg));
	
	// Quadrature Encoder, Channel A setup:
	pcnt_chan_config_t chan_a_config = {};
	chan_a_config.edge_gpio_num = pin_a;
	chan_a_config.level_gpio_num = pin_b;
	ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_a_config, &chan_a));
	
	// Quadrature Encoder, Channel B setup:
	pcnt_chan_config_t chan_b_config = {};
	chan_b_config.edge_gpio_num = pin_b;
	chan_b_config.level_gpio_num = pin_a;
	ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_b_config, &chan_b));
	
	// Motor Encoder, Channel A setup:
	pcnt_chan_config_t chan_a_cfg = {};
	chan_a_cfg.edge_gpio_num = pulsePinA;
	chan_a_cfg.level_gpio_num = (gpio_num_t)-1;
	ESP_ERROR_CHECK(pcnt_new_channel(pcntA, &chan_a_cfg, &_chanA));
	
	// Motor Encoder, Channel B setup:
	pcnt_chan_config_t chan_b_cfg = {};
	chan_b_cfg.edge_gpio_num = pulsePinB;
	chan_b_cfg.level_gpio_num = (gpio_num_t)-1;
	ESP_ERROR_CHECK(pcnt_new_channel(pcntB, &chan_b_cfg, &_chanB));
	
	// Quadrature Encoder, Set edge and level actions for both channels:
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
	
	// Motor Encoder, Set edge and level actions for both channels:
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(_chanA, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(_chanA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(_chanB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(_chanB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
	
	// Quadrature & Motor Encoder, Enable, clear and start the PCNT unit:
	ESP_ERROR_CHECK(pcnt_unit_enable(unit));
	ESP_ERROR_CHECK(pcnt_unit_enable(pcntA));
	ESP_ERROR_CHECK(pcnt_unit_enable(pcntB));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcntA));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcntB));
	ESP_ERROR_CHECK(pcnt_unit_start(unit));
	ESP_ERROR_CHECK(pcnt_unit_start(pcntA));
	ESP_ERROR_CHECK(pcnt_unit_start(pcntB));
	
	_started = true;
	
}

void encoderEncoderPCNT::deinit()
{
	// Free PCNT resources when destroyed.
	pcnt_unit_disable(unit);
	pcnt_del_channel(chan_a);
	pcnt_del_channel(chan_b);
	pcnt_del_unit(unit);
  
	if(_started)
	{
		pcnt_unit_disable(pcntA);
		pcnt_del_channel(_chanA);
		pcnt_del_unit(pcntA);
  
		pcnt_unit_disable(pcntB);
		pcnt_del_channel(_chanB);
		pcnt_del_unit(pcntB);
  
		_started = false;
	}
}

int encoderEncoderPCNT::position()
{
	pcnt_unit_get_count(unit, &count);
	return (count + offset);
}

void encoderEncoderPCNT::setPosition(int pos)
{
	offset = pos;
	pcnt_unit_get_count(unit, &count);
	zero();
}

void encoderEncoderPCNT::zero()
{
	pcnt_unit_clear_count(unit);
}

void encoderEncoderPCNT::clearQuadrature()
{
	pcnt_unit_clear_count(unit);
}

void encoderEncoderPCNT::clearA()
{
	pcnt_unit_clear_count(pcntA);
}

void encoderEncoderPCNT::clearB()
{
	pcnt_unit_clear_count(pcntB);
}

void encoderEncoderPCNT::clearA_and_B()
{
	pcnt_unit_clear_count(pcntA);
	pcnt_unit_clear_count(pcntB);
}

void encoderEncoderPCNT::clearTarget(pcnt_unit_handle_t aUnit)
{
	pcnt_unit_clear_count(aUnit);
}

int encoderEncoderPCNT::getCount(bool resetAfterRead)
{	
	int cnt = 0;
	
	pcnt_unit_get_count(unit, &cnt);
	
	if(resetAfterRead)
		pcnt_unit_clear_count(unit);
	
	return (cnt + offset);
}

int encoderEncoderPCNT::getCountA(bool resetAfterRead)
{
	if(!_started) return 0;
	
	int cnt = 0;
	
	pcnt_unit_get_count(pcntA, &cnt);
	//_totalA += cnt;
	
	if(resetAfterRead)
		pcnt_unit_clear_count(pcntA);
	
	return cnt;
}

int encoderEncoderPCNT::getCountB(bool resetAfterRead)
{
	if(!_started) return 0;
	
	int cnt = 0;
	
	pcnt_unit_get_count(pcntB, &cnt);
	//_totalB += cnt;
	
	if(resetAfterRead)
		pcnt_unit_clear_count(pcntB);
	
	return cnt;
}

int encoderEncoderPCNT::getCountFrom(pcnt_unit_handle_t aUnit, bool resetAfterRead)
{
	if(!_started) return 0;
	
	int cnt = 0;
	
	pcnt_unit_get_count(aUnit, &cnt);
	//_totalB += cnt;
	
	if(resetAfterRead)
		pcnt_unit_clear_count(aUnit);
	
	return cnt;
}

void encoderEncoderPCNT::setTicksPerRevolution(int ticks)
{
    ticksPerRevolution = ticks;
}

void encoderEncoderPCNT::attachLambdaFunctions(
        std::function<void()> spoolLambdaFunc,
        std::function<void()> unspoolLambdaFunc,
        std::function<bool()> getFlagLambdaFunc,
		std::function<uint32_t()> getResoLambdaFunc
)
{
	setSpoolDirectionFunc = spoolLambdaFunc;
	setUnspoolDirectionFunc = unspoolLambdaFunc;
	getDirectionFlagFunc = getFlagLambdaFunc;
	getResolutionFunc = getResoLambdaFunc;
	
	functionAttachedFlag = (spoolLambdaFunc && unspoolLambdaFunc && getFlagLambdaFunc && getResoLambdaFunc);
}

void encoderEncoderPCNT::attachFunctionPointers(
        void (*spoolFuncPtr)(),
        void (*unspoolFuncPtr)(),
        bool (*getFlagFuncPtr)(),
		uint32_t (*getResoFuncPtr)()
)
{
	setSpoolDirectionFuncPtr = spoolFuncPtr;
    setUnspoolDirectionFuncPtr = unspoolFuncPtr;
    getDirectionFlagFuncPtr = getFlagFuncPtr;
	getResolutionFuncPtr = getResoFuncPtr;
	
	functionPointerAttachedFlag = (spoolFuncPtr && unspoolFuncPtr && getFlagFuncPtr && getResoFuncPtr);
}

int encoderEncoderPCNT::updateQuadraturePosition()
{
	encoderPosition = this->position();
	if (encoderPosition != oldPosition)
	{
		delta = encoderPosition - oldPosition;
		deltaDirection = signum(delta);
		positionSignValue = signum(encoderPosition);
		absPosition = abs(encoderPosition);
	
		// Direction control using deadband window
		if (encoderPosition < -MOTOR_SET_DIR_AT && encoderPosition > -INNER_DEAD_BAND)
		{
			if (getDirectionFlag())
			{
				setUnspoolDirection();
				//printf("set to Unspool direction \n");
			}
		}
		else if (encoderPosition > MOTOR_SET_DIR_AT && encoderPosition < INNER_DEAD_BAND)
		{
			if (!getDirectionFlag())
			{
				setSpoolDirection();
				//printf("set to Spool direction \n");
			}
		}
	
		// Deadband enforcement to allow re-entry
		if (encoderPosition < oldPosition) // encoder rotating counter-clockwise
		{
			if (encoderPosition < KICK_DEAD_BAND && encoderPosition > 0 && encoderPosition > INNER_DEAD_BAND)
				this->setPosition(INNER_DEAD_BAND);
	
			if (encoderPosition < -INNER_DEAD_BAND && encoderPosition > -KICK_DEAD_BAND)
				this->setPosition(-KICK_DEAD_BAND);
		}
		else if (encoderPosition > oldPosition) // encoder rotating clockwise
		{
			if (encoderPosition > INNER_DEAD_BAND && encoderPosition < KICK_DEAD_BAND)
                this->setPosition(KICK_DEAD_BAND);
			
            if (encoderPosition > -KICK_DEAD_BAND && encoderPosition < 0 && encoderPosition < -INNER_DEAD_BAND)
                this->setPosition(-INNER_DEAD_BAND);
		}
	
		// Clamp encoder encoderPosition to avoid overflow
		if (absPosition > getResolution())
		{
			encoderPosition = positionSignValue * getResolution();
			this->setPosition(encoderPosition);
		}
	
		//setpoint = encoderPosition;
		oldPosition = encoderPosition;
	}
	return encoderPosition;
}

void encoderEncoderPCNT::setSpoolDirection()
{
    if (setSpoolDirectionFunc)
        setSpoolDirectionFunc();
}

void encoderEncoderPCNT::setUnspoolDirection()
{
    if (setUnspoolDirectionFunc)
        setUnspoolDirectionFunc();
}

bool encoderEncoderPCNT::getDirectionFlag() const
{
    return getDirectionFlagFunc ? getDirectionFlagFunc() : false;
}

uint32_t encoderEncoderPCNT::getResolution() const
{
    return getResolutionFunc ? getResolutionFunc() : false;
}
