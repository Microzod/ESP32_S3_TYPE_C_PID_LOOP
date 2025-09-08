#ifndef ENCODER_ENCODER_PCNT_H
#define ENCODER_ENCODER_PCNT_H

#include <functional>
#include <algorithm>

extern "C"
{
	#include "driver/gpio.h"
	#include "driver/pulse_cnt.h"
}

#define START_POS_DEFAULT 0
#define GLITCH_NS_DEFAULT 1000

class encoderEncoderPCNT {
  public:
    encoderEncoderPCNT(int a, int b, int enc1, int enc2, int start_pos, uint16_t glitch_ns);
    encoderEncoderPCNT(int a, int b, int enc1, int enc2, int start_pos);
    encoderEncoderPCNT(int a, int b, int enc1, int enc2 );
    encoderEncoderPCNT();
    ~encoderEncoderPCNT();
    
	void init();
    void deinit();
    int  position();
    void setPosition(int pos);
    void zero();
    uint8_t  pin_a = 255;
    uint8_t  pin_b = 255;
	uint8_t  pulsePinA = 255;
	uint8_t  pulsePinB = 255;
    uint16_t glitch_time = GLITCH_NS_DEFAULT;
	
	pcnt_unit_handle_t    	pcntA;
	pcnt_unit_handle_t    	pcntB;
	int ticksPerRevolution = 56;  // default; can be changed at runtime

	
	void clearQuadrature();
	void clearA();
	void clearB();
	void clearA_and_B();
	void clearTarget(pcnt_unit_handle_t aUnit);
	
	int getCount(bool resetAfterRead = false);
	int getCountA(bool resetAfterRead = false);
	int getCountB(bool resetAfterRead = false);
	int getCountFrom(pcnt_unit_handle_t aUnit, bool resetAfterRead = false);
	
	void setTicksPerRevolution(int ticks);
	int updateQuadraturePosition();
	
	// Attach external motor direction functions (spool/unspool/flag/resolution)
	// --- Using std::function:
    void attachLambdaFunctions(
		std::function<void()> spoolLambdaFunc,
		std::function<void()> unspoolLambdaFunc,
		std::function<bool()> getFlagLambdaFunc,
		std::function<uint32_t()> getResoLambdaFunc);


	bool functionAttached() const { return functionAttachedFlag; }
	
	// Attach external motor functions (spool/unspool/flag/resolution)
	// --- Using function pointers:
	void attachFunctionPointers(
        void 		(*spoolFuncPtr)(),
        void 		(*unspoolFuncPtr)(),
        bool 		(*getFlagFuncPtr)(),
		uint32_t 	(*getResoFuncPtr)()
	);
	
	// Wrapper Functions for Safer Usage:
	void 		setSpoolDirection();
	void 		setUnspoolDirection();
	bool 		getDirectionFlag() const;
	uint32_t 	getResolution() const;
	
	

	
  private:
	
	inline int signum(int32_t x) { return (x > 0) - (x < 0); }
	
    pcnt_unit_handle_t 		unit;
    pcnt_channel_handle_t 	chan_a;
    pcnt_channel_handle_t 	chan_b;
	pcnt_channel_handle_t 	_chanA;
	pcnt_channel_handle_t 	_chanB;
	bool                    _started;
    int16_t low_limit 	  = INT16_MIN;
    int16_t high_limit    = INT16_MAX;
    int count             = 0;
    int offset            = START_POS_DEFAULT;
	
	int encoderPosition;
	int oldPosition;
	int delta;
	int deltaDirection;
	int positionSignValue;
	int absPosition;

	static constexpr int        MOTOR_SET_DIR_AT  = 5;
	static constexpr int        INNER_DEAD_BAND   = 10;    // ±10 ticks → truly zero
	static constexpr int        KICK_DEAD_BAND    = 260;   // 11…260 → jump to 260
	static constexpr int        KICK_START_DUTYCYCLE = 1024;
	static constexpr int        AFTER_KICK_DUTYCYCLE = 260;
	
	// --- Optional motor direction handlers ---
	// ---       Using std::function:        ---
    std::function<void()> setSpoolDirectionFunc   = nullptr;
    std::function<void()> setUnspoolDirectionFunc = nullptr;
    std::function<bool()> getDirectionFlagFunc 	  = nullptr;
	std::function<uint32_t()> getResolutionFunc   = nullptr;
	bool functionAttachedFlag = false;
	
	// --- Optional motor direction handlers ---
	// ---     Using function pointers:      ---
	void (*setSpoolDirectionFuncPtr)() 	 = nullptr;
    void (*setUnspoolDirectionFuncPtr)() = nullptr;
    bool (*getDirectionFlagFuncPtr)() 	 = nullptr;
	uint32_t (*getResolutionFuncPtr)() 	 = nullptr;
	bool functionPointerAttachedFlag = false;
};
#endif