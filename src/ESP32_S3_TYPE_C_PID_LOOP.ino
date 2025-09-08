

//possible replacements:
//pid.configure(settings.pid);

#include "IntervalTimer.h"
#include <LittleFS.h>
#include <inttypes.h>
#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"
#include "esp_system.h"
#include "esp_timer.h"
#include <cmath>  // for roundf()
#include <cfloat>  // or <float.h>

#include <Bounce2.h>
#include <encoderEncoderPCNT.h>
#include <MCPWM_DUAL_PWM.h>
#include <ESP32_L298N.h>
#include <GptimerBeat.h>


#include "ErrorMetrics.h"
#include "MeasurementReporter.h"
#include "smarter_new_nvs_stuff.h"
#include "motorLUT.h"
#include "motorLUT_helpers.h"
#include "nvsSettings_definition.h"
#include "collectLUT_rampDownUp.h"
#include "LutDataStruct.h"
#include "MotorBSyncController.h"
#include "DynamicLUTBlender.h"  // Header-only version
#include "TypeC_PIDController.h"
#include "ControlTask.h"
#include "UartRouter.h"
#include "UartConsole.h"
#include "SettingsNVS.h"
#include "esp_err.h"  // for esp_err_to_name()

extern "C"
{
    #include "driver/gpio.h"
    #include "esp_err.h"
}

// ===== L298 Control Pins =====
static const int IN1_pin = 5;
static const int IN2_pin = 4;

static const int IN3_pin = 2;
static const int IN4_pin = 1;

// ===== quadrature encoder & Button =====
static const int        quadrature_encoder_pin_A    = 10;
static const int        quadrature_encoder_pin_B    = 9;
static const int        quadrature_button_pin       = 8;
static const int        motor_encoder_pin_A         = 15;
static const int        motor_encoder_pin_B         = 16;
static const int        rgb_pin                     = 21;

static const int        pwm_pin_A                   = 6;
static const int        pwm_pin_B                   = 7;
static const uint32_t   pwm_hertz                   = 5000;
static const uint8_t    pwm_number_of_bits          = 10;
static const int        pwm_max_counts              = (1u << pwm_number_of_bits);
static const int        pwm_min_counts              = (-pwm_max_counts);


encoderEncoderPCNT encoder(quadrature_encoder_pin_A, quadrature_encoder_pin_B, motor_encoder_pin_A, motor_encoder_pin_B);
//MCPWM_DUAL_PWM pwm(pwm_pin_A, pwm_pin_B, pwm_hertz, pwm_number_of_bits);
ESP32_L298N l298n(pwm_pin_A, pwm_pin_B, pwm_hertz, pwm_number_of_bits, IN1_pin, IN2_pin, IN3_pin, IN4_pin, encoder);
Bounce2::Button button = Bounce2::Button();
//RuntimeIntervalTimer controlLoopPeriod(100);  // 100 ms
controlSettings settings;
MotorBSyncController motorB_pid;
//IntervalTimer pidTimer(50, false);
GptimerBeat beat;


extern const int LUT_SIZE;
extern const float motorB_ticksPer50ms_fast_LUT[];
extern const float motorB_ticksPer50ms_slow_LUT[];




// ─────────────── tunable dead-band & PI params ────────────────
constexpr float MIN_RPS = 3.0f;
constexpr float MAX_RPS = 20.0f;
constexpr float RPS_PER_ENCODER_STEP = 0.05f;
constexpr float sampleIntervalSec = 0.1f; // 100ms loop
const int       CPR = 56; // 28.0f * 2.0f

float Kp = 5.0f;
float Ki = 0.1f;
float Kd = 0.00f;
float LPF_ALPHA = 0.2f;

float setpointRPS = 0.0f;
float measuredRPS = 0.0f;
float integral_A = 0.0f;
float lastMeasurement_A = 0.0f;

// ──────────────── state for PI loop ────────────────
static float            pi_integral     = 0;
static float            total_integral  = 0;
static unsigned long    lastPI          = 0;
static float            scaleA;
static float            scaleB;
static float            vA;
static float            vA_full;
static float            vB_full;
const uint32_t          Q = 1000000;  
static float            error = 0.0f;
static float            lastTwist     = 0.0f;
static float            twistRateRps;
static unsigned long    lastTwistTime = 0;
static unsigned long    restart_counter;
static bool             setReset = false;
static int              pid_cycles_to_run = 10;
static unsigned long    pid_run_duration_ms = 5000;
static unsigned long    pid_off_duration_ms = 5000;
static int              pid_pulse_dutycycle_low = 750;
static int              pid_pulse_dutycycle_high = 1000;
static int              pulsePeriods = 10;
static bool             buttonUpdate = true;
static bool             startPlotting = false;
static bool             startPulsing = false;
static bool             testMotorB = false;
static bool             testMotorA = false;
static bool             printMotorBResults = false;
static bool             printMotorAResults = false;
static bool             runSaveResults = false;
static bool             runLoadResults = false;
static bool             pid_on_off_control = false;
static bool             quadrature_encoder_on_off_control = true;

bool    directionFlag;
int     position;
int     positionA;
int     newPosition;
int     oldPosition = 0;
int     absPosition;



static const int JUMP_THRESH   = 10;
static const int JUMP_TO       = 1100;
static const int MAX_POS       = 2048;
static const int BOTTOM_LIMIT  = 200;

int currentPosition = 0;  // your software‐tracked position

inline float totalTwist() { return total_integral / CPR; }
//–– signum helper ––
inline int signum(int32_t x) { return (x > 0) - (x < 0); }
int     delta = 0;              // the most
int     delta_total = 0;
int     deltaDirection;  // will be +1 if the encoder just moved “up” (delta > 0), –1 if it moved “down” (delta < 0), or 0 if it didn’t change.
int     positionSignValue;     // will be +1 if position is above zero, –1 if below, or 0 if you happen to be exactly at 0.
int     oldPositionSignValue;
int     absolute;



// —— Functions prototypes: ——
void updateMotorB_toFollowMotorA();
void globalPrintKickFailTime();
void globalPrintLutsAsFloatArray();
void kickStart( mcpwm_cmpr_handle_t cmprA,
                pcnt_unit_handle_t encA,
                int targetDutycycleA,
                mcpwm_cmpr_handle_t cmprB,
                pcnt_unit_handle_t encB,
                int targetDutycycleB);
void updateIPD_MotorA(float sampleIntervalSec);
void serialTalk();
void serialRespons();
void printSerialCommands();
void pidPulseResponse(int &count);
void saveResults();
void loadResults();
void simpleKickStart(int motor, int kickStartDuty, int afterKickDuty, int errorStopTimeMS);
int remapPosition(int pos, int oldPos);
void calibrateSpeed();

int dutycycleList[736] = {};
float upLUT[736] = {};
float downLUT[736] = {};
float avgLUT[736] = {};

float monotonic[691] = {};

float ticksPer50msA[40] = {};
float ticksPer50msB[40] = {};


void spoolDir()      { l298n.setSpoolDirection(); }
void unspoolDir()    { l298n.setUnspoolDirection(); }
bool getDirFlag()    { return l298n.getDirectionFlag(); }
uint32_t getResVal()    { return l298n.pwm.getResolution(); }

size_t N = 736;
/*
void setup()
{
    delay(1000);
    printf("=== Feature Test Begin ===");

    // Init L298N + PWM
    if (l298n.init() == ESP_OK)
        printf("L298N init OK");
    else
        printf("L298N init FAIL");

    // Init encoder
    encoder.init();
    printf("Encoder init OK");

    // Test spool direction
    l298n.setSpoolDirection();
    printf("Set direction: Spool");
    delay(1000);

    // Test unspool direction
    l298n.setUnspoolDirection();
    printf("Set direction: Unspool");
    delay(1000);

    // Test duty cycle set (low speed)
    l298n.setDutyMotorA(200);
    l298n.setDutyMotorB(200);
    printf("Motors set to duty 200");
    delay(1500);

    // Stop motors
    l298n.stop();
    printf("Motors stopped");
    delay(1000);

    // Kickstart test (force very low duty so kick is required)
    printf("Kickstart test...");
    l298n.kickStart(encoder, 150, 150);
    delay(1000);

    // Read encoder positions
    int32_t posA = encoder.getCountA(true);
    int32_t posB = encoder.getCountB(true);
    printf("Encoder A pos: %ld, Encoder B pos: %ld\n", posA, posB);

    // LUT access test
    int dutyTest = 300;
    printf("LUT fast[B] @%d = %.2f ticks/50ms\n", dutyTest,
                  getTicksFromDuty_B_fast(dutyTest));
    printf("LUT slow[B] @%d = %.2f ticks/50ms\n", dutyTest,
                  getTicksFromDuty_B_slow(dutyTest));

    
    printf("Testing 50ms timer for 10 cycles...");
    int cycles = 0;
    while (cycles < 10)
    {
        if (pidTimer.ready())
        {
            Serial.printf("Timer tick #%d\n", ++cycles);
        }
    }

    printf("=== Feature Test End ===");

    for(int i = 0; i < LUT_SIZE; i++)
    {
        if(i == 690)
        {
            return;
        }
        else
        {
            monotonic[i] = motorB_ticksPer50ms_fast_LUT[i+1] - motorB_ticksPer50ms_fast_LUT[i];
        }
    }
}

*/


void setup()
{
    delay(5000);
    // --- Using std::function:
    encoder.attachLambdaFunctions(  []() { l298n.setSpoolDirection(); },
                                    []() { l298n.setUnspoolDirection(); },
                                    []() -> bool { return l298n.getDirectionFlag(); },
                                    []() -> uint32_t { return l298n.pwm.getResolution(); }
    );

    // --- Using function pointers:
    encoder.attachFunctionPointers(spoolDir, unspoolDir, getDirFlag, getResVal);
    encoder.setTicksPerRevolution(CPR);

    
    //NVSRestartCounting();
    Serial.begin(115200);
    // 1) L298N setup:
    l298n.init();

    // 3) Quadrature encoder & button setup:
    button.attach(quadrature_button_pin, INPUT_PULLDOWN);
    button.interval(5);
    button.setPressedState(HIGH);

    // 6) Establish correct direction
    l298n.setSpoolDirection();
    l298n.pwm.setDutyCycle(l298n.pwm.cmprA, 0);
    l298n.pwm.setDutyCycle(l298n.pwm.cmprB, 0);

    motorB_pid.configureFromSettings(settings, CPR);

    // Start control task sampling at 50 ms on Core 1
    if (!startControlTask(beat, 50000, 1))
    {
        printf("Failed to start control task\n");
    }

    int dValue = 1024 - N;
    for (int i = 0; i < N; ++i)
    {
        dutycycleList[i] = dValue;
        dValue++;
    }
    
    while(!button.pressed())
    {
        button.update();
    }

    
    int dutycycle = 260;
    for(int i = 0; i < 39; i++)
    {
        l298n.kickStartA(dutycycle);
        delay(1000);

        //if (pidTimer.ready())   {}

    }

    /*
    kickStart(nullptr, nullptr, 0, l298n.pwm.cmprB, encoder.pcntB, l298n.pwm.getResolution());
    delay(5000);
    collectLUT_rampDownUp(  encoder.pcntB,
                            l298n.pwm.cmprB,
                            l298n.pwm,
                            encoder,
                            dutycycleList,
                            downLUT,
                            upLUT,
                            avgLUT,
                            N);
    
    */
    /*
    printf("[the kick itself][wind down to zero][full kick then detect zero]\n");
    for(int i = 0; i < 10; i++)
    {
        globalPrintKickFailTime();
    }
    */
    
    // Just Motor A 
    //kickStart(l298n.pwm.cmprA, encoder.pcntA, 250, nullptr, nullptr, 0);
    //kickStart(encoder, encoder.pcntA, 250, nullptr, nullptr, 0);
    // Just Motor B
    //kickStart(nullptr, nullptr, 0, l298n.pwm.cmprB, encoder.pcntB, 270);
    // Both motors
    //kickStart(l298n.pwm.cmprA, encoder.pcntA, 250, l298n.pwm.cmprB, encoder.pcntB, 270);
}

void loop()
{
    
    //serialTalk();
    //serialRespons();
    if(buttonUpdate)
    {
        button.update();

        if (button.pressed())
        {
            encoder.setPosition(0);
            pi_integral = 0.0f;
            lastPI      = millis();
            encoder.getCountA(true);
            encoder.getCountB(true);
            l298n.pwm.setDutyCycle(l298n.pwm.cmprA, 0);
            l298n.pwm.setDutyCycle(l298n.pwm.cmprB, 0);
            Serial.printf("== ZERO ==\n");
        }
    }
    
    if(quadrature_encoder_on_off_control)
    {
		position = encoder.updateQuadraturePosition();
        printf("[%d]\n", position);
        /*
        bool is_A_stalled = (encoder.getCountA(true) == 0) ? true : false;
        bool is_B_stalled = (encoder.getCountB(true) == 0) ? true : false;
        if(is_A_stalled && is_B_stalled)
        {
            l298n.kickStart(encoder, position, position);
        }
        else
        {
            if(is_A_stalled)
            {
                l298n.kickStart(encoder, position, -1);
            }
            if(is_B_stalled)
            {
                l298n.kickStart(encoder, -1, position);
            }
        }
        */
        l298n.pwm.setDutyCycle(l298n.pwm.cmprA, abs(position));
        l298n.pwm.setDutyCycle(l298n.pwm.cmprB, abs(position));
        //oldPosition = position;
        /*
        positionSignValue = signum(position);
        if(positionSignValue > 0 || position >= 5)
        {
            if(oldPositionSignValue < 0)
            {
                encoder.setSpoolDirection();
                printf("SWITCHED DIRECTION TO SPOOL( +position )\n");
            }
            oldPositionSignValue = positionSignValue;
        }
        else if(positionSignValue < 0 || position <= -5)
        {
            if(oldPositionSignValue > 0)
            {
                encoder.setUnspoolDirection();
                printf("SWITCHED DIRECTION TO UNSPOOL( -position )\n");
            }
            oldPositionSignValue = positionSignValue;
        }
        
        if(position != oldPosition)
        {
            printf("[%d]\n", position);
            l298n.pwm.setDutyCycle(l298n.pwm.cmprA, position);
            l298n.pwm.setDutyCycle(l298n.pwm.cmprB, position);
        }
        
        oldPosition = position;
        */
    }

    
    if(pid_on_off_control)
    {
        /*
        if (ipdUpdateTimer.shouldRun())
        {
            updateIPD_MotorA(sampleIntervalSec);
        }
        */
        encoder.clearTarget(encoder.pcntA);
        encoder.clearTarget(encoder.pcntB);
        vTaskDelay(pdMS_TO_TICKS(50));
        int A_ticks = encoder.getCountFrom(encoder.pcntA, true);
        int B_ticks = encoder.getCountFrom(encoder.pcntB, true);

        motorB_pid.update(A_ticks, B_ticks);
        float pid_output = motorB_pid.getOutput();

        // Apply feedforward + PID correction
        int ff_duty = dutycycleList[findClosestIndex(avgLUT, LUT_SIZE, A_ticks)];
        int corrected_duty = ff_duty + static_cast<int>(pid_output);

        corrected_duty = constrain(corrected_duty, 0, 1023);
        l298n.pwm.setDutyCycle(l298n.pwm.cmprB, corrected_duty);

        // Debug
        motorB_pid.printDebug();

    }  
}

void printMotorSyncError()
{
    int ticksA = encoder.getCountFrom(encoder.pcntA, true);
    int ticksB = encoder.getCountFrom(encoder.pcntB, true);
    int diff = ticksA - ticksB;
    printf("SyncErr: A=%d, B=%d → Diff=%d ticks\n", ticksA, ticksB, diff);
}


inline int findClosestIndex(const float* lut, size_t size, float target)
{
    float bestDiff = FLT_MAX;
    int bestIndex = -1;

    for (size_t i = 0; i < size; ++i)
    {
        float diff = fabsf(lut[i] - target);
        if (diff < bestDiff)
        {
            bestDiff = diff;
            bestIndex = i;
        }
    }

    return bestIndex;
}


void updateMotorB_toFollowMotorA()
{
    // 1. Measure Motor A’s actual speed in ticks/50ms
    encoder.clearTarget(encoder.pcntA);
    int64_t t0 = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(50));
    int64_t t1 = esp_timer_get_time();
    float dt = (t1 - t0) / 1e6f;  // seconds
    int ticks = encoder.getCountFrom(encoder.pcntA, true);
    int speed_A = ticks / dt; // ticks/sec

    // 2. Convert to ticks/50ms for LUT matching
    int speed_A_50ms = static_cast<int>(ticks);

    // 3. Find best matching index in avgLUT[]
    int index = findClosestIndex(avgLUT, N, speed_A_50ms);

    // 4. Get initial feedforward duty for Motor B
    int dutyB_feedforward = dutycycleList[index];
    l298n.pwm.setDutyCycle(l298n.pwm.cmprB, dutyB_feedforward);

    // 5. Optionally: start PID loop afterward or immediately
    printf("MotorA speed=%d ticks/50ms → Feedforward DutyB=%d (index=%d)\n",
           speed_A_50ms, dutyB_feedforward, index);
}

void globalPrintKickFailTime()
{
    encoder.clearB();    // Clear encoders
    int64_t t0 = esp_timer_get_time();
    l298n.pwm.setDutyCycle(l298n.pwm.cmprB, l298n.pwm.getResolution()); // Kick-start both
    while (encoder.getCountB(false) < 56)
    {
        __asm__ __volatile__("nop");   // Wait for movement
    }
    int64_t t1 = esp_timer_get_time();
    l298n.pwm.setDutyCycle(l298n.pwm.cmprB, 0);  // Apply target duty cycles
    while (encoder.getCountB(true) != 0)
    {
        __asm__ __volatile__("nop");
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    int64_t t2 = esp_timer_get_time();
    float dt_kicking = (t1 - t0) / 1e3f;
    float dt_afterKick = (t2 - t1) / 1e3f;
    float dt_full = (t2 - t0) / 1e3f;

    printf("[%f][%f][%f]\n", dt_kicking, dt_afterKick, dt_full);
}

void globalPrintLutsAsFloatArray()
{
    printf("\n\n\n");
    printf("const float rampDownLUT[] =\n{\n\t");
    for (int i = 0; i < N; ++i)
    {
        printf("%f,", downLUT[i]);
    }
    printf("\n\n\n");
    printf("const float rampUpLUT[] =\n{\n\t");
    for (int i = 0; i < N; ++i)
    {
        printf("%f,", upLUT[i]);
    }
    printf("\n\n\n");
    printf("const float averageLUT[] =\n{\n\t");
    for (int i = 0; i < N; ++i)
    {
        printf("%f,", avgLUT[i]);
    }
    printf("\n");
}

/*
void kickStart(
    mcpwm_cmpr_handle_t cmprA,
    pcnt_unit_handle_t encA,
    int targetDutycycleA,
    mcpwm_cmpr_handle_t cmprB,
    pcnt_unit_handle_t encB,
    int targetDutycycleB
)
{
    bool useA = cmprA != nullptr && encA != nullptr;
    bool useB = cmprB != nullptr && encB != nullptr;

    // Clear encoders
    if (useA) encoder.clearTarget(encA);
    if (useB) encoder.clearTarget(encB);

    // Kick-start both
    if (useA) l298n.pwm.setDutyCycle(cmprA, l298n.pwm.getResolution());
    if (useB) l298n.pwm.setDutyCycle(cmprB, l298n.pwm.getResolution());

    // Wait for movement
    if (useA) while (encoder.getCountFrom(encA, false) < 56) { __asm__ __volatile__("nop"); }
    if (useB) while (encoder.getCountFrom(encB, false) < 56) { __asm__ __volatile__("nop"); }

    // Apply target duty cycles
    if (useA) l298n.pwm.setDutyCycle(cmprA, targetDutycycleA);
    if (useB) l298n.pwm.setDutyCycle(cmprB, targetDutycycleB);

    // Wait for steady-state
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

    bool stableA = useA ? waitForSteady(encA) : true;
    bool stableB = useB ? waitForSteady(encB) : true;

    if (!stableA || !stableB)
    {
        printf("[WARN] Unstable kick-start: A:%d B:%d\n", stableA, stableB);
        if (useA) l298n.pwm.setDutyCycle(cmprA, 0);
        if (useB) l298n.pwm.setDutyCycle(cmprB, 0);
    }
    else
    {
        printf("[INFO] Kick-start complete → A:%d, B:%d\n", targetDutycycleA, targetDutycycleB);
    }
}
*/

/*
void collectLUT_rampDownUp(
    pcnt_unit_handle_t enc,
    mcpwm_cmpr_handle_t pwm,
    const int* dutyList,
    int* resultDownLUT,
    int* resultUpLUT,
    int* resultAvgLUT,
    size_t N)
{
    // Spin up to max
    printf("Spinning up to max duty...\n");
    l298n.pwm.setDutyCycle(pwm, dutyList[N - 1]);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Helper lambda to measure a 50ms speed
    auto measure50ms = [&](int index)
    {
        encoder.clearTarget(enc);
        int64_t t0 = esp_timer_get_time();
        vTaskDelay(pdMS_TO_TICKS(50));
        int64_t t1 = esp_timer_get_time();
        float dt = (t1 - t0) / 1e6;
        int ticks = encoder.getCountFrom(enc, true);
        resultLUT[index] = ticks / dt;
    };

    // Ramp down
    for (int i = N - 1; i >= 0; i--)
    {
        l298n.pwm.setDutyCycle(pwm, dutyList[i]);
        vTaskDelay(pdMS_TO_TICKS(20));
        measure50ms(i);
        printf("Down i=%d Duty=%d Speed=%.2f\n", i, dutyList[i], (float)resultLUT[i]);
    }

    // Optional coast delay before ramping back up
    vTaskDelay(pdMS_TO_TICKS(500));

    // Ramp up
    for (int i = 0; i < N; i++)
    {
        l298n.pwm.setDutyCycle(pwm, dutyList[i]);
        vTaskDelay(pdMS_TO_TICKS(20));
        encoder.clearTarget(enc);
        int64_t t0 = esp_timer_get_time();
        vTaskDelay(pdMS_TO_TICKS(50));
        int64_t t1 = esp_timer_get_time();
        float dt = (t1 - t0) / 1e6;
        int ticks = encoder.getCountFrom(enc, true);
        float secondPass = ticks / dt;
        resultLUT[i] = (resultLUT[i] + secondPass) / 2;
        printf("Up i=%d Duty=%d Speed=%.2f\n", i, dutyList[i], (float)resultLUT[i]);
    }

    // Stop
    l298n.pwm.setDutyCycle(pwm, 0);
    printf("LUT collection finished.\n");
}
*/

void updateIPD_MotorA(float sampleIntervalSec)
{
    // 1. Measure current speed in RPS
    //float measuredRPS = encoder.getRPS(encoder.pcntA, sampleIntervalSec);

    // 2. Compute error
    float error = setpointRPS - measuredRPS;

    // 3. Update integral term (only integral uses error in I-PD)
    integral_A += Ki * error * sampleIntervalSec;

    // 4. Derivative on measurement (not error)
    float dMeas = (measuredRPS - lastMeasurement_A) / sampleIntervalSec;

    // 5. Compute control effort
    float controlOutput = integral_A - (Kp * measuredRPS + Kd * dMeas);

    // 6. Clamp output to PWM bounds
    controlOutput = fmaxf(0.0f, fminf(controlOutput, 1023.0f));  // for 10-bit resolution

    // 7. Apply duty
    l298n.pwm.setDutyCycle(l298n.pwm.cmprA, static_cast<int>(controlOutput));

    // 8. Save state
    lastMeasurement_A = measuredRPS;

    // 9. Debug output (CSV-style for plotting)
    printf("RPS_SP=%.2f,RPS_FB=%.2f,PWM=%.2f\n", setpointRPS, measuredRPS, controlOutput);
}

void serialTalk()
{
    if (Serial.available())
    {
        String onString = "ON";
        String offString = "OFF";
        String cmd = Serial.readStringUntil('\n');
        if (cmd.startsWith("testMotorA"))
        {
            testMotorA = true;
            /*if(!startPlotting) {*/ Serial.printf("Start motor A testing\n"); //}
        }
        else if (cmd.startsWith("printMotorBResults"))
        {
            printMotorAResults = true;
            /*if(!startPlotting) {*/ Serial.printf("Print motor A test result\n"); //}
        }
        else if (cmd.startsWith("runSaveResults"))
        {
            runSaveResults = true;
            /*if(!startPlotting) {*/ Serial.printf("Run saveResults() function\n"); //}
        }
        else if (cmd.startsWith("runLoadResults"))
        {
            runLoadResults = true;
            /*if(!startPlotting) {*/ Serial.printf("Run loadResults() function\n"); //}
        }
        else if (cmd.startsWith("PID=ON"))
        {
            pid_on_off_control = true;
            /*if(!startPlotting) {*/ Serial.printf("PI Loop = Activated\n"); //}
        }
        else if (cmd.startsWith("PID=OFF"))
        {
            bool pid_on_off_control = false;
            /*if(!startPlotting) {*/ Serial.printf("PI Loop = Inactivated\n"); //}
        }
        else if (cmd.startsWith("quadEncoder=ON"))
        {
            quadrature_encoder_on_off_control = true;
            /*if(!startPlotting) {*/ Serial.printf("Print the encoder update = Activated\n"); //}
        }
        else if (cmd.startsWith("quadEncoder=OFF"))
        {
            quadrature_encoder_on_off_control = false;
            /*if(!startPlotting) {*/ Serial.printf("Print the encoder update = Inactivated\n"); //}
        }
        else if (cmd.startsWith("startPulsing"))
        {
            startPulsing = true;
            /*if(!startPlotting) {*/ Serial.printf("starting using the step response"); //}
        }
        else if (cmd.startsWith("stopPulsing"))
        {
            startPulsing = false;
            /*if(!startPlotting) {*/ Serial.printf("stopped using the step response"); //}
        }
        else if (cmd.startsWith("Kp="))
        {
            Kp = cmd.substring(3).toFloat();
            /*if(!startPlotting) {*/ Serial.printf("Kp = %.4f\n", Kp); //}
        }
        else if (cmd.startsWith("Ki="))
        {
            Ki = cmd.substring(3).toFloat();
            /*if(!startPlotting) {*/ Serial.printf("Ki = %.6f\n", Ki); //}
        }
        else if (cmd.startsWith("LPF="))
        {
            LPF_ALPHA = cmd.substring(3).toFloat();
            /*if(!startPlotting) {*/ Serial.printf("LPF_ALPHA = %.6f\n", LPF_ALPHA); //}
        }
        else if (cmd.startsWith("pulseOnTime="))
        {
            pid_run_duration_ms = cmd.substring(3).toInt();
            /*if(!startPlotting) {*/ Serial.printf("PID Pulse ON time = %ul mS\n", pid_run_duration_ms); //}
        }
        else if (cmd.startsWith("pulseOffTime="))
        {
            pid_off_duration_ms = cmd.substring(3).toInt();
            /*if(!startPlotting) {*/ Serial.printf("PID Pulse OFF time = %ul mS\n", pid_off_duration_ms); //}
        }
        else if (cmd.startsWith("pulseDutyHigh="))
        {
            pid_pulse_dutycycle_high = cmd.substring(3).toInt();
            /*if(!startPlotting) {*/ Serial.printf("PID Pulse High Dutycycle = %u\n", pid_pulse_dutycycle_high); //}
        }
        else if (cmd.startsWith("pulseDutyLow="))
        {
            pid_pulse_dutycycle_low = cmd.substring(3).toInt();
            /*if(!startPlotting) {*/ Serial.printf("PID Pulse Low Dutycycle = %u\n", pid_pulse_dutycycle_low); //}
        }
        else if (cmd.startsWith("pulsePeriods="))
        {
            pid_cycles_to_run = cmd.substring(3).toInt();
            /*if(!startPlotting) {*/ Serial.printf("Number of PID Pulses = %u\n", pid_cycles_to_run); //}
        }
        else if (cmd.startsWith("startPlotting"))
        {
            startPlotting = true;
            /*if(!startPlotting) {*/ Serial.printf("Start Plotting\n"); //}
        }
        else if (cmd.startsWith("stopPlotting"))
        {
            startPlotting = false;
            /*if(!startPlotting) {*/ Serial.printf("Stop Plotting\n"); //}
        }
    }
}

void serialRespons()
{
    
}

void pidPulseResponse(int &cycles_to_run)
{
    static bool          pulseIsOn       = false;       // current output state
    static unsigned long lastSwitchMs    = 0;           // when we last toggled
    unsigned long        now             = millis();
    bool                 newState        = pulseIsOn;
    bool                 justToggled     = false;

    if(cycles_to_run)
    {
        pid_on_off_control = true;
        if (pulseIsOn)
        {
            // we’re currently HIGH; see if we’ve run long enough and should go LOW
            if (now - lastSwitchMs >= pid_run_duration_ms)
            {
                newState     = false;
                lastSwitchMs = now;
                justToggled  = true;
                // falling edge means one full on/off pulse just completed:
                cycles_to_run--;
            }
        }
        else
        {
            // we’re currently LOW; see if we’ve been low long enough and should go HIGH
            if (now - lastSwitchMs >= pid_off_duration_ms)
            {
                newState     = true;
                lastSwitchMs = now;
                justToggled  = true;
            }
        }

        // only re‐write the encoder position when the state actually changes:
        if (justToggled)
        {
            position = newState ? pid_pulse_dutycycle_high : pid_pulse_dutycycle_low;
            encoder.setPosition( position );
            pulseIsOn = newState;
            vA = 1.0;
            //l298n.pwm.setDutyCycle(l298n.pwm.cmprA, position);
        }
    }
    else
    {
        pid_on_off_control = false;
        startPulsing = false;
        l298n.pwm.setDutyCycle(l298n.pwm.cmprA, 0);
        l298n.pwm.setDutyCycle(l298n.pwm.cmprB, 0);
    }
}

void printSerialCommands()
{
    Serial.printf("[PID=[ON]/[OFF]]\n");
    Serial.printf("[startPulsing]\n");
    Serial.printf("[stopPulsing]\n");
    Serial.printf("[Kp=[float]]  ");
    Serial.printf("(current value of Kp = %.6f)\n", Kp);
    Serial.printf("[Ki=[float]]  ");
    Serial.printf("(current value of Ki = %.6f)\n", Ki);
    Serial.printf("[LPF=[float]]  ");
    Serial.printf("(current value of LPF = %.6f)\n", LPF_ALPHA);
    Serial.printf("[printEncoder=[ON]/[OFF]]\n");
    Serial.printf("[pulseOnTime=[int]]  ");
    Serial.printf("(current value of pulseOnTime = %u)\n", pid_run_duration_ms);
    Serial.printf("[pulseOffTime=[int]]  ");
    Serial.printf("(current value of pulseOffTime = %u)\n", pid_off_duration_ms);
    Serial.printf("[pulseDutyHigh=[int]]  ");
    Serial.printf("(current value of pulseDutyHigh = %u)\n", pid_pulse_dutycycle_high);
    Serial.printf("[pulseDutyLow=[int]]  ");
    Serial.printf("(current value of pulseDutyLow = %u)\n", pid_pulse_dutycycle_low);
    Serial.printf("[pulsePeriods=[int]]  ");
    Serial.printf("(current value of pulsePeriods = %u)\n", pulsePeriods);
    Serial.printf("[startPlotting]\n");
    Serial.printf("[stopPlotting]\n");
}



void simpleKickStart(int motor, int kickStartDuty, int afterKickDuty, int errorStopTimeMS)
{
    
    // 1) clear any leftover count
    encoder.clearA_and_B();

    // scale dutycycle values for both motors for the initial kick:
    int32_t dutyA_kick = int32_t(round(kickStartDuty * scaleA));
    dutyA_kick = constrain(dutyA_kick, 0, l298n.pwm.getResolution());
    int32_t dutyB_kick = int32_t(round(kickStartDuty * scaleB));
    dutyB_kick = constrain(dutyB_kick, 0, l298n.pwm.getResolution());

    // scale dutycycle values for both motors for after kick is finished:
    int32_t dutyA_after = int32_t(round(afterKickDuty * scaleA));
    dutyA_after = constrain(dutyA_after, 0, l298n.pwm.getResolution());
    int32_t dutyB_after = int32_t(round(afterKickDuty * scaleB));
    dutyB_after = constrain(dutyB_after, 0, l298n.pwm.getResolution());

    // store time:
    unsigned long start = millis();

    // flags to indicate if motors have been detected to move:
    bool checkA = false;
    bool checkB = false;

    // 2) initial kick of both motors:
    l298n.pwm.setDutyCycle(l298n.pwm.cmprA, dutyA_kick);
    l298n.pwm.setDutyCycle(l298n.pwm.cmprB, dutyB_kick);
    
    if(motor == 2)
    {
        while (true)
        {
            // run the following untill both checkA & checkB have been set to true:
            while(!checkA && !checkB)
            {
                int32_t detectTicksA = encoder.getCountA(false);
                int32_t detectTicksB = encoder.getCountB(false);

                if (detectTicksA > 0)  
                {
                    l298n.pwm.setDutyCycle(l298n.pwm.cmprA, dutyA_after);
                    checkA = true;
                }
                if (detectTicksB > 0)  
                {
                    l298n.pwm.setDutyCycle(l298n.pwm.cmprB, dutyB_after);
                    checkB = true;
                }
                if (millis() - start > errorStopTimeMS)  
                {
                    break;
                }
                delay(1);
            }
            break;
        }
    }
    else if(motor == 1)
    {
        while (true)
        {
            // run the following untill both checkA & checkB have been set to true:
            while(!checkB)
            {
                int32_t detectTicksB = encoder.getCountB(false);

                if (detectTicksB > 0)  
                {
                    l298n.pwm.setDutyCycle(l298n.pwm.cmprB, dutyB_after);
                    checkB = true;
                }
                if (millis() - start > errorStopTimeMS)  
                {
                    break;
                }
                delay(1);
            }
            break;
        }
    }
    else if(motor == 0)
    {
        while (true)
        {
            // run the following untill both checkA & checkB have been set to true:
            while(!checkA)
            {
                int32_t detectTicksA = encoder.getCountA(false);

                if (detectTicksA > 0)  
                {
                    l298n.pwm.setDutyCycle(l298n.pwm.cmprA, dutyA_after);
                    checkA = true;
                }
                if (millis() - start > errorStopTimeMS)  
                {
                    break;
                }
                delay(1);
            }
            break;
        }
        
    }

}



