#include "ESP32_L298N.h"

constexpr uint8_t ESP32_L298N::dir_map[3][2];

ESP32_L298N::ESP32_L298N(int                pwmPinA,
                         int                pwmPinB,
                         uint32_t           pwm_freq,
                         uint8_t            pwm_resolution,
                         int                in1,
                         int                in2,
                         int                in3,
                         int                in4,
                        encoderEncoderPCNT& encoderAccess)
  : pwm(pwmPinA, pwmPinB, pwm_freq, pwm_resolution)
  , encoder(encoderAccess)
  , _in1(in1)
  , _in2(in2)
  , _in3(in3)
  , _in4(in4)
  , directionFlag(false)
{
    motor_A = pwm.cmprA;
    motor_B = pwm.cmprB;
    encoder_A = encoder.pcntA;
    encoder_B = encoder.pcntB;



}

esp_err_t ESP32_L298N::init()
{
    
    // map motors→pins
    _pins[MOTOR_A][0] = static_cast<gpio_num_t>(_in1);
    _pins[MOTOR_A][1] = static_cast<gpio_num_t>(_in2);
    _pins[MOTOR_B][0] = static_cast<gpio_num_t>(_in3);
    _pins[MOTOR_B][1] = static_cast<gpio_num_t>(_in4);

    // 1) Start the PWM hardware
    pwm.initPWM(0);

    // 2) Configure all direction-control pins as outputs
    for (int m = 0; m < 2; ++m) {
        for (int c = 0; c < 2; ++c) {
            gpio_reset_pin(_pins[m][c]);
            gpio_set_direction(_pins[m][c], GPIO_MODE_OUTPUT);
        }
    }

    return ESP_OK;
}

void ESP32_L298N::setMotorDirection(uint8_t motor, uint8_t dir)
{
    if (motor > MOTOR_B || dir > BACKWARD) {
        return;
    }
    gpio_set_level(_pins[motor][0], dir_map[dir][0]);
    gpio_set_level(_pins[motor][1], dir_map[dir][1]);
}

void ESP32_L298N::setSpoolDirection()
{
    directionFlag = true;
    setMotorDirection(MOTOR_A, FORWARD);
    setMotorDirection(MOTOR_B, BACKWARD);
}

void ESP32_L298N::setUnspoolDirection()
{
    directionFlag = false;
    setMotorDirection(MOTOR_A, BACKWARD);
    setMotorDirection(MOTOR_B, FORWARD);
}


bool ESP32_L298N::getDirectionFlag() const
{
    return directionFlag;
}

void ESP32_L298N::setDutyMotorA(uint32_t duty)
{
    pwm.setDutyCycle(pwm.cmprA, duty);
}

void ESP32_L298N::setDutyMotorB(uint32_t duty)
{
    pwm.setDutyCycle(pwm.cmprB, duty);
}

void ESP32_L298N::stop()
{
    pwm.setDutyCycle(pwm.cmprA, 0);
    pwm.setDutyCycle(pwm.cmprB, 0);
}

void ESP32_L298N::kickStart(int targetDutycycleA, int targetDutycycleB)
{
    bool useA;
    bool useB;
    
    if(targetDutycycleA > 0)
        useA = true;
    else if(targetDutycycleA < 0)
        useA = false;
    else
        useA = false;
    
    
    if(targetDutycycleB > 0)
        useB = true;
    else if(targetDutycycleA < 0)
        useB = false;
    else
        useB = false;
        

    // Clear encoders
    if (useA) encoder.clearTarget(encoder.pcntA);
    if (useB) encoder.clearTarget(encoder.pcntB);

    // Kick-start both
    if (useA) pwm.setDutyCycle(pwm.cmprA, pwm.getResolution());
    if (useB) pwm.setDutyCycle(pwm.cmprB, pwm.getResolution());

    // Wait for movement
    if (useA) while (encoder.getCountA(false) < 56)
    {
        __asm__ __volatile__("nop");
    }
    if (useB) while (encoder.getCountB(false) < 56)
    {
        __asm__ __volatile__("nop");
    }

    // Apply target duty cycles
    if (useA) pwm.setDutyCycle(pwm.cmprA, targetDutycycleA);
    if (useB) pwm.setDutyCycle(pwm.cmprB, targetDutycycleB);
    
    /*   // Wait for steady-state
    bool stableA = useA ? waitForSteady(encoder.pcntA, encoder) : true;
    bool stableB = useB ? waitForSteady(encoder.pcntB, encoder) : true;


    if (!stableA || !stableB)
    {
        printf("[WARN] Unstable kick-start: A:%d B:%d\n", stableA, stableB);
        if (useA) pwm.setDutyCycle(pwm.cmprA, 0);
        if (useB) pwm.setDutyCycle(pwm.cmprB, 0);
    }
    else
    {
        printf("[INFO] Kick-start complete → A:%d, B:%d\n", targetDutycycleA, targetDutycycleB);
    }
    */
}

void ESP32_L298N::kickStartA(int targetDutycycleA)
{

    // Clear encoders
    encoder.clearTarget(encoder.pcntA);

    // Kick-start both
    pwm.setDutyCycle(pwm.cmprA, pwm.getResolution());

    // Wait for movement
    while (encoder.getCountA(false) < 56)
    {
        __asm__ __volatile__("nop");
    }

    // Apply target duty cycles
    pwm.setDutyCycle(pwm.cmprA, targetDutycycleA);
    
}

void ESP32_L298N::kickStartB(int targetDutycycleB)
{

    // Clear encoders
    encoder.clearTarget(encoder.pcntB);

    // Kick-start both
    pwm.setDutyCycle(pwm.cmprB, pwm.getResolution());

    // Wait for movement
    while (encoder.getCountB(false) < 56)
    {
        __asm__ __volatile__("nop");
    }

    // Apply target duty cycles
    pwm.setDutyCycle(pwm.cmprB, targetDutycycleB);
    
}

void ESP32_L298N::kickStartAsNeeded(int dutyRequest,
                                    int noKickNeededAbove,
                                    int position
)
{
    bool isA_stalled = encoder.getCountA(true) == 0;
    bool isB_stalled = encoder.getCountB(true) == 0;
    bool useKick = isA_stalled && (dutyRequest < noKickNeededAbove);

    if (useKick)
    {
        kickStart(dutyRequest, dutyRequest);
    }

    // Normal duty set (after kick, or if kick not needed)
    pwm.setDutyCycle(pwm.cmprA, dutyRequest);
    pwm.setDutyCycle(pwm.cmprB, dutyRequest);
}

bool ESP32_L298N::waitForSteady(pcnt_unit_handle_t enc)
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

        if (stable >= (STEADY_COUNT - 1))
            return true;
        if ((esp_timer_get_time() - t0) > (STALL_TIMEOUT_MS * 1000))
            return false;
    }
}
