#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

extern "C"
{
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

#include "MicroTimePlus.h"
#include "MTP_Integration.h"

// Forward-declare your globals (from your project)
extern int pwm_number_of_bits;

struct both_motors
{
    struct single_motor
    {
        int   dutycycleArray[1024]       = {};
        float resultFromStallArray[1024] = {};
        float resultDownArray[1024]      = {};
        float resultUpArray[1024]        = {};
        float resultAvgArray[1024]       = {};
        int   startDutycycleValue        = 0;
        int   haltedDutycycleValue       = 0;
        int   safeStartDutycycleValue    = 0;
    };
    single_motor A;
    single_motor B;
};

struct MeasureCfg
{
    // Timing (us/ms)
    uint32_t window_us                 = 50'000; // sampling window (e.g., 50ms)
    uint32_t settle_ms                 = 500;    // settle after duty change
    uint32_t kick_timeout_ms           = 300;    // how long to wait for movement during kick
    uint32_t after_kick_check_ms       = 120;    // verify movement window after kick

    // Averaging
    int      windows_per_point         = 5;      // N successive windows to average

    // Startup discovery
    int      safe_margin_duty          = 5;      // safeStart = stalling + margin

    // Printing
    bool     verbose                   = false;  // keep as false; we print concise status anyway
};

enum class MotorID : uint8_t
{
    A,
    B
};

// Abstract the minimal I/O we need for each motor
struct MotorIO
{
    // Set duty [0..maxDuty]
    void (*setDuty)(int duty) = nullptr;

    // Encoder hooks for this motor
    void (*clearEncoderTarget)() = nullptr;
    int  (*getCountFrom)(bool clear) = nullptr;

    // Optional convenience (unused)
    int  (*getCountNoClear)() = nullptr;

    // Max duty for this hardware (computed per call)
    int maxDuty = 1023;
};

// Build adapters from your existing instances (defined in LUT_Measure.cpp)
MotorIO make_motor_io(MotorID id);

// Top-level function: measure LUTs for one motor (blocking).
bool measure_motor_lut(MotorID id, both_motors& out, const MeasureCfg& cfg);

// Helpers to print arrays in two long rows (Windows copy/paste friendly)
void print_array_two_rows(const char* label, const float* v, size_t n);
void print_array_two_rows_int(const char* label, const int* v, size_t n);
