#pragma once
#include <string>
#include <string_view>
#include <freertos/FreeRTOS.h>
#include <driver/pulse_cnt.h>
#include <driver/mcpwm_prelude.h>
#include <encoderEncoderPCNT.h>
#include <MCPWM_DUAL_PWM.h>
#include <LutDataStruct.h>

void collectLUT_rampDownUp(
    pcnt_unit_handle_t enc,
    mcpwm_cmpr_handle_t pwm,
    MCPWM_DUAL_PWM& pwmDriver,
    encoderEncoderPCNT& encoderAccess,
    both_motors& data);
