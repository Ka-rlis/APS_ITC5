// pwm_control.c

// pwm_control.h
#include "stm32f4xx_hal.h"
#include "main.h"
#include "app_mems.h"
#include "pwm_control.h"

uint32_t gyroZAxisValue;


uint32_t PWM_GyroMapping(uint32_t gyroZAxisValue) {
	// Min Pulse width = 0micros, Max = 20000micros
	// Min motor PWM 900micros, Max = 2100micros
	uint32_t current_duty_cycle = 1500;


    return current_duty_cycle;
}
