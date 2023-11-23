// pwm_control.h
#include "stm32f4xx_hal.h"
#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H
#define PWM_PERIOD_TICKS 20000
#define PWM_MIN 850
#define PWM_MAX 2150

void UpdatePWMFrequency(TIM_HandleTypeDef *htim, uint32_t desired_pwm_freq);
uint32_t MapZAxisToPWMDutyCycle(float z_axis_value);
void UpdatePWMDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, float z_axis_value);
#endif // PWM_CONTROL_H

