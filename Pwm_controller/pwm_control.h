// pwm_control.h
#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include "stm32f4xx_hal.h"
// Include the TIM HAL header
void UpdatePWMFrequency(TIM_HandleTypeDef *htim, uint32_t desired_pwm_freq);

#endif // PWM_CONTROL_H

