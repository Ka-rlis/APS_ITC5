
#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H
#include "stm32f4xx_hal.h" // Change this include to match your specific STM32 series
#include "main.h" // Include the TIM HAL header
extern TIM_HandleTypeDef htim14;

void UpdatePWMFrequency(TIM_HandleTypeDef *htim, uint32_t desired_pwm_freq) {
    // Assuming a Timer Clock Frequency of 72 MHz for STM32F1 series
    // You need to replace this with the actual clock frequency of the timer used for PWM
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq(); // Get the actual clock frequency
    uint32_t prescaler = 0;
    uint32_t period = 0;

    prescaler = (timer_clock / (desired_pwm_freq * (htim->Init.Period + 1))) - 1;
    period = (timer_clock / (desired_pwm_freq * (prescaler + 1))) - 1;

    // Update Timer settings
    htim->Init.Prescaler = prescaler;
    htim->Init.Period = period;

    // Reinitialize the Timer with the new settings
    if (HAL_TIM_Base_Init(htim) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(htim) != HAL_OK) {
        Error_Handler();
    }
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = period / 2; // 50% duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }


    if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}

void SetNewPWMFrequency() {
    uint32_t new_frequency = 10000; // Set the new frequency to 10 kHz
    UpdatePWMFrequency(&htim14, new_frequency);
}

#endif // PWM_CONTROL_H
