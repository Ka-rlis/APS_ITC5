// pwm_control.c

#include "stm32f4xx_hal.h"
#include "main.h"
#include "app_mems.h"
#include "pwm_control.h"


int32_t gyroZAxisValue;
int current_duty_cycle = 1500;
volatile int32_t angular_position_z;

float Gyro_Data_Integration(int32_t gyroZAxisValue){
	uint32_t currentTime = HAL_GetTick();
	uint32_t lastUpdateTime = 0;

	float deltaTime = (currentTime - lastUpdateTime) / 1000.0f;
	angular_position_z += (gyroZAxisValue * deltaTime);
	lastUpdateTime = currentTime;

	return angular_position_z;

}

void PWM_GyroMapping(int32_t gyroZAxisValue){
	// Min Pulse width = 0micros, Max = 20000micros
	// Min motor PWM 900micros, Max = 2100micros
	current_duty_cycle = (Gyro_Data_Integration(gyroZAxisValue) / 5000) + 1500;

    if (current_duty_cycle > 1700) {
        htim2.Instance->CCR1 = 1700;
    } else if (current_duty_cycle < 1300) {
        htim2.Instance->CCR1 = 1300;
    } else {
    	htim2.Instance->CCR1 = current_duty_cycle;
    }
}


