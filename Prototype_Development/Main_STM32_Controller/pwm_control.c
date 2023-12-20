// pwm_control.c

#include "stm32f4xx_hal.h"
#include "main.h"
#include "app_mems.h"
#include "pwm_control.h"
#include "motion_fx.h"

int current_duty_cycle = 1500;
int gyroZAxisValue = 0;


int get_data(MFX_input_t data_in){
	gyroZAxisValue = data_in.gyro[2];

	return gyroZAxisValue;
}

float Gyro_Data_Integration(gyroZAxisValue){

	uint32_t currentTime = HAL_GetTick();
	uint32_t lastUpdateTime = 0;
	float angular_position_z = 0;

	float deltaTime = (currentTime - lastUpdateTime) / 1000.0f;
	angular_position_z += (gyroZAxisValue * deltaTime);
	lastUpdateTime = currentTime;

	return angular_position_z;
}

int PWM_GyroMapping(int gyroZAxisValue){
	// Min Pulse width = 0micros, Max = 20000micros
	// Min motor PWM 900micros, Max = 2100micros
	current_duty_cycle = (Gyro_Data_Integration(gyroZAxisValue) / 5000) + 1500;

    if (current_duty_cycle > 1700) {
        return 1700;
    } else if (current_duty_cycle < 1300) {
        return 1300;
    } else {
    	return current_duty_cycle;
    }
}


