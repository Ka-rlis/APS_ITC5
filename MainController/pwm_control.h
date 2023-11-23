// pwm_control.h

#include "stm32f4xx_hal.h"
#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

extern volatile int32_t angular_position_z;
int32_t PWM_GyroMapping(int32_t angular_position_z);
float Gyro_Data_Integration(int32_t gyroZAxisValue);




#endif // PWM_CONTROL_H

