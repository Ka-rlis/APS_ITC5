/**
  ******************************************************************************
  * File Name          : app_mems.h
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.9.6.0 instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "main.h"
#include "iks01a2_motion_sensors.h"
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_MEMS_H
#define __APP_MEMS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
extern int32_t gyroZAxisValue;
extern IKS01A2_MOTION_SENSOR_Axes_t acceleration;
extern IKS01A2_MOTION_SENSOR_Axes_t magnetic_field;
extern IKS01A2_MOTION_SENSOR_Axes_t angular_velocity;
/* Exported defines ----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void MX_MEMS_Init(void);
void MX_MEMS_Process(void);
extern TIM_HandleTypeDef htim3;
#ifdef __cplusplus
}
#endif

#endif /* __APP_MEMS_H */
