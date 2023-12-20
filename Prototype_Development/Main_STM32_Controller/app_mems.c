#include "app_mems.h"
#include "main.h"
#include "iks01a2_motion_sensors.h"
#include "stm32f4xx_nucleo.h"
#include "pwm_control.h"
#include "motion_fx.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static IKS01A2_MOTION_SENSOR_Axes_t Accelero_Sensor_Handler(uint32_t Instance);
static IKS01A2_MOTION_SENSOR_Axes_t Gyro_Sensor_Handler(uint32_t Instance);
//static IKS01A2_MOTION_SENSOR_Axes_t Magneto_Sensor_Handler(uint32_t Instance);
IKS01A2_MOTION_SENSOR_Axes_t acceleration;
IKS01A2_MOTION_SENSOR_Axes_t angular_velocity;
//IKS01A2_MOTION_SENSOR_Axes_t magnetic_field;
static void MX_IKS01A2_DataLogTerminal_Init(void);


MFX_input_t data_in;
MFX_input_t data_out;
#define MOTION_FX_ENGINE_DELTATIME  0.1f
float delta_time = MOTION_FX_ENGINE_DELTATIME;
static MFXState_t mfxstate;

void MX_MotionFX_Init(void) {
    MFX_knobs_t iKnobs;
    MotionFX_initialize(&mfxstate);
    MotionFX_getKnobs(&mfxstate, &iKnobs);
    // Configure knobs as per your requirements
    MotionFX_setKnobs(&mfxstate, &iKnobs);
}

void TIM3_IRQHandler(void)
{

  HAL_TIM_IRQHandler(&htim3);

  /* USER CODE BEGIN TIM3_IRQn 1 */
  // Read sensor data
  acceleration = Accelero_Sensor_Handler(IKS01A2_LSM6DSL_0);
  angular_velocity = Gyro_Sensor_Handler(IKS01A2_LSM6DSL_0);

  // Prepare input data structure
  data_in.acc[0] = acceleration.x;
  data_in.acc[1] = acceleration.y;
  data_in.acc[2] = acceleration.z;

  data_in.gyro[0] = angular_velocity.x;
  data_in.gyro[1] = angular_velocity.y;
  data_in.gyro[2] = angular_velocity.z;
  MotionFX_propagate(&mfxstate, &data_out, &data_in, &delta_time);
  MotionFX_update(&mfxstate, &data_out, &data_in, &delta_time, NULL);

}

void MX_MEMS_Init(void)
{

  MX_IKS01A2_DataLogTerminal_Init();
}

void MX_MEMS_Process(void)
{
  MX_IKS01A2_DataLogTerminal_Process();
}

void MX_IKS01A2_DataLogTerminal_Init(void)
{
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);


}

static IKS01A2_MOTION_SENSOR_Axes_t Accelero_Sensor_Handler(uint32_t Instance) {
	IKS01A2_MOTION_SENSOR_Axes_t acceleration;
    if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration) == 0)
      {

    	return acceleration;
      }
}

static IKS01A2_MOTION_SENSOR_Axes_t Gyro_Sensor_Handler(uint32_t Instance) {
	IKS01A2_MOTION_SENSOR_Axes_t angular_velocity;

    if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity) == 0)
    	{
        return angular_velocity;
    	}
}
