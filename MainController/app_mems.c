#include "app_mems.h"
#include "main.h"
#include "iks01a2_motion_sensors.h"
#include "stm32f4xx_nucleo.h"
#include "pwm_control.h"


/* Private variables ---------------------------------------------------------*/
static IKS01A2_MOTION_SENSOR_Capabilities_t MotionCapabilities[IKS01A2_MOTION_INSTANCES_NBR];
/* Private function prototypes -----------------------------------------------*/
static void Accelero_Sensor_Handler(uint32_t Instance);
static uint32_t Gyro_Sensor_Handler(uint32_t Instance);
static void Magneto_Sensor_Handler(uint32_t Instance);
static void MX_IKS01A2_DataLogTerminal_Init(void);
static void MX_IKS01A2_DataLogTerminal_Process(void);


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
  /* Initialize motion sensors */
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);

}

void MX_IKS01A2_DataLogTerminal_Process(void)
{
  for(int i = 0; i < 2; i++)
  {
    if(MotionCapabilities[i].Acc ==0)
    {
      Accelero_Sensor_Handler(i);
    }
    if(MotionCapabilities[i].Gyro ==0)
    {
      Gyro_Sensor_Handler(i);
    }
    if(MotionCapabilities[i].Magneto ==0)
    {
      Magneto_Sensor_Handler(i);
    }
  }
  HAL_Delay(300);
}

static void Accelero_Sensor_Handler(uint32_t Instance) {
    IKS01A2_MOTION_SENSOR_Axes_t acceleration;
    if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration) == 0)
      {

      }
}

static uint32_t Gyro_Sensor_Handler(uint32_t Instance) {
    IKS01A2_MOTION_SENSOR_Axes_t angular_velocity;


    if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity) == 0)
    	{

        gyroZAxisValue = angular_velocity.z;


        return gyroZAxisValue;
    } else {
    }
}
static void Magneto_Sensor_Handler(uint32_t Instance)
{
  IKS01A2_MOTION_SENSOR_Axes_t magnetic_field;
  if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &magnetic_field) == 0)
  {

  }
}
