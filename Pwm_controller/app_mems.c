#include "app_mems.h"
#include "main.h"
#include "iks01a2_motion_sensors.h"
#include "stm32f4xx_nucleo.h"
#include "pwm_control.h"
#include "stdio.h"
#include "string.h"

/* Private variables ---------------------------------------------------------*/
static IKS01A2_MOTION_SENSOR_Capabilities_t MotionCapabilities[IKS01A2_MOTION_INSTANCES_NBR];
extern TIM_HandleTypeDef htim14;
/* Private function prototypes -----------------------------------------------*/
static void Accelero_Sensor_Handler(uint32_t Instance);
static void Gyro_Sensor_Handler(uint32_t Instance);
static void Magneto_Sensor_Handler(uint32_t Instance);
static void MX_IKS01A2_DataLogTerminal_Init(void);
static void MX_IKS01A2_DataLogTerminal_Process(void);

void MX_MEMS_Init(void)
{
  /* Initialize the peripherals and the MEMS components */
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
  for(int i = 0; i < IKS01A2_MOTION_INSTANCES_NBR; i++)
  {
    if(MotionCapabilities[i].Acc)
    {
      Accelero_Sensor_Handler(i);
    }
    if(MotionCapabilities[i].Gyro)
    {
      Gyro_Sensor_Handler(i);
    }
    if(MotionCapabilities[i].Magneto)
    {
      Magneto_Sensor_Handler(i);
    }
  }
  HAL_Delay(1000); // Delay for sensor data processing
}

static void Accelero_Sensor_Handler(uint32_t Instance) {
    IKS01A2_MOTION_SENSOR_Axes_t acceleration;
    if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration) == 0)
      {
        // Process gyroscope data
      }
}

static void Gyro_Sensor_Handler(uint32_t Instance) {
    IKS01A2_MOTION_SENSOR_Axes_t angular_velocity;
    char uartBuffer[100]; // Adjust the buffer size as needed

    if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity) == 0) {
        // Update the PWM duty cycle based on the Z-axis data of the gyroscope
        UpdatePWMDutyCycle(&htim14, TIM_CHANNEL_1, angular_velocity.z);

        // Prepare the UART message
        uint32_t pwm_duty_cycle = MapZAxisToPWMDutyCycle(angular_velocity.z);
        snprintf(uartBuffer, sizeof(uartBuffer),
                 "PWM: %lu, Gyro Z-Axis: %ld\r\n",
                 pwm_duty_cycle,
                 (long)angular_velocity.z); // Cast to long to match the %ld specifier


        // Transmit the UART message
        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
    }
}



static void Magneto_Sensor_Handler(uint32_t Instance)
{
  IKS01A2_MOTION_SENSOR_Axes_t magnetic_field;
  if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &magnetic_field) == 0)
  {

  }
}
