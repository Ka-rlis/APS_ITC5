#include "motion_fx.h"
#include "LSM6DSLSensor.h"
#include <BasicLinearAlgebra.h>

using namespace BLA;

#define ALGO_FREQ  100U /* Algorithm frequency 100Hz */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
#define MOTION_FX_ENGINE_DELTATIME  0.01f
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f

#define STATE_SIZE                      (size_t)(2432)

#define SAMPLETODISCARD                 15

#define GBIAS_ACC_TH_SC                 (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC                (2.0f*0.002f)

#define DECIMATION                      1U

/* Private variables ---------------------------------------------------------*/

static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;
static uint8_t mfxstate[STATE_SIZE];


static volatile int sampleToDiscard = SAMPLETODISCARD;
static int discardedCount = 0;

char LibVersion[35];
int LibVersionLen;

static volatile uint32_t TimeStamp = 0;

int32_t accelerometer[3];
int32_t gyroscope[3];
int32_t magnetometer[3];
int32_t MagOffset[3];

LSM6DSLSensor AccGyr(&Wire);

HardwareTimer *MyTim;

volatile uint8_t fusion_flag; 
Matrix<2,1> ObserverUpdate(Matrix<2, 1> y, Matrix<2, 1> u);
Matrix<2,1> IntegError(Matrix<2, 1> y, Matrix<2, 1> r);
  // MATRIX STUFF 
  
  // Observer definitions
  BLA::Matrix<6, 6> A = 
  {
  0,    1.0000,         0,         0,         0,         0,
  -4.5796,         0,         0,         0,    0.1460,    0.1460,
  0,         0,         0,    1.0000,         0,         0,
  0,         0,   -0.9344,         0,    0.5376,   -0.5376,
  0,         0,         0,         0,   -2.2472,         0,
  0,         0,         0,        0,         0,   -2.2472
  }; 


  BLA::Matrix<6, 2> B = { 
  0,         0,
  0,         0,
  0,         0,
  0,         0,
  20.6292,   0,
  0,   20.6292
  }; 


  BLA::Matrix<2, 6> C = { 
  1,     0,     0,     0,     0,     0,
  0,     0,     1,     0,     0,     0
  };

  BLA::Matrix<6, 2> L = { 
  5.0579,    0.0773,
  -3.1079,    0.3535,
  0.0966,    5.1477,
  0.4394,    0.5170,
  10.9207,    2.1885,
  11.5894,   -4.1788
  }; 

  BLA::Matrix<2, 2> K_i = {
   -2.2361,   -2.2361,    
   -2.2361,    2.2361
   };
  
  BLA::Matrix<2, 6> K_f = {
  0.1856,    1.0714,    3.5135,    1.6571,    0.2086,   -0.1121,
  0.1856,    1.0714,   -3.5135,   -1.6571,   -0.1121,    0.2086
  };

  BLA::Matrix<2, 1> integralError = { 
    0, 0
  };

  BLA::Matrix<2, 1> u_e = { 
    0, 0
  };

  // OTHER MATRIX definition
  BLA::Matrix<2, 1> r = {0, 0};
  BLA::Matrix<2, 1> y = {0, 0};
  BLA::Matrix<2, 1> u = {1, 1};
  BLA::Matrix<6, 1> x_est = {0,0,0,0,0,0};
 

void fusion_update(void)
{
  fusion_flag = 1;
}

void setup() {
  /* Initialize Serial */
  Serial.begin(115200);
  while (!Serial) yield();

  /* Initialize I2C bus */
  Wire.begin();
  Wire.setClock(400000);

  /* Start communication with IMU */
  AccGyr.begin();
  AccGyr.Set_X_ODR((float)ALGO_FREQ);
  AccGyr.Set_X_FS(4);
  AccGyr.Set_G_ODR((float)ALGO_FREQ);
  AccGyr.Set_G_FS(2000);
  AccGyr.Enable_X();
  AccGyr.Enable_G();
  delay(10);

 
  MotionFX_initialize((MFXState_t *)mfxstate);

  MotionFX_getKnobs(mfxstate, ipKnobs);

  ipKnobs->acc_orientation[0] = 'n';
  ipKnobs->acc_orientation[1] = 'w';
  ipKnobs->acc_orientation[2] = 'u';
  ipKnobs->gyro_orientation[0] = 'n';
  ipKnobs->gyro_orientation[1] = 'w';
  ipKnobs->gyro_orientation[2] = 'u';

  ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC;
  ipKnobs->gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;

  ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
  ipKnobs->LMode = 1;
  ipKnobs->modx = DECIMATION;

  MotionFX_setKnobs(mfxstate, ipKnobs);
  MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);
  MotionFX_enable_9X(mfxstate, MFX_ENGINE_DISABLE);

  /* OPTIONAL */
  /* Get library version */
  LibVersionLen = (int)MotionFX_GetLibVersion(LibVersion);

  MyTim = new HardwareTimer(TIM3);
  MyTim->setOverflow(ALGO_FREQ, HERTZ_FORMAT);
  MyTim->attachInterrupt(fusion_update);
  MyTim->resume();


}

void loop() {

  if(fusion_flag)
  {

    MFX_input_t data_in;
    MFX_output_t data_out;

    float delta_time = MOTION_FX_ENGINE_DELTATIME;
    fusion_flag = 0;
    AccGyr.Get_X_Axes(accelerometer);
    AccGyr.Get_G_Axes(gyroscope);

    /* Convert angular velocity from [mdps] to [dps] */
    data_in.gyro[0] = (float)gyroscope[0] * FROM_MDPS_TO_DPS;
    data_in.gyro[1] = (float)gyroscope[1] * FROM_MDPS_TO_DPS;
    data_in.gyro[2] = (float)gyroscope[2] * FROM_MDPS_TO_DPS;

    /* Convert acceleration from [mg] to [g] */
    data_in.acc[0] = (float)accelerometer[0] * FROM_MG_TO_G;
    data_in.acc[1] = (float)accelerometer[1] * FROM_MG_TO_G;
    data_in.acc[2] = (float)accelerometer[2] * FROM_MG_TO_G;

    /* Don't set mag values because we use only acc and gyro */
    data_in.mag[0] = 0.0f;
    data_in.mag[1] = 0.0f;
    data_in.mag[2] = 0.0f;

    if (discardedCount == sampleToDiscard)
    {

      MotionFX_propagate(mfxstate, &data_out, &data_in, &delta_time);
      MotionFX_update(mfxstate, &data_out, &data_in, &delta_time, NULL);

      y(0, 0) = data_out.rotation[0];
      y(1, 0) = data_out.linear_acceleration[1];

      ObserverUpdate(y, u);
      IntegError(y, r);
    }
    else
    {
      discardedCount++;
    }
  
  }
}

Matrix<2,1> ObserverUpdate(Matrix<2, 1> y, Matrix<2, 1> u) { 
  // Compute A_obs = A - LC
  BLA::Matrix<6, 6> lc = L * C; 
  BLA::Matrix<6, 6> A_obs = A - lc;
  
  // Compute B_obs = [B, L]
  BLA::Matrix<6, 4> B_obs = B || L;

  // Comput u ; y_meas
  BLA::Matrix<4, 1> uy = u && y;

  // Compute x_est_next = A_obs * x_est + B_obs * [u; y_meas]
  BLA::Matrix<6, 1> x_est_next = (A_obs * x_est) + (B_obs * uy);

  // Update the x_est to the current calc
  x_est = x_est_next;
  u_e = -K_f * x_est_next; 
  return u_e;
}

Matrix<2,1> IntegError(Matrix<2, 1> y, Matrix<2, 1> r) { 
  BLA::Matrix<2,1> Error = r - y; 
  float_t dt = 0.01f;
  integralError += Error * dt;
  integralError = K_i * integralError; 

  return integralError; 
}
