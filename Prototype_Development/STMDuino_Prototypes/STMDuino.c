#include "motion_fx.h"
#include "LSM6DSLSensor.h"

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



      Serial.print("Yaw: ");
      Serial.print(data_out.rotation[0]);
      Serial.print("  Pitch: ");
      Serial.print(data_out.rotation[1]);
      Serial.print("  Roll: ");
      Serial.print(data_out.rotation[2]);
      Serial.println("");

    }
    else
    {
      discardedCount++;
    }
  }
}
