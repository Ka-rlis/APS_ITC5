#if (__CORTEX_M == 0U)
#include "motion_fx_cm0p.h"
#else
#include "motion_fx.h"
#endif
#include "LSM6DSLSensor.h"
#include <BasicLinearAlgebra.h>
#include <Servo.h>
Servo motorleft;
Servo motorright;

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


float deltaTime;
unsigned long lastTime;



// Other necessary global variables and matrices from your first code

Matrix<2, 1, float> ObserverUpdate(const Matrix<2, 1, float>& y);
Matrix<2, 1, float> IntegError(const Matrix<2, 1, float>& y, const Matrix<2, 1, float>& r);
void InputXD(const Matrix<2,1, float> u_e);



// Observer definitions
BLA::Matrix<6, 6, float> A = 
{
    0.9998,    0.0100,         0,         0,    0.0000,    0.0000,
   -0.0458,    0.9998,         0,         0,    0.0014,    0.0014,
         0,         0,    1.0000,    0.0100,    0.0000,   -0.0000,
         0,         0,   -0.0093,    1.0000,    0.0053,   -0.0053,
         0,         0,         0,         0,    0.9778,         0,
         0,         0,         0,         0,         0,    0.9778
}; 


BLA::Matrix<6, 2> B = { 
    0.,    0,
    0,    0,
    0,   -0,
    0,   -0,
    0.2040,         0,
         0,    0.2040
}; 


BLA::Matrix<2, 6> C = { 
1,     0,     0,     0,     0,     0,
0,     0,     1,     0,     0,     0
};

BLA::Matrix<6, 2, float> L = { 
0.3267, -0.0256,
3.2360, -0.5240,
0.0073, 0.3483,
0.1756, 3.6898,
29.6574, 1.7413,
28.0739, -16.9959
}; 

BLA::Matrix<2, 2, float> K_i = {
   -0.03536,   -0.03536,   
   -0.03536,    0.03536   
  };

BLA::Matrix<2, 6, float> K_f = {
   -0.1577,    0.1963,    0.9678,    0.7581,    0.1078,   -0.0847,
  -0.1577 ,   0.1963,   -0.9678,   -0.7581,   -0.0847,    0.1078
};


BLA::Matrix<2, 1, float> integralError = { 
  0.0f, 0.0f
};

BLA::Matrix<2, 1, float> u_e = { 
  0.0f, 0.0f
};

// OTHER MATRIX definition
BLA::Matrix<2, 1, float> r = {-0.05f, -0.1f};
BLA::Matrix<2, 1, float> y = {0, 0};
BLA::Matrix<2, 1, float> x_est = {0, 0};
BLA::Matrix<2, 1, float> x_est_next = {0, 0};
BLA::Matrix<6, 1, float> curr_state = {0, 0, 0, 0, 0, 0};
BLA::Matrix<6, 1, float> next_state = {0, 0, 0, 0, 0, 0};
BLA::Matrix<2, 1, float> input = {0, 0};

/* Private variables ---------------------------------------------------------*/






float mapf(float x, float in_min, float in_max, float out_min, float out_max);
 
#if !(__CORTEX_M == 0U)
static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;
static uint8_t mfxstate[STATE_SIZE];
#endif

static volatile int sampleToDiscard = SAMPLETODISCARD;
static int discardedCount = 0;

char LibVersion[35];
int LibVersionLen;

static volatile uint32_t TimeStamp = 0;

int32_t accelerometer[3];
int32_t gyroscope[3];
int32_t magnetometer[3];
int32_t MagOffset[3];
float delta_time;

LSM6DSLSensor AccGyr(&Wire);

HardwareTimer *MyTim;

volatile uint8_t fusion_flag;

void fusion_update(void)
{
  fusion_flag = 1;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


Matrix<2, 1, float>ObserverUpdate(const Matrix<2, 1, float>& y) {
BLA::Matrix<6, 1> L_y = L * ((r-y) - (C * curr_state));

    // Compute the input effect
    BLA::Matrix<6, 1> B_u = B * input;

    // Update equation with Euler integration
    BLA::Matrix<6, 1> next_state = (A * curr_state) + B_u + L_y ;

  // Update the x_est to the current calc
  curr_state = next_state;

  
  u_e = -K_f * next_state;
 // Serial.println("    ue1: "); 
 // Serial.print(u_e(0,0));
 // Serial.print("      ue2: "); 
 // Serial.print(u_e(1,0));
  return u_e;
}
  
Matrix<2, 1, float> IntegError(const Matrix<2, 1, float>& y, const Matrix<2, 1, float>& r) {
    // Calculate the error
    BLA::Matrix<2, 1, float> Error = r - y;
  

 // Serial.print("            in1: ");
 // Serial.print((input(0,0)));
 // Serial.print("              in2: ");
 // Serial.println((input(1,0)));
    // Deadband threshold
    float deadband = 0.01f;

    // Integrate the error over time only if error is outside the deadband
     if (abs(Error(0,0)) > deadband || abs(Error(1,0)) > deadband) {
      //  Serial.println("error is: " );
      //  Serial.print(Error(1,0));
        integralError += Error * deltaTime;
    } else {
        // Optional: Reset the integral error within the deadband
         integralError = {0.0f, 0.0f};
    }

    // Apply the integral gain
    BLA::Matrix<2, 1, float> integralTerm = K_i * integralError;
    return integralTerm;
}

void InputXD(const Matrix<2,1, float> u_e){ 
  BLA::Matrix<2,1, float> integralErrors = IntegError(x_est_next, r);
  input = integralErrors + u_e; 
  int in1 = mapf(input(0,0), -1.00, 1.00, 1300.0, 1700.0);
  int in2 = mapf(input(1,0), -1.00, 1.00, 1300.0, 1700.0);
  Serial.println("    in1: "); 
  Serial.print(in1);
  Serial.print("                  in2: "); 
  Serial.print(in2);
  motorleft.writeMicroseconds(in1);
  motorright.writeMicroseconds(in2);
  
}


void saturation(){
  input(0, 0) = (input(0, 0) > 1.0f) ? 1.0f : ((input(0, 0) < -1.0f) ? -1.0f : input(0, 0));
  input(1, 0) = (input(1, 0) > 1.0f) ? 1.0f : ((input(1, 0) < -1.0f) ? -1.0f : input(1, 0));

}



const int PotPin=A0;
void setup() {

  pinMode(PB8, OUTPUT);
  pinMode(PB9, OUTPUT);
  
  /* Initialize Serial */
  Serial.begin(9600);
  motorleft.attach(PB8); // Attach the servo to digital pin 9
  motorright.attach(PB9); // Attach the servo to digital pin 9
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

  /* Initialize sensor fusion library */
#if (__CORTEX_M == 0U)
  MotionFX_CM0P_initialize(MFX_CM0P_MCU_STM32);
  MotionFX_CM0P_setOrientation("nwu", "nwu", NULL);
  MotionFX_CM0P_enable_gbias(MFX_CM0P_ENGINE_ENABLE);
  MotionFX_CM0P_enable_euler(MFX_CM0P_ENGINE_ENABLE);
  MotionFX_CM0P_enable_6X(MFX_CM0P_ENGINE_ENABLE);
  MotionFX_CM0P_enable_9X(MFX_CM0P_ENGINE_DISABLE);

  /* OPTIONAL */
  /* Get library version */
  LibVersionLen = (int)MotionFX_CM0P_GetLibVersion(LibVersion);
#else
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
#endif
  MyTim = new HardwareTimer(TIM3);
  MyTim->setOverflow(ALGO_FREQ, HERTZ_FORMAT);
  MyTim->attachInterrupt(fusion_update);
  MyTim->resume();

        // Calibrate the ESCs
    Serial.println("Starting ESC Calibration...");

    // Step 1: Send Maximum Throttle
    Serial.println("Sending Maximum Throttle...");
    motorright.writeMicroseconds(1500);
    motorleft.writeMicroseconds(1500);

    // Wait a bit to let the ESC recognize the max throttle
    delay(1000);

    // Step 2: Send Minimum Throttle
    Serial.println("Sending Minimum Throttle...");
    motorright.writeMicroseconds(1450);
    motorleft.writeMicroseconds(1450);

    // Wait a bit to let the ESC recognize the min throttle
    delay(1000);

    // Step 3: Set to Neutral (1500 microseconds)
    Serial.println("Setting Neutral Point...");
    motorright.writeMicroseconds(1550);
    motorleft.writeMicroseconds(1550);
    delay(1000);
    // Calibration is complete
    Serial.println("ESC Calibration Complete!");
}

void loop() {


  if(fusion_flag)
  {
#if (__CORTEX_M == 0U)
    MFX_CM0P_input_t data_in;
    MFX_CM0P_output_t data_out;
#else
    MFX_input_t data_in;
    MFX_output_t data_out;
#endif
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
      float testValue = mapf(data_out.rotation[0],0.0f,57.0f,0.0f,6.28f); 
      //x_est_next(1,0) = mapf(data_out.rotation[0],0.0f,57.0f,0.0f,6.28f);
      if (testValue > 10 ) { 
        testValue -= 39.65; 
        x_est_next(1,0) = testValue;
        Serial.print(testValue);
      } else { 
        x_est_next(1,0) = mapf(data_out.rotation[0],0.0f,57.0f,0.0f,6.28f);
        Serial.print(mapf(data_out.rotation[0],0.0f,57.0f,0.0f,6.28f));
      }
      // map 0-360 to -180 - 180
      // -180 - 180 to -3.14 - 3.14
      
      Serial.print("  Pitch: ");
      Serial.print(mapf(data_out.rotation[1],0.0f,360.0f,0.0f,3.0f));
      x_est_next(0,0) = 1.5*tan(mapf(data_out.rotation[1],0.0f,360.0f,0.0f,3.0f));
      


    }
    else
    {
      discardedCount++;
    }
    
  }
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 100.0; // Time difference in seconds
  lastTime = currentTime;
  

 // runForDuration(5000);

  ObserverUpdate(x_est_next); 
  InputXD(u_e); 
  saturation();
  

}

