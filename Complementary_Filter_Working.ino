#include <Arduino.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
#include <LSM6DSLSensor.h>
#include <Wire.h> 
#include <BasicLinearAlgebra.h>
#include <STM32_PWM.h>
using namespace BLA;

// Constants and Macros from your first code
#define ALGO_FREQ  100U /* Algorithm frequency 100Hz */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f
#define STATE_SIZE  (size_t)(2432)
#define SAMPLETODISCARD  15
#define GBIAS_ACC_TH_SC  (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC  (2.0f*0.002f)
#define DECIMATION  1U

// Constants from your second code
#define PI 3.1415926535897932384626433832795
#define mddps_to_rads 0.001*(PI/180)
#define g 9.81553035
#define mg_to_si 0.001 * g
#define alpha 0.1

#define MOTOR_LEFT_PIN     D6  // Corresponds to PB8
#define MOTOR_RIGHT_PIN    D3  // Corresponds to PB9

// Standard ESCs use a frequency of 50Hz
#define PWM_FREQUENCY      50

// Define the pulse widths for calibration
#define MAX_THROTTLE       2000  // Maximum throttle pulse width in microseconds
#define MIN_THROTTLE       1000  // Minimum throttle pulse width in microseconds
#define NEUTRAL_THROTTLE   1520  // Neutral throttle pulse width in microseconds

HardwareTimer* motorLeftTimer = nullptr;
HardwareTimer* motorRightTimer = nullptr;

LSM6DSLSensor AccGyr(&Wire); 
LSM303AGR_MAG_Sensor Mag(&Wire);  
LSM303AGR_ACC_Sensor Acc2(&Wire);  

float deltaTime;
unsigned long lastTime;

// Global variables for sensor data
int32_t accelerometer[3];
int32_t gyroscope[3];
int32_t magnetometer[3];

// Variables for the complementary filter
Matrix<3, 1, float> Sensor_angle_Gyro = {0, 0, 0};
Matrix<2, 1, float> Sensor_angle_Acc = {0, 0};
Matrix<3, 1, float> Sensor_Acc = {0, 0, 0};
Matrix<2, 1, float> Sensor_Mag = {0, 0};
Matrix<1, 1, float> Sensor_angle_Mag = {0};

// Other necessary global variables and matrices from your first code
volatile uint8_t fusion_flag; 
Matrix<2,1> ObserverUpdate(const Matrix<2, 1, float> y, const Matrix<2, 1, float> input);
Matrix<2,1> IntegError(const Matrix<2, 1, float> y, const Matrix<2, 1, float> r);
Matrix<2,1> InputXD(const Matrix<2,1, float> u_e);
int pwmMap(float inputs);
double map_x_to_y(double x);
void calibration();
// Observer definitions
BLA::Matrix<6, 6, float> A = 
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

BLA::Matrix<6, 2, float> L = { 
5.0579,    0.0773,
-3.1079,    0.3535,
0.0966,    5.1477,
0.4394,    0.5170,
10.9207,    2.1885,
11.5894,   -4.1788
}; 

BLA::Matrix<2, 2, float> K_i = {
  -2.2361,   -2.2361,    
  -2.2361,    2.2361
  };

BLA::Matrix<2, 6, float> K_f = {
0.1856,    1.0714,    3.5135,    1.6571,    0.2086,   -0.1121,
0.1856,    1.0714,   -3.5135,   -1.6571,   -0.1121,    0.2086
};

Matrix<2,1, float> input = { 
  0.0f, 0.0f
};

BLA::Matrix<2, 1, float> integralError = { 
  0.0f, 0.0f
};

BLA::Matrix<2, 1, float> u_e = { 
  0.0f, 0.0f
};

// OTHER MATRIX definition
BLA::Matrix<2, 1, float> r = {0.0f, 0.0f};
BLA::Matrix<2, 1, float> y = {0, 0};
BLA::Matrix<6, 1, float> x_est = {0, 0, 0, 0, 0, 0};
BLA::Matrix<6, 1, float> x_est_next = {0, 0, 0, 0, 0, 0};


void setup() {
  // Initialize Serial
  Serial.begin(115200);
  while (!Serial) yield();  // Wait for Serial to be ready
  calibration();
  // Initialize I2C bus
  Wire.begin();
  Wire.setClock(400000);  // Set I2C clock speed

  // Initialize LSM6DSLSensor for Accelerometer and Gyroscope
  AccGyr.begin();
  AccGyr.Set_X_ODR((float)ALGO_FREQ);
  AccGyr.Set_X_FS(4);
  AccGyr.Set_G_ODR((float)ALGO_FREQ);
  AccGyr.Set_G_FS(2000);
  AccGyr.Enable_X();
  AccGyr.Enable_G();

  // Initialize LSM303AGR for Magnetometer
  Mag.begin();
  Mag.SetODR(10.0f);  // Set Output Data Rate
  Mag.SetFS(50);      // Set full scale
  Mag.Enable();        // Enable Magnetometer

  // Initialize LSM303AGR for Additional Accelerometer
  Acc2.begin();
  Acc2.SetODR(10.0f);  // Set Output Data Rate
  Acc2.SetFS(2);       // Set full scale
  Acc2.Enable();        // Enable Accelerometer


 
}

double map_x_to_y(double x) {
    return x * 200.0 + 1500.0;
}


void calibration(){ 
  Serial.println("Starting ESC calibration...");

  // Initialize the PWM for the left motor (D15)
  motorLeftTimer = new HardwareTimer(TIM2); // Use appropriate timer for D15
  Serial.println("1");
  motorLeftTimer->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_LEFT_PIN);
  Serial.println("2");
  motorLeftTimer->setOverflow(PWM_FREQUENCY, HERTZ_FORMAT);
  Serial.println("3");
  // Initialize the PWM for the right motor (D14)
  motorRightTimer = new HardwareTimer(TIM2); // Use appropriate timer for D14
  Serial.println("4");
  motorRightTimer->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_RIGHT_PIN);
  Serial.println("5");
  motorRightTimer->setOverflow(PWM_FREQUENCY, HERTZ_FORMAT);
  Serial.println("6");
  // Start with Maximum Throttle
  motorLeftTimer->setCaptureCompare(3, MAX_THROTTLE, MICROSEC_COMPARE_FORMAT);
  Serial.println("7");
  motorRightTimer->setCaptureCompare(2, MAX_THROTTLE, MICROSEC_COMPARE_FORMAT);
  motorLeftTimer->resume();
  motorRightTimer->resume();
  Serial.println("Setting Maximum Throttle...");
  delay(2000); // Wait for the ESC to recognize the max throttle

  // Set to Minimum Throttle
  motorLeftTimer->setCaptureCompare(3, MIN_THROTTLE, MICROSEC_COMPARE_FORMAT);
  motorRightTimer->setCaptureCompare(2, MIN_THROTTLE, MICROSEC_COMPARE_FORMAT);
  Serial.println("Setting Minimum Throttle...");
  delay(2000); // Wait for the ESC to recognize the min throttle

  // Set to Neutral (Idle) Throttle
  motorLeftTimer->setCaptureCompare(3, NEUTRAL_THROTTLE, MICROSEC_COMPARE_FORMAT);
  motorRightTimer->setCaptureCompare(2, NEUTRAL_THROTTLE, MICROSEC_COMPARE_FORMAT);
  Serial.println("Setting Neutral Throttle...");

  Serial.println("ESC Calibration is complete!");
}

void sensor_all(){
  // Read accelerometer and gyroscope data from LSM6DSLSensor
  AccGyr.Get_X_Axes(accelerometer);
  AccGyr.Get_G_Axes(gyroscope);

  // Read magnetometer data from LSM303AGR_MAG_Sensor
  Mag.GetAxes(magnetometer);

  // Convert gyroscope data from mdps to radians/s and update the Sensor_angle_Gyro matrix
  Sensor_angle_Gyro(0, 0) += (gyroscope[0] * mddps_to_rads) * deltaTime;
  Sensor_angle_Gyro(1, 0) += (gyroscope[1] * mddps_to_rads) * deltaTime;
  Sensor_angle_Gyro(2, 0) += (gyroscope[2] * mddps_to_rads) * deltaTime;

  // Convert accelerometer data from mg to m/s^2 and update the Sensor_Acc matrix
  Sensor_Acc(0, 0) = accelerometer[0] * mg_to_si;
  Sensor_Acc(1, 0) = accelerometer[1] * mg_to_si;
  Sensor_Acc(2, 0) = accelerometer[2] * mg_to_si;

  // Calculate accelerometer angles (roll and pitch)
  Sensor_angle_Acc(0, 0) = atan2(Sensor_Acc(0, 0), Sensor_Acc(2, 0));
  Sensor_angle_Acc(1, 0) = atan2(Sensor_Acc(2, 0), g);

  // Calculate magnetometer angle (yaw)
  Sensor_angle_Mag(0, 0) = atan2(Sensor_Mag(0, 0), Sensor_Mag(1, 0));

  fusion_flag = 1;
}
void complementaryFilter() {
  // Update the estimated angles using a complementary filter approach
  x_est_next(0, 0) = alpha * (x_est(0, 0) + Sensor_angle_Gyro(0, 0)) + (1 - alpha) * Sensor_angle_Acc(0, 0);
  x_est_next(1, 0) = alpha * (x_est(1, 0) + Sensor_angle_Gyro(1, 0)) + (1 - alpha) * Sensor_angle_Acc(1, 0);
  x_est_next(2, 0) = alpha * (x_est(2, 0) + Sensor_angle_Gyro(2, 0)) + (1 - alpha) * Sensor_angle_Mag(0, 0);

  // Update the current state
  x_est = x_est_next;
}

void loop() {
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0; 
  lastTime = currentTime;


  sensor_all();

  //if (fusion_flag) {
  //  fusion_flag = 1;
  complementaryFilter();

  // Example of how you might use the updated estimates
  Serial.print("   Estimated Roll: "); Serial.print(x_est(0, 0));
  Serial.print("   Estimated Pitch: "); Serial.print(x_est(1, 0));
  Serial.print("   Estimated Yaw: "); Serial.println(x_est(2, 0));
  motorLeftTimer->setCaptureCompare(3, map_x_to_y(x_est(2,0)), MICROSEC_COMPARE_FORMAT);
  //}

}


Matrix<2,1> ObserverUpdate(Matrix<2, 1> y, Matrix<2, 1> input) { 
   
    BLA::Matrix<6, 1> L_y = L * (y - (C * x_est));

    // Compute the input effect
    BLA::Matrix<6, 1> B_u = B * input;

    // Update equation with Euler integration
    BLA::Matrix<6, 1> x_est_next = x_est +  (((A * x_est) + B_u + L_y) * 0.01f) ;

  // Update the x_est to the current calc
    x_est = x_est_next;
    u_e = -K_f * x_est_next; 

  return u_e;
}
  
Matrix<2,1, float> IntegError(const Matrix<2, 1, float> y, const Matrix<2, 1, float> r) { 
  BLA::Matrix<2,1, float> Error = r - y; 
  float  dt = 0.01f;
  integralError += Error * dt;
  integralError = /* K_i * integralError*/ 0; 

  return 0; 
}
  
Matrix<2,1, float> InputXD(const Matrix<2,1, float> u_e){ 
  BLA::Matrix<2,1, float> integralErrors = IntegError(y, r);
  input = integralErrors + u_e;  

  return input;
}
  
int pwmMap(float inputs) {
  if (inputs < -1){ 
    return 1000;
  
  } 
  else if (inputs > 1){
    return 2000; 
  }
  else {
    int pwm = (inputs * 500) + 1500;
    return pwm;
  }


}
