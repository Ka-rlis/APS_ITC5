Code which maybe works : #include <Arduino.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
#include <LSM6DSLSensor.h>
#include <Wire.h> 
#include <BasicLinearAlgebra.h>
#include "STM32_PWM.h"

// Define the pins for the motors
#define MOTOR_LEFT_PIN   D15  // Corresponds to PB8
#define MOTOR_RIGHT_PIN  D14  // Corresponds to PB9

// Define the PWM frequency for servo motors
#define PWM_FREQUENCY   50    // Standard servo frequency is 50Hz
#define SAMPLETODISCARD 15
using namespace BLA;




#define ALGO_FREQ  100U /* Algorithm frequency 100Hz */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
#define MOTION_FX_ENGINE_DELTATIME  0.01f
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f

BLA::Matrix<3, 1, float> Sensor_angle_Gyro;
BLA::Matrix<3, 1, float> Sensor_Acc;
BLA::Matrix<2, 1, float> Sensor_Mag;
BLA::Matrix<2, 1, float> Sensor_angle_Acc;
BLA::Matrix<1, 1, float> Sensor_angle_Mag;
float mddps_to_rads = 0.001 * 3.14159 / 180;  // Convert from mdps to radians per second
float mg_to_si = 0.00981;  // Convert from mg to m/s^2
float alpha = 0.98;  // Complementary filter parameter for roll and pitch
float alpha2 = 0.98;  // Complementary filter parameter for yaw

// Declare global variables for motor timers
HardwareTimer *leftMotorTimer;
HardwareTimer *rightMotorTimer;
// Other necessary global variables and matrices

Matrix<2, 1, float> ObserverUpdate(const Matrix<2, 1, float>& y);
Matrix<2, 1, float> IntegError(const Matrix<2, 1, float>& y, const Matrix<2, 1, float>& r);
void InputXD(const Matrix<2,1, float> u_e);
void sensor_all();
void complementaryFilter();

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
    0.2040, 0,
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


static volatile int sampleToDiscard = SAMPLETODISCARD;
static int discardedCount = 0;

static volatile uint32_t TimeStamp = 0;

LSM6DSLSensor AccGyr(&Wire); 
LSM303AGR_MAG_Sensor Mag(&Wire);  
LSM303AGR_ACC_Sensor Acc2(&Wire);  

float deltaTime;
unsigned long lastTime;

// Global variables for sensor data
int32_t accelerometer[3];
int32_t gyroscope[3];
int32_t magnetometer[3];

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
// PWM-related definitions
HardwareTimer *leftMotorTimer;
HardwareTimer *rightMotorTimer;
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

  // Calculate pulse width for each motor based on control signals
  int pulseWidthLeft = mapf(input(0,0), -1.00, 1.00, 1000, 2000);
  int pulseWidthRight = mapf(input(1,0), -1.00, 1.00, 1000, 2000);


  
}


void saturation(){
  input(0, 0) = (input(0, 0) > 1.0f) ? 1.0f : ((input(0, 0) < -1.0f) ? -1.0f : input(0, 0));
  input(1, 0) = (input(1, 0) > 1.0f) ? 1.0f : ((input(1, 0) < -1.0f) ? -1.0f : input(1, 0));

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

  // Magnetometer data
  Sensor_Mag(0, 0) = magnetometer[0];
  Sensor_Mag(1, 0) = magnetometer[1];

  // Calculate accelerometer angles (roll and pitch)
  Sensor_angle_Acc(0, 0) = atan2(Sensor_Acc(1, 0), Sensor_Acc(2, 0)); // Roll
  Sensor_angle_Acc(1, 0) = atan2(-Sensor_Acc(0, 0), sqrt(Sensor_Acc(1, 0) * Sensor_Acc(1, 0) + Sensor_Acc(2, 0) * Sensor_Acc(2, 0))); // Pitch

  // Calculate magnetometer angle (yaw)
  Sensor_angle_Mag(0, 0) = atan2(Sensor_Mag(0, 0), Sensor_Mag(1, 0));
}

void complementaryFilter() {
  // Update the estimated angles using a complementary filter approach
  x_est_next(0, 0) = alpha * (x_est(0, 0) + Sensor_angle_Gyro(0, 0)) + (1 - alpha) * Sensor_angle_Acc(0, 0); // Roll
  x_est_next(1, 0) = alpha * (x_est(1, 0) + Sensor_angle_Gyro(1, 0)) + (1 - alpha) * Sensor_angle_Acc(1, 0); // Pitch
  x_est_next(2, 0) = alpha2 * (x_est(2, 0) + Sensor_angle_Gyro(2, 0)) + (1 - alpha2) * Sensor_angle_Mag(0, 0); // Yaw
  
  x_est = x_est_next;
}



void setup() {
    // Initialize Serial
    Serial.begin(9600);
    while (!Serial) yield(); 

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

    // Initialize PWM for motors
    leftMotorTimer = new HardwareTimer(TIM3);  // Replace TIM1 with the correct timer for your board
    rightMotorTimer = new HardwareTimer(TIM4); // Replace TIM2 with the correct timer for your board

    leftMotorTimer->setPWM(1, MOTOR_LEFT_PIN, PWM_FREQUENCY, 0);   // Initialize with 0% duty cycle
    rightMotorTimer->setPWM(1, MOTOR_RIGHT_PIN, PWM_FREQUENCY, 0); // Initialize with 0% duty cycle

}
void loop() {
    // Main control loop
    unsigned long currentTime = millis();
    deltaTime = (currentTime - lastTime) / 1000.0f; // Time difference in seconds
    lastTime = currentTime;

    // Sensor data acquisition and processing
    sensor_all();
    complementaryFilter();
   
    Serial.print(x_est_next(1, 0));
    Serial.print("  ");
    Serial.print(x_est_next(2, 0));
    Serial.print("  ");
    Serial.println(x_est_next(3, 0));

    // PWM Duty cycle adjustment
    int pulseWidthLeft = mapf(input(0,0), -1.00, 1.00, 1000, 2000);
    int pulseWidthRight = mapf(input(1,0), -1.00, 1.00, 1000, 2000);

    // Convert pulse width to duty cycle percentage
    int dutyCycleLeft = map(pulseWidthLeft, 1000, 2000, 0, 100);
    int dutyCycleRight = map(pulseWidthRight, 1000, 2000, 0, 100);

    leftMotorTimer->setPWM(1, MOTOR_LEFT_PIN, PWM_FREQUENCY, dutyCycleLeft);
    rightMotorTimer->setPWM(1, MOTOR_RIGHT_PIN, PWM_FREQUENCY, dutyCycleRight);

}
