
#include <Arduino.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
#include <LSM6DSLSensor.h>
#include <Wire.h> 
#include <BasicLinearAlgebra.h>


using namespace BLA;

// Constants and Macros from your first code
#define ALGO_FREQ  100U 
#define ALGO_PERIOD  (1000U / ALGO_FREQ)
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f
#define STATE_SIZE  (size_t)(2432)
#define SAMPLETODISCARD  15
#define GBIAS_ACC_TH_SC  (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC  (2.0f*0.002f)
#define DECIMATION  1U
#define PWM_PIN_1 D3  // Check what pin
#define PWM_PIN_2 D5  // Check what pin
// Constants from your second code
#define PI 3.1415926
#define mddps_to_rads 0.001*(PI/180)
#define g 9.81553035
#define mg_to_si 0.001 * g
#define alpha 0.1
#define alpha2 0.3


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
-0.3536,   -0.3536,
-0.3536,    0.3536
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
BLA::Matrix<2, 1, float> r = {0.0f, 0.0f};
BLA::Matrix<2, 1, float> y = {0, 0};
BLA::Matrix<2, 1, float> x_est = {0, 0};
BLA::Matrix<2, 1, float> x_est_next = {0, 0};
BLA::Matrix<6, 1, float> curr_state = {0, 0, 0, 0, 0, 0};
BLA::Matrix<6, 1, float> next_state = {0, 0, 0, 0, 0, 0};
BLA::Matrix<2, 1, float> input = {0, 0};


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

  // magnetometer data
  Sensor_Mag(0, 0) = magnetometer[0];
  Sensor_Mag(1, 0) = magnetometer[1];

  // Calculate accelerometer angles (roll and pitch)
  Sensor_angle_Acc(0, 0) = atan2(Sensor_Acc(0, 0), Sensor_Acc(2, 0));
  Sensor_angle_Acc(1, 0) = atan2(Sensor_Acc(2, 0), g);

  // Calculate magnetometer angle (yaw)
  Sensor_angle_Mag(0, 0) = atan2(Sensor_Mag(0, 0), Sensor_Mag(1, 0));
}
void complementaryFilter() {
  // Update the estimated angles using a complementary filter approach
  x_est_next(0, 0) = 2 * tan(alpha * (x_est(0, 0) + Sensor_angle_Gyro(0, 0)) + (1 - alpha) * Sensor_angle_Acc(0, 0));
  //x_est_next(1, 0) = alpha * (x_est(1, 0) + Sensor_angle_Gyro(1, 0)) + (1 - alpha) * Sensor_angle_Acc(1, 0);
  x_est_next(1, 0) = alpha2 * (x_est(1, 0) + Sensor_angle_Gyro(2, 0)) + (1 - alpha2) * Sensor_angle_Mag(0, 0);
  
  

  x_est = x_est_next;
}

void loop() {

  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0; // Time difference in seconds
  lastTime = currentTime;

  sensor_all();

  complementaryFilter();
  ObserverUpdate(x_est_next); 
  InputXD(u_e); 
  saturation();
  Serial.print("raw: "); 
  Serial.print(input(0,0));
  Serial.print( "   " );
  Serial.print("raw2: ");
  Serial.print(input(1,0));
  Serial.print( "   " );
  Serial.print( "yaw: " );
  Serial.print(x_est_next(1,0));
  Serial.print( "   " );
  Serial.print( "sway: " );
  Serial.println(x_est_next(0,0));
  delay(10);
}


Matrix<2, 1, float>ObserverUpdate(const Matrix<2, 1, float>& y) {
BLA::Matrix<6, 1> L_y = L * (y - (C * curr_state));

    // Compute the input effect
    BLA::Matrix<6, 1> B_u = B * input;

    // Update equation with Euler integration
    BLA::Matrix<6, 1> next_state = (A * curr_state) + B_u + L_y ;

  // Update the x_est to the current calc
  curr_state = next_state;

  
  u_e = -K_f * next_state; 
  return u_e;
}
  
Matrix<2, 1, float> IntegError(const Matrix<2, 1, float>& y, const Matrix<2, 1, float>& r) {
    // Calculate the error
    BLA::Matrix<2, 1, float> Error = r - y;

    // Calculate the time difference in seconds

    // Integrate the error over time
    integralError += Error * deltaTime;

    // Apply the integral gain
    BLA::Matrix<2, 1, float> integralTerm = K_i * integralError;

    return integralTerm;
}

void InputXD(const Matrix<2,1, float> u_e){ 
  BLA::Matrix<2,1, float> integralErrors = IntegError(x_est_next, r);
  input = integralErrors + u_e; 
}
  
int pwmMap(float inputs) {
  if (inputs < -1){ 
    return 1000;
  
  } 
  else if (inputs > 1){
    return 2000; 
  }
  else {
    int pwm = (inputs * 100) + 1500;
    return pwm;
  }
}

void setupPWM() {
  pinMode(PWM_PIN_2, OUTPUT);
}

void writePWM1(int pwmValue) {
  analogWrite(PWM_PIN_1, pwmValue);
}

void writePWM2(int pwmValue) {
  analogWrite(PWM_PIN_2, pwmValue);
}

void saturation(){
  input(0, 0) = (input(0, 0) > 1.0f) ? 1.0f : ((input(0, 0) < -1.0f) ? -1.0f : input(0, 0));
  input(1, 0) = (input(1, 0) > 1.0f) ? 1.0f : ((input(1, 0) < -1.0f) ? -1.0f : input(1, 0));
}
