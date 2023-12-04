

// Includes.
#include <Arduino.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
#include <LSM6DSLSensor.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

#define DEV_I2C Wire
#define SerialPort Serial

// Components.
LSM6DSLSensor AccGyr(&DEV_I2C);
LSM303AGR_ACC_Sensor Acc2(&DEV_I2C);
LSM303AGR_MAG_Sensor Mag(&DEV_I2C);



// pi
#define PI 3.1415926535897932384626433832795

// mdps to radians/s constant
#define mddps_to_rads 0.001*(PI/180)

// esbjerg G constant
# define g 9.81553035

// mg to m/s squared
#define mg_to_si 0.001 * g

// accelero meter angles
int32_t Qacc1 = 0;
int32_t Qacc2 = 0;
float alpha = 0.1;

// function dec
void sensor_all();
void kalman();


// Measured sampling time





int32_t accelerometer[3];
int32_t gyroscope[3];



float deltaTime;
unsigned long lastTime;

Matrix<3,1,float> Sensor_angle_Gyro = { 
    0, 
    0,
    0
};


Matrix<2,1,float> Sensor_angle_Acc = { 
    0, 
    0,
};

Matrix<3,1,float> Sensor_Acc = { 
    0, 
    0,
    0,
};

Matrix<2,1,float> Sensor_Mag = { 
    0, 
    0,
};

Matrix<1, 1, float> Sensor_angle_Mag{
  0,
};

/*
Matrix<2,1,float> ob_prev = {
  0,
  0
};

Matrix<2,1,float> ob_next = {
  0, 
  0
};

Matrix<2, 2, float> A = {
  1,0,
  0,1
};

Matrix<2, 2, float> B = {
  0 , 0,
  0 , 0
};

Matrix<2, 2, float> L = {
  0.2 , 0,
  0 , 0.2
};
*/

Matrix<3, 1, float> x_est{
  0,
  0,
  0
};

Matrix<3, 1, float> x_est_next{
  0,
  0,
  0
};




void sensor_all(){
  // Get sensor data in mdps and mg
  AccGyr.Get_X_Axes(accelerometer);
  AccGyr.Get_G_Axes(gyroscope);
    // Read magnetometer LSM303AGR.
  int32_t magnetometer[3];
  Mag.GetAxes(magnetometer);

  // Get raw sensor data as matrix in rad
  Sensor_angle_Gyro(0, 0) += (gyroscope[0] * mddps_to_rads) * deltaTime;
  Sensor_angle_Gyro(1, 0) += (gyroscope[1] * mddps_to_rads) * deltaTime;
  Sensor_angle_Gyro(2, 0) += (gyroscope[2] * mddps_to_rads) * deltaTime;

  // Get sensor data as matrix in rad/s squared
  Sensor_Acc(0, 0) = accelerometer[0] * mg_to_si;
  Sensor_Acc(1, 0) = accelerometer[1] * mg_to_si;
  Sensor_Acc(2, 0) = accelerometer[2] * mg_to_si;

  // magnetometer data
  Sensor_Mag(0, 0) = magnetometer[0];
  Sensor_Mag(1, 0) = magnetometer[1];




  // roll n pitch acc
  Sensor_angle_Acc(0, 0) = atan2(Sensor_Acc(0, 0),Sensor_Acc(2, 0));
  Sensor_angle_Acc(1, 0) = atan2(Sensor_Acc(2, 0), g);

  // yaw mag netto
  Sensor_angle_Mag(0, 0) =  atan2(Sensor_Mag(0, 0),Sensor_Mag(1, 0));

};


void ob() { 


  x_est_next(1, 0) = alpha * (x_est(1, 0) + Sensor_angle_Gyro(0, 0)) + (1 - alpha) * Sensor_angle_Acc(0, 0);
  x_est_next(2, 0) = alpha * (x_est(2, 0) + Sensor_angle_Gyro(2, 0)) + (1 - alpha) * Sensor_angle_Mag(0, 0);

  // Compute x_est_next = A_obs * x_est + B_obs * [u; y_meas]
  //BLA::Matrix<2, 1> x_est_next = (A * x_est) + (Sensor_angle_Gyro)+ L*(Sensor_angle_Gyro - Sensor_angle_Acc);

  // Update the x_est to the current calc
  x_est(1, 0) = x_est_next(1, 0);
  //Serial.println();
};



void setup() {
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(9600);
  
  // Initialize I2C bus.
  DEV_I2C.begin();

  // Initialize components.
  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();
  Acc2.begin();
  Acc2.Enable();
  Mag.begin();
  Mag.Enable();
}

void loop() {
  
  // Read accelerometer LSM303AGR.
  //int32_t accelerometer2[3];
  //Acc2.GetAxes(accelerometer2);
  
  // Read magnetometer LSM303AGR.
  //int32_t magnetometer[3];
  //Mag.GetAxes(magnetometer);

  // get all sensor data 
    unsigned long currentTime = millis();
    deltaTime = (currentTime - lastTime) / 1000.0; // Time difference in seconds
    lastTime = currentTime;
  sensor_all();

  ob();
  
}

  
