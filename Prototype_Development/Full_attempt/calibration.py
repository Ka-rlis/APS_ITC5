#include "STM32_PWM.h"

#define MOTOR_LEFT_PIN     D15  // Corresponds to PB8
#define MOTOR_RIGHT_PIN    D14  // Corresponds to PB9

// Standard ESCs use a frequency of 50Hz
#define PWM_FREQUENCY      50

// Define the pulse widths for calibration
#define MAX_THROTTLE       2000  // Maximum throttle pulse width in microseconds
#define MIN_THROTTLE       1000  // Minimum throttle pulse width in microseconds
#define NEUTRAL_THROTTLE   1500  // Neutral throttle pulse width in microseconds

HardwareTimer* motorLeftTimer = nullptr;
HardwareTimer* motorRightTimer = nullptr;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting ESC calibration...");

  // Initialize the PWM for the left motor (D15)
  motorLeftTimer = new HardwareTimer(TIM1); // Use appropriate timer for D15
  motorLeftTimer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_LEFT_PIN);
  motorLeftTimer->setOverflow(PWM_FREQUENCY, HERTZ_FORMAT);

  // Initialize the PWM for the right motor (D14)
  motorRightTimer = new HardwareTimer(TIM2); // Use appropriate timer for D14
  motorRightTimer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_RIGHT_PIN);
  motorRightTimer->setOverflow(PWM_FREQUENCY, HERTZ_FORMAT);

  // Start with Maximum Throttle
  motorLeftTimer->setCaptureCompare(1, MAX_THROTTLE, MICROSEC_COMPARE_FORMAT);
  motorRightTimer->setCaptureCompare(1, MAX_THROTTLE, MICROSEC_COMPARE_FORMAT);
  motorLeftTimer->resume();
  motorRightTimer->resume();
  Serial.println("Setting Maximum Throttle...");
  delay(2000); // Wait for the ESC to recognize the max throttle

  // Set to Minimum Throttle
  motorLeftTimer->setCaptureCompare(1, MIN_THROTTLE, MICROSEC_COMPARE_FORMAT);
  motorRightTimer->setCaptureCompare(1, MIN_THROTTLE, MICROSEC_COMPARE_FORMAT);
  Serial.println("Setting Minimum Throttle...");
  delay(2000); // Wait for the ESC to recognize the min throttle

  // Set to Neutral (Idle) Throttle
  motorLeftTimer->setCaptureCompare(1, NEUTRAL_THROTTLE, MICROSEC_COMPARE_FORMAT);
  motorRightTimer->setCaptureCompare(1, NEUTRAL_THROTTLE, MICROSEC_COMPARE_FORMAT);
  Serial.println("Setting Neutral Throttle...");
  delay(2000); // Wait for the ESC to recognize the neutral throttle

  Serial.println("ESC Calibration is complete!");
}

void loop() {
  // ESC calibration is done in setup, nothing to do in loop
}
