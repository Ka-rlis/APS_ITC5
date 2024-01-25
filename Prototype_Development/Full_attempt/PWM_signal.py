#include "STM32_PWM.h"

// Ensure the core library is compatible with your board
#if !(defined(STM32F4))
  #error This code is designed to run on STM32F4 platform! Please check your Tools->Board setting.
#endif

// Use pin D15, which is the label in the Arduino environment corresponding to PB8 in the MCU
#define PWM_PIN         D15

// Define the PWM frequency and duty cycle
#define PWM_FREQUENCY   5000   // 5kHz frequency
#define PWM_DUTY_CYCLE  50     // 50% duty cycle

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial);

  Serial.print(F("\nStarting PWM_Single_D15 on "));
  Serial.println(BOARD_NAME);
  Serial.println(STM32_PWM_VERSION);

  // Initialize pin D15 as a PWM output
  pinMode(PWM_PIN, OUTPUT);

  // Retrieve the timer instance and channel associated with pin D15
  PinName pinNameToUse = digitalPinToPinName(PWM_PIN);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pinNameToUse, PinMap_PWM);

  if (Instance != nullptr) {
    // Get the timer channel associated with the pin
    uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pinNameToUse, PinMap_PWM));

    // Create a new instance of HardwareTimer
    HardwareTimer *MyTim = new HardwareTimer(Instance);

    // Initialize PWM with the specified frequency and duty cycle
    MyTim->setPWM(channel, PWM_PIN, PWM_FREQUENCY, PWM_DUTY_CYCLE);

    Serial.println(F("PWM initialized on D15 with 5kHz frequency and 50% duty cycle."));
  } else {
    Serial.println(F("ERROR: PWM initialization failed!"));
  }
}

void loop() {
  // Nothing to do here, PWM signal is generated in hardware
}
