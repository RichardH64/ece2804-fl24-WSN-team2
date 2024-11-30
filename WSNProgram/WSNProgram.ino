#include <SoftwareSerial.h>
#include "WSNLibrary.h"

//===BEGIN PROGRAM===//
int pwmValue = 100;                         // Start with 50% duty cycle
unsigned long previousFeedbackMillis = 0;

void setupTimer1();

void PWM_run();

void setup() {
  pinMode(PWM_VOLTAGEPIN, INPUT);
  pinMode(PWM_PIN1, OUTPUT);
  pinMode(PWM_PIN2, OUTPUT);

  setupTimer1();
} 

void loop() {
  unsigned long currentMillis = millis(); 
  if (currentMillis - previousFeedbackMillis >= PWM_INTERVAL) {
    previousFeedbackMillis = currentMillis;
    PWM_run();
  }
}

void setupTimer1() {
  // Stop Timer1
  TCCR1A = 0;
  TCCR1B = 0;

  // Set Fast PWM mode, no prescaler (CS10 = 1)
  // WGM13 = 1, WGM12 = 1, WGM11 = 1, WGM10 = 0
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);

  // Set TOP value to 249 for exact 64kHz
  ICR1 = 249;  
}

void PWM_run() {
  int sensorValue = analogRead(PWM_VOLTAGEPIN);                                     // Step 1:  Read the voltage from the A0 pin (after the voltage divider)
  float measuredVoltageAtA0 = (sensorValue * 5.0) / 1023.0;                         // Step 2A: Convert the analog reading (0-1023) to voltage
  float measuredOutputVoltage = measuredVoltageAtA0 / PWM_VOLTAGEDIVIDERRATIO;      // Step 2B: Convert the measured voltage to the actual voltage

  if (measuredOutputVoltage < PWM_VOLTAGETHRESHOLDRESET) {
    pwmValue = PWM_MAX - PWM_MIN;
  }
  else if (measuredOutputVoltage < PWM_VOLTAGETHRESHOLDDOWN) {                           // Step 3:  Adjust the PWM duty cycle
    pwmValue += 1; // Increase duty cycle
  } 
  else if (measuredOutputVoltage > PWM_VOLTAGETHRESHOLDUP) {
    pwmValue -= 1; // Decrease duty cycle
  }
  
  pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);                                 // Step 4:  Constrain PWM value between MINPWM and MAXPWM
  OCR1A = pwmValue;                                                                 // Step 5A: Update Compare Match Values (Update duty cycle for Pin 9)
  OCR1B = pwmValue;                                                               // Step 5B: Update Compare Match Values (Update duty cycle for Pin 10)
}
