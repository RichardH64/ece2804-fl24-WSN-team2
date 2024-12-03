#include <SoftwareSerial.h>
#include "WSNLibrary.h"

//===BEGIN PROGRAM===//
unsigned long previousFeedbackMillis = 0;
unsigned long previousBluetoothMillis = 0;

bool timer1AlreadyEnabled = true;
SoftwareSerial bluetooth(BT_RXDPIN, BT_TXDPIN);

volatile int pwmValue = 100;                         // Start with 50% duty cycle
volatile bool isInLowPowerMode = false;
volatile bool timer2_30SecondsPassed = false;
volatile unsigned long timer2_OverflowCount = 0;

void setupTimer1();
void setupTimer2();

void enableTimer1();
void disableTimer1();

void handleBluetoothPower(bool powerOn);

float getTemperature();
float getSolarVoltage();

void checkLowPowerMode();
void enterLowPowerMode();

void PWM_run();
void BT_run();

void setup() {
  pinMode(PWM_VOLTAGEPIN, INPUT);
  pinMode(BT_VOLTAGEPIN, INPUT);
  pinMode(SOLAR_VOLTAGEPIN, INPUT);
  pinMode(PWM_PIN1, OUTPUT);
  pinMode(PWM_PIN2, OUTPUT);

  bluetooth.begin(9600);

  setupTimer1();
  setupTimer2();

  sei();
} 

void loop() {
  checkLowPowerMode();
  if (isInLowPowerMode) {
    if (timer2_30SecondsPassed) { // Ends the loop prematurely
      timer2_30SecondsPassed = false;
      handleBluetoothPower(true);
      BT_run();
      handleBluetoothPower(false);
    }
    enterLowPowerMode();
    return;
  }

  // Run Regular Cycle
  unsigned long currentMilliseconds = millis();

  if (currentMilliseconds - previousFeedbackMillis >= PWM_INTERVAL){
    previousFeedbackMillis = currentMilliseconds;
    PWM_run();
  }

  if (currentMilliseconds - previousBluetoothMillis >= BT_INTERVAL) {
    previousBluetoothMillis = currentMilliseconds;
    BT_run();
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

void setupTimer2() {
  TCCR2A = 0;
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 = (1 << TOIE2);
  TCNT2 = 0;
  timer2_OverflowCount = 0;
}

void enableTimer1() {
  // Stop Timer1
  TCCR1A = 0;
  TCCR1B = 0;

  // Set Fast PWM mode, no prescaler (CS10 = 1)
  // WGM13 = 1, WGM12 = 1, WGM11 = 1, WGM10 = 0
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);

  // Set TOP value to 249 for exact 64kHz
  ICR1 = 249;  

  OCR1A = pwmValue;                                                                 // Step 5A: Update Compare Match Values (Update duty cycle for Pin 9)
  OCR1B = pwmValue;  
}

void disableTimer1() {
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  OCR1A = 0;
  OCR1B = 0;
}

void handleBluetoothPower(bool powerOn) {
  if (powerOn) {
    bluetooth.write("AT+RESET\r\n");
    delay(100); 
  } 
  else {
    bluetooth.write("AT+SLEEP\r\n");
    delay(100);
  }
}

float getTemperature() {
  int sensorValue = analogRead(BT_VOLTAGEPIN);
  float measuredVoltage = (sensorValue * 5.0) / 1023.0;

  float R2 = BT_R1 * (5.0 / measuredVoltage - 1.0);
  float logR2 = log(R2);

  float tempK = 1.0 / (BT_C1 + (BT_C2 * logR2) + (BT_C3 * logR2 * logR2 * logR2));

  return tempK;
}

float getSolarVoltage() {
  int sensorValue = analogRead(SOLAR_VOLTAGEPIN);
  float measuredVoltage = (sensorValue * 5.0) / 1023.0;
  float measuredOutputVoltage = measuredVoltage / SOLAR_VOLTAGEDIVIDERRATIO;
  return measuredOutputVoltage;
}

void checkLowPowerMode() {
  float voltage = getSolarVoltage();
  if (voltage <= SOLAR_VOLTAGETHRESHOLDDOWN) {
    isInLowPowerMode = true;
    timer1AlreadyEnabled = false;
    disableTimer1();
    handleBluetoothPower(false);
  }
  else if (voltage > SOLAR_VOLTAGETHRESHOLDUP){
    isInLowPowerMode = false;
    if (!timer1AlreadyEnabled) { // So that Timer1 Isn't constantly enabling
      timer1AlreadyEnabled = true;
      enableTimer1();
      handleBluetoothPower(true);
    }
  }
}

void enterLowPowerMode() {
  SMCR = (0 << SM2) | (0 << SM1) | (0 << SM0) | (1 << SE); // Setting the SMCR to LPM (Idle Mode) // First 3 set the SMCR to LPM, SE enables LPM

  // Enter sleep mode
  __asm__ __volatile__("sleep");
  
  // MCU wakes up here after an interrupt
  SMCR &= ~(1 << SE); // Disable sleep after wake-up
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

void BT_run() {
  float tempK = getTemperature();
  float tempC = tempK - 273.15;
  float tempF = (tempC * 9.0) / 5.0 + 32.0;

  bluetooth.println("Temperature:");
  bluetooth.print(tempC, 2); bluetooth.println(" °C");
  bluetooth.print(tempF, 2); bluetooth.println(" °F");
}

ISR(TIMER2_OVF_vect) {
  timer2_OverflowCount++;

  if (timer2_OverflowCount >= 1832) {
    timer2_OverflowCount = 0;
    timer2_30SecondsPassed = true;
  }
}
