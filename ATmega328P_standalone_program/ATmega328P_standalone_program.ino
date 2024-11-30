#include "WSNLibrary.h"
#include <SoftwareSerial.h>

//===BEGIN PROGRAM===//
bool isInLowPowerMode = false;
SoftwareSerial bluetooth(BT_RXDPIN, BT_TXDPIN);   // RX, TX

// INTERRUPT RELATED SECTION
volatile bool timer2_30SecondsPassed = false;
volatile unsigned long timer2_OverflowCount = 0;

void setupTimer1();
void setupTImer2();

float getTemperature();
float getSolarVoltage();

void checkLowPowerMode();

void PWM_run();
void BT_run();

void setup() {
  pinMode(PWM_VOLTAGEPIN, INPUT);
  pinMode(BT_VOLTAGEPIN, INPUT);
  pinMode(SOLAR_VOLTAGEPIN, INPUT);
  pinMode(PWMPIN1, OUTPUT);
  pinMode(PWMPIN2, OUTPUT);

  setupTimer1();
  setupTImer2();
} 

void loop() {
  checkLowPowerMode();
  if (isInLowPowerMode) {
    // Set to sleep mode for the rest of the code
    if (!timer2_30SecondsPassed) {
      return;
    }
    timer2_30SecondsPassed = false;
  }

  BT_run();

  if (!isInLowPowerMode) {
    delay(1000);
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

  // Set TOP value to 499 for exact 64kHz
  ICR1 = 249;

  // Set initial PWM duty cycle
  OCR1A = pwmValue; // Compare value for OC1A
  OCR1B = pwmValue; // Compare value for OC1B

  // Enable Timer1 Compare Match Interrupt
  TIMSK1 = (1 << OCIE1A); // Enable compare match interrupt for OCR1A
}

void setupTImer2() {
  overflowCount = 0;                                                            // Reset overflow counter
  TCCR2A = 0;                                                                   // Set Timer2 to Normal Mode (overflow interrupt)
  TCCR2B = 0;                                                                   // Refresh
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);                             // Set the prescaler to 1024
  TIMSK2 = (1 << TOIE2);                                                        // Enable Timer2 Overflow Interrupt
  TCNT2 = 0;                                                                    // Initialize Timer2
}

float getTemperature() {
  int sensorValue = analogRead(BT_VOLTAGEPIN);
  float voltage = sensorValue * (5.0 / 1023.0);

  float R2 = BT_R1 * (5.0 / voltage - 1);

  float logR2 = log(R2);
  float temperatureK = 1.0 / (C1 + C2 * logR2 + C3 * logR2 * logR2 * logR2);

  return temperatureK;
}

float getSolarVoltage() {
  int sensorValue = analogRead(SOLAR_VOLTAGEPIN);
  float voltage = sensorValue * (5.0 / 1023.0);
  float voltageSolarPanel = voltage / SOLAR_VOLTAGEDIVIDERRATIO;
  return voltage;
}

void checkLowPowerMode() {
  float voltage = getSolarVoltage();
  if (voltage <= SOLAR_VOLTAGETHRESHOLDDOWN) {
    isInLowPowerMode = true;
  }
  else if (volateg >= SOLAR_VOLTAGETHRESHOLDUP) {
    isInLowPowerMode = false;
  }
}

void PWM_run() {
  int sensorValue = analogRead(VOLTAGEPIN);                                     // Step 1:  Read the voltage from the A0 pin (after the voltage divider)
  float measuredVoltageAtA0 = (sensorValue * 5.0) / 1023.0;                     // Step 2A: Convert the analog reading (0-1023) to voltage
  float measuredOutputVoltage = measuredVoltageAtA0 / VOLTAGEDIVIDERRATIO;      // Step 2B: Convert the measured voltage to the actual voltage

  if (measuredOutputVoltage < PWM_VOLTAGETHRESHOLDDOWN) {                       // Step 3:  Adjust the PWM duty cycle
    pwmValue += 1; // Increase duty cycle
  } else if (measuredOutputVoltage > PWM_VOLTAGETHRESHOLDUP) {
    pwmValue -= 1; // Decrease duty cycle
  }
  
  pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);                             // Step 4:  Constrain PWM value between MINPWM and MAXPWM
  OCR1A = pwmValue;                                                             // Step 5A: Update Compare Match Values (Update duty cycle for Pin 9)
  OCR1B = pwmValue;                                                             // Step 5:B Update Compare Match Values (Update duty cycle for Pin 10)
}

void BT_run() {
  float tempK = getTemperature();
  float tempC = tempK - 273.15;
  float tempF = (tempC * 9.0) / 5.0 + 32.0;

  bluetooth.println("Temperature:");
  bluetooth.print(tempC, 2); bluetooth.println(" °C");
  bluetooth.print(tempF, 2); bluetooth.println(" °F");
}

ISR(TIMER1_COMPA_vect) {
  if (isInLowPowerMode) {
    return;
  }
  PWM_run();
}

ISR(TIMER2_OVF_vect) {
  timer2_OverflowCount++; // Increment overflow counter

  // Timer2 overflows approximately every 16.384 ms with 16MHz clock and 1024 prescaler
  if (timer2_OverflowCount >= 1832) {   // 30 Seconds
    timer2_OverflowCount = 0;           // Reset overflow counter
    timer2_30SecondsPassed = true;
  }
}