#include "WSNLibrary.h"
#include <SoftwareSerial.h>

// INTERRUPT RELATED SECTION
volatile bool timer2_30SecondsPassed = false;
volatile unsigned long timer2_OverflowCount = 0;

//===BEGIN PROGRAM===//
SoftwareSerial bluetooth(BT_RXDPIN, BT_TXDPIN);   // RX, TX

bool isInLowPowerMode = false;

void setupTimer1();
void setupTImer2();

float getTemp();
float getSolarVoltage();

void checkLowPowerMode();

void PWM_run();
void BT_run();

void setup() {

}

void loop() {
  checkLowPowerMode();
  if (isInLowPowerMode) {
    // Set to sleep mode for the rest of the code
    if (!timer2_30SecondsPassed) {
      return;
    }
  }

  BT_run();
}

void setupTimer1() {

}

void setupTImer2() {
  overflowCount = 0;                                  // Reset overflow counter
  TCCR2A = 0;                                         // Set Timer2 to Normal Mode (overflow interrupt)
  TCCR2B = 0;                                         // Refresh
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);   // Set the prescaler to 1024
  TIMSK2 = (1 << TOIE2);                              // Enable Timer2 Overflow Interrupt
  TCNT2 = 0;                                          // Initialize Timer2
}

float getTemp() {

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

}

void BT_run() {

}

ISR(TIMER1_COMPA_vect) {
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