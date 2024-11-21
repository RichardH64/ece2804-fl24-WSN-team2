#include "WSNLibrary.h"
#include <SoftwareSerial.h>

// INTERRUPT RELATED SECTION
volatile bool timer2_30SecondsPassed = false;
volatile unsigned long timer2_OverflowCount = 0;

//===BEGIN PROGRAM===//
SoftwareSerial bluetooth(BT_RXDPIN, BT_TXDPIN);   // RX, TX


void setupTimer1();
void setupTImer2();

float getTemp();
float getSolarVoltage();

void PWM_run();
void BT_run();

void setup() {

}

void loop() {

}

ISR(TIMER1_COMPA_vect) {

}

ISR(TIMER2_OVF_vect) {
  timer2_OverflowCount++; // Increment overflow counter

  // Timer2 overflows approximately every 16.384 ms with 16MHz clock and 1024 prescaler
  if (timer2_OverflowCount >= 1832) {   // 30 Seconds
    timer2_OverflowCount = 0;           // Reset overflow counter
    timer2_30SecondsPassed = true;
  }
}