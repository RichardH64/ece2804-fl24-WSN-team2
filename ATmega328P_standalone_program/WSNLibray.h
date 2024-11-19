#ifndef WSN_LIBRARY_H
#define WSN_LIBRARY_H

//===PWM Constants===//
#define VOLTAGEPIN          (uint8_t)                    A0 // Pin to read the voltage from the voltage divider
#define PWMPIN1             (uint8_t)                     9 // PWM output pin to control MOSFET (OC1A)
#define PWMPIN2             (uint8_t)                    10 // Secondary PWM pin (OC1B)
#define TARGETVOLTAGE       (float)                    10.5 // Desired output voltage
#define MINPWM              (int)                        50 // Minimum PWM for 0% duty cycle
#define MAXPWM              (int)                       249 // Maximum PWM for 100% duty cycle

//===Bluetooth & Thermistor Constants===/
#define THERMISTORPIN       (uint8_t)                    A1 // Pin connected to the voltage divider for temperature
#define RXDPin              (uint8_t)                     8 // Pin connected to RXD
#define TXDPin              (uint8_t)                     7 // Pin connected to TXD
#define B                   (float)                    3950 // B-parameter
#define R1                  (float)                   10000 // Fixed resistor value in ohms (10kΩ)
#define C1                  (float)         1.009249522e-03 // Steinhart-Hart coefficient 1
#define C2                  (float)         2.378405444e-04 // Steinhart-Hart coefficient 2
#define C3                  (float)         2.019202697e-07 // Steinhart-Hart coefficient 3

//===Time Controls (Non Power-Save)===/
#define FEEDBACKINTERVAL    (unsigned long)              50 // 50 ms for boost converter feedback
#define BLUETOOTHINTERVAL   (unsigned long)            1000 // 1000 ms (1 second) for Bluetooth transmission

// Voltage divider constants (for R3 = 33kΩ, R4 = 8.2kΩ
#define VOLTAGEDIVIDERRATIO (float)      8.2 / (33.0 + 8.2) // = 0.2
#define LOWVOLTAGETHRESHOLD (int)                         7 // If voltage drops below this, reset PWM

//===Sleep Mode constants===//
#define SOLARPANELPIN            (uint8_t)               A2 // Pin to read the voltage from the solar panel
#define SOLARPANELTHRESHOLDDOWN  (float)                  3 // Voltage Threshold before going low power
#define SOLARPANELTHRESHOLDUP    (float)                4.1 //
#define SOLARVOLTAGEDIVIDERRATIO (float) 120.0 / (120.0 + 220.0) // = 0.353


#endif