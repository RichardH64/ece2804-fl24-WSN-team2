#ifndef WSN_LIBRARY_H
#define WSN_LIBRARY_H

//===PWM Constants===//
#define PWM_VOLTAGEPIN                  (uint8_t)                    A0 // Pin to read the voltage from the voltage divider
#define PWM_PIN1                        (uint8_t)                     9 // PWM output pin to control MOSFET (OC1A)
#define PWM_PIN2                        (uint8_t)                    10 // Secondary PWM pin (OC1B)
#define PWM_MIN                         (int)                        74 // Minimum PWM for 0% duty cycle
#define PWM_MAX                         (int)                       174 // Maximum PWM for 100% duty cycle
#define PWM_VOLTAGEDIVIDERRATIO         (float)      8.2 / (33.0 + 8.2) // Voltage divider constants (for R3 = 33kΩ, R4 = 8.2kΩ // = 0.2 
#define PWM_LOWVOLTAGETHRESHOLD         (float)                     6.5 // If voltage drops below this, reset PWM
#define PWM_TARGETVOLTAGE               (float)                    10.5 // Desired output voltage

//===Bluetooth & Thermistor Constants===/
#define BT_VOLTAGEPIN                   (uint8_t)                    A1 // Pin connected to the voltage divider for temperature
#define BT_RXDPIN                       (uint8_t)                     8 // Pin connected to RXD
#define BT_TXDPIN                       (uint8_t)                     7 // Pin connected to TXD
#define BT_B                            (float)                    3950 // B-parameter
#define BT_R1                           (float)                   10000 // Fixed resistor value in ohms (10kΩ)
#define BT_C1                           (float)         1.009249522e-03 // Steinhart-Hart coefficient 1
#define BT_C2                           (float)         2.378405444e-04 // Steinhart-Hart coefficient 2
#define BT_C3                           (float)         2.019202697e-07 // Steinhart-Hart coefficient 3

//===Sleep Mode constants===//
#define SOLAR_VOLTAGEPIN                (uint8_t)               A2      // Pin to read the voltage from the solar panel
#define SOLAR_VOLTAGEDIVIDERRATIO       (float) 120.0 / (220.0 + 120.0) // Voltage divider constants (for R3 = 220kΩ, R4 = 120kΩ // = 0.353
#define SOLAR_VOLTAGETHRESHOLDDOWN      (float)                    3.65 // Voltage Threshold before going low power
#define SOLAR_VOLTAGETHRESHOLDUP        (float)                    4.10 //



//===Time Controls (Non Power-Save)===/
#define FEEDBACKINTERVAL                (unsigned long)              50 // 50 ms for boost converter feedback
#define BLUETOOTHINTERVAL               (unsigned long)            1000 // 1000 ms (1 second) for Bluetooth transmission


#endif