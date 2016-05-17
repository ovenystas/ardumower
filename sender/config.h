/*
 * config.h
 *
 *  Created on: Mar 29, 2016
 *      Author: ove
 */

#ifndef CONFIG_H
#define CONFIG_H

// Select one config
//#define CONFIG_DEFAULT
#define CONFIG_MBOARD

#ifdef CONFIG_DEFAULT
  // --- MC33926 motor driver ---
  #define USE_DOUBLE_AMPLTIUDE    1         // uncomment to use +/- input voltage for amplitude (default),
  // comment to use only +input/GND voltage for amplitude

  #define PIN_IN1       9  // M1_IN1         (if using old L298N driver, connect this pin to L298N-IN1)
  #define PIN_IN2       2  // M1_IN2         (if using old L298N driver, connect this pin to L298N-IN2)
  #define PIN_PWM       3  // M1_PWM / nD2   (if using old L298N driver, leave open)
  #define PIN_ENABLE    5  // EN             (connect to motor driver enable)

  // motor driver fault pin
  #define PIN_FAULT     4  // M1_nSF
  #define USE_PERI_FAULT        0     // use pinFault for driver fault detection? (set to '0' if not connected!)

  // motor driver feedback pin (=perimeter open/close detection, used for status LED)
  #define USE_PERI_CURRENT      1     // use pinFeedback for perimeter current measurements? (set to '0' if not connected!)
  #define PIN_FEEDBACK A0  // M1_FB
  #define PERI_CURRENT_MIN    0.03     // minimum Ampere for perimeter-is-closed detection

  // ---- sender current control (via potentiometer) ----
  // sender modulates signal (PWM), based on duty-cycle set via this potentiometer
  #define PIN_POT      A3  // 100k potentiometer (current control)

  // ---- sender automatic standby (via current sensor for charger) ----
  // sender detects robot via a charging current through the charging pins
  #define USE_CHG_CURRENT       0     // use charging current sensor for robot detection? (set to '0' if not connected!)
  #define PIN_CHARGE_CURRENT    A2     // ACS712-05 current sensor OUT
  #define CHG_CURRENT_MIN   0.008      // minimum Ampere for charging detection

  // ---- sender status LED ----
  #define  PIN_LED 13  // ON: perimeter closed, OFF: perimeter open, BLINK: robot is charging
#endif


#ifdef CONFIG_MBOARD
  // --- MC33926 motor driver ---
  #define USE_DOUBLE_AMPLTIUDE   1  // uncomment to use +/- input voltage for amplitude (default),
  // comment to use only +input/GND voltage for amplitude

  #define PIN_IN1                7  // M1_IN1         (if using old L298N driver, connect this pin to L298N-IN1)
  #define PIN_IN2                8  // M1_IN2         (if using old L298N driver, connect this pin to L298N-IN2)
  //#define PIN_PWM                   // M1_PWM / nD2   (if using old L298N driver, leave open)
  #define PIN_ENABLE            10  // EN             (connect to motor driver enable)

  // motor driver fault pin
  //#define PIN_FAULT                 // M1_nSF
  #define USE_PERI_FAULT         0  // use pinFault for driver fault detection? (set to '0' if not connected!)

  // motor driver feedback pi n (=perimeter open/close detection, used for status LED)
//  #define USE_PERI_CURRENT       0  // use pinFeedback for perimeter current measurements? (set to '0' if not connected!)
//  #define PIN_FEEDBACK          A0  // M1_FB
//  #define PERI_CURRENT_MIN    0.03  // minimum Ampere for perimeter-is-closed detection

  // ---- sender current control (via potentiometer) ----
  // sender modulates signal (PWM), based on duty-cycle set via this potentiometer
//  #define PIN_POT               A3  // 100k potentiometer (current control)

  // ---- sender automatic standby (via current sensor for charger) ----
  // sender detects robot via a charging current through the charging pins
//  #define USE_CHG_CURRENT        0  // use charging current sensor for robot detection? (set to '0' if not connected!)
//  #define PIN_CHARGE_CURRENT    A2  // ACS712-05 current sensor OUT
//  #define CHG_CURRENT_MIN    0.008  // minimum Ampere for charging detection

  // ---- sender status LED ----
  #define  PIN_LED              13  // ON: perimeter closed, OFF: perimeter open, BLINK: robot is charging
#endif


#endif /* CONFIG_H */
