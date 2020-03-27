/*
 * Battery.h
 *
 *  Created on: Aug 24, 2016
 *      Author: ove
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include "Arduino.h"

typedef struct
{
    // --------- pins ---------------------------------
    uint8_t pinVoltage;
    uint8_t pinChargeVoltage;
    uint8_t pinChargeCurrent;
    uint8_t pinChargeRelay;
    uint8_t pinBatterySwitch;
    // --------- configuration ---------------------------------
    bool monitored;                // Monitor battery and charge voltage?
    float batGoHomeIfBelow;            // Drive home voltage (Volt)
    float batSwitchOffIfBelow;         // Switch off if below voltage (Volt)
    int batSwitchOffIfIdle;               // Switch off battery if idle for x minutes
    float batFactor;                  // Battery conversion factor
    float batChgFactor;               // Battery conversion factor
    float batFull;                     // Battery reference Voltage (fully charged)
    float batChargingCurrentMax;        // Maximum current your charger can deliver
    float batFullCurrent;               // Current flowing when battery is fully charged
    float startChargingIfBelow;        // Start charging if battery Voltage is below
    unsigned long chargingTimeout; // Safety timer for charging (ms) 12600000 = 3.5hrs
    float chgSenseZero;                 // Charge current sense zero point //TODO: autocalibrate?
    float chgFactor;                     // Charge current conversion factor
    float chgSense;                   // Sensitivity of the charging current sensor (mV/A) (For ACS712 5A = 185)
    byte chgChange;                       // Reading reversal from - to + 1 or 0
    // sensor output console      (chgSelection = 0)
    // settings for ACS712 5A     (chgSelection = 1 / chgSenseZero ~ 511 / chgFactor = 39 / chgSense = 185.0 / chgChange = 0 oder 1 (je nach Stromrichtung) / chgNull = 2)
    // settings for INA169 board  (chgSelection = 2)
    byte chgSelection;                    // Senor selection
    int chgNull;                          // Zero crossing charge current sensor
    // --------- charging ---------------------------------
    int batADC;
    float voltage;  // battery voltage (Volt)
    float batCapacity; // battery capacity (mAh)
    float chgVoltage;  // charge voltage (Volt)
    float chgCurrent;  // charge current  (Ampere)
    float lastTimeBatCapacity;
} Battery;

void battery_setup(const uint8_t pinVoltage,
                   const uint8_t pinChargeVoltage,
                   const uint8_t pinChargeCurrent,
                   const uint8_t pinChargeRelay,
                   const uint8_t pinBatterySwitch,
                   Battery* battery_p);

void battery_read(Battery* battery_p);

static inline
float battery_getVoltage(Battery* battery_p)
{
  return battery_p->voltage;
}

static inline
float battery_getCapacity(Battery* battery_p)
{
  return battery_p->batCapacity;
}

static inline
void battery_clearCapacity(Battery* battery_p)
{
  battery_p->batCapacity = 0;
}

static inline
float battery_getLastTimeCapacity(Battery* battery_p)
{
  return battery_p->lastTimeBatCapacity;
}

static inline
void battery_updateLastTimeCapacity(Battery* battery_p)
{
  battery_p->lastTimeBatCapacity = battery_p->batCapacity;
}

static inline
bool battery_isMonitored(Battery* battery_p)
{
  return battery_p->monitored;
}

static inline
float battery_getChargeCurrent(Battery* battery_p)
{
  return battery_p->chgCurrent;
}

static inline
float battery_getChargeVoltage(Battery* battery_p)
{
  return battery_p->chgVoltage;
}

static inline
void battery_setChargeRelay(bool state, Battery* battery_p)
{
  digitalWrite(battery_p->pinChargeRelay, state);
}

static inline
void battery_setBatterySwitch(bool state, Battery* battery_p)
{
  digitalWrite(battery_p->pinBatterySwitch, state);
}

#endif /* BATTERY_H_ */
