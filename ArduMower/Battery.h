/*
 * Battery.h
 *
 *  Created on: Aug 24, 2016
 *      Author: ove
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include "Arduino.h"

class Battery
{
  public:
    boolean monitored { false };                // Monitor battery and charge voltage?
    float batGoHomeIfBelow { 11.8 };            // Drive home voltage (Volt)
    float batSwitchOffIfBelow { 10.8 };         // Switch off if below voltage (Volt)
    int batSwitchOffIfIdle { 1 };               // Switch off battery if idle for x minutes
    float batFactor { 0.495 };                  // Battery conversion factor
    float batChgFactor { 0.495 };               // Battery conversion factor
    float batFull { 14.7 };                     // Battery reference Voltage (fully charged)
    float batChargingCurrentMax { 1.6 };        // Maximum current your charger can deliver
    float batFullCurrent { 0.3 };               // Current flowing when battery is fully charged
    float startChargingIfBelow { 13.5 };        // Start charging if battery Voltage is below
    unsigned long chargingTimeout { 12600000 }; // Cafety timer for charging (ms) 12600000 = 3.5hrs
    float chgSenseZero { 511 };                 // Charge current sense zero point //TODO: autocalibrate?
    float chgFactor { 39 };                     // Charge current conversion factor
    float chgSense { 185.0 };                   // Sensitivity of the charging current sensor (mV/A) (For ACS712 5A = 185)
    byte chgChange { 0 };                       // Reading reversal from - to + 1 or 0
    // sensor output console      (chgSelection = 0)
    // settings for ACS712 5A     (chgSelection = 1 / chgSenseZero ~ 511 / chgFactor = 39 / chgSense = 185.0 / chgChange = 0 oder 1 (je nach Stromrichtung) / chgNull = 2)
    // settings for INA169 board  (chgSelection = 2)
    byte chgSelection { 2 };                    // Senor selection
    int chgNull { 2 };                          // Zero crossing charge current sensor

    void setup(const uint8_t pinVoltage,
               const uint8_t pinChargeVoltage,
               const uint8_t pinChargeCurrent,
               const uint8_t pinChargeRelay,
               const uint8_t pinBatterySwitch);

    float getVoltage() const
    {
      return voltage;
    }

    float getCapacity() const
    {
      return batCapacity;
    }
    void clearCapacity()
    {
      batCapacity = 0;
    }

    float getLastTimeCapacity() const
    {
      return lastTimeBatCapacity;
    }
    void updateLastTimeCapacity()
    {
      lastTimeBatCapacity = batCapacity;
    }

    bool isMonitored() const
    {
      return monitored;
    }
    float getChargeCurrent() const
    {
      return chgCurrent;
    }

    float getChargeVoltage() const
    {
      return chgVoltage;
    }
    void setChargeRelay(bool state)
    {
      digitalWrite(pinChargeRelay, state);
    }
    void setBatterySwitch(bool state)
    {
      digitalWrite(pinBatterySwitch, state);
    }

    void read(void);
    bool isTimeToRead(void);
    bool isTimeToCheck(void);

  private:
    static const uint8_t TIME_BETWEEN_READS { 100 };
    static const uint16_t TIME_BETWEEN_CHECKS { 1000 };

    uint8_t pinVoltage;
    uint8_t pinChargeVoltage;
    uint8_t pinChargeCurrent;
    uint8_t pinChargeRelay;
    uint8_t pinBatterySwitch;

    unsigned long nextTimeCheck { };
    // --------- charging ---------------------------------
    int batADC;
    float voltage;  // battery voltage (Volt)
    float batCapacity; // battery capacity (mAh)
    float chgVoltage;  // charge voltage (Volt)
    float chgCurrent;  // charge current  (Ampere)
    float lastTimeBatCapacity;

};

#endif /* BATTERY_H_ */
