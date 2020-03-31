/*
 * Battery.h
 *
 *  Created on: Aug 24, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>

class Battery
{
public:
  Battery() = default;
  Battery(uint8_t pinVoltage, uint8_t pinChargeVoltage,
      uint8_t pinChargeCurrent, uint8_t pinChargeRelay,
      uint8_t pinBatterySwitch)
  {
    setup(pinVoltage, pinChargeVoltage, pinChargeCurrent, pinChargeRelay,
        pinBatterySwitch);
  }
  void setup(uint8_t pinVoltage, uint8_t pinChargeVoltage,
      uint8_t pinChargeCurrent, uint8_t pinChargeRelay,
      uint8_t pinBatterySwitch);

  void read();

  float getVoltage()
  {
    return m_voltage;
  }

  float getCapacity()
  {
    return m_batCapacity;
  }

  void clearCapacity()
  {
    m_batCapacity = 0;
  }

  float getLastTimeCapacity()
  {
    return m_lastTimeBatCapacity;
  }

  void updateLastTimeCapacity()
  {
    m_lastTimeBatCapacity = m_batCapacity;
  }

  bool isMonitored()
  {
    return m_monitored;
  }

  float getChargeCurrent()
  {
    return m_chgCurrent;
  }

  float getChargeVoltage()
  {
    return m_chgVoltage;
  }

  void setChargeRelay(bool state)
  {
    digitalWrite(m_pinChargeRelay, state);
  }

  void setBatterySwitch(bool state)
  {
    digitalWrite(m_pinBatterySwitch, state);
  }

public:
  // --------- configuration ---------------------------------
  bool m_monitored { false };                // Monitor battery and charge voltage?
  float m_batGoHomeIfBelow {};            // Drive home voltage (Volt)
  float m_batSwitchOffIfBelow {};         // Switch off if below voltage (Volt)
  int m_batSwitchOffIfIdle {};               // Switch off battery if idle for x minutes
  float m_batFactor {};                  // Battery conversion factor
  float m_batChgFactor {};               // Battery conversion factor
  float m_batFull {};                     // Battery reference Voltage (fully charged)
  float m_batChargingCurrentMax {};        // Maximum current your charger can deliver
  float m_batFullCurrent {};               // Current flowing when battery is fully charged
  float m_startChargingIfBelow {};        // Start charging if battery Voltage is below
  float m_chgSenseZero {};                 // Charge current sense zero point //TODO: autocalibrate?
  float m_chgFactor {};                     // Charge current conversion factor
  unsigned long m_chargingTimeout {}; // Safety timer for charging (ms) 12600000 = 3.5hrs
  float m_chgSense {};                   // Sensitivity of the charging current sensor (mV/A) (For ACS712 5A = 185)
  byte m_chgChange {};                       // Reading reversal from - to + 1 or 0
  // sensor output console      (chgSelection = 0)
  // settings for ACS712 5A     (chgSelection = 1 / chgSenseZero ~ 511 / chgFactor = 39 / chgSense = 185.0 / chgChange = 0 oder 1 (je nach Stromrichtung) / chgNull = 2)
  // settings for INA169 board  (chgSelection = 2)
  byte m_chgSelection {};                    // Senor selection
  int m_chgNull {};                          // Zero crossing charge current sensor

private:
    // --------- pins ---------------------------------
    uint8_t m_pinVoltage {};
    uint8_t m_pinChargeVoltage {};
    uint8_t m_pinChargeCurrent {};
    uint8_t m_pinChargeRelay {};
    uint8_t m_pinBatterySwitch {};

    // --------- charging ---------------------------------
    int m_batADC {};
    float m_voltage {};  // battery voltage (Volt)
    float m_batCapacity {}; // battery capacity (mAh)
    float m_chgVoltage {};  // charge voltage (Volt)
    float m_chgCurrent {};  // charge current  (Ampere)
    float m_lastTimeBatCapacity {};
};
