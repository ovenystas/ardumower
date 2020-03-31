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
  bool m_monitored { false };                   // Monitor battery and charge voltage?
  float m_batGoHomeIfBelow { 11.8f };           // Drive home voltage (Volt)
  float m_batSwitchOffIfBelow { 10.8f };        // Switch off if below voltage (Volt)
  int m_batSwitchOffIfIdle { 1 };               // Switch off battery if idle for x minutes
  float m_batFactor { 0.495f };                 // Battery conversion factor
  float m_batChgFactor { 0.495f };              // Battery conversion factor
  float m_batFull { 14.7f };                    // Battery reference Voltage (fully charged)
  float m_batChargingCurrentMax { 1.6f };       // Maximum current the charger can deliver
  float m_batFullCurrent { 0.3f };              // Current flowing when battery is fully charged
  float m_startChargingIfBelow { 13.5f };       // Start charging if battery Voltage is below
  float m_chgSenseZero { 511 };                 // Charge current sense zero point //TODO: autocalibrate?
  float m_chgFactor { 39 };                     // Charge current conversion factor
  unsigned long m_chargingTimeout { 12600000 }; // Safety timer for charging (ms) 12600000 = 3.5hrs
  float m_chgSense { 185.0f };                  // Sensitivity of the charging current sensor (mV/A) (For ACS712 5A = 185)
  byte m_chgChange { 0 };                       // Reading reversal from - to + 1 or 0
  int m_chgNull { 2 };                          // Zero crossing charge current sensor

  // settings for ACS712 5A     (chgSenseZero ~ 511 / chgFactor = 39 / chgSense = 185.0 / chgChange = 0 oder 1 (je nach Stromrichtung) / chgNull = 2)

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
