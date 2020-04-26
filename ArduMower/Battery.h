/*
 * Battery.h
 *
 *  Created on: Aug 24, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>
#include "Setting.h"

struct BatterySettings
{
  Setting<float>   batGoHomeIfBelow;      // Drive home voltage (Volt)
  Setting<float>   batSwitchOffIfBelow;   // Switch off if below voltage (Volt)
  Setting<float>   batFactor;             // Battery conversion factor
  Setting<float>   batChgFactor;          // Battery conversion factor
  Setting<float>   batFullCurrent;        // Current flowing when battery is fully charged
  Setting<float>   startChargingIfBelow;  // Start charging if battery Voltage is below
  Setting<float>   chgSenseZero;          // Charge current sense zero point //TODO: autocalibrate?
  Setting<float>   chgFactor;             // Charge current conversion factor
  Setting<float>   batFull;               // Battery reference Voltage (fully charged)
  Setting<float>   batChargingCurrentMax; // Maximum current the charger can deliver
  Setting<uint8_t> batSwitchOffIfIdle;    // Switch off battery if idle for x minutes
  Setting<bool>    monitored;             // Monitor battery and charge voltage?
};

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

  bool isVoltageBelowSwitchOffLimit()
  {
    return m_voltage < m_batSwitchOffIfBelow;
  }

  bool isVoltageBelowGoHomeLimit()
  {
    return m_voltage < m_batGoHomeIfBelow;
  }

  bool isVoltageBelowStartChargingLimit()
  {
    return m_voltage < m_startChargingIfBelow;
  }

  bool isChargeCurrentBelowBatFullLimit()
  {
    return m_chgCurrent < m_batFullCurrent;
  }

  bool hasIdleTimePassedSwitchOffLimit(int idleTime_sec)
  {
    return idleTime_sec > m_batSwitchOffIfIdle * 60;
  }

  BatterySettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(BatterySettings* settings_p)
  {
    m_settings.batGoHomeIfBelow.value = settings_p->batGoHomeIfBelow.value;
    m_settings.batSwitchOffIfBelow.value = settings_p->batSwitchOffIfBelow.value;
    m_settings.batFactor.value = settings_p->batFactor.value;
    m_settings.batChgFactor.value = settings_p->batChgFactor.value;
    m_settings.batFullCurrent.value = settings_p->batFullCurrent.value;
    m_settings.startChargingIfBelow.value = settings_p->startChargingIfBelow.value;
    m_settings.chgSenseZero.value = settings_p->chgSenseZero.value;
    m_settings.chgFactor.value = settings_p->chgFactor.value;
    m_settings.batFull.value = settings_p->batFull.value;
    m_settings.batChargingCurrentMax.value = settings_p->batChargingCurrentMax.value;
    m_settings.batSwitchOffIfIdle.value = settings_p->batSwitchOffIfIdle.value;
    m_settings.monitored.value = settings_p->monitored.value;
 }

public:
  // --------- configuration ---------------------------------
  unsigned long m_chargingTimeout { 12600000 }; // Safety timer for charging (ms) 12600000 = 3.5hrs
  float m_chgSense { 185.0f };                  // Sensitivity of the charging current sensor (mV/A) (For ACS712 5A = 185)
  byte m_chgChange { 0 };                       // Reading reversal from - to + 1 or 0
  int m_chgNull { 2 };                          // Zero crossing charge current sensor

  // settings for ACS712 5A     (chgSenseZero ~ 511 / chgFactor = 39 / chgSense = 185.0 / chgChange = 0 oder 1 (je nach Stromrichtung) / chgNull = 2)

private:
    const float batFullValue_V = 14.7f;
    const float batChargingCurrentMaxValue_A = 1.6f;

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

    // --------- settings ---------------------------------
    BatterySettings m_settings
    {
      { "Go home if below", "V", 11.8f, batFullValue_V * 0.72f, batFullValue_V, 0.1f },
      { "Switch off if below", "V", 10.8f, batFullValue_V * 0.72f, batFullValue_V, 0.1f },
      { "Calibrate batFactor", "", 0.495f, 0.0f, 1.0f, 0.001f },
      { "Calibrate batChgFactor", "", 0.495f, 0.0f, 1.0f, 0.001f },
      { "Battery is fully charged if current is below", "A", 0.3f, 0.0f, batChargingCurrentMaxValue_A, 0.1f },
      { "Charging starts if Voltage is below", "V", 13.5f, 0.0f, batFullValue_V, 0.1f },
      { "Charge sense zero", "", 511.0f, 400.0f, 600.0f, 1.0f },
      { "Charge factor", "", 39.0f, 0.0f, 80.0f, 0.01f },
      { "Battery full", "V", batFullValue_V, 12.0f, 15.0f, 0.01f },
      { "Max charger current", "A", batChargingCurrentMaxValue_A, 0.0f, 5.0f, 0.1f },
      { "Switch off if idle for", "min", 1, 0, 240, 1.0f },
      { "Monitor", "", false, false, true },
    };

    // Shorter convenient variables for settings variables
    float& m_batGoHomeIfBelow = m_settings.batGoHomeIfBelow.value;
    float& m_batSwitchOffIfBelow = m_settings.batSwitchOffIfBelow.value;
    float& m_batFactor = m_settings.batFactor.value;
    float& m_batChgFactor = m_settings.batChgFactor.value;
    float& m_batFullCurrent = m_settings.batFullCurrent.value;
    float& m_startChargingIfBelow = m_settings.startChargingIfBelow.value;
    float& m_chgSenseZero = m_settings.chgSenseZero.value;
    float& m_chgFactor = m_settings.chgFactor.value;
    float& m_batFull = m_settings.batFull.value;
    float& m_batChargingCurrentMax = m_settings.batChargingCurrentMax.value;
    uint8_t& m_batSwitchOffIfIdle = m_settings.batSwitchOffIfIdle.value;
    bool& m_monitored = m_settings.monitored.value;
};
