/*
 * Battery.h
 *
 *  Created on: Aug 24, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>
#include "Setting.h"
#include "Filter.h"

struct BatterySettings
{
  Setting<int16_t>  batGoHomeIfBelow_mV;        // Drive home if below this voltage
  Setting<int16_t>  batSwitchOffIfBelow_mV;     // Switch off if below this voltage
  Setting<int16_t>  batFactor_mV_per_LSB;       // Battery conversion factor // TODO: Is this accurate?
  Setting<int16_t>  batChargeFactor_mV_per_LSB; // Battery conversion factor // TODO: Is this accurate?
  Setting<int16_t>  batFullCurrent_mA;          // Current flowing when battery is fully charged
  Setting<int16_t>  startChargingIfBelow_mV;    // Start charging if battery Voltage is below
  Setting<int16_t>  chgSenseZero;               // Charge current sense zero point A/D value //TODO: autocalibrate?
  Setting<float>    chgFactor;                  // Charge current conversion factor
  Setting<int16_t>  batFull_mV;                 // Battery reference Voltage (fully charged)
  Setting<int16_t>  batChargingCurrentMax_mA;   // Maximum current the charger can deliver
  Setting<uint8_t>  batSwitchOffIfIdle_min;     // Switch off battery if idle for more than this many minutes
  Setting<bool>     monitored;                  // Monitor battery and charge voltage?
};

class Battery
{
public:
  Battery(const uint8_t pinBatVoltage, const uint8_t pinChargeVoltage,
      const uint8_t pinChargeCurrent, const uint8_t pinChargeRelay,
      const uint8_t pinBatterySwitch);

  void read();

  int16_t getBatVoltage_mV() const
  {
    return m_batVoltage_mV;
  }

  float getBatVoltage_V() const
  {
    return m_batVoltage_mV / 1000.0f;
  }

  int16_t getCapacity_mAh() const
  {
    return static_cast<int16_t>((m_batCapacity_mAs + 1800) / 3600);
  }

  float getCapacity_Ah() const
  {
    return static_cast<float>(m_batCapacity_mAs / 3600000L);
  }

  void clearCapacity()
  {
    m_batCapacity_mAs = 0;
  }

  int16_t getLastTimeCapacity_mAh() const
  {
    return static_cast<int16_t>((m_lastTimeBatCapacity_mAs + 1800) / 3600);
  }

  void updateLastTimeCapacity()
  {
    m_lastTimeBatCapacity_mAs = m_batCapacity_mAs;
  }

  bool isMonitored() const
  {
    return m_monitored;
  }

  int16_t getChargeCurrent_mA() const
  {
    return m_chargeCurrent_mA;
  }

  float getChargeCurrent_A() const
  {
    return m_chargeCurrent_mA / 1000.0f;
  }

  int16_t getChargeVoltage_mV() const
  {
    return m_chargeVoltage_mV;
  }

  float getChargeVoltage_V() const
  {
    return m_chargeVoltage_mV / 1000.0f;
  }

  // State can be HIGH or LOW
  void setChargeRelay(uint8_t state)
  {
    digitalWrite(m_pinChargeRelay, state);
  }

  // State can be HIGH or LOW
  void setBatterySwitch(uint8_t state)
  {
    digitalWrite(m_pinBatterySwitch, state);
  }

  bool isVoltageBelowSwitchOffLimit() const
  {
    return m_batVoltage_mV < m_batSwitchOffIfBelow_mV;
  }

  bool isVoltageBelowGoHomeLimit() const
  {
    return m_batVoltage_mV < m_batGoHomeIfBelow_mV;
  }

  bool isVoltageBelowStartChargingLimit() const
  {
    return m_batVoltage_mV < m_startChargingIfBelow_mV;
  }

  bool isChargeCurrentBelowBatFullLimit() const
  {
    return m_chargeCurrent_mA < m_batFullCurrent_mA;
  }

  bool hasIdleTimePassedSwitchOffLimit(int16_t idleTime_sec)
  {
    return idleTime_sec >= static_cast<int16_t>(m_batSwitchOffIfIdle_min) * secondsPerMinute;
  }

  BatterySettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(BatterySettings* settings_p)
  {
    m_settings.batGoHomeIfBelow_mV.value = settings_p->batGoHomeIfBelow_mV.value;
    m_settings.batSwitchOffIfBelow_mV.value = settings_p->batSwitchOffIfBelow_mV.value;
    m_settings.batFactor_mV_per_LSB.value = settings_p->batFactor_mV_per_LSB.value;
    m_settings.batChargeFactor_mV_per_LSB.value = settings_p->batChargeFactor_mV_per_LSB.value;
    m_settings.batFullCurrent_mA.value = settings_p->batFullCurrent_mA.value;
    m_settings.startChargingIfBelow_mV.value = settings_p->startChargingIfBelow_mV.value;
    m_settings.chgSenseZero.value = settings_p->chgSenseZero.value;
    m_settings.chgFactor.value = settings_p->chgFactor.value;
    m_settings.batFull_mV.value = settings_p->batFull_mV.value;
    m_settings.batChargingCurrentMax_mA.value = settings_p->batChargingCurrentMax_mA.value;
    m_settings.batSwitchOffIfIdle_min.value = settings_p->batSwitchOffIfIdle_min.value;
    m_settings.monitored.value = settings_p->monitored.value;
 }

private:
  void readBatVoltage();
  void readChargeVoltage();
  void readChargeCurrent();

  void updateCapacity();

public:
  // --------- configuration ---------------------------------
  uint32_t m_chargingTimeout_ms { 12600000 }; // Safety timer for charging (ms) 12600000 = 3.5hrs

  // settings for ACS712 5A     (chgSenseZero ~ 511 / chgFactor = 39 / chgSense = 185 / chgChange = 0 oder 1 (je nach Stromrichtung) / chgNull = 2)

private:
  static const int16_t batFullVoltage_mV = 14700;
  static const int16_t batChargingCurrentMaxValue_mA = 1600;
  static const uint8_t secondsPerMinute = 60;
  static const uint8_t currentSensorSensitivity_mV_per_A { 185 }; // Sensitivity of the charging current sensor (mV/A) (For ACS712 5A = 185)

  // --------- pins ---------------------------------
  uint8_t m_pinBatVoltage {};
  uint8_t m_pinChargeVoltage {};
  uint8_t m_pinChargeCurrent {};
  uint8_t m_pinChargeRelay {};
  uint8_t m_pinBatterySwitch {};

  // --------- charging ---------------------------------
  int16_t m_batVoltage_mV {};
  int32_t m_batCapacity_mAs {};
  int16_t m_chargeVoltage_mV {};
  int16_t m_chargeCurrent_mA {};
  int32_t m_lastTimeBatCapacity_mAs {};

  // Filter
  FilterEmaI16 m_batVoltLpFilter { 0.01f };
  FilterEmaI16 m_chargeVoltLpFilter { 0.01f };

  // --------- settings ---------------------------------
  BatterySettings m_settings
  {
    { "Go home if below", "mV", 11800, (batFullVoltage_mV * 72L) / 100, batFullVoltage_mV, 100.0f },
    { "Switch off if below", "mV", 10800, (batFullVoltage_mV * 72L) / 100, batFullVoltage_mV, 100.0f },
    { "Calibrate batFactor", "mV/LSB", 30, 0, 32, 1.0f },
    { "Calibrate batChgFactor", "", 30, 0, 32, 1.0f },
    { "Battery is fully charged if current is below", "mA", 300, 0, batChargingCurrentMaxValue_mA, 10.0f },
    { "Charging starts if Voltage is below", "mV", 13500, 0, batFullVoltage_mV, 100.0f },
    { "Charge sense zero", "", 511, 400, 600, 1.0f },
    { "Charge factor", "", 39.0f, 0.0f, 80.0f, 0.01f },
    { "Battery full", "mV", batFullVoltage_mV, 12000, 15000, 100.0f },
    { "Max charger current", "mA", batChargingCurrentMaxValue_mA, 0, 5000, 100.0f },
    { "Switch off if idle for", "min", 1, 0, 240, 1.0f },
    { "Monitor", "", false, false, true },
  };

  // Shorter convenient variables for settings variables
  int16_t& m_batGoHomeIfBelow_mV = m_settings.batGoHomeIfBelow_mV.value;
  int16_t& m_batSwitchOffIfBelow_mV = m_settings.batSwitchOffIfBelow_mV.value;
  int16_t& m_batFactor_mV_per_LSB = m_settings.batFactor_mV_per_LSB.value;
  int16_t& m_batChargeFactor_mV_per_LSB = m_settings.batChargeFactor_mV_per_LSB.value;
  int16_t& m_batFullCurrent_mA = m_settings.batFullCurrent_mA.value;
  int16_t& m_startChargingIfBelow_mV = m_settings.startChargingIfBelow_mV.value;
  int16_t& m_chgSenseZero = m_settings.chgSenseZero.value;
  float& m_chgFactor = m_settings.chgFactor.value;
  int16_t& m_batFull_mV = m_settings.batFull_mV.value;
  int16_t& m_batChargingCurrentMax_mA = m_settings.batChargingCurrentMax_mA.value;
  uint8_t& m_batSwitchOffIfIdle_min = m_settings.batSwitchOffIfIdle_min.value;
  bool& m_monitored = m_settings.monitored.value;
};
