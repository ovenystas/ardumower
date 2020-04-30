/*
 * Battery.cpp
 *
 *  Created on: Aug 24, 2016
 *      Author: ove
 */

#include "Battery.h"
#include "AdcManager.h"

Battery::Battery(const uint8_t pinBatVoltage, const uint8_t pinChargeVoltage,
    const uint8_t pinChargeCurrent, const uint8_t pinChargeRelay,
    const uint8_t pinBatterySwitch) :
      m_pinBatVoltage(pinBatVoltage),
      m_pinChargeVoltage(pinChargeVoltage),
      m_pinChargeCurrent(pinChargeCurrent),
      m_pinChargeRelay(pinChargeRelay),
      m_pinBatterySwitch(pinBatterySwitch)
{
  pinMode(m_pinBatVoltage, INPUT);
  pinMode(m_pinChargeVoltage, INPUT);
  pinMode(m_pinChargeCurrent, INPUT);
  pinMode(m_pinChargeRelay, OUTPUT);
  pinMode(m_pinBatterySwitch, OUTPUT);

  digitalWrite(m_pinChargeRelay, LOW);
  digitalWrite(m_pinBatterySwitch, LOW);

  ADCMan.setCapture(m_pinChargeCurrent, 1, true);
  ADCMan.setCapture(m_pinBatVoltage, 1, false);
  ADCMan.setCapture(m_pinChargeVoltage, 1, false);
}

void Battery::readBatVoltage()
{
  int16_t batVoltage_ad = ADCMan.read(m_pinBatVoltage);
  int16_t batvoltage_mV =
      static_cast<int16_t>(batVoltage_ad * m_batFactor_mV_per_LSB);

  // Low-pass filter small variations but react fast to big changes
  if (abs(m_batVoltage_mV - batvoltage_mV) > 5000)
  {
    m_batVoltage_mV = batvoltage_mV;
  }
  else
  {
    m_batVoltLpFilter.addValue(batvoltage_mV);
    m_batVoltage_mV = m_batVoltLpFilter.getAverage();
  }
}

void Battery::readChargeVoltage()
{
  int16_t chargeVoltage_ad = ADCMan.read(m_pinChargeVoltage);
  int16_t chargeVoltage_mV =
      static_cast<int16_t>(chargeVoltage_ad * m_batChargeFactor_mV_per_LSB);

  // Low-pass filter small variations but react fast to big changes
  if (abs(m_chargeVoltage_mV - chargeVoltage_mV) > 5000)
  {
    m_chargeVoltage_mV = chargeVoltage_mV;
  }
  else
  {
    m_chargeVoltLpFilter.addValue(chargeVoltage_mV);
    m_chargeVoltage_mV = m_chargeVoltLpFilter.getAverage();
  }
}

void Battery::readChargeCurrent()
{
  // Read AD converter
  int16_t chargeCurrent_ad = ADCMan.read(m_pinChargeCurrent);

  // Current sensor ACS712 5A
  int16_t chargeCurrent_ad_adj = static_cast<int16_t>(chargeCurrent_ad - (1023 / 2));
  int16_t chargeCurrent_mA = static_cast<int16_t>(
      (chargeCurrent_ad_adj * (1000L * 1000 * 5)) /
      (1023L * currentSensorSensitivity_mV_per_A));

  // TODO: Does this need LP filtering?

  if (chargeCurrent_mA < 0)
  {
    m_chargeCurrent_mA = 0;
  }
  else
  {
    m_chargeCurrent_mA = chargeCurrent_mA;
  }
}

void Battery::updateCapacity()
{
  if (m_chargeCurrent_mA > 40 && m_chargeVoltage_mV > 5000)
  {
    m_batCapacity_mAs += (m_chargeCurrent_mA  + 5) / 10;
  }
}

// Expected to be called at 100 ms interval.
void Battery::read()
{
  readBatVoltage();
  readChargeVoltage();
  readChargeCurrent();

  updateCapacity();
}
