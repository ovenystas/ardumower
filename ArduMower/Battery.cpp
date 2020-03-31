/*
 * Battery.cpp
 *
 *  Created on: Aug 24, 2016
 *      Author: ove
 */

#include "Battery.h"
#include "AdcManager.h"

#define OFF 0
#define ON  1

void Battery::setup(uint8_t pinVoltage, uint8_t pinChargeVoltage,
    uint8_t pinChargeCurrent, uint8_t pinChargeRelay, uint8_t pinBatterySwitch)
{
  m_pinVoltage = pinVoltage;
  m_pinChargeVoltage = pinChargeVoltage;
  m_pinChargeCurrent = pinChargeCurrent;
  m_pinChargeRelay = pinChargeRelay;
  m_pinBatterySwitch = pinBatterySwitch;

  pinMode(m_pinVoltage, INPUT);
  pinMode(m_pinChargeVoltage, INPUT);
  pinMode(m_pinChargeCurrent, INPUT);
  pinMode(m_pinChargeRelay, OUTPUT);
  pinMode(m_pinBatterySwitch, OUTPUT);

  digitalWrite(m_pinChargeRelay, OFF);
  digitalWrite(m_pinBatterySwitch, OFF);

  ADCMan.setCapture(m_pinChargeCurrent, 1, true);
  ADCMan.setCapture(m_pinVoltage, 1, false);
  ADCMan.setCapture(m_pinChargeVoltage, 1, false);

  m_monitored = false;                  // Monitor battery and charge voltage?
  m_batGoHomeIfBelow = 11.8f;           // Drive home voltage (Volt)
  m_batSwitchOffIfBelow = 10.8f;        // Switch off if below voltage (Volt)
  m_batSwitchOffIfIdle = 1;             // Switch off battery if idle for x minutes
  m_batFactor = 0.495f;                 // Battery conversion factor
  m_batChgFactor = 0.495f;              // Battery conversion factor
  m_batFull = 14.7f;                    // Battery reference Voltage (fully charged)
  m_batChargingCurrentMax = 1.6f;       // Maximum current your charger can deliver
  m_batFullCurrent = 0.3f;              // Current flowing when battery is fully charged
  m_startChargingIfBelow = 13.5f;       // Start charging if battery Voltage is below
  m_chargingTimeout = 12600000;         // Safety timer for charging (ms) 12600000 = 3.5hrs
  m_chgSenseZero = 511;                 // Charge current sense zero point //TODO: autocalibrate?
  m_chgFactor = 39;                     // Charge current conversion factor
  m_chgSense = 185.0f;                  // Sensitivity of the charging current sensor (mV/A) (For ACS712 5A = 185)
  m_chgChange = 0;                      // Reading reversal from - to + 1 or 0
  // sensor output console      (chgSelection = 0)
  // settings for ACS712 5A     (chgSelection = 1 / chgSenseZero ~ 511 / chgFactor = 39 / chgSense = 185.0 / chgChange = 0 oder 1 (je nach Stromrichtung) / chgNull = 2)
  // settings for INA169 board  (chgSelection = 2)
  m_chgSelection = 2;                   // Sensor selection
  m_chgNull = 2;                        // Zero crossing charge current sensor
}

void Battery::read()
{
  // read battery
  if (abs(m_chgCurrent) > 0.04 && m_chgVoltage > 5)
  {
    // charging
    m_batCapacity += (m_chgCurrent / 36.0f); //TODO: What is 36.0?
  }

  // convert to float
  int16_t batADC = ADCMan.read(m_pinVoltage);
  float batvolt = (float)batADC * m_batFactor;
  int chgADC = ADCMan.read(m_pinChargeVoltage);
  float chgvolt = (float)chgADC * m_batChgFactor;
  float current = (float)ADCMan.read(m_pinChargeCurrent);

  // low-pass filter
  const float accel = 0.01f;

  if (abs(m_voltage - batvolt) > 5)
  {
    m_voltage = batvolt;
  }
  else
  {
    m_voltage = (1.0f - accel) * m_voltage + accel * batvolt;
  }

  if (abs(m_chgVoltage - chgvolt) > 5)
  {
    m_chgVoltage = chgvolt;
  }
  else
  {
    m_chgVoltage = (1.0f - accel) * m_chgVoltage + accel * chgvolt;
  }

  //Deaktiviert für Ladestromsensor berechnung
//    if (abs(chgCurrent - current) > 0.4)
//    {
//      chgCurrent = current;
//    }
//    else
//    {
//      chgCurrent = (1.0 - accel) * chgCurrent + accel * current;
//    }

  // Sensor Wert Ausgabe auf Seriellen Monitor oder HandyApp
  if (m_chgSelection == 0)
  {
    m_chgCurrent = current;
  }

  // Berechnung für Ladestromsensor ACS712 5A
  if (m_chgSelection == 1)
  {
    float chgAMP = current;              //Sensorwert übergabe vom Ladestrompin
    float vcc = 3.30f / m_chgSenseZero * 1023.0f; // Versorgungsspannung ermitteln!  chgSenseZero=511  ->Die Genauigkeit kann erhöt werden wenn der 3.3V Pin an ein Analogen Pin eingelesen wird. Dann ist vcc = (float) 3.30 / analogRead(X) * 1023.0;
    float asensor = chgAMP * vcc / 1023.0f;              // Messwert auslesen
    asensor = asensor - (vcc / (float)m_chgNull); // Nulldurchgang (vcc/2) abziehen
    m_chgSense = m_chgSense - ((5.00f - vcc) * m_chgFactor); // Korrekturfactor für Vcc!  chgFactor=39
    float amp = asensor / m_chgSense * 1000;               // Ampere berechnen
    if (m_chgChange == 1)
    {
      amp = -amp;                 //Lade Strom Messwertumkehr von - nach +
    }
    if (amp < 0.0)
    {
      m_chgCurrent = 0.0;
    }
    else
    {
      m_chgCurrent = amp; // Messwertrückgabe in chgCurrent   (Wenn Messwert kleiner als 0 dann Messwert =0 anssonsten messwertau8sgabe in Ampere)
    }
  }

  // Berechnung für Ladestromsensor INA169 board
  if (m_chgSelection == 2)
  {
    float chgAMP = current;
    float asensor = (chgAMP * 5) / 1023; // umrechnen von messwert in Spannung (5V Reference)
    float amp = asensor / (10 * 0.1f); // Ampere berechnen RL = 10k    Is = (Vout x 1k) / (RS x RL)
    if (amp < 0.0)
    {
      m_chgCurrent = 0.0;
    }
    else
    {
      m_chgCurrent = amp; // Messwertrückgabe in chgCurrent   (Wenn Messwert kleiner als 0 dann Messwert =0 ansonsten Messwertaußsgabe in Ampere)
    }
  }
}
