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

void battery_setup(const uint8_t pinVoltage,
                   const uint8_t pinChargeVoltage,
                   const uint8_t pinChargeCurrent,
                   const uint8_t pinChargeRelay,
                   const uint8_t pinBatterySwitch,
                   Battery* battery_p)
{
  battery_p->pinVoltage = pinVoltage;
  battery_p->pinChargeVoltage = pinChargeVoltage;
  battery_p->pinChargeCurrent = pinChargeCurrent;
  battery_p->pinChargeRelay = pinChargeRelay;
  battery_p->pinBatterySwitch = pinBatterySwitch;

  pinMode(battery_p->pinVoltage, INPUT);
  pinMode(battery_p->pinChargeVoltage, INPUT);
  pinMode(battery_p->pinChargeCurrent, INPUT);
  pinMode(battery_p->pinChargeRelay, OUTPUT);
  pinMode(battery_p->pinBatterySwitch, OUTPUT);

  digitalWrite(battery_p->pinChargeRelay, OFF);
  digitalWrite(battery_p->pinBatterySwitch, OFF);

  ADCMan.setCapture(battery_p->pinChargeCurrent, 1, true);
  ADCMan.setCapture(battery_p->pinVoltage, 1, false);
  ADCMan.setCapture(battery_p->pinChargeVoltage, 1, false);

  battery_p->monitored = false;                // Monitor battery and charge voltage?
  battery_p->batGoHomeIfBelow = 11.8;            // Drive home voltage (Volt)
  battery_p->batSwitchOffIfBelow = 10.8;         // Switch off if below voltage (Volt)
  battery_p->batSwitchOffIfIdle = 1;               // Switch off battery if idle for x minutes
  battery_p->batFactor = 0.495;                  // Battery conversion factor
  battery_p->batChgFactor = 0.495;               // Battery conversion factor
  battery_p->batFull = 14.7;                     // Battery reference Voltage (fully charged)
  battery_p->batChargingCurrentMax = 1.6;        // Maximum current your charger can deliver
  battery_p->batFullCurrent = 0.3;               // Current flowing when battery is fully charged
  battery_p->startChargingIfBelow = 13.5;        // Start charging if battery Voltage is below
  battery_p->chargingTimeout = 12600000; // Cafety timer for charging (ms) 12600000 = 3.5hrs
  battery_p->chgSenseZero = 511;                 // Charge current sense zero point //TODO: autocalibrate?
  battery_p->chgFactor = 39;                     // Charge current conversion factor
  battery_p->chgSense = 185.0;                   // Sensitivity of the charging current sensor (mV/A) (For ACS712 5A = 185)
  battery_p->chgChange = 0;                       // Reading reversal from - to + 1 or 0
  // sensor output console      (chgSelection = 0)
  // settings for ACS712 5A     (chgSelection = 1 / chgSenseZero ~ 511 / chgFactor = 39 / chgSense = 185.0 / chgChange = 0 oder 1 (je nach Stromrichtung) / chgNull = 2)
  // settings for INA169 board  (chgSelection = 2)
  battery_p->chgSelection = 2;                    // Senor selection
  battery_p->chgNull = 2;                          // Zero crossing charge current sensor
}

void battery_read(Battery* battery_p)
{
  // read battery
  if (abs(battery_p->chgCurrent) > 0.04 && battery_p->chgVoltage > 5)
  {
    // charging
    battery_p->batCapacity += (battery_p->chgCurrent / 36.0); //TODO: What is 36.0?
  }

  // convert to double
  int16_t batADC = ADCMan.read(battery_p->pinVoltage);
  double batvolt = (double)batADC * battery_p->batFactor;
  int chgADC = ADCMan.read(battery_p->pinChargeVoltage);
  double chgvolt = (double)chgADC * battery_p->batChgFactor;
  double current = (double)ADCMan.read(battery_p->pinChargeCurrent);

  // low-pass filter
  const double accel = 0.01;

  if (abs(battery_p->voltage - batvolt) > 5)
  {
    battery_p->voltage = batvolt;
  }
  else
  {
    battery_p->voltage = (1.0 - accel) * battery_p->voltage + accel * batvolt;
  }

  if (abs(battery_p->chgVoltage - chgvolt) > 5)
  {
    battery_p->chgVoltage = chgvolt;
  }
  else
  {
    battery_p->chgVoltage = (1.0 - accel) * battery_p->chgVoltage + accel * chgvolt;
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
  if (battery_p->chgSelection == 0)
  {
    battery_p->chgCurrent = current;
  }

  // Berechnung für Ladestromsensor ACS712 5A
  if (battery_p->chgSelection == 1)
  {
    float chgAMP = current;              //Sensorwert übergabe vom Ladestrompin
    float vcc = 3.30 / battery_p->chgSenseZero * 1023.0; // Versorgungsspannung ermitteln!  chgSenseZero=511  ->Die Genauigkeit kann erhöt werden wenn der 3.3V Pin an ein Analogen Pin eingelesen wird. Dann ist vcc = (float) 3.30 / analogRead(X) * 1023.0;
    float asensor = chgAMP * vcc / 1023.0;              // Messwert auslesen
    asensor = asensor - (vcc / battery_p->chgNull); // Nulldurchgang (vcc/2) abziehen
    battery_p->chgSense = battery_p->chgSense - ((5.00 - vcc) * battery_p->chgFactor); // Korrekturfactor für Vcc!  chgFactor=39
    float amp = asensor / battery_p->chgSense * 1000;               // Ampere berechnen
    if (battery_p->chgChange == 1)
    {
      amp = -amp;                 //Lade Strom Messwertumkehr von - nach +
    }
    if (amp < 0.0)
    {
      battery_p->chgCurrent = 0.0;
    }
    else
    {
      battery_p->chgCurrent = amp; // Messwertrückgabe in chgCurrent   (Wenn Messwert kleiner als 0 dann Messwert =0 anssonsten messwertau8sgabe in Ampere)
    }
  }

  // Berechnung für Ladestromsensor INA169 board
  if (battery_p->chgSelection == 2)
  {
    float chgAMP = current;
    float asensor = (chgAMP * 5) / 1023; // umrechnen von messwert in Spannung (5V Reference)
    float amp = asensor / (10 * 0.1); // Ampere berechnen RL = 10k    Is = (Vout x 1k) / (RS x RL)
    if (amp < 0.0)
    {
      battery_p->chgCurrent = 0.0;
    }
    else
    {
      battery_p->chgCurrent = amp; // Messwertrückgabe in chgCurrent   (Wenn Messwert kleiner als 0 dann Messwert =0 ansonsten Messwertaußsgabe in Ampere)
    }
  }
}
