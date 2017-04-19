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

void Battery::setup(const uint8_t pinVoltage,
                    const uint8_t pinChargeVoltage,
                    const uint8_t pinChargeCurrent,
                    const uint8_t pinChargeRelay,
                    const uint8_t pinBatterySwitch)
{
  this->pinVoltage = pinVoltage;
  this->pinChargeVoltage = pinChargeVoltage;
  this->pinChargeCurrent = pinChargeCurrent;
  this->pinChargeRelay = pinChargeRelay;
  this->pinBatterySwitch = pinBatterySwitch;

  pinMode(pinVoltage, INPUT);
  pinMode(pinChargeVoltage, INPUT);
  pinMode(pinChargeCurrent, INPUT);
  pinMode(pinChargeRelay, OUTPUT);
  pinMode(pinBatterySwitch, OUTPUT);

  digitalWrite(pinChargeRelay, OFF);
  digitalWrite(pinBatterySwitch, OFF);

  ADCMan.setCapture(pinChargeCurrent, 1, true);
  ADCMan.setCapture(pinVoltage, 1, false);
  ADCMan.setCapture(pinChargeVoltage, 1, false);
}

bool Battery::isTimeToRead(void)
{
  unsigned long curMillis = millis();
  if (curMillis >= nextTimeCheck)
  {
    nextTimeCheck = curMillis + TIME_BETWEEN_READS;
    return true;
  }
  return false;
}

bool Battery::isTimeToCheck(void)
{
  unsigned long curMillis = millis();
  if (curMillis >= nextTimeCheck)
  {
    nextTimeCheck = curMillis + TIME_BETWEEN_CHECKS;
    return true;
  }
  return false;
}

void Battery::read(void)
{
  // read battery
  if (abs(chgCurrent) > 0.04 && chgVoltage > 5)
  {
    // charging
    batCapacity += (chgCurrent / 36.0); //TODO: What is 36.0?
  }

  // convert to double
  int16_t batADC = ADCMan.read(pinVoltage);
  double batvolt = (double)batADC * batFactor;
  int chgADC = ADCMan.read(pinChargeVoltage);
  double chgvolt = (double)chgADC * batChgFactor;
  double current = (double)ADCMan.read(pinChargeCurrent);

  // low-pass filter
  const double accel = 0.01;

  if (abs(voltage - batvolt) > 5)
  {
    voltage = batvolt;
  }
  else
  {
    voltage = (1.0 - accel) * voltage + accel * batvolt;
  }

  if (abs(chgVoltage - chgvolt) > 5)
  {
    chgVoltage = chgvolt;
  }
  else
  {
    chgVoltage = (1.0 - accel) * chgVoltage + accel * chgvolt;
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
  if (chgSelection == 0)
  {
    chgCurrent = current;
  }

  // Berechnung für Ladestromsensor ACS712 5A
  if (chgSelection == 1)
  {
    float chgAMP = current;              //Sensorwert übergabe vom Ladestrompin
    float vcc = 3.30 / chgSenseZero * 1023.0; // Versorgungsspannung ermitteln!  chgSenseZero=511  ->Die Genauigkeit kann erhöt werden wenn der 3.3V Pin an ein Analogen Pin eingelesen wird. Dann ist vcc = (float) 3.30 / analogRead(X) * 1023.0;
    float asensor = chgAMP * vcc / 1023.0;              // Messwert auslesen
    asensor = asensor - (vcc / chgNull); // Nulldurchgang (vcc/2) abziehen
    chgSense = chgSense - ((5.00 - vcc) * chgFactor); // Korrekturfactor für Vcc!  chgFactor=39
    float amp = asensor / chgSense * 1000;               // Ampere berechnen
    if (chgChange == 1)
    {
      amp = -amp;                 //Lade Strom Messwertumkehr von - nach +
    }
    if (amp < 0.0)
    {
      chgCurrent = 0.0;
    }
    else
    {
      chgCurrent = amp; // Messwertrückgabe in chgCurrent   (Wenn Messwert kleiner als 0 dann Messwert =0 anssonsten messwertau8sgabe in Ampere)
    }
  }

  // Berechnung für Ladestromsensor INA169 board
  if (chgSelection == 2)
  {
    float chgAMP = current;
    float asensor = (chgAMP * 5) / 1023; // umrechnen von messwert in Spannung (5V Reference)
    float amp = asensor / (10 * 0.1); // Ampere berechnen RL = 10k    Is = (Vout x 1k) / (RS x RL)
    if (amp < 0.0)
    {
      chgCurrent = 0.0;
    }
    else
    {
      chgCurrent = amp; // Messwertrückgabe in chgCurrent   (Wenn Messwert kleiner als 0 dann Messwert =0 ansonsten Messwertaußsgabe in Ampere)
    }
  }
}

