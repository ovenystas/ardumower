/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 Perimeter sender v2   (for details see   http://wiki.ardumower.de/index.php?title=Perimeter_wire  )
 Requires: Perimeter sender PCB v1.0   ( https://www.marotronics.de/Perimeter-Sender-Prototyp )
 */

#include <Arduino.h>
#include "config.h"
#include "TimerOne.h"
//#include "EEPROM.h"
//#include "RunningMedian.h"

// #define USE_DEVELOPER_TEST    1      // uncomment for new perimeter signal test (developers)

// code version
#define VER "0.1"

// --------------------------------------

volatile int step = 0;
volatile boolean enableSender = true;

//uint8_t dutyPWM = 0;
//float chargeCurrent = 0;
//float periCurrentAvg = 0;
//float periCurrentMax = 0;
//int faults = 0;
//boolean isCharging = false;
boolean stateLED = false;
//unsigned int chargeADCZero = 0;
//RunningMedian<unsigned int, 16> periCurrentMeasurements;
//RunningMedian<unsigned int, 96> chargeCurrentMeasurements;

unsigned long nextTimeControl = 0;
unsigned long nextTimeInfo = 0;
unsigned long nextTimeToggleLED = 0;

// must be multiple of 2 !
// http://grauonline.de/alexwww/ardumower/filter/filter.html
// "pseudonoise4_pw" signal (sender)

#ifdef USE_DEVELOPER_TEST
// a more motor driver friendly signal (sender)
int8_t sigcode[] =
{
  1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
  -1, 0, 0, 0, 0, 1, 0, 0, 0, 0
};
#else
int8_t sigcode[] =
{
  1,  1, -1, -1,  1, -1,  1, -1, -1,  1, -1,  1,
  1, -1, -1,  1, -1, -1,  1, -1, -1,  1,  1, -1
};
#endif


void timerCallback(void)
{
  if (enableSender)
  {
    if (sigcode[step] == 1)
    {
      digitalWrite(PIN_IN1, LOW);
#ifdef USE_DOUBLE_AMPLTIUDE
      digitalWrite(PIN_IN2, HIGH);
#endif
      digitalWrite(PIN_ENABLE, HIGH);
    }
    else if (sigcode[step] == -1)
    {
      digitalWrite(PIN_IN1, HIGH);
      digitalWrite(PIN_IN2, LOW);
      digitalWrite(PIN_ENABLE, HIGH);
    }
    else
    {
      digitalWrite(PIN_ENABLE, LOW);
    }
    step++;
    if (step == sizeof(sigcode))
    {
      step = 0;
    }
  }
  else
  {
    digitalWrite(PIN_ENABLE, LOW);
  }
}


//void readEEPROM()
//{
//  if (EEPROM.read(0) == 43)
//  {
//    // EEPROM data available
//    chargeADCZero = (EEPROM.read(1) << 8) | EEPROM.read(2);
//  }
//  else
//  {
//    Serial.println("no EEPROM data found, using default calibration (INA169)");
//  }
//  Serial.print("chargeADCZero=");
//  Serial.println(chargeADCZero);
//}


//void calibrateChargeCurrentSensor()
//{
//  Serial.println("calibrateChargeCurrentSensor "
//                 "(note: robot must be outside charging station!)");
//  RunningMedian<unsigned int, 32> measurements;
//  for (unsigned int i = 0; i < measurements.getSize(); i++)
//  {
//    unsigned int m = analogRead(PIN_CHARGE_CURRENT);
//    //Serial.println(m);
//    measurements.add(m);
//    delay(50);
//  }
//  float v;
//  measurements.getAverage(v);
//  chargeADCZero = v;
//  EEPROM.write(0, 43);
//  EEPROM.write(1, chargeADCZero >> 8);
//  EEPROM.write(2, chargeADCZero & 0xFF);
//  Serial.println("calibration done");
//  readEEPROM();
//}


//void checkKey()
//{
//  if (Serial.available() > 0)
//  {
//    char ch = (char)Serial.read();
//    Serial.print("received key=");
//    Serial.println(ch);
//    while (Serial.available())
//    {
//      Serial.read();
//    }
//    switch (ch)
//    {
//      case '1':
//        calibrateChargeCurrentSensor();
//        break;
//    }
//  }
//}


// Interrupt service run when Timer/Counter2 reaches OCR2A
//ISR(TIMER2_COMPA_vect) {
//  if (digitalRead(pinFault) == LOW) fault = true;
//if (digitalRead(pinPWM) == HIGH) fault = true;
//fault = true;


//void fault()
//{
//  Serial.println("MC_FAULT");
//  for (int i = 0; i < 10; i++)
//  {
//    digitalWrite(PIN_LED, HIGH);
//    delay(50);
//    digitalWrite(PIN_LED, LOW);
//    delay(50);
//  }
//  faults++;
//}


void setup()
{
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  //pinMode(PIN_PWM, OUTPUT);

  //pinMode(PIN_FEEDBACK, INPUT);
  //pinMode(PIN_FAULT, INPUT);
  //pinMode(PIN_POT, INPUT);
  //pinMode(PIN_CHARGE_CURRENT, INPUT);

  // configure ADC reference
  // analogReference(DEFAULT); // ADC 5.0v ref
  //analogReference(INTERNAL); // ADC 1.1v ref

  // sample rate 9615 Hz (19230,76923076923 / 2 => 9615.38)
  int T = 1000.0 * 1000.0 / 9615.38;
  Serial.begin(19200);

  Serial.println("START");
  Serial.print("Ardumower Sender ");
  Serial.println(VER);
#ifdef USE_DEVELOPER_TEST
  Serial.println("Warning: USE_DEVELOPER_TEST activated");
#endif
  Serial.println("press...");
  Serial.println("  1  for current sensor calibration");
  Serial.println();

//  readEEPROM();
  Serial.print("T=");
  Serial.println(T);
  Serial.print("f=");
  Serial.println(1000.0 * 1000.0 / T);
  Timer1.initialize(T);         // initialize timer1, and set period
  //Timer1.pwm(pinPWM, 256);
  Timer1.attachInterrupt(timerCallback);
  //digitalWrite(pinIN1, HIGH);
  //digitalWrite(pinIN2, LOW);
  //tone(pinPWM, 7800);

  // http://playground.arduino.cc/Main/TimerPWMCheatsheet
  // timer 2 pwm freq 31 khz
  //cli();
//  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
  //TIMSK2 |= (1 << OCIE2A);     // Enable Output Compare Match A Interrupt
  //OCR2A = 255;                 // Set compared value
  //sei();
}


void loop()
{
  unsigned long curMillis;

//  curMillis = millis();
//  if (curMillis >= nextTimeControl)
//  {
//    nextTimeControl = curMillis + 100;
//    dutyPWM = 255;
//    if (isCharging)
//    {
//      // switch off perimeter
//      enableSender = false;
//    }
//    else
//    {
      // switch on perimeter
      //enableSender = true;
      //analogWrite(pinPWM, 255);
      //analogWrite(PIN_PWM, dutyPWM);
//      if (USE_PERI_FAULT
//          && dutyPWM == 255
//          && digitalRead(PIN_FAULT) == LOW)
//      {
//        enableSender = false;
//        dutyPWM = 0;
//        analogWrite(PIN_PWM, dutyPWM);
//        fault();
//      }
//    }
//  }

  curMillis = millis();
  if (curMillis >= nextTimeInfo)
  {
    nextTimeInfo = curMillis + 500;
//    checkKey();

    //unsigned int v = 0;
//    float v = 0;
    // determine charging current (Ampere)
//    if (USE_CHG_CURRENT)
//    {
//      chargeCurrentMeasurements.getAverage(v);
//      chargeCurrent = ((float)((int)v - (int)chargeADCZero)) / 1023.0 * 1.1;
//      isCharging = abs(chargeCurrent) >= CHG_CURRENT_MIN;
//    }
//
//    if (USE_PERI_CURRENT)
//    {
//      // determine perimeter current (Ampere)
//      periCurrentMeasurements.getAverage(v);
//      periCurrentAvg = (float)v / 1023.0 * 1.1 / 0.525;   // 525 mV per amp
//      unsigned int h;
//      periCurrentMeasurements.getHighest(h);
//      periCurrentMax = (float)h / 1023.0 * 1.1 / 0.525;   // 525 mV per amp
//    }

    Serial.print("time=");
    Serial.print(curMillis / 1000);
//    Serial.print("\tchgCurrent=");
//    Serial.print(chargeCurrent, 3);
//    Serial.print("\tchgCurrentADC=");
//    chargeCurrentMeasurements.getAverage(v);
//    Serial.print(v);
//    Serial.print("\tisCharging=");
//    Serial.print(isCharging);
//    Serial.print("\tperiCurrent avg=");
//    Serial.print(periCurrentAvg);
//    Serial.print("\tmax=");
//    Serial.print(periCurrentMax);
//    Serial.print("\tdutyPWM=");
//    Serial.print(dutyPWM);
//    Serial.print("\tfaults=");
//    Serial.print(faults);
    Serial.println();
  }

//  if (USE_PERI_CURRENT)
//  {
//    periCurrentMeasurements.add(analogRead(PIN_FEEDBACK));
//  }
//
//  if (USE_CHG_CURRENT)
//  {
//    // determine charging current (Ampere)
//    chargeCurrentMeasurements.add(analogRead(PIN_CHARGE_CURRENT));
//  }

  // LED status
//  if (isCharging)
//  {
//    // charging
//    curMillis = millis();
//    if (curMillis >= nextTimeToggleLED)
//    {
//      nextTimeToggleLED = curMillis + 500;
//      stateLED = !stateLED;
//    }
//  }
//  else
//  {
    // not charging => indicate perimeter wire state (OFF=broken)
    //stateLED = periCurrentAvg >= PERI_CURRENT_MIN;
    stateLED = true;
//  }
  digitalWrite(PIN_LED, stateLED);
}
