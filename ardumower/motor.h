/*
 * motor.h
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include <Arduino.h>
#include "adcman.h"
#include "drivers.h"
#include "filter.h"

class Motor
{
  public:
    PID pid;
    float acceleration {};
    bool regulate { false };
    int rpmMax {};
    int rpmSet {};
    int rpmMeas {};
    int pwmMax {};
    int pwmSet {};
    float pwmCur {};
    float senseScale;
    int senseAdc {};
    float currentMeas {};
    float powerMax {};
    float powerMeas {};
    int overloadCounter {};
    bool swapDir { false };

    unsigned long lastRpmTime {};
    unsigned long nextTimeControl {};
    int lastSpeedPWM {};
    unsigned long lastSetpeedTime {};
    unsigned long nextTimeCheckPower {};
    unsigned long nextTimeReadSensor {};
    unsigned long lastTimeStucked {};
    unsigned long lastSetSpeedTime {};
    int rpmCounter {};
    bool rpmLastState { false };
    int powerIgnoreTime {};
    int zeroSettleTime {};
    unsigned long zeroTimeout {};

    void setup(const float acceleration, const int pwmMax, const float powerMax,
               const bool regulate, const int rpmMax, const int rpmSet,
               const float senseScale, const uint8_t pinDir,
               const uint8_t pinPwm, const uint8_t pinSense,
               const uint8_t pinRpm, const uint8_t pinBrake)
    {
      this->acceleration = acceleration;
      this->pwmMax = pwmMax;
      this->powerMax = powerMax;
      this->regulate = regulate;
      this->rpmMax = rpmMax;
      this->rpmSet = rpmSet;
      this->senseScale = senseScale;
      this->pinDir = pinDir;
      this->pinPwm = pinPwm;
      this->pinSense = pinSense;
      this->pinRpm = pinRpm;
      this->pinBrake = pinBrake;
      pinMode(pinDir, OUTPUT);
      digitalWrite(pinDir, LOW);
      pinMode(pinPwm, OUTPUT);
      digitalWrite(pinDir, LOW);
      pinMode(pinSense, INPUT);
      pinMode(pinRpm, INPUT);
      pinMode(pinBrake, OUTPUT);
      digitalWrite(pinDir, LOW);
    }

    bool readRpmPin()
    {
      return digitalRead(pinRpm);
    }

    int readSensePin()
    {
      senseAdc = ADCMan.read(pinSense);
      return senseAdc;
    }

    float calcCurrent(double alpha)
    {
      // Exponential Moving Average (EMA) filter (ardumower version)
      currentMeas = currentMeas * (1.0 - alpha) + (double)senseAdc * senseScale * alpha;
      // Exponential Moving Average (EMA) filter (Ove's version)
      //senseCurrent = dsp_ema_i16((double)senseAdc * senseScale, senseCurrent, DSP_EMA_I16_ALPHA(0.05));
      return currentMeas;
    }

    float calcPower(float batV)
    {
      powerMeas = currentMeas * batV / 1000;
      return powerMeas;
    }

    // call this from hall sensor interrupt
    void setRpmState()
    {
      boolean newRpmState = readRpmPin();
      if (newRpmState != rpmLastState)
      {
        rpmLastState = newRpmState;
        if (rpmLastState)
        {
          rpmCounter++;
        }
      }
    }

    unsigned long getSamplingTime()
    {
      unsigned long curMillis = millis();
      unsigned long samplingTime = curMillis - lastSetSpeedTime;
      lastSetSpeedTime = curMillis;

      if (samplingTime > 1000)
      {
        samplingTime = 1;
      }
      return samplingTime;
    }

    void setSpeed()
    {
      int speed;
      if (swapDir)
      {
        speed = -pwmCur;
      }
      else
      {
        speed = pwmCur;
      }
      setArdumoto(pinDir, pinPwm, speed);
    }

  private:
    uint8_t pinDir;
    uint8_t pinPwm;
    uint8_t pinSense;
    uint8_t pinRpm;
    uint8_t pinBrake;
};

#endif /* MOTOR_H */
