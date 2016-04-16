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

class Motor
{
  public:
    PID pid;
    float acceleration;
    bool modulate;
    float maxPower;
    int setRpm;
    int curRpm;
    int maxPwm;
    int setPwm;
    float curPwm;
    float senseScale;
    int senseAdc;
    float senseCurrent;
    float sensePower;
    int overloadCounter;

    unsigned long lastRpmTime;
    unsigned long nextTimeControl;
    int lastSpeedPWM;
    unsigned long lastSetpeedTime;
    unsigned long nextTimeCheckCurrent;
    unsigned long lastTimeStucked;
    int rpmCounter;
    bool rpmLastState;

    void setup(const float acceleration, const int maxPwm,
               const float maxPower, const bool modulate,
               const int setRpm, const float senseScale,
               const uint8_t pinDir, const uint8_t pinPwm,
               const uint8_t pinSense, const uint8_t pinRpm,
               const uint8_t pinEnable, const uint8_t pinFault)
    {
      this->acceleration = acceleration;
      this->maxPwm = maxPwm;
      this->maxPower = maxPower;
      this->modulate = modulate;
      this->setRpm = setRpm;
      this->senseScale = senseScale;
      this->pinDir = pinDir;
      this->pinPwm = pinPwm;
      this->pinSense = pinSense;
      this->pinRpm = pinRpm;
      this->pinEnable = pinEnable;
      this->pinFault = pinFault;
      pinMode(pinDir, OUTPUT);
      pinMode(pinPwm, OUTPUT);
      pinMode(pinSense, INPUT);
      pinMode(pinRpm, INPUT);
      pinMode(pinEnable, OUTPUT);
      digitalWrite(pinEnable, HIGH);
      pinMode(pinFault, INPUT);
    }

  private:
    uint8_t pinDir;
    uint8_t pinPwm;
    uint8_t pinSense;
    uint8_t pinRpm;
    uint8_t pinEnable;
    uint8_t pinFault;
};

#endif /* MOTOR_H */
