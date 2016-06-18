/*
 * MotorShield.h
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

/**
 * Deek-Robot or Arduino, Motor Shield R3
 */

#ifndef MOTORSHIELD_H
#define MOTORSHIELD_H

#include "MotorDrv.h"

class MotorShield: public MotorDrv
{
  public:
    void setup(void);
    void setSpeed(void);
    void setSpeed(const int16_t speed);
    void setSpeed(const int16_t speed, const bool brake);
    void readCurrent(void);

  private:
    static const uint8_t PIN_DIRA {12};
    static const uint8_t PIN_DIRB {13};
    static const uint8_t PIN_PWMA {3};
    static const uint8_t PIN_PWMB {11};
    static const uint8_t PIN_BRAKEA {9};
    static const uint8_t PIN_BRAKEB {8};
    static const uint8_t PIN_SENSEA {A0};
    static const uint8_t PIN_SENSEB {A1};

    const uint8_t pinDir[2] {PIN_DIRA, PIN_DIRB};
    const uint8_t pinPwm[2] {PIN_PWMA, PIN_PWMB};
    const uint8_t pinBrake[2] {PIN_BRAKEA, PIN_BRAKEB};
    const uint8_t pinSense[2] {PIN_SENSEA, PIN_SENSEB};
};

#endif /* MOTORSHIELD_H */
