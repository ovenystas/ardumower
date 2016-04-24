/*
 * MotorMosFet.h
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

/**
 * MosFet motor driver
 */

#ifndef MOTORMOSFET_H
#define MOTORMOSFET_H

#include "MotorDrv.h"

class MotorMosFet: public MotorDrv
{
  public:
    void setup(void);
    void setSpeed(void);
    void setSpeed(const int speed);
    void setSpeed(const int speed, const bool brake);
    void readCurrent(void);

  private:
    const uint8_t PIN_PWM = 2;
    const uint8_t PIN_SENSE = A3;

    const uint8_t pinPwm { PIN_PWM };
    const uint8_t pinSense { PIN_SENSE };
};

#endif /* MOTORMOSFET_H */
