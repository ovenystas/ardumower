/*
 * MotorMosFet.h
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

/**
 * MosFet motor driver
 */
#pragma once

#include <Arduino.h>
#include "MotorDrv.h"

class MotorMosFet: public MotorDrv
{
  public:
    void setup(void);
    void setSpeed(void);
    void setSpeed(const int16_t speed);
    void setSpeed(const int16_t speed, const bool brake);
    void readCurrent(void);
    void control(void);

  private:
    static const uint8_t PIN_PWM {2};
    static const uint8_t PIN_SENSE {A3};

    const uint8_t pinPwm {PIN_PWM};
    const uint8_t pinSense {PIN_SENSE};
};
