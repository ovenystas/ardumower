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
#include "Motor.h"

class MotorMosFet: public Motor
{
public:
  void setup() override;
  void setSpeed() override;
  void setSpeed(int16_t speed) override;
  void setSpeed(int16_t speed, bool brake) override;
  void readCurrent() override;
  void control();

private:
  const uint8_t m_pinPwm { 2 };
  const uint8_t m_pinSense { A3 };
};
