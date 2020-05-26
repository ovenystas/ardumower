/*
 * wheel.h
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */
#pragma once

#include "Motor.h"
#include "MotorShield.h"
#include "Encoder.h"

class Wheel
{
public:
  MotorShield m_motor;
  Encoder m_encoder;

  void control(int8_t speed);
};
