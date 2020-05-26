/*
 * wheel.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#include "Wheel.h"

void Wheel::control(int8_t speed)
{
  m_motor.setRpmMeas(m_encoder.getWheelRpmCurr());

  m_motor.setRpmSet((m_motor.m_rpmMax * speed) / 100);

  m_motor.control();
}
