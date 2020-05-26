/*
 * cutter.h
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */
#pragma once

#include "Motor.h"
#include "MotorMosFet.h"

class Cutter
{
public:
  Cutter() {};

  MotorMosFet m_motor;

  void turnOn();
  void turnOff();
  void toggleOnOff();
  bool isOn() const
  {
    return m_isOn;
  }

  void control(void);

private:
  bool m_isOn {};

  // Settings
  bool m_use {}; // TODO: Disabled during prototyping
  uint8_t m_idleRpm_percent { 80 }; // Motor regulates to this percentage of max rpm.
  bool m_randomDir {}; // Motor direction is random each turn on.
};
