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

  bool isEnabled() const
  {
    return m_enabled;
  }

  bool isDisabled() const
  {
    return !m_enabled;
  }

  void enable()
  {
    m_enabled = true;
  }

  void disable()
  {
    m_enabled = false;
    m_motor.setRpmSet(0);
    m_motor.setPwmSet(0);
  }

  void toggleEnabled()
  {
    if (m_enabled)
    {
      disable();
    }
    else
    {
      enable();
    }
  }

  bool isEnableOverriden() const
  {
    return m_enableOverriden;
  }

  void setEnableOverriden(bool enableOverriden)
  {
    m_enableOverriden = enableOverriden;
  }

  void toggleEnableOverriden()
  {
    m_enableOverriden = !m_enableOverriden;
  }

  int8_t getSpeed() const
  {
    return m_speed;
  }

  void setSpeed(int8_t speed)
  {
    m_speed = speed;
  }

  void control(void);

private:
  bool m_enabled {};
  bool m_enableOverriden {};
  int8_t m_speed {}; // Range 0..100
};
