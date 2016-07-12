/*
 * cutter.h
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#ifndef CUTTER_H
#define CUTTER_H

#include "Motor.h"
#include "MotorMosFet.h"

class Cutter
{
  public:
    MotorMosFet motor;

    const bool isEnabled() const
    {
      return enabled;
    }

    const bool isDisabled() const
    {
      return !enabled;
    }

    void enable()
    {
      enabled = true;
    }

    void disable()
    {
      enabled = false;
      motor.setRpmSet(0);
      motor.setPwmSet(0);
    }

    void toggleEnabled()
    {
      enabled = !enabled;
    }

    const bool isEnableOverriden() const
    {
      return enableOverriden;
    }

    void setEnableOverriden(bool enableOverriden)
    {
      this->enableOverriden = enableOverriden;
    }

    void toggleEnableOverriden()
    {
      enableOverriden = !enableOverriden;
    }

    const int8_t getSpeed() const
    {
      return speed;
    }

    void setSpeed(int8_t speed)
    {
      this->speed = speed;
    }

    void control(void);

  private:
    bool enabled {false};
    bool enableOverriden {false};
    int8_t speed; // Range 0..100
};

#endif /* CUTTER_H */
