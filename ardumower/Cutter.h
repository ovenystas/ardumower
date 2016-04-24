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

    bool isEnabled() const
    {
      return enabled;
    }

    bool isDisabled() const
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
    }

    void toggleEnabled()
    {
      enabled = !enabled;
    }

    bool isEnableOverriden() const
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

  private:
    bool enabled { false };
    bool enableOverriden { false };
};

#endif /* CUTTER_H */
