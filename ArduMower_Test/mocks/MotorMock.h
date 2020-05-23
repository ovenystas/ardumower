/*
 * MotorMock.h
 *
 *  Created on: May 22, 2020
 *      Author: ove
 */

#pragma once

#include "CppUTestExt/MockSupport.h"
#include "Motor.h"

class MotorMock : public Motor
{
  virtual void setup() override
  {
    mock().actualCall("Motor::setup");
  }

  virtual void setSpeed() override
  {
    mock().actualCall("Motor::setSpeed");
  }

  virtual void setSpeed(int16_t speed) override
  {
    mock().actualCall("Motor::setSpeed")
        .withParameter("speed", speed);
  }

  virtual void setSpeed(int16_t speed, bool brake) override
  {
    mock().actualCall("Motor::setSpeed")
        .withParameter("speed", speed)
        .withParameter("brake", brake);
  }

  virtual void readCurrent() override
  {
    mock().actualCall("Motor::readCurrent");
  }
};


