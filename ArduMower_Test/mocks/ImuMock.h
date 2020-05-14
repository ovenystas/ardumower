/*
 * ImuMock.h
 *
 *  Created on: May 10, 2020
 *      Author: ove
 */

#pragma once

#include "CppUTestExt/MockSupport.h"
#include "Imu.h"

class ImuMock : public Imu
{
public:
  ImuMock() {};

  virtual bool isUsed() override
  {
    mock().actualCall("Imu::isUsed")
        .onObject(this);
    return mock().boolReturnValue();
  }

  virtual float getYaw() const override
  {
    mock().actualCall("Imu::getYaw")
        .onObject(this);
    return static_cast<float>(mock().doubleReturnValue());
  }
};
