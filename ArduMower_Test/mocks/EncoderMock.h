/*
 * EncoderMock.h
 *
 *  Created on: May 10, 2020
 *      Author: ove
 */

#pragma once

#include "CppUTestExt/MockSupport.h"
#include "Encoder.h"

class EncoderMock : public Encoder
{
public:
  EncoderMock(uint8_t pin, bool swapDir) : Encoder(pin, swapDir) {};

  virtual void setup(uint8_t pin, bool swapDir) override
  {
    mock().actualCall("Encoder::setup")
        .onObject(this)
        .withParameter("pin", pin)
        .withParameter("swapDir", swapDir);
  }

  virtual void read() override
  {
    mock().actualCall("Encoder::read")
        .onObject(this);
  }

  virtual int16_t getCounter() override
  {
    mock().actualCall("Encoder::getCounter")
        .onObject(this);
    return static_cast<int16_t>(mock().intReturnValue());
  }

  virtual void clearCounter() override
  {
    mock().actualCall("Encoder::clearCounter")
        .onObject(this);
  }

  virtual int16_t getWheelRpmCurr() override
  {
    mock().actualCall("Encoder::getWheelRpmCurr")
        .onObject(this);
    return static_cast<int16_t>(mock().intReturnValue());
  }

  virtual void setWheelRpmCurr(int16_t wheelRpmCurr) override
  {
    mock().actualCall("Encoder::setWheelRpmCurr")
        .onObject(this)
        .withParameter("wheelRpmCurr", wheelRpmCurr);
  }
};
