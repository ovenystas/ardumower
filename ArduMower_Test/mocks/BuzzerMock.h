/*
 * BuzzerMock.h
 *
 *  Created on: May 10, 2020
 *      Author: ove
 */

#pragma once

#include "CppUTestExt/MockSupport.h"
#include "Buzzer.h"

class BuzzerMock : public Buzzer
{
public:
  BuzzerMock() : Buzzer(1, true) {};

  virtual void beep(uint16_t frequency, uint32_t duration_ms = 0) override
  {
    mock().actualCall("Buzzer::beep")
        .onObject(this)
        .withParameter("frequency", frequency)
        .withParameter("duration_ms", duration_ms);
  }

  virtual void beep(const BeepData* data_p, uint8_t len = 1) override
  {
    mock().actualCall("Buzzer::beep")
        .onObject(this)
        .withParameter("data_p", data_p)
        .withParameter("len", len);
  }

  virtual void beepStop() override
  {
    mock().actualCall("Buzzer::beepStop")
        .onObject(this);
  }
};
