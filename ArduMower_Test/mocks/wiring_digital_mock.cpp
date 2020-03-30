/*
 * wiring_digital.cpp
 *
 *  Created on: 4 okt. 2017
 *      Author: oveny
 */

#include "CppUTestExt/MockSupport.h"
#include <stdint.h>

// Mocked functions -----------------------------------------------------------

void pinMode(uint8_t pin, uint8_t mode)
{
  mock().actualCall("pinMode")
      .withParameter("pin", pin)
      .withParameter("mode", mode);
}

int16_t digitalRead(uint8_t pin)
{
  mock().actualCall("digitalRead")
        .withParameter("pin", pin);
  return static_cast<int16_t>(mock().intReturnValue());
}

void digitalWrite(uint8_t pin, uint8_t val)
{
  mock().actualCall("digitalWrite")
      .withParameter("pin", pin)
      .withParameter("val", val);
}
