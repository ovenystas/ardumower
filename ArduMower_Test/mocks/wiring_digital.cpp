/*
 * wiring_digital.cpp
 *
 *  Created on: 4 okt. 2017
 *      Author: oveny
 */

#include "CppUTestExt/MockSupport.h"

// Mocked functions -----------------------------------------------------------

void pinMode(uint8_t pin, uint8_t mode)
{
  mock().actualCall("pinMode")
      .withIntParameter("pin", pin)
      .withIntParameter("mode", mode);
}

int digitalRead(uint8_t pin)
{
  mock().actualCall("digitalRead")
        .withIntParameter("pin", pin);
  return mock().intReturnValue();
}
