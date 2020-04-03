/*
 * wiring.cpp
 *
 *  Created on: 4 okt. 2017
 *      Author: oveny
 */

#include "CppUTestExt/MockSupport.h"
#include <stdint.h>

// Mocked functions -----------------------------------------------------------

// frequency (in hertz) and duration (in milliseconds).
void tone(uint8_t _pin, uint16_t frequency, uint32_t duration)
{
  mock().actualCall("tone")
      .withParameter("_pin", _pin)
      .withParameter("frequency", frequency)
      .withParameter("duration", duration);
}


void noTone(uint8_t _pin)
{
  mock().actualCall("tone")
      .withParameter("_pin", _pin);
}
