/*
 * wiring.cpp
 *
 *  Created on: 4 okt. 2017
 *      Author: oveny
 */

#include "CppUTestExt/MockSupport.h"
#include <stdint.h>

// Mocked functions -----------------------------------------------------------

uint32_t micros()
{
  mock().actualCall("micros");
  return mock().unsignedIntReturnValue();
}

uint32_t millis()
{
  mock().actualCall("millis");
  return mock().unsignedIntReturnValue();
}

void delayMicroseconds(uint16_t us)
{
  mock().actualCall("delayMicroseconds")
      .withParameter("us", us);
}
