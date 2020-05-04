/*
 * HardwareSerial_mock.cpp
 *
 *  Created on: May 4, 2020
 *      Author: ove
 */

#include "CppUTestExt/MockSupport.h"
#include "HardwareSerial.h"
#include <stdint.h>

// Mocked functions -----------------------------------------------------------

int16_t HardwareSerial::available(void)
{
  mock().actualCall("available")
      .onObject(this);
  return static_cast<int16_t>(mock().intReturnValue());
}

int16_t HardwareSerial::read(void)
{
  mock().actualCall("read")
      .onObject(this);
  return static_cast<int16_t>(mock().intReturnValue());
}
