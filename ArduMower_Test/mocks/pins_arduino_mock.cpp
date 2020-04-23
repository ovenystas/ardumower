/*
 * pins_arduino_mock.cpp
 *
 *  Created on: 26 okt. 2017
 *      Author: oveny
 */

#include "CppUTestExt/MockSupport.h"
#include <stdint.h>

// Mocked functions -----------------------------------------------------------

uint8_t digitalPinToBitMask(uint8_t pin)
{
  mock().actualCall("digitalPinToBitMask")
      .withParameter("pin", pin);
  return static_cast<uint8_t>(mock().unsignedIntReturnValue());
}

uint8_t digitalPinToPort(uint8_t pin)
{
  mock().actualCall("digitalPinToPort")
      .withParameter("pin", pin);
  return static_cast<uint8_t>(mock().unsignedIntReturnValue());
}

uint8_t* portOutputRegister(uint8_t port)
{
  mock().actualCall("portOutputRegister")
      .withParameter("port", port);
  return static_cast<uint8_t*>(mock().pointerReturnValue());
}

uint8_t* portInputRegister(uint8_t port)
{
  mock().actualCall("portInputRegister")
      .withParameter("port", port);
  return static_cast<uint8_t*>(mock().pointerReturnValue());
}
