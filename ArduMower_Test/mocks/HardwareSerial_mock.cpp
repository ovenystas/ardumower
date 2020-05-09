/*
 * HardwareSerial_mock.cpp
 *
 *  Created on: May 4, 2020
 *      Author: ove
 */

#include "CppUTestExt/MockSupport.h"
#include "HardwareSerial.h"
#include <stdint.h>

#include <iostream>

// Instantiate Serial objects
HardwareSerial Serial;
HardwareSerial Serial1;
HardwareSerial Serial2;
HardwareSerial Serial3;

// Mocked functions -----------------------------------------------------------

void HardwareSerial::begin(uint32_t baud, uint8_t config)
{
  mock().actualCall("HardwareSerial::begin")
      .onObject(this)
      .withParameter("baud", baud)
      .withParameter("config", config);
}

int16_t HardwareSerial::available(void)
{
  mock().actualCall("HardwareSerial::available")
      .onObject(this);
  return static_cast<int16_t>(mock().intReturnValue());
}

int16_t HardwareSerial::peek(void)
{
  mock().actualCall("HardwareSerial::peek")
      .onObject(this);
  return static_cast<int16_t>(mock().intReturnValue());
}

int16_t HardwareSerial::read(void)
{
  mock().actualCall("HardwareSerial::read")
      .onObject(this);
  return static_cast<int16_t>(mock().intReturnValue());
}

int16_t HardwareSerial::availableForWrite(void)
{
  mock().actualCall("HardwareSerial::availableForWrite")
      .onObject(this);
  return static_cast<int16_t>(mock().intReturnValue());
}

size_t HardwareSerial::write(uint8_t c)
{
//  mock().actualCall("HardwareSerial::write")
//      .onObject(this)
//      .withParameter("c", c);
//  return static_cast<size_t>(mock().unsignedLongIntReturnValue());

  if (m_mockPrintToStdout)
  {
    std::cout << c;
  }

  return 1;
}

void HardwareSerial::flush()
{
  mock().actualCall("HardwareSerial::flush")
      .onObject(this);
}
