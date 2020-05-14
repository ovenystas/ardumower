/*
 * Wire_mock.cpp
 *
 *  Created on: May 13, 2020
 *      Author: ove
 */

#include "Wire.h"

// Instantiate Wire objects
TwoWire Wire;

void TwoWire::beginTransmission(uint8_t address)
{
  mock().actualCall("TwoWire::beginTransmission")
      .withParameter("address", address);
}

uint8_t TwoWire::endTransmission(void)
{
  mock().actualCall("TwoWire::endTransmission");
  return static_cast<uint8_t>(mock().unsignedIntReturnValue());
}

size_t TwoWire::write(uint8_t data)
{
  mock().actualCall("TwoWire::write")
      .withParameter("data", data);
  return mock().unsignedIntReturnValue();
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
  mock().actualCall("TwoWire::requestFrom")
      .withParameter("address", address)
      .withParameter("quantity", quantity);
  return static_cast<uint8_t>(mock().unsignedIntReturnValue());
}

int16_t TwoWire::available(void)
{
  mock().actualCall("TwoWire::available");
  return static_cast<int16_t>(mock().intReturnValue());
}

int16_t TwoWire::read(void)
{
  mock().actualCall("TwoWire::read");
  return static_cast<int16_t>(mock().intReturnValue());
}

int16_t TwoWire::peek(void)
{
  mock().actualCall("TwoWire::peek");
  return static_cast<int16_t>(mock().intReturnValue());
}
