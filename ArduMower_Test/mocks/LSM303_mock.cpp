/*
 * LSM303_mock.cpp
 *
 *  Created on: May 5, 2020
 *      Author: ove
 */

#include "CppUTestExt/MockSupport.h"
#include "LSM303.h"

LSM303::LSM303(void)
{
  mock().actualCall("LSM303::LSM303");
}

bool LSM303::init(DeviceTypeE device, Sa0StateE sa0)
{
  mock().actualCall("LSM303::init")
      .withParameter("device", device)
      .withParameter("sa0", sa0);
  return mock().boolReturnValue();
}

uint8_t LSM303::readReg(int16_t reg)
{
  mock().actualCall("LSM303::readReg")
      .withParameter("reg", reg);
  return static_cast<uint8_t>(mock().unsignedIntReturnValue());
}

void LSM303::writeReg(uint8_t reg, uint8_t value)
{
  mock().actualCall("LSM303::readReg")
      .withParameter("reg", reg)
      .withParameter("value", value);
}

void LSM303::readMag(void)
{
  mock().actualCall("LSM303::readMag");
}

void LSM303::readAcc(void)
{
  mock().actualCall("LSM303::readAcc");
}

void LSM303::enableDefault(void)
{
  mock().actualCall("LSM303::enableDefault");
}
