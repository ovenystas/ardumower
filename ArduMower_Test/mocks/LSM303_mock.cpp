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

bool LSM303::init(deviceTypeE device, sa0StateE sa0)
{
  mock().actualCall("LSM303::init")
      .withParameter("device", device)
      .withParameter("sa0", sa0);
  return mock().boolReturnValue();
}

byte LSM303::readReg(int reg)
{
  mock().actualCall("LSM303::readReg")
      .withParameter("reg", reg);
  return static_cast<byte>(mock().unsignedIntReturnValue());
}

void LSM303::writeReg(byte reg, byte value)
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
