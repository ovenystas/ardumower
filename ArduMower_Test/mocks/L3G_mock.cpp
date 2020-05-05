/*
 * L3G_mock.cpp
 *
 *  Created on: May 5, 2020
 *      Author: ove
 */

#include "CppUTestExt/MockSupport.h"
#include "L3G.h"

bool L3G::init(deviceTypeE device, sa0StateE sa0)
{
  mock().actualCall("L3G::init")
      .withParameter("device", device)
      .withParameter("sa0", sa0);
  return mock().boolReturnValue();
}

byte L3G::readReg(const byte reg)
{
  mock().actualCall("L3G::readReg")
      .withParameter("reg", reg);
  return static_cast<byte>(mock().unsignedIntReturnValue());
}

void L3G::enableDefault(void)
{
  mock().actualCall("L3G::enableDefault");
}

void L3G::read()
{
  mock().actualCall("L3G::read");
}
