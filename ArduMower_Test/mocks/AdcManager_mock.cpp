/*
 * AdcManager_mock.cpp
 *
 *  Created on: 31 mar. 2020
 *      Author: oveny
 */

#include "CppUTestExt/MockSupport.h"
#include <stdint.h>
#include "AdcManager.h"

// Mocked functions -----------------------------------------------------------

void AdcManager::setCapture(const uint8_t pin,
                            const uint8_t samplecount,
                            const bool autoCalibrate)
{
  mock().actualCall("AdcManager::setCapture")
      .withParameter("pin", pin)
      .withParameter("samplecount", samplecount)
      .withParameter("autoCalibrate", autoCalibrate);
}


int16_t AdcManager::read(const uint8_t pin)
{
  mock().actualCall("AdcManager::read")
      .withParameter("pin", pin);
  return static_cast<int16_t>(mock().intReturnValue());
}

uint8_t AdcManager::getCaptureSize(const uint8_t pin)
{
  mock().actualCall("AdcManager::getCaptureSize")
      .withParameter("pin", pin);
  return static_cast<uint8_t>(mock().unsignedIntReturnValue());
}

int8_t* AdcManager::getCapture(const uint8_t pin)
{
  mock().actualCall("AdcManager::getCapture")
      .withParameter("pin", pin);
  return static_cast<int8_t*>(mock().pointerReturnValue());
}

void AdcManager::restart(const uint8_t pin)
{
  mock().actualCall("AdcManager::restart")
      .withParameter("pin", pin);
}

bool AdcManager::isCaptureComplete(const uint8_t pin)
{
  mock().actualCall("AdcManager::isCaptureComplete")
      .withParameter("pin", pin);
  return mock().boolReturnValue();
}
