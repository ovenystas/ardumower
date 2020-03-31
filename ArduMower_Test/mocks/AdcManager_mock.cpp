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
