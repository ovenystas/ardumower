/*
 * CrossCorrelationFilter_mock.cpp
 *
 *  Created on: May 17, 2020
 *      Author: ove
 */

#include "CppUTestExt/MockSupport.h"
#include "CrossCorrelationFilter.h"

int16_t CrossCorrelationFilter::update(
    const int8_t* H_p,
    const uint8_t subsample,
    const uint8_t M,
    const uint8_t Hsum,
    const int8_t* ip_p,
    const uint8_t nPts,
    float &quality,
    bool print)
{
  mock().actualCall("CrossCorrelationFilter::update")
      .withParameter("H_p", H_p)
      .withParameter("subsample", subsample)
      .withParameter("M", M)
      .withParameter("Hsum", Hsum)
      .withParameter("ip_p", ip_p)
      .withParameter("nPts", nPts)
      .withParameter("quality", quality)
      .withParameter("print", print);
  return static_cast<int16_t>(mock().intReturnValue());
}
