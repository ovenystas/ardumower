/*
 * CrossCorrelationFilter_test.cpp
 *
 *  Created on: May 18, 2020
 *      Author: ove
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include <Arduino.h>

// Get access to private class members in SUT
#define private public
#include "CrossCorrelationFilter.h"

// CrossCorrelationFilter ------------------------------------------------------------

TEST_GROUP(CrossCorrelationFilter)
{
  CrossCorrelationFilter* corrfilter_p;

  void setup()
  {
  }

  void teardown()
  {
    if (corrfilter_p)
    {
      delete corrfilter_p;
    }
  }
};

TEST(CrossCorrelationFilter, init)
{
  corrfilter_p = new CrossCorrelationFilter(Serial);
}

//TEST(CrossCorrelationFilter, update)
//{
//  const int8_t sigcode[] = { 1, 0, -1, 0 };
//
//  corrfilter_p = new CrossCorrelationFilter(Serial);
//
//  corrfilter_p->update(sigcode, 2, 4, );
//}
