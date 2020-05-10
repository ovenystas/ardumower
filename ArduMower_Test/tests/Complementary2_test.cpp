/*
 * Kalman_test.cpp
 *
 *  Created on: May 10, 2020
 *      Author: ove
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Complementary2.h"

TEST_GROUP(Complementary2)
{
  Complementary2* complementary2_p;

  void setup()
  {
    complementary2_p = new Complementary2();
  }

  void teardown()
  {
    if (complementary2_p)
    {
      delete complementary2_p;
    }
    mock().clear();
  }
};

TEST(Complementary2, update)
{
  DOUBLES_EQUAL(1.806415776f, complementary2_p->update(PI/4, PI/8, 100, PI/16), 0.00001);

  mock().checkExpectations();
}
