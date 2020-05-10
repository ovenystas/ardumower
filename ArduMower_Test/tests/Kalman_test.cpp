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
#include "Kalman.h"

TEST_GROUP(Kalman)
{
  Kalman* kalman_p;

  void setup()
  {
    kalman_p = new Kalman();
  }

  void teardown()
  {
    if (kalman_p)
    {
      delete kalman_p;
    }
    mock().clear();
  }
};

TEST(Kalman, update)
{
  DOUBLES_EQUAL(0.2855994f, kalman_p->update(PI/4, PI/8, 100, PI/16), 0.00001);
  DOUBLES_EQUAL(0.3237519f, kalman_p->update(PI/4, PI/8, 100, PI/16), 0.00001);
  DOUBLES_EQUAL(0.3492003f, kalman_p->update(PI/4, PI/8, 100, PI/16), 0.00001);
  DOUBLES_EQUAL(0.3647034f, kalman_p->update(PI/4, PI/8, 100, PI/16), 0.00001);

  mock().checkExpectations();
}

