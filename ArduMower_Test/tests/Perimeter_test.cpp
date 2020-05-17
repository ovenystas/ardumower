/*
 * Perimeter_test.cpp
 *
 *  Created on: May 16, 2020
 *      Author: ove
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "HardwareSerial.h"
#include "Drivers.h"

// Get access to private class members in SUT
#define private public
#include "Perimeter.h"

// PerimeterInit ------------------------------------------------------------

TEST_GROUP(PerimeterInit)
{
  Perimeter* perimeter_p;

  void setup()
  {
    perimeter_p = new Perimeter();

    Console.setMockPrintFunctions(false);
    Console.setMockPrintToStdout(false);
    Console.startCapture();
  }

  void teardown()
  {
    if (perimeter_p)
    {
      delete perimeter_p;
    }
    mock().clear();

    Console.stopCapture();
    Console.setMockPrintFunctions();
    Console.setMockPrintToStdout();
  }
};

TEST(PerimeterInit, setup)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT);
  mock().expectOneCall("AdcManager::setCapture")
      .withParameter("pin", 1)
      .withParameter("samplecount", 220)
      .withParameter("autoCalibrate", true);
  mock().expectOneCall("AdcManager::getCaptureSize")
      .withParameter("pin", 1)
      .andReturnValue(220);

  perimeter_p->setup(1);

  UNSIGNED_LONGS_EQUAL(1, perimeter_p->m_idxPin);
  UNSIGNED_LONGS_EQUAL(4, perimeter_p->m_subSample);

  STRCMP_EQUAL(
      "matchSignal size=11 subSample=4 samplecount=220 capture size=220\r\n",
      Console.getMockOutString());

  mock().checkExpectations();
}

// Perimeter ------------------------------------------------------------

TEST_GROUP(Perimeter)
{
  Perimeter* perimeter_p;

  void setup()
  {
    mock().disable();
    perimeter_p = new Perimeter();
    perimeter_p->setup(1);
    mock().enable();

    Console.setMockPrintFunctions(false);
    Console.setMockPrintToStdout(false);
    Console.startCapture();
  }

  void teardown()
  {
    if (perimeter_p)
    {
      delete perimeter_p;
    }
    mock().clear();

    Console.stopCapture();
    Console.setMockPrintFunctions();
    Console.setMockPrintToStdout();
  }
};

TEST(Perimeter, isUsed)
{
  CHECK_FALSE(perimeter_p->isUsed());
  perimeter_p->m_use = true;
  CHECK_TRUE(perimeter_p->isUsed());

  mock().checkExpectations();
}

TEST(Perimeter, calcMagnitude_captureIsNotComplete)
{
  mock().expectOneCall("AdcManager::isCaptureComplete")
      .withParameter("pin", 1)
      .andReturnValue(false);

  perimeter_p->m_mag = -9876;
  LONGS_EQUAL(-9876, perimeter_p->calcMagnitude());

  mock().checkExpectations();
}

IGNORE_TEST(Perimeter, calcMagnitude_captureIsComplete)
{
  // TODO: mock same as matchedFilter()
  mock().expectOneCall("AdcManager::isCaptureComplete")
      .withParameter("pin", 1)
      .andReturnValue(true);

  perimeter_p->m_mag = -9876;
  LONGS_EQUAL(-9876, perimeter_p->calcMagnitude());

  mock().checkExpectations();
}

TEST(Perimeter, signalTimedOut_tooLowSignal)
{
  perimeter_p->m_smoothMag = 299.999f;

  CHECK_TRUE(perimeter_p->signalTimedOut());

  mock().checkExpectations();
}

TEST(Perimeter, signalTimedOut_tooLongNotInside)
{
  perimeter_p->m_smoothMag = 300.001f;
  perimeter_p->m_lastInsideTime = 1450;

  mock().expectOneCall("millis")
      .andReturnValue(9450);

  CHECK_TRUE(perimeter_p->signalTimedOut());

  mock().checkExpectations();
}

TEST(Perimeter, signalTimedOut_shortTimeNotInside)
{
  perimeter_p->m_smoothMag = 300.001f;
  perimeter_p->m_lastInsideTime = 1450;

  mock().expectOneCall("millis")
      .andReturnValue(9449);

  CHECK_FALSE(perimeter_p->signalTimedOut());

  mock().checkExpectations();
}

TEST(Perimeter, printInfo)
{
  perimeter_p->printInfo(Serial);

  STRCMP_EQUAL(
      "sig min 127  max -128 avg 0   "
      " mag 0     smag 0    qty 0  "
      " (sum min 0      max 0     )",
      Console.getMockOutString());

  mock().checkExpectations();
}

TEST(Perimeter, calcStatistics)
{
  int8_t capture[4] = { 12, 16, 10, 8 };

  mock().expectOneCall("AdcManager::getCaptureSize")
      .withParameter("pin", 1)
      .andReturnValue(4);

  perimeter_p->calcStatistics(capture);

  LONGS_EQUAL( 8, perimeter_p->m_signalMin);
  LONGS_EQUAL(16, perimeter_p->m_signalMax);
  LONGS_EQUAL(12, perimeter_p->m_signalAvg);

  mock().checkExpectations();
}

IGNORE_TEST(Perimeter, matchedFilter_diffSignal)
{
  int8_t capture[4] = { 12, 16, 10, 8 };

  mock().expectOneCall("AdcManager::getCaptureSize")
      .withParameter("pin", 1)
      .andReturnValue(4);
  mock().expectOneCall("AdcManager::getCapture")
      .withParameter("pin", 1)
      .andReturnValue(capture);
  mock().expectOneCall("CrossCorrelationFilter::update")
      .withParameter("H_p", 0)
      .withParameter("subsample", 4)
      .withParameter("M", 2)
      .withParameter("Hsum", 12)
      .withParameter("ip_p", capture)
      .withParameter("nPts", 0)
      .withParameter("quality", 0)
      .withParameter("print", false)
      .andReturnValue(0);
  mock().expectOneCall("millis")
      .andReturnValue(100);
  mock().expectOneCall("AdcManager::restart")
      .withParameter("pin", 1);

  perimeter_p->matchedFilter();

  LONGS_EQUAL(0, perimeter_p->m_mag);
  DOUBLES_EQUAL(0.0f, perimeter_p->m_smoothMag, 0.001f);
  LONGS_EQUAL(0, perimeter_p->m_signalCounter);
  CHECK_FALSE(perimeter_p->m_inside);
  UNSIGNED_LONGS_EQUAL(0, perimeter_p->m_lastInsideTime);
  UNSIGNED_LONGS_EQUAL(1, perimeter_p->m_callCounter);
  UNSIGNED_LONGS_EQUAL(1, perimeter_p->m_callCounter2);

  mock().checkExpectations();
}
