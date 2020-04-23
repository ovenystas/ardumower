#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "RainSensor.h"

// RainSensorInit ------------------------------------------------------------

TEST_GROUP(RainSensorInit)
{
  RainSensor* rainSensor_p;

  void setup()
  {
  }

  void teardown()
  {
    if (rainSensor_p)
    {
      delete rainSensor_p;
    }
    mock().clear();
  }

  void mockSetup_defaultInit_setup_and_parameterizedInit()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", INPUT);
  }

  void checks_defaultInit_setup_and_parameterizedInit()
  {
    CHECK_FALSE(rainSensor_p->m_use);
    UNSIGNED_LONGS_EQUAL(1, rainSensor_p->m_pin);
    CHECK_FALSE(rainSensor_p->m_raining);
    UNSIGNED_LONGS_EQUAL(0, rainSensor_p->m_counter);

    mock().checkExpectations();
  }
};

TEST(RainSensorInit, defaultInit)
{
  rainSensor_p = new RainSensor();

  CHECK_FALSE(rainSensor_p->m_use);
  UNSIGNED_LONGS_EQUAL(0, rainSensor_p->m_pin);
  CHECK_FALSE(rainSensor_p->m_raining);
  UNSIGNED_LONGS_EQUAL(0, rainSensor_p->m_counter);
}

TEST(RainSensorInit, defaultInit_setup)
{
  mockSetup_defaultInit_setup_and_parameterizedInit();

  rainSensor_p = new RainSensor();
  rainSensor_p->setup(1);

  checks_defaultInit_setup_and_parameterizedInit();
}

TEST(RainSensorInit, parameterizedInit)
{
  mockSetup_defaultInit_setup_and_parameterizedInit();

  rainSensor_p = new RainSensor(1);

  checks_defaultInit_setup_and_parameterizedInit();
}

// RainSensor -----------------------------------------------------------------

TEST_GROUP(RainSensor)
{
  RainSensor* rainSensor_p;

  void setup()
  {
    mock().disable();
    rainSensor_p = new RainSensor(1);
    mock().enable();
  }

  void teardown()
  {
    if (rainSensor_p)
    {
      delete rainSensor_p;
    }
    mock().clear();
  }
};

TEST(RainSensor, CheckOnceHigh)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  rainSensor_p->check();
  CHECK_EQUAL(false, rainSensor_p->isRaining());
  CHECK_EQUAL(0, rainSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(RainSensor, CheckOnceLow)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  rainSensor_p->check();
  CHECK_EQUAL(true, rainSensor_p->isRaining());
  CHECK_EQUAL(1, rainSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(RainSensor, CheckTwiceHigh)
{
  mock().expectNCalls(2, "digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  rainSensor_p->check();
  CHECK_EQUAL(false, rainSensor_p->isRaining());
  CHECK_EQUAL(0, rainSensor_p->getCounter());

  rainSensor_p->check();
  CHECK_EQUAL(false, rainSensor_p->isRaining());
  CHECK_EQUAL(0, rainSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(RainSensor, CheckTwiceLow)
{
  mock().expectNCalls(2, "digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  rainSensor_p->check();
  CHECK_EQUAL(true, rainSensor_p->isRaining());
  CHECK_EQUAL(1, rainSensor_p->getCounter());

  rainSensor_p->check();
  CHECK_EQUAL(true, rainSensor_p->isRaining());
  CHECK_EQUAL(2, rainSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(RainSensor, CheckLowHigh)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  rainSensor_p->check();
  CHECK_EQUAL(true, rainSensor_p->isRaining());
  CHECK_EQUAL(1, rainSensor_p->getCounter());

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  rainSensor_p->check();
  CHECK_EQUAL(false, rainSensor_p->isRaining());
  CHECK_EQUAL(1, rainSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(RainSensor, CheckHighLow)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  rainSensor_p->check();
  CHECK_EQUAL(false, rainSensor_p->isRaining());
  CHECK_EQUAL(0, rainSensor_p->getCounter());

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  rainSensor_p->check();
  CHECK_EQUAL(true, rainSensor_p->isRaining());
  CHECK_EQUAL(1, rainSensor_p->getCounter());

  mock().checkExpectations();
}

