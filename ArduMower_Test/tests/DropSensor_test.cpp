#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "DropSensor.h"

// DropSensorInit ------------------------------------------------------------

TEST_GROUP(DropSensorInit)
{
  DropSensor* dropSensor_p;

  void setup()
  {
  }

  void teardown()
  {
    if (dropSensor_p)
    {
      delete dropSensor_p;
    }
    mock().clear();
  }
};

TEST(DropSensorInit, defaultInit)
{
  dropSensor_p = new DropSensor();

  UNSIGNED_LONGS_EQUAL(0, dropSensor_p->m_pin);
  CHECK(!dropSensor_p->m_detected);
  UNSIGNED_LONGS_EQUAL(0, dropSensor_p->m_counter);
}

TEST(DropSensorInit, defaultInit_setup)
{
  mock().expectOneCall("pinMode")
      .withIntParameter("pin", 1)
      .withIntParameter("mode", INPUT_PULLUP);

  dropSensor_p = new DropSensor();
  dropSensor_p->setup(1);

  UNSIGNED_LONGS_EQUAL(1, dropSensor_p->m_pin);
  CHECK(!dropSensor_p->m_detected);
  UNSIGNED_LONGS_EQUAL(0, dropSensor_p->m_counter);

  mock().checkExpectations();
}

// DropSensor ------------------------------------------------------------

TEST_GROUP(DropSensor)
{
  DropSensor* dropSensor_p;

  void setup()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", INPUT_PULLUP);

    dropSensor_p = new DropSensor();
    dropSensor_p->setup(1);

    mock().checkExpectations();
  }

  void teardown()
  {
    if (dropSensor_p)
    {
      delete dropSensor_p;
    }
    mock().clear();
  }
};

TEST(DropSensor, CheckOnceHighNo)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  dropSensor_p->check(DropSensor_Contact::NO);

  CHECK(!dropSensor_p->isDetected());
  CHECK_EQUAL(0, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckOnceLowNo)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  dropSensor_p->check(DropSensor_Contact::NO);

  CHECK_EQUAL(true, dropSensor_p->isDetected());
  CHECK_EQUAL(1, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckOnceHighNc)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  dropSensor_p->check(DropSensor_Contact::NC);
  CHECK(dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(1, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckOnceLowNc)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  dropSensor_p->check(DropSensor_Contact::NC);

  CHECK(!dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(0, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckTwiceHighNo)
{
  mock().expectNCalls(2, "digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  dropSensor_p->check(DropSensor_Contact::NO);

  CHECK(!dropSensor_p->isDetected());
  CHECK_EQUAL(0, dropSensor_p->getCounter());

  dropSensor_p->check(DropSensor_Contact::NO);

  CHECK(!dropSensor_p->isDetected());
  CHECK_EQUAL(0, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckTwiceLowNo)
{
  mock().expectNCalls(2, "digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  dropSensor_p->check(DropSensor_Contact::NO);
  CHECK_EQUAL(true, dropSensor_p->isDetected());
  CHECK_EQUAL(1, dropSensor_p->getCounter());

  dropSensor_p->check(DropSensor_Contact::NO);
  CHECK_EQUAL(true, dropSensor_p->isDetected());
  CHECK_EQUAL(2, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckTwiceSim)
{
  dropSensor_p->simDetected();
  CHECK_EQUAL(true, dropSensor_p->isDetected());
  CHECK_EQUAL(1, dropSensor_p->getCounter());

  dropSensor_p->simDetected();
  CHECK_EQUAL(true, dropSensor_p->isDetected());
  CHECK_EQUAL(2, dropSensor_p->getCounter());

  mock().checkExpectations();
}
