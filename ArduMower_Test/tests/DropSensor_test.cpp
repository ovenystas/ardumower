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
  CHECK_FALSE(dropSensor_p->m_detected);
  UNSIGNED_LONGS_EQUAL(0, dropSensor_p->m_counter);
}

TEST(DropSensorInit, parameterizedInit)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT_PULLUP);

  dropSensor_p = new DropSensor(1);

  UNSIGNED_LONGS_EQUAL(1, dropSensor_p->m_pin);
  CHECK_FALSE(dropSensor_p->m_detected);
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
        .withParameter("pin", 1)
        .withParameter("mode", INPUT_PULLUP);

    dropSensor_p = new DropSensor(1);

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
      .withParameter("pin", 1)
      .andReturnValue(HIGH);

  dropSensor_p->check(DropSensor_Contact::NO);

  CHECK_FALSE(dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(0, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckOnceLowNo)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(LOW);

  dropSensor_p->check(DropSensor_Contact::NO);

  CHECK_TRUE(dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(1, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckOnceHighNc)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(HIGH);

  dropSensor_p->check(DropSensor_Contact::NC);

  CHECK_TRUE(dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(1, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckOnceLowNc)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(LOW);

  dropSensor_p->check(DropSensor_Contact::NC);

  CHECK_FALSE(dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(0, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckTwiceHighNo)
{
  mock().expectNCalls(2, "digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(HIGH);

  dropSensor_p->check(DropSensor_Contact::NO);

  CHECK_FALSE(dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(0, dropSensor_p->getCounter());

  dropSensor_p->check(DropSensor_Contact::NO);

  CHECK_FALSE(dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(0, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckTwiceLowNo)
{
  mock().expectNCalls(2, "digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(LOW);

  dropSensor_p->check(DropSensor_Contact::NO);
  CHECK_TRUE(dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(1, dropSensor_p->getCounter());

  dropSensor_p->check(DropSensor_Contact::NO);
  CHECK_TRUE(dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(2, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, CheckTwiceSim)
{
  dropSensor_p->simDetected();
  CHECK_TRUE(dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(1, dropSensor_p->getCounter());

  dropSensor_p->simDetected();
  CHECK_TRUE(dropSensor_p->isDetected());
  UNSIGNED_LONGS_EQUAL(2, dropSensor_p->getCounter());

  mock().checkExpectations();
}

TEST(DropSensor, clearDetected)
{
  dropSensor_p->simDetected();
  dropSensor_p->clearDetected();

  CHECK_FALSE(dropSensor_p->isDetected());

  mock().checkExpectations();
}

TEST(DropSensor, clearCounter)
{
  dropSensor_p->simDetected();
  dropSensor_p->clearCounter();

  UNSIGNED_LONGS_EQUAL(0, dropSensor_p->getCounter());

  mock().checkExpectations();
}

// DropSensorsInit -----------------------------------------------------------

const uint8_t DROP_SENSORS_NUM = 2;

TEST_GROUP(DropSensorsInit)
{
  DropSensor dropSensorArray[DROP_SENSORS_NUM];
  DropSensors* dropSensors_p;

  void setup()
  {
  }

  void teardown()
  {
    if (dropSensors_p)
    {
      delete dropSensors_p;
    }
    mock().clear();
  }
};

TEST(DropSensorsInit, parameterizedInit)
{
  dropSensors_p = new DropSensors(DropSensor_Contact::NO, dropSensorArray, 2);

  ENUMS_EQUAL_INT(DropSensor_Contact::NO, dropSensors_p->m_contactType);
  POINTERS_EQUAL(dropSensorArray, dropSensors_p->m_dropSensorArray_p);
  UNSIGNED_LONGS_EQUAL(0, dropSensors_p->m_lastRun);
  UNSIGNED_LONGS_EQUAL(2, dropSensors_p->m_len);
  CHECK_FALSE(dropSensors_p->m_use);
}

// DropSensors ------------------------------------------------------------

TEST_GROUP(DropSensors)
{
  DropSensor dropSensorArray[DROP_SENSORS_NUM];
  DropSensors* dropSensors_p;

  void setup()
  {
    mock().disable();
    dropSensors_p = new DropSensors(DropSensor_Contact::NO, dropSensorArray, 2);
    mock().enable();
  }

  void teardown()
  {
    if (dropSensors_p)
    {
      delete dropSensors_p;
    }
    mock().clear();
  }
};

TEST(DropSensors, getContactType)
{
  ENUMS_EQUAL_INT(DropSensor_Contact::NO, dropSensors_p->getContactType());
}

TEST(DropSensors, check)
{
  mock().expectOneCall("millis")
      .andReturnValue(12345);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT_PULLUP);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 2)
      .withParameter("mode", INPUT_PULLUP);
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(LOW);
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 2)
      .andReturnValue(HIGH);

  dropSensorArray[0].m_pin = 1;
  dropSensorArray[1].m_pin = 2;

  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);

  dropSensors_p->check();

  UNSIGNED_LONGS_EQUAL(12345, dropSensors_p->m_lastRun);
  CHECK_TRUE(dropSensorArray[0].isDetected());
  CHECK_FALSE(dropSensorArray[1].isDetected());
  UNSIGNED_LONGS_EQUAL(1, dropSensorArray[0].getCounter());
  UNSIGNED_LONGS_EQUAL(0, dropSensorArray[1].getCounter());

  mock().checkExpectations();
}

TEST(DropSensors, clearDetected)
{
  dropSensorArray[0].m_detected = true;
  dropSensorArray[1].m_detected = true;

  CHECK_TRUE(dropSensorArray[0].isDetected());
  CHECK_TRUE(dropSensorArray[1].isDetected());

  dropSensors_p->clearDetected();

  CHECK_FALSE(dropSensorArray[0].isDetected());
  CHECK_FALSE(dropSensorArray[1].isDetected());
}

TEST(DropSensors, isUsed)
{
  CHECK_FALSE(dropSensors_p->isUsed());

  dropSensors_p->m_use = true;

  CHECK_TRUE(dropSensors_p->isUsed());
}

TEST(DropSensors, GetSettings_setSettings)
{
  DropSensorsSettings* settings_p;

  settings_p = dropSensors_p->getSettings();

  CHECK_FALSE(settings_p->use.value);

  settings_p->use.value = true;

  dropSensors_p->setSettings(settings_p);

  settings_p = dropSensors_p->getSettings();

  CHECK_TRUE(settings_p->use.value);
}
