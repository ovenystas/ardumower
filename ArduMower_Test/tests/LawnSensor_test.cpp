#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "LawnSensor.h"

// LawnSensorInit --------------------------------------------------------

TEST_GROUP(LawnSensorInit)
{
  LawnSensor* lawnSensor_p;

  void setup()
  {
  }

  void teardown()
  {
    if (lawnSensor_p)
    {
      delete lawnSensor_p;
    }
    mock().clear();
  }
};

TEST(LawnSensorInit, defaultInit)
{
  lawnSensor_p = new LawnSensor();

  UNSIGNED_LONGS_EQUAL(0, lawnSensor_p->m_sendPin);
  UNSIGNED_LONGS_EQUAL(0, lawnSensor_p->m_receivePin);
  UNSIGNED_LONGS_EQUAL(0, lawnSensor_p->m_value);
  UNSIGNED_LONGS_EQUAL(0, lawnSensor_p->m_valueOld);
}

TEST(LawnSensorInit, defaultInit_setup)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", OUTPUT);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 2)
      .withParameter("mode", INPUT);

  lawnSensor_p = new LawnSensor();
  lawnSensor_p->setup(1, 2);

  UNSIGNED_LONGS_EQUAL(1, lawnSensor_p->m_sendPin);
  UNSIGNED_LONGS_EQUAL(2, lawnSensor_p->m_receivePin);
  UNSIGNED_LONGS_EQUAL(0, lawnSensor_p->m_value);
  UNSIGNED_LONGS_EQUAL(0, lawnSensor_p->m_valueOld);

  mock().checkExpectations();
}

TEST(LawnSensorInit, parameterizedInit)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", OUTPUT);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 2)
      .withParameter("mode", INPUT);

  lawnSensor_p = new LawnSensor(1, 2);

  UNSIGNED_LONGS_EQUAL(1, lawnSensor_p->m_sendPin);
  UNSIGNED_LONGS_EQUAL(2, lawnSensor_p->m_receivePin);
  UNSIGNED_LONGS_EQUAL(0, lawnSensor_p->m_value);
  UNSIGNED_LONGS_EQUAL(0, lawnSensor_p->m_valueOld);

  mock().checkExpectations();
}

// LawnSensor ------------------------------------------------------------

TEST_GROUP(LawnSensor)
{
  LawnSensor* lawnSensor_p;

  void setup()
  {
    mock().disable();
    lawnSensor_p = new LawnSensor(1, 2);
    mock().enable();
  }

  void teardown()
  {
    if (lawnSensor_p)
    {
      delete lawnSensor_p;
    }
    mock().clear();
  }
};

TEST(LawnSensor, measureLawnCapacity)
{
  mock().expectOneCall("digitalWrite")
      .withIntParameter("pin", 1)
      .withIntParameter("val", HIGH);
  mock().expectNCalls(5, "digitalRead")
      .withIntParameter("pin", 2)
      .andReturnValue(LOW);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 2)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalWrite")
      .withIntParameter("pin", 1)
      .withIntParameter("val", LOW);

  UNSIGNED_LONGS_EQUAL(5, lawnSensor_p->measureLawnCapacity());

  mock().checkExpectations();
}

TEST(LawnSensor, getValue)
{
  DOUBLES_EQUAL(0.0, lawnSensor_p->getValue(), 0.01);
  lawnSensor_p->m_value = 1.2f;
  DOUBLES_EQUAL(1.2, lawnSensor_p->getValue(), 0.01);
}

TEST(LawnSensor, readVal0)
{
  const float accel = 0.03f;

  mock().expectOneCall("digitalWrite")
      .withIntParameter("pin", 1)
      .withIntParameter("val", HIGH);
  mock().expectNCalls(5, "digitalRead")
      .withIntParameter("pin", 2)
      .andReturnValue(LOW);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 2)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalWrite")
      .withIntParameter("pin", 1)
      .withIntParameter("val", LOW);

  lawnSensor_p->read();
  DOUBLES_EQUAL(accel * 5.0f, lawnSensor_p->getValue(), 0.01);

  mock().checkExpectations();
}

TEST(LawnSensor, readVal2)
{
  const float accel = 0.03f;

  mock().expectOneCall("digitalWrite")
      .withIntParameter("pin", 1)
      .withIntParameter("val", HIGH);
  mock().expectNCalls(5, "digitalRead")
      .withIntParameter("pin", 2)
      .andReturnValue(LOW);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 2)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalWrite")
      .withIntParameter("pin", 1)
      .withIntParameter("val", LOW);

  lawnSensor_p->m_value = 2.0f;
  lawnSensor_p->read();
  DOUBLES_EQUAL((1.0f - accel) * 2.0f + accel * 5.0f, lawnSensor_p->getValue(),
      0.01);

  mock().checkExpectations();
}

// LawnSensorsInit ------------------------------------------------------------

const uint8_t LAWN_SENSORS_NUM = 2;
const uint8_t sendPins[LAWN_SENSORS_NUM] = { 1, 3 };
const uint8_t receivePins[LAWN_SENSORS_NUM] = { 2, 4 };

TEST_GROUP(LawnSensorsInit)
{
  LawnSensor lawnSensorArray[LAWN_SENSORS_NUM];
  LawnSensors* lawnSensors_p;

  void setup()
  {
  }

  void teardown()
  {
    if (lawnSensors_p)
    {
      delete lawnSensors_p;
    }
    mock().clear();
  }
};

TEST(LawnSensorsInit, defaultInit)
{
  lawnSensors_p = new LawnSensors();

  UNSIGNED_LONGS_EQUAL(0, lawnSensors_p->m_len);
  POINTERS_EQUAL(nullptr, lawnSensors_p->m_lawnSensorArray_p);
  CHECK_FALSE(lawnSensors_p->m_detected);
  UNSIGNED_LONGS_EQUAL(0, lawnSensors_p->m_counter);
  CHECK_FALSE(lawnSensors_p->m_use);
}

TEST(LawnSensorsInit, parameterizedInit)
{
  lawnSensors_p = new LawnSensors(lawnSensorArray, LAWN_SENSORS_NUM);

  UNSIGNED_LONGS_EQUAL(LAWN_SENSORS_NUM, lawnSensors_p->m_len);
  CHECK_FALSE(lawnSensors_p->m_detected);
  UNSIGNED_LONGS_EQUAL(0, lawnSensors_p->m_counter);
  CHECK_FALSE(lawnSensors_p->m_use);
  CHECK_FALSE(nullptr == lawnSensors_p->m_lawnSensorArray_p);

  mock().checkExpectations();
}

// LawnSensors ----------------------------------------------------------------

TEST_GROUP(LawnSensors)
{
  LawnSensor lawnSensorArray[LAWN_SENSORS_NUM];
  LawnSensors* lawnSensors_p;

  void setup()
  {
    mock().disable();
    lawnSensors_p = new LawnSensors(lawnSensorArray, LAWN_SENSORS_NUM);
    mock().enable();
  }

  void teardown()
  {
    if (lawnSensors_p)
    {
      delete lawnSensors_p;
    }
    mock().clear();
  }
};

TEST(LawnSensors, read)
{
  const float accel = 0.03f;

  mock().disable();
  lawnSensorArray[0].setup(1, 2);
  lawnSensorArray[1].setup(3, 4);
  mock().enable();

  mock().expectOneCall("digitalWrite")
      .withIntParameter("pin", 1)
      .withIntParameter("val", HIGH);
  mock().expectNCalls(5, "digitalRead")
      .withIntParameter("pin", 2)
      .andReturnValue(LOW);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 2)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalWrite")
      .withIntParameter("pin", 1)
      .withIntParameter("val", LOW);

  mock().expectOneCall("digitalWrite")
      .withIntParameter("pin", 3)
      .withIntParameter("val", HIGH);
  mock().expectNCalls(5, "digitalRead")
      .withIntParameter("pin", 4)
      .andReturnValue(LOW);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 4)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalWrite")
      .withIntParameter("pin", 3)
      .withIntParameter("val", LOW);

  lawnSensors_p->read();

  DOUBLES_EQUAL(accel * 5.0f, lawnSensors_p->m_lawnSensorArray_p[0].m_value,
      0.01);
  DOUBLES_EQUAL(accel * 5.0f, lawnSensors_p->m_lawnSensorArray_p[1].m_value,
      0.01);

  mock().checkExpectations();
}

TEST(LawnSensors, isDetected)
{
  CHECK_FALSE(lawnSensors_p->isDetected());

  lawnSensors_p->m_detected = true;

  CHECK_TRUE(lawnSensors_p->isDetected());
}

TEST(LawnSensors, clearDetected)
{
  lawnSensors_p->m_detected = true;
  CHECK_TRUE(lawnSensors_p->isDetected());

  lawnSensors_p->clearDetected();

  CHECK_FALSE(lawnSensors_p->isDetected());
}

TEST(LawnSensors, getCounter)
{
  UNSIGNED_LONGS_EQUAL(0, lawnSensors_p->getCounter());

  lawnSensors_p->m_counter = 12345;

  UNSIGNED_LONGS_EQUAL(12345, lawnSensors_p->getCounter());
}

TEST(LawnSensors, simDetected)
{
  CHECK_FALSE(lawnSensors_p->isDetected());
  UNSIGNED_LONGS_EQUAL(0, lawnSensors_p->getCounter());

  lawnSensors_p->simDetected();

  CHECK_TRUE(lawnSensors_p->isDetected());
  UNSIGNED_LONGS_EQUAL(1, lawnSensors_p->getCounter());
}

TEST(LawnSensors, checkNoDetect)
{
  lawnSensors_p->m_lawnSensorArray_p[0].m_value = 0.951f;
  lawnSensors_p->m_lawnSensorArray_p[0].m_valueOld = 1.0f;
  lawnSensors_p->m_lawnSensorArray_p[1].m_value = 0.951f * 2;
  lawnSensors_p->m_lawnSensorArray_p[1].m_valueOld = 1.0f * 2;

  lawnSensors_p->check();

  UNSIGNED_LONGS_EQUAL(0, lawnSensors_p->getCounter());
  CHECK_FALSE(lawnSensors_p->isDetected());
  DOUBLES_EQUAL(0.951f, lawnSensors_p->m_lawnSensorArray_p[0].m_valueOld,
      0.01);
  DOUBLES_EQUAL(0.951f *2, lawnSensors_p->m_lawnSensorArray_p[1].m_valueOld,
      0.01);
}

TEST(LawnSensors, check0Detect)
{
  lawnSensors_p->m_lawnSensorArray_p[0].m_value = 0.949f;
  lawnSensors_p->m_lawnSensorArray_p[0].m_valueOld = 1.0f;
  lawnSensors_p->m_lawnSensorArray_p[1].m_value = 0.951f * 2;
  lawnSensors_p->m_lawnSensorArray_p[1].m_valueOld = 1.0f * 2;

  lawnSensors_p->check();

  UNSIGNED_LONGS_EQUAL(1, lawnSensors_p->getCounter());
  CHECK_TRUE(lawnSensors_p->isDetected());
  DOUBLES_EQUAL(0.949f, lawnSensors_p->m_lawnSensorArray_p[0].m_valueOld, 0.01);
  DOUBLES_EQUAL(0.951f *2, lawnSensors_p->m_lawnSensorArray_p[1].m_valueOld, 0.01);
}

TEST(LawnSensors, check1Detect)
{
  lawnSensors_p->m_lawnSensorArray_p[0].m_value = 0.951f;
  lawnSensors_p->m_lawnSensorArray_p[0].m_valueOld = 1.0f;
  lawnSensors_p->m_lawnSensorArray_p[1].m_value = 0.949f * 2;
  lawnSensors_p->m_lawnSensorArray_p[1].m_valueOld = 1.0f * 2;

  lawnSensors_p->check();

  UNSIGNED_LONGS_EQUAL(1, lawnSensors_p->getCounter());
  CHECK_TRUE(lawnSensors_p->isDetected());
  DOUBLES_EQUAL(0.951f, lawnSensors_p->m_lawnSensorArray_p[0].m_valueOld, 0.01);
  DOUBLES_EQUAL(0.949f *2, lawnSensors_p->m_lawnSensorArray_p[1].m_valueOld, 0.01);
}

TEST(LawnSensors, checkBothDetect)
{
  lawnSensors_p->m_lawnSensorArray_p[0].m_value = 0.949f;
  lawnSensors_p->m_lawnSensorArray_p[0].m_valueOld = 1.0f;
  lawnSensors_p->m_lawnSensorArray_p[1].m_value = 0.949f * 2;
  lawnSensors_p->m_lawnSensorArray_p[1].m_valueOld = 1.0f * 2;

  lawnSensors_p->check();

  UNSIGNED_LONGS_EQUAL(1, lawnSensors_p->getCounter());
  CHECK_TRUE(lawnSensors_p->isDetected());
  DOUBLES_EQUAL(0.949f, lawnSensors_p->m_lawnSensorArray_p[0].m_valueOld, 0.01);
  DOUBLES_EQUAL(0.949f *2, lawnSensors_p->m_lawnSensorArray_p[1].m_valueOld, 0.01);
}
