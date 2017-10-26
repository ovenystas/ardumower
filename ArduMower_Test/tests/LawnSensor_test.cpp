#include "LawnSensor.h"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

LawnSensor lawnSensor;

// LawnSensor ------------------------------------------------------------

TEST_GROUP(LawnSensor)
{
  void setup()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", OUTPUT);
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 2)
        .withIntParameter("mode", INPUT);
    lawnSensor_setup(1, 2, &lawnSensor);

    mock().checkExpectations();
  }

  void teardown()
  {
    mock().clear();
  }
};

TEST(LawnSensor, Setup)
{
  CHECK_EQUAL(1, lawnSensor.sendPin);
  CHECK_EQUAL(2, lawnSensor.receivePin);
  DOUBLES_EQUAL(0.0, lawnSensor.value, 0.01);
  DOUBLES_EQUAL(0.0, lawnSensor.valueOld, 0.01);
}

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

  CHECK_EQUAL(5, lawnSensor_measureLawnCapacity(&lawnSensor));

  mock().checkExpectations();
}

TEST(LawnSensor, getValue)
{
  DOUBLES_EQUAL(0.0, lawnSensor_getValue(&lawnSensor), 0.01);
  lawnSensor.value = 1.2f;
  DOUBLES_EQUAL(1.2, lawnSensor_getValue(&lawnSensor), 0.01);
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

  lawnSensor_read(&lawnSensor);
  DOUBLES_EQUAL(accel * 5.0f, lawnSensor.value, 0.01);

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

  lawnSensor.value = 2.0f;
  lawnSensor_read(&lawnSensor);
  DOUBLES_EQUAL((1.0f - accel) * 2.0f + accel * 5.0f, lawnSensor.value, 0.01);

  mock().checkExpectations();
}

// LawnSensorArray ------------------------------------------------------------

const uint8_t LAWN_SENSORS_NUM = 2;
const uint8_t sendPins[LAWN_SENSORS_NUM] = { 1, 3 };
const uint8_t receivePins[LAWN_SENSORS_NUM] = { 2, 4 };
LawnSensor array[LAWN_SENSORS_NUM];
LawnSensors lawnSensors;

TEST_GROUP(LawnSensorArray)
{
  void setup()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", OUTPUT);
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 2)
        .withIntParameter("mode", INPUT);
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 3)
        .withIntParameter("mode", OUTPUT);
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 4)
        .withIntParameter("mode", INPUT);

    lawnSensors_setup(sendPins, receivePins, array, &lawnSensors, LAWN_SENSORS_NUM);

    mock().checkExpectations();
  }

  void teardown()
  {
    mock().clear();
  }
};

TEST(LawnSensorArray, Setup)
{
  CHECK_EQUAL(false, lawnSensors.use);
  CHECK_EQUAL(LAWN_SENSORS_NUM, lawnSensors.len);
  CHECK_EQUAL(0, lawnSensors.counter);
  CHECK_EQUAL(false, lawnSensors.detected);

  CHECK_EQUAL(1, lawnSensors.lawnSensorArray_p[0].sendPin);
  CHECK_EQUAL(2, lawnSensors.lawnSensorArray_p[0].receivePin);
  DOUBLES_EQUAL(false, lawnSensors.lawnSensorArray_p[0].value, 0.01);
  DOUBLES_EQUAL(0, lawnSensors.lawnSensorArray_p[0].valueOld, 0.01);

  CHECK_EQUAL(3, lawnSensors.lawnSensorArray_p[1].sendPin);
  CHECK_EQUAL(4, lawnSensors.lawnSensorArray_p[1].receivePin);
  DOUBLES_EQUAL(false, lawnSensors.lawnSensorArray_p[1].value, 0.01);
  DOUBLES_EQUAL(0, lawnSensors.lawnSensorArray_p[1].valueOld, 0.01);
}

TEST(LawnSensorArray, read)
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

  lawnSensors_read(&lawnSensors);
  DOUBLES_EQUAL(accel * 5.0f, lawnSensors.lawnSensorArray_p[0].value, 0.01);
  DOUBLES_EQUAL(accel * 5.0f, lawnSensors.lawnSensorArray_p[1].value, 0.01);

  mock().checkExpectations();
}

TEST(LawnSensorArray, isDetected)
{
  CHECK_EQUAL(false, lawnSensors_isDetected(&lawnSensors));

  lawnSensors.detected = true;

  CHECK_EQUAL(true, lawnSensors_isDetected(&lawnSensors));
}

TEST(LawnSensorArray, clearDetected)
{
  lawnSensors.detected = true;
  CHECK_EQUAL(true, lawnSensors.detected);

  lawnSensors_clearDetected(&lawnSensors);

  CHECK_EQUAL(false, lawnSensors.detected);
}

TEST(LawnSensorArray, simDetected)
{
  CHECK_EQUAL(false, lawnSensors.detected);
  CHECK_EQUAL(0, lawnSensors.counter);

  lawnSensors_simDetected(&lawnSensors);

  CHECK_EQUAL(true, lawnSensors.detected);
  CHECK_EQUAL(1, lawnSensors.counter);
}

TEST(LawnSensorArray, getCounter)
{
  CHECK_EQUAL(0, lawnSensors_getCounter(&lawnSensors));

  lawnSensors.counter = 12345;

  CHECK_EQUAL(12345, lawnSensors_getCounter(&lawnSensors));
}

TEST(LawnSensorArray, checkNoDetect)
{
  lawnSensors.lawnSensorArray_p[0].value = 0.951f;
  lawnSensors.lawnSensorArray_p[0].valueOld = 1.0f;
  lawnSensors.lawnSensorArray_p[1].value = 0.951f * 2;
  lawnSensors.lawnSensorArray_p[1].valueOld = 1.0f * 2;

  lawnSensors_check(&lawnSensors);

  CHECK_EQUAL(0, lawnSensors.counter);
  CHECK_EQUAL(false, lawnSensors.detected);
  DOUBLES_EQUAL(0.951f, lawnSensors.lawnSensorArray_p[0].valueOld, 0.01);
  DOUBLES_EQUAL(0.951f *2, lawnSensors.lawnSensorArray_p[1].valueOld, 0.01);
}

TEST(LawnSensorArray, check0Detect)
{
  lawnSensors.lawnSensorArray_p[0].value = 0.949f;
  lawnSensors.lawnSensorArray_p[0].valueOld = 1.0f;
  lawnSensors.lawnSensorArray_p[1].value = 0.951f * 2;
  lawnSensors.lawnSensorArray_p[1].valueOld = 1.0f * 2;

  lawnSensors_check(&lawnSensors);

  CHECK_EQUAL(1, lawnSensors.counter);
  CHECK_EQUAL(true, lawnSensors.detected);
  DOUBLES_EQUAL(0.949f, lawnSensors.lawnSensorArray_p[0].valueOld, 0.01);
  DOUBLES_EQUAL(0.951f *2, lawnSensors.lawnSensorArray_p[1].valueOld, 0.01);
}

TEST(LawnSensorArray, check1Detect)
{
  lawnSensors.lawnSensorArray_p[0].value = 0.951f;
  lawnSensors.lawnSensorArray_p[0].valueOld = 1.0f;
  lawnSensors.lawnSensorArray_p[1].value = 0.949f * 2;
  lawnSensors.lawnSensorArray_p[1].valueOld = 1.0f * 2;

  lawnSensors_check(&lawnSensors);

  CHECK_EQUAL(1, lawnSensors.counter);
  CHECK_EQUAL(true, lawnSensors.detected);
  DOUBLES_EQUAL(0.951f, lawnSensors.lawnSensorArray_p[0].valueOld, 0.01);
  DOUBLES_EQUAL(0.949f *2, lawnSensors.lawnSensorArray_p[1].valueOld, 0.01);
}

TEST(LawnSensorArray, checkBothDetect)
{
  lawnSensors.lawnSensorArray_p[0].value = 0.949f;
  lawnSensors.lawnSensorArray_p[0].valueOld = 1.0f;
  lawnSensors.lawnSensorArray_p[1].value = 0.949f * 2;
  lawnSensors.lawnSensorArray_p[1].valueOld = 1.0f * 2;

  lawnSensors_check(&lawnSensors);

  CHECK_EQUAL(1, lawnSensors.counter);
  CHECK_EQUAL(true, lawnSensors.detected);
  DOUBLES_EQUAL(0.949f, lawnSensors.lawnSensorArray_p[0].valueOld, 0.01);
  DOUBLES_EQUAL(0.949f *2, lawnSensors.lawnSensorArray_p[1].valueOld, 0.01);
}
