#include "DropSensor.h"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

DropSensor dropSensor;

// DropSensorGroup ------------------------------------------------------------

TEST_GROUP(DropSensorGroup)
{
  void setup()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", INPUT_PULLUP);
    dropSensor_setup(1, &dropSensor);
  }

  void teardown()
  {
    mock().clear();
  }
};

TEST(DropSensorGroup, Setup)
{
  CHECK_EQUAL(1, dropSensor.pin);
  CHECK_EQUAL(false, dropSensor.detected);
  CHECK_EQUAL(0, dropSensor.counter);
}

TEST(DropSensorGroup, CheckOnceHighNo)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  dropSensor_check(&dropSensor, DROPSENSOR_NO);
  CHECK_EQUAL(false, dropSensor_isDetected(&dropSensor));
  CHECK_EQUAL(0, dropSensor_getCounter(&dropSensor));
}

TEST(DropSensorGroup, CheckOnceLowNo)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  dropSensor_check(&dropSensor, DROPSENSOR_NO);
  CHECK_EQUAL(true, dropSensor_isDetected(&dropSensor));
  CHECK_EQUAL(1, dropSensor_getCounter(&dropSensor));
}

TEST(DropSensorGroup, CheckOnceHighNc)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  dropSensor_check(&dropSensor, DROPSENSOR_NC);
  CHECK_EQUAL(true, dropSensor_isDetected(&dropSensor));
  CHECK_EQUAL(1, dropSensor_getCounter(&dropSensor));
}

TEST(DropSensorGroup, CheckOnceLowNc)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  dropSensor_check(&dropSensor, DROPSENSOR_NC);
  CHECK_EQUAL(false, dropSensor_isDetected(&dropSensor));
  CHECK_EQUAL(0, dropSensor_getCounter(&dropSensor));
}

TEST(DropSensorGroup, CheckTwiceHighNo)
{
  mock().expectNCalls(2, "digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  dropSensor_check(&dropSensor, DROPSENSOR_NO);
  CHECK_EQUAL(false, dropSensor_isDetected(&dropSensor));
  CHECK_EQUAL(0, dropSensor_getCounter(&dropSensor));

  dropSensor_check(&dropSensor, DROPSENSOR_NO);
  CHECK_EQUAL(false, dropSensor_isDetected(&dropSensor));
  CHECK_EQUAL(0, dropSensor_getCounter(&dropSensor));
}

TEST(DropSensorGroup, CheckTwiceLowNo)
{
  mock().expectNCalls(2, "digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  dropSensor_check(&dropSensor, DROPSENSOR_NO);
  CHECK_EQUAL(true, dropSensor_isDetected(&dropSensor));
  CHECK_EQUAL(1, dropSensor_getCounter(&dropSensor));

  dropSensor_check(&dropSensor, DROPSENSOR_NO);
  CHECK_EQUAL(true, dropSensor_isDetected(&dropSensor));
  CHECK_EQUAL(2, dropSensor_getCounter(&dropSensor));
}

TEST(DropSensorGroup, CheckTwiceSim)
{
  dropSensor_simDetected(&dropSensor);
  CHECK_EQUAL(true, dropSensor_isDetected(&dropSensor));
  CHECK_EQUAL(1, dropSensor_getCounter(&dropSensor));

  dropSensor_simDetected(&dropSensor);
  CHECK_EQUAL(true, dropSensor_isDetected(&dropSensor));
  CHECK_EQUAL(2, dropSensor_getCounter(&dropSensor));
}
