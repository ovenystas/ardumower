#include "RainSensor.h"
#include "RainSensor.cpp"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

RainSensor rainSensor;

// Mocked functions -----------------------------------------------------------

void pinMode(uint8_t pin, uint8_t mode)
{
  mock().actualCall("pinMode")
      .withIntParameter("pin", pin)
      .withIntParameter("mode", mode);
}

int digitalRead(uint8_t pin)
{
  mock().actualCall("digitalRead")
        .withIntParameter("pin", pin);
  return mock().intReturnValue();
}

// RainSensorGroup ------------------------------------------------------------

TEST_GROUP(RainSensorGroup)
{
  void setup()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", INPUT);
    rainSensor_setup(1, &rainSensor);
  }

  void teardown()
  {
    mock().clear();
  }
};

TEST(RainSensorGroup, Setup)
{
  CHECK_EQUAL(false, rainSensor.use);
  CHECK_EQUAL(1, rainSensor.pin);
  CHECK_EQUAL(false, rainSensor.raining);
  CHECK_EQUAL(0, rainSensor.counter);
}

TEST(RainSensorGroup, CheckOnceHigh)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  rainSensor_check(&rainSensor);
  CHECK_EQUAL(false, rainSensor_isRaining(&rainSensor));
  CHECK_EQUAL(0, rainSensor_getCounter(&rainSensor));
}

TEST(RainSensorGroup, CheckOnceLow)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  rainSensor_check(&rainSensor);
  CHECK_EQUAL(true, rainSensor_isRaining(&rainSensor));
  CHECK_EQUAL(1, rainSensor_getCounter(&rainSensor));
}

TEST(RainSensorGroup, CheckTwiceHigh)
{
  mock().expectNCalls(2, "digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  rainSensor_check(&rainSensor);
  CHECK_EQUAL(false, rainSensor_isRaining(&rainSensor));
  CHECK_EQUAL(0, rainSensor_getCounter(&rainSensor));

  rainSensor_check(&rainSensor);
  CHECK_EQUAL(false, rainSensor_isRaining(&rainSensor));
  CHECK_EQUAL(0, rainSensor_getCounter(&rainSensor));
}

TEST(RainSensorGroup, CheckTwiceLow)
{
  mock().expectNCalls(2, "digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  rainSensor_check(&rainSensor);
  CHECK_EQUAL(true, rainSensor_isRaining(&rainSensor));
  CHECK_EQUAL(1, rainSensor_getCounter(&rainSensor));

  rainSensor_check(&rainSensor);
  CHECK_EQUAL(true, rainSensor_isRaining(&rainSensor));
  CHECK_EQUAL(2, rainSensor_getCounter(&rainSensor));
}

TEST(RainSensorGroup, CheckLowHigh)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);
  rainSensor_check(&rainSensor);
  CHECK_EQUAL(true, rainSensor_isRaining(&rainSensor));
  CHECK_EQUAL(1, rainSensor_getCounter(&rainSensor));

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);
  rainSensor_check(&rainSensor);
  CHECK_EQUAL(false, rainSensor_isRaining(&rainSensor));
  CHECK_EQUAL(1, rainSensor_getCounter(&rainSensor));
}

TEST(RainSensorGroup, CheckHighLow)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);
  rainSensor_check(&rainSensor);
  CHECK_EQUAL(false, rainSensor_isRaining(&rainSensor));
  CHECK_EQUAL(0, rainSensor_getCounter(&rainSensor));

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);
  rainSensor_check(&rainSensor);
  CHECK_EQUAL(true, rainSensor_isRaining(&rainSensor));
  CHECK_EQUAL(1, rainSensor_getCounter(&rainSensor));
}

