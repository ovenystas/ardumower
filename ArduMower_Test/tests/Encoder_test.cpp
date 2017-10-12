#include "Encoder.h"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

Encoder encoder;

// EncoderGroup ------------------------------------------------------------

TEST_GROUP(EncoderGroup)
{
  void setup()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", INPUT_PULLUP);
    encoder_setup(1, false, &encoder);
  }

  void teardown()
  {
    mock().clear();
  }
};

TEST(EncoderGroup, setup)
{
  CHECK_EQUAL(1, encoder.pin);
  CHECK_EQUAL(false, encoder.swapDir);
  CHECK_EQUAL(LOW, encoder.curState);
  CHECK_EQUAL(LOW, encoder.lastState);
  CHECK_EQUAL(0, encoder.counter);
  CHECK_EQUAL(0, encoder.wheelRpmCurr);
}

TEST(EncoderGroup, readLowLow)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  encoder_read(&encoder);
  CHECK_EQUAL(LOW, encoder.curState);
  CHECK_EQUAL(LOW, encoder.lastState);
  CHECK_EQUAL(0, encoder.counter);

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  encoder_read(&encoder);
  CHECK_EQUAL(LOW, encoder.curState);
  CHECK_EQUAL(LOW, encoder.lastState);
  CHECK_EQUAL(0, encoder.counter);
}

TEST(EncoderGroup, readLowHigh)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  encoder_read(&encoder);
  CHECK_EQUAL(LOW, encoder.curState);
  CHECK_EQUAL(LOW, encoder.lastState);
  CHECK_EQUAL(0, encoder.counter);

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  encoder_read(&encoder);
  CHECK_EQUAL(HIGH, encoder.curState);
  CHECK_EQUAL(HIGH, encoder.lastState);
  CHECK_EQUAL(1, encoder.counter);
}

TEST(EncoderGroup, readHighLow)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  encoder_read(&encoder);
  CHECK_EQUAL(HIGH, encoder.curState);
  CHECK_EQUAL(HIGH, encoder.lastState);
  CHECK_EQUAL(1, encoder.counter);

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  encoder_read(&encoder);
  CHECK_EQUAL(LOW, encoder.curState);
  CHECK_EQUAL(LOW, encoder.lastState);
  CHECK_EQUAL(2, encoder.counter);
}

TEST(EncoderGroup, readHighHigh)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  encoder_read(&encoder);
  CHECK_EQUAL(HIGH, encoder.curState);
  CHECK_EQUAL(HIGH, encoder.lastState);
  CHECK_EQUAL(1, encoder.counter);

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  encoder_read(&encoder);
  CHECK_EQUAL(HIGH, encoder.curState);
  CHECK_EQUAL(HIGH, encoder.lastState);
  CHECK_EQUAL(1, encoder.counter);
}

TEST(EncoderGroup, readHighLowSwapDir)
{
  encoder.swapDir = true;
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  encoder_read(&encoder);
  CHECK_EQUAL(HIGH, encoder.curState);
  CHECK_EQUAL(HIGH, encoder.lastState);
  CHECK_EQUAL(-1, encoder.counter);

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  encoder_read(&encoder);
  CHECK_EQUAL(LOW, encoder.curState);
  CHECK_EQUAL(LOW, encoder.lastState);
  CHECK_EQUAL(-2, encoder.counter);
}

TEST(EncoderGroup, readHighLowNegWheelRpm)
{
  encoder.wheelRpmCurr = -23;
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  encoder_read(&encoder);
  CHECK_EQUAL(HIGH, encoder.curState);
  CHECK_EQUAL(HIGH, encoder.lastState);
  CHECK_EQUAL(-1, encoder.counter);

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  encoder_read(&encoder);
  CHECK_EQUAL(LOW, encoder.curState);
  CHECK_EQUAL(LOW, encoder.lastState);
  CHECK_EQUAL(-2, encoder.counter);
}

TEST(EncoderGroup, clearCounter)
{
  encoder.counter = 32;
  encoder_clearCounter(&encoder);
  CHECK_EQUAL(0, encoder.counter);
}

TEST(EncoderGroup, getWheelRpmCurr)
{
  encoder.wheelRpmCurr = 12345;
  CHECK_EQUAL(12345, encoder_getWheelRpmCurr(&encoder));
}

TEST(EncoderGroup, setWheelRpmCurr)
{
  encoder_setWheelRpmCurr(-456, &encoder);
  CHECK_EQUAL(-456, encoder.wheelRpmCurr);
}
