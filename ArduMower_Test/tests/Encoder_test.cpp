#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Encoder.h"

// EncoderInit ------------------------------------------------------------

TEST_GROUP(EncoderInit)
{
  Encoder* encoder_p;

  void setup()
  {
  }

  void teardown()
  {
    if (encoder_p)
    {
      delete encoder_p;
    }
    mock().clear();
  }
};

TEST(EncoderInit, defaultInit)
{
  encoder_p = new Encoder();

  UNSIGNED_LONGS_EQUAL(0, encoder_p->m_pin);
  CHECK(!encoder_p->m_swapDir);
  CHECK_EQUAL(LOW, encoder_p->m_curState);
  CHECK_EQUAL(LOW, encoder_p->m_lastState);
  LONGS_EQUAL(0, encoder_p->m_counter);
  LONGS_EQUAL(0, encoder_p->m_wheelRpmCurr);
}

TEST(EncoderInit, defaultInit_setup)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT_PULLUP);

  encoder_p = new Encoder();
  encoder_p->setup(1, true);

  UNSIGNED_LONGS_EQUAL(1, encoder_p->m_pin);
  CHECK(encoder_p->m_swapDir);
  CHECK_EQUAL(LOW, encoder_p->m_curState);
  CHECK_EQUAL(LOW, encoder_p->m_lastState);
  LONGS_EQUAL(0, encoder_p->m_counter);
  LONGS_EQUAL(0, encoder_p->m_wheelRpmCurr);

  mock().checkExpectations();
}

TEST(EncoderInit, parameterizedInit)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT_PULLUP);

  encoder_p = new Encoder(1, true);

  UNSIGNED_LONGS_EQUAL(1, encoder_p->m_pin);
  CHECK(encoder_p->m_swapDir);
  CHECK_EQUAL(LOW, encoder_p->m_curState);
  CHECK_EQUAL(LOW, encoder_p->m_lastState);
  LONGS_EQUAL(0, encoder_p->m_counter);
  LONGS_EQUAL(0, encoder_p->m_wheelRpmCurr);

  mock().checkExpectations();
}

// Encoder ------------------------------------------------------------

TEST_GROUP(Encoder)
{
  Encoder* encoder_p;

  void setup()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", INPUT_PULLUP);

    encoder_p = new Encoder(1, false);

    mock().checkExpectations();
  }

  void teardown()
  {
    if (encoder_p)
    {
      delete encoder_p;
    }
    mock().clear();
  }
};

TEST(Encoder, readLowLow)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  encoder_p->read();
  CHECK_EQUAL(LOW, encoder_p->m_curState);
  CHECK_EQUAL(LOW, encoder_p->m_lastState);
  LONGS_EQUAL(0, encoder_p->m_counter);

  encoder_p->read();
  CHECK_EQUAL(LOW, encoder_p->m_curState);
  CHECK_EQUAL(LOW, encoder_p->m_lastState);
  LONGS_EQUAL(0, encoder_p->m_counter);

  mock().checkExpectations();
}

TEST(Encoder, readLowHigh)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  encoder_p->read();
  CHECK_EQUAL(LOW, encoder_p->m_curState);
  CHECK_EQUAL(LOW, encoder_p->m_lastState);
  LONGS_EQUAL(0, encoder_p->m_counter);

  encoder_p->read();
  CHECK_EQUAL(HIGH, encoder_p->m_curState);
  CHECK_EQUAL(HIGH, encoder_p->m_lastState);
  LONGS_EQUAL(1, encoder_p->m_counter);

  mock().checkExpectations();
}

TEST(Encoder, readHighLow)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  encoder_p->read();
  CHECK_EQUAL(HIGH, encoder_p->m_curState);
  CHECK_EQUAL(HIGH, encoder_p->m_lastState);
  LONGS_EQUAL(1, encoder_p->m_counter);


  encoder_p->read();
  CHECK_EQUAL(LOW, encoder_p->m_curState);
  CHECK_EQUAL(LOW, encoder_p->m_lastState);
  LONGS_EQUAL(2, encoder_p->m_counter);

  mock().checkExpectations();
}

TEST(Encoder, readHighHigh)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  encoder_p->read();
  CHECK_EQUAL(HIGH, encoder_p->m_curState);
  CHECK_EQUAL(HIGH, encoder_p->m_lastState);
  LONGS_EQUAL(1, encoder_p->m_counter);

  encoder_p->read();
  CHECK_EQUAL(HIGH, encoder_p->m_curState);
  CHECK_EQUAL(HIGH, encoder_p->m_lastState);
  LONGS_EQUAL(1, encoder_p->m_counter);

  mock().checkExpectations();
}

TEST(Encoder, readHighLowSwapDir)
{
  encoder_p->m_swapDir = true;

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  encoder_p->read();
  CHECK_EQUAL(HIGH, encoder_p->m_curState);
  CHECK_EQUAL(HIGH, encoder_p->m_lastState);
  LONGS_EQUAL(-1, encoder_p->m_counter);


  encoder_p->read();
  CHECK_EQUAL(LOW, encoder_p->m_curState);
  CHECK_EQUAL(LOW, encoder_p->m_lastState);
  LONGS_EQUAL(-2, encoder_p->m_counter);

  mock().checkExpectations();
}

TEST(Encoder, readHighLowNegWheelRpm)
{
  encoder_p->m_wheelRpmCurr = -23;

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalRead")
        .withIntParameter("pin", 1)
        .andReturnValue(LOW);

  encoder_p->read();
  CHECK_EQUAL(HIGH, encoder_p->m_curState);
  CHECK_EQUAL(HIGH, encoder_p->m_lastState);
  LONGS_EQUAL(-1, encoder_p->m_counter);

  encoder_p->read();
  CHECK_EQUAL(LOW, encoder_p->m_curState);
  CHECK_EQUAL(LOW, encoder_p->m_lastState);
  LONGS_EQUAL(-2, encoder_p->m_counter);

  mock().checkExpectations();
}

TEST(Encoder, clearCounter)
{
  encoder_p->m_counter = 32;
  encoder_p->clearCounter();
  LONGS_EQUAL(0, encoder_p->m_counter);
}

TEST(Encoder, getWheelRpmCurr)
{
  encoder_p->m_wheelRpmCurr = 12345;
  LONGS_EQUAL(12345, encoder_p->getWheelRpmCurr());
}

TEST(Encoder, setWheelRpmCurr)
{
  encoder_p->setWheelRpmCurr(-456);
  LONGS_EQUAL(-456, encoder_p->getWheelRpmCurr());
}
