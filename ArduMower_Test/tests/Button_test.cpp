#include "Button.h"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

Button button;

// ButtonGroup ------------------------------------------------------------

TEST_GROUP(ButtonGroup)
{
  void setup()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", INPUT_PULLUP);
    button_setup(1, &button);
    mock().checkExpectations();
  }

  void teardown()
  {
    mock().clear();
  }
};

TEST(ButtonGroup, Setup)
{
  CHECK_EQUAL(true, button.use);
  CHECK_EQUAL(1, button.pin);
  CHECK_EQUAL(0, button.lastRun);
  CHECK_EQUAL(0, button.counter);
}

TEST(ButtonGroup, isPressed)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);
  CHECK_EQUAL(false, button_isPressed(&button));

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);
  CHECK_EQUAL(true, button_isPressed(&button));
  mock().checkExpectations();
}

TEST(ButtonGroup, getCounter)
{
  CHECK_EQUAL(0, button_getCounter(&button));
  button.counter = 255;
  CHECK_EQUAL(255, button_getCounter(&button));
}

TEST(ButtonGroup, incCounter)
{
  CHECK_EQUAL(0, button.counter);
  button_incCounter(&button);
  CHECK_EQUAL(1, button.counter);
}

TEST(ButtonGroup, clearCounter)
{
  button.counter = 255;
  CHECK_EQUAL(255, button.counter);
  button_clearCounter(&button);
  CHECK_EQUAL(0, button.counter);
}

TEST(ButtonGroup, isTimeToRun)
{
  mock().expectOneCall("millis").andReturnValue(500u);
  CHECK_EQUAL(false, button_isTimeToRun(&button));
  CHECK_EQUAL(0, button.lastRun);

  mock().expectOneCall("millis").andReturnValue(999u);
  CHECK_EQUAL(false, button_isTimeToRun(&button));
  CHECK_EQUAL(0, button.lastRun);

  mock().expectOneCall("millis").andReturnValue(1000u);
  CHECK_EQUAL(true, button_isTimeToRun(&button));
  CHECK_EQUAL(1000, button.lastRun);

  mock().expectOneCall("millis").andReturnValue(1001u);
  CHECK_EQUAL(false, button_isTimeToRun(&button));
  CHECK_EQUAL(1000, button.lastRun);

  mock().expectOneCall("millis").andReturnValue(2000u);
  CHECK_EQUAL(true, button_isTimeToRun(&button));
  CHECK_EQUAL(2000, button.lastRun);

  button.use = false;

  CHECK_EQUAL(false, button_isTimeToRun(&button));
  CHECK_EQUAL(2000, button.lastRun);

  mock().checkExpectations();
}
