#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Button.h"

// Button ------------------------------------------------------------

TEST_GROUP(Button)
{
  Button* button_p;

  void setup()
  {
    mock().expectOneCall("pinMode")
        .withParameter("pin", 1)
        .withParameter("mode", INPUT_PULLUP);

    button_p = new Button(1);

    mock().checkExpectations();
  }

  void teardown()
  {
    if (button_p)
    {
      delete button_p;
    }
    mock().clear();
  }
};

TEST(Button, init)
{
  CHECK_EQUAL(true, button_p->m_use);
  CHECK_EQUAL(1, button_p->m_pin);
  CHECK_EQUAL(0, button_p->m_lastRun);
  CHECK_EQUAL(0, button_p->m_counter);
}

TEST(Button, isPressed)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(HIGH);
  CHECK_EQUAL(false, button_p->isPressed());

  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(LOW);
  CHECK_EQUAL(true, button_p->isPressed());

  mock().checkExpectations();
}

TEST(Button, counter)
{
  CHECK_EQUAL(0, button_p->getCounter());
  button_p->incCounter();
  CHECK_EQUAL(1, button_p->getCounter());
  button_p->incCounter();
  CHECK_EQUAL(2, button_p->getCounter());
  button_p->clearCounter();
  CHECK_EQUAL(0, button_p->getCounter());
}

TEST(Button, isTimeToRun_unused)
{
  button_p->m_use = false;

  mock().expectNoCall("millis");
  CHECK_EQUAL(false, button_p->isTimeToRun());
  CHECK_EQUAL(0, button_p->m_lastRun);

  mock().checkExpectations();
}

TEST(Button, isTimeToRun_used)
{
  mock().expectOneCall("millis").andReturnValue(999u);
  CHECK_EQUAL(false, button_p->isTimeToRun());
  CHECK_EQUAL(0, button_p->m_lastRun);

  mock().expectOneCall("millis").andReturnValue(1000u);
  CHECK_EQUAL(true, button_p->isTimeToRun());
  CHECK_EQUAL(1000, button_p->m_lastRun);

  mock().expectOneCall("millis").andReturnValue(1999u);
  CHECK_EQUAL(false, button_p->isTimeToRun());
  CHECK_EQUAL(1000, button_p->m_lastRun);

  mock().expectOneCall("millis").andReturnValue(2000u);
  CHECK_EQUAL(true, button_p->isTimeToRun());
  CHECK_EQUAL(2000, button_p->m_lastRun);

  mock().checkExpectations();
}
