#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Button.h"

Button* p_button = nullptr;

// ButtonGroup ------------------------------------------------------------

TEST_GROUP(ButtonGroup)
{
  void setup()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", INPUT_PULLUP);

    p_button = new Button(1);

    mock().checkExpectations();
  }

  void teardown()
  {
    if (p_button)
    {
      delete p_button;
    }
    mock().clear();
  }
};

TEST(ButtonGroup, init)
{
  CHECK_EQUAL(true, p_button->m_use);
  CHECK_EQUAL(1, p_button->m_pin);
  CHECK_EQUAL(0, p_button->m_lastRun);
  CHECK_EQUAL(0, p_button->m_counter);
}

TEST(ButtonGroup, isPressed)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);
  CHECK_EQUAL(false, p_button->isPressed());

  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);
  CHECK_EQUAL(true, p_button->isPressed());

  mock().checkExpectations();
}

TEST(ButtonGroup, counter)
{
  CHECK_EQUAL(0, p_button->getCounter());
  p_button->incCounter();
  CHECK_EQUAL(1, p_button->getCounter());
  p_button->incCounter();
  CHECK_EQUAL(2, p_button->getCounter());
  p_button->clearCounter();
  CHECK_EQUAL(0, p_button->getCounter());
}

TEST(ButtonGroup, isTimeToRun_unused)
{
  p_button->m_use = false;

  mock().expectNoCall("millis");
  CHECK_EQUAL(false, p_button->isTimeToRun());
  CHECK_EQUAL(0, p_button->m_lastRun);

  mock().checkExpectations();
}

TEST(ButtonGroup, isTimeToRun_used)
{
  mock().expectOneCall("millis").andReturnValue(999u);
  CHECK_EQUAL(false, p_button->isTimeToRun());
  CHECK_EQUAL(0, p_button->m_lastRun);

  mock().expectOneCall("millis").andReturnValue(1000u);
  CHECK_EQUAL(true, p_button->isTimeToRun());
  CHECK_EQUAL(1000, p_button->m_lastRun);

  mock().expectOneCall("millis").andReturnValue(1999u);
  CHECK_EQUAL(false, p_button->isTimeToRun());
  CHECK_EQUAL(1000, p_button->m_lastRun);

  mock().expectOneCall("millis").andReturnValue(2000u);
  CHECK_EQUAL(true, p_button->isTimeToRun());
  CHECK_EQUAL(2000, p_button->m_lastRun);

  mock().checkExpectations();
}
