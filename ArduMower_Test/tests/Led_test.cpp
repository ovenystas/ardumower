#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Led.h"

// LedInit ------------------------------------------------------------

TEST_GROUP(LedInit)
{
  Led* led_p;

  void setup()
  {
  }

  void teardown()
  {
    if (led_p)
    {
      delete led_p;
    }
    mock().clear();
  }
};

TEST(LedInit, init)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", OUTPUT);
  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 1)
      .withParameter("val", LOW);

  led_p = new Led(1);

  UNSIGNED_LONGS_EQUAL(1, led_p->m_pin);

  mock().checkExpectations();
}

TEST(LedInit, initHigh)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", OUTPUT);
  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 1)
      .withParameter("val", HIGH);

  led_p = new Led(1, HIGH);

  UNSIGNED_LONGS_EQUAL(1, led_p->m_pin);

  mock().checkExpectations();
}

TEST(LedInit, initLow)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", OUTPUT);
  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 1)
      .withParameter("val", LOW);

  led_p = new Led(1, LOW);

  UNSIGNED_LONGS_EQUAL(1, led_p->m_pin);

  mock().checkExpectations();
}

// Led ------------------------------------------------------------

TEST_GROUP(Led)
{
  Led* led_p;

  void setup()
  {
    mock().disable();
    led_p = new Led(1);
    mock().enable();
  }

  void teardown()
  {
    if (led_p)
    {
      delete led_p;
    }
    mock().clear();
  }
};

TEST(Led, setHigh)
{
  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 1)
      .withParameter("val", HIGH);

  led_p->setHigh();

  mock().checkExpectations();
}

TEST(Led, setLow)
{
  mock().disable();
  led_p->setHigh();
  mock().enable();

  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 1)
      .withParameter("val", LOW);

  led_p->setLow();

  mock().checkExpectations();
}

TEST(Led, set_High)
{
  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 1)
      .withParameter("val", HIGH);

  led_p->set(HIGH);

  mock().checkExpectations();
}

TEST(Led, set_Low)
{
  mock().disable();
  led_p->setHigh();
  mock().enable();

  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 1)
      .withParameter("val", LOW);

  led_p->set(LOW);

  mock().checkExpectations();
}

TEST(Led, get_high)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(static_cast<int16_t>(HIGH));

  CHECK(led_p->get());
}

TEST(Led, get_low)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(static_cast<int16_t>(LOW));

  CHECK(!led_p->get());
}

TEST(Led, toggle)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(static_cast<int16_t>(LOW));
  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 1)
      .withParameter("val", HIGH);

  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(static_cast<int16_t>(HIGH));
  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 1)
      .withParameter("val", LOW);

  led_p->toggle();
  led_p->toggle();

  mock().checkExpectations();
}
