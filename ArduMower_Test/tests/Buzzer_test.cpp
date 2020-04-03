#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Buzzer.h"

const uint16_t frequency_hz = 4200;
const uint32_t durationShort_ms = 50;
const uint32_t durationLong_ms = 500;
const uint32_t delayShort_ms = 250;
const uint32_t delayLong_ms = 500;

// BuzzerInit ------------------------------------------------------------

TEST_GROUP(BuzzerInit)
{
  Buzzer* buzzer_p;

  void setup()
  {
  }

  void teardown()
  {
    if (buzzer_p)
    {
      delete buzzer_p;
    }
    mock().clear();
  }
};

TEST(BuzzerInit, init)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", OUTPUT);
  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 1)
      .withParameter("val", LOW);

  buzzer_p = new Buzzer(1);

  UNSIGNED_LONGS_EQUAL(1, buzzer_p->m_pin);
  CHECK(!buzzer_p->isEnabled());

  mock().checkExpectations();
}

TEST(BuzzerInit, initEnabled)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", OUTPUT);
  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 1)
      .withParameter("val", LOW);

  buzzer_p = new Buzzer(1, true);

  UNSIGNED_LONGS_EQUAL(1, buzzer_p->m_pin);
  CHECK(buzzer_p->isEnabled());

  mock().checkExpectations();
}

TEST(BuzzerInit, initThenEnable)
{
  mock().disable();
  buzzer_p = new Buzzer(1);
  mock().enable();

  buzzer_p->setEnabled(true);

  CHECK(buzzer_p->isEnabled());

  mock().checkExpectations();
}

// Buzzer ------------------------------------------------------------
TEST_GROUP(Buzzer)
{
  Buzzer* buzzer_p;

  void setup()
  {
    mock().disable();
    buzzer_p = new Buzzer(1);
    mock().enable();
  }

  void teardown()
  {
    if (buzzer_p)
    {
      delete buzzer_p;
    }
    mock().clear();
  }
};

TEST(Buzzer, beepShort_disabled)
{
  mock().expectNoCall("tone");
  mock().expectNoCall("noTone");
  mock().expectNoCall("delay");

  buzzer_p->beepShort(1);

  mock().checkExpectations();
}

TEST(Buzzer, beepShort_0Beeps)
{
  buzzer_p->setEnabled(true);

  mock().expectNoCall("tone");
  mock().expectNoCall("noTone");
  mock().expectNoCall("delay");

  buzzer_p->beepShort(0);

  mock().checkExpectations();
}

TEST(Buzzer, beepShort_1Beep)
{
  buzzer_p->setEnabled(true);

  mock().expectOneCall("tone")
      .withParameter("_pin", 1)
      .withParameter("frequency", frequency_hz)
      .withParameter("duration", durationShort_ms);
  mock().expectOneCall("delay")
      .withParameter("ms", delayShort_ms);

  buzzer_p->beepShort(1);

  mock().checkExpectations();
}

TEST(Buzzer, beepShort_255Beeps)
{
  buzzer_p->setEnabled(true);

  mock().expectNCalls(255, "tone")
      .withParameter("_pin", 1)
      .withParameter("frequency", frequency_hz)
      .withParameter("duration", durationShort_ms);
  mock().expectNCalls(255, "delay")
      .withParameter("ms", delayShort_ms);

  buzzer_p->beepShort(255);

  mock().checkExpectations();
}

TEST(Buzzer, beepLong_disabled)
{
  mock().expectNoCall("tone");
  mock().expectNoCall("noTone");
  mock().expectNoCall("delay");

  buzzer_p->beepLong(1);

  mock().checkExpectations();
}

TEST(Buzzer, beepLong_0Beeps)
{
  buzzer_p->setEnabled(true);

  mock().expectNoCall("tone");
  mock().expectNoCall("noTone");
  mock().expectNoCall("delay");

  buzzer_p->beepLong(0);

  mock().checkExpectations();
}

TEST(Buzzer, beepLong_1Beep)
{
  buzzer_p->setEnabled(true);

  mock().expectOneCall("tone")
      .withParameter("_pin", 1)
      .withParameter("frequency", frequency_hz)
      .withParameter("duration", durationLong_ms);
  mock().expectOneCall("delay")
      .withParameter("ms", delayLong_ms);

  buzzer_p->beepLong(1);

  mock().checkExpectations();
}

TEST(Buzzer, beepLong_255Beeps)
{
  buzzer_p->setEnabled(true);

  mock().expectNCalls(255, "tone")
      .withParameter("_pin", 1)
      .withParameter("frequency", frequency_hz)
      .withParameter("duration", durationLong_ms);
  mock().expectNCalls(255, "delay")
      .withParameter("ms", delayLong_ms);

  buzzer_p->beepLong(255);

  mock().checkExpectations();
}

TEST(Buzzer, beep_440HzContinuous)
{
  buzzer_p->setEnabled(true);

  mock().expectOneCall("tone")
      .withParameter("_pin", 1)
      .withParameter("frequency", 440)
      .withParameter("duration", 0);

  buzzer_p->beep(440);

  mock().checkExpectations();
}

TEST(Buzzer, beep_440Hz200ms)
{
  buzzer_p->setEnabled(true);

  mock().expectOneCall("tone")
      .withParameter("_pin", 1)
      .withParameter("frequency", 440)
      .withParameter("duration", 200);

  buzzer_p->beep(440, 200);

  mock().checkExpectations();
}

TEST(Buzzer, beep_BeepData_880Hz400ms)
{
  buzzer_p->setEnabled(true);

  mock().expectOneCall("tone")
      .withParameter("_pin", 1)
      .withParameter("frequency", 880)
      .withParameter("duration", 400);

  BeepData data = { 880, 400 };
  buzzer_p->beep(&data);

  mock().checkExpectations();
}

TEST(Buzzer, beep_BeepData_880Hz400ms_1720Hz1000ms)
{
  buzzer_p->setEnabled(true);

  mock().expectOneCall("tone")
      .withParameter("_pin", 1)
      .withParameter("frequency", 880)
      .withParameter("duration", 400);
  mock().expectOneCall("tone")
      .withParameter("_pin", 1)
      .withParameter("frequency", 1720)
      .withParameter("duration", 1000);

  BeepData data[] =
  {
      { 880, 400 },
      { 1720, 1000 }
  };
  buzzer_p->beep(data, sizeof(data) / sizeof(data[0]));

  mock().checkExpectations();
}
