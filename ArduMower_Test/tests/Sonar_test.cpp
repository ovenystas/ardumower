#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Sonar.h"

Sonar sonar;

// SonarInit --------------------------------------------------------

TEST_GROUP(SonarInit)
{
  Sonar* sonar_p;

  void setup()
  {
  }

  void teardown()
  {
    if (sonar_p)
    {
      delete sonar_p;
    }
    mock().clear();
  }

  void mockSetup_defaultInit_setup_and_parameterizedInit()
  {
    mock().expectOneCall("digitalWrite")
        .withParameter("pin", 1)
        .withParameter("val", LOW);
    mock().expectOneCall("pinMode")
        .withParameter("pin", 1)
        .withParameter("mode", OUTPUT);
    mock().expectOneCall("pinMode")
        .withParameter("pin", 2)
        .withParameter("mode", INPUT);

    mock().expectOneCall("digitalPinToBitMask")
        .withParameter("pin", 1)
        .andReturnValue(0x02);
    mock().expectOneCall("digitalPinToBitMask")
        .withParameter("pin", 2)
        .andReturnValue(0x04);;

    mock().expectOneCall("digitalPinToPort")
        .withParameter("pin", 1)
        .andReturnValue(4);
    mock().expectOneCall("digitalPinToPort")
        .withParameter("pin", 2)
        .andReturnValue(8);

    mock().expectOneCall("portOutputRegister")
        .withParameter("port", 4)
        .andReturnValue((uint8_t*)0x10);
    mock().expectOneCall("portInputRegister")
        .withParameter("port", 8)
        .andReturnValue((uint8_t*)0x20);
  }

  void checks_defaultInit_setup_and_parameterizedInit()
  {
    CHECK_FALSE(sonar_p->m_use);
    UNSIGNED_LONGS_EQUAL(1, sonar_p->m_triggerPin);
    UNSIGNED_LONGS_EQUAL(2, sonar_p->m_echoPin);
    UNSIGNED_LONGS_EQUAL(3000, sonar_p->m_maxEchoTime);
    UNSIGNED_LONGS_EQUAL(150, sonar_p->m_minEchoTime);
    UNSIGNED_LONGS_EQUAL(0, sonar_p->m_distance_us);
    UNSIGNED_LONGS_EQUAL(0x02, sonar_p->m_triggerBitMask);
    UNSIGNED_LONGS_EQUAL(0x04, sonar_p->m_echoBitMask);
    POINTERS_EQUAL((uint8_t*)0x10, sonar_p->m_triggerOutputRegister_p);
    POINTERS_EQUAL((uint8_t*)0x20, sonar_p->m_echoInputRegister_p);
    UNSIGNED_LONGS_EQUAL(0, sonar_p->m_maxTime);

    mock().checkExpectations();
  }
};

TEST(SonarInit, defaultInit)
{
  sonar_p = new Sonar();

  CHECK_FALSE(sonar_p->m_use);
  UNSIGNED_LONGS_EQUAL(0, sonar_p->m_triggerPin);
  UNSIGNED_LONGS_EQUAL(0, sonar_p->m_echoPin);
  UNSIGNED_LONGS_EQUAL(0, sonar_p->m_maxEchoTime);
  UNSIGNED_LONGS_EQUAL(0, sonar_p->m_minEchoTime);
  UNSIGNED_LONGS_EQUAL(0, sonar_p->m_distance_us);
  UNSIGNED_LONGS_EQUAL(0, sonar_p->m_triggerBitMask);
  UNSIGNED_LONGS_EQUAL(0, sonar_p->m_echoBitMask);
  POINTERS_EQUAL(nullptr, sonar_p->m_triggerOutputRegister_p);
  POINTERS_EQUAL(nullptr, sonar_p->m_echoInputRegister_p);
  UNSIGNED_LONGS_EQUAL(0, sonar_p->m_maxTime);
}

TEST(SonarInit, defaultInit_setup)
{
  mockSetup_defaultInit_setup_and_parameterizedInit();

  sonar_p = new Sonar();
  sonar_p->setup(1, 2,
      SONAR_DEFAULT_MAX_ECHO_TIME, SONAR_DEFAULT_MIN_ECHO_TIME);

  checks_defaultInit_setup_and_parameterizedInit();
}

TEST(SonarInit, parameterizedInit)
{
  mockSetup_defaultInit_setup_and_parameterizedInit();

  sonar_p = new Sonar(1, 2,
      SONAR_DEFAULT_MAX_ECHO_TIME, SONAR_DEFAULT_MIN_ECHO_TIME);

  checks_defaultInit_setup_and_parameterizedInit();
}

// Sonar ------------------------------------------------------------

TEST_GROUP(Sonar)
{
  Sonar* sonar_p;

  uint8_t fake_triggerOutputRegister;
  uint8_t fake_echoInputRegister;

  void setup()
  {
    mock().expectOneCall("digitalPinToBitMask")
        .withParameter("pin", 1)
        .andReturnValue(0x02);
    mock().expectOneCall("digitalPinToBitMask")
        .withParameter("pin", 2)
        .andReturnValue(0x04);

    mock().expectOneCall("digitalPinToPort")
        .withParameter("pin", 1)
        .andReturnValue(4);
    mock().expectOneCall("digitalPinToPort")
        .withParameter("pin", 2)
        .andReturnValue(8);

    mock().expectOneCall("portOutputRegister")
        .withParameter("port", 4)
        .andReturnValue(&fake_triggerOutputRegister);
    mock().expectOneCall("portInputRegister")
        .withParameter("port", 8)
        .andReturnValue(&fake_echoInputRegister);
    mock().ignoreOtherCalls();

    sonar_p = new Sonar(1, 2,
        SONAR_DEFAULT_MAX_ECHO_TIME, SONAR_DEFAULT_MIN_ECHO_TIME);

    mock().checkExpectations();
  }

  void teardown()
  {
    if (sonar_p)
    {
      delete sonar_p;
    }
    mock().clear();
  }
};


IGNORE_TEST(Sonar, pingTrigger)
{
  mock().expectOneCall("delayMicroseconds")
      .withParameter("us", 4);
  mock().expectOneCall("delayMicroseconds")
      .withParameter("us", 10);
  mock().expectOneCall("micros")
      .andReturnValue(30000);
  mock().expectOneCall("micros")
      .andReturnValue(35000);

  CHECK_FALSE(sonar_p->pingTrigger());

  mock().checkExpectations();
}

IGNORE_TEST(Sonar, pingInternal)
{
  mock().expectOneCall("delayMicroseconds")
      .withUnsignedIntParameter("us", 4);
  mock().expectOneCall("delayMicroseconds")
      .withUnsignedIntParameter("us", 10);
  mock().expectOneCall("micros")
      .andReturnValue(30000);
  mock().expectOneCall("micros")
      .andReturnValue(35000);
  mock().expectOneCall("micros")
      .andReturnValue(40000);

  UNSIGNED_LONGS_EQUAL(2000, sonar_p->pingInternal());

  mock().checkExpectations();
}

IGNORE_TEST(Sonar, ping)
{
  sonar_p->ping();
  UNSIGNED_LONGS_EQUAL(0, sonar_p->getDistance_us());
}
