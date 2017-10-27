#include "Sonar.h"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

Sonar sonar;

// Sonar ------------------------------------------------------------

TEST_GROUP(Sonar)
{
  void setup()
  {
    mock().expectOneCall("digitalWrite")
        .withIntParameter("pin", 1)
        .withIntParameter("val", LOW);
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", OUTPUT);
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 2)
        .withIntParameter("mode", INPUT);

    mock().expectOneCall("digitalPinToBitMask")
        .withIntParameter("pin", 1)
        .andReturnValue(0x01);
    mock().expectOneCall("digitalPinToBitMask")
        .withIntParameter("pin", 2)
        .andReturnValue(0x02);;

    mock().expectOneCall("digitalPinToPort")
        .withIntParameter("pin", 1)
        .andReturnValue(4);
    mock().expectOneCall("digitalPinToPort")
        .withIntParameter("pin", 2)
        .andReturnValue(8);

    mock().expectOneCall("portOutputRegister")
        .withIntParameter("pin", 4)
        .andReturnValue((uint8_t*)0x10);
    mock().expectOneCall("portInputRegister")
        .withIntParameter("pin", 8)
        .andReturnValue((uint8_t*)0x20);

    sonar_setup(1, 2, SONAR_DEFAULT_MAX_ECHO_TIME, SONAR_DEFAULT_MIN_ECHO_TIME, &sonar);

    mock().checkExpectations();
  }

  void teardown()
  {
    mock().clear();
  }
};

TEST(Sonar, Setup)
{
  CHECK_EQUAL(1, sonar.triggerPin);
  CHECK_EQUAL(2, sonar.echoPin);
  CHECK_EQUAL(3000, sonar.maxEchoTime);
  CHECK_EQUAL(150, sonar.minEchoTime);
  CHECK_EQUAL(1, sonar.triggerBitMask);
  CHECK_EQUAL(2, sonar.echoBitMask);
  CHECK_EQUAL((uint8_t*)0x10, sonar.triggerOutputRegister_p);
  CHECK_EQUAL((uint8_t*)0x20, sonar.echoInputRegister_p);
}