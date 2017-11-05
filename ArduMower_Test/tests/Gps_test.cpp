#include "Gps.h"
#include "Gps.cpp"

#include <HardwareSerial.h>
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

Gps gps;

TEST_GROUP(Gps)
{

  void setup()
  {
  }

  void teardown()
  {
  }
};

TEST(Gps, Init)
{
  gps.init();

  CHECK_EQUAL(9600, Serial3.getBaud());
}

TEST(Gps, cardinal)
{
  STRCMP_EQUAL("N", gps.cardinal(0.0f));
  STRCMP_EQUAL("NNE", gps.cardinal(22.5f));
  STRCMP_EQUAL("NE", gps.cardinal(45.0f));
  STRCMP_EQUAL("ENE", gps.cardinal(67.5f));
  STRCMP_EQUAL("E", gps.cardinal(90.0f));
  STRCMP_EQUAL("ESE", gps.cardinal(112.5f));
  STRCMP_EQUAL("SE", gps.cardinal(135.0f));
  STRCMP_EQUAL("SSE", gps.cardinal(157.5f));
  STRCMP_EQUAL("S", gps.cardinal(180.0f));
  STRCMP_EQUAL("SSW", gps.cardinal(202.5f));
  STRCMP_EQUAL("SW", gps.cardinal(225.0f));
  STRCMP_EQUAL("WSW", gps.cardinal(247.5f));
  STRCMP_EQUAL("W", gps.cardinal(270.0f));
  STRCMP_EQUAL("WNW", gps.cardinal(292.5f));
  STRCMP_EQUAL("NW", gps.cardinal(315.0f));
  STRCMP_EQUAL("NNW", gps.cardinal(337.5f));
  STRCMP_EQUAL("N", gps.cardinal(360.0f));
}
