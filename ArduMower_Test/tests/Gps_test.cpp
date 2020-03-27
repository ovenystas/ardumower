#include "Gps.h"

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

TEST(Gps, cardinal_middleInSector)
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

TEST(Gps, cardinal_borderOfSector)
{
  STRCMP_EQUAL("SSE", gps.cardinal(180.0f - 11.25f - 0.001f));
  STRCMP_EQUAL("S", gps.cardinal(180.0f - 11.25f + 0.001f));
  STRCMP_EQUAL("S", gps.cardinal(180.0f + 11.25f - 0.001f));
  STRCMP_EQUAL("SSW", gps.cardinal(180.0f + 11.25f + 0.001f));
}

struct gpsCoord
{
  float latitude;
  float longitude;
};

gpsCoord home( { 57.769980f, 12.279648f });
gpsCoord work( { 57.7087878f, 11.980496f });

TEST(Gps, distance_between)
{
  DOUBLES_EQUAL(0.0f, gps.distance_between(0.0f, 0.0f, 0.0f, 0.0f), 0.001f);
  DOUBLES_EQUAL(0.0f, gps.distance_between(40.0f, 60.0f, 40.0f, 60.0f), 0.001f);
  DOUBLES_EQUAL(19019.85f, gps.distance_between(
      home.latitude, home.longitude, work.latitude, work.longitude), 0.1f);
}

TEST(Gps, course_to)
{
  // North
  DOUBLES_EQUAL(0.0f, gps.course_to(0.0f, 0.0f, 50.0f, 0.0f), 0.001f);
  // South
  DOUBLES_EQUAL(180.0f, gps.course_to(0.0f, 0.0f, -50.0f, 0.0f), 0.001f);
  // East
  DOUBLES_EQUAL(90.0f, gps.course_to(0.0f, 0.0f, 0.0f, 50.0f), 0.001f);
  // West
  DOUBLES_EQUAL(270.0f, gps.course_to(0.0f, 0.0f, 0.0f, -50.0f), 0.001f);
  // home to work
  DOUBLES_EQUAL(249.159, gps.course_to(
      home.latitude, home.longitude, work.latitude, work.longitude), 0.1f);
  // work to home
  DOUBLES_EQUAL(68.90592, gps.course_to(
      work.latitude, work.longitude, home.latitude, home.longitude), 0.1f);
}
