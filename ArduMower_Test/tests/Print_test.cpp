#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

//// Get access to private class members in SUT
//#define private public
#include "Arduino.h"

TEST_GROUP(Print)
{
  void setup()
  {
    Serial.setMockPrintFunctions(false);
  }

  void teardown()
  {
    Serial.setMockPrintFunctions();
  }
};

TEST(Print, printStr)
{
  Serial.print("\nHello world!\n");
}
