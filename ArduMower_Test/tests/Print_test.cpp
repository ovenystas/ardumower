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
    Serial.startCapture();
  }

  void teardown()
  {
    Serial.stopCapture();
    Serial.setMockPrintFunctions(true);
  }
};

TEST(Print, printStr)
{
  Serial.print("Hello");
  Serial.println(" world!");

  STRCMP_CONTAINS("Hello world!\r\n", Serial.getMockOutString());
}

TEST(Print, printStr2)
{
  Serial.print("Hello Ove!");

  STRCMP_CONTAINS("Hello Ove!", Serial.getMockOutString());
}
