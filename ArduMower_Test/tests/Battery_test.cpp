#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Battery.h"
#include "AdcManager.h"

AdcManager ADCMan;

// BatteryInit ------------------------------------------------------------

TEST_GROUP(BatteryInit)
{
  Battery* battery_p;

  void setup()
  {
  }

  void teardown()
  {
    if (battery_p)
    {
      delete battery_p;
    }
    mock().clear();
  }
};

TEST(BatteryInit, defaultInit)
{
  battery_p = new Battery();

  UNSIGNED_LONGS_EQUAL(0, battery_p->m_pinVoltage);
  UNSIGNED_LONGS_EQUAL(0, battery_p->m_pinChargeVoltage);
  UNSIGNED_LONGS_EQUAL(0, battery_p->m_pinChargeCurrent);
  UNSIGNED_LONGS_EQUAL(0, battery_p->m_pinChargeRelay);
  UNSIGNED_LONGS_EQUAL(0, battery_p->m_pinBatterySwitch);
}

TEST(BatteryInit, defaultInit_setup)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 2)
      .withParameter("mode", INPUT);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 3)
      .withParameter("mode", INPUT);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 4)
      .withParameter("mode", OUTPUT);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 5)
      .withParameter("mode", OUTPUT);

  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 4)
      .withParameter("val", 0);
  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 5)
      .withParameter("val", 0);

  mock().expectOneCall("AdcManager::setCapture")
      .withParameter("pin", 3)
      .withParameter("samplecount", 1)
      .withParameter("autoCalibrate", true);
  mock().expectOneCall("AdcManager::setCapture")
      .withParameter("pin", 1)
      .withParameter("samplecount", 1)
      .withParameter("autoCalibrate", false);
  mock().expectOneCall("AdcManager::setCapture")
      .withParameter("pin", 2)
      .withParameter("samplecount", 1)
      .withParameter("autoCalibrate", false);

  battery_p = new Battery();
  battery_p->setup(1, 2, 3, 4, 5);

  UNSIGNED_LONGS_EQUAL(1, battery_p->m_pinVoltage);
  UNSIGNED_LONGS_EQUAL(2, battery_p->m_pinChargeVoltage);
  UNSIGNED_LONGS_EQUAL(3, battery_p->m_pinChargeCurrent);
  UNSIGNED_LONGS_EQUAL(4, battery_p->m_pinChargeRelay);
  UNSIGNED_LONGS_EQUAL(5, battery_p->m_pinBatterySwitch);

  mock().checkExpectations();
}

TEST(BatteryInit, parameterizedInit)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 2)
      .withParameter("mode", INPUT);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 3)
      .withParameter("mode", INPUT);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 4)
      .withParameter("mode", OUTPUT);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 5)
      .withParameter("mode", OUTPUT);

  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 4)
      .withParameter("val", 0);
  mock().expectOneCall("digitalWrite")
      .withParameter("pin", 5)
      .withParameter("val", 0);

  mock().expectOneCall("AdcManager::setCapture")
      .withParameter("pin", 3)
      .withParameter("samplecount", 1)
      .withParameter("autoCalibrate", true);
  mock().expectOneCall("AdcManager::setCapture")
      .withParameter("pin", 1)
      .withParameter("samplecount", 1)
      .withParameter("autoCalibrate", false);
  mock().expectOneCall("AdcManager::setCapture")
      .withParameter("pin", 2)
      .withParameter("samplecount", 1)
      .withParameter("autoCalibrate", false);

  battery_p = new Battery(1, 2, 3, 4, 5);

  UNSIGNED_LONGS_EQUAL(1, battery_p->m_pinVoltage);
  UNSIGNED_LONGS_EQUAL(2, battery_p->m_pinChargeVoltage);
  UNSIGNED_LONGS_EQUAL(3, battery_p->m_pinChargeCurrent);
  UNSIGNED_LONGS_EQUAL(4, battery_p->m_pinChargeRelay);
  UNSIGNED_LONGS_EQUAL(5, battery_p->m_pinBatterySwitch);

  mock().checkExpectations();
}

// Battery ------------------------------------------------------------

TEST_GROUP(Battery)
{
  Battery* battery_p;

  void setup()
  {
    mock().disable();
    battery_p = new Battery(1, 2, 3, 4, 5);
    mock().enable();
  }

  void teardown()
  {
    if (battery_p)
    {
      delete battery_p;
    }
    mock().clear();
  }
};

TEST(Battery, read1)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 1)
      .andReturnValue(100);
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 2)
      .andReturnValue(200);
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 3)
      .andReturnValue(300);

  battery_p->read();

  DOUBLES_EQUAL(0.0f, battery_p->m_batCapacity, 0.1f);
  DOUBLES_EQUAL(49.5f, battery_p->m_voltage, 0.1f);
  DOUBLES_EQUAL(99.0f, battery_p->m_chgVoltage, 0.1f);
  DOUBLES_EQUAL(247.6519f, battery_p->m_chgSense, 0.1f);
  DOUBLES_EQUAL(0.0f, battery_p->m_chgCurrent, 0.1f);

  mock().checkExpectations();
}

TEST(Battery, read2)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 1)
      .andReturnValue(600);
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 2)
      .andReturnValue(700);
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 3)
      .andReturnValue(800);

  battery_p->read();

  DOUBLES_EQUAL(0.0f, battery_p->m_batCapacity, 0.1f);
  DOUBLES_EQUAL(297.0f, battery_p->m_voltage, 0.1f);
  DOUBLES_EQUAL(346.5f, battery_p->m_chgVoltage, 0.1f);
  DOUBLES_EQUAL(247.6519f, battery_p->m_chgSense, 0.1f);
  DOUBLES_EQUAL(7.523108f, battery_p->m_chgCurrent, 0.1f);

  mock().checkExpectations();
}
