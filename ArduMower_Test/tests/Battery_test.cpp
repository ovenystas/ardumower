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

  UNSIGNED_LONGS_EQUAL(1, battery_p->m_pinBatVoltage);
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

TEST(Battery, readBatVoltage_bigChange)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 1)
      .andReturnValue(1000);

  battery_p->readBatVoltage();

  LONGS_EQUAL(30000, battery_p->getBatVoltage_mV());
  DOUBLES_EQUAL(30.0f, battery_p->getBatVoltage_V(), 0.1f);

  mock().checkExpectations();
}

TEST(Battery, readBatVoltage_smallChange)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 1)
      .andReturnValue(100);

  battery_p->readBatVoltage();

  // 30 becomes 24 because of limited filter resolution
  LONGS_EQUAL(24, battery_p->getBatVoltage_mV());
  DOUBLES_EQUAL(0.024f, battery_p->getBatVoltage_V(), 0.001f);

  mock().checkExpectations();
}

TEST(Battery, readChargeVoltage_bigChange)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 2)
      .andReturnValue(1000);

  battery_p->readChargeVoltage();

  LONGS_EQUAL(30000, battery_p->getChargeVoltage_mV());
  DOUBLES_EQUAL(30.0f, battery_p->getChargeVoltage_V(), 0.001f);

  mock().checkExpectations();
}

TEST(Battery, readChargeVoltage_smallChange)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 2)
      .andReturnValue(100);

  battery_p->readChargeVoltage();

  // 30 becomes 24 because of limited filter resolution
  LONGS_EQUAL(24, battery_p->getChargeVoltage_mV());
  DOUBLES_EQUAL(0.024f, battery_p->getChargeVoltage_V(), 0.001f);

  mock().checkExpectations();
}

TEST(Battery, readChargeCurrent_0mA)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 3)
      .andReturnValue(511); // floor(2.5 / (5 / 1023))

  battery_p->readChargeCurrent();

  LONGS_EQUAL(0, battery_p->getChargeCurrent_mA());
  DOUBLES_EQUAL(0.0f, battery_p->getChargeCurrent_A(), 0.001f);

  mock().checkExpectations();
}

TEST(Battery, readChargeCurrent_plus5000mA)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 3)
      .andReturnValue(700); // floor(2.5 / (5 / 1023)) + round((0.185 * 5) / (5 / 1023))

  battery_p->readChargeCurrent();

  // 26.5 mA per ADC LSB => +/-14 mA tolerance
  CHECK(battery_p->getChargeCurrent_mA() < 5000 + 14);
  CHECK(battery_p->getChargeCurrent_mA() > 5000 - 14);
  DOUBLES_EQUAL(5.000f, battery_p->getChargeCurrent_A(), 0.014f);

  mock().checkExpectations();
}

TEST(Battery, readChargeCurrent_plus1AdcStep)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 3)
      .andReturnValue(512); // floor(2.5 / (5 / 1023)) + 1)

  battery_p->readChargeCurrent();

  // 26.5 mA per ADC LSB
  CHECK(battery_p->getChargeCurrent_mA() <= 27);
  CHECK(battery_p->getChargeCurrent_mA() >= 26);
  DOUBLES_EQUAL(0.0265f, battery_p->getChargeCurrent_A(), 0.0005f);

  mock().checkExpectations();
}

TEST(Battery, readChargeCurrent_minus5000mA)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 3)
      .andReturnValue(322); // floor(2.5 / (5 / 1023)) - round((0.185 * 5) / (5 / 1023))

  battery_p->readChargeCurrent();

  // Negative currents get limited to 0 mA
  LONGS_EQUAL(0, battery_p->getChargeCurrent_mA());
  DOUBLES_EQUAL(0.000f, battery_p->getChargeCurrent_A(), 0.001f);

  mock().checkExpectations();
}

TEST(Battery, readChargeCurrent_minus1AdcStep)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 3)
      .andReturnValue(510); // floor(2.5 / (5 / 1023)) - 1)

  battery_p->readChargeCurrent();

  // Negative currents get limited to 0 mA
  LONGS_EQUAL(0, battery_p->getChargeCurrent_mA());
  DOUBLES_EQUAL(0.000f, battery_p->getChargeCurrent_A(), 0.001f);

  mock().checkExpectations();
}

TEST(Battery, updateCapacity_tooLowChargeCurrent_okChargeVoltage)
{
  battery_p->m_chargeCurrent_mA = 40;
  battery_p->m_chargeVoltage_mV = 5001;

  battery_p->updateCapacity();

  LONGS_EQUAL(0, battery_p->m_batCapacity_mAs);
}

TEST(Battery, updateCapacity_okChargeCurrent_tooLowChargeVoltage)
{
  battery_p->m_chargeCurrent_mA = 41;
  battery_p->m_chargeVoltage_mV = 5000;

  battery_p->updateCapacity();

  LONGS_EQUAL(0, battery_p->m_batCapacity_mAs);
}

TEST(Battery, updateCapacity_justEnoughChargeCurrent_okChargeVoltage)
{
  battery_p->m_chargeCurrent_mA = 41;
  battery_p->m_chargeVoltage_mV = 5001;

  battery_p->updateCapacity();

  LONGS_EQUAL(4, battery_p->m_batCapacity_mAs);
}

TEST(Battery, updateCapacity_5000mAChargeCurrent_okChargeVoltage)
{
  battery_p->m_chargeCurrent_mA = 5000;
  battery_p->m_chargeVoltage_mV = 5001;

  battery_p->updateCapacity();

  LONGS_EQUAL(500, battery_p->m_batCapacity_mAs);
}

TEST(Battery, read)
{
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 1)
      .andReturnValue(100);
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 2)
      .andReturnValue(200);
  mock().expectOneCall("AdcManager::read")
      .withParameter("pin", 3)
      .andReturnValue(572); // 1.6 A => floor(2.5 / (5 / 1023)) + round((0.185 * 1.6) / (5 / 1023))

  battery_p->read();

  CHECK(battery_p->m_batCapacity_mAs <= 160 + 1);
  CHECK(battery_p->m_batCapacity_mAs >= 160 - 1);
  DOUBLES_EQUAL(0.0f, battery_p->getCapacity_Ah(), 0.1f);
  DOUBLES_EQUAL(0.024f, battery_p->getBatVoltage_V(), 0.0005f);
  DOUBLES_EQUAL(6.0f, battery_p->getChargeVoltage_V(), 0.0005f);
  DOUBLES_EQUAL(1.6f, battery_p->getChargeCurrent_A(), 0.014f);

  mock().checkExpectations();
}
