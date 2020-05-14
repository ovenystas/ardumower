/*
 * Odometer_test.cpp
 *
 *  Created on: May 14, 2020
 *      Author: ove
 */
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Odometer.h"
#include "EncoderMock.h"
#include "ImuMock.h"

// OdometerInit ------------------------------------------------------------

TEST_GROUP(OdometerInit)
{
  Odometer* odometer_p;

  void setup()
  {
  }

  void teardown()
  {
    if (odometer_p)
    {
      delete odometer_p;
    }
    mock().clear();
  }
};

TEST(OdometerInit, init)
{
  mock().disable();
  EncoderMock enc_l(1, false);
  EncoderMock enc_r(2, true);
  ImuMock imu;
  mock().enable();

  odometer_p = new Odometer(
      static_cast<Encoder&>(enc_l),
      static_cast<Encoder&>(enc_r),
      static_cast<Imu&>(imu));

  mock().checkExpectations();
}

// Odometer ------------------------------------------------------------

TEST_GROUP(Odometer)
{
  Odometer* odometer_p;
  EncoderMock* enc_l_p;
  EncoderMock* enc_r_p;
  ImuMock* imu_p;

  void setup()
  {
    mock().disable();
    enc_l_p = new EncoderMock(1, false);
    enc_r_p = new EncoderMock(2, false);
    imu_p = new ImuMock();

    odometer_p = new Odometer(
        static_cast<Encoder&>(*enc_l_p),
        static_cast<Encoder&>(*enc_r_p),
        static_cast<Imu&>(*imu_p));
    mock().enable();
  }

  void teardown()
  {
    if (odometer_p) delete odometer_p;
    if (enc_l_p) delete enc_l_p;
    if (enc_r_p) delete enc_r_p;
    if (imu_p) delete imu_p;
    mock().clear();
  }
};

TEST(Odometer, isUsed)
{
  CHECK_FALSE(odometer_p->isUsed());
  odometer_p->m_use = true;
  CHECK_TRUE(odometer_p->isUsed());

  mock().checkExpectations();
}

TEST(Odometer, getX)
{
  DOUBLES_EQUAL(0.0f, odometer_p->getX(), 0.0001f);
  odometer_p->m_x = 123.456f;
  DOUBLES_EQUAL(123.456f, odometer_p->getX(), 0.0001f);

  mock().checkExpectations();
}

TEST(Odometer, getY)
{
  DOUBLES_EQUAL(0.0f, odometer_p->getY(), 0.0001f);
  odometer_p->m_y = 123.456f;
  DOUBLES_EQUAL(123.456f, odometer_p->getY(), 0.0001f);

  mock().checkExpectations();
}

TEST(Odometer, getSettings)
{
  OdometerSettings* settings_p = odometer_p->getSettings();

  CHECK_FALSE(settings_p->use.value);
  LONGS_EQUAL(ODOMETER_TICKS_PER_REVOLUTION, settings_p->ticksPerRevolution.value);
  DOUBLES_EQUAL(ODOMETER_TICKS_PER_CM, settings_p->ticksPerCm.value, 0.0001);
  DOUBLES_EQUAL(ODOMETER_WHEELBASE_CM, settings_p->wheelBaseCm.value, 0.0001);

  mock().checkExpectations();
}

IGNORE_TEST(Odometer, setSettings)
{
//  OdometerSettings settings;
//
//  settings.use.value = true;
//  settings.ticksPerRevolution.value = 12345;
//  settings.ticksPerCm.value = 2.4f;
//  settings.wheelBaseCm.value = 14.1f;
//
//  odometer_p->setSettings(&settings);
//
//  CHECK_TRUE(odometer_p->m_use);
//  LONGS_EQUAL(12345, odometer_p->m_ticksPerRevolution);
//  DOUBLES_EQUAL(2.4f, odometer_p->m_ticksPerCm, 0.0001);
//  DOUBLES_EQUAL(14.1f, odometer_p->m_wheelBaseCm, 0.0001);
//
//  mock().checkExpectations();
}

TEST(Odometer, read)
{
  mock().expectOneCall("Encoder::read")
      .onObject(enc_l_p);
  mock().expectOneCall("Encoder::read")
        .onObject(enc_r_p);

  odometer_p->read();

  mock().checkExpectations();
}

TEST(Odometer, calc_imuNotUsed)
{
  mock().expectOneCall("millis")
      .andReturnValue(100);
  mock().expectOneCall("Encoder::getCounter")
        .onObject(enc_l_p)
        .andReturnValue(-1000);
  mock().expectOneCall("Encoder::getCounter")
        .onObject(enc_r_p)
        .andReturnValue(2000);
  mock().expectOneCall("Encoder::setWheelRpmCurr")
        .onObject(enc_l_p)
        .withParameter("wheelRpmCurr", -566);
  mock().expectOneCall("Encoder::setWheelRpmCurr")
        .onObject(enc_r_p)
        .withParameter("wheelRpmCurr", 1132);
  mock().expectOneCall("Imu::isUsed")
        .onObject(imu_p)
        .andReturnValue(false);

  odometer_p->calc();

  DOUBLES_EQUAL(0.0f, odometer_p->getX(), 0.0001);
  DOUBLES_EQUAL(37.06449f, odometer_p->getY(), 0.0001);

  mock().checkExpectations();
}

TEST(Odometer, calc_imuUsed)
{
  mock().expectOneCall("millis")
      .andReturnValue(100);
  mock().expectOneCall("Encoder::getCounter")
        .onObject(enc_l_p)
        .andReturnValue(1000);
  mock().expectOneCall("Encoder::getCounter")
        .onObject(enc_r_p)
        .andReturnValue(-2000);
  mock().expectOneCall("Encoder::setWheelRpmCurr")
        .onObject(enc_l_p)
        .withParameter("wheelRpmCurr", 566);
  mock().expectOneCall("Encoder::setWheelRpmCurr")
        .onObject(enc_r_p)
        .withParameter("wheelRpmCurr", -1132);
  mock().expectOneCall("Imu::isUsed")
        .onObject(imu_p)
        .andReturnValue(true);
  mock().expectOneCall("Imu::getYaw")
        .onObject(imu_p)
        .andReturnValue(PI);

  imu_p->m_use = true;

  odometer_p->calc();

  DOUBLES_EQUAL(0.0f, odometer_p->getX(), 0.0001);
  DOUBLES_EQUAL(37.06449f, odometer_p->getY(), 0.0001);

  mock().checkExpectations();
}

