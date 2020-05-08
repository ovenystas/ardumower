#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Imu.h"

// ImuInit ------------------------------------------------------------

TEST_GROUP(ImuInit)
{
  Imu* imu_p;

  void setup()
  {
  }

  void teardown()
  {
    if (imu_p)
    {
      delete imu_p;
    }
    mock().clear();
  }
};

TEST(ImuInit, init)
{
  mock().expectOneCall("LSM303::LSM303");

  imu_p = new Imu();

  mock().checkExpectations();
}

// Imu ------------------------------------------------------------

TEST_GROUP(Imu)
{
  Imu* imu_p;

  void setup()
  {
    mock().disable();
    imu_p = new Imu();
    mock().enable();
  }

  void teardown()
  {
    if (imu_p)
    {
      delete imu_p;
    }
    mock().clear();
  }
};

TEST(Imu, getAndClearCallCounter)
{
  imu_p->m_callCounter = 12345;

  UNSIGNED_LONGS_EQUAL(12345, imu_p->getAndClearCallCounter());
  UNSIGNED_LONGS_EQUAL(0, imu_p->m_callCounter);

  mock().checkExpectations();
}

TEST(Imu, getAndClearErrorCounter)
{
  imu_p->m_errorCounter = 12345;

  UNSIGNED_LONGS_EQUAL(12345, imu_p->getAndClearErrorCounter());
  UNSIGNED_LONGS_EQUAL(0, imu_p->m_errorCounter);

  mock().checkExpectations();
}

TEST(Imu, getYaw)
{
  imu_p->m_ypr.yaw = PI;

  DOUBLES_EQUAL(PI, imu_p->getYaw(), 0.00001);
  DOUBLES_EQUAL(180.0f, imu_p->getYawDeg(), 0.00001);
  LONGS_EQUAL(180, imu_p->getYawDegInt());

  mock().checkExpectations();
}

TEST(Imu, getPitch)
{
  imu_p->m_ypr.pitch = -PI;

  DOUBLES_EQUAL(-PI, imu_p->getPitch(), 0.00001);
  DOUBLES_EQUAL(-180.0f, imu_p->getPitchDeg(), 0.00001);
  LONGS_EQUAL(-180, imu_p->getPitchDegInt());

  mock().checkExpectations();
}

TEST(Imu, getRoll)
{
  imu_p->m_ypr.roll = PI / 2;

  DOUBLES_EQUAL(PI / 2, imu_p->getRoll(), 0.00001);
  DOUBLES_EQUAL(90.0f, imu_p->getRollDeg(), 0.00001);
  LONGS_EQUAL(90, imu_p->getRollDegInt());

  mock().checkExpectations();
}

TEST(Imu, readGyroscope_useCalibrationIsFalse)
{
  mock().expectOneCall("L3G::read");

  imu_p->m_useGyroCalibration = false;
  imu_p->m_gyroOffset = Vector<int16_t>(11, 22, 33);
  imu_p->m_gyro.m_g = Vector<int16_t>(1, 2, 3);

  imu_p->readGyroscope();

  LONGS_EQUAL(1, imu_p->m_gyro.m_g.x);
  LONGS_EQUAL(2, imu_p->m_gyro.m_g.y);
  LONGS_EQUAL(3, imu_p->m_gyro.m_g.z);

  mock().checkExpectations();
}

TEST(Imu, readGyroscope_useCalibrationIsTrue)
{
  mock().expectOneCall("L3G::read");

  imu_p->m_useGyroCalibration = true;
  imu_p->m_gyroOffset = Vector<int16_t>(11, 22, 33);
  imu_p->m_gyro.m_g = Vector<int16_t>(1, 2, 3);

  imu_p->readGyroscope();

  LONGS_EQUAL(-10, imu_p->m_gyro.m_g.x);
  LONGS_EQUAL(-20, imu_p->m_gyro.m_g.y);
  LONGS_EQUAL(-30, imu_p->m_gyro.m_g.z);

  mock().checkExpectations();
}

TEST(Imu, readMagnetometer_useCalibrationIsFalse)
{
  mock().expectOneCall("LSM303::readMag");

  imu_p->m_useMagCalibration = false;
  imu_p->m_calibrationData.magnetometerOffset = Vector<int16_t>(11, 22, 33);
  imu_p->m_calibrationData.magnetometerScale = Vector<int16_t>(2, 3, 8);
  imu_p->m_accMag.m_mag = Vector<int16_t>(1, 2, 3);
  imu_p->m_mag = Vector<float>();

  imu_p->readMagnetometer();

  DOUBLES_EQUAL(1.0f, imu_p->m_mag.x, 0.00001);
  DOUBLES_EQUAL(2.0f, imu_p->m_mag.y, 0.00001);
  DOUBLES_EQUAL(3.0f, imu_p->m_mag.z, 0.00001);

  mock().checkExpectations();
}

TEST(Imu, readMagnetometer_useCalibrationIsTrue)
{
  mock().expectOneCall("LSM303::readMag");

  imu_p->m_useMagCalibration = true;
  imu_p->m_calibrationData.magnetometerOffset = Vector<int16_t>(11, 22, 33);
  imu_p->m_calibrationData.magnetometerScale = Vector<int16_t>(2, 3, 8);
  imu_p->m_accMag.m_mag = Vector<int16_t>(1, 2, 3);
  imu_p->m_mag = Vector<float>();

  imu_p->readMagnetometer();

  DOUBLES_EQUAL(-5.0f, imu_p->m_mag.x, 0.00001);
  DOUBLES_EQUAL(-6.666667f, imu_p->m_mag.y, 0.00001);
  DOUBLES_EQUAL(-3.75f, imu_p->m_mag.z, 0.00001);

  mock().checkExpectations();
}

TEST(Imu, readAccelerometer_useCalibrationIsFalse)
{
  mock().expectOneCall("LSM303::readAcc");

  imu_p->m_useAccCalibration = false;
  imu_p->m_calibrationData.accelOffset = Vector<float>(11.0f, 22.0f, 33.0f);
  imu_p->m_calibrationData.accelScale = Vector<int16_t>(2.0f, 3.0f, 8.0f);
  imu_p->m_accMag.m_acc = Vector<int16_t>(1, 2, 3);
  imu_p->m_acc = Vector<float>();

  imu_p->readAccelerometer();

  DOUBLES_EQUAL(1.0f, imu_p->m_acc.x, 0.00001);
  DOUBLES_EQUAL(2.0f, imu_p->m_acc.y, 0.00001);
  DOUBLES_EQUAL(3.0f, imu_p->m_acc.z, 0.00001);

  mock().checkExpectations();
}

TEST(Imu, readAccelerometer_useCalibrationIsTrue)
{
  mock().expectOneCall("LSM303::readAcc");

  imu_p->m_useAccCalibration = true;
  imu_p->m_calibrationData.accelOffset = Vector<float>(11.0f, 22.0f, 33.0f);
  imu_p->m_calibrationData.accelScale = Vector<int16_t>(2.0f, 3.0f, 8.0f);
  imu_p->m_accMag.m_acc = Vector<int16_t>(1, 2, 3);
  imu_p->m_acc = Vector<float>();

  imu_p->readAccelerometer();

  DOUBLES_EQUAL(-5.0f, imu_p->m_acc.x, 0.00001);
  DOUBLES_EQUAL(-6.666667f, imu_p->m_acc.y, 0.00001);
  DOUBLES_EQUAL(-3.75f, imu_p->m_acc.z, 0.00001);

  mock().checkExpectations();
}

TEST(Imu, read_hardwareNotInitialized)
{
  mock().expectNoCall("L3G::read");
  mock().expectNoCall("LSM303::readAcc");

  imu_p->m_hardwareInitialized = false;

  imu_p->read();

  LONGS_EQUAL(1, imu_p->getAndClearErrorCounter());
  LONGS_EQUAL(0, imu_p->getAndClearCallCounter());

  mock().checkExpectations();
}

TEST(Imu, read_hardwareIsInitialized)
{
  mock().expectOneCall("L3G::read");
  mock().expectOneCall("LSM303::readAcc");
  mock().expectOneCall("LSM303::readMag");

  imu_p->m_hardwareInitialized = true;

  imu_p->read();

  LONGS_EQUAL(0, imu_p->getAndClearErrorCounter());
  LONGS_EQUAL(1, imu_p->getAndClearCallCounter());

  mock().checkExpectations();
}

TEST(Imu, scalePIangles)
{
  DOUBLES_EQUAL( PI, imu_p->scalePIangles( PI,  PI), 0.00001);
  DOUBLES_EQUAL(-PI, imu_p->scalePIangles( PI, -PI), 0.00001);
  DOUBLES_EQUAL( PI, imu_p->scalePIangles(-PI,  PI), 0.00001);
  DOUBLES_EQUAL(-PI, imu_p->scalePIangles(-PI, -PI), 0.00001);

  DOUBLES_EQUAL( PI/4, imu_p->scalePIangles( PI/4,  PI/4), 0.00001);
  DOUBLES_EQUAL( PI/4, imu_p->scalePIangles( PI/4, -PI/4), 0.00001);
  DOUBLES_EQUAL(-PI/4, imu_p->scalePIangles(-PI/4,  PI/4), 0.00001);
  DOUBLES_EQUAL(-PI/4, imu_p->scalePIangles(-PI/4, -PI/4), 0.00001);

  mock().checkExpectations();
}

TEST(Imu, complementary2)
{
  DOUBLES_EQUAL(1.806415776f, imu_p->complementary2(PI/4, PI/8, 100, PI/16), 0.00001);

  mock().checkExpectations();
}

TEST(Imu, kalman)
{
  DOUBLES_EQUAL(0.2855994f, imu_p->kalman(PI/4, PI/8, 100, PI/16), 0.00001);

  mock().checkExpectations();
}

