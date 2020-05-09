#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Imu.h"
#include "HardwareSerial.h"
#include "Drivers.h"

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

    Console.setMockPrintFunctions(false);
    Console.setMockPrintToStdout(false);
    Console.startCapture();
  }

  void teardown()
  {
    if (imu_p)
    {
      delete imu_p;
    }
    mock().clear();

    Console.stopCapture();
    Console.setMockPrintFunctions(true);
    Console.setMockPrintToStdout(false);
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

TEST(Imu, printPointln_int16_t)
{
  Vector<int16_t> v(123, -456, -32000);
  imu_p->printPointln(v);

  STRCMP_EQUAL("123,-456,-32000\r\n", Console.getMockOutString());
}

TEST(Imu, printPointln_float)
{
  Vector<float> v(123.123f, -456.456f, -32000.32000f);

  imu_p->printPointln(v);

  STRCMP_EQUAL("123.12,-456.46,-32000.32\r\n", Console.getMockOutString());
}

//TEST(Imu, printPointMinAvgMax_int16_t)
//{
//  Vector<int16_t> v_min(-23, -456, -32000);
//  Vector<int16_t> v_avg(43, -4, 20000);
//  Vector<int16_t> v_max(123, 456, 24242);
//
//  imu_p->printPointMinAvgMax(v_min, v_avg, v_max);
//
//  STRCMP_EQUAL("123,-456,-32000\r\n", Console.getMockOutString());
//}

TEST(Imu, printPointMinAvgMax_float)
{
  Vector<float> v_min(-23.23f, -456.456f, -32000.319f);
  Vector<float> v_avg(43.1f, -4.9f, 20000.0f);
  Vector<float> v_max(123.123f, 456.456f, 24242.999f);

  imu_p->printPointMinAvgMax(v_min, v_avg, v_max);

  STRCMP_EQUAL(
      " x: min=-23.23 avg=43.10 max=123.12\r\n"
      " y: min=-456.46 avg=-4.90 max=456.46\r\n"
      " z: min=-32000.32 avg=20000.00 max=24243.00\r\n",
      Console.getMockOutString());
}

TEST(Imu, printPointMinMax_int16_t)
{
  Vector<int16_t> v_min(-23, -456, -32000);
  Vector<int16_t> v_max(123, 456, 24242);

  imu_p->printPointMinMax(v_min, v_max);

  STRCMP_EQUAL("x:-23,123 y:-456,456 z:-32000,24242\r\n",
      Console.getMockOutString());
}

//TEST(Imu, printPointMinMax_float)
//{
//    Vector<float> v_min(-23.23f, -456.456f, -32000.319f);
//    Vector<float> v_max(123.123f, 456.456f, 24242.999f);
//
//  imu_p->printPointMinMax(v_min, v_max);
//
//  STRCMP_EQUAL("x:-23,123 y:-456,456 z:-32000,24242\r\n",
//      Console.getMockOutString());
//}

TEST(Imu, printCalibrationData)
{
  imu_p->m_calibrationData.accelOffset =
      Vector<float>(-23.23f, -456.456f, -32000.319f);
  imu_p->m_calibrationData.accelScale =
      Vector<float>(0.1f, 1.0f, 0.5f);
  imu_p->m_calibrationData.magnetometerOffset =
      Vector<int16_t>(20, 30, 40);
  imu_p->m_calibrationData.magnetometerScale =
      Vector<int16_t>(10, 5, 4);

  imu_p->printCalibrationData();

  STRCMP_EQUAL(
      "--------\r\n"
      "accOffset=-23.23,-456.46,-32000.32\r\n"
      "accScale=0.10,1.00,0.50\r\n"
      "magOffset=20,30,40\r\n"
      "magScale=10,5,4\r\n"
      "--------\r\n",
      Console.getMockOutString());
}

IGNORE_TEST(Imu, printInfo)
{
  imu_p->m_acc = Vector<float>(-23.23f, -456.456f, -32000.319f);
  imu_p->m_mag = Vector<float>(0.1f, 1.0f, 0.5f);
  imu_p->m_gyro.m_g = Vector<int16_t>(20, 30, 40);
  imu_p->m_accPitch = 1.23f;
  imu_p->m_scaledPitch = 2.34f;
  imu_p->m_filtPitch = 3.45f;
  imu_p->m_ypr.pitch = 4.56f;
  imu_p->m_accRoll = 5.67f;
  imu_p->m_scaledRoll = 6.78f;
  imu_p->m_filtRoll = 7.89f;
  imu_p->m_ypr.roll = 8.90f;
  imu_p->m_magTilt = Vector<float>(1.99f, -2.3f, 0.6f);

  imu_p->m_yaw = -0.12f;
  imu_p->m_scaledYaw = -1.23f;
  imu_p->m_scaled2Yaw = -2.34f;
  imu_p->m_filtYaw = -3.45f;
  imu_p->m_ypr.yaw = -4.56f;

  imu_p->printInfo(Console);

  STRCMP_EQUAL("imu"
      " a=-23.23 -456.46 -32000.32"
      " m=+0.10 +1.00 +0.50"
      " g=  20   30   40"
      " P=+1.23 +2.34 +3.45 +4.56"
      " R=+5.67 +6.78 +7.89 +8.90"
      " T=+1.99 -2.30 +0.60"
      " Y=-0.12 -1.23 -2.34 -3.45 -4.56\r\n",
      Console.getMockOutString());
}
