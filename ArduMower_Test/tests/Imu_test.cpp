#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Imu.h"
#include "HardwareSerial.h"
#include "Drivers.h"
#include "BuzzerMock.h"

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

TEST(Imu, printInfo)
{
  imu_p->m_acc = Vector<float>(-23.23f, -45.456f, -32.319f);
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
      " a=-23.23 -45.46 -32.32"
      " m= +0.10  +1.00  +0.50"
      " g=  20   30   40"
      " P= +1.23  +2.34  +3.45  +4.56"
      " R= +5.67  +6.78  +7.89  +8.90"
      " T= +1.99  -2.30  +0.60"
      " Y= -0.12  -1.23  -2.34  -3.45  -4.56\r\n",
      Console.getMockOutString());
}

TEST(Imu, playCompletedSound)
{
  mock().disable();
  BuzzerMock* buzzerMock_p = new BuzzerMock();
  mock().enable();

  imu_p->m_buzzer_p = buzzerMock_p;

  mock().expectOneCall("Buzzer::beep")
      .onObject(buzzerMock_p)
      .withParameter("data_p", imu_p->m_completedSound)
      .withParameter("len", 3);

  imu_p->playCompletedSound();

  delete buzzerMock_p;

  mock().checkExpectations();
}

TEST(Imu, saveCalibrationData)
{
  const int16_t ADDR = 600;
  const uint8_t MAGIC = 6;

  mock().expectOneCall("EEPROM::put")
      .withParameter("idx", ADDR + 1);
  mock().expectOneCall("EEPROM::write")
      .withParameter("idx", ADDR)
      .withParameter("val", MAGIC);

  imu_p->saveCalibrationData();

  mock().checkExpectations();
}

TEST(Imu, deleteCalibrationData)
{
  const int16_t ADDR = 600;

  imu_p->m_calibrationData.accelOffset = Vector<float>(123.456f);
  imu_p->m_calibrationData.accelScale = Vector<float>(123.456f);
  imu_p->m_calibrationData.magnetometerOffset = Vector<int16_t>(INT16_MAX);
  imu_p->m_calibrationData.magnetometerScale = Vector<int16_t>(INT16_MAX);
  imu_p->m_calibrationAvailable = true;

  mock().expectOneCall("EEPROM::write")
      .withParameter("idx", ADDR)
      .withParameter("val", 0);

  imu_p->deleteCalibrationData();

  CHECK(Vector<float>() == imu_p->m_calibrationData.accelOffset);
  CHECK(Vector<float>(1.0f) == imu_p->m_calibrationData.accelScale);
  CHECK(Vector<int16_t>() == imu_p->m_calibrationData.magnetometerOffset);
  CHECK(Vector<int16_t>(1) == imu_p->m_calibrationData.magnetometerScale);
  CHECK_FALSE(imu_p->m_calibrationAvailable);
  STRCMP_EQUAL("IMU calibration deleted\r\n", Console.getMockOutString());

  mock().checkExpectations();
}

TEST(Imu, loadCalibrationData_invalidMagic)
{
  const int16_t ADDR = 600;

  mock().expectOneCall("EEPROM::read")
      .withParameter("idx", ADDR)
      .andReturnValue(0);

  imu_p->loadCalibrationData();

  STRCMP_EQUAL("IMU error: No calibration data\r\n", Console.getMockOutString());

  mock().checkExpectations();
}

TEST(Imu, loadCalibrationData_validMagic)
{
  const int16_t ADDR = 600;
  const uint8_t MAGIC = 6;

  imu_p->m_calibrationAvailable = false;

  mock().expectOneCall("EEPROM::read")
      .withParameter("idx", ADDR)
      .andReturnValue(MAGIC);
  mock().expectOneCall("EEPROM::get")
      .withParameter("idx", ADDR + 1);

  imu_p->loadCalibrationData();

  CHECK_TRUE(imu_p->m_calibrationAvailable);
  STRCMP_EQUAL("IMU: Found calibration data\r\n", Console.getMockOutString());

  mock().checkExpectations();
}

TEST(Imu, initAccelerometer_DEVICE_D_OK)
{
  imu_p->m_accMag.m_deviceType = LSM303::DEVICE_D;

  mock().expectOneCall("LSM303::init")
      .withParameter("device", LSM303::DEVICE_AUTO)
      .withParameter("sa0", LSM303::SA0_AUTO)
      .andReturnValue(true);
  mock().expectOneCall("LSM303::readReg")
      .withParameter("reg", LSM303::WHO_AM_I_M)
      .andReturnValue(LSM303::DEV_ID_LSM303DLM);
  mock().expectOneCall("LSM303::enableDefault");
  mock().expectOneCall("LSM303::writeReg")
      .withParameter("reg", LSM303::CTRL2)
      .withParameter("value", 0x18);

  imu_p->initAccelerometer();

  UNSIGNED_LONGS_EQUAL(0, imu_p->m_errorCounter);
  STRCMP_EQUAL(
      "Init Accelerometer/Magnetometer\r\n"
      "deviceType=3 devId=60\r\n",
      Console.getMockOutString());

  mock().checkExpectations();
}

TEST(Imu, initMagnetometer_alreadyInitialized)
{
  imu_p->m_accMag.m_deviceType = LSM303::DEVICE_D;

  mock().expectNoCall("LSM303::init");

  imu_p->initMagnetometer();

  mock().checkExpectations();
}

TEST(Imu, initMagnetometer_notInitialized)
{
  imu_p->m_accMag.m_deviceType = LSM303::DEVICE_AUTO;

  mock().expectOneCall("LSM303::init")
      .withParameter("device", LSM303::DEVICE_AUTO)
      .withParameter("sa0", LSM303::SA0_AUTO)
      .andReturnValue(true);
  mock().ignoreOtherCalls();

  imu_p->initMagnetometer();

  mock().checkExpectations();
}

TEST(Imu, initGyroscope)
{
  mock().expectOneCall("L3G::init")
      .withParameter("device", L3G::DEVICE_AUTO)
      .withParameter("sa0", L3G::SA0_AUTO)
      .andReturnValue(true);
  mock().expectOneCall("L3G::readReg")
      .withParameter("reg", L3G::WHO_AM_I)
      .andReturnValue(L3G::DEV_ID_4200D);
  mock().expectOneCall("L3G::enableDefault");
  mock().expectOneCall("delay")
      .withParameter("ms", 250);

  CHECK_TRUE(imu_p->initGyroscope());

  STRCMP_EQUAL(
      "Init Gyroscope\r\n"
      "deviceType=3 devId=211\r\n",
      Console.getMockOutString());

  mock().checkExpectations();
}

TEST(Imu, calibrateGyro_zeroInput)
{
  const uint8_t numberOfSamples = 32;

  mock().expectNCalls(numberOfSamples, "L3G::read");
  mock().expectNCalls(numberOfSamples, "delay")
      .withParameter("ms", 10);


  imu_p->calibrateGyro();

  CHECK(Vector<int16_t>() == imu_p->m_gyroOffset);
  CHECK(Vector<int16_t>() == imu_p->m_gyroNoise);
  CHECK_TRUE(imu_p->m_useGyroCalibration);
  STRCMP_EQUAL(
      "---CalibGyro---\r\n"
      "gyro calib min=0 max=0 Offset=0 noise=0\r\n"
      "Offset=0,0,0\r\n"
      "------------\r\n",
      Console.getMockOutString());

  mock().checkExpectations();
}

TEST(Imu, calibrateMagnetometerStartStop_start)
{
  imu_p->m_state = Imu::RUN;

  mock().expectOneCall("HardwareSerial::available")
      .onObject(&Console)
      .andReturnValue(1);
  mock().expectOneCall("HardwareSerial::read")
      .onObject(&Console)
      .andReturnValue('x');
  mock().expectOneCall("HardwareSerial::available")
      .onObject(&Console)
      .andReturnValue(0);

  imu_p->calibrateMagnetometerStartStop();

  CHECK_FALSE(imu_p->m_foundNewMinMax);
  CHECK_FALSE(imu_p->m_useMagCalibration);
  ENUMS_EQUAL_INT(Imu::CALIBRATE_MAG, imu_p->m_state);
  CHECK(Vector<int16_t>(INT16_MAX) == imu_p->m_magMin);
  CHECK(Vector<int16_t>(INT16_MIN) == imu_p->m_magMax);
  STRCMP_EQUAL(
      "Magnetometer calibration...\r\n"
      "Rotate sensor 360 degree around all three axis\r\n",
      Console.getMockOutString());

  mock().checkExpectations();
}

TEST(Imu, calibrateMagnetometerStartStop_stop)
{
  const int16_t ADDR = 600;
  const uint8_t MAGIC = 6;

  mock().disable();
  BuzzerMock* buzzerMock_p = new BuzzerMock();
  mock().enable();

  imu_p->m_buzzer_p = buzzerMock_p;
  imu_p->m_state = Imu::CALIBRATE_MAG;

  // calibrateMagnetometerStartStop
  mock().expectOneCall("HardwareSerial::available")
      .onObject(&Console)
      .andReturnValue(1);
  mock().expectOneCall("HardwareSerial::read")
      .onObject(&Console)
      .andReturnValue('x');
  mock().expectOneCall("HardwareSerial::available")
      .onObject(&Console)
      .andReturnValue(0);

  // saveCalibrationData
  mock().expectOneCall("EEPROM::put")
      .withParameter("idx", ADDR + 1);
  mock().expectOneCall("EEPROM::write")
      .withParameter("idx", ADDR)
      .withParameter("val", MAGIC);

  // playCompletedSound
  mock().expectOneCall("Buzzer::beep")
      .onObject(buzzerMock_p)
      .withParameter("data_p", imu_p->m_completedSound)
      .withParameter("len", 3);

  // delay
  mock().expectOneCall("delay")
      .withParameter("ms", 500);

  imu_p->calibrateMagnetometerStartStop();

  CHECK_TRUE(imu_p->m_calibrationAvailable);
  CHECK_TRUE(imu_p->m_useMagCalibration);
  ENUMS_EQUAL_INT(Imu::RUN, imu_p->m_state);
  CHECK(Vector<int16_t>() == imu_p->m_calibrationData.magnetometerScale);
  CHECK(Vector<int16_t>() == imu_p->m_calibrationData.magnetometerOffset);
  STRCMP_EQUAL(
      "Magnetometer calibration completed\r\n"
      "--------\r\n"
      "accOffset=0.00,0.00,0.00\r\n"
      "accScale=1.00,1.00,1.00\r\n"
      "magOffset=0,0,0\r\n"
      "magScale=0,0,0\r\n"
      "--------\r\n",
      Console.getMockOutString());

  delete buzzerMock_p;

  mock().checkExpectations();
}

TEST(Imu, calibrateMagnetometerUpdate)
{
  mock().disable();
  BuzzerMock* buzzerMock_p = new BuzzerMock();
  mock().enable();

  imu_p->m_buzzer_p = buzzerMock_p;

  // delay
  mock().expectOneCall("delay")
      .withParameter("ms", 20);

  // readMagnetometer
  mock().expectOneCall("LSM303::readMag");

  // buzzer
  mock().expectOneCall("Buzzer::beepStop")
      .onObject(buzzerMock_p);

  imu_p->calibrateMagnetometerUpdate();

  delete buzzerMock_p;

  mock().checkExpectations();
}

TEST(Imu, calibrateAccelerometerNextAxis)
{
  const uint8_t numberOfSamples = 32;

  mock().disable();
  BuzzerMock* buzzerMock_p = new BuzzerMock();
  mock().enable();

  imu_p->m_buzzer_p = buzzerMock_p;
  imu_p->m_calibAccAxisCounter = 6;

  // buzzer
  mock().expectOneCall("Buzzer::beep")
      .onObject(buzzerMock_p)
      .withParameter("frequency", 440)
      .withParameter("duration_ms", 0);

  // calibrateAccelerometerNextAxis
  mock().expectOneCall("HardwareSerial::available")
      .onObject(&Console)
      .andReturnValue(1);
  mock().expectOneCall("HardwareSerial::read")
      .onObject(&Console)
      .andReturnValue('x');
  mock().expectOneCall("HardwareSerial::available")
      .onObject(&Console)
      .andReturnValue(0);

  // readAccelerometer
  mock().expectNCalls(numberOfSamples, "LSM303::readAcc");

  // delay
  mock().expectNCalls(numberOfSamples, "delay")
      .withParameter("ms", 10);
  mock().expectOneCall("delay")
      .withParameter("ms", 500);

  CHECK_FALSE(imu_p->calibrateAccelerometerNextAxis());

  STRCMP_EQUAL(
      "Accelerometer calibration start...\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      " x: min=0.00 avg=0.00 max=0.00\r\n"
      " y: min=0.00 avg=0.00 max=0.00\r\n"
      " z: min=0.00 avg=0.00 max=0.00\r\n"
      "side 1 of 6 completed\r\n",
      Console.getMockOutString());

  delete buzzerMock_p;

  mock().checkExpectations();
}

TEST(Imu, calibrateAccelerometerNextAxis_lastAxis)
{
  const uint8_t numberOfSamples = 32;
  const int16_t ADDR = 600;
  const uint8_t MAGIC = 6;

  mock().disable();
  BuzzerMock* buzzerMock_p = new BuzzerMock();
  mock().enable();

  imu_p->m_buzzer_p = buzzerMock_p;
  imu_p->m_calibAccAxisCounter = 5;

  // buzzer
  mock().expectOneCall("Buzzer::beep")
      .onObject(buzzerMock_p)
      .withParameter("frequency", 440)
      .withParameter("duration_ms", 0);

  // calibrateAccelerometerNextAxis
  mock().expectOneCall("HardwareSerial::available")
      .onObject(&Console)
      .andReturnValue(0);

  // readAccelerometer
  mock().expectNCalls(numberOfSamples, "LSM303::readAcc");

  // delay
  mock().expectNCalls(numberOfSamples, "delay")
      .withParameter("ms", 10);

  // saveCalibrationData
  mock().expectOneCall("EEPROM::put")
      .withParameter("idx", ADDR + 1);
  mock().expectOneCall("EEPROM::write")
      .withParameter("idx", ADDR)
      .withParameter("val", MAGIC);

  // playCompletedSound
  mock().expectOneCall("Buzzer::beep")
      .onObject(buzzerMock_p)
      .withParameter("data_p", imu_p->m_completedSound)
      .withParameter("len", 3);

  // delay
  mock().expectOneCall("delay")
      .withParameter("ms", 500);

  CHECK_TRUE(imu_p->calibrateAccelerometerNextAxis());

  STRCMP_EQUAL(
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      "0,0,0\r\n"
      " x: min=0.00 avg=0.00 max=0.00\r\n"
      " y: min=0.00 avg=0.00 max=0.00\r\n"
      " z: min=0.00 avg=0.00 max=0.00\r\n"
      "side 6 of 6 completed\r\n"
      "--------\r\n"
      "accOffset=0.00,0.00,0.00\r\n"
      "accScale=0.00,0.00,0.00\r\n"
      "magOffset=0,0,0\r\n"
      "magScale=1,1,1\r\n"
      "--------\r\n"
      "Accelerometer calibration completed\r\n",
      Console.getMockOutString());

  delete buzzerMock_p;

  mock().checkExpectations();
}

TEST(Imu, init)
{
  const int16_t ADDR = 600;
  const uint8_t MAGIC = 6;

  mock().disable();
  BuzzerMock* buzzerMock_p = new BuzzerMock();
  mock().enable();

  imu_p->m_accMag.m_deviceType = LSM303::DEVICE_D;

  // loadCalibrationData
  mock().expectOneCall("EEPROM::read")
      .withParameter("idx", ADDR)
      .andReturnValue(MAGIC);
  mock().expectOneCall("EEPROM::get")
      .withParameter("idx", ADDR + 1);

  // initGyroscope
  mock().expectOneCall("L3G::init")
      .withParameter("device", L3G::DEVICE_AUTO)
      .withParameter("sa0", L3G::SA0_AUTO)
      .andReturnValue(true);
  mock().expectOneCall("L3G::readReg")
      .withParameter("reg", L3G::WHO_AM_I)
      .andReturnValue(L3G::DEV_ID_4200D);
  mock().expectOneCall("L3G::enableDefault");
  mock().expectOneCall("delay")
      .withParameter("ms", 250);

  // initAccelerometer
  mock().expectOneCall("LSM303::init")
      .withParameter("device", LSM303::DEVICE_AUTO)
      .withParameter("sa0", LSM303::SA0_AUTO)
      .andReturnValue(true);
  mock().expectOneCall("LSM303::readReg")
      .withParameter("reg", LSM303::WHO_AM_I_M)
      .andReturnValue(LSM303::DEV_ID_LSM303DLM);
  mock().expectOneCall("LSM303::enableDefault");
  mock().expectOneCall("LSM303::writeReg")
      .withParameter("reg", LSM303::CTRL2)
      .withParameter("value", 0x18);

  CHECK_TRUE(imu_p->init(buzzerMock_p));

  POINTERS_EQUAL(buzzerMock_p, imu_p->m_buzzer_p);
  CHECK_TRUE(imu_p->m_hardwareInitialized);
  STRCMP_EQUAL(
      "IMU: Found calibration data\r\n"
      "--------\r\n"
      "accOffset=0.00,0.00,0.00\r\n"
      "accScale=1.00,1.00,1.00\r\n"
      "magOffset=0,0,0\r\n"
      "magScale=1,1,1\r\n"
      "--------\r\n"
      "Init Gyroscope\r\n"
      "deviceType=3 devId=211\r\n"
      "Init Accelerometer/Magnetometer\r\n"
      "deviceType=3 devId=60\r\n"
      , Console.getMockOutString());

  delete buzzerMock_p;

  mock().checkExpectations();
}

IGNORE_TEST(Imu, update)
{
  // millis
  mock().expectOneCall("millis")
      .andReturnValue(100);

  imu_p->update();

  mock().checkExpectations();
}
