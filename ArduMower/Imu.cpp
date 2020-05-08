/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat

 Private-use only! (you need to ask for a magmercial-use)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 Private-use only! (you need to ask for a magmercial-use)
 */

#include <Wire.h>
#include "Imu.h"
#include "L3G.h"
#include "Buzzer.h"
#include "Drivers.h"

#define ADDR 600
#define MAGIC 6

#define roundDivision(a, b) \
  ((a) >= 0 ? ((a) + (b) / 2) / (b) : ((a) - (b) / 2) / (b))

void Imu::loadCalibrationData(void)
{
  uint8_t magic = EEPROM.read(ADDR);
  if (magic != MAGIC)
  {
    Console.println(F("IMU error: no calib data"));
    return;
  }
  m_calibrationAvailable = true;
  Console.println(F("IMU: Found calibration data"));
  EEPROM.get(ADDR + 1, m_calibrationData);
}

void Imu::saveCalibrationData(void)
{
  EEPROM.put(ADDR + 1, m_calibrationData);
}

void Imu::deleteCalibrationData(void)
{
  EEPROM.write(ADDR, 0); // clear magic
  m_calibrationData.accelOffset = Vector<float>();
  m_calibrationData.accelScale = Vector<float>(1.0f);
  m_calibrationData.magnetometerOffset = Vector<int16_t>();
  m_calibrationData.magnetometerScale = Vector<int16_t>(1);
  m_calibrationAvailable = false;
  Console.println(F("IMU calibration deleted"));
}

template <class T>
void Imu::printPointln(const Vector<T>& point)
{
  Console.print(point.x);
  Console.print(',');
  Console.print(point.y);
  Console.print(',');
  Console.println(point.z);
}

template <class T>
void Imu::printPointMinAvgMax(
    Vector<T>& v_min, Vector<T>& v_avg, Vector<T>& v_max)
{
  const char axisChars[] = { 'x', 'y', 'z' };

  uint8_t i = 0;
  for (const char axis : axisChars)
  {
    Console.print(' ');
    Console.print(axis);
    Console.print(F(": min="));
    Console.print(v_min.data[i]);
    Console.print(F(" avg="));
    Console.print(v_avg.data[i]);
    Console.print(F(" max="));
    Console.println(v_max.data[i]);
    ++i;
  }
}

template <class T>
void Imu::printPointMinMax(Vector<T>& v_min, Vector<T>& v_max)
{
  const char axisChars[] = { 'x', 'y', 'z' };

  uint8_t i = 0;
  for (const char axis : axisChars)
  {
    if (i > 0)
    {
      Console.print(' ');
    }
    Console.print(axis);
    Console.print(':');
    Console.print(v_min.data[i]);
    Console.print(',');
    Console.print(v_max.data[i]);
    ++i;
  }
  Console.println();
}

void Imu::printCalibrationData(void)
{
  Console.println(F("--------"));
  Console.print(F("accOffset="));
  printPointln(m_calibrationData.accelOffset);
  Console.print(F("accScale="));
  printPointln(m_calibrationData.accelScale);
  Console.print(F("magOffset="));
  printPointln(m_calibrationData.magnetometerOffset);
  Console.print(F("magScale="));
  printPointln(m_calibrationData.magnetometerScale);
  Console.println(F("--------"));
}

// Calculate gyro offsets
void Imu::calibrateGyro(void)
{
  const uint8_t numberOfSamples = 32;
  m_useGyroCalibration = false;
  m_gyroOffset = Vector<int16_t>();
  Vector<int16_t> offset;

  Console.println(F("---CalibGyro---"));

  bool calibrationDone = false;
  while (!calibrationDone)
  {
    int16_t zmin = INT16_MAX;
    int16_t zmax = INT16_MIN;
    int16_t noise = 0;

    for (uint8_t i = 0; i < numberOfSamples; i++)
    {
      readGyroscope();

      zmin = min(zmin, m_gyro.m_g.z);
      zmax = max(zmax, m_gyro.m_g.z);

      offset += m_gyro.m_g;

      // noise is computed with last offset calculation
      noise = static_cast<int16_t>(noise + sq(m_gyro.m_g.z - m_gyroOffset.z));

      delay(10);
    }

    offset.x = static_cast<int16_t>(roundDivision(offset.x, numberOfSamples));
    offset.y = static_cast<int16_t>(roundDivision(offset.y, numberOfSamples));
    offset.z = static_cast<int16_t>(roundDivision(offset.z, numberOfSamples));

    noise = static_cast<int16_t>(roundDivision(noise, numberOfSamples));

    Console.print(F("gyro calib min="));
    Console.print(zmin);
    Console.print(F(" max="));
    Console.print(zmax);
    Console.print(F(" Offset="));
    Console.print(offset.z);
    Console.print(F(" noise="));
    Console.println(m_gyroNoise);

    if (noise < 20)
    {
      // Optimum found
      m_gyroOffset = offset;
      m_gyroNoise = noise;
      calibrationDone = true;
    }
  }
  m_useGyroCalibration = true;

  Console.print(F("Offset="));
  printPointln(m_gyroOffset);
  Console.println(F("------------"));
}

void Imu::initAccelerometer()
{
  Console.println(F("Init Accelerometer/Magnetometer"));
  m_accMag.init();
  Console.print(F("deviceType="));
  Console.print(m_accMag.getDeviceType());

  uint8_t retry = 0;
  for (;;)
  {
    uint8_t devId = m_accMag.readReg(LSM303::WHO_AM_I_M);
    Console.print(F(" devId="));
    Console.println(devId);
    if (devId != LSM303::DEV_ID_LSM303DLM)
    {
      Console.println(F("magnetometer read error"));
      retry++;
      if (retry > 2)
      {
        m_errorCounter++;
        return;
      }
      delay(1000);
    }
    else
    {
      break;
    }
  }

  m_accMag.enableDefault();
  switch (m_accMag.getDeviceType())
  {
    case LSM303::DEVICE_D:
      // 8 g full scale: AFS = 011
      m_accMag.writeReg(LSM303::CTRL2, 0x18);
      break;

    case LSM303::DEVICE_DLHC:
      // 8 g full scale: FS = 10; high resolution output mode
      m_accMag.writeReg(LSM303::CTRL_REG4_A, 0x28);
      break;

    case LSM303::DEVICE_DLM:
    case LSM303::DEVICE_DLH:
      // 8 g full scale: FS = 11
      m_accMag.writeReg(LSM303::CTRL_REG4_A, 0x30);
      break;

    case LSM303::DEVICE_AUTO:
    default:
      break;
  }
}

void Imu::readAccelerometer()
{
  m_accMag.readAcc();

  if (m_useAccCalibration)
  {
    m_acc = (static_cast<Vector<float>>(m_accMag.m_acc) -
        m_calibrationData.accelOffset) / m_calibrationData.accelScale;
  }
  else
  {
    m_acc = m_accMag.m_acc;
  }
}

// L3G4200D gyro sensor driver
bool Imu::initGyroscope()
{
  Console.println(F("initL3G4200D"));
  m_gyro.init(L3G::DEVICE_AUTO, L3G::SA0_AUTO);
  Console.print(F("deviceType="));
  Console.print(m_gyro.getDeviceType());

  uint8_t retry = 0;
  for (;;)
  {
    uint8_t devId = m_gyro.readReg(L3G::WHO_AM_I);
    Console.print(F(" devId="));
    Console.println(devId);
    if (devId != L3G::DEV_ID_4200D)
    {
      Console.println(F("gyro read error"));
      retry++;
      if (retry > 2)
      {
        m_errorCounter++;
        return false;
      }
      delay(1000);
    }
    else
    {
      break;
    }
  }

  m_gyro.enableDefault();

  delay(250);
  //calibrateGyro(); // TODO: Shall calibration not be used?
  return true;
}

void Imu::readGyroscope(void)
{
  m_gyro.read();

  if (m_useGyroCalibration)
  {
    // Compensate for offset
    m_gyro.m_g -= m_gyroOffset;
  }
}

// LSM303 magnetometer sensor driver
void Imu::initMagnetometer()
{
  // Init of magnetometer is handled by initAccelerometer since it is the same
  // device.
  if (!m_accMag.isInitialized())
  {
    initAccelerometer();
  }
}

void Imu::readMagnetometer()
{
  m_accMag.readMag();

  if (m_useMagCalibration)
  {
    m_mag = static_cast<Vector<float>>(
        m_accMag.m_mag - m_calibrationData.magnetometerOffset) /
            static_cast<Vector<float>>(m_calibrationData.magnetometerScale);
  }
  else
  {
    m_mag = m_accMag.m_mag;
  }
}

void Imu::calibrateMagnetometerStartStop(void)
{
  while (Console.available())
  {
    Console.read();
  }

  if (m_state == ImuStateE::RUN)
  {
    // start
    Console.println(F("Magnetometer calibration..."));
    Console.println(F("Rotate sensor 360 degree around all three axis"));
    m_foundNewMinMax = false;
    m_useMagCalibration = false;
    m_state = ImuStateE::CALIBRATE_MAG;
    m_magMin = Vector<int16_t>(INT16_MAX);
    m_magMax = Vector<int16_t>(INT16_MIN);
  }
  else
  {
    // stop
    Console.println(F("Magnetometer calibration completed"));
    m_calibrationAvailable = true;

    Vector<int16_t> range = m_magMax - m_magMin;

    m_calibrationData.magnetometerScale.x =
        static_cast<int16_t>(roundDivision(range.x, 2));
    m_calibrationData.magnetometerScale.y =
        static_cast<int16_t>(roundDivision(range.y, 2));
    m_calibrationData.magnetometerScale.z =
        static_cast<int16_t>(roundDivision(range.z, 2));

    m_calibrationData.magnetometerOffset =
        m_calibrationData.magnetometerScale + m_magMin;

    saveCalibrationData();
    printCalibrationData();
    m_useMagCalibration = true;
    m_state = ImuStateE::RUN;

    playCompletedSound();
    delay(500);
  }
}

void Imu::calibrateMagnetometerUpdate(void)
{
  m_magLast = m_accMag.m_mag;

  delay(20);

  readMagnetometer();

  if (abs(m_accMag.m_mag.x - m_magLast.x) < 10 &&
      abs(m_accMag.m_mag.y - m_magLast.y) < 10 &&
      abs(m_accMag.m_mag.z - m_magLast.z) < 10)
  {
    bool newfound = false;

    for (uint8_t axis = 0; axis < 3; axis++)
    {
      if (m_accMag.m_mag.data[axis] < m_magMin.data[axis])
      {
        m_magMin.data[axis] = m_accMag.m_mag.data[axis];
        newfound = true;
      }

      if (m_accMag.m_mag.data[axis] > m_magMax.data[axis])
      {
        m_magMax.data[axis] = m_accMag.m_mag.data[axis];
        newfound = true;
      }
    }

    if (newfound)
    {
      m_foundNewMinMax = true;
      m_buzzer_p->beep(440);

      printPointMinMax(m_magMin, m_magMax);
    }
    else
    {
      m_buzzer_p->beepStop();
    }
  }
}

// calculate acceleration sensor offsets
bool Imu::calibrateAccelerometerNextAxis(void)
{
  const uint8_t numberOfSamples = 32;
  bool complete = false;
  m_buzzer_p->beep(440);

  while (Console.available())
  {
    Console.read();
  }

  m_useAccCalibration = false;

  if (m_calibAccAxisCounter >= 6)
  {
    m_calibAccAxisCounter = 0;
  }

  if (m_calibAccAxisCounter == 0)
  {
    // restart
    Console.println(F("Accelerometer calibration start..."));
    m_accMin = Vector<float>(9999);
    m_accMax = Vector<float>(-9999);
  }

  // Get sample values from accelerometer
  Vector<int16_t> acc;
  for (uint8_t i = 0; i < numberOfSamples; i++)
  {
    readAccelerometer();

    acc += m_accMag.m_acc;

    printPointln(m_accMag.m_acc);

    delay(10);
  }

  // Average values
  Vector<float> average = static_cast<Vector<float>>(acc) / numberOfSamples;

  // Update min & max values
  m_accMin.x = min(m_accMin.x, average.x);
  m_accMax.x = max(m_accMax.x, average.x);
  m_accMin.y = min(m_accMin.y, average.y);
  m_accMax.y = max(m_accMax.y, average.y);
  m_accMin.z = min(m_accMin.z, average.z);
  m_accMax.z = max(m_accMax.z, average.z);

  printPointMinAvgMax(m_accMin, average, m_accMax);

  m_calibAccAxisCounter++;
  m_useAccCalibration = true;

  Console.print(F("side "));
  Console.print(m_calibAccAxisCounter);
  Console.println(F(" of 6 completed"));

  if (m_calibAccAxisCounter == 6)
  {
    // all axis complete
    m_calibrationData.accelScale = (m_accMax - m_accMin) / 2;

    m_calibrationData.accelOffset = m_calibrationData.accelScale + m_accMin;

    printCalibrationData();
    saveCalibrationData();

    Console.println(F("Accelerometer calibration completed"));
    complete = true;

    playCompletedSound();
  }
  delay(500);

  return complete;
}

// Second-order complementary filter
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Imu::complementary2(const float newAngle, const float newRate,
    const int16_t looptime, float angle)
{
  const float k = 10;

  float dt = static_cast<float>(looptime) / 1000.0f;

  float dAngle = newAngle - angle;
  float x1 = dAngle * 2 * k;
  float x2 = dAngle * k * k * dt;
  float y1 = newRate + x1 + x2;

  angle = dt * y1;

  return angle;
}

// Kalman filter
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Imu::kalman(const float newAngle, const float newRate,
                  const int16_t looptime, float x_angle)
{
  const float Q_angle = 0.01f; //0.001
  const float Q_gyro = 0.0003f;  //0.003
  const float R_angle = 0.01f;  //0.03

  static float x_bias = 0;
  static float P_00 = 0;
  static float P_01 = 0;
  static float P_10 = 0;
  static float P_11 = 0;

  float dt = static_cast<float>(looptime) / 1000;
  x_angle += dt * (newRate - x_bias);
  P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
  P_01 += -dt * P_11;
  P_10 += -dt * P_11;
  P_11 += Q_gyro * dt;

  float y = newAngle - x_angle;
  float S = P_00 + R_angle;
  float K_0 = P_00 / S;
  float K_1 = P_10 / S;

  x_angle += K_0 * y;
  x_bias += K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return x_angle;
}

// scale setangle, so that both PI angles have the same sign
float Imu::scalePIangles(const float setAngle, const float currAngle)
{
  if (setAngle >= PI / 2 && currAngle <= -PI / 2)
  {
    return (setAngle - 2 * PI);
  }
  else if (setAngle <= -PI / 2 && currAngle >= PI / 2)
  {
    return (setAngle + 2 * PI);
  }
  else
  {
    return setAngle;
  }
}

void Imu::update(void)
{
  read();
  uint32_t curMillis = millis();
  int16_t looptime = static_cast<int16_t>(curMillis - m_lastAhrsTime);
  m_lastAhrsTime = curMillis;

  if (m_state == ImuStateE::RUN)
  {
    // ------ roll, pitch --------------
    m_accPitch = atan2(m_acc.x, sqrt(sq(m_acc.y) + sq(m_acc.z)));
    m_accRoll = atan2(m_acc.y, m_acc.z);
    m_scaledPitch = scalePIangles(m_accPitch, m_ypr.pitch);
    m_scaledRoll = scalePIangles(m_accRoll, m_ypr.roll);

    // complementary filter
    m_filtPitch = kalman(m_scaledPitch, m_gyro.m_g.x, looptime, m_ypr.pitch);
    m_filtRoll = kalman(m_scaledRoll, m_gyro.m_g.y, looptime, m_ypr.roll);

    m_ypr.pitch = scalePI(m_filtPitch);
    m_ypr.roll = scalePI(m_filtRoll);

    // ------ yaw --------------
    // tilt-compensated yaw
    m_magTilt.x = m_mag.x * cos(m_ypr.pitch) + m_mag.z * sin(m_ypr.pitch);

    m_magTilt.y = m_mag.x * sin(m_ypr.roll) * sin(m_ypr.pitch) +
                m_mag.y * cos(m_ypr.roll) -
                m_mag.z * sin(m_ypr.roll) * cos(m_ypr.pitch);

    m_magTilt.z = -m_mag.x * cos(m_ypr.roll) * sin(m_ypr.pitch) +
                 m_mag.y * sin(m_ypr.roll) +
                 m_mag.z * cos(m_ypr.roll) * cos(m_ypr.pitch);

    m_yaw = atan2(m_magTilt.y, m_magTilt.x);
    m_scaledYaw = scalePI(m_yaw);
    m_scaled2Yaw = scalePIangles(m_scaledYaw, m_ypr.yaw);

    // Complementary filter
    m_filtYaw = complementary2(m_scaled2Yaw, static_cast<float>(-m_gyro.m_g.z),
        looptime, m_ypr.yaw);
    m_ypr.yaw = scalePI(m_filtYaw);
  }
  else if (m_state == ImuStateE::CALIBRATE_MAG)
  {
    calibrateMagnetometerUpdate();
  }
}

void Imu::printInfo(Stream &s)
{
  Streamprint(s, "imu a=+%2.2f +%2.2f +%2.2f",
      m_acc.x, m_acc.y, m_acc.z);

  Streamprint(s, " m=+%2.2f +%2.2f +%2.2f",
      m_mag.x, m_mag.y, m_mag.z);

  Streamprint(s, " g=%4d %4d %4d",
      m_gyro.m_g.x, m_gyro.m_g.y, m_gyro.m_g.z);

  Streamprint(s, " P=+%2.2f +%2.2f +%2.2f +%2.2f",
      m_accPitch, m_scaledPitch, m_filtPitch, m_ypr.pitch);

  Streamprint(s, " R=+%2.2f +%2.2f +%2.2f +%2.2f",
      m_accRoll, m_scaledRoll, m_filtRoll, m_ypr.roll);

  Streamprint(s, " T=+%2.2f +%2.2f +%2.2f",
      m_magTilt.x, m_magTilt.y, m_magTilt.z);

  Streamprint(s, " Y=+%2.2f +%2.2f +%2.2f +%2.2f +%2.2f\n",
      m_yaw, m_scaledYaw, m_scaled2Yaw, m_filtYaw, m_ypr.yaw);
}

bool Imu::init(Buzzer* buzzer_p)
{
  m_buzzer_p = buzzer_p;
  loadCalibrationData();
  printCalibrationData();
  if (!initGyroscope())
  {
    return false;
  }
  initAccelerometer();
  initMagnetometer();
  m_hardwareInitialized = true;
  return true;
}

uint16_t Imu::getAndClearCallCounter(void)
{
  uint16_t res = m_callCounter;
  m_callCounter = 0;
  return res;
}

uint16_t Imu::getAndClearErrorCounter(void)
{
  uint16_t res = m_errorCounter;
  m_errorCounter = 0;
  return res;
}

void Imu::read(void)
{
  if (!m_hardwareInitialized)
  {
    m_errorCounter++;
    return;
  }
  m_callCounter++;
  readGyroscope();
  readAccelerometer();
  readMagnetometer();
}

void Imu::playCompletedSound(void)
{
  BeepData data[] =
  {
      { 600, 200 },
      { 880, 200 },
      { 1320, 200 },
  };

  m_buzzer_p->beep(data, sizeof(data) / sizeof(data[0]));
}
