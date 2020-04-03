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
#include "drivers.h"
#include "Imu.h"
#include "L3G.h"
#include "Buzzer.h"

#define ADDR 600
#define MAGIC 6

#define roundDivision(a, b) ((a) >= 0 ? ((a)+(b)/2) / (b):((a)-(b)/2) / (b))

// rescale to -PI..+PI
float Imu::scalePI(const float v)
{
  float d = v;
  while (d < 0)
  {
    d += 2 * PI;
  }
  while (d >= 2 * PI)
  {
    d -= 2 * PI;
  }
  if (d >= PI)
  {
    return (-2 * PI + d);
  }
  else if (d < -PI)
  {
    return (2 * PI + d);
  }
  else
  {
    return d;
  }
}

// rescale to -180..+180
float Imu::scale180(const float v)
{
  float d = v;
  while (d < 0)
  {
    d += 2 * 180;
  }
  while (d >= 2 * 180)
  {
    d -= 2 * 180;
  }
  if (d >= 180)
  {
    return (-2 * 180 + d);
  }
  else if (d < -180)
  {
    return (2 * 180 + d);
  }
  else
  {
    return d;
  }
}

// Computes minimum distance between x radiant (current-value) and w radiant (set-value)
// cases:
// w=330 degree, x=350 degree => -20 degree
// w=350 degree, x=10  degree => -20 degree
// w=10  degree, x=350 degree =>  20 degree
// w=0   degree, x=190 degree => 170 degree
// w=190 degree, x=0   degree => -170 degree
float Imu::distance180(const float x, const float w)
{
  float d = scale180(w - x);
  if (d < -180)
  {
    d = d + 2 * 180;
  }
  else if (d > 180)
  {
    d = d - 2 * 180;
  }
  return d;
}

// Computes minimum distance between x radiant (current-value) and w radiant (set-value)
float Imu::distancePI(const float x, const float w)
{
  float d = scalePI(w - x);
  if (d < -PI)
  {
    d = d + 2 * PI;
  }
  else if (d > PI)
  {
    d = d - 2 * PI;
  }
  return d;
}

// weight fusion (w=0..1) of two radiant values (a, b)
float Imu::fusionPI(const float w, const float a, const float b)
{
  float c;
  if ((b >= PI / 2) && (a <= -PI / 2))
  {
    c = w * a + (1.0 - w) * (b - 2 * PI);
  }
  else if ((b <= -PI / 2) && (a >= PI / 2))
  {
    c = w * (a - 2 * PI) + (1.0 - w) * b;
  }
  else
  {
    c = w * a + (1.0 - w) * b;
  }
  return scalePI(c);
}

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
  m_calibrationData.accelOffset = {0.0, 0.0, 0.0};
  m_calibrationData.accelScale = {1.0, 1.0, 1.0};
  m_calibrationData.magnetometerOffset = {0, 0, 0};
  m_calibrationData.magnetometerScale = {1, 1, 1};
  m_calibrationAvailable = false;
  Console.println(F("IMU calibration deleted"));
}

void Imu::printPoint(const point_float_t point)
{
  Console.print(point.x);
  Console.print(",");
  Console.print(point.y);
  Console.print(",");
  Console.print(point.z);
}
void Imu::printPointln(const point_float_t point)
{
  printPoint(point);
  Console.println();
}

void Imu::printPoint(const point_int_t point)
{
  Console.print(point.x);
  Console.print(",");
  Console.print(point.y);
  Console.print(",");
  Console.print(point.z);
}
void Imu::printPointln(const point_int_t point)
{
  printPoint(point);
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
  Console.println(F("---calibGyro---"));
  m_useGyroCalibration = false;
  m_gyroOffset = { 0, 0, 0 };
  point_int_t offset;
  for (;;)
  {
    int16_t zmin = INT16_MAX;
    int16_t zmax = INT16_MIN;
    int16_t noise = 0;
    offset = { 0, 0, 0 };

    for (uint8_t i = 0; i < numberOfSamples; i++)
    {
      readGyroscope();

      zmin = min(zmin, m_gyro.m_g.z);
      zmax = max(zmax, m_gyro.m_g.z);

      offset.x += m_gyro.m_g.x;
      offset.y += m_gyro.m_g.y;
      offset.z += m_gyro.m_g.z;
      noise += sq(m_gyro.m_g.z - m_gyroOffset.z); // noise is computed with last offset calculation

      delay(10);
    }

    offset.x = roundDivision(offset.x, numberOfSamples);
    offset.y = roundDivision(offset.y, numberOfSamples);;
    offset.z = roundDivision(offset.z, numberOfSamples);;
    noise = roundDivision(noise, numberOfSamples);;

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
      // optimum found
      m_gyroOffset = offset;
      m_gyroNoise = noise;
      break;
    }
  }
  m_useGyroCalibration = true;

  Console.print(F("Offset="));
  printPointln(m_gyroOffset);
  Console.println(F("------------"));
}

void Imu::initAccelerometer()
{
  Console.println(F("init Accelerometer/Magnetometer"));
  m_accMag.init();
  Console.print(F("deviceType="));
  Console.print(m_accMag.getDeviceType());

  uint8_t devId;
  int retry = 0;
  for (;;)
  {
    devId = m_accMag.readReg(LSM303::WHO_AM_I_M);
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
      m_accMag.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::DEVICE_DLHC:
      m_accMag.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      m_accMag.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
}

void Imu::readAccelerometer()
{
  m_accMag.readAcc();

  if (m_useAccelCalibration)
  {
    m_acc.x = ((float)m_accMag.m_acc.x - m_calibrationData.accelOffset.x) / m_calibrationData.accelScale.x;
    m_acc.y = ((float)m_accMag.m_acc.y - m_calibrationData.accelOffset.y) / m_calibrationData.accelScale.y;
    m_acc.z = ((float)m_accMag.m_acc.z - m_calibrationData.accelOffset.z) / m_calibrationData.accelScale.z;
  }
  else
  {
    m_acc.x = (float)m_accMag.m_acc.x;
    m_acc.y = (float)m_accMag.m_acc.y;
    m_acc.z = (float)m_accMag.m_acc.z;
  }
}

// L3G4200D gyro sensor driver
bool Imu::initGyroscope()
{
  Console.println(F("initL3G4200D"));
  m_gyro.init(L3G::DEVICE_AUTO, L3G::SA0_AUTO);
  Console.print(F("deviceType="));
  Console.print(m_gyro.getDeviceType());

  uint8_t devId;
  int retry = 0;
  for (;;)
  {
    devId = m_gyro.readReg(L3G::WHO_AM_I);
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
  //calibGyro();
  return true;
}

void Imu::readGyroscope(void)
{
  m_gyro.read();

  if (m_useGyroCalibration)
  {
    // Compensate for offset
    m_gyro.m_g.x -= m_gyroOffset.x;
    m_gyro.m_g.y -= m_gyroOffset.y;
    m_gyro.m_g.z -= m_gyroOffset.z;
  }
}

// LSM303 magnetometer sensor driver
void Imu::initMagnetometer()
{
  // Init is handled by initAccelerometer since it is the same device
  if (!m_accMag.isInitialized())
  {
    initAccelerometer();
  }
}

void Imu::readMagnetometer()
{
  m_accMag.readMag();

  if (m_useMagnetometerCalibration)
  {
    m_mag.x = (float)(m_accMag.m_acc.x - m_calibrationData.magnetometerOffset.x) /
            (float)m_calibrationData.magnetometerScale.x;
    m_mag.y = (float)(m_accMag.m_acc.y - m_calibrationData.magnetometerOffset.y) /
            (float)m_calibrationData.magnetometerScale.y;
    m_mag.z = (float)(m_accMag.m_acc.z - m_calibrationData.magnetometerOffset.z) /
            (float)m_calibrationData.magnetometerScale.z;
  }
  else
  {
    m_mag.x = m_accMag.m_acc.x;
    m_mag.y = m_accMag.m_acc.y;
    m_mag.z = m_accMag.m_acc.z;
  }
}

void Imu::calibrateMagnetometerStartStop(void)
{
  while (Console.available())
  {
    Console.read();
  }

  if (m_state == IMU_RUN)
  {
    // start
    Console.println(F("Magnetometer calibration..."));
    Console.println(F("Rotate sensor 360 degree around all three axis"));
    m_foundNewMinMax = false;
    m_useMagnetometerCalibration = false;
    m_state = IMU_CAL_COM;
    m_magMin = { INT16_MAX, INT16_MAX, INT16_MAX };
    m_magMax = { INT16_MIN, INT16_MIN, INT16_MIN };
  }
  else
  {
    // stop
    Console.println(F("Magnetometer calibration completed"));
    m_calibrationAvailable = true;

    point_int_t range;
    range.x = m_magMax.x - m_magMin.x;
    range.y = m_magMax.y - m_magMin.y;
    range.z = m_magMax.z - m_magMin.z;

    m_calibrationData.magnetometerScale.x = roundDivision(range.x, 2);
    m_calibrationData.magnetometerScale.y = roundDivision(range.y, 2);
    m_calibrationData.magnetometerScale.z = roundDivision(range.z, 2);

    m_calibrationData.magnetometerOffset.x = m_calibrationData.magnetometerScale.x + m_magMin.x;
    m_calibrationData.magnetometerOffset.y = m_calibrationData.magnetometerScale.y + m_magMin.y;
    m_calibrationData.magnetometerOffset.z = m_calibrationData.magnetometerScale.z + m_magMin.z;

    saveCalibrationData();
    printCalibrationData();
    m_useMagnetometerCalibration = true;
    m_state = IMU_RUN;

    playCompletedSound();
    delay(500);
  }
}

void Imu::calibrateMagnetometerUpdate(void)
{
  m_magLast.x = m_accMag.m_mag.x;
  m_magLast.y = m_accMag.m_mag.y;
  m_magLast.z = m_accMag.m_mag.z;

  delay(20);

  readMagnetometer();

  bool newfound = false;

  if (abs(m_accMag.m_mag.x - m_magLast.x) < 10 &&
      abs(m_accMag.m_mag.y - m_magLast.y) < 10 &&
      abs(m_accMag.m_mag.z - m_magLast.z) < 10)
  {
    if (m_accMag.m_mag.x < m_magMin.x)
    {
      m_magMin.x = m_accMag.m_mag.x;
      newfound = true;
    }
    if (m_accMag.m_mag.y < m_magMin.y)
    {
      m_magMin.y = m_accMag.m_mag.y;
      newfound = true;
    }
    if (m_accMag.m_mag.z < m_magMin.z)
    {
      m_magMin.z = m_accMag.m_mag.z;
      newfound = true;
    }

    if (m_accMag.m_mag.x > m_magMax.x)
    {
      m_magMax.x = m_accMag.m_mag.x;
      newfound = true;
    }
    if (m_accMag.m_mag.y > m_magMax.y)
    {
      m_magMax.y = m_accMag.m_mag.y;
      newfound = true;
    }
    if (m_accMag.m_mag.z > m_magMax.z)
    {
      m_magMax.z = m_accMag.m_mag.z;
      newfound = true;
    }

    if (newfound)
    {
      m_foundNewMinMax = true;
      m_buzzer_p->beep(440);

      Console.print("x:");
      Console.print(m_magMin.x);
      Console.print(",");
      Console.print(m_magMax.x);
      Console.print("  y:");
      Console.print(m_magMin.y);
      Console.print(",");
      Console.print(m_magMax.y);
      Console.print("  z:");
      Console.print(m_magMin.z);
      Console.print(",");
      Console.println(m_magMax.z);
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

  m_useAccelCalibration = false;

  if (m_calibAccelAxisCounter >= 6)
  {
    m_calibAccelAxisCounter = 0;
  }

  if (m_calibAccelAxisCounter == 0)
  {
    // restart
    Console.println(F("Accelerometer calibration start..."));
    m_accMin = { 9999, 9999, 9999 };
    m_accMax = { -9999, -9999, -9999 };
  }

  // Get sample values from accelerometer
  point_int_t acc = { 0, 0, 0 };
  for (uint8_t i = 0; i < numberOfSamples; i++)
  {
    readAccelerometer();

    acc.x += m_accMag.m_acc.x;
    acc.y += m_accMag.m_acc.y;
    acc.z += m_accMag.m_acc.z;

    Console.print(m_accMag.m_acc.x);
    Console.print(",");
    Console.print(m_accMag.m_acc.y);
    Console.print(",");
    Console.println(m_accMag.m_acc.z);

    delay(10);
  }

  // Average values
  point_float_t average;
  average.x = (float)acc.x / numberOfSamples;
  average.y = (float)acc.y / numberOfSamples;
  average.z = (float)acc.z / numberOfSamples;

  // Update min & max values
  m_accMin.x = min(m_accMin.x, average.x);
  m_accMax.x = max(m_accMax.x, average.x);

  Console.print(" x: min=");
  Console.print(m_accMin.x);
  Console.print(" avg=");
  Console.print(average.x);
  Console.print(" max=");
  Console.println(m_accMax.x);

  m_accMin.y = min(m_accMin.y, average.y);
  m_accMax.y = max(m_accMax.y, average.y);

  Console.print(" y: min=");
  Console.print(m_accMin.y);
  Console.print(" avg=");
  Console.print(average.y);
  Console.print(" max=");
  Console.println(m_accMax.y);

  m_accMin.z = min(m_accMin.z, average.z);
  m_accMax.z = max(m_accMax.z, average.z);

  Console.print(" z: min=");
  Console.print(m_accMin.z);
  Console.print(" avg=");
  Console.print(average.z);
  Console.print(" max=");
  Console.println(m_accMax.z);

  m_calibAccelAxisCounter++;
  m_useAccelCalibration = true;

  Console.print("side ");
  Console.print(m_calibAccelAxisCounter);
  Console.println(" of 6 completed");

  if (m_calibAccelAxisCounter == 6)
  {
    // all axis complete
    point_float_t range;
    range.x = m_accMax.x - m_accMin.x;
    range.y = m_accMax.y - m_accMin.y;
    range.z = m_accMax.z - m_accMin.z;

    m_calibrationData.accelScale.x = range.x / 2;
    m_calibrationData.accelScale.y = range.y / 2;
    m_calibrationData.accelScale.z = range.z / 2;

    m_calibrationData.accelOffset.x = m_calibrationData.accelScale.x + m_accMin.x;
    m_calibrationData.accelOffset.y = m_calibrationData.accelScale.y + m_accMin.y;
    m_calibrationData.accelOffset.z = m_calibrationData.accelScale.z + m_accMin.z;

    printCalibrationData();
    saveCalibrationData();

    Console.println("Accelerometer calibration completed");
    complete = true;

    playCompletedSound();
  }
  delay(500);

  return complete;
}

// First-order complementary filter
// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Imu::Complementary(const float newAngle, const float newRate,
                         const int looptime, float angle)
{
  const float tau = 0.075;
  const float dtC = float(looptime) / 1000.0;
  const float a = tau / (tau + dtC);

  angle = a * (angle + newRate * dtC) + (1 - a) * newAngle;

  return angle;
}

// Second-order complementary filter
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Imu::Complementary2(const float newAngle, const float newRate,
                          const int looptime, float angle)
{
  const float k = 10;
  const float dtc2 = float(looptime) / 1000.0;
  const float x1 = (newAngle - angle) * k * k;
  static float y1 = dtc2 * x1 + y1;  // FIXME: y1?
  const float x2 = y1 + (newAngle - angle) * 2 * k + newRate;

  angle = dtc2 * x2 + angle;

  return angle;
}

// Kalman filter
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Imu::Kalman(const float newAngle, const float newRate,
                  const int looptime, float x_angle)
{
  const float Q_angle = 0.01; //0.001
  const float Q_gyro = 0.0003;  //0.003
  const float R_angle = 0.01;  //0.03

  static float x_bias = 0;
  static float P_00 = 0;
  static float P_01 = 0;
  static float P_10 = 0;
  static float P_11 = 0;

  const float dt = (float)looptime / 1000;
  x_angle += dt * (newRate - x_bias);
  P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
  P_01 += -dt * P_11;
  P_10 += -dt * P_11;
  P_11 += Q_gyro * dt;

  const float y = newAngle - x_angle;
  const float S = P_00 + R_angle;
  const float K_0 = P_00 / S;
  const float K_1 = P_10 / S;

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
  unsigned long curMillis = millis();
  int looptime = (curMillis - m_lastAHRSTime);
  m_lastAHRSTime = curMillis;

  if (m_state == IMU_RUN)
  {
    // ------ roll, pitch --------------
    m_accPitch = atan2(m_acc.x, sqrt(sq(m_acc.y) + sq(m_acc.z)));
    m_accRoll = atan2(m_acc.y, m_acc.z);
    m_scaledPitch = scalePIangles(m_accPitch, m_ypr.pitch);
    m_scaledRoll = scalePIangles(m_accRoll, m_ypr.roll);

    // complementary filter
    m_filtPitch = Kalman(m_scaledPitch, m_gyro.m_g.x, looptime, m_ypr.pitch);
    m_filtRoll = Kalman(m_scaledRoll, m_gyro.m_g.y, looptime, m_ypr.roll);

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
    m_filtYaw = Complementary2(m_scaled2Yaw, -m_gyro.m_g.z, looptime, m_ypr.yaw);
    m_ypr.yaw = scalePI(m_filtYaw);
  }
  else if (m_state == IMU_CAL_COM)
  {
    calibrateMagnetometerUpdate();
  }
}

void Imu::printInfo(Stream &s)
{
  Streamprint(s, "imu a=%s%2d.%02d %s%2d.%02d %s%2d.%02d",
              m_acc.x >= 0.0 ? "+" : "-", abs((int)m_acc.x), abs((int)(m_acc.x * 100) - ((int)m_acc.x * 100)),
              m_acc.y >= 0.0 ? "+" : "-", abs((int)m_acc.y), abs((int)(m_acc.y * 100) - ((int)m_acc.y * 100)),
              m_acc.z >= 0.0 ? "+" : "-", abs((int)m_acc.z), abs((int)(m_acc.z * 100) - ((int)m_acc.z * 100)));
  Streamprint(s, " m=%s%2d.%02d %s%2d.%02d %s%2d.%02d",
              m_mag.x >= 0.0 ? "+" : "-", abs((int)m_mag.x), abs((int)(m_mag.x * 100) - ((int)m_mag.x * 100)),
              m_mag.y >= 0.0 ? "+" : "-", abs((int)m_mag.y), abs((int)(m_mag.y * 100) - ((int)m_mag.y * 100)),
              m_mag.z >= 0.0 ? "+" : "-", abs((int)m_mag.z), abs((int)(m_mag.z * 100) - ((int)m_mag.z * 100)));
  Streamprint(s, " g=%4d %4d %4d", m_gyro.m_g.x, m_gyro.m_g.y, m_gyro.m_g.z);
  Streamprint(s, " P=%s%2d.%02d %s%2d.%02d %s%2d.%02d %s%2d.%02d",
              m_accPitch >= 0.0 ? "+" : "-", abs((int)m_accPitch), abs((int)(m_accPitch * 100) - ((int)m_accPitch * 100)),
              m_scaledPitch >= 0.0 ? "+" : "-", abs((int)m_scaledPitch), abs((int)(m_scaledPitch * 100) - ((int)m_scaledPitch * 100)),
              m_filtPitch >= 0.0 ? "+" : "-", abs((int)m_filtPitch), abs((int)(m_filtPitch * 100) - ((int)m_filtPitch * 100)),
              m_ypr.pitch >= 0.0 ? "+" : "-", abs((int)m_ypr.pitch), abs((int)(m_ypr.pitch * 100) - ((int)m_ypr.pitch * 100)));
  Streamprint(s, " R=%s%2d.%02d %s%2d.%02d %s%2d.%02d %s%2d.%02d",
              m_accRoll >= 0.0 ? "+" : "-", abs((int)m_accRoll), abs((int)(m_accRoll * 100) - ((int)m_accRoll * 100)),
              m_scaledRoll >= 0.0 ? "+" : "-", abs((int)m_scaledRoll), abs((int)(m_scaledRoll * 100) - ((int)m_scaledRoll * 100)),
              m_filtRoll >= 0.0 ? "+" : "-", abs((int)m_filtRoll), abs((int)(m_filtRoll * 100) - ((int)m_filtRoll * 100)),
              m_ypr.roll >= 0.0 ? "+" : "-", abs((int)m_ypr.roll), abs((int)(m_ypr.roll * 100) - ((int)m_ypr.roll * 100)));
  Streamprint(s, " T=%s%2d.%02d %s%2d.%02d %s%2d.%02d",
              m_magTilt.x >= 0.0 ? "+" : "-", abs((int)m_magTilt.x), abs((int)(m_magTilt.x * 100) - ((int)m_magTilt.x * 100)),
              m_magTilt.y >= 0.0 ? "+" : "-", abs((int)m_magTilt.y), abs((int)(m_magTilt.y * 100) - ((int)m_magTilt.y * 100)),
              m_magTilt.z >= 0.0 ? "+" : "-", abs((int)m_magTilt.z), abs((int)(m_magTilt.z * 100) - ((int)m_magTilt.z * 100)));
  Streamprint(s, " Y=%s%2d.%02d %s%2d.%02d %s%2d.%02d\r\n",
              m_yaw >= 0.0 ? "+" : "-", abs((int)m_yaw), abs((int)(m_yaw * 100) - ((int)m_yaw * 100)),
              m_scaledYaw >= 0.0 ? "+" : "-", abs((int)m_scaledYaw), abs((int)(m_scaledYaw * 100) - ((int)m_scaledYaw * 100)),
              m_scaled2Yaw >= 0.0 ? "+" : "-", abs((int)m_scaled2Yaw), abs((int)(m_scaled2Yaw * 100) - ((int)m_scaled2Yaw * 100)),
              m_filtYaw >= 0.0 ? "+" : "-", abs((int)m_filtYaw), abs((int)(m_filtYaw * 100) - ((int)m_filtYaw * 100)),
              m_ypr.yaw >= 0.0 ? "+" : "-", abs((int)m_ypr.yaw), abs((int)(m_ypr.yaw * 100) - ((int)m_ypr.yaw * 100)));
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

int Imu::getCallCounter(void)
{
  int res = m_callCounter;
  m_callCounter = 0;
  return res;
}

int Imu::getErrorCounter(void)
{
  int res = m_errorCounter;
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
