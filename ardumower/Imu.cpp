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

#include <Arduino.h>
#include <Wire.h>
#include "drivers.h"
#include "Imu.h"
#include "L3G.h"

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

void Imu::loadSaveCalibrationData(const boolean readflag)
{
  int addr = ADDR;
  short magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  eereadwrite(readflag, addr, accelOffset);
  eereadwrite(readflag, addr, accelScale);
  eereadwrite(readflag, addr, magnetometerOffset);
  eereadwrite(readflag, addr, magnetometerScale);
}

void Imu::loadCalibrationData(void)
{
  short magic;
  int addr = ADDR;
  eeread(addr, magic);
  if (magic != MAGIC)
  {
    Console.println(F("IMU error: no calib data"));
    return;
  }
  calibrationAvailable = true;
  Console.println(F("IMU: Found calibration data"));
  loadSaveCalibrationData(true);
}

void Imu::deleteCalibrationData(void)
{
  int addr = ADDR;
  eewrite(addr, (short)0); // magic
  memset(&accelOffset, 0, sizeof(accelOffset));
  memset(&accelScale, 1, sizeof(accelScale));
  memset(&magnetometerOffset, 0, sizeof(magnetometerOffset));
  memset(&magnetometerScale, 1, sizeof(magnetometerScale));
  calibrationAvailable = false;
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
  Console.println(point.z);
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
  printPointln(accelOffset);
  Console.print(F("accScale="));
  printPointln(accelScale);
  Console.print(F("magOffset="));
  printPointln(magnetometerOffset);
  Console.print(F("magScale="));
  printPointln(magnetometerScale);
  Console.println(F("--------"));
}

// Calculate gyro offsets
void Imu::calibrateGyro(void)
{
  unsigned int numberOfSamples = 32;
  Console.println(F("---calibGyro---"));
  useGyroCalibration = false;
  memset(&gyroOffset, 0, sizeof(gyroOffset));
  point_int_t offset;
  for (;;)
  {
    int16_t zmin = INT16_MAX;
    int16_t zmax = INT16_MIN;
    int16_t noise = 0;
    memset(&offset, 0.0, sizeof(offset));

    for (unsigned int i = 0; i < numberOfSamples; i++)
    {
      readGyroscope();

      zmin = min(zmin, gyro.g.z);
      zmax = max(zmax, gyro.g.z);

      offset.x += gyro.g.x;
      offset.y += gyro.g.y;
      offset.z += gyro.g.z;
      noise += sq(gyro.g.z - gyroOffset.z); // noise is computed with last offset calculation

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
    Console.println(gyroNoise);

    if (noise < 20)
    {
      // optimum found
      gyroOffset = offset;
      gyroNoise = noise;
      break;
    }
  }
  useGyroCalibration = true;

  Console.print(F("Offset="));
  printPointln(gyroOffset);
  Console.println(F("------------"));
}

void Imu::initAccelerometer()
{
  Console.println(F("init Accelerometer/Magnetometer"));
  accMag.init();
  Console.print(F("deviceType="));
  Console.print(accMag.getDeviceType());

  uint8_t devId;
  int retry = 0;
  for (;;)
  {
    devId = accMag.readReg(LSM303::WHO_AM_I_M);
    Console.print(F(" devId="));
    Console.println(devId);
    if (devId != LSM303::DEV_ID_LSM303DLM)
    {
      Console.println(F("magnetometer read error"));
      retry++;
      if (retry > 2)
      {
        errorCounter++;
        return;
      }
      delay(1000);
    }
    else
    {
      break;
    }
  }

  accMag.enableDefault();
  switch (accMag.getDeviceType())
  {
    case LSM303::DEVICE_D:
      accMag.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::DEVICE_DLHC:
      accMag.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      accMag.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
}

void Imu::readAccelerometer()
{
  accMag.readAcc();

  if (useAccelCalibration)
  {
    accel.x = (accMag.a.x - accelOffset.x) / accelScale.x;
    accel.y = (accMag.a.y - accelOffset.y) / accelScale.y;
    accel.z = (accMag.a.z - accelOffset.z) / accelScale.z;
  }
  else
  {
    accel.x = (float)accMag.a.x;
    accel.y = (float)accMag.a.y;
    accel.z = (float)accMag.a.z;
  }
}

// L3G4200D gyro sensor driver
boolean Imu::initGyroscope()
{
  Console.println(F("initL3G4200D"));
  gyro.init(L3G::DEVICE_AUTO, L3G::SA0_AUTO);
  Console.print(F("deviceType="));
  Console.print(gyro.getDeviceType());

  uint8_t devId;
  int retry = 0;
  for (;;)
  {
    devId = gyro.readReg(L3G::WHO_AM_I);
    Console.print(F(" devId="));
    Console.println(devId);
    if (devId != L3G::DEV_ID_4200D)
    {
      Console.println(F("gyro read error"));
      retry++;
      if (retry > 2)
      {
        errorCounter++;
        return false;
      }
      delay(1000);
    }
    else
    {
      break;
    }
  }

  gyro.enableDefault();

  delay(250);
  //calibGyro();
  return true;
}

void Imu::readGyroscope(void)
{
  gyro.read();

  if (useGyroCalibration)
  {
    // Compensate for offset
    gyro.g.x -= gyroOffset.x;
    gyro.g.y -= gyroOffset.y;
    gyro.g.z -= gyroOffset.z;
  }
}

// LSM303 magnetometer sensor driver
void Imu::initMagnetometer()
{
  // Init is handled by initAccelerometer since it is the same device
  if (!accMag.isInitialized())
  {
    initAccelerometer();
  }
}

void Imu::readMagnetometer()
{
  accMag.readMag();

  if (useMagnetometerCalibration)
  {
    mag.x = (float)(accMag.a.x - magnetometerOffset.x) /
            (float)magnetometerScale.x;
    mag.y = (float)(accMag.a.y - magnetometerOffset.y) /
            (float)magnetometerScale.y;
    mag.z = (float)(accMag.a.z - magnetometerOffset.z) /
            (float)magnetometerScale.z;
  }
  else
  {
    mag.x = accMag.a.x;
    mag.y = accMag.a.y;
    mag.z = accMag.a.z;
  }
}

void Imu::calibrateMagnetometerStartStop(void)
{
  while (Console.available())
  {
    Console.read();
  }

  if (state == IMU_RUN)
  {
    // start
    Console.println(F("Magnetometer calibration..."));
    Console.println(F("Rotate sensor 360 degree around all three axis"));
    foundNewMinMax = false;
    useMagnetometerCalibration = false;
    state = IMU_CAL_COM;
    magMin = { INT16_MAX, INT16_MAX, INT16_MAX };
    magMax = { INT16_MIN, INT16_MIN, INT16_MIN };
  }
  else
  {
    // stop
    Console.println(F("Magnetometer calibration completed"));
    calibrationAvailable = true;

    point_int_t range;
    range.x = magMax.x - magMin.x;
    range.y = magMax.y - magMin.y;
    range.z = magMax.z - magMin.z;

    magnetometerScale.x = roundDivision(range.x, 2);
    magnetometerScale.y = roundDivision(range.y, 2);
    magnetometerScale.z = roundDivision(range.z, 2);

    magnetometerOffset.x = magnetometerScale.x + magMin.x;
    magnetometerOffset.y = magnetometerScale.y + magMin.y;
    magnetometerOffset.z = magnetometerScale.z + magMin.z;

    saveCalibrationData();
    printCalibrationData();
    useMagnetometerCalibration = true;
    state = IMU_RUN;

    playCompletedSound();
    noTone(pinBuzzer);
    delay(500);
  }
}

void Imu::calibrateMagnetometerUpdate(void)
{
  magLast.x = accMag.m.x;
  magLast.y = accMag.m.y;
  magLast.z = accMag.m.z;

  delay(20);

  readMagnetometer();

  boolean newfound = false;

  if (abs(accMag.m.x - magLast.x) < 10 &&
      abs(accMag.m.y - magLast.y) < 10 &&
      abs(accMag.m.z - magLast.z) < 10)
  {
    if (accMag.m.x < magMin.x)
    {
      magMin.x = accMag.m.x;
      newfound = true;
    }
    if (accMag.m.y < magMin.y)
    {
      magMin.y = accMag.m.y;
      newfound = true;
    }
    if (accMag.m.z < magMin.z)
    {
      magMin.z = accMag.m.z;
      newfound = true;
    }

    if (accMag.m.x > magMax.x)
    {
      magMax.x = accMag.m.x;
      newfound = true;
    }
    if (accMag.m.y > magMax.y)
    {
      magMax.y = accMag.m.y;
      newfound = true;
    }
    if (accMag.m.z > magMax.z)
    {
      magMax.z = accMag.m.z;
      newfound = true;
    }

    if (newfound)
    {
      foundNewMinMax = true;
      tone(pinBuzzer, 440);

      Console.print("x:");
      Console.print(magMin.x);
      Console.print(",");
      Console.print(magMax.x);
      Console.print("  y:");
      Console.print(magMin.y);
      Console.print(",");
      Console.print(magMax.y);
      Console.print("  z:");
      Console.print(magMin.z);
      Console.print(",");
      Console.println(magMax.z);
    }
    else
    {
      noTone(pinBuzzer);
    }
  }
}

// calculate acceleration sensor offsets
boolean Imu::calibrateAccelerometerNextAxis(void)
{
  unsigned int numberOfSamples = 32;
  boolean complete = false;
  tone(pinBuzzer, 440);

  while (Console.available())
  {
    Console.read();
  }

  useAccelCalibration = false;

  if (calibAccelAxisCounter >= 6)
  {
    calibAccelAxisCounter = 0;
  }

  if (calibAccelAxisCounter == 0)
  {
    // restart
    Console.println(F("Accelerometer calibration start..."));
    accelMin = { 9999, 9999, 9999 };
    accelMax = { -9999, -9999, -9999 };
  }

  // Get sample values from accelerometer
  point_int_t acc = { 0, 0, 0 };
  for (unsigned int i = 0; i < numberOfSamples; i++)
  {
    readAccelerometer();

    acc.x += accMag.a.x;
    acc.y += accMag.a.y;
    acc.z += accMag.a.z;

    Console.print(accMag.a.x);
    Console.print(",");
    Console.print(accMag.a.y);
    Console.print(",");
    Console.println(accMag.a.z);

    delay(10);
  }

  // Average values
  point_float_t average;
  average.x = (float)acc.x / numberOfSamples;
  average.y = (float)acc.y / numberOfSamples;
  average.z = (float)acc.z / numberOfSamples;

  // Update min & max values
  accelMin.x = min(accelMin.x, average.x);
  accelMax.x = max(accelMax.x, average.x);

  Console.print(" x: min=");
  Console.print(accelMin.x);
  Console.print(" avg=");
  Console.print(average.x);
  Console.print(" max=");
  Console.println(accelMax.x);

  accelMin.y = min(accelMin.y, average.y);
  accelMax.y = max(accelMax.y, average.y);

  Console.print(" y: min=");
  Console.print(accelMin.y);
  Console.print(" avg=");
  Console.print(average.y);
  Console.print(" max=");
  Console.println(accelMax.y);

  accelMin.z = min(accelMin.z, average.z);
  accelMax.z = max(accelMax.z, average.z);

  Console.print(" z: min=");
  Console.print(accelMin.z);
  Console.print(" avg=");
  Console.print(average.z);
  Console.print(" max=");
  Console.println(accelMax.z);

  calibAccelAxisCounter++;
  useAccelCalibration = true;

  Console.print("side ");
  Console.print(calibAccelAxisCounter);
  Console.println(" of 6 completed");

  if (calibAccelAxisCounter == 6)
  {
    // all axis complete
    point_float_t range;
    range.x = accelMax.x - accelMin.x;
    range.y = accelMax.y - accelMin.y;
    range.z = accelMax.z - accelMin.z;

    accelScale.x = range.x / 2;
    accelScale.y = range.y / 2;
    accelScale.z = range.z / 2;

    accelOffset.x = accelScale.x + accelMin.x;
    accelOffset.y = accelScale.y + accelMin.y;
    accelOffset.z = accelScale.z + accelMin.z;

    printCalibrationData();
    saveCalibrationData();

    Console.println("Accelerometer calibration completed");
    complete = true;

    playCompletedSound();
  }
  noTone(pinBuzzer);
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
  int looptime = (curMillis - lastAHRSTime);
  lastAHRSTime = curMillis;

  if (state == IMU_RUN)
  {
    // ------ roll, pitch --------------
    accelPitch = atan2(accel.x, sqrt(sq(accel.y) + sq(accel.z)));
    accelRoll = atan2(accel.y, accel.z);
    scaledPitch = scalePIangles(accelPitch, ypr.pitch);
    scaledRoll = scalePIangles(accelRoll, ypr.roll);

    // complementary filter
    filtPitch = Kalman(scaledPitch, gyro.g.x, looptime, ypr.pitch);
    filtRoll = Kalman(scaledRoll, gyro.g.y, looptime, ypr.roll);

    ypr.pitch = scalePI(filtPitch);
    ypr.roll = scalePI(filtRoll);

    // ------ yaw --------------
    // tilt-compensated yaw
    magTilt.x = mag.x * cos(ypr.pitch) + mag.z * sin(ypr.pitch);

    magTilt.y = mag.x * sin(ypr.roll) * sin(ypr.pitch) +
                mag.y * cos(ypr.roll) -
                mag.z * sin(ypr.roll) * cos(ypr.pitch);

    magTilt.z = -mag.x * cos(ypr.roll) * sin(ypr.pitch) +
                 mag.y * sin(ypr.roll) +
                 mag.z * cos(ypr.roll) * cos(ypr.pitch);

    yaw = atan2(magTilt.y, magTilt.x);
    scaledYaw = scalePI(yaw);
    scaled2Yaw = scalePIangles(scaledYaw, ypr.yaw);

    // Complementary filter
    filtYaw = Complementary2(scaled2Yaw, -gyro.g.z, looptime, ypr.yaw);
    ypr.yaw = scalePI(filtYaw);
  }
  else if (state == IMU_CAL_COM)
  {
    calibrateMagnetometerUpdate();
  }
}

void Imu::printInfo(Stream &s)
{
  Streamprint(s, "imu a=%s%2d.%02d %s%2d.%02d %s%2d.%02d",
              accel.x >= 0.0 ? "+" : "-", abs((int)accel.x), abs((int)(accel.x * 100) - ((int)accel.x * 100)),
              accel.y >= 0.0 ? "+" : "-", abs((int)accel.y), abs((int)(accel.y * 100) - ((int)accel.y * 100)),
              accel.z >= 0.0 ? "+" : "-", abs((int)accel.z), abs((int)(accel.z * 100) - ((int)accel.z * 100)));
  Streamprint(s, " m=%s%2d.%02d %s%2d.%02d %s%2d.%02d",
              mag.x >= 0.0 ? "+" : "-", abs((int)mag.x), abs((int)(mag.x * 100) - ((int)mag.x * 100)),
              mag.y >= 0.0 ? "+" : "-", abs((int)mag.y), abs((int)(mag.y * 100) - ((int)mag.y * 100)),
              mag.z >= 0.0 ? "+" : "-", abs((int)mag.z), abs((int)(mag.z * 100) - ((int)mag.z * 100)));
  Streamprint(s, " g=%4d %4d %4d", gyro.g.x, gyro.g.y, gyro.g.z);
  Streamprint(s, " P=%s%2d.%02d %s%2d.%02d %s%2d.%02d %s%2d.%02d",
              accelPitch >= 0.0 ? "+" : "-", abs((int)accelPitch), abs((int)(accelPitch * 100) - ((int)accelPitch * 100)),
              scaledPitch >= 0.0 ? "+" : "-", abs((int)scaledPitch), abs((int)(scaledPitch * 100) - ((int)scaledPitch * 100)),
              filtPitch >= 0.0 ? "+" : "-", abs((int)filtPitch), abs((int)(filtPitch * 100) - ((int)filtPitch * 100)),
              ypr.pitch >= 0.0 ? "+" : "-", abs((int)ypr.pitch), abs((int)(ypr.pitch * 100) - ((int)ypr.pitch * 100)));
  Streamprint(s, " R=%s%2d.%02d %s%2d.%02d %s%2d.%02d %s%2d.%02d",
              accelRoll >= 0.0 ? "+" : "-", abs((int)accelRoll), abs((int)(accelRoll * 100) - ((int)accelRoll * 100)),
              scaledRoll >= 0.0 ? "+" : "-", abs((int)scaledRoll), abs((int)(scaledRoll * 100) - ((int)scaledRoll * 100)),
              filtRoll >= 0.0 ? "+" : "-", abs((int)filtRoll), abs((int)(filtRoll * 100) - ((int)filtRoll * 100)),
              ypr.roll >= 0.0 ? "+" : "-", abs((int)ypr.roll), abs((int)(ypr.roll * 100) - ((int)ypr.roll * 100)));
  Streamprint(s, " T=%s%2d.%02d %s%2d.%02d %s%2d.%02d",
              magTilt.x >= 0.0 ? "+" : "-", abs((int)magTilt.x), abs((int)(magTilt.x * 100) - ((int)magTilt.x * 100)),
              magTilt.y >= 0.0 ? "+" : "-", abs((int)magTilt.y), abs((int)(magTilt.y * 100) - ((int)magTilt.y * 100)),
              magTilt.z >= 0.0 ? "+" : "-", abs((int)magTilt.z), abs((int)(magTilt.z * 100) - ((int)magTilt.z * 100)));
  Streamprint(s, " Y=%s%2d.%02d %s%2d.%02d %s%2d.%02d\r\n",
              yaw >= 0.0 ? "+" : "-", abs((int)yaw), abs((int)(yaw * 100) - ((int)yaw * 100)),
              scaledYaw >= 0.0 ? "+" : "-", abs((int)scaledYaw), abs((int)(scaledYaw * 100) - ((int)scaledYaw * 100)),
              scaled2Yaw >= 0.0 ? "+" : "-", abs((int)scaled2Yaw), abs((int)(scaled2Yaw * 100) - ((int)scaled2Yaw * 100)),
              filtYaw >= 0.0 ? "+" : "-", abs((int)filtYaw), abs((int)(filtYaw * 100) - ((int)filtYaw * 100)),
              ypr.yaw >= 0.0 ? "+" : "-", abs((int)ypr.yaw), abs((int)(ypr.yaw * 100) - ((int)ypr.yaw * 100)));
}

boolean Imu::init(int aPinBuzzer)
{
  pinBuzzer = aPinBuzzer;
  loadCalibrationData();
  printCalibrationData();
  if (!initGyroscope())
  {
    return false;
  }
  initAccelerometer();
  initMagnetometer();
  hardwareInitialized = true;
  return true;
}

int Imu::getCallCounter(void)
{
  int res = callCounter;
  callCounter = 0;
  return res;
}

int Imu::getErrorCounter(void)
{
  int res = errorCounter;
  errorCounter = 0;
  return res;
}

void Imu::read(void)
{
  if (!hardwareInitialized)
  {
    errorCounter++;
    return;
  }
  callCounter++;
  readGyroscope();
  readAccelerometer();
  readMagnetometer();
}

void Imu::playCompletedSound(void)
{
  tone(pinBuzzer, 600);
  delay(200);
  tone(pinBuzzer, 880);
  delay(200);
  tone(pinBuzzer, 1320);
  delay(200);
}

bool Imu::isTimeToRun(void)
{
  unsigned long curMillis = millis();
  if (use && curMillis >= nextTime)
  {
    nextTime = curMillis + timeBetweenRuns;
    return true;
  }
  return false;
}

bool Imu::isTimeToControl(void)
{
  unsigned long curMillis = millis();
  if (use && curMillis >= nextTimeControl)
  {
    nextTime = curMillis + timeBetweenControl;
    return true;
  }
  return false;
}

float Imu::convertRadToDeg(const float rad)
{
  return rad / PI * 180.0;
}
