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

/* pitch/roll and heading estimation (IMU sensor fusion)
 requires: GY-80 module (L3G4200D, ADXL345B, HMC5883L)

 How to use it (example):
 1. initialize IMU:                 IMU imu;  imu.init();
 2. read IMU (yaw/pitch/roll:       Serial.println( imu.ypr.yaw );
 */
#pragma once

#include <Arduino.h>
#include "Pid.h"
#include "L3G.h"
#include "LSM303.h"

// IMU state
enum
{
  IMU_RUN,
  IMU_CAL_COM
};

typedef struct point_int_t
{
  int x;
  int y;
  int z;
} point_int_t;

typedef struct point_longt_t
{
  long x;
  long y;
  long z;
} point_long_t;

typedef struct point_float_t
{
  float x;
  float y;
  float z;
} point_float_t;

typedef struct ypr_t
{
  float yaw;
  float pitch;
  float roll;
} ypr_t;

typedef struct calibrationData_t
{
  point_float_t accelOffset;
  point_float_t accelScale;
  point_int_t magnetometerOffset;
  point_int_t magnetometerScale;
} calibrationData_t;

class Imu
{
public:
  enum imuPidE
  {
    DIR,
    ROLL,
    END
  };

  bool init(int aPinBuzzer);
  void update(void);
  int getCallCounter(void);
  int getErrorCounter(void);
  void deleteCalibrationData(void);
  bool isCalibrationAvailable(void) const
  {
    return m_calibrationAvailable;
  }
  void printInfo(Stream& s);

  bool m_use { false };
  bool m_correctDir { false };  // correct direction by magnetometer?
  Pid m_pid[END];             // direction and roll PID controllers

  float getYaw() const
  {
    return m_ypr.yaw;
  }
  float getYawDeg()
  {
    return degrees(m_ypr.yaw);
  }

  float getPitch() const
  {
    return m_ypr.pitch;
  }
  float getPitchDeg()
  {
    return degrees(m_ypr.pitch);
  }

  float getRoll() const
  {
    return m_ypr.roll;
  }
  float getRollDeg()
  {
    return degrees(m_ypr.roll);
  }

  // --------- gyro state -----------------------------
  L3G m_gyro;

  // calibrate acceleration sensor
  bool calibrateAccelerometerNextAxis();

  // --------- accelerometer/magnetometer state -------
  point_float_t m_acc { 0.0, 0.0, 0.0 };
  point_float_t m_mag { 0.0, 0.0, 0.0 };
  bool getUseAccelCalibration(void) const
  {
    return m_useAccelCalibration;
  }
  void toggleUseAccelCalibration(void)
  {
    m_useAccelCalibration = !m_useAccelCalibration;
  }

  // calibrate magnetometer sensor
  void calibrateMagnetometerStartStop(void);

  // --------------------------------------------------

private:
  void read();
  void calibrateGyro(void);
  void loadCalibrationData(void);

  // print IMU values
  void printPoint(const point_int_t point);
  void printPointln(const point_int_t point);
  void printPoint(const point_float_t point);
  void printPointln(const point_float_t point);
  void printCalibrationData(void);
  void saveCalibrationData(void);

  // hardware
  void initAccelerometer(void);
  bool initGyroscope(void);
  void initMagnetometer(void);

  void readGyroscope(void);
  void readAccelerometer(void);
  void readMagnetometer(void);

  void calibrateMagnetometerUpdate(void);
  void playCompletedSound();

  // helpers
  float scalePI(const float v);
  float scale180(const float v);
  float scalePIangles(const float setAngle, const float currAngle);
  float distancePI(const float x, const float w);
  float distance180(const float x, const float w);
  float fusionPI(const float w, const float a, const float b);

  // Filter
  float Complementary(const float newAngle, const float newRate,
      const int looptime, float angle);
  float Complementary2(const float newAngle, const float newRate,
      const int looptime, float angle);
  float Kalman(const float newAngle, const float newRate, const int looptime,
      float x_angle);

  unsigned long m_nextTime {};
  unsigned long m_nextTimeControl {};
  unsigned int m_timeBetweenRuns { 200 }; // 5 Hz
  unsigned int m_timeBetweenControl { 100 }; // 10 Hz

  bool m_foundNewMinMax { false };
  int m_pinBuzzer {};
  int m_callCounter {};
  int m_errorCounter {};
  bool m_hardwareInitialized { false };
  bool m_calibrationAvailable { false };
  byte m_state { IMU_RUN };
  unsigned long m_lastAHRSTime {};
  calibrationData_t m_calibrationData
  {
    { 0, 0, 0 },
    { 1, 1, 1 },
    { 0, 0, 0 },
    { 1, 1, 1 }
  };

  // --------- acceleration state ---------------------
  LSM303 m_accMag;
  point_float_t m_accMin { 0.0, 0.0, 0.0 };
  point_float_t m_accMax { 0.0, 0.0, 0.0 };
  int m_calibAccelAxisCounter {};
  bool m_useAccelCalibration { true };

  // --------- gyro state -----------------------------
  point_int_t m_gyroOffset { 0, 0, 0 }; // gyro calibration data
  int m_gyroNoise {};          // gyro noise
  bool m_useGyroCalibration { true }; // gyro calibration flag
  ypr_t m_ypr { 0.0, 0.0, 0.0 };  // gyro yaw,pitch,roll

  // --------- magnetometer state --------------------------
  point_int_t m_magLast { 0, 0, 0 };
  point_int_t m_magMin { 0, 0, 0 }; // magnetometer sensor data (raw)
  point_int_t m_magMax { 0, 0, 0 }; // magnetometer sensor data (raw)
  bool m_useMagnetometerCalibration { true };

  float m_accPitch {};
  float m_accRoll {};
  float m_scaledPitch {};
  float m_scaledRoll {};
  float m_filtPitch {};
  float m_filtRoll {};
  float m_yaw {};
  float m_scaledYaw {};
  float m_scaled2Yaw {};
  float m_filtYaw {};
  point_float_t m_magTilt { 0.0, 0.0, 0.0 };
};
