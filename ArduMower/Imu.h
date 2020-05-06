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
#include "Buzzer.h"
#include "Setting.h"
#include "Vector.h"

// IMU state
enum
{
  IMU_RUN,
  IMU_CAL_COM
};

struct YawPitchRoll_t
{
  float yaw;
  float pitch;
  float roll;
};

typedef struct calibrationData_t
{
  Vector<float> accelOffset;
  Vector<float> accelScale;
  Vector<int16_t> magnetometerOffset;
  Vector<int16_t> magnetometerScale;
} calibrationData_t;

struct ImuSettings
{
  Setting<bool> use;
  Setting<bool> correctDir;
};

class Imu
{
public:
  Imu() {};

  enum imuPidE
  {
    DIR,
    ROLL,
    END
  };

  bool init(Buzzer* buzzer_p);

  bool isUsed()
  {
    return m_use;
  }
  bool isCorrectDir()
  {
    return m_correctDir;
  }

  void update(void);
  int16_t getCallCounter(void);
  int16_t getErrorCounter(void);
  void deleteCalibrationData(void);
  bool isCalibrationAvailable(void) const
  {
    return m_calibrationAvailable;
  }
  void printInfo(Stream& s);

  Pid m_pid[END];             // direction and roll PID controllers

  float getYaw() const
  {
    return m_ypr.yaw;
  }

  float getYawDeg()
  {
    return degrees(m_ypr.yaw);
  }

  int16_t getYawDegInt()
  {
    return static_cast<int16_t>(round(degrees(m_ypr.yaw)));
  }

  float getPitch() const
  {
    return m_ypr.pitch;
  }

  float getPitchDeg()
  {
    return degrees(m_ypr.pitch);
  }

  int16_t getPitchDegInt()
  {
    return static_cast<int16_t>(round(degrees(m_ypr.pitch)));
  }

  float getRoll() const
  {
    return m_ypr.roll;
  }

  float getRollDeg()
  {
    return degrees(m_ypr.roll);
  }

  int16_t getRollDegInt()
  {
    return static_cast<int16_t>(round(degrees(m_ypr.roll)));
  }

  // --------- gyro state -----------------------------
  L3G m_gyro;

  // calibrate acceleration sensor
  bool calibrateAccelerometerNextAxis();

  // --------- accelerometer/magnetometer state -------
  Vector<float> m_acc {};
  Vector<float> m_mag {};
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

  ImuSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(ImuSettings* settings_p)
  {
    m_settings.use.value = settings_p->use.value;
    m_settings.correctDir.value = settings_p->correctDir.value;
  }

private:
  void read();
  void calibrateGyro(void);
  void loadCalibrationData(void);

  // print IMU values
  void printPoint(const Vector<int16_t> point);
  void printPointln(const Vector<int16_t> point);
  void printPoint(const Vector<float> point);
  void printPointln(const Vector<float> point);
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
      const int16_t looptime, float angle);
  float Complementary2(const float newAngle, const float newRate,
      const int16_t looptime, float angle);
  float Kalman(const float newAngle, const float newRate, const int16_t looptime,
      float x_angle);

  uint32_t m_nextTime {};
  uint32_t m_nextTimeControl {};
  uint16_t m_timeBetweenRuns { 200 }; // 5 Hz
  uint16_t m_timeBetweenControl { 100 }; // 10 Hz

  bool m_foundNewMinMax {};
  Buzzer* m_buzzer_p;
  int16_t m_callCounter {};
  int16_t m_errorCounter {};
  bool m_hardwareInitialized {};
  bool m_calibrationAvailable {};
  uint8_t m_state { IMU_RUN };
  uint32_t m_lastAHRSTime {};
  calibrationData_t m_calibrationData
  {
    { 0, 0, 0 },
    { 1, 1, 1 },
    { 0, 0, 0 },
    { 1, 1, 1 }
  };

  // --------- acceleration state ---------------------
  LSM303 m_accMag;
  Vector<float> m_accMin {};
  Vector<float> m_accMax {};
  int16_t m_calibAccelAxisCounter {};
  bool m_useAccelCalibration { true };

  // --------- gyro state -----------------------------
  Vector<int16_t> m_gyroOffset {}; // gyro calibration data
  int16_t m_gyroNoise {};          // gyro noise
  bool m_useGyroCalibration { true }; // gyro calibration flag
  YawPitchRoll_t m_ypr { 0.0, 0.0, 0.0 };  // gyro yaw,pitch,roll

  // --------- magnetometer state --------------------------
  Vector<int16_t> m_magLast {};
  Vector<int16_t> m_magMin {}; // magnetometer sensor data (raw)
  Vector<int16_t> m_magMax {}; // magnetometer sensor data (raw)
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
  Vector<float> m_magTilt {};

  ImuSettings m_settings
  {
    { "Use", false },
    { "Correct dir", false } // Correct direction by magnetometer?
  };

  // Shorter convenient variables for settings variables
  bool& m_use = m_settings.use.value;
  bool& m_correctDir = m_settings.correctDir.value;
};
