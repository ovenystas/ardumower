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
#include "Complementary2.h"
#include "Kalman.h"

#ifndef TEST_VIRTUAL
#define TEST_VIRTUAL
#endif

struct YawPitchRoll_t
{
  float yaw {};
  float pitch {};
  float roll {};
};

struct CalibrationData_t
{
  Vector<float> accelOffset;
  Vector<float> accelScale;
  Vector<int16_t> magnetometerOffset;
  Vector<int16_t> magnetometerScale;
};

struct ImuSettings
{
  Setting<bool> use;
  Setting<bool> correctDir;
};

class Imu
{
public:
  Imu() {};
  TEST_VIRTUAL ~Imu() {};

  bool init(Buzzer* buzzer_p);

  TEST_VIRTUAL bool isUsed();

  bool isCorrectDir()
  {
    return m_correctDir;
  }

  void update(void);

  uint16_t getAndClearCallCounter(void);
  uint16_t getAndClearErrorCounter(void);

  void deleteCalibrationData(void);

  bool isCalibrationAvailable(void) const
  {
    return m_calibrationAvailable;
  }

  void printInfo(Stream& s);

  TEST_VIRTUAL float getYaw() const;

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

  bool calibrateAccelerometerNextAxis();

  bool getUseAccelCalibration(void) const
  {
    return m_useAccCalibration;
  }

  void toggleUseAccelCalibration(void)
  {
    m_useAccCalibration = !m_useAccCalibration;
  }

  void calibrateMagnetometerStartStop(void);

  ImuSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(ImuSettings* settings_p)
  {
    m_settings.use.value = settings_p->use.value;
    m_settings.correctDir.value = settings_p->correctDir.value;
  }

public:
  Pid m_pidDir;
  Pid m_pidRoll;

  L3G m_gyro;

  Vector<float> m_acc {};
  Vector<float> m_mag {};

private:
  enum ImuStateE
  {
    RUN,
    CALIBRATE_MAG
  };

  void initAccelerometer(void);
  bool initGyroscope(void);
  void initMagnetometer(void);

  void read();
  void readGyroscope(void);
  void readAccelerometer(void);
  void readMagnetometer(void);

  void loadCalibrationData(void);
  void saveCalibrationData(void);
  void printCalibrationData(void);
  void calibrateGyro(void);
  void calibrateMagnetometerUpdate(void);
  void playCompletedSound();

  template <class T> void printPointln(const Vector<T>& point);
  template <class T> void printPointMinAvgMax(
      Vector<T>& v_min, Vector<T>& v_avg, Vector<T>& v_max);
  template <class T> void printPointMinMax(Vector<T>& v_min, Vector<T>& v_max);

  float scalePIangles(const float setAngle, const float currAngle);

private:
  const BeepData m_completedSound[3] =
  {
      { 600, 200 },
      { 880, 200 },
      { 1320, 200 },
  };

  Complementary2 comp2Filter;
  Kalman kalmanPitch;
  Kalman kalmanRoll;
  bool m_foundNewMinMax {};
  Buzzer* m_buzzer_p {};
  uint16_t m_callCounter {};
  uint16_t m_errorCounter {};
  bool m_hardwareInitialized {};
  bool m_calibrationAvailable {};
  ImuStateE m_state { ImuStateE::RUN };
  uint32_t m_lastAhrsTime {};

  CalibrationData_t m_calibrationData
  {
    { 0, 0, 0 },
    { 1, 1, 1 },
    { 0, 0, 0 },
    { 1, 1, 1 }
  };

  // --------- Accelerometer state ---------------------------------------------
  LSM303 m_accMag;
  Vector<float> m_accMin {};
  Vector<float> m_accMax {};
  uint8_t m_calibAccAxisCounter {};
  bool m_useAccCalibration { true };

  // --------- Gyroscope state -------------------------------------------------
  Vector<int16_t> m_gyroOffset {}; // gyro calibration data
  int16_t m_gyroNoise {};          // gyro noise
  bool m_useGyroCalibration { true }; // gyro calibration flag
  YawPitchRoll_t m_ypr {};  // gyro yaw, pitch, roll

  // --------- Magnetometer state ----------------------------------------------
  Vector<int16_t> m_magLast {};
  Vector<int16_t> m_magMin {}; // magnetometer sensor data (raw)
  Vector<int16_t> m_magMax {}; // magnetometer sensor data (raw)
  bool m_useMagCalibration { true };

  float m_yaw {};
  float m_scaledYaw {};
  float m_scaled2Yaw {};
  float m_filtYaw {};

  float m_accPitch {};
  float m_scaledPitch {};
  float m_filtPitch {};

  float m_accRoll {};
  float m_scaledRoll {};
  float m_filtRoll {};

  Vector<float> m_magTilt {};

  // --------- Settings --------------------------------------------------------
  ImuSettings m_settings
  {
    { "Use", false },
    { "Correct dir", false } // Correct direction by magnetometer?
  };

  // Shorter convenient variables for settings variables
  bool& m_use = m_settings.use.value;
  bool& m_correctDir = m_settings.correctDir.value;
};
