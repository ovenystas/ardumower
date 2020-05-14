/*
 * odometer.h
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>
#include "Encoder.h"
#include "Imu.h"
#include "Setting.h"
#include "Config.h"

struct EncoderS
{
  Encoder* left_p { nullptr };
  Encoder* right_p { nullptr };
};

struct OdometerSettings
{
  Setting<bool> use;                // Use this odometer or not
  Setting<int> ticksPerRevolution;  // Encoder ticks per one full resolution
  Setting<float> ticksPerCm;        // Encoder ticks per cm
  Setting<float> wheelBaseCm;       // Wheel-to-wheel distance (cm)
};

class Odometer
{
public:
  enum odometerE
  {
    LEFT,
    RIGHT,
    END
  };

  Odometer(Encoder* encoderLeft_p, Encoder* encoderRight_p, Imu* imu_p) :
    m_imu_p(imu_p)
  {
    m_encoder.left_p = encoderLeft_p;
    m_encoder.right_p = encoderRight_p;
  };

  bool isUsed()
  {
    return m_use;
  }

  void calc(void);
  void read(void);

  float getX() const
  {
    return m_x;
  }

  float getY() const
  {
    return m_y;
  }

  OdometerSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(OdometerSettings* settings_p)
  {
    m_settings.use.value = settings_p->use.value;
    m_settings.ticksPerRevolution.value = settings_p->ticksPerRevolution.value;
    m_settings.ticksPerCm.value = settings_p->ticksPerCm.value;
    m_settings.wheelBaseCm.value = settings_p->wheelBaseCm.value;
  }

  EncoderS m_encoder;
  Imu* m_imu_p;

private:
  unsigned long m_lastWheelRpmTime {};  // last time it was updated
  int16_t m_lastOdoLeft {};
  int16_t m_lastOdoRight {};
  float m_theta {};                     // theta angle (radiant)
  float m_x {};                         // X map position (cm)
  float m_y {};                         // Y map position (cm)

  OdometerSettings m_settings
  {
    { "Use", false },
    { "Ticks per revolution", "", ODOMETER_TICKS_PER_REVOLUTION, 1, 2000 },
    { "Ticks per cm", "", ODOMETER_TICKS_PER_CM, 0.1, 30.0, 0.1 },
    { "Wheel base", "cm", ODOMETER_WHEELBASE_CM, 0.1, 50.0, 0.1 }
  };

  // Shorter convenient variables for settings variables
  bool& m_use = m_settings.use.value;
  int& m_ticksPerRevolution = m_settings.ticksPerRevolution.value;
  float& m_ticksPerCm = m_settings.ticksPerCm.value;
  float& m_wheelBaseCm = m_settings.wheelBaseCm.value;
};
