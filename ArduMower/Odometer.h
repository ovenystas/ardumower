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

typedef struct encoderS
{
  Encoder* left_p { nullptr };
  Encoder* right_p { nullptr };
} encoderS;

class Odometer
{
  public:
    enum odometerE
    {
      LEFT,
      RIGHT,
      END
    };

    void setup(const int ticksPerRevolution,
               const float ticksPerCm,
               const float wheelBaseCm,
               Encoder* encoderLeft,
               Encoder* encoderRight,
               Imu* imu);
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

    bool m_use { false };      // Use this odometer or not
    int m_ticksPerRevolution;  // encoder ticks per one full resolution
    float m_ticksPerCm;        // encoder ticks per cm
    float m_wheelBaseCm;       // wheel-to-wheel distance (cm)
    encoderS m_encoder;
    Imu* m_imu_p;

  private:
    unsigned long m_lastWheelRpmTime {};  // last time it was updated
    int16_t m_lastOdoLeft {};
    int16_t m_lastOdoRight {};
    float m_theta {};                     // theta angle (radiant)
    float m_x {};                         // X map position (cm)
    float m_y {};                         // Y map position (cm)
};
