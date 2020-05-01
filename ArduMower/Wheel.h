/*
 * wheel.h
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */
#pragma once

#include "Motor.h"
#include "MotorShield.h"
#include "Encoder.h"

class Wheel
{
  public:
    enum WheelE
    {
      LEFT,
      RIGHT,
      END
    };

    MotorShield m_motor;
    Encoder m_encoder;

    void control(int8_t speed);
};

class Wheels
{
  public:
    bool isTimeToRotationChange(void);
    void control(void);

    int8_t getSpeed() const
    {
      return m_speed;
    }

    void setSpeed(int8_t speed)
    {
      m_speed = speed;
    }

    int8_t getSteer() const
    {
      return m_steer;
    }

    void setSteer(int8_t steer)
    {
      m_steer = steer;
    }

public:
    Wheel m_wheel[Wheel::END];
    int16_t m_rollTimeMax {};         // max. roll time (ms)
    int16_t m_rollTimeMin {};         // min. roll time (ms)
    int16_t m_reverseTime {};         // max. reverse time (ms)
    uint32_t m_forwardTimeMax {};     // max. forward time (ms) / timeout
    float m_biDirSpeedRatio1 {};  // bidir mow pattern speed ratio 1
    float m_biDirSpeedRatio2 {};  // bidir mow pattern speed ratio 2
    Wheel::WheelE m_rotateDir { Wheel::LEFT };

  private:
    const uint16_t TIME_BETWEEN_ROTATION_CHANGE_ms { 60000 };

    unsigned long m_nextTimeRotationChange {};
    int8_t m_speed {}; // Range -100..+100, - = reverse, + = forward
    int8_t m_steer {}; // Range -100..+100, - = left, + = right
};
