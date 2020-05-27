/*
 * DifferentialDrive.h
 *
 *  Created on: May 25, 2020
 *      Author: ove
 */

#pragma once

#include <Arduino.h>
#include "Wheel.h"

class DifferentialDrive
{
public:
  enum SideE
  {
    LEFT,
    RIGHT
  };

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

  Wheel* getWheel(SideE side);

  SideE getRotateDir() const
  {
    return m_rotateDir;
  }

  void toggleRotateDir();

public:
  Wheel m_wheelLeft;
  Wheel m_wheelRight;
  uint16_t m_rollTimeMax {};         // max. roll time (ms)
  uint16_t m_rollTimeMin {};         // min. roll time (ms)
  uint16_t m_reverseTime {};         // max. reverse time (ms)
  uint32_t m_forwardTimeMax {};     // max. forward time (ms) / timeout
  float m_biDirSpeedRatio1 {};      // bidir mow pattern speed ratio 1
  float m_biDirSpeedRatio2 {};      // bidir mow pattern speed ratio 2

private:
  const uint16_t TIME_BETWEEN_ROTATION_CHANGE_ms { 60000 };

  uint32_t m_lastTimeRotationChange {};
  int8_t m_speed {}; // Range -100..+100, - = reverse, + = forward
  int8_t m_steer {}; // Range -100..+100, - = left, + = right
  SideE m_rotateDir { SideE::LEFT };
};
