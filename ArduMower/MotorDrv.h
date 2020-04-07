/*
 * MotorDrv.h
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>
#include "Motor.h"
#include "Filter.h"

class MotorDrv: public Motor
{
public:
  virtual ~MotorDrv() {};
  virtual void setup() = 0;
  virtual void setSpeed() = 0;
  virtual void setSpeed(int16_t speed) = 0;
  virtual void setSpeed(int16_t speed, bool brake) = 0;
  virtual void readCurrent() = 0;

  // Get average ADC value 0..1023
  int16_t getAverageSenseAdc()
  {
    return m_filter.getAverage();
  }

  int16_t getAverageCurrent();   // Get average motor current in mA
  void calcPower(float batV);

  void setFilterAlpha(float alpha)
  {
    m_filter.setAlpha(alpha);
  }

  float getFilterAlpha()
  {
    return m_filter.getAlpha();
  }

  void setChannel(uint8_t channel)
  {
    m_channel = channel;
  }

  float getScale()
  {
    return m_scale;
  }

  void setScale(float scale)
  {
    m_scale = scale;
  }

public:
  Pid m_pid;

  // 5.0V Ref. Max=1023. 10 times amplification.
  // 1A measured over 0,15ohm => 1 * 0,15 * 10 = 1,5V.
  // Full-scale = 5,0 / 1,5 = 3,33A
  // Current/bit = 3,33 / 1023 = 0,00325839A/bit = 3,25839mA/bit
  float m_scale { 3.25839 };  // TODO: Move to protected

protected:
  uint8_t m_channel {};
  FilterEmaI16 m_filter { 1.0 }; // Default 1.0 is no filtering
};
