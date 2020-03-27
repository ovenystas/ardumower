/*
 * MotorDrv.h
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

#ifndef MOTORDRV_H
#define MOTORDRV_H

#include <Arduino.h>
#include "Motor.h"
#include "Filter.h"

class MotorDrv: public Motor
{
  public:
    Pid m_pid;
    virtual ~MotorDrv() {};
    virtual void setup(void) = 0;
    virtual void setSpeed(void) = 0;
    virtual void setSpeed(const int16_t speed) = 0;
    virtual void setSpeed(const int16_t speed, bool brake) = 0;
    virtual void readCurrent(void) = 0;

    // Get average ADC value 0..1023
    int16_t getAverageSenseAdc(void) const
    {
      return FilterEmaI16_getAverage(&m_filter);
    }

    int16_t getAverageCurrent(void);   // Get average motor current in mA
    void calcPower(float batV);

    void setFilterAlpha(double alpha)
    {
      FilterEmaI16_setAlpha(alpha, &m_filter);
    }

    void setChannel(uint8_t channel)
    {
      m_channel = channel;
    }

    double getScale(void) const
    {
      return m_scale;
    }

    void setScale(double scale)
    {
      m_scale = scale;
    }

    // 5.0V Ref. Max=1023. 10 times amplification.
    // 1A measured over 0,15ohm => 1 * 0,15 * 10 = 1,5V.
    // Full-scale = 5,0 / 1,5 = 3,33A
    // Current/bit = 3,33 / 1023 = 0,00325839A/bit = 3,25839mA/bit
    double m_scale {3.25839};  // TODO: Move to protected

  protected:
    uint8_t m_channel {};
    FilterEmaI16 m_filter;
};

#endif /* MOTORDRV_H */