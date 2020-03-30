/*
 * motor.h
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

// TODO: Move functions from Robot.cpp to this class
#pragma once

#include <Arduino.h>
#include "Pid.h"

class Motor
{
  public:
    float m_acceleration {};
    bool m_regulate { false };
    int16_t m_rpmMax {};
    int16_t m_rpmSet {};
    int16_t m_pwmMax {};
    int16_t m_pwmCur {};  // TODO: Move to private
    int16_t m_powerMax {};
    bool m_swapDir { false };
    int16_t m_powerIgnoreTime {};
    int16_t m_zeroSettleTime {};  // TODO: Make it configurable

    void config(const float acceleration, const int16_t pwmMax,
                const int16_t powerMax, const bool regulate,
                const int16_t rpmMax, const int16_t rpmSet);

    bool readRpmPin(void)
    {
      return digitalRead(m_pinRpm);
    }

    int16_t readSensePin(void);
    void setRpmState(void);  // call this from hall sensor interrupt
    uint16_t getSamplingTime(void);
    bool isTimeForRpmMeas(unsigned long* timeSinceLast_p);

    bool isTimeToControl(void)
    {
      return isTimeTo(&m_nextTimeControl, TIME_BETWEEN_CONTROL);
    }

    bool isTimeToCheckPower(void)
    {
      return isTimeTo(&m_nextTimeCheckPower, TIME_BETWEEN_CHECK_POWER);
    }

    bool isTimeToReadCurrent(void)
    {
      return isTimeTo(&m_nextTimeReadSensor, TIME_BETWEEN_READ_CURRENT);
    }

    bool isWaitAfterStuckEnd(void)
    {
      return (millis() >= m_lastTimeStucked + TIME_WAIT_AFTER_STUCK);
    }

    void gotStuck(void)
    {
      m_lastTimeStucked = millis();
    }

    int16_t getRpmMeas(void) const
    {
      return m_rpmMeas;
    }

    void setRpmMeas(int16_t rpmMeas)
    {
      m_rpmMeas = rpmMeas;
    }

    int16_t getPwmSet(void) const
    {
      return m_pwmSet;
    }

    void setPwmSet(int16_t pwmSet)
    {
      m_pwmSet = pwmSet;
    }

    int16_t getPowerMeas(void) const
    {
      return m_powerMeas;
    }

    uint16_t getOverloadCounter(void) const
    {
      return m_overloadCounter;
    }

    void incOverloadCounter(void)
    {
      ++m_overloadCounter;
    }

    void clearOverloadCounter(void)
    {
      m_overloadCounter = 0;
    }

    int16_t getRpmCounter(void) const
    {
      return m_rpmCounter;
    }

    void clearRpmCounter(void)
    {
      m_rpmCounter = 0;
    }

    uint16_t getZeroTimeout(void) const
    {
      return m_zeroTimeout;
    }

    void setZeroTimeout(uint16_t zeroTimeout)
    {
      m_zeroTimeout = zeroTimeout;
    }

    bool isOverpowered(void)
    {
      return m_powerMeas > m_powerMax;
    }

    void updateRpms(void);

    int16_t getRpmSet() const
    {
      return m_rpmSet;
    }
    void setRpmSet(int16_t rpmSet)
    {
      m_rpmSet = rpmSet;
    }

    int16_t getPwmCur() const
    {
      return m_pwmCur;
    }
    void setPwmCur(int16_t pwmCur)
    {
      m_pwmCur = pwmCur;
    }

  protected:
    int16_t m_powerMeas {};
    int16_t m_pwmSet {};
    int16_t m_rpmMeas {};

  private:
    static const uint16_t TIME_BETWEEN_RPM_MEAS { 500 };
    static const uint8_t TIME_BETWEEN_CONTROL { 100 };
    static const uint8_t TIME_BETWEEN_CHECK_POWER { 100 };
    static const uint8_t TIME_BETWEEN_READ_CURRENT { 50 };
    static const uint16_t TIME_WAIT_AFTER_STUCK { 30000 };

    static constexpr float RPM_DIVISOR_FAST { 1.25 };
    static constexpr float RPM_DIVISOR_HALF { 1.5 };
    static constexpr float RPM_DIVISOR_SLOW { 2.0 };

    uint8_t m_pinDir;
    uint8_t m_pinPwm;
    uint8_t m_pinSense;
    uint8_t m_pinRpm;
    uint8_t m_pinBrake;
    uint16_t m_overloadCounter {};
    uint32_t m_lastRpmTime {};
    uint32_t m_nextTimeRpmMeas {};
    uint32_t m_nextTimeControl {};
    uint32_t m_nextTimeCheckPower {};
    uint32_t m_nextTimeReadSensor {};
    uint32_t m_lastTimeStucked {};
    uint32_t m_lastSetSpeedTime {};
    uint16_t m_rpmCounter {};
    bool m_rpmLastState { false };
    uint16_t m_zeroTimeout {};

    bool isTimeTo(uint32_t* nextTime_p, const uint16_t timeBetween);
};
