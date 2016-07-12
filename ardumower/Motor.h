/*
 * motor.h
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

// TODO: Move functions from Robot.cpp to this class

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Pid.h"

class Motor
{
  public:
    Pid pid;
    float acceleration {};
    bool regulate { false };
    int rpmMax {};
    int rpmFast {};
    int rpmHalf {};
    int rpmSlow {};
    int rpmSet {};
    uint8_t pwmMax {};
    int16_t pwmCur {};  // TODO: Move to private
    int powerMax {};
    bool swapDir { false };
    int powerIgnoreTime {};
    int zeroSettleTime {};  // TODO: Make it configurable

    void config(const float acceleration, const int pwmMax, const int powerMax,
                const bool regulate, const int rpmMax, const int rpmSet);

    bool readRpmPin(void)
    {
      return digitalRead(pinRpm);
    }

    int readSensePin(void);
    void setRpmState(void);  // call this from hall sensor interrupt
    unsigned long getSamplingTime(void);
    bool isTimeForRpmMeas(unsigned long* timeSinceLast_p);

    bool isTimeToControl(void)
    {
      return isTimeTo(&nextTimeControl, TIME_BETWEEN_CONTROL);
    }

    bool isTimeToCheckPower(void)
    {
      return isTimeTo(&nextTimeCheckPower, TIME_BETWEEN_CHECK_POWER);
    }

    bool isTimeToReadCurrent(void)
    {
      return isTimeTo(&nextTimeReadSensor, TIME_BETWEEN_READ_CURRENT);
    }

    bool isWaitAfterStuckEnd(void)
    {
      return (millis() >= lastTimeStucked + TIME_WAIT_AFTER_STUCK);
    }

    void gotStuck(void)
    {
      lastTimeStucked = millis();
    }

    const int getRpmMeas(void) const
    {
      return rpmMeas;
    }

    void setRpmMeas(int rpmMeas)
    {
      this->rpmMeas = rpmMeas;
    }

    const int getPwmSet(void) const
    {
      return pwmSet;
    }

    void setPwmSet(int pwmSet)
    {
      this->pwmSet = pwmSet;
    }

    const int getPowerMeas(void) const
    {
      return powerMeas;
    }

    const uint16_t getOverloadCounter(void) const
    {
      return overloadCounter;
    }

    void incOverloadCounter(void)
    {
      ++overloadCounter;
    }

    void clearOverloadCounter(void)
    {
      overloadCounter = 0;
    }

    const int getRpmCounter(void) const
    {
      return rpmCounter;
    }

    void clearRpmCounter(void)
    {
      rpmCounter = 0;
    }

    const unsigned long getZeroTimeout(void) const
    {
      return zeroTimeout;
    }

    void setZeroTimeout(unsigned long zeroTimeout)
    {
      this->zeroTimeout = zeroTimeout;
    }

    bool isOverpowered(void)
    {
      return powerMeas > powerMax;
    }

    void updateRpms(void);

    const int getRpmSet() const
    {
      return rpmSet;
    }
    void setRpmSet(int rpmSet)
    {
      this->rpmSet = rpmSet;
    }

    const uint8_t getPwmCur() const
    {
      return pwmCur;
    }
    void setPwmCur(uint8_t pwmCur)
    {
      this->pwmCur = pwmCur;
    }

  protected:
    int16_t powerMeas {};
    int pwmSet {};
    int rpmMeas {};

  private:
    static const uint16_t TIME_BETWEEN_RPM_MEAS { 500 };
    static const uint8_t TIME_BETWEEN_CONTROL { 100 };
    static const uint8_t TIME_BETWEEN_CHECK_POWER { 100 };
    static const uint8_t TIME_BETWEEN_READ_CURRENT { 50 };
    static const uint16_t TIME_WAIT_AFTER_STUCK { 30000 };

    static constexpr float RPM_DIVISOR_FAST { 1.25 };
    static constexpr float RPM_DIVISOR_HALF { 1.5 };
    static constexpr float RPM_DIVISOR_SLOW { 2.0 };

    uint8_t pinDir;
    uint8_t pinPwm;
    uint8_t pinSense;
    uint8_t pinRpm;
    uint8_t pinBrake;
    uint16_t overloadCounter {};
    uint32_t lastRpmTime {};
    uint32_t nextTimeRpmMeas {};
    uint32_t nextTimeControl {};
    uint32_t nextTimeCheckPower {};
    uint32_t nextTimeReadSensor {};
    uint32_t lastTimeStucked {};
    uint32_t lastSetSpeedTime {};
    uint16_t rpmCounter {};
    bool rpmLastState { false };
    uint32_t zeroTimeout {};

    bool isTimeTo(uint32_t* nextTime_p, const uint16_t timeBetween);
};

#endif /* MOTOR_H */
