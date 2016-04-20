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
    int rpmSet {};
    int pwmMax {};
    float pwmCur {};  // TODO: Move to private
    float senseScale;
    float powerMax {};
    bool swapDir { false };
    int powerIgnoreTime {};
    int zeroSettleTime {};  // TODO: Make it configurable

    void setup(const float acceleration, const int pwmMax, const float powerMax,
               const bool regulate, const int rpmMax, const int rpmSet,
               const float senseScale, const uint8_t pinDir,
               const uint8_t pinPwm, const uint8_t pinSense,
               const uint8_t pinRpm, const uint8_t pinBrake);
    bool readRpmPin();
    int readSensePin();
    float calcCurrent(double alpha);
    float calcPower(float batV);
    void setRpmState();  // call this from hall sensor interrupt
    unsigned long getSamplingTime();
    void setSpeed();
    bool isTimeForRpmMeas(unsigned long* timeSinceLast_p);
    bool isTimeToControl();
    bool isTimeToCheckPower();
    bool isTimeToReadSensor();
    bool isWaitAfterStuckEnd();
    void gotStuck();

    int getRpmMeas() const
    {
      return rpmMeas;
    }

    void setRpmMeas(int rpmMeas)
    {
      this->rpmMeas = rpmMeas;
    }

    int getPwmSet() const
    {
      return pwmSet;
    }

    void setPwmSet(int pwmSet)
    {
      this->pwmSet = pwmSet;
    }

    int getSenseAdc() const
    {
      return senseAdc;
    }

    float getCurrentMeas() const
    {
      return currentMeas;
    }

    float getPowerMeas() const
    {
      return powerMeas;
    }

    int getOverloadCounter() const
    {
      return overloadCounter;
    }

    void incOverloadCounter()
    {
      ++overloadCounter;
    }

    void clearOverloadCounter()
    {
      overloadCounter = 0;
    }

    int getRpmCounter() const
    {
      return rpmCounter;
    }

    void clearRpmCounter()
    {
      rpmCounter = 0;
    }

    unsigned long getZeroTimeout() const
    {
      return zeroTimeout;
    }

    void setZeroTimeout(unsigned long zeroTimeout)
    {
      this->zeroTimeout = zeroTimeout;
    }

  private:
    uint8_t pinDir;
    uint8_t pinPwm;
    uint8_t pinSense;
    uint8_t pinRpm;
    uint8_t pinBrake;
    int rpmMeas {};
    int pwmSet {};
    int senseAdc {};
    float currentMeas {};
    float powerMeas {};
    int overloadCounter {};
    unsigned long lastRpmTime {};
    unsigned long nextTimeRpmMeas {};
    unsigned int timeBetweenRpmMeas { 500 };
    unsigned long nextTimeControl {};
    unsigned int timeBetweenControl { 100 };
    unsigned long nextTimeCheckPower {};
    unsigned int timeBetweenCheckPower { 100 };
    unsigned long nextTimeReadSensor {};
    unsigned int timeBetweenReadSensor { 50 };
    unsigned long lastTimeStucked {};
    unsigned int timeWaitAfterStuck { 30000 };
    unsigned long lastSetSpeedTime {};
    int rpmCounter {};
    bool rpmLastState { false };
    unsigned long zeroTimeout {};

    bool isTimeTo(unsigned long* nextTime_p, const unsigned int timeBetween);
};

#endif /* MOTOR_H */
