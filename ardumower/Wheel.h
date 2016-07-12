/*
 * wheel.h
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#ifndef WHEEL_H
#define WHEEL_H

#include "Motor.h"
#include "MotorShield.h"
#include "Encoder.h"

class Wheel
{
  public:
    typedef enum wheelE
    {
      LEFT,
      RIGHT,
      END
    } wheelE;

    MotorShield motor;
    Encoder encoder;
};

class Wheels
{
  public:
    Wheel wheel[Wheel::END];
    int rollTimeMax {};         // max. roll time (ms)
    int rollTimeMin {};         // min. roll time (ms)
    int reverseTime {};         // max. reverse time (ms)
    unsigned long forwardTimeMax {};     // max. forward time (ms) / timeout
    float biDirSpeedRatio1 {};  // bidir mow pattern speed ratio 1
    float biDirSpeedRatio2 {};  // bidir mow pattern speed ratio 2
    Wheel::wheelE rotateDir {Wheel::LEFT};

    bool isTimeToRotationChange(void);

    void stop(void);
    void slowDown(void);
    void speedUp(void);
    void forwardFullSpeed(void);
    void forwardHalfSpeed(void);
    void reverseFullSpeed(void);
    void reverseFastSpeed(void);
    void reverseSlowSpeed(void);
    void rollFullRight(void);
    void rollFullLeft(void);
    void rollFull(bool dir);
    void rollFastRight(void);
    void rollFastLeft(void);
    void rollFast(bool dir);
    void rollHalfRight(void);
    void rollHalfLeft(void);
    void rollHalf(bool dir);
    void rollSlowRight(void);
    void rollSlowLeft(void);
    void rollSlow(bool dir);

  private:
    static const uint16_t TIME_BETWEEN_ROTATION_CHANGE {60000};

    unsigned long nextTimeRotationChange {};
};

#endif /* WHEEL_H */
