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

class Wheel
{
  public:
    enum wheelE
    {
      LEFT,
      RIGHT,
      END
    };

    MotorShield motor;
};

class Wheels
{
  public:
    Wheel wheel[Wheel::END];
    int rollTimeMax {};         // max. roll time (ms)
    int rollTimeMin {};         // min. roll time (ms)
    int reverseTime {};         // max. reverse time (ms)
    long forwardTimeMax {};     // max. forward time (ms) / timeout
    float biDirSpeedRatio1 {};  // bidir mow pattern speed ratio 1
    float biDirSpeedRatio2 {};  // bidir mow pattern speed ratio 2
    Wheel::wheelE rotateDir {Wheel::LEFT};

    bool isTimeToRotationChange(void);

  private:
    static const uint16_t TIME_BETWEEN_ROTATION_CHANGE {60000};

    unsigned long nextTimeRotationChange {};
};

#endif /* WHEEL_H */
