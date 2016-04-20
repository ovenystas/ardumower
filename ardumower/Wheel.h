/*
 * wheel.h
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#ifndef WHEEL_H
#define WHEEL_H

#include "Motor.h"

class Wheel
{
  public:
    enum wheelE
    {
      LEFT,
      RIGHT,
      END
    };

    Motor motor;
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

    bool isTimeToRotationChange();

  private:
    unsigned long nextTimeRotationChange {};
    unsigned int timeBetweenRotationChange { 60000 };
};


#endif /* WHEEL_H */
