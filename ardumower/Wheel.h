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

    void control(int8_t speed);
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
    void control(void);

    const int8_t getSpeed() const
    {
      return speed;
    }

    void setSpeed(int8_t speed)
    {
      this->speed = speed;
    }

    const int8_t getSteer() const
    {
      return steer;
    }

    void setSteer(int8_t steer)
    {
      this->steer = steer;
    }

  private:
    static const uint16_t TIME_BETWEEN_ROTATION_CHANGE {60000};

    unsigned long nextTimeRotationChange {};
    int8_t speed {}; // Range -100..+100, - = reverse, + = forward
    int8_t steer {}; // Range -100..+100, - = left, + = right
};

#endif /* WHEEL_H */
