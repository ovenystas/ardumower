/*
 * bumper.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ove
 */

#ifndef BUMPER_H
#define BUMPER_H

#include <Arduino.h>

class Bumper
{

  private:
    uint8_t pin {};

  public:
    uint16_t counter {};
    boolean hit { false };

    void setup(const uint8_t pin)
    {
      this->pin = pin;
      pinMode(pin, INPUT_PULLUP);
    }

    bool isHit(void)
    {
      return hit;
    }

    void clearHit(void)
    {
      hit = false;
    }

    void simHit(void)
    {
      hit = true;
      counter++;
    }

    void check(void)
    {
      if (digitalRead(pin) == LOW)
      {
        hit = true;
        counter++;
      }
    }

    void resetCounter(void)
    {
      counter = 0;
    }
};

class Bumpers
{
  public:
    enum bumpersE
    {
      LEFT,
      RIGHT,
      END
    };

    bool use;
    unsigned long nextTime;
    Bumper bumper[END];

    void check()
    {
      bumper[LEFT].check();
      bumper[RIGHT].check();
    }

    void clearHit()
    {
      bumper[LEFT].clearHit();
      bumper[RIGHT].clearHit();
    }
};

#endif /* BUMPER_H */
