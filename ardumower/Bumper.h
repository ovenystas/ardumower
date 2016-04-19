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
  public:
    void setup(const uint8_t pin);
    void simHit(void);
    void check(void);

    bool isHit(void) const
    {
      return hit;
    }

    void clearHit(void)
    {
      hit = false;
    }

    uint16_t getCounter() const
    {
      return counter;
    }

    void clearCounter(void)
    {
      counter = 0;
    }

  private:
    uint8_t pin {};
    uint16_t counter {};
    boolean hit { false };
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

    bool used { false };
    Bumper bumper[END];

    void check();
    void clearHit();
    bool isTimeToRun();

private:
    unsigned long nextTime {};
    unsigned int timeBetweenRuns { 100 };
};

#endif /* BUMPER_H */
