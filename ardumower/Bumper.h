/*
 * bumper.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ove
 */

#ifndef BUMPER_H
#define BUMPER_H

#include <Arduino.h>

typedef struct Bumper_settingsT
{
  bool used {false};
} Bumper_settingsT;

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

    uint16_t getCounter(void) const
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
    boolean hit {false};
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

    Bumper_settingsT settings;
    Bumper bumper[END];

    void check(void);
    bool isAnyHit(void);
    void clearHit(void);
    bool isTimeToRun(void);

private:
    static const uint8_t TIME_BETWEEN_RUNS {100};

    unsigned long nextTime {};
};

#endif /* BUMPER_H */
