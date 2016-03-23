/*
 * bumper.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ove
 */

#ifndef BUMPER_H
#define BUMPER_H

class Bumper
{

  private:
    uint8_t pin{};
    uint16_t counter{};
    boolean hit{false};

  public:
    void setup(const uint8_t pin)
    {
      this->pin = pin;
      pinMode(pin, INPUT_PULLUP);
    }

    bool hasHit(void)
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

    uint16_t getCounter(void)
    {
      return counter;
    }

    void resetCounter(void)
    {
      counter = 0;
    }
};

#endif /* BUMPER_H */
