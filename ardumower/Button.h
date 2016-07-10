/*
 * button.h
 *
 *  Created on: Mar 10, 2016
 *      Author: ove
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

class Button
{
  public:
    bool used { true };
    unsigned long nextTime {};
    unsigned long nextTimeCheck {};

    void setup(const uint8_t pin);
    bool isPressed(void);
    bool isTimeToCheck();
    bool isTimeToRun();

    const uint8_t getCounter(void) const
    {
      return counter;
    }

    void incCounter(void)
    {
      counter++;
    }

    void clearCounter(void)
    {
      counter = 0;
    }

  private:
    uint8_t pin {};
    uint8_t counter {};
    unsigned int timeBetweenChecks { 50 };
    unsigned int timeBetweenRuns { 1000 };
};

#endif /* BUTTON_H */
