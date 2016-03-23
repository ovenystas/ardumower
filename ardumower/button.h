/*
 * button.h
 *
 *  Created on: Mar 10, 2016
 *      Author: ove
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include <Arduino.h>

class Button
{
  private:
    uint8_t pin{};
    uint8_t counter{};

  public:
    void setup(const uint8_t pin)
    {
      this->pin = pin;
      pinMode(pin, INPUT_PULLUP);
    }

    bool isPressed(void)
    {
      return (digitalRead(this->pin) == LOW);
    }

    uint8_t getCounter(void)
    {
      return counter;
    }

    void incCounter(void)
    {
      counter++;
    }

    void resetCounter(void)
    {
      counter = 0;
    }
};

#endif /* BUTTON_H_ */
