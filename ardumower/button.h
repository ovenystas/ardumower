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
    uint8_t pin;
    uint8_t counter;

  public:
    bool isPressed()
    {
      return (digitalRead(this->pin) == LOW);
    }

    uint8_t getCounter() const
    {
      return counter;
    }

    void incCounter()
    {
      this->counter++;
    }

    void resetCounter()
    {
      this->counter = 0;
    }

    void setPin(uint8_t pin)
    {
      this->pin = pin;
    }
};

#endif /* BUTTON_H_ */
