/*
 * rainSensor.h
 *
 *  Created on: Apr 7, 2016
 *      Author: ove
 */

#ifndef RAINSENSOR_H
#define RAINSENSOR_H

#include <Arduino.h>

class RainSensor
{
  public:
    boolean raining { false };
    boolean use { false };
    unsigned int counter {};
    unsigned long nextTime {};

    void setup(const uint8_t pin)
    {
      this->pin = pin;
      pinMode(pin, INPUT);
    }

    void read()
    {
      raining = (digitalRead(pin) == LOW);
    }

    void check()
    {
      read();
      if (raining)
      {
        counter++;
      }
    }

  private:
    uint8_t pin;
};

#endif /* RAINSENSOR_H */
