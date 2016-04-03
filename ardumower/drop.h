/*
 * drop.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ove
 */

#ifndef DROP_H
#define DROP_H

#define DROP_CONTACT_NC 1
#define DROP_CONTACT_NO 0

#include <Arduino.h>

class Drop
{

  private:
    uint8_t pin{};
    uint8_t contactType{DROP_CONTACT_NO}; // contact 1=NC 0=NO against GND
    uint16_t counter{};
    boolean detected{false};

  public:
    enum
    {
      LEFT,
      RIGHT,
      END
    };

    void setup(const uint8_t pin, const boolean contactType)
    {
      this->pin = pin;
      this->contactType = contactType;
      pinMode(pin, INPUT_PULLUP);
    }

    bool isDetected(void)
    {
      return detected;
    }

    void clearDetected(void)
    {
      detected = false;
    }

    void simDetected(void)
    {
      detected = true;
      counter++;
    }

    void check(void)
    {
      if (digitalRead(pin) == contactType)
      {
        detected = true;
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

    uint8_t getContactType(void)
    {
      return contactType;
    }
};

#endif /* DROP_H */
