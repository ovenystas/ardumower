/*
 * drop.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ove
 */

#ifndef DROP_SENSOR_H
#define DROP_SENSOR_H

#include <Arduino.h>

class DropSensor
{
  private:
    uint8_t pin {};
    boolean detected { false };

  public:
    uint8_t contactType { NO }; // contact 1=NC 0=NO against GND
    uint16_t counter {};

    enum dropSensorContactE
    {
      NO = 0,
      NC = 1
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

    void simDetected(void)
    {
      detected = true;
      counter++;
    }

    void clearDetected(void)
    {
      detected = false;
    }

    void check(void)
    {
      if (digitalRead(pin) == contactType)
      {
        detected = true;
        counter++;
      }
    }

    void resetCounter(void)
    {
      counter = 0;
    }
};

class DropSensors
{
  public:
    enum dropSensorE
    {
      LEFT,
      RIGHT,
      END
    };

    bool use;
    unsigned long nextTime;
    DropSensor dropSensor[END];

    void check(void)
    {
      dropSensor[LEFT].check();
      dropSensor[RIGHT].check();
    }

    void clearDetected(void)
    {
      dropSensor[LEFT].clearDetected();
      dropSensor[RIGHT].clearDetected();
    }
};

#endif /* DROP_SENSOR_H */
