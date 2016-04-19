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
  public:
    enum dropSensorContactE
    {
      NO = 0,
      NC = 1
    };

    void setup(const uint8_t pin, const boolean contactType);
    void simDetected();
    void check();


    bool isDetected()
    {
      return detected;
    }

    void clearDetected()
    {
      detected = false;
    }

    uint16_t getCounter() const
    {
      return counter;
    }

    void clearCounter()
    {
      counter = 0;
    }

    uint8_t getContactType() const
    {
      return contactType;
    }

  private:
    uint8_t pin {};
    boolean detected { false };
    uint8_t contactType { NO }; // contact 1=NC 0=NO against GND
    uint16_t counter {};
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

    bool used;
    DropSensor dropSensor[END];

    void check();
    void clearDetected();
    bool isTimeToRun();

  private:
    unsigned long nextTime;
    unsigned int timeBetweenRuns { 100 };
};

#endif /* DROP_SENSOR_H */
