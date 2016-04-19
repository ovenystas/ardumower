/*
 * rainSensor.h
 *
 *  Created on: Apr 7, 2016
 *      Author: ove
 */

#ifndef RAINSENSOR_H
#define RAINSENSOR_H

class RainSensor
{
  public:
    boolean used { false };

    void setup(const uint8_t pin);
    void check();
    bool isTimeToRun();

    boolean isRaining() const
    {
      return raining;
    }

    unsigned int getCounter() const
    {
      return counter;
    }

  private:
    uint8_t pin;
    boolean raining { false };
    unsigned int counter {};
    unsigned int timeBetweenRuns { 5000 };
    unsigned long nextTime {};

    void read();
};

#endif /* RAINSENSOR_H */
