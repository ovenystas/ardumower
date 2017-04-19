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
    bool use {false};

    void setup(const uint8_t pin);
    void check(void);
    bool isTimeToRun(void);

    bool isRaining(void) const
    {
      return raining;
    }

    uint16_t getCounter(void) const
    {
      return counter;
    }

  private:
    static const uint16_t TIME_BETWEEN_RUNS {5000};

    uint8_t pin;
    bool raining {false};
    uint16_t counter {};
    unsigned long nextTime {};

    void read(void);
};

#endif /* RAINSENSOR_H */
