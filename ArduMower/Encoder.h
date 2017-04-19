/*
 * encoder.h
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder
{
  public:
    // Setup for one-way encoder (1-pin)
    void setup(const uint8_t pin, const bool swapDir);

    // Setup for two-way encoder (2-pins)
    void setup(const uint8_t pin, const uint8_t pin2, const bool swapDir);

    void read(void);
    void setState(void);

    int16_t getCounter(void) const
    {
      return counter;
    }

    void clearCounter(void)
    {
      counter = 0;
    }

    int16_t getWheelRpmCurr() const
    {
      return wheelRpmCurr;
    }

    void setWheelRpmCurr(int16_t wheelRpmCurr)
    {
      this->wheelRpmCurr = wheelRpmCurr;
    }

    bool twoWay { false };  // using 2-pin encoder
    bool swapDir { false }; // invert encoder direction?

  private:
    uint8_t pin;
    uint8_t pin2;
    bool curState { LOW };   // current state
    bool curState2 { LOW };  // current state on 2nd pin
    bool lastState { LOW };  // last state
    bool lastState2 { LOW }; // last state on 2nd pin
    int16_t counter {};             // wheel counter
    int16_t wheelRpmCurr {};      // wheel rpm
};

#endif /* ENCODER_H */
