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
    void setup(const uint8_t pin, const uint8_t pin2, const bool swapDir);
    void read();
    void setState(unsigned long timeMicros);

    bool twoWay;                // using 2-pin encoder
    bool swapDir;               // inverse encoder direction?
    int counter{};              // wheel counter
    boolean lastState { LOW };  // last state
    boolean lastState2 { LOW }; // last state on 2nd pin
    float wheelRpmCurr {};      // wheel rpm

  private:
    uint8_t pin;
    uint8_t pin2;
    boolean curState { LOW };   // current state
    boolean curState2 { LOW };  // current state on 2nd pin
};

#endif /* ENCODER_H */