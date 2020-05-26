/*
 * MotorShield.h
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

/**
 * Deek-Robot or Arduino, Motor Shield R3
 */
#pragma once

#include <Arduino.h>
#include "Motor.h"

class MotorShield: public Motor
{
public:
  void setup() override;
  void setSpeed() override;
  void setSpeed(int16_t speed) override;
  void setSpeed(int16_t speed, bool brake) override;
  void readCurrent() override;
  void control();

  void setChannel(uint8_t channel)
  {
    m_channel = channel;
  }

private:
  // Pins: channel { A , B }
  const uint8_t pinDir[2] { 12, 13 };
  const uint8_t pinPwm[2] { 3, 11 };
  const uint8_t pinBrake[2] { 9, 8 };
  const uint8_t pinSense[2] { A0, A1 };

  uint8_t m_channel {};
};
