/**
 * sonar.h
 *
 *  Created on: Mar 22, 2016
 *      Author: ove
 */

/**
 * HC-SR04 ultrasonic sensor driver
 *
 * Features:
 *   Power Supply :+5V DC
 *   Quiescent Current : <2mA
 *   Working Current: 15mA
 *   Effectual Angle: <15°
 *   Ranging Distance : 2cm – 400 cm/1″ – 13ft
 *   Resolution : 0.3 cm
 *   Measuring Angle: 30 degree
 *   Trigger Input Pulse width: 10uS
 *   Dimension: 45mm x 20mm x 15mm
 *
 *
 */

#ifndef SONAR_H
#define SONAR_H

class Sonar
{
  public:
    Sonar() {};

    void setup(const uint8_t triggerPin, const uint8_t echoPin);

    void setup(const uint8_t triggerPin, const uint8_t echoPin,
               const uint16_t maxEchoTime, const uint16_t minEchoTime);

    uint32_t ping(void);

  protected:
    uint8_t triggerPin{};
    uint8_t echoPin{};
    uint16_t maxEchoTime{};
    uint16_t minEchoTime{};
};

#endif /* SONAR_H_ */
