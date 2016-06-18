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
 */

#ifndef SONAR_H
#define SONAR_H

#define DEFAULT_MAX_ECHO_TIME 3000  // 3000 us / 58.8 = 51 cm
#define DEFAULT_MIN_ECHO_TIME 150   // 150 us / 58,8 = 2.5 cm

// Conversion from uS to distance (round result to nearest cm or inch).
#define US_ROUNDTRIP_INCH 146 // Microseconds (uS) it takes sound to travel round-trip 1 inch (2 inches total), uses integer to save compiled code space.
#define US_ROUNDTRIP_CM 57    // Microseconds (uS) it takes sound to travel round-trip 1 cm (2 cm total), uses integer to save compiled code space.
#define PING_CONVERT(us, conversionFactor) (max((us + conversionFactor / 2) / conversionFactor, (us ? 1 : 0)))

class Sonar
{
  public:
    bool use {false};

    void setup(const uint8_t triggerPin, const uint8_t echoPin,
               const uint16_t maxEchoTime = DEFAULT_MAX_ECHO_TIME,
               const uint16_t minEchoTime = DEFAULT_MIN_ECHO_TIME);

    void ping(void);

    const uint16_t getDistance_us(void) const
    {
      return distance_us;
    }

    const uint16_t getDistance_cm(void)
    {
      return PING_CONVERT(distance_us, US_ROUNDTRIP_CM);
    }

    const uint16_t getDistance_inch(void)
    {
      return PING_CONVERT(distance_us, US_ROUNDTRIP_INCH);
    }

  private:
    uint8_t triggerPin {};
    uint8_t echoPin {};
    uint16_t maxEchoTime {};
    uint16_t minEchoTime {};
    uint16_t distance_us {};   // As time in us

    uint8_t triggerBitMask;
    uint8_t echoBitMask;
    volatile uint8_t* triggerOutputRegister_p;
    volatile uint8_t* echoInputRegister_p;
    //unsigned int maxEchoTime;
    uint32_t maxTime;

    inline bool pingTrigger(void);
    inline uint16_t pingInternal(void);
};

class Sonars
{
  public:
    enum sonarE
    {
      LEFT,
      CENTER,
      RIGHT,
      END
    };

    bool use {false};

    unsigned int triggerBelow {1050};  // trigger distance
    uint8_t tempDistanceCounter {};
    uint32_t obstacleTimeout {};
    Sonar sonar[END];

    bool isTimeToRun(void);
    bool isTimeToCheck(void);

    void ping(void)
    {
      for (uint8_t i = 0; i < END; i++)
      {
        sonar[i].ping();
      }
    }

    const uint16_t getDistanceCounter(void) const
    {
      return distanceCounter;
    }

    void incDistanceCounter(void)
    {
      ++distanceCounter;
    }

  private:
    static const uint8_t TIME_BETWEEN_RUN {250};
    static const uint8_t TIME_BETWEEN_CHECK {200};

    uint16_t distanceCounter {};
    uint32_t nextTime {};
    uint32_t nextTimeCheck {};
};
#endif /* SONAR_H_ */
