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
    bool use { false };

    void setup(const uint8_t triggerPin, const uint8_t echoPin,
               const uint16_t maxEchoTime = DEFAULT_MAX_ECHO_TIME,
               const uint16_t minEchoTime = DEFAULT_MIN_ECHO_TIME);
    void ping(void);

    unsigned int getDistance_us() const
    {
      return distance_us;
    }
    unsigned int getDistance_cm()
    {
      return PING_CONVERT(distance_us, US_ROUNDTRIP_CM);
    }
    unsigned int getDistance_inch()
    {
      return PING_CONVERT(distance_us, US_ROUNDTRIP_INCH);
    }

  private:
    uint8_t triggerPin {};
    uint8_t echoPin {};
    uint16_t maxEchoTime {};
    uint16_t minEchoTime {};
    unsigned int distance_us {};   // As time in us

    uint8_t triggerBitMask;
    uint8_t echoBitMask;
    volatile uint8_t* triggerOutputRegister_p;
    volatile uint8_t* echoInputRegister_p;
    //unsigned int maxEchoTime;
    unsigned long maxTime;

    inline bool pingTrigger(void);
    inline uint32_t pingInternal(void);
};

class Sonars
{
  public:
    enum sonarE
    {
      LEFT,
      RIGHT,
      CENTER,
      END
    };

    bool use { false };

    unsigned int triggerBelow { 1050 };  // trigger distance
    unsigned int tempDistanceCounter {};
    unsigned long obstacleTimeout {};
    Sonar sonar[END];

    bool isTimeToRun();
    bool isTimeToCheck();

    void ping()
    {
      for (uint8_t i = 0; i < END; i++)
      {
        sonar[i].ping();
      }
    }

    unsigned int getDistanceCounter() const
    {
      return distanceCounter;
    }

    void incDistanceCounter()
    {
      ++distanceCounter;
    }

  private:
    unsigned int distanceCounter {};
    unsigned long nextTime {};
    unsigned int timeBetweenRun { 250 };
    unsigned long nextTimeCheck {};
    unsigned int timeBetweenCheck { 200 };
};
#endif /* SONAR_H_ */
