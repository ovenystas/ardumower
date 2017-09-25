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

#include <Arduino.h>

#define SONAR_DEFAULT_MAX_ECHO_TIME 3000  // 3000 us / 58.8 = 51 cm
#define SONAR_DEFAULT_MIN_ECHO_TIME 150   // 150 us / 58,8 = 2.5 cm

// Conversion from uS to distance (round result to nearest cm or inch).
#define US_ROUNDTRIP_INCH 146 // Microseconds (uS) it takes sound to travel round-trip 1 inch (2 inches total), uses integer to save compiled code space.
#define US_ROUNDTRIP_CM 57    // Microseconds (uS) it takes sound to travel round-trip 1 cm (2 cm total), uses integer to save compiled code space.
#define PING_CONVERT(us, conversionFactor) (max((us + conversionFactor / 2) / conversionFactor, (us ? 1 : 0)))

typedef enum
{
  SONAR_LEFT,
  SONAR_CENTER,
  SONAR_RIGHT,
  SONAR_END
} sonarE;

typedef struct
{
  bool use;
  uint8_t triggerPin;
  uint8_t echoPin;
  uint16_t maxEchoTime;
  uint16_t minEchoTime;
  uint16_t distance_us;   // As time in us
  uint8_t triggerBitMask;
  uint8_t echoBitMask;
  volatile uint8_t* triggerOutputRegister_p;
  volatile uint8_t* echoInputRegister_p;
  uint32_t maxTime;
} Sonar;

typedef struct
{
  bool use {false};
  unsigned int triggerBelow {1050};  // trigger distance
  uint8_t tempDistanceCounter;
  uint32_t obstacleTimeout;
  uint8_t len;
  Sonar* sonarArray_p;
  uint16_t distanceCounter;
} Sonars;

void sonar_setup(const uint8_t triggerPin, const uint8_t echoPin,
                 const uint16_t maxEchoTime,
                 const uint16_t minEchoTime,
                 Sonar* sonar_p);

void sonar_ping(Sonar* sonar_p);

static inline
uint16_t sonar_getDistance_us(Sonar* sonar_p)
{
  return sonar_p->distance_us;
}

static inline
uint16_t sonar_getDistance_cm(Sonar* sonar_p)
{
  return PING_CONVERT(sonar_p->distance_us, US_ROUNDTRIP_CM);
}

static inline
uint16_t sonar_getDistance_inch(Sonar* sonar_p)
{
  return PING_CONVERT(sonar_p->distance_us, US_ROUNDTRIP_INCH);
}

inline bool sonar_pingTrigger(Sonar* sonar_p);
inline uint16_t sonar_pingInternal(Sonar* sonar_p);




static inline
void sonars_ping(Sonars* sonars_p)
{
  for (uint8_t i = 0; i < SONAR_END; i++)
  {
    sonar_ping(&sonars_p->sonarArray_p[i]);
  }
}

static inline
uint16_t sonars_getDistanceCounter(Sonars* sonars_p)
{
  return sonars_p->distanceCounter;
}

static inline
void sonars_incDistanceCounter(Sonars* sonars_p)
{
  ++sonars_p->distanceCounter;
}

bool sonars_isClose(Sonars* sonars_p);

#endif /* SONAR_H */
