/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat

 Private-use only! (you need to ask for a commercial-use)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 Private-use only! (you need to ask for a commercial-use)

 */

//  GPS neo6m  (NMEA-0183 protocol)

#pragma once

#include <Arduino.h>

#define _GPS_MPH_PER_KNOT 1.15077945f
#define _GPS_MPS_PER_KNOT 0.51444444f
#define _GPS_KMPH_PER_KNOT 1.852f
#define _GPS_MILES_PER_METER 0.00062137112f
#define _GPS_KM_PER_METER 0.001f
// #define _GPS_NO_STATS

class Gps
{
public:
  enum
  {
    GPS_INVALID_AGE = 0xFFFFFFFF,
    GPS_INVALID_ANGLE = 999999999,
    GPS_INVALID_ALTITUDE = 999999999,
    GPS_INVALID_DATE = 0,
    GPS_INVALID_TIME = 0xFFFFFFFF,
    GPS_INVALID_SPEED = 999999999,
    GPS_INVALID_FIX_TIME = 0xFFFFFFFF,
    GPS_INVALID_SATELLITES = 0xFF,
    GPS_INVALID_HDOP = 0xFFFFFFFF
  };

  const float GPS_INVALID_F_ANGLE = 1000.0f;
  const float GPS_INVALID_F_ALTITUDE = 1000000.0f;
  const float GPS_INVALID_F_SPEED = -1.0f;

  Gps();

  bool encode(char c); // process one character received from GPS
  void init();
  bool feed();
  Gps& operator <<(char c)
  {
    encode(c);
    return *this;
  }

  // lat/long in hundred thousands of a degree and age of fix in milliseconds
  void get_position(int32_t* latitude, int32_t* longitude,
      uint32_t* fix_age = 0);

  // date as ddmmyy, time as hhmmsscc, and age in milliseconds
  void get_datetime(uint32_t* date, uint32_t* time, uint32_t* age = 0);

  // signed altitude in centimeters (from GPGGA sentence)
  int32_t altitude() const
  {
    return m_altitude;
  }

  // course in last full GPRMC sentence in 100th of a degree
  uint32_t course() const
  {
    return m_course;
  }

  // speed in last full GPRMC sentence in 100ths of a knot
  uint32_t speed() const
  {
    return m_speed;
  }

  // satellites used in last full GPGGA sentence
  uint16_t satellites() const
  {
    return m_numsats;
  }

  // horizontal dilution of precision in 100ths
  uint32_t hdop() const
  {
    return m_hdop;
  }

  void f_get_position(float* latitude, float* longitude, uint32_t* fix_age = 0);

  void crack_datetime(int16_t* year, byte* month, byte* day, byte* hour,
      byte* minute, byte* second, byte* hundredths = 0, uint32_t* fix_age = 0);

  float f_altitude();
  float f_course();
  float f_speed_knots();
  float f_speed_mph();
  float f_speed_mps();
  float f_speed_kmph();

  //static int library_version() { return _GPS_VERSION; }

  static float distance_between(float lat1, float long1, float lat2,
      float long2);
  static float course_to(float lat1, float long1, float lat2, float long2);
  static const char* cardinal(float course);

#ifndef _GPS_NO_STATS
  void stats(uint32_t *chars, uint16_t *sentences, uint16_t *failed_cs);
#endif

private:
  enum
  {
    _GPS_SENTENCE_GPGGA,
    _GPS_SENTENCE_GPRMC,
    _GPS_SENTENCE_OTHER
  };

  // properties
  uint32_t m_time;
  uint32_t m_newTime;
  uint32_t m_date;
  uint32_t m_newDate;
  int32_t m_latitude;
  int32_t m_newLatitude;
  int32_t m_longitude;
  int32_t m_newLongitude;
  int32_t m_altitude;
  int32_t m_newAltitude;
  uint32_t m_speed;
  uint32_t m_newSpeed;
  uint32_t m_course;
  uint32_t m_newCourse;
  uint32_t m_hdop;
  uint32_t m_newHdop;
  uint16_t m_numsats;
  uint16_t m_newNumsats;

  uint32_t m_lastTimeFix;
  uint32_t m_newTimeFix;
  uint32_t m_lastPositionFix;
  uint32_t m_newPositionFix;

  // parsing state variables
  uint8_t m_parity;
  bool m_isChecksumTerm;
  char m_term[15];
  uint8_t m_sentenceType;
  uint8_t m_termNumber;
  uint8_t m_termOffset;
  bool m_gpsDataGood;

#ifndef _GPS_NO_STATS
  // statistics
  uint32_t m_encodedCharacters;
  uint16_t m_goodSentences;
  uint16_t m_failedChecksum;
  uint16_t m_passedChecksum;
#endif

  // internal utilities
  int16_t from_hex(char a);
  uint32_t parse_decimal();
  uint32_t parse_degrees();
  bool term_complete();
  bool gpsisdigit(char c)
  {
    return c >= '0' && c <= '9';
  }
  int32_t gpsatol(const char *str);
  int16_t gpsstrcmp(const char* str1, const char* str2);
};
