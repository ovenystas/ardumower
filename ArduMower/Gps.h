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

#define GPS_MPH_PER_HUNDREDTHS_KNOT 0.0115077945f
#define GPS_MPS_PER_HUNDREDTHS_KNOT 0.0051444444f
#define GPS_KMPH_PER_HUNDREDTHS_KNOT 0.01852f
// #define _GPS_NO_STATS

class Gps
{
public:
  static const uint32_t GPS_INVALID_AGE = 0xFFFFFFFF;
  static const  int32_t GPS_INVALID_ANGLE = 999999999;
  static const uint16_t GPS_INVALID_COURSE = 59999;
  static const  int32_t GPS_INVALID_ALTITUDE = 999999999;
  static const uint32_t GPS_INVALID_DATE = 0;
  static const uint32_t GPS_INVALID_TIME = 0xFFFFFFFF;
  static const uint32_t GPS_INVALID_SPEED = 999999999;
  static const uint32_t GPS_INVALID_FIX_TIME = 0xFFFFFFFF;
  static const uint16_t GPS_INVALID_SATELLITES = 0xFF;
  static const uint32_t GPS_INVALID_HDOP = 0xFFFFFFFF;

  const float GPS_INVALID_F_ANGLE = 1000.0f;
  const float GPS_INVALID_F_ALTITUDE = 1000000.0f;
  const float GPS_INVALID_F_SPEED = -1.0f;

  Gps() {};

  bool encode(char c); // process one character received from GPS
  void init();
  bool feed();
  Gps& operator <<(char c)
  {
    encode(c);
    return *this;
  }

  // lat/long in hundred thousands of a degree and age of fix in milliseconds
  void get_position(int32_t& latitude, int32_t& longitude, uint32_t& fix_age);

  // date as ddmmyy, time as hhmmsscc, and age in milliseconds
  void get_datetime(uint32_t& date, uint32_t& time, uint32_t& age);

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

  void f_get_position(float& latitude, float& longitude, uint32_t& fix_age);

  void crack_datetime(uint16_t& year, uint8_t& month, uint8_t& day,
      uint8_t& hour, uint8_t& minute, uint8_t& second, uint8_t& hundredths,
      uint32_t& age);

  float f_altitude();
  float f_course();
  float f_speed_knots();
  float f_speed_mph();
  float f_speed_mps();
  float f_speed_kmph();

  //static int library_version() { return _GPS_VERSION; }

  static float distance_between(float lat1, float long1,
      float lat2, float long2);
  static float course_to(float lat1, float long1, float lat2, float long2);
  static const char* cardinal(float course);

#ifndef _GPS_NO_STATS
  void stats(uint32_t *chars, uint16_t *sentences, uint16_t *failed_cs);
#endif

private:
  enum
  {
    GPS_SENTENCE_GPGGA,
    GPS_SENTENCE_GPRMC,
    GPS_SENTENCE_OTHER
  };

  // properties
  uint32_t m_time { GPS_INVALID_TIME };
  uint32_t m_newTime { GPS_INVALID_TIME };
  uint32_t m_date { GPS_INVALID_DATE };
  uint32_t m_newDate { GPS_INVALID_DATE };
  int32_t m_latitude { GPS_INVALID_ANGLE };
  int32_t m_newLatitude { GPS_INVALID_ANGLE };
  int32_t m_longitude { GPS_INVALID_ANGLE };
  int32_t m_newLongitude { GPS_INVALID_ANGLE };
  int32_t m_altitude { GPS_INVALID_ALTITUDE };
  int32_t m_newAltitude { GPS_INVALID_ALTITUDE };
  uint32_t m_speed { GPS_INVALID_SPEED };
  uint32_t m_newSpeed { GPS_INVALID_SPEED };
  uint16_t m_course { GPS_INVALID_COURSE };
  uint16_t m_newCourse { GPS_INVALID_COURSE };
  uint32_t m_hdop { GPS_INVALID_HDOP };
  uint32_t m_newHdop { GPS_INVALID_HDOP };
  uint16_t m_numsats { GPS_INVALID_SATELLITES };
  uint16_t m_newNumsats { GPS_INVALID_SATELLITES };

  uint32_t m_lastTimeFix { GPS_INVALID_FIX_TIME };
  uint32_t m_newTimeFix { GPS_INVALID_FIX_TIME };
  uint32_t m_lastPositionFix { GPS_INVALID_FIX_TIME };
  uint32_t m_newPositionFix { GPS_INVALID_FIX_TIME };

  // parsing state variables
  uint8_t m_parity {};
  bool m_isChecksumTerm {};
  char m_term[15] { '\0' };
  uint8_t m_sentenceType { GPS_SENTENCE_OTHER };
  uint8_t m_termNumber {};
  uint8_t m_termOffset {};
  bool m_gpsDataGood {};

#ifndef _GPS_NO_STATS
  // statistics
  uint32_t m_encodedCharacters {};
  uint16_t m_goodSentences {};
  uint16_t m_failedChecksum {};
  uint16_t m_passedChecksum {};
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
