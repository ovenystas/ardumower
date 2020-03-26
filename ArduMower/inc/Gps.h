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

#include "Arduino.h"

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
  void get_position(long* latitude, long* longitude,
      unsigned long* fix_age = 0);

  // date as ddmmyy, time as hhmmsscc, and age in milliseconds
  void get_datetime(unsigned long* date, unsigned long* time,
      unsigned long* age = 0);

  // signed altitude in centimeters (from GPGGA sentence)
  long altitude() const
  {
    return m_altitude;
  }

  // course in last full GPRMC sentence in 100th of a degree
  unsigned long course() const
  {
    return m_course;
  }

  // speed in last full GPRMC sentence in 100ths of a knot
  unsigned long speed() const
  {
    return m_speed;
  }

  // satellites used in last full GPGGA sentence
  unsigned short satellites() const
  {
    return m_numsats;
  }

  // horizontal dilution of precision in 100ths
  unsigned long hdop() const
  {
    return m_hdop;
  }

  void f_get_position(float* latitude, float* longitude,
      unsigned long* fix_age = 0);

  void crack_datetime(int* year, byte* month, byte* day, byte* hour,
      byte* minute, byte* second, byte* hundredths = 0, unsigned long* fix_age =
          0);

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
  void stats(unsigned long* chars, unsigned short* good_sentences,
      unsigned short* failed_cs);
#endif

private:
  enum
  {
    _GPS_SENTENCE_GPGGA,
    _GPS_SENTENCE_GPRMC,
    _GPS_SENTENCE_OTHER
  };

  // properties
  unsigned long m_time, m_newTime;
  unsigned long m_date, m_newDate;
  long m_latitude, m_newLatitude;
  long m_longitude, m_newLongitude;
  long m_altitude, m_newAltitude;
  unsigned long m_speed, m_newSpeed;
  unsigned long m_course, m_newCourse;
  unsigned long m_hdop, m_newHdop;
  unsigned short m_numsats, m_newNumsats;

  unsigned long m_lastTimeFix, m_newTimeFix;
  unsigned long m_lastPositionFix, m_newPositionFix;

  // parsing state variables
  byte m_parity;
  bool m_isChecksumTerm;
  char m_term[15];
  byte m_sentenceType;
  byte m_termNumber;
  byte m_termOffset;
  bool m_gpsDataGood;

#ifndef _GPS_NO_STATS
  // statistics
  unsigned long m_encodedCharacters;
  unsigned short m_goodSentences;
  unsigned short m_failedChecksum;
  unsigned short m_passedChecksum;
#endif

  // internal utilities
  int from_hex(char a);
  unsigned long parse_decimal();
  unsigned long parse_degrees();
  bool term_complete();
  bool gpsisdigit(char c)
  {
    return c >= '0' && c <= '9';
  }
  long gpsatol(const char *str);
  int gpsstrcmp(const char* str1, const char* str2);
};
