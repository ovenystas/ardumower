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

#include "Gps.h"

#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"

Gps::Gps() :
      m_time(GPS_INVALID_TIME),
      m_newTime(GPS_INVALID_TIME),
      m_date(GPS_INVALID_DATE),
      m_newDate(GPS_INVALID_DATE),
      m_latitude(GPS_INVALID_ANGLE),
      m_newLatitude(GPS_INVALID_ANGLE),
      m_longitude(GPS_INVALID_ANGLE),
      m_newLongitude(GPS_INVALID_ANGLE),
      m_altitude(GPS_INVALID_ALTITUDE),
      m_newAltitude(GPS_INVALID_ALTITUDE),
      m_speed(GPS_INVALID_SPEED),
      m_newSpeed(GPS_INVALID_SPEED),
      m_course(GPS_INVALID_ANGLE),
      m_newCourse(GPS_INVALID_ANGLE),
      m_hdop(GPS_INVALID_HDOP),
      m_newHdop(GPS_INVALID_HDOP),
      m_numsats(GPS_INVALID_SATELLITES),
      m_newNumsats(GPS_INVALID_SATELLITES),
      m_lastTimeFix(GPS_INVALID_FIX_TIME),
      m_newTimeFix(GPS_INVALID_FIX_TIME),
      m_lastPositionFix(GPS_INVALID_FIX_TIME),
      m_newPositionFix(GPS_INVALID_FIX_TIME),
      m_parity(0),
      m_isChecksumTerm(false),
      m_sentenceType(_GPS_SENTENCE_OTHER),
      m_termNumber(0),
      m_termOffset(0),
      m_gpsDataGood(false)
#ifndef _GPS_NO_STATS
          ,
      m_encodedCharacters(0),
      m_goodSentences(0),
      m_failedChecksum(0),
      m_passedChecksum(0)
#endif
{
  m_term[0] = '\0';
}

//
// public methods
//

void Gps::init()
{
  Serial3.begin(9600);
}

bool Gps::feed()
{
  while (Serial3.available())
  {
    if (encode(static_cast<char>(Serial3.read())))
    {
      return true;
    }
  }
  return false;
}

bool Gps::encode(char c)
{
  bool valid_sentence = false;

#ifndef _GPS_NO_STATS
  ++m_encodedCharacters;
#endif
  switch (c)
  {
    case ',': // term terminators
      m_parity ^= (byte)c;
      // fall through
    case '\r':
    case '\n':
    case '*':
      if (m_termOffset < sizeof(m_term))
      {
        m_term[m_termOffset] = 0;
        valid_sentence = term_complete();
      }
      ++m_termNumber;
      m_termOffset = 0;
      m_isChecksumTerm = c == '*';
      return valid_sentence;

    case '$': // sentence begin
      m_termNumber = m_termOffset = 0;
      m_parity = 0;
      m_sentenceType = _GPS_SENTENCE_OTHER;
      m_isChecksumTerm = false;
      m_gpsDataGood = false;
      return valid_sentence;

    default:
      break;
  }

  // ordinary characters
  if (m_termOffset < sizeof(m_term) - 1)
  {
    m_term[m_termOffset++] = c;
  }

  if (!m_isChecksumTerm)
  {
    m_parity ^= static_cast<uint8_t>(c);
  }

  return valid_sentence;
}

#ifndef _GPS_NO_STATS
void Gps::stats(uint32_t *chars, uint16_t *sentences, uint16_t *failed_cs)
{
  if (chars)
  {
    *chars = m_encodedCharacters;
  }

  if (sentences)
  {
    *sentences = m_goodSentences;
  }

  if (failed_cs)
  {
    *failed_cs = m_failedChecksum;
  }
}
#endif

//
// internal utilities
//
int16_t Gps::from_hex(char c)
{
  if (c >= 'A' && c <= 'F')
  {
    return static_cast<int16_t>(c - 'A' + 10);
  }
  else if (c >= 'a' && c <= 'f')
  {
    return static_cast<int16_t>(c - 'a' + 10);
  }
  else
  {
    return static_cast<int16_t>(c - '0');
  }
}

uint32_t Gps::parse_decimal()
{
  char *p = m_term;
  bool isneg = (*p == '-');

  if (isneg)
  {
    ++p;  // skip heading '-'
  }

  int32_t ret = 100 * gpsatol(p); // gpsatol = generalPtrStrToLong => convert int-part to long: 90.239 => 100*90
  while (gpsisdigit(*p))
  {
    ++p; // skip converted digits '.'
  }

  if (*p == '.')
  {
    if (gpsisdigit(p[1])) // any number behind '.' ?
    {
      ret += 10 * (p[1] - '0'); // add first decimal

      if (gpsisdigit(p[2]))
      {
        ret += p[2] - '0'; // add second decimal - note all meter values are converted to cm with the precision of 2 digits
      }
    }
  }
  return static_cast<uint32_t>(isneg ? -ret : ret); // negate result if isneg is set
}

uint32_t Gps::parse_degrees()  // term=5000.0095 (50° 00' 0.0095*60'')
{                                        // (D)DDMM.MMMM is the format
  char *p;
  uint32_t left = static_cast<uint32_t>(gpsatol(m_term)); // get (D)DDMM in left
  uint32_t tenk_minutes = (left % 100U) * 10000U; // get MM*10000

  for (p = m_term; gpsisdigit(*p); ++p)
  {
    ;  // advance to '.'
  }

  if (*p == '.')
  {
    uint32_t mult = 1000;
    while (gpsisdigit(*++p))
    {
      tenk_minutes += mult * static_cast<uint32_t>(*p - '0');
      mult /= 10;
    }
  }
  return (left / 100) * 100000 + tenk_minutes / 6; // DDD * 100000 + tenk_minutes/6=°*10*10000
}

// Sample sentences:
// $GPRMC,HHMMSS,A,BBBB.BBBB,b,LLLLL.LLLL,l,GG.G,RR.R,DDMMYY,M.M,m,F*PP
// $GPGGA,HHMMSS.ss,BBBB.BBBB,b,LLLLL.LLLL,l,Q,NN,D.D,H.H,h,G.G,g,A.A,RRRR*PP
// BBBB.BBBB 	= Breitengrad in Grad und Minuten (ddmm.mmmmmm)
// LLLLL.LLLL 	Längengrad in Grad und Minuten (dddmm.mmmmmm)

#define COMBINE(sentence_type, term_number) \
  ((static_cast<unsigned>(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool Gps::term_complete()
{
  //Console.println("compl");
  if (m_isChecksumTerm)
  {
    uint8_t checksum =
        static_cast<uint8_t>(16 * from_hex(m_term[0]) + from_hex(m_term[1]));
    //Console.print(checksum);
    //Console.print(',');
    //Console.println(_parity);
    if (checksum == m_parity)
    {
      //if (1==1)
      if (m_gpsDataGood)
      {
#ifndef _GPS_NO_STATS
        ++m_goodSentences;
#endif
        m_lastTimeFix = m_newTimeFix;
        m_lastPositionFix = m_newPositionFix;

        switch (m_sentenceType)
        {
          case _GPS_SENTENCE_GPRMC:
            m_time = m_newTime;
            m_date = m_newDate;
            m_latitude = m_newLatitude;
            m_longitude = m_newLongitude;
            m_speed = m_newSpeed;
            m_course = m_newCourse;
            break;

          case _GPS_SENTENCE_GPGGA:
            m_altitude = m_newAltitude;
            m_time = m_newTime;
            m_latitude = m_newLatitude;
            m_longitude = m_newLongitude;
            m_numsats = m_newNumsats;
            m_hdop = m_newHdop;
            break;

          default:
            break;
        }

        return true;
      }
    }

#ifndef _GPS_NO_STATS
    else
    {
      ++m_failedChecksum;
    }
#endif
    return false;
  }

  // the first term determines the sentence type
  if (m_termNumber == 0)
  {
    if (!gpsstrcmp(m_term, _GPRMC_TERM))
    {
      m_sentenceType = _GPS_SENTENCE_GPRMC;
    }
    else if (!gpsstrcmp(m_term, _GPGGA_TERM))
    {
      m_sentenceType = _GPS_SENTENCE_GPGGA;
    }
    else
    {
      m_sentenceType = _GPS_SENTENCE_OTHER;
    }
    return false;
  }

  if (m_sentenceType != _GPS_SENTENCE_OTHER && m_term[0])
  {
    switch (COMBINE(m_sentenceType, m_termNumber))
    {
      case COMBINE(_GPS_SENTENCE_GPRMC, 1): // Time in both sentences
      case COMBINE(_GPS_SENTENCE_GPGGA, 1):
        m_newTime = parse_decimal();
        m_newTimeFix = millis();
        break;

      case COMBINE(_GPS_SENTENCE_GPRMC, 2): // GPRMC validity
        m_gpsDataGood = m_term[0] == 'A';
        break;

      case COMBINE(_GPS_SENTENCE_GPRMC, 3): // Latitude
      case COMBINE(_GPS_SENTENCE_GPGGA, 2):
        m_newLatitude = static_cast<int32_t>(parse_degrees());
        m_newPositionFix = millis();
        break;

      case COMBINE(_GPS_SENTENCE_GPRMC, 4): // N/S
      case COMBINE(_GPS_SENTENCE_GPGGA, 3):
        if (m_term[0] == 'S')
          m_newLatitude = -m_newLatitude;
        break;

      case COMBINE(_GPS_SENTENCE_GPRMC, 5): // Longitude
      case COMBINE(_GPS_SENTENCE_GPGGA, 4):
        m_newLongitude = static_cast<int32_t>(parse_degrees());
        break;

      case COMBINE(_GPS_SENTENCE_GPRMC, 6): // E/W
      case COMBINE(_GPS_SENTENCE_GPGGA, 5):
        if (m_term[0] == 'W')
          m_newLongitude = -m_newLongitude;
        break;

      case COMBINE(_GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
        m_newSpeed = parse_decimal();
        break;

      case COMBINE(_GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
        m_newCourse = parse_decimal();
        break;

      case COMBINE(_GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
        m_newDate = static_cast<uint32_t>(gpsatol(m_term));
        break;

      case COMBINE(_GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA) ; 0=invalid, 1=GPS fix, 2=DGPS fix, 6=estimation
        m_gpsDataGood = m_term[0] > '0';
        break;

      case COMBINE(_GPS_SENTENCE_GPGGA, 7): // NN-Satellites used (GPGGA): 00-12
        m_newNumsats = static_cast<uint8_t>(atoi(m_term));
        break;

      case COMBINE(_GPS_SENTENCE_GPGGA, 8): // D.D - HDOP (GPGGA) - horizontal deviation
        m_newHdop = parse_decimal();
        break;

      case COMBINE(_GPS_SENTENCE_GPGGA, 9): // H.H - Altitude (GPGGA)
        m_newAltitude = static_cast<int32_t>(parse_decimal());
        break;

      default:
        break;
    }
  }
  return false;
}

int32_t Gps::gpsatol(const char *str) // convert string to long - does only work for unsigned ints!
{
  int32_t ret = 0;
  while (gpsisdigit(*str)) // process only digits
  {
    ret = 10 * ret + *str++ - '0'; // inc str after assignment
  }
  return ret;
}

int16_t Gps::gpsstrcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2)
  {
    ++str1, ++str2;
  }
  return *str1;
}

/* static */
float Gps::distance_between(float lat1, float long1, float lat2, float long2)
{
  // http://www.movable-type.co.uk/scripts/latlong.html
  // https://forum.sparkfun.com/viewtopic.php?f=17&t=22520
  // http://boulter.com/gps/distance/?from=51.71577+8.74353&to=51.71578+8.74355&units=k#more
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1 - long2);
  float sdlong = static_cast<float>(sin(delta));
  float cdlong = static_cast<float>(cos(delta));
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = static_cast<float>(sin(lat1));
  float clat1 = static_cast<float>(cos(lat1));
  float slat2 = static_cast<float>(sin(lat2));
  float clat2 = static_cast<float>(cos(lat2));
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = static_cast<float>(sqrt(delta));
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = static_cast<float>(atan2(delta, denom));
  return delta * 6372795.0f;
}

float Gps::course_to(float lat1, float long1, float lat2, float long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = static_cast<float>(sin(dlon) * cos(lat2));
  float a2 = static_cast<float>(sin(lat1) * cos(lat2) * cos(dlon));
  a2 = static_cast<float>(cos(lat1) * sin(lat2)) - a2;
  a2 = static_cast<float>(atan2(a1, a2));
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

const char* Gps::cardinal(float course)
{
  static const char *directions[] =
  {
      "N", "NNE", "NE", "ENE",
      "E", "ESE", "SE", "SSE",
      "S", "SSW", "SW", "WSW",
      "W", "WNW", "NW", "NNW"
  };

  int16_t direction = static_cast<int16_t>((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}

// lat/long in hundred thousandths of a degree and age of fix in milliseconds
void Gps::get_position(int32_t* latitude, int32_t* longitude, uint32_t* fix_age)
{
  if (latitude)
  {
    *latitude = m_latitude;
  }
  if (longitude)
  {
    *longitude = m_longitude;
  }
  if (fix_age)
  {
    *fix_age = m_lastPositionFix == GPS_INVALID_FIX_TIME ?
        static_cast<uint32_t>(GPS_INVALID_AGE) : millis() - m_lastPositionFix;
  }
}

// date as ddmmyy, time as hhmmsscc, and age in milliseconds
void Gps::get_datetime(uint32_t* date, uint32_t* time, uint32_t* age)
{
  if (date)
  {
    *date = m_date;
  }

  if (time)
  {
    *time = m_time;
  }

  if (age)
  {
    *age = m_lastTimeFix == GPS_INVALID_FIX_TIME ?
        static_cast<uint32_t>(GPS_INVALID_AGE) : millis() - m_lastTimeFix;
  }
}

void Gps::f_get_position(float* latitude, float* longitude, uint32_t* fix_age)
{
  int32_t lat, lon;

  get_position(&lat, &lon, fix_age);

  if (lat == GPS_INVALID_ANGLE)
  {
    *latitude = GPS_INVALID_F_ANGLE;
    *longitude = GPS_INVALID_ANGLE;
  }
  else
  {
    *latitude = static_cast<float>(lat) / 100000.0f;
    *longitude = static_cast<float>(lon) / 100000.0f;
  }
}

void Gps::crack_datetime(int16_t* year, byte* month, byte* day, byte* hour,
    byte* minute, byte* second, byte* hundredths, uint32_t* age)
{
  uint32_t date, time;
  get_datetime(&date, &time, age);
  if (year)
  {
    *year = static_cast<int16_t>(date % 100);
    *year = static_cast<int16_t>(*year + (*year > 80 ? 1900 : 2000));
  }
  if (month)
  {
    *month = static_cast<uint8_t>((date / 100) % 100);
  }
  if (day)
  {
    *day = static_cast<uint8_t>(date / 10000);
  }
  if (hour)
  {
    *hour = static_cast<uint8_t>(time / 1000000);
  }
  if (minute)
  {
    *minute = static_cast<uint8_t>((time / 10000) % 100);
  }
  if (second)
  {
    *second = static_cast<uint8_t>((time / 100) % 100);
  }
  if (hundredths)
  {
    *hundredths = static_cast<uint8_t>(time % 100);
  }
}

float Gps::f_altitude()
{
  if (m_altitude == GPS_INVALID_ALTITUDE)
  {
    return GPS_INVALID_F_ALTITUDE;
  }
  else
  {
    return static_cast<float>(m_altitude) / 100.0f;
  }
}

float Gps::f_course()
{
  return m_course == GPS_INVALID_ANGLE ?
      GPS_INVALID_F_ANGLE : static_cast<float>(m_course) / 100.0f;
}

float Gps::f_speed_knots()
{
  return m_speed == GPS_INVALID_SPEED ?
      GPS_INVALID_F_SPEED : static_cast<float>(m_speed) / 100.0f;
}

float Gps::f_speed_mph()
{
  float sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ?
      GPS_INVALID_F_SPEED : _GPS_MPH_PER_KNOT * f_speed_knots();
}

float Gps::f_speed_mps()
{
  float sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ?
      GPS_INVALID_F_SPEED : _GPS_MPS_PER_KNOT * f_speed_knots();
}

float Gps::f_speed_kmph()
{
  float sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ?
      GPS_INVALID_F_SPEED : _GPS_KMPH_PER_KNOT * f_speed_knots();
}
