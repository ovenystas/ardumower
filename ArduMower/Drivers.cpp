/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat
 Copyright (c) 2014 by Maxime Carpentieri

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

#include "Drivers.h"

#include <Wire.h>

const char* dayOfWeek[] = { "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };

// ---- print helpers ----------------------------------------------------------

void StreamPrint_progmem(Print &out, PGM_P format, ...)
{
  // program memory version of printf - copy of format string and result share a
  // buffer so as to avoid too much memory use
  char formatString[128];
  char *ptr;

  strncpy_P(formatString, format, sizeof(formatString)); // copy in from program mem

  // null terminate - leave char since we might need it in worst case for result's \0
  formatString[sizeof(formatString) - 2] = '\0';
  ptr = &formatString[strlen(formatString) + 1];// our result buffer...

  va_list args;
  va_start (args, format);
  vsnprintf(ptr,
            sizeof(formatString) - 1 - strlen(formatString),
            formatString, args );
  va_end (args);

  formatString[sizeof(formatString) - 1] = '\0';
  out.print(ptr);
}


#if 0
int freeRam(void)
{
  extern int __heap_start, *__brkval;
  int v;

  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
#endif

// rescale to -PI..+PI
float scalePI(float v)
{
  float d = v;

  while (d < 0)
  {
    d += 2 * PI;
  }

  while (d >= 2 * PI)
  {
    d -= 2 * PI;
  }

  if (d >= PI)
  {
    return (-2 * PI + d);
  }
  else if (d < -PI)
  {
    return (2 * PI + d);
  }
  else
  {
    return d;
  }
}


// computes minimum distance between x radiant (current-value) and w radiant (set-value)
float distancePI(float x, float w)
{
  // cases:
  // w=330 degree, x=350 degree => -20 degree
  // w=350 degree, x=10  degree => -20 degree
  // w=10  degree, x=350 degree =>  20 degree
  // w=0   degree, x=190 degree => 170 degree
  // w=190 degree, x=0   degree => -170 degree

  float d = scalePI(w - x);

  if (d < -PI)
  {
    d = d + 2 * PI;
  }
  else if (d > PI)
  {
    d = d - 2 * PI;
  }

  return d;
}

/*
 * Convert timehm_t format to minutes.
 *
 * Valid range of time.hour is 0 to 23.
 * Valid range of time.minute is 0 to 59.
 * 23 hours and 59 minutes equals 1439 minutes.
 * There are no checks of valid values.
 */
int time2minutes(const timehm_t& time)
{
  return (time.hour * 60 + time.minute);
}

/*
 * Convert minutes to timehm_t format.
 *
 * Valid range of minutes is 0 to 1439.
 * 1439 minutes equals 23 hours and 59 minutes.
 * There are no checks of valid values.
 */
void minutes2time(int minutes, timehm_t& time)
{
  time.hour = static_cast<uint8_t>(minutes / 60);
  time.minute = static_cast<uint8_t>(minutes % 60);
}

/*
 * Create a time string in 24h format "HH:MM" from a time in timehm_t format.
 *
 * Valid range of time.hour is 0 to 23.
 * Valid range of time.minute is 0 to 59.
 * There are no checks of valid values.
 */
String time2str(const timehm_t& time)
{
  String s = String(time.hour / 10);
  s += (time.hour % 10);
  s += ":";
  s += (time.minute / 10);
  s += (time.minute % 10);

  return s;
}


/*
 * Create a date string in format "ddd yyyy-mm-dd" from a date in date_t format.
 *
 * Valid range of date.year is 1000 to 9999.
 * Valid range of date.month is 1 to 12.
 * Valid range of date.day is 1 to 31.
 * Valid range of date.dayOfWeek is 0 to 6 (Mon to Sun).
 * There are no checks of valid values.
 */
String date2str(const date_t& date)
{
  String s = dayOfWeek[date.dayOfWeek];
  s += " ";
  s += date.year;
  s += "-";
  s += date.month / 10;
  s += date.month % 10;
  s += "-";
  s += date.day / 10;
  s += date.day % 10;
  return s;
}

/*
 *  Returns the day of week (0=Monday, 6=Sunday) for a given date
 */
uint8_t getDayOfWeek(uint16_t year, uint8_t month, uint8_t day,
    CalendarSystem calendarSystem)
{
  // CalendarSystem = 0 for Gregorian Calendar, -1 for Julian Calendar
  if (month < 3)
  {
    month = static_cast<uint8_t>(month + 12);
    year--;
  }

  return static_cast<uint8_t>(
      (static_cast<uint16_t>(day + (2 * month) + 6 * (month + 1) / 10) +
          year + year / 4 - year / 100 + year / 400 +
          static_cast<uint16_t>(calendarSystem)) % 7);
}

// ---- I2C helpers --------------------------------------------------------------
void I2CwriteTo(uint8_t device, uint8_t address, uint8_t val)
{
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}


void I2CwriteTo(uint8_t device, uint8_t address, int num, const uint8_t* buf_p)
{
  Wire.beginTransmission(device);
  Wire.write(address);

  for (int i = 0; i < num; i++)
  {
    Wire.write(buf_p[i]);
  }

  Wire.endTransmission();
}


int I2CreadFrom(uint8_t device, uint8_t address,
                uint8_t num, uint8_t* buf_p, int retryCount)
{
  int i = 0;

  for (int j = 0; j < retryCount + 1; j++)
  {
    i = 0;

    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.endTransmission();

    // Request num bytes from device
    Wire.requestFrom(device, num);

    // Device may send less than requested (abnormal)
    while (Wire.available())
    {
      // Receive one byte
      buf_p[i] = static_cast<uint8_t>(Wire.read());
      i++;
    }

    if (num == i)
    {
      return i;
    }

    if (j != retryCount)
    {
      delay(3);
    }
  }

  return i;
}


// ---- sensor drivers --------------------------------------------------------------

// DS1307 real time driver
bool readDS1307(datetime_t& dt)
{
  byte buf[8];

  if (I2CreadFrom(DS1307_ADDRESS, 0x00, 8, buf, 3) != 8)
  {
    Console.println("DS1307 comm error");
    //addErrorCounter(ERR_RTC_COMM);
    return false;
  }

  if ((buf[0] >> 7 != 0) ||
      (buf[1] >> 7 != 0) ||
      (buf[2] >> 7 != 0) ||
      (buf[3] >> 3 != 0) ||
      (buf[4] >> 6 != 0) ||
      (buf[5] >> 5 != 0) ||
      ((buf[7] & B01101100) != 0))
  {
    Console.println("DS1307 fata1 error");
    //addErrorCounter(ERR_RTC_DATA);
    return false;
  }

  datetime_t r;
  r.time.minute = static_cast<uint8_t>(
      10 * ((buf[1] >> 4) & B00000111) + (buf[1] & B00001111));
  r.time.hour = static_cast<uint8_t>(
      10 * ((buf[2] >> 4) & B00000111) + (buf[2] & B00001111));
  r.date.dayOfWeek = (buf[3] & B00000111);
  r.date.day = static_cast<uint8_t>(
      10 * ((buf[4] >> 4) & B00000011) + (buf[4] & B00001111));
  r.date.month = static_cast<uint8_t>(
      10 * ((buf[5] >> 4) & B00000001) + (buf[5] & B00001111));
  r.date.year = static_cast<uint16_t>(
      10 * ((buf[6] >> 4) & B00001111) + (buf[6] & B00001111));

  if ((r.time.minute > 59) ||
      (r.time.hour > 23) ||
      (r.date.dayOfWeek > 6) ||
      (r.date.month > 12) ||
      (r.date.day > 31) ||
      (r.date.day < 1) ||
      (r.date.month < 1) ||
      (r.date.year > 99))
  {
    Console.println("DS1307 data2 error");
    //addErrorCounter(ERR_RTC_DATA);
    return false;
  }

  r.date.year = static_cast<uint16_t>(r.date.year + 2000u);
  dt = r;

  return true;
}


bool setDS1307(const datetime_t& dt)
{
  byte buf[7];

  if (I2CreadFrom(DS1307_ADDRESS, 0x00, 7, buf, 3) != 7)
  {
    Console.println("DS1307 comm error");
    //addErrorCounter(ERR_RTC_COMM);
    return false;
  }

  buf[0] = buf[0] & B01111111; // enable clock
  buf[1] = static_cast<uint8_t>(
      ((dt.time.minute / 10) << 4) | (dt.time.minute % 10));
  buf[2] = static_cast<uint8_t>(
      ((dt.time.hour / 10) << 4) | (dt.time.hour % 10));
  buf[3] = dt.date.dayOfWeek;
  buf[4] = static_cast<uint8_t>(
      ((dt.date.day / 10) << 4) | (dt.date.day % 10));
  buf[5] = static_cast<uint8_t>(
      ((dt.date.month / 10) << 4) | (dt.date.month % 10));
  buf[6] = static_cast<uint8_t>(
      ((dt.date.year % 100 / 10) << 4) | (dt.date.year % 10));

  I2CwriteTo(DS1307_ADDRESS, 0x00, 7, buf);

  return true;
}
