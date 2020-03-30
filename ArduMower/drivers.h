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

// drivers (motor driver, sensor drivers, etc.)
#pragma once

#include <Arduino.h>
#include <EEPROM.h>

#define Console Serial
#define Bluetooth Serial2

// I2C addresses
#define STEPPER_ADDRESS 0xBB
#define DS1307_ADDRESS B1101000

// ---------- date time --------------------------------------

extern const char *dayOfWeek[];

typedef struct timehm_t
{
  byte hour;
  byte minute;
} timehm_t;

typedef struct date_t
{
  byte dayOfWeek;
  byte day;
  byte month;
  short year;
} date_t;

typedef struct datetime_t
{
  timehm_t time;
  date_t date;
} datetime_t;


// ---------- timers --------------------------------------

typedef struct ttimer_t
{
  bool active;
  timehm_t startTime;
  timehm_t stopTime;
  byte daysOfWeek;
} ttimer_t;


// ---- other ----------------------------------

// returns sign of variable (-1, 0, +1)
template<typename T> int8_t sign(T val)
{
  return (T(0) < val) - (val < T(0));
}


// ---------- EEPROM helpers ----------------------------------

template<class T> int eewrite(int &ee, const T& value)
{
  const byte* p = (const byte*) (const void*) &value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
  {
    EEPROM.write(ee++, *p++);
  }
  return i;
}

template<class T> int eeread(int &ee, T& value)
{
  byte* p = (byte*) (void*) &value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
  {
    *p++ = EEPROM.read(ee++);
  }
  return i;
}

template<class T> int eereadwrite(bool readflag, int& ee, T& value)
{
  byte* p = (byte*) (void*) &value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
  {
    if (readflag)
    {
      *p++ = EEPROM.read(ee++);
    }
    else
    {
      EEPROM.write(ee++, *p++);
    }
  }
  return i;
}


// ---------- driver functions ----------------------------------

int freeRam(void);

// print helpers
void StreamPrint_progmem(Print &out, PGM_P format, ...);
#define Serialprint(format, ...) StreamPrint_progmem(Serial, PSTR(format), ##__VA_ARGS__)
#define Streamprint(stream, format, ...) StreamPrint_progmem(stream, PSTR(format), ##__VA_ARGS__)
String verToString(const int v);

// time helpers
void minutes2time(const int minutes, timehm_t &time);
int time2minutes(const timehm_t time);
String time2str(const timehm_t time);
String date2str(const date_t date);

// I2C helpers
void I2CwriteTo(const uint8_t device, const uint8_t address, const uint8_t val);
int I2CreadFrom(const uint8_t device, const uint8_t address, const uint8_t num,
                uint8_t buff[], const int retryCount = 0);

// rescale to -PI..+PI
double scalePI(const double v);

// computes minimum distance between x radiant (current-value) and w radiant (set-value)
double distancePI(const double x, const double w);

// real time drivers
bool readDS1307(datetime_t& dt);
bool setDS1307(const datetime_t& dt);

// Returns the day of week (0=Sunday, 6=Saturday) for a given date
int getDayOfWeek(int month, const int day, int year, const int CalendarSystem);
