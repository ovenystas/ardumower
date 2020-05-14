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

enum class CalendarSystem
{
  Julian = -1,
  Gregorian = 0
};

enum class Weekdays
{
  Monday = 0,
  Tuesday,
  Wednesday,
  Thursday,
  Friday,
  Saturday,
  Sunday
};

struct timehm_t
{
  uint8_t hour;
  uint8_t minute;
};

struct date_t
{
  uint8_t dayOfWeek;
  uint8_t day;
  uint8_t month;
  uint16_t year;
};

struct datetime_t
{
  timehm_t time;
  date_t date;
};


// ---------- timers --------------------------------------

struct ttimer_t
{
  bool active;
  timehm_t startTime;
  timehm_t stopTime;
  uint8_t daysOfWeek; // Bit-field b7=unused, b6=Saturday, b0=Sunday
};


// ---- other ----------------------------------

// returns sign of variable (-1, 0, +1)
template <typename T>
int8_t sign(T val)
{
  return static_cast<int8_t>((T(0) < val) - (val < T(0)));
}


// ---------- EEPROM helpers ----------------------------------

template <class T>
void eewrite(uint16_t &eeIdx, const T& value)
{
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
  for (int count = sizeof(value); count; --count)
  {
    EEPROM.write(eeIdx++, *p++);
  }
}

template <class T>
void eeread(uint16_t &eeIdx, T& value)
{
  uint8_t* p = reinterpret_cast<uint8_t*>(&value);
  for (int count = sizeof(value); count; --count)
  {
    *p++ = EEPROM.read(eeIdx++);
  }
}

template <class T>
void eereadwrite(const bool readflag, uint16_t& eeIdx, T& value)
{
  if (readflag)
  {
    eeread(eeIdx, value);
  }
  else
  {
    eewrite(eeIdx, value);
  }
}


// ---------- driver functions ----------------------------------

int freeRam(void);

// print helpers
void StreamPrint_progmem(Print &out, PGM_P format, ...);
#define Serialprint(format, ...) StreamPrint_progmem(Serial, PSTR(format), ##__VA_ARGS__)
#define Streamprint(stream, format, ...) StreamPrint_progmem(stream, PSTR(format), ##__VA_ARGS__)

// time helpers
void minutes2time(int minutes, timehm_t& time);
int time2minutes(const timehm_t& time);
String time2str(const timehm_t& time);
String date2str(const date_t& date);

// I2C helpers
void I2CwriteTo(uint8_t device, uint8_t address, uint8_t val);
void I2CwriteTo(uint8_t device, uint8_t address, int num, const uint8_t* buf_p);
int I2CreadFrom(uint8_t device, uint8_t address, uint8_t num, uint8_t* buf_p,
    int retryCount = 0);

// rescale to -PI..+PI
float scalePI(float v);

// computes minimum distance between x radiant (current-value) and w radiant (set-value)
float distancePI(float x, float w);

// real time drivers
bool readDS1307(datetime_t& dt);
bool setDS1307(const datetime_t& dt);

// Returns the day of week (0=Monday, 6=Sunday) for a given date
uint8_t getDayOfWeek(uint16_t year, uint8_t month, uint8_t day,
    CalendarSystem calendarSystem = CalendarSystem::Gregorian);
