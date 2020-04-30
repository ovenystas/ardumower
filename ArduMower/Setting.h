/*
 * Settings.h
 *
 *  Created on: Apr 13, 2020
 *      Author: ove
 */

#pragma once

#include <Arduino.h>

#if 0
template <class T>
struct Setting
{
  const String name;
  const String unit;
  T value;
  const T minValue;
  T maxValue;
  float scale;

  Setting(const String _name, const String _unit, T _value,
      const T _minValue = 0, const T _maxValue = 0, const float _scale = 1.0 ) :
        name(_name), unit(_unit), value(_value),
        minValue(_minValue), maxValue(_maxValue), scale(_scale) {}
};
#endif

template <class T>
struct Setting
{
  const String name;
  T value;

  Setting(const String _name, T _value) :
        name(_name), value(_value) {}
};

template <>
struct Setting<int8_t>
{
  const String name;
  const String unit;
  int8_t value;
  const int8_t minValue;
  int8_t maxValue;
  float scale;

  Setting(const String _name, const String _unit, int8_t _value,
      const int8_t _minValue = INT8_MIN, const int8_t _maxValue = INT8_MAX,
      const float _scale = 1.0 ) :
        name(_name), unit(_unit), value(_value),
        minValue(_minValue), maxValue(_maxValue),
        scale(_scale) {}
};

template <>
struct Setting<uint8_t>
{
  const String name;
  const String unit;
  uint8_t value;
  const uint8_t minValue;
  uint8_t maxValue;
  float scale;

  Setting(const String _name, const String _unit, uint8_t _value,
      const uint8_t _minValue = 0, const uint8_t _maxValue = UINT8_MAX,
      const float _scale = 1.0 ) :
        name(_name), unit(_unit), value(_value),
        minValue(_minValue), maxValue(_maxValue),
        scale(_scale) {}
};

template <>
struct Setting<int16_t>
{
  const String name;
  const String unit;
  int16_t value;
  const int16_t minValue;
  int16_t maxValue;
  float scale;

  Setting(const String _name, const String _unit, int16_t _value,
      const int16_t _minValue = INT16_MIN, const int16_t _maxValue = INT16_MAX,
      const float _scale = 1.0 ) :
        name(_name), unit(_unit), value(_value),
        minValue(_minValue), maxValue(_maxValue),
        scale(_scale) {}
};

template <>
struct Setting<uint16_t>
{
  const String name;
  const String unit;
  uint16_t value;
  const uint16_t minValue;
  uint16_t maxValue;
  float scale;

  Setting(const String _name, const String _unit, uint16_t _value,
      const uint16_t _minValue = 0, const uint16_t _maxValue = UINT16_MAX,
      const float _scale = 1.0 ) :
        name(_name), unit(_unit), value(_value),
        minValue(_minValue), maxValue(_maxValue),
        scale(_scale) {}
};

template <>
struct Setting<int32_t>
{
  const String name;
  const String unit;
  int32_t value;
  const int32_t minValue;
  int32_t maxValue;
  float scale;

  Setting(const String _name, const String _unit, int32_t _value,
      const int32_t _minValue = INT32_MIN, const int32_t _maxValue = INT32_MAX,
      const float _scale = 1.0 ) :
        name(_name), unit(_unit), value(_value),
        minValue(_minValue), maxValue(_maxValue),
        scale(_scale) {}
};

template <>
struct Setting<uint32_t>
{
  const String name;
  const String unit;
  uint32_t value;
  const uint32_t minValue;
  uint32_t maxValue;
  float scale;

  Setting(const String _name, const String _unit, uint32_t _value,
      const uint32_t _minValue = 0, const uint32_t _maxValue = UINT32_MAX,
      const float _scale = 1.0 ) :
        name(_name), unit(_unit), value(_value),
        minValue(_minValue), maxValue(_maxValue),
        scale(_scale) {}
};

template <>
struct Setting<float>
{
  const String name;
  const String unit;
  float value;
  const float minValue;
  float maxValue;
  float scale;

  Setting(const String _name, const String _unit, float _value,
      const float _minValue = 0.0f, const float _maxValue = 0.0f,
      const float _scale = 1.0 ) :
        name(_name), unit(_unit), value(_value),
        minValue(_minValue), maxValue(_maxValue),
        scale(_scale) {}
};

