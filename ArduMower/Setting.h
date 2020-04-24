/*
 * Settings.h
 *
 *  Created on: Apr 13, 2020
 *      Author: ove
 */

#pragma once

#include <Arduino.h>

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

