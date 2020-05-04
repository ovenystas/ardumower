/*
 * Print.h
 *
 *  Created on: May 3, 2020
 *      Author: ove
 */

#pragma once

#include "CppUTestExt/MockSupport.h"
#include "WString.h"

#define DEC 10
#define HEX 16
#define OCT 8
#ifdef BIN // Prevent warnings if BIN is previously defined in "iotnx4.h" or similar
#undef BIN
#endif
#define BIN 2

class Print
{
public:

  template <class T>
  size_t print(T val)
  {
    return mock().actualCall("print")
        .onObject(this)
        .withParameter("val", val)
        .returnUnsignedLongIntValue();
  }

  template <class T>
  size_t print(T num, int16_t base)
  {
    return mock().actualCall("print")
        .onObject(this)
        .withParameter("num", num)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }

  size_t println()
  {
    return mock().actualCall("println")
        .onObject(this)
        .returnUnsignedLongIntValue();
  }

  template <class T>
  size_t println(T val)
  {
    return mock().actualCall("println")
        .onObject(this)
        .withParameter("val", val)
        .returnUnsignedLongIntValue();
  }

  template <class T>
  size_t println(T num, int16_t base)
  {
    return mock().actualCall("println")
        .onObject(this)
        .withParameter("num", num)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }
};


