/*
 * EEPROM.h
 *
 *  Created on: May 3, 2020
 *      Author: ove
 *
 * Mocked version of EEPROM.h.
 */

#pragma once

#include "CppUTestExt/MockSupport.h"
#include <stdint.h>

// Mocked class -----------------------------------------------------------

struct EEPROMClass
{
  uint8_t read(int idx)
  {
    mock().actualCall("EEPROM::read")
        .withParameter("idx", idx);
    return static_cast<uint8_t>(mock().unsignedIntReturnValue());
  }

  void write(int idx, uint8_t val)
  {
    mock().actualCall("EEPROM::write")
        .withParameter("idx", idx)
        .withParameter("val", val);
  }

  template <typename T>
  T &get(int idx, T &t)
  {
    (void)t;

    mock().actualCall("EEPROM::get")
        .withParameter("idx", idx);
    return t;
  }

  template <typename T>
  const T &put(int idx, const T &t)
  {
    (void)t;

    mock().actualCall("EEPROM::put")
          .withParameter("idx", idx);
    return t;
  }
};

static EEPROMClass EEPROM;
