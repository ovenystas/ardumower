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
    mock().actualCall("EEPROM_read")
        .withParameter("idx", idx);
    return static_cast<uint8_t>(mock().unsignedIntReturnValue());
  }

  void write(int idx, uint8_t val)
  {
    mock().actualCall("EEPROM_write")
        .withParameter("idx", idx)
        .withParameter("val", val);
  }
};

static EEPROMClass EEPROM;
