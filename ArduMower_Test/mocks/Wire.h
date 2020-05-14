/*
 * Wire.h
 *
 *  Created on: May 4, 2020
 *      Author: ove
 */

#pragma once

#include "CppUTestExt/MockSupport.h"
#include <inttypes.h>
#include "Stream.h"

class TwoWire : public Stream
{
public:
  virtual ~TwoWire() {};

  void beginTransmission(uint8_t);
  uint8_t endTransmission(void);

  size_t write(uint8_t) override;

  size_t write(uint32_t n) override
  {
    return write(static_cast<uint8_t>(n));
  }

  size_t write(int32_t n) override
  {
    return write(static_cast<uint8_t>(n));
  }

  size_t write(uint16_t n) override
  {
    return write(static_cast<uint8_t>(n));
  }

  size_t write(int16_t n) override
  {
    return write(static_cast<uint8_t>(n));
  }

  using Print::write;

  uint8_t requestFrom(uint8_t, uint8_t);

  int16_t available(void) override;
  int16_t read(void) override;
  int16_t peek() override;
};

extern TwoWire Wire;
