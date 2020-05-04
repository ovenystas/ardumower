#pragma once

#include <inttypes.h>
#include "Stream.h"

// Define config for Serial.begin(baud, config);
#define SERIAL_5N1 0x00
#define SERIAL_6N1 0x02
#define SERIAL_7N1 0x04
#define SERIAL_8N1 0x06
#define SERIAL_5N2 0x08
#define SERIAL_6N2 0x0A
#define SERIAL_7N2 0x0C
#define SERIAL_8N2 0x0E
#define SERIAL_5E1 0x20
#define SERIAL_6E1 0x22
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_5E2 0x28
#define SERIAL_6E2 0x2A
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_5O1 0x30
#define SERIAL_6O1 0x32
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_5O2 0x38
#define SERIAL_6O2 0x3A
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E

class HardwareSerial : public Stream
{
  public:
    void begin(uint32_t baud, uint8_t config)
    {
      mBaud = baud;
      mConfig = config;
    }

    void begin(uint32_t baud)
    {
      mBaud = baud;
    }

    int16_t available(void);

    int16_t read(void);

    // Extra helper functions.
    uint32_t getBaud()
    {
      return mBaud;
    }

    uint8_t getConfig()
    {
      return mConfig;
    }

  private:
    uint32_t mBaud;
    uint8_t mConfig;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
