#pragma once

#include <inttypes.h>
#include <Stream.h>
#include <string>

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
  void begin(uint32_t baud)
  {
    m_baud = baud;
  }

  void begin(uint32_t baud, uint8_t config);

  void end();

  int16_t available(void) override;
  int16_t peek(void) override;
  int16_t read(void) override;

  int16_t availableForWrite(void);
  void flush(void);

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

  using Print::write; // pull in write(str) and write(buf, size) from Print

  operator bool()
  {
    return true;
  }

public:
  // Extra helper functions.
  uint32_t getBaud()
  {
    return m_baud;
  }

  uint8_t getConfig()
  {
    return m_config;
  }

  const char* getMockOutString()
  {
    if (m_mockOutString_p)
    {
      return m_mockOutString_p->c_str();
    }
    else
    {
      return "";
    }
  }

  void clearMockOutString()
  {
    if (m_mockOutString_p)
    {
      m_mockOutString_p->clear();
    }
  }

  void startCapture()
  {
    m_mockOutString_p = new std::string();
  }

  void stopCapture()
  {
    if (m_mockPrintToStdout)
    {
      flush();
    }

    if (m_mockOutString_p)
    {
      delete m_mockOutString_p;
    }
    m_mockOutString_p = nullptr;
  }

private:
    uint32_t m_baud;
    uint8_t m_config;
    std::string* m_mockOutString_p;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
