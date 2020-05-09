#pragma once

#include <inttypes.h>
#include <stdio.h> // for size_t

#include "WString.h"
//#include "Printable.h"

#define DEC 10
#define HEX 16
#define OCT 8
//#ifdef BIN // Prevent warnings if BIN is previously defined in "iotnx4.h" or similar
//#undef BIN
//#endif
#define BIN 2

class Print
{
public:
  void setMockPrintFunctions(bool mockPrintFunctions = true)
  {
    m_mockPrintFunctions = mockPrintFunctions;
  }

  void setMockPrintToStdout(bool mockPrintToStdout = true)
  {
    m_mockPrintToStdout = mockPrintToStdout;
  }

protected:
  bool m_mockPrintFunctions { true };
  bool m_mockPrintToStdout { true };

private:
//    int write_error;
  size_t printNumber(uint32_t, uint8_t);
  size_t printFloat(double, uint8_t);
protected:
//    void setWriteError(int err = 1) { write_error = err; }
public:
//    Print() : write_error(0) {}

//    int getWriteError() { return write_error; }
//    void clearWriteError() { setWriteError(0); }

  virtual size_t write(uint8_t) = 0;
  virtual size_t write(uint32_t n) = 0;
  virtual size_t write(int32_t n) = 0;
  virtual size_t write(uint16_t n) = 0;
  virtual size_t write(int16_t n) = 0;

  size_t write(const char* str)
  {
    if (str == NULL)
    {
      return 0;
    }
    return write((const uint8_t*)(str), strlen(str));
  }

  virtual size_t write(const uint8_t* buffer, size_t size);

  size_t write(const char *buffer, size_t size)
  {
    return write((const uint8_t*)(buffer), size);
  }

  // default to zero, meaning "a single write may block"
  // should be overriden by subclasses with buffering
  virtual int16_t availableForWrite()
  {
    return 0;
  }

//    size_t print(const __FlashStringHelper *);
  size_t print(const String &);
  size_t print(const char[]);
  size_t print(char);
  size_t print(uint8_t, int16_t = DEC);
  size_t print(int16_t, int16_t = DEC);
  size_t print(uint16_t, int16_t = DEC);
  size_t print(int32_t, int16_t = DEC);
  size_t print(uint32_t, int16_t = DEC);
  size_t print(double, int16_t = 2);
//    size_t print(const Printable&);

//    size_t println(const __FlashStringHelper *);
  size_t println(const String &s);
  size_t println(const char[]);
  size_t println(char);
  size_t println(uint8_t, int16_t = DEC);
  size_t println(int16_t, int16_t = DEC);
  size_t println(uint16_t, int16_t = DEC);
  size_t println(int32_t, int16_t = DEC);
  size_t println(uint32_t, int16_t = DEC);
  size_t println(double, int16_t = 2);
//    size_t println(const Printable&);
  size_t println(void);

  virtual void flush()
  {
    /* Empty implementation for backward compatibility */
  }
};
