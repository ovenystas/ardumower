/*
 * Print_mock.cpp
 *
 *  Created on: May 9, 2020
 *      Author: ove
 */

#include "CppUTestExt/MockSupport.h"
//#include <stdlib.h>
//#include <stdio.h>
//#include <string.h>
//#include <math.h>
#include "Arduino.h"

#include "Print.h"

// Public Methods //////////////////////////////////////////////////////////////

/* default implementation: may be overridden */
size_t Print::write(const uint8_t *buffer, size_t size)
{
  size_t n = 0;
  while (size--)
  {
    if (write(*buffer++))
    {
      n++;
    }
    else
    {
      break;
    }
  }
  return n;
}

//size_t Print::print(const __FlashStringHelper *ifsh)
//{
//  PGM_P p = reinterpret_cast<PGM_P>(ifsh);
//  size_t n = 0;
//  while (1) {
//    unsigned char c = pgm_read_byte(p++);
//    if (c == 0) break;
//    if (write(c)) n++;
//    else break;
//  }
//  return n;
//}

size_t Print::print(const String &s)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::print")
        .onObject(this)
        .withParameter("s", s)
        .returnUnsignedLongIntValue();
  }
  return write(s.c_str(), s.length());
}

size_t Print::print(const char str[])
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::print")
        .onObject(this)
        .withParameter("str", str)
        .returnUnsignedLongIntValue();
  }
  return write(str);
}

size_t Print::print(char c)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::print")
        .onObject(this)
        .withParameter("c", c)
        .returnUnsignedLongIntValue();
  }
  return write(static_cast<uint8_t>(c));
}

size_t Print::print(uint8_t b, int16_t base)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::print")
        .onObject(this)
        .withParameter("b", b)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }
  return print(static_cast<uint32_t>(b), base);
}

size_t Print::print(int16_t n, int16_t base)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::print")
        .onObject(this)
        .withParameter("n", n)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }
  return print(static_cast<int32_t>(n), base);
}

size_t Print::print(uint16_t n, int16_t base)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::print")
        .onObject(this)
        .withParameter("n", n)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }
  return print(static_cast<uint32_t>(n), base);
}

size_t Print::print(int32_t n, int16_t base)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::print")
        .onObject(this)
        .withParameter("n", n)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }

  if (base == 0)
  {
    return write(n);
  }
  else if (base == 10)
  {
    if (n < 0)
    {
      size_t t = print('-');
      n = -n;
      return printNumber(static_cast<uint32_t>(n), 10) + t;
    }
    return printNumber(static_cast<uint32_t>(n), 10);
  }
  else
  {
    return printNumber(static_cast<uint32_t>(n), static_cast<uint8_t>(base));
  }
}

size_t Print::print(uint32_t n, int16_t base)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::print")
        .onObject(this)
        .withParameter("n", n)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }

  if (base == 0)
  {
    return write(n);
  }
  else
  {
    return printNumber(n, static_cast<uint8_t>(base));
  }
}

size_t Print::print(double n, int16_t digits)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::print")
        .onObject(this)
        .withParameter("n", n)
        .withParameter("digits", digits)
        .returnUnsignedLongIntValue();
  }
  return printFloat(n, static_cast<uint8_t>(digits));
}

//size_t Print::println(const __FlashStringHelper *ifsh)
//{
//  size_t n = print(ifsh);
//  n += println();
//  return n;
//}

//size_t Print::print(const Printable& x)
//{
//  return x.printTo(*this);
//}

size_t Print::println(void)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::println")
        .onObject(this)
        .returnUnsignedLongIntValue();
  }
  return write("\r\n");
}

size_t Print::println(const String &s)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::println")
        .onObject(this)
        .withParameter("s", s)
        .returnUnsignedLongIntValue();
  }
  size_t n = print(s);
  n += println();
  return n;
}

size_t Print::println(const char c[])
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::println")
        .onObject(this)
        .withParameter("c", c)
        .returnUnsignedLongIntValue();
  }
  size_t n = print(c);
  n += println();
  return n;
}

size_t Print::println(char c)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::println")
        .onObject(this)
        .withParameter("c", c)
        .returnUnsignedLongIntValue();
  }
  size_t n = print(c);
  n += println();
  return n;
}

size_t Print::println(uint8_t b, int16_t base)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::println")
        .onObject(this)
        .withParameter("b", b)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }
  size_t n = print(b, base);
  n += println();
  return n;
}

size_t Print::println(int16_t num, int16_t base)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::println")
        .onObject(this)
        .withParameter("num", num)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(uint16_t num, int16_t base)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::println")
        .onObject(this)
        .withParameter("num", num)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(int32_t num, int16_t base)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::println")
        .onObject(this)
        .withParameter("num", num)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(uint32_t num, int16_t base)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::println")
        .onObject(this)
        .withParameter("num", num)
        .withParameter("base", base)
        .returnUnsignedLongIntValue();
  }
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(double num, int16_t digits)
{
  if (m_mockPrintFunctions)
  {
    return mock().actualCall("Print::println")
        .onObject(this)
        .withParameter("num", num)
        .withParameter("digits", digits)
        .returnUnsignedLongIntValue();
  }
  size_t n = print(num, digits);
  n += println();
  return n;
}

//size_t Print::println(const Printable& x)
//{
//  size_t n = print(x);
//  n += println();
//  return n;
//}

// Private Methods /////////////////////////////////////////////////////////////

size_t Print::printNumber(uint32_t n, uint8_t base)
{
  char buf[8 * sizeof(int32_t) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do
  {
    char c = static_cast<char>(n % base);
    n /= base;

    *--str = c < 10 ?
        static_cast<char>(c + '0') :
        static_cast<char>(c + 'A' - 10);
  } while (n);

  return write(str);
}

size_t Print::printFloat(double number, uint8_t digits)
{
  size_t n = 0;

  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
  if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
  if (number <-4294967040.0) return print ("ovf");  // constant determined empirically

  // Handle negative numbers
  if (number < 0.0)
  {
     n += print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  uint32_t int_part = static_cast<uint32_t>(number);
  double remainder = number - static_cast<double>(int_part);
  n += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    n += print('.');
  }

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    unsigned int toPrint = static_cast<uint16_t>(remainder);
    n += print(toPrint);
    remainder -= toPrint;
  }

  return n;
}
