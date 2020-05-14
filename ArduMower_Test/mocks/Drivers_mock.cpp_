/*
 * Drivers_mock.cpp
 *
 *  Created on: May 5, 2020
 *      Author: ove
 */

#include "CppUTestExt/MockSupport.h"
#include "Drivers.h"
#include "pgmspace.h"

void StreamPrint_progmem(Print &out, PGM_P format, ...)
{
//  mock().installComparator("printComparator", printComparator);
//  mock().actualCall("StreamPrint_progmem_")
//      .withParameterOfType("printComparator", "out", &out)
//      .withParameter("format", format);

  // program memory version of printf - copy of format string and result share a
  // buffer so as to avoid too much memory use
  char formatString[128];
  char *ptr;

  strncpy(formatString, format, sizeof(formatString)); // copy in from program mem

  // null terminate - leave char since we might need it in worst case for result's \0
  formatString[sizeof(formatString) - 2] = '\0';
  ptr = &formatString[strlen(formatString) + 1];// our result buffer...

  va_list args;
  va_start (args, format);
  vsnprintf(ptr,
            sizeof(formatString) - 1 - strlen(formatString),
            formatString, args );
  va_end (args);

  formatString[sizeof(formatString) - 1] = '\0';
  out.print(ptr);
}

float scalePI(float v)
{
  float d = v;
  while (d < 0)
  {
    d += 2 * PI;
  }
  while (d >= 2 * PI)
  {
    d -= 2 * PI;
  }

  if (d >= PI)
  {
    return (-2 * PI + d);
  }
  else if (d < -PI)
  {
    return (2 * PI + d);
  }
  else
  {
    return d;
  }
}
