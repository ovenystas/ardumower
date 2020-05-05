/*
 * Drivers_mock.cpp
 *
 *  Created on: May 5, 2020
 *      Author: ove
 */

#include "CppUTestExt/MockSupport.h"
#include "Drivers.h"
#include "pgmspace.h"

class PrintComparator : public MockNamedValueComparator
{
public:
    virtual bool isEqual(const void* object1, const void* object2)
    {
        return object1 == object2;
    }
    virtual SimpleString valueToString(const void* object)
    {
        return StringFrom(object);
    }
};

PrintComparator printComparator;

void StreamPrint_progmem(Print &out, PGM_P format, ...)
{
  mock().installComparator("printComparator", printComparator);
  mock().actualCall("StreamPrint_progmem")
      .withParameterOfType("printComparator", "out", &out)
      .withParameter("format", format);
}
