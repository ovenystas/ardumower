/*
 * wiring.cpp
 *
 *  Created on: 4 okt. 2017
 *      Author: oveny
 */

#include "CppUTestExt/MockSupport.h"
#include <stdint.h>

// Mocked functions -----------------------------------------------------------

unsigned long micros()
{
    mock().actualCall("micros");
    return mock().unsignedLongIntReturnValue();
}
