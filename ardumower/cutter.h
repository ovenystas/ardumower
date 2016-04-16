/*
 * cutter.h
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#ifndef CUTTER_H
#define CUTTER_H

#include "motor.h"

class Cutter
{
  public:
    Motor motor;
    bool enable;
    bool enableOverride;
};

#endif /* CUTTER_H */