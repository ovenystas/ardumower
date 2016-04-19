/*
 * cutter.h
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#ifndef CUTTER_H
#define CUTTER_H

#include "Motor.h"

class Cutter
{
  public:
    Motor motor;
    bool enable{false};
    bool enableOverride{false};
};

#endif /* CUTTER_H */
