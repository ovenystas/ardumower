/*
 * Arduino.h
 *
 *  Created on: 26 sep. 2017
 *      Author: oveny
 */

#ifndef ARDUINO_H_
#define ARDUINO_H_

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

unsigned long micros(void);

#endif /* ARDUINO_H_ */
