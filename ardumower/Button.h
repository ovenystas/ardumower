/*
 * button.h
 *
 *  Created on: Mar 10, 2016
 *      Author: ove
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

extern bool button_use;

void button_setup(void);
bool button_isPressed(void);
bool button_isTimeToRun(void);
uint8_t button_getCounter(void);
void button_incCounter(void);
void button_clearCounter(void);

#endif /* BUTTON_H */
