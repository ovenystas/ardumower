/*
 * Arduino.h
 *
 *  Created on: 26 sep. 2017
 *      Author: oveny
 */

#ifndef ARDUINO_H_
#define ARDUINO_H_

#include <stdint.h>
#include <math.h>

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define PI 3.1415926535897932384626433832795f
#define HALF_PI 1.5707963267948966192313216916398f
#define TWO_PI 6.283185307179586476925286766559f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f
#define EULER 2.718281828459045235360287471352f

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(int32_t)((x)+0.5):(int32_t)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

typedef uint16_t word; // 16 bit on ATMEGA, 32 bit on Due, Zero

#define bit(b) (1UL << (b))

typedef bool boolean;
typedef uint8_t byte;

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int16_t digitalRead(uint8_t);
int16_t analogRead(uint8_t);
void analogReference(uint8_t mode);
void analogWrite(uint8_t, int16_t);

uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t);
void delayMicroseconds(uint16_t us);
uint32_t pulseIn(uint8_t pin, uint8_t state, uint32_t timeout);
uint32_t pulseInLong(uint8_t pin, uint8_t state, uint32_t timeout);

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

uint8_t digitalPinToBitMask(uint8_t pin);
uint8_t digitalPinToPort(uint8_t pin);
uint8_t* portOutputRegister(uint8_t pin);
uint8_t* portInputRegister(uint8_t pin);

#endif /* ARDUINO_H_ */
