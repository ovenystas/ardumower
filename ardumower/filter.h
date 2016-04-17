/*
 * filter.h
 *
 *  Created on: Apr 17, 2016
 *      Author: ove
 */

#ifndef FILTER_H
#define FILTER_H

#include <Arduino.h>

#define DSP_EMA_I32_ALPHA(x) ( (uint16_t)(x * 65535) )
#define DSP_EMA_I16_ALPHA(x) ( (uint8_t)(x * 255) )

int32_t dsp_ema_i32(int32_t in, int32_t average, uint16_t alpha);
int16_t dsp_ema_i16(int16_t in, int16_t average, uint8_t alpha);


#endif /* FILTER_H */
