/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat
 Copyright (c) 2014 by Maxime Carpentieri

 Private-use only! (you need to ask for a commercial-use)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 Private-use only! (you need to ask for a commercial-use)
 */

#include <limits.h>

#include "AdcManager.h"

#include "drivers.h"

volatile uint8_t position = 0;
volatile uint8_t channel = 0;
volatile bool busy = false;
uint8_t captureSize[CHANNELS] = {0};      // ADC capture buffer size (ADC0-ADC15)
bool captureComplete[CHANNELS] = {false}; // ADC buffer filled?
int16_t* sample[CHANNELS] = {0};          // ADC one sample (ADC0-ADC15) - 10 bit unsigned
int8_t* capture[CHANNELS] = {0};          // ADC capture buffer (ADC0-ADC15) - 8 bit signed (signed: zero = ADC/2)
int16_t zeroOffset[CHANNELS] = {0};       // ADC zero offset (ADC0-ADC15)
int16_t AdcMin[CHANNELS] = {9999};        // ADC min sample value (ADC-ADC15)
int16_t AdcMax[CHANNELS] = {-9999};       // ADC max sample value (ADC-ADC15)

AdcManager ADCMan;

void AdcManager::init(void)
{
  delay(500); // wait for ADCRef to settle (stable ADCRef required for later calibration)
  if (loadCalibration())
  {
    printCalibration();
  }
}

void AdcManager::setCapture(const uint8_t pin,
                            const uint8_t samplecount,
                            const bool autoCalibrate)
{
  const uint8_t ch = pin - A0;
  captureSize[ch] = samplecount;
  capture[ch] = new int8_t[samplecount];
  sample[ch] = new int16_t[samplecount];
  this->m_autoCalibrate[ch] = autoCalibrate;
  Console.print("ADC setCapture:");
  Console.print(" pin=");
  Console.print(pin);
  Console.print(" samplecount=");
  Console.print(samplecount);
  Console.print(" autoCalibrate=");
  Console.print(this->m_autoCalibrate[ch]);
  Console.print(" ch=");
  Console.println(ch);
}

void AdcManager::calibrate(void)
{
  Console.println("ADC calibration...");
  for (uint8_t ch = 0; ch < CHANNELS; ch++)
  {
    AdcMax[ch] = -9999;
    AdcMin[ch] = 9999;
    zeroOffset[ch] = 0;
    if (m_autoCalibrate[ch])
    {
      calibrateZeroOffset(ch);
    }
  }
  printCalibration();
  saveCalibration();
  m_calibrationAvailable = true;
}

void AdcManager::calibrateZeroOffset(const uint8_t ch)
{
  for (uint8_t i = 0; i < 10; i++)
  {
    captureComplete[ch] = false;
    while (!captureComplete[ch])
    {
      delay(20);
      run();
    }
  }
  const int16_t center = AdcMin[ch] + (AdcMax[ch] - AdcMin[ch]) / 2;
  zeroOffset[ch] = center;

  Console.print(F("ADC calibration ch"));
  Console.print(ch);
  Console.print(F("="));
  Console.println(zeroOffset[ch]);
}

void AdcManager::printCalibration(void)
{
  Console.println(F("---ADC calib---"));
  Console.print(F("ADC sampleRate="));
  switch (SAMPLE_RATE)
  {
    case SRATE_38462:
      Console.println(F("38462"));
      break;
    case SRATE_19231:
      Console.println(F("19231"));
      break;
    case SRATE_9615:
      Console.println(F("9615"));
      break;
  }
  for (int ch = 0; ch < CHANNELS; ch++)
  {
    Console.print(F("AD"));
    Console.print(ch);
    Console.print(F(", "));
    Console.print(F("min="));
    Console.print(AdcMin[ch]);
    Console.print(F(", "));
    Console.print(F("max="));
    Console.print(AdcMax[ch]);
    Console.print(F(", "));
    Console.print(F("diff="));
    Console.print(AdcMax[ch] - AdcMin[ch]);
    Console.print(F(", "));
    Console.print(F("ofs="));
    Console.println(zeroOffset[ch]);
  }
}

void AdcManager::startADC(void)
{
//  Console.print("startADC ch");
//  Console.println(channel);
  // http://www.atmel.com/images/doc2549.pdf
  /*
   *  REFS0    : VCC use as a ref,
   *  IR_AUDIO : channel selection,
   *  ADEN     : ADC Enable,
   *  ADSC     : ADC Start,
   *  ADATE    : ADC Auto Trigger Enable,
   *  ADIE     : ADC Interrupt Enable,
   *  ADPS     : ADC Prescaler
   */
  // free running ADC mode,
  // f = ( 16MHz / prescaler ) / 13 cycles per conversion
  ADMUX = _BV(REFS0) | (channel & 0x07);
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((channel >> 3) & 0x01) << MUX5);

  const uint8_t sampleCount = captureSize[channel];

  // use slow but accurate sampling if one sample only
  if (sampleCount == 1)
  {
    // slow but accurate
    // prescaler 128 : 9615 Hz
    ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) |
             _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  }
  else
  {
    switch (SAMPLE_RATE)
    {
      case SRATE_38462:
        ADCSRA = _BV(ADSC) | _BV(ADEN)  | _BV(ADATE) | _BV(ADIE) |
                 _BV(ADPS2) | _BV(ADPS0);
        break; // prescaler 32 : 38462 Hz

      case SRATE_19231:
        ADCSRA = _BV(ADSC) | _BV(ADEN)  | _BV(ADATE) | _BV(ADIE) |
                 _BV(ADPS2) | _BV(ADPS1);
        break; //prescaler 64 : 19231 Hz

      case SRATE_9615:
        ADCSRA = _BV(ADSC) | _BV(ADEN)  | _BV(ADATE) | _BV(ADIE) |
                 _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
        break; // prescaler 128 : 9615 Hz
    }
  }

  // disable digital buffers (reduces noise/capacity)
  if (channel < 8)
  {
    DIDR0 |= (1 << channel);
  }
  else
  {
    DIDR2 |= (1 << (channel - 8));
  }
  //sei();
}

void AdcManager::startCapture(void)
{
//  Console.print("starting capture ch");
//  Console.println(channel);
  position = 0;
  busy = true;
  startADC();
}

// free running ADC fills capture buffer
ISR(ADC_vect)
{
  volatile int16_t value = ADC;

  if (!busy)
  {
    return;
  }

  if (position >= captureSize[channel])
  {
    // stop capture
    captureComplete[channel] = true;
    busy = false;
    return;
  }
  value -= zeroOffset[channel];
  // convert to signed (zero = ADC/2)
  capture[channel][position] = min(SCHAR_MAX, max(SCHAR_MIN, value / 4));
  sample[channel][position] = value;

  // determine min/max
  if (value < AdcMin[channel])
  {
    AdcMin[channel] = value;
  }

  if (value > AdcMax[channel])
  {
    AdcMax[channel] = value;
  }

  ++position;
}

void AdcManager::stopCapture(void)
{
//  Console.print("stopping capture ch");
//  Console.println(channel);
  position = 0;
  ADCSRA &= ~_BV(ADEN);
}

void AdcManager::run(void)
{
//  Console.print("AdcManager::run: ch=");
//  Console.print(channel);
//  Console.print(" pos=");
//  Console.println(position);
  if (busy)
  {
//    Console.print("busy pos=");
//    Console.println(position);
    return;
  }

  if (position != 0)
  {
    // Stop free running
    stopCapture();
    m_capturedChannels++;
  }

  // Find next channel for capturing
  for (uint8_t i = 0; i < CHANNELS; i++)
  {
    ++channel;
    if (channel == CHANNELS)
    {
      channel = 0;
    }

    if (captureSize[channel] != 0 && !captureComplete[channel])
    {
      // Found channel for sampling
      startCapture();
      break;
    }
  }
}

int16_t AdcManager::read(const uint8_t pin)
{
  const uint8_t ch = pin - A0;
  captureComplete[ch] = false;
  if (captureSize[ch] == 0)
  {
    return 0;
  }
  else
  {
    return sample[ch][captureSize[ch] - 1];
  }
}

int16_t AdcManager::readMedian(const uint8_t pin)
{
  const uint8_t ch = pin - A0;
  captureComplete[ch] = false;
  if (captureSize[ch] == 0)
  {
    return 0;
  }
  else if (captureSize[ch] == 1)
  {
    return sample[ch][0];
  }
  else
  {
    for (uint8_t i = 1; i < captureSize[ch]; ++i)
    {
      const int16_t j = sample[ch][i];
      int8_t k;
      for (k = i - 1; (k >= 0) && (j > sample[ch][k]); k--)
      {
        sample[ch][k + 1] = sample[ch][k];
      }
      sample[ch][k + 1] = j;
    }
    return sample[ch][captureSize[ch] / 2];
  }
}

uint8_t AdcManager::getCapturedChannels(void)
{
  const uint8_t res = m_capturedChannels;
  m_capturedChannels = 0;
  return res;
}

int16_t AdcManager::getAdcMinCh(const uint8_t ch)
{
  if (ch >= CHANNELS)
  {
    return 0;
  }
  return AdcMin[ch];
}

int16_t AdcManager::getAdcMaxCh(const uint8_t ch)
{
  if (ch >= CHANNELS)
  {
    return 0;
  }
  return AdcMax[ch];
}

int16_t AdcManager::getAdcZeroOffsetCh(const uint8_t ch)
{
  if (ch >= CHANNELS)
  {
    return 0;
  }
  return zeroOffset[ch];
}

void AdcManager::restart(const uint8_t pin)
{
  captureComplete[pin - A0] = false;
}

bool AdcManager::isCaptureCompleteCh(const uint8_t ch)
{
  return captureComplete[ch];
}

uint8_t AdcManager::getCaptureSize(const uint8_t pin)
{
  return captureSize[pin - A0];
}

int8_t* AdcManager::getCapture(const uint8_t pin)
{
  return capture[pin - A0];
}

void AdcManager::loadSaveCalibration(const bool readflag)
{
  uint16_t addr = ADDR;
  uint8_t magic = MAGIC;
  eereadwrite(readflag, addr, magic);
  for (uint8_t ch = 0; ch < CHANNELS; ch++)
  {
    eereadwrite(readflag, addr, zeroOffset[ch]);
  }
}

bool AdcManager::loadCalibration(void)
{
  uint8_t magic = 0;
  uint16_t addr = ADDR;
  eeread(addr, magic);
  if (magic != MAGIC)
  {
    Console.println(F("ADCMan Error: No calibration data"));
    return false;
  }
  m_calibrationAvailable = true;
  Console.println(F("ADCMan: Found calibration data"));
  loadSaveCalibration(true);
  return true;
}

void AdcManager::saveCalibration(void)
{
  loadSaveCalibration(false);
  Console.println(F("ADC calibration data saved"));
}
