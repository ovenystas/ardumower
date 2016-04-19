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

#include <Arduino.h>
#include <limits.h>

#include "AdcManager.h"

#include "drivers.h"

#define ADDR 500
#define MAGIC 1

#define CHANNELS 16

#define NO_CHANNEL 255

volatile short position = 0;
volatile int16_t lastvalue = 0;
volatile uint8_t channel = 0;
volatile boolean busy = false;
int8_t *capture[CHANNELS]; // ADC capture buffer (ADC0-ADC7) - 8 bit signed (signed: zero = ADC/2)
uint8_t captureSize[CHANNELS]; // ADC sample buffer size (ADC0-ADC7)
int16_t ofs[CHANNELS]; // ADC zero offset (ADC0-ADC7)
int16_t ADCMin[CHANNELS]; // ADC min sample value (ADC-ADC7)
int16_t ADCMax[CHANNELS]; // ADC max sample value (ADC-ADC7)
boolean captureComplete[CHANNELS]; // ADC buffer filled?
boolean autoCalibrate[CHANNELS]; // do auto-calibrate? (ADC0-ADC7)
int16_t *sample[CHANNELS];   // ADC one sample (ADC0-ADC7) - 10 bit unsigned
AdcManager ADCMan;

AdcManager::AdcManager()
{
  calibrationAvail = false;
  for (int i = 0; i < CHANNELS; i++)
  {
    captureSize[i] = 0;
    ofs[i] = 0;
    captureComplete[i] = false;
    capture[i] = NULL;
    autoCalibrate[i] = false;
    ADCMax[i] = -9999;
    ADCMin[i] = 9999;
  }
  capturedChannels = 0;
  // NOTE: when choosing a higher perimeter sample rate (38 kHz) and using odometer interrupts,
  // the Arduino Mega cannot handle all ADC interrupts anymore - the result will be a 'noisy'
  // perimeter filter output (mag value) which disappears when disabling odometer interrupts.
  // SOLUTION: allow odometer interrupt handler nesting (see odometer interrupt function)
  //sampleRate = SRATE_19231;
  sampleRate = SRATE_38462;
  //sampleRate = SRATE_9615;
}

void AdcManager::init()
{
  delay(500); // wait for ADCRef to settle (stable ADCRef required for later calibration)
  if (loadCalib())
  {
    printCalib();
  }
}

void AdcManager::setCapture(byte pin, byte samplecount,
                            boolean autoCalibrateOfs)
{
  int ch = pin - A0;
  captureSize[ch] = samplecount;
  capture[ch] = new int8_t[samplecount];
  sample[ch] = new int16_t[samplecount];
  autoCalibrate[ch] = autoCalibrateOfs;
}

void AdcManager::calibrate()
{
  Console.println("ADC calibration...");
  for (int ch = 0; ch < CHANNELS; ch++)
  {
    ADCMax[ch] = -9999;
    ADCMin[ch] = 9999;
    ofs[ch] = 0;
    if (autoCalibrate[ch])
    {
      calibrateOfs(A0 + ch);
    }
  }
  printCalib();
  saveCalib();
  calibrationAvail = true;
}

boolean AdcManager::calibrationDataAvail()
{
  return calibrationAvail;
}

void AdcManager::calibrateOfs(byte pin)
{
  int ch = pin - A0;
  ADCMax[ch] = -9999;
  ADCMin[ch] = 9999;
  ofs[ch] = 0;
  for (int i = 0; i < 10; i++)
  {
    captureComplete[ch] = false;
    while (!isCaptureComplete(pin))
    {
      delay(20);
      run();
    }
  }
  int16_t center = ADCMin[ch] + (ADCMax[ch] - ADCMin[ch]) / 2.0;
  ofs[ch] = center;

  Console.print(F("ADC calibration ch"));
  Console.print(ch);
  Console.print(F("="));
  Console.println(ofs[ch]);
}

void AdcManager::printCalib()
{
  Console.println(F("---ADC calib---"));
  Console.print(F("ADC sampleRate="));
  switch (sampleRate)
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
    Console.print(ADCMin[ch]);
    Console.print(F(", "));
    Console.print(F("max="));
    Console.print(ADCMax[ch]);
    Console.print(F(", "));
    Console.print(F("diff="));
    Console.print(ADCMax[ch] - ADCMin[ch]);
    Console.print(F(", "));
    Console.print(F("ofs="));
    Console.println(ofs[ch]);
  }
}

void AdcManager::startADC(int sampleCount)
{
//  Console.print("startADC ch");
//  Console.println(channel);
  // http://www.atmel.com/images/doc2549.pdf
  /*  REFS0 : VCC use as a ref, IR_AUDIO : channel selection, ADEN : ADC Enable, ADSC : ADC Start, ADATE : ADC Auto Trigger Enable, ADIE : ADC Interrupt Enable,  ADPS : ADC Prescaler  */
  // free running ADC mode, f = ( 16MHz / prescaler ) / 13 cycles per conversion
  ADMUX = _BV(REFS0) | (channel & 0x07); // | _BV(ADLAR);
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((channel >> 3) & 0x01) << MUX5);
  // use slow but accurate sampling if one sample only
  if (sampleCount == 1)
    // slow but accurate
    ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) |
             _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // prescaler 128 : 9615 Hz
  else
  {
    switch (sampleRate)
    {
      case SRATE_38462:
        ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) |
                 _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0);
        break; // prescaler 32 : 38462 Hz
      case SRATE_19231:
        ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) |
                 _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1);
        break; //prescaler 64 : 19231 Hz
      case SRATE_9615:
        ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) |
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

void AdcManager::startCapture(int sampleCount)
{
  //Console.print("starting capture ch");
  //Console.println(channel);
  position = 0;
  busy = true;
  startADC(sampleCount);
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
  value -= ofs[channel];
  capture[channel][position] = min(SCHAR_MAX, max(SCHAR_MIN, value / 4)); // convert to signed (zero = ADC/2)
  sample[channel][position] = value;
  // determine min/max
  if (value < ADCMin[channel])
  {
    ADCMin[channel] = value;
  }
  if (value > ADCMax[channel])
  {
    ADCMax[channel] = value;
  }
  position++;
}

void AdcManager::stopCapture()
{
  //Console.print("stopping capture ch");
  //Console.println(channel);
  position = 0;
  ADCSRA &= ~_BV(ADEN);
}

void AdcManager::run()
{
  if (busy)
  {
    //Console.print("busy pos=");
    //Console.println(position);
    return;
  }
  if (position != 0)
  {
    // stop free running
    stopCapture();
    capturedChannels++;
  }
  // find next channel for capturing
  for (int i = 0; i < CHANNELS; i++)
  {
    channel++;
    if (channel == CHANNELS)
    {
      channel = 0;
    }
    if (captureSize[channel] != 0 && !captureComplete[channel])
    {
      // found channel for sampling
      startCapture(captureSize[channel]);
      break;
    }
  }
}

int8_t* AdcManager::getCapture(byte pin)
{
  return (int8_t*)capture[pin - A0];
}

boolean AdcManager::isCaptureComplete(byte pin)
{
  int ch = pin - A0;
  return captureComplete[ch];
}

int AdcManager::read(byte pin)
{
  int ch = pin - A0;
  captureComplete[ch] = false;
  if (captureSize[ch] == 0)
  {
    return 0;
  }
  else
  {
    return sample[ch][(captureSize[ch] - 1)];
  }
}

int AdcManager::readMedian(byte pin)
{
  int ch = pin - A0;
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
    for (int i = 1; i < captureSize[ch]; ++i)
    {
      int j = sample[ch][i];
      int k;
      for (k = i - 1; (k >= 0) && (j > sample[ch][k]); k--)
      {
        sample[ch][k + 1] = sample[ch][k];
      }
      sample[ch][k + 1] = j;
    }
    return sample[ch][(int) (captureSize[ch] / 2)];
  }
}

void AdcManager::restart(byte pin)
{
  captureComplete[pin - A0] = false;
}

int AdcManager::getCapturedChannels()
{
  int res = capturedChannels;
  capturedChannels = 0;
  return res;
}

int AdcManager::getCaptureSize(byte pin)
{
  int ch = pin - A0;
  return captureSize[ch];

}

int16_t AdcManager::getADCMin(byte pin)
{
  int ch = pin - A0;
  if (ch >= CHANNELS)
  {
    return 0;
  }
  return ADCMin[ch];
}

int16_t AdcManager::getADCMax(byte pin)
{
  int ch = pin - A0;
  if (ch >= CHANNELS)
  {
    return 0;
  }
  return ADCMax[ch];
}

int16_t AdcManager::getADCOfs(byte pin)
{
  int ch = pin - A0;
  if (ch >= CHANNELS)
  {
    return 0;
  }
  return ofs[ch];
}

void AdcManager::loadSaveCalib(boolean readflag)
{
  int addr = ADDR;
  short magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  for (int ch = 0; ch < CHANNELS; ch++)
  {
    eereadwrite(readflag, addr, ofs[ch]);
  }
}

boolean AdcManager::loadCalib()
{
  short magic = 0;
  int addr = ADDR;
  eeread(addr, magic);
  if (magic != MAGIC)
  {
    Console.println(F("ADCMan error: no calib data"));
    return false;
  }
  calibrationAvail = true;
  Console.println(F("ADCMan: found calib data"));
  loadSaveCalib(true);
  return true;
}

void AdcManager::saveCalib()
{
  loadSaveCalib(false);
}
