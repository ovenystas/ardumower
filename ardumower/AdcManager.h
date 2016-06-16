/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat

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
/*
 Problem: you have multiple analog inputs, some need only to be sampled once, other need
 a fixed sample rate.

 Solution:
 Arduino ADC manager (ADC0-ADC9)
 - can capture multiple pins one after the other (example ADC0: 1000 samples, ADC1: 100 samples, ADC2: 1 sample etc.)
 - can capture more than one sample into buffers (fixed sample rate)
 - runs in background: interrupt-based (free-running)
 - two types of ADC capture:
 1) free-running ADC capturing (for certain sample count) (8 bit signed - zero = VCC/2)
 2) ordinary ADC sampling (one-time sampling) (10 bit unsigned)
 - WARNING: never use any 'analogRead()' in your code when using this class!

 How to use it (example):
 1. Initialize ADC:  ADCMan.init();
 2. Set ADC pin:     ADCMan.setCapture(pinMotorMowSense, 1, 1);
 3. Program loop:    while (true){
 ADCMan.run();
 if (ADCMan.isCaptureComplete(pinMotorMowSense)){
 int value = ADCMan.read(pinMotorMowSense);
 }
 }
 */

#ifndef ADC_MANAGER_H
#define ADC_MANAGER_H

#include <Arduino.h>

// sample rates
enum
{
  SRATE_9615, SRATE_19231, SRATE_38462
};

class AdcManager
{
  public:
    AdcManager(void);

    // call this in setup
    void init(void);

    // call this to calibrate offsets of channels with autocalibrate
    void calibrate(void);

    // configure sampling for pin:
    // samplecount = 1: 10 bit sampling (unsigned)
    // samplecount > 1: 8 bit sampling (signed - zero = VCC/2)
    void setCapture(const uint8_t pin,
                    const uint8_t samplecount,
                    const bool autoCalibrateOfs);

    // get buffer with samples for pin
    int8_t* getCapture(const uint8_t pin);

    // restart sampling for pin
    void restart(const uint8_t pin);

    // samplecount=1: get one sample for pin
    // samplecount>1: get first sample for pin
    int read(const uint8_t pin);

    // read the median value of samples
    int readMedian(const byte pin);
    bool isCaptureComplete(const uint8_t pin);

    // statistics only
    int getCapturedChannels(void);
    int16_t getAdcMin(const uint8_t pin);
    int16_t getAdcMax(const uint8_t pin);
    int16_t getAdcZeroOffset(const uint8_t pin);

    // return number samples to capture
    int getCaptureSize(const uint8_t pin);

    // calibration data available?
    bool calibrationDataAvail(void);

    // get the manager running, starts sampling next pin
    void run(void);
    uint8_t sampleRate;

    const bool isBusy(void) const
    {
      return busy;
    }
    void setBusy(const bool val)
    {
      busy = val;
    }

    const bool isCaptureComplete(const uint8_t ch) const
    {
      return captureComplete[ch];
    }
    void setCaptureComplete(const uint8_t ch, const bool value)
    {
      captureComplete[ch] = value;
    }

    const uint8_t getCaptureSize(const uint8_t ch) const
    {
      return captureSize[ch];
    }

    const int16_t getZeroOffset(const uint8_t ch) const
    {
      return zeroOffset[ch];
    }

    uint8_t getPosition(void) const
    {
      return position;
    }
    void incPosition(void)
    {
      ++position;
    }

    uint8_t getChannel(void) const
    {
      return channel;
    }

    const int16_t getAdcMax(const uint8_t ch) const
    {
      return ADCMax[ch];
    }
    void setAdcMax(const uint8_t ch, const int16_t value)
    {
      ADCMax[ch] = value;
    }

    const int16_t getAdcMin(const uint8_t ch) const
    {
      return ADCMin[ch];
    }
    void setAdcMin(const uint8_t ch, const int16_t value)
    {
      ADCMin[ch] = value;
    }

    void setCaptureValue(const uint8_t ch, const uint8_t pos, const int16_t value)
    {
      capture[ch][pos] = value;
    }

    void setSampleValue(const uint8_t ch, const uint8_t pos, const int16_t value)
    {
      sample[ch][pos] = value;
    }

  private:
    static const uint8_t CHANNELS = 16;
    static const uint16_t ADDR = 500;
    static const uint8_t MAGIC = 1;

    uint8_t position = 0;
    uint8_t channel = 0;
    bool busy = false;
    bool calibrationAvail = false;
    int8_t* capture[CHANNELS]; // ADC capture buffer (ADC0-ADC7) - 8 bit signed (signed: zero = ADC/2)
    uint8_t captureSize[CHANNELS]; // ADC sample buffer size (ADC0-ADC7)
    int16_t zeroOffset[CHANNELS]; // ADC zero offset (ADC0-ADC7)
    int16_t ADCMin[CHANNELS]; // ADC min sample value (ADC-ADC7)
    int16_t ADCMax[CHANNELS]; // ADC max sample value (ADC-ADC7)
    bool captureComplete[CHANNELS]; // ADC buffer filled?
    bool autoCalibrate[CHANNELS]; // do auto-calibrate? (ADC0-ADC7)
    int16_t* sample[CHANNELS];   // ADC one sample (ADC0-ADC7) - 10 bit unsigned

    int capturedChannels;
    void startADC(int sampleCount);
    void calibrateOfs(uint8_t pin);
    void startCapture(int sampleCount);
    void stopCapture(void);
    bool loadCalib(void);
    void loadSaveCalib(bool readflag);
    void saveCalib(void);
    void printCalib(void);
};

extern AdcManager ADCMan;

#endif
