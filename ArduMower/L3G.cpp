#include "L3G.h"
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define D20_SA0_HIGH_ADDRESS      0x6B //0b1101011 // also applies to D20H
#define D20_SA0_LOW_ADDRESS       0x6A //0b1101010 // also applies to D20H
#define L3G4200D_SA0_HIGH_ADDRESS 0x69 //0b1101001
#define L3G4200D_SA0_LOW_ADDRESS  0x68 //0b1101000

#define TEST_REG_ERROR -1

#define D20H_WHO_ID     0xD7
#define D20_WHO_ID      0xD4
#define L3G4200D_WHO_ID 0xD3

// Constructors ////////////////////////////////////////////////////////////////


// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in read() since the last call to timeoutOccurred()?
bool L3G::timeoutOccurred()
{
  bool tmp = m_did_timeout;
  m_did_timeout = false;
  return tmp;
}

bool L3G::init(deviceTypeE device, sa0StateE sa0)
{
  // perform auto-detection unless device type and SA0 state were both specified
  if (device == DEVICE_AUTO || sa0 == SA0_AUTO)
  {
    // check for L3GD20H, D20 if device is unidentified or was specified to be one of these types
    if (device == DEVICE_AUTO || device == DEVICE_D20H || device == DEVICE_D20)
    {
      int id;

      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != SA0_LOW && (id = testReg(D20_SA0_HIGH_ADDRESS, WHO_AM_I)) != TEST_REG_ERROR)
      {
        // device responds to address 1101011; it's a D20H or D20 with SA0 high
        sa0 = SA0_HIGH;
        if (device == DEVICE_AUTO)
        {
          // use ID from WHO_AM_I register to determine device type
          device = (id == D20H_WHO_ID) ? DEVICE_D20H : DEVICE_D20;
        }
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != SA0_HIGH && (id = testReg(D20_SA0_LOW_ADDRESS, WHO_AM_I)) != TEST_REG_ERROR)
      {
        // device responds to address 1101010; it's a D20H or D20 with SA0 low
        sa0 = SA0_LOW;
        if (device == DEVICE_AUTO)
        {
          // use ID from WHO_AM_I register to determine device type
          device = (id == D20H_WHO_ID) ? DEVICE_D20H : DEVICE_D20;
        }
      }
    }

    // check for L3G4200D if device is still unidentified or was specified to be this type
    if (device == DEVICE_AUTO || device == DEVICE_4200D)
    {
      if (sa0 != SA0_LOW && testReg(L3G4200D_SA0_HIGH_ADDRESS, WHO_AM_I) == L3G4200D_WHO_ID)
      {
        // device responds to address 1101001; it's a 4200D with SA0 high
        device = DEVICE_4200D;
        sa0 = SA0_HIGH;
      }
      else if (sa0 != SA0_HIGH && testReg(L3G4200D_SA0_LOW_ADDRESS, WHO_AM_I) == L3G4200D_WHO_ID)
      {
        // device responds to address 1101000; it's a 4200D with SA0 low
        device = DEVICE_4200D;
        sa0 = SA0_LOW;
      }
    }

    // make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == DEVICE_AUTO || sa0 == SA0_AUTO)
    {
      return false;
    }
  }

  m_deviceType = device;

  // set device address
  switch (device)
  {
    case DEVICE_D20H:
    case DEVICE_D20:
      m_address = (sa0 == SA0_HIGH) ? D20_SA0_HIGH_ADDRESS : D20_SA0_LOW_ADDRESS;
      break;

    case DEVICE_4200D:
      m_address = (sa0 == SA0_HIGH) ? L3G4200D_SA0_HIGH_ADDRESS : L3G4200D_SA0_LOW_ADDRESS;
      break;

    case DEVICE_AUTO:
      return false;
  }

  return true;
}

/*
Enables the L3G's gyro. Also:
- Sets gyro full scale (gain) to default power-on value of +/- 250 dps
  (specified as +/- 245 dps for L3GD20H).
- Selects 200 Hz ODR (output data rate). (Exact rate is specified as 189.4 Hz
  for L3GD20H and 190 Hz for L3GD20.)
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void L3G::enableDefault(void)
{
  if (m_deviceType == DEVICE_D20H)
  {
    // 0x00 = 0b00000000
    // Low_ODR = 0 (low speed ODR disabled)
    writeReg(LOW_ODR, 0x00);
  }

  // 0x00 = 0b00000000
  // FS = 00 (+/- 250 dps full scale)
  writeReg(CTRL_REG4, 0x00);

  // 0x6F = 0b01101111
  // DR = 01 (200 Hz ODR);
  // BW = 10 (50 Hz bandwidth);
  // PD = 1 (normal mode);
  // Zen = Yen = Xen = 1 (all axes enabled)
  writeReg(CTRL_REG1, 0x6F);
}

// Writes a gyro register
void L3G::writeReg(const byte reg, const byte value)
{
  Wire.beginTransmission(m_address);
  Wire.write(reg);
  Wire.write(value);
  m_last_status = Wire.endTransmission();
}

// Reads a gyro register
byte L3G::readReg(const byte reg)
{
  byte value;

  Wire.beginTransmission(m_address);
  Wire.write(reg);
  m_last_status = Wire.endTransmission(false);
  Wire.requestFrom(m_address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

// Reads the 3 gyro channels and stores them in vector g
void L3G::read()
{
  Wire.beginTransmission(m_address);
  // assert the MSB of the address to get the gyro
  // to do slave-transmit subaddress updating.
  Wire.write(OUT_X_L | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(m_address, (byte)6);

  unsigned int millis_start = millis();
  while (Wire.available() < 6)
  {
    if (m_io_timeout > 0 && ((unsigned int)millis() - millis_start) > m_io_timeout)
    {
      m_did_timeout = true;
      return;
    }
  }

  uint8_t xlg = Wire.read();
  uint8_t xhg = Wire.read();
  uint8_t ylg = Wire.read();
  uint8_t yhg = Wire.read();
  uint8_t zlg = Wire.read();
  uint8_t zhg = Wire.read();

  // combine high and low bytes
  m_g.x = static_cast<int16_t>(xhg << 8 | xlg);
  m_g.y = static_cast<int16_t>(yhg << 8 | ylg);
  m_g.z = static_cast<int16_t>(zhg << 8 | zlg);
}

//void L3G::vectorNormalize(vector<float> *a)
//{
//  float mag = sqrt(vectorDot(a, a));
//  a->x /= mag;
//  a->y /= mag;
//  a->z /= mag;
//}

// Private Methods //////////////////////////////////////////////////////////////

int L3G::testReg(const byte address, const regAddrE reg)
{
  Wire.beginTransmission(address);
  Wire.write((byte)reg);
  if (Wire.endTransmission() != 0)
  {
    return TEST_REG_ERROR;
  }

  Wire.requestFrom(address, (byte)1);
  if (Wire.available())
  {
    return Wire.read();
  }
  else
  {
    return TEST_REG_ERROR;
  }
}
