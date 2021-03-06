#include "LSM303.h"
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define D_SA0_HIGH_ADDRESS                0x1D //0b0011101
#define D_SA0_LOW_ADDRESS                 0x1E //0b0011110
#define DLHC_DLM_DLH_MAG_ADDRESS          0x1E //0b0011110
#define DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS 0x19 //0b0011001
#define DLM_DLH_ACC_SA0_LOW_ADDRESS       0x18 //0b0011000

#define TEST_REG_ERROR -1

#define D_WHO_ID    0x49
#define DLM_WHO_ID  0x3C

// Constructors ////////////////////////////////////////////////////////////////

LSM303::LSM303(void)
{
  /*
  These values lead to an assumed magnetometer bias of 0.
  Use the Calibrate example program to determine appropriate values
  for your particular unit. The Heading example demonstrates how to
  adjust these values in your own sketch.
  */
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in readAcc(), readMag(), or read() since the last call
// to timeoutOccurred()?
bool LSM303::timeoutOccurred()
{
  bool tmp = m_did_timeout;
  m_did_timeout = false;
  return tmp;
}

void LSM303::setTimeout(uint16_t timeout)
{
  m_io_timeout = timeout;
}

uint16_t LSM303::getTimeout()
{
  return m_io_timeout;
}

bool LSM303::init(DeviceTypeE device, Sa0StateE sa0)
{
  // perform auto-detection unless device type and SA0 state were both specified
  if (device == DEVICE_AUTO || sa0 == SA0_AUTO)
  {
    // check for LSM303D if device is unidentified or was specified to be this type
    if (device == DEVICE_AUTO || device == DEVICE_D)
    {
      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != SA0_LOW &&
          testReg(D_SA0_HIGH_ADDRESS, WHO_AM_I) == D_WHO_ID)
      {
        // device responds to address 0011101 with D ID; it's a D with SA0 high
        device = DEVICE_D;
        sa0 = SA0_HIGH;
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != SA0_HIGH &&
          testReg(D_SA0_LOW_ADDRESS, WHO_AM_I) == D_WHO_ID)
      {
        // device responds to address 0011110 with D ID; it's a D with SA0 low
        device = DEVICE_D;
        sa0 = SA0_LOW;
      }
    }

    // check for LSM303DLHC, DLM, DLH if device is still unidentified or was
    // specified to be one of these types
    if (device == DEVICE_AUTO ||
        device == DEVICE_DLHC ||
        device == DEVICE_DLM ||
        device == DEVICE_DLH)
    {
      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != SA0_LOW &&
          testReg(DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS, CTRL_REG1_A) !=
              TEST_REG_ERROR)
      {
        // device responds to address 0011001; it's a DLHC, DLM with SA0 high,
        // or DLH with SA0 high
        sa0 = SA0_HIGH;
        if (device == DEVICE_AUTO)
        {
          // use magnetometer WHO_AM_I register to determine device type
          //
          // DLHC seems to respond to WHO_AM_I request the same way as DLM, even
          // though this register isn't documented in its data sheet.
          // Since the DLHC accelerometer address is the same as the DLM with
          // SA0 high, but Pololu DLM boards pull SA0 low by default, we'll
          // guess that a device whose accelerometer responds to the SA0 high
          // address and whose magnetometer gives the DLM ID is actually a DLHC.
          device =
              testReg(DLHC_DLM_DLH_MAG_ADDRESS, WHO_AM_I_M) == DLM_WHO_ID ?
              DEVICE_DLHC : DEVICE_DLH;
        }
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != SA0_HIGH &&
          testReg(DLM_DLH_ACC_SA0_LOW_ADDRESS, CTRL_REG1_A) != TEST_REG_ERROR)
      {
        // device responds to address 0011000; it's a DLM with SA0 low or
        // DLH with SA0 low
        sa0 = SA0_LOW;
        if (device == DEVICE_AUTO)
        {
          // use magnetometer WHO_AM_I register to determine device type
          device =
              testReg(DLHC_DLM_DLH_MAG_ADDRESS, WHO_AM_I_M) == DLM_WHO_ID ?
                  DEVICE_DLM : DEVICE_DLH;
        }
      }
    }

    // make sure device and SA0 were successfully detected; otherwise,
    // indicate failure
    if (device == DEVICE_AUTO || sa0 == SA0_AUTO)
    {
      return false;
    }
  }

  m_deviceType = device;

  // set device addresses and translated register addresses
  switch (device)
  {
    case DEVICE_D:
      m_acc_address = m_mag_address = (sa0 == SA0_HIGH) ?
          D_SA0_HIGH_ADDRESS : D_SA0_LOW_ADDRESS;
      m_translated_regs[-OUT_X_L_M] = D_OUT_X_L_M;
      m_translated_regs[-OUT_X_H_M] = D_OUT_X_H_M;
      m_translated_regs[-OUT_Y_L_M] = D_OUT_Y_L_M;
      m_translated_regs[-OUT_Y_H_M] = D_OUT_Y_H_M;
      m_translated_regs[-OUT_Z_L_M] = D_OUT_Z_L_M;
      m_translated_regs[-OUT_Z_H_M] = D_OUT_Z_H_M;
      break;

    case DEVICE_DLHC:
      // DLHC doesn't have configurable SA0 but uses same acc address as
      // DLM/DLH with SA0 high
      m_acc_address = DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS;
      m_mag_address = DLHC_DLM_DLH_MAG_ADDRESS;
      m_translated_regs[-OUT_X_H_M] = DLHC_OUT_X_H_M;
      m_translated_regs[-OUT_X_L_M] = DLHC_OUT_X_L_M;
      m_translated_regs[-OUT_Y_H_M] = DLHC_OUT_Y_H_M;
      m_translated_regs[-OUT_Y_L_M] = DLHC_OUT_Y_L_M;
      m_translated_regs[-OUT_Z_H_M] = DLHC_OUT_Z_H_M;
      m_translated_regs[-OUT_Z_L_M] = DLHC_OUT_Z_L_M;
      break;

    case DEVICE_DLM:
      m_acc_address = (sa0 == SA0_HIGH) ?
          DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS : DLM_DLH_ACC_SA0_LOW_ADDRESS;
      m_mag_address = DLHC_DLM_DLH_MAG_ADDRESS;
      m_translated_regs[-OUT_X_H_M] = DLM_OUT_X_H_M;
      m_translated_regs[-OUT_X_L_M] = DLM_OUT_X_L_M;
      m_translated_regs[-OUT_Y_H_M] = DLM_OUT_Y_H_M;
      m_translated_regs[-OUT_Y_L_M] = DLM_OUT_Y_L_M;
      m_translated_regs[-OUT_Z_H_M] = DLM_OUT_Z_H_M;
      m_translated_regs[-OUT_Z_L_M] = DLM_OUT_Z_L_M;
      break;

    case DEVICE_DLH:
      m_acc_address = (sa0 == SA0_HIGH) ?
          DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS : DLM_DLH_ACC_SA0_LOW_ADDRESS;
      m_mag_address = DLHC_DLM_DLH_MAG_ADDRESS;
      m_translated_regs[-OUT_X_H_M] = DLH_OUT_X_H_M;
      m_translated_regs[-OUT_X_L_M] = DLH_OUT_X_L_M;
      m_translated_regs[-OUT_Y_H_M] = DLH_OUT_Y_H_M;
      m_translated_regs[-OUT_Y_L_M] = DLH_OUT_Y_L_M;
      m_translated_regs[-OUT_Z_H_M] = DLH_OUT_Z_H_M;
      m_translated_regs[-OUT_Z_L_M] = DLH_OUT_Z_L_M;
      break;

    case DEVICE_AUTO:
      return false;
  }

  return true;
}

/*
Enables the LSM303's accelerometer and magnetometer. Also:
- Sets sensor full scales (gain) to default power-on values, which are
  +/- 2 g for accelerometer and +/- 1.3 gauss for magnetometer
  (+/- 4 gauss on LSM303D).
- Selects 50 Hz ODR (output data rate) for accelerometer and 7.5 Hz
  ODR for magnetometer (6.25 Hz on LSM303D). (These are the ODR
  settings for which the electrical characteristics are specified in
  the datasheets.)
- Enables high resolution modes (if available).
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LSM303::enableDefault(void)
{

  if (m_deviceType == DEVICE_D)
  {
    // Accelerometer

    // 0x00 = 0b00000000
    // AFS = 0 (+/- 2 g full scale)
    writeReg(CTRL2, 0x00);

    // 0x57 = 0b01010111
    // AODR = 0101 (50 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
    writeReg(CTRL1, 0x57);

    // Magnetometer

    // 0x64 = 0b01100100
    // M_RES = 11 (high resolution mode); M_ODR = 001 (6.25 Hz ODR)
    writeReg(CTRL5, 0x64);

    // 0x20 = 0b00100000
    // MFS = 01 (+/- 4 gauss full scale)
    writeReg(CTRL6, 0x20);

    // 0x00 = 0b00000000
    // MLP = 0 (low power mode off); MD = 00 (continuous-conversion mode)
    writeReg(CTRL7, 0x00);
  }
  else
  {
    // Accelerometer

    if (m_deviceType == DEVICE_DLHC)
    {
      // 0x08 = 0b00001000
      // FS = 00 (+/- 2 g full scale); HR = 1 (high resolution enable)
      writeAccReg(CTRL_REG4_A, 0x08);

      // 0x47 = 0b01000111
      // ODR = 0100 (50 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
      writeAccReg(CTRL_REG1_A, 0x47);
    }
    else // DLM, DLH
    {
      // 0x00 = 0b00000000
      // FS = 00 (+/- 2 g full scale)
      writeAccReg(CTRL_REG4_A, 0x00);

      // 0x27 = 0b00100111
      // PM = 001 (normal mode); DR = 00 (50 Hz ODR); Zen = Yen = Xen = 1 (all axes enabled)
      writeAccReg(CTRL_REG1_A, 0x27);
    }

    // Magnetometer

    // 0x0C = 0b00001100
    // DO = 011 (7.5 Hz ODR)
    writeMagReg(CRA_REG_M, 0x0C);

    // 0x20 = 0b00100000
    // GN = 001 (+/- 1.3 gauss full scale)
    writeMagReg(CRB_REG_M, 0x20);

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    writeMagReg(MR_REG_M, 0x00);
  }
}

// Writes an accelerometer register
void LSM303::writeAccReg(byte reg, byte value)
{
  Wire.beginTransmission(m_acc_address);
  Wire.write(reg);
  Wire.write(value);
  m_last_status = Wire.endTransmission();
}

// Reads an accelerometer register
byte LSM303::readAccReg(byte reg)
{
  byte value;

  Wire.beginTransmission(m_acc_address);
  Wire.write(reg);
  m_last_status = Wire.endTransmission();
  Wire.requestFrom(m_acc_address, static_cast<uint8_t>(1));
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

// Writes a magnetometer register
void LSM303::writeMagReg(byte reg, byte value)
{
  Wire.beginTransmission(m_mag_address);
  Wire.write(reg);
  Wire.write(value);
  m_last_status = Wire.endTransmission();
}

// Reads a magnetometer register
byte LSM303::readMagReg(int16_t reg)
{
  byte value;

  // If dummy register address (magnetometer Y/Z),
  // look up actual translated address (based on device type).
  if (reg < 0)
  {
    reg = m_translated_regs[-reg];
  }

  Wire.beginTransmission(m_mag_address);
  Wire.write(reg);
  m_last_status = Wire.endTransmission();
  Wire.requestFrom(m_mag_address, static_cast<uint8_t>(1));
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

void LSM303::writeReg(byte reg, byte value)
{
  // mag address == acc_address for LSM303D,
  // so it doesn't really matter which one we use.
  if (m_deviceType == DEVICE_D || reg < CTRL_REG1_A)
  {
    writeMagReg(reg, value);
  }
  else
  {
    writeAccReg(reg, value);
  }
}

// Note that this function will not work for reading TEMP_OUT_H_M and
// TEMP_OUT_L_M on the DLHC.
// To read those two registers, use readMagReg() instead.
byte LSM303::readReg(int16_t reg)
{
  // mag address == acc_address for LSM303D, so it doesn't really matter which
  // one we use.
  // Use readMagReg so it can translate OUT_[XYZ]_[HL]_M
  if (m_deviceType == DEVICE_D || reg < CTRL_REG1_A)
  {
    return readMagReg(reg);
  }
  else
  {
    return readAccReg(reg);
  }
}

// Reads the 3 accelerometer channels and stores them in vector m_acc.
void LSM303::readAcc(void)
{
  Wire.beginTransmission(m_acc_address);
  // assert the MSB of the address to get the accelerometer
  // to do slave-transmit sub address updating.
  Wire.write(OUT_X_L_A | B10000000);
  m_last_status = Wire.endTransmission();
  Wire.requestFrom(m_acc_address, static_cast<uint8_t>(6));

  uint32_t millis_start = millis();
  while (Wire.available() < 6)
  {
    if (m_io_timeout > 0 &&
        static_cast<uint32_t>(millis() - millis_start) > m_io_timeout)
    {
      m_did_timeout = true;
      return;
    }
  }

  // Read from device, combine high and low bytes.
  // This no longer drops the lowest 4 bits of the readings from the
  // DLH/DLM/DLHC, which are always 0.
  // (12-bit resolution, left-aligned). The D has 16-bit resolution.
  // Ove: Changed to drop lowest 4 bits again
  Vector<int16_t> acc;
  acc.x = (Wire.read() >> 4) | (Wire.read() << 4);
  acc.y = (Wire.read() >> 4) | (Wire.read() << 4);
  acc.z = (Wire.read() >> 4) | (Wire.read() << 4);

  m_acc = acc;
}

// Reads the 3 magnetometer channels and stores them in vector m_mag.
void LSM303::readMag(void)
{
  Wire.beginTransmission(m_mag_address);
  // If LSM303D, assert MSB to enable sub address updating
  // OUT_X_L_M comes first on D, OUT_X_H_M on others
  Wire.write(m_deviceType == DEVICE_D ?
      m_translated_regs[-OUT_X_L_M] | B10000000 :
      m_translated_regs[-OUT_X_H_M]);

  m_last_status = Wire.endTransmission();
  Wire.requestFrom(m_mag_address, static_cast<uint8_t>(6));

  unsigned long millis_start = millis();
  while (Wire.available() < 6)
  {
    if (m_io_timeout > 0 &&
        static_cast<uint32_t>(millis() - millis_start) > m_io_timeout)
    {
      m_did_timeout = true;
      return;
    }
  }

  Vector<int16_t> mag;

  if (m_deviceType == DEVICE_D)
  {
    // D: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    mag.x = Wire.read() | (Wire.read() << 8);
    mag.y = Wire.read() | (Wire.read() << 8);
    mag.z = Wire.read() | (Wire.read() << 8);
  }
  else
  {
    // DLHC, DLM, DLH: X_H, X_L...
    mag.x = (Wire.read() << 8) | Wire.read();

    if (m_deviceType == DEVICE_DLH)
    {
      // DLH: ...Y_H, Y_L, Z_H, Z_L
      mag.y = (Wire.read() << 8) | Wire.read();
      mag.z = (Wire.read() << 8) | Wire.read();
    }
    else
    {
      // DLM, DLHC: ...Z_H, Z_L, Y_H, Y_L
      mag.z = (Wire.read() << 8) | Wire.read();
      mag.y = (Wire.read() << 8) | Wire.read();
    }
  }

  m_mag = mag;
}

// Reads all 6 channels of the LSM303 and stores them in the object variables
void LSM303::read(void)
{
  readAcc();
  readMag();
}

/*
Returns the angular difference in the horizontal plane between a
default vector and north, in degrees.

The default vector here is chosen to point along the surface of the
PCB, in the direction of the top of the text on the silk screen.
This is the +X axis on the Pololu LSM303D carrier and the -Y axis on
the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH carriers.
*/
float LSM303::heading(void)
{
  if (m_deviceType == DEVICE_D)
  {
    return heading(Vector<int16_t>(1, 0, 0));
  }
  else
  {
    return heading(Vector<int16_t>(0, -1, 0));
  }
}

// Private Methods /////////////////////////////////////////////////////////////

int16_t LSM303::testReg(byte address, RegAddrE reg)
{
  Wire.beginTransmission(address);
  Wire.write(static_cast<uint8_t>(reg));
  if (Wire.endTransmission() != 0)
  {
    return TEST_REG_ERROR;
  }

  Wire.requestFrom(address, static_cast<uint8_t>(1));
  if (Wire.available())
  {
    return Wire.read();
  }
  else
  {
    return TEST_REG_ERROR;
  }
}
