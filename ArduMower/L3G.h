/*
 * Gyroscope
 */
#pragma once

#include <Arduino.h> // for byte data type
#include "Vector.h"

class L3G
{
  public:
    enum DeviceTypeE
    {
      DEVICE_4200D,
      DEVICE_D20,
      DEVICE_D20H,
      DEVICE_AUTO
    };

    enum Sa0StateE
    {
      SA0_LOW,
      SA0_HIGH,
      SA0_AUTO
    };

    enum DeviceIdE
    {
      DEV_ID_4200D = 0xD3
    };

    // register addresses
    enum RegAddrE
    {
       WHO_AM_I       = 0x0F,

       CTRL1          = 0x20, // D20H
       CTRL_REG1      = 0x20, // D20, 4200D
       CTRL2          = 0x21, // D20H
       CTRL_REG2      = 0x21, // D20, 4200D
       CTRL3          = 0x22, // D20H
       CTRL_REG3      = 0x22, // D20, 4200D
       CTRL4          = 0x23, // D20H
       CTRL_REG4      = 0x23, // D20, 4200D
       CTRL5          = 0x24, // D20H
       CTRL_REG5      = 0x24, // D20, 4200D
       REFERENCE      = 0x25,
       OUT_TEMP       = 0x26,
       STATUS         = 0x27, // D20H
       STATUS_REG     = 0x27, // D20, 4200D

       OUT_X_L        = 0x28,
       OUT_X_H        = 0x29,
       OUT_Y_L        = 0x2A,
       OUT_Y_H        = 0x2B,
       OUT_Z_L        = 0x2C,
       OUT_Z_H        = 0x2D,

       FIFO_CTRL      = 0x2E, // D20H
       FIFO_CTRL_REG  = 0x2E, // D20, 4200D
       FIFO_SRC       = 0x2F, // D20H
       FIFO_SRC_REG   = 0x2F, // D20, 4200D

       IG_CFG         = 0x30, // D20H
       INT1_CFG       = 0x30, // D20, 4200D
       IG_SRC         = 0x31, // D20H
       INT1_SRC       = 0x31, // D20, 4200D
       IG_THS_XH      = 0x32, // D20H
       INT1_THS_XH    = 0x32, // D20, 4200D
       IG_THS_XL      = 0x33, // D20H
       INT1_THS_XL    = 0x33, // D20, 4200D
       IG_THS_YH      = 0x34, // D20H
       INT1_THS_YH    = 0x34, // D20, 4200D
       IG_THS_YL      = 0x35, // D20H
       INT1_THS_YL    = 0x35, // D20, 4200D
       IG_THS_ZH      = 0x36, // D20H
       INT1_THS_ZH    = 0x36, // D20, 4200D
       IG_THS_ZL      = 0x37, // D20H
       INT1_THS_ZL    = 0x37, // D20, 4200D
       IG_DURATION    = 0x38, // D20H
       INT1_DURATION  = 0x38, // D20, 4200D

       LOW_ODR        = 0x39  // D20H
    };

    Vector<int16_t> m_g {}; // gyro angular velocity readings

    uint8_t m_last_status {}; // status of last I2C transmission

    L3G() {};

    bool init(DeviceTypeE device = DEVICE_AUTO, Sa0StateE sa0 = SA0_AUTO);
    DeviceTypeE getDeviceType() const
    {
      return m_deviceType;
    }

    void enableDefault();

    void writeReg(const uint8_t reg, const uint8_t value);
    uint8_t readReg(const uint8_t reg);

    void read();

    uint16_t getTimeout() const
    {
      return m_io_timeout;
    }

    void setTimeout(const uint16_t timeout)
    {
      m_io_timeout = timeout;
    }

    bool timeoutOccurred();

  private:
      DeviceTypeE m_deviceType { DEVICE_AUTO }; // chip type (D20H, D20, or 4200D)
      uint8_t m_address {};

      uint16_t m_io_timeout {};
      bool m_did_timeout {};

      int16_t testReg(const uint8_t address, const RegAddrE reg);
};
