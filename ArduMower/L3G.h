/*
 * Gyroscope
 */
#pragma once

#include <Arduino.h> // for byte data type

class L3G
{
  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };

    enum deviceTypeE
    {
      DEVICE_4200D,
      DEVICE_D20,
      DEVICE_D20H,
      DEVICE_AUTO
    };

    enum sa0StateE
    {
      SA0_LOW,
      SA0_HIGH,
      SA0_AUTO
    };

    enum deviceIdE
    {
      DEV_ID_4200D = 0xD3
    };

    // register addresses
    enum regAddrE
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

    vector<int16_t> m_g {}; // gyro angular velocity readings

    byte m_last_status {}; // status of last I2C transmission

    L3G() {};

    bool init(deviceTypeE device = DEVICE_AUTO, sa0StateE sa0 = SA0_AUTO);
    deviceTypeE getDeviceType(void) const
    {
      return m_deviceType;
    }

    void enableDefault(void);

    void writeReg(const byte reg, const byte value);
    byte readReg(const byte reg);

    void read(void);

    unsigned int getTimeout(void) const
    {
      return m_io_timeout;
    }

    void setTimeout(const unsigned int timeout)
    {
      m_io_timeout = timeout;
    }

    bool timeoutOccurred(void);

    // vector functions
    template<typename Ta, typename Tb, typename To> static void vectorCross(
        const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);

    template<typename Ta, typename Tb> static float vectorDot(
        const vector<Ta> *a, const vector<Tb> *b);

    static void vectorNormalize(vector<float> *a);

  private:
      deviceTypeE m_deviceType { DEVICE_AUTO }; // chip type (D20H, D20, or 4200D)
      byte m_address {};

      unsigned int m_io_timeout {};
      bool m_did_timeout { false };

      int testReg(const byte address, const regAddrE reg);
};

template<typename Ta, typename Tb, typename To> void L3G::vectorCross(
    const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float L3G::vectorDot(
    const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}
