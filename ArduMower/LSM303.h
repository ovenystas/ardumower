#pragma once

#include <Arduino.h> // for byte data type
#include "Vector.h"

class LSM303
{
  public:
    enum DeviceTypeE
    {
      DEVICE_DLH,
      DEVICE_DLM,
      DEVICE_DLHC,
      DEVICE_D,
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
      DEV_ID_LSM303DLM = 60
    };

    // register addresses
    enum RegAddrE
    {
      TEMP_OUT_L        = 0x05, // D
      TEMP_OUT_H        = 0x06, // D

      STATUS_M          = 0x07, // D

      INT_CTRL_M        = 0x12, // D
      INT_SRC_M         = 0x13, // D
      INT_THS_L_M       = 0x14, // D
      INT_THS_H_M       = 0x15, // D

      OFFSET_X_L_M      = 0x16, // D
      OFFSET_X_H_M      = 0x17, // D
      OFFSET_Y_L_M      = 0x18, // D
      OFFSET_Y_H_M      = 0x19, // D
      OFFSET_Z_L_M      = 0x1A, // D
      OFFSET_Z_H_M      = 0x1B, // D
      REFERENCE_X       = 0x1C, // D
      REFERENCE_Y       = 0x1D, // D
      REFERENCE_Z       = 0x1E, // D

      CTRL0             = 0x1F, // D
      CTRL1             = 0x20, // D
      CTRL_REG1_A       = 0x20, // DLH, DLM, DLHC
      CTRL2             = 0x21, // D
      CTRL_REG2_A       = 0x21, // DLH, DLM, DLHC
      CTRL3             = 0x22, // D
      CTRL_REG3_A       = 0x22, // DLH, DLM, DLHC
      CTRL4             = 0x23, // D
      CTRL_REG4_A       = 0x23, // DLH, DLM, DLHC
      CTRL5             = 0x24, // D
      CTRL_REG5_A       = 0x24, // DLH, DLM, DLHC
      CTRL6             = 0x25, // D
      CTRL_REG6_A       = 0x25, // DLHC
      HP_FILTER_RESET_A = 0x25, // DLH, DLM
      CTRL7             = 0x26, // D
      REFERENCE_A       = 0x26, // DLH, DLM, DLHC
      STATUS_A          = 0x27, // D
      STATUS_REG_A      = 0x27, // DLH, DLM, DLHC

      OUT_X_L_A         = 0x28,
      OUT_X_H_A         = 0x29,
      OUT_Y_L_A         = 0x2A,
      OUT_Y_H_A         = 0x2B,
      OUT_Z_L_A         = 0x2C,
      OUT_Z_H_A         = 0x2D,

      FIFO_CTRL         = 0x2E, // D
      FIFO_CTRL_REG_A   = 0x2E, // DLHC
      FIFO_SRC          = 0x2F, // D
      FIFO_SRC_REG_A    = 0x2F, // DLHC

      IG_CFG1           = 0x30, // D
      INT1_CFG_A        = 0x30, // DLH, DLM, DLHC
      IG_SRC1           = 0x31, // D
      INT1_SRC_A        = 0x31, // DLH, DLM, DLHC
      IG_THS1           = 0x32, // D
      INT1_THS_A        = 0x32, // DLH, DLM, DLHC
      IG_DUR1           = 0x33, // D
      INT1_DURATION_A   = 0x33, // DLH, DLM, DLHC
      IG_CFG2           = 0x34, // D
      INT2_CFG_A        = 0x34, // DLH, DLM, DLHC
      IG_SRC2           = 0x35, // D
      INT2_SRC_A        = 0x35, // DLH, DLM, DLHC
      IG_THS2           = 0x36, // D
      INT2_THS_A        = 0x36, // DLH, DLM, DLHC
      IG_DUR2           = 0x37, // D
      INT2_DURATION_A   = 0x37, // DLH, DLM, DLHC

      CLICK_CFG         = 0x38, // D
      CLICK_CFG_A       = 0x38, // DLHC
      CLICK_SRC         = 0x39, // D
      CLICK_SRC_A       = 0x39, // DLHC
      CLICK_THS         = 0x3A, // D
      CLICK_THS_A       = 0x3A, // DLHC
      TIME_LIMIT        = 0x3B, // D
      TIME_LIMIT_A      = 0x3B, // DLHC
      TIME_LATENCY      = 0x3C, // D
      TIME_LATENCY_A    = 0x3C, // DLHC
      TIME_WINDOW       = 0x3D, // D
      TIME_WINDOW_A     = 0x3D, // DLHC

      Act_THS           = 0x3E, // D
      Act_DUR           = 0x3F, // D

      CRA_REG_M         = 0x00, // DLH, DLM, DLHC
      CRB_REG_M         = 0x01, // DLH, DLM, DLHC
      MR_REG_M          = 0x02, // DLH, DLM, DLHC

      SR_REG_M          = 0x09, // DLH, DLM, DLHC
      IRA_REG_M         = 0x0A, // DLH, DLM, DLHC
      IRB_REG_M         = 0x0B, // DLH, DLM, DLHC
      IRC_REG_M         = 0x0C, // DLH, DLM, DLHC

      WHO_AM_I          = 0x0F, // D
      WHO_AM_I_M        = 0x0F, // DLM

      TEMP_OUT_H_M      = 0x31, // DLHC
      TEMP_OUT_L_M      = 0x32, // DLHC


      // dummy addresses for registers in different locations on different devices;
      // the library translates these based on device type
      // value with sign flipped is used as index into translated_regs array

      OUT_X_H_M         = -1,
      OUT_X_L_M         = -2,
      OUT_Y_H_M         = -3,
      OUT_Y_L_M         = -4,
      OUT_Z_H_M         = -5,
      OUT_Z_L_M         = -6,
      // update dummy_reg_count if registers are added here!

      // device-specific register addresses

      DLH_OUT_X_H_M     = 0x03,
      DLH_OUT_X_L_M     = 0x04,
      DLH_OUT_Y_H_M     = 0x05,
      DLH_OUT_Y_L_M     = 0x06,
      DLH_OUT_Z_H_M     = 0x07,
      DLH_OUT_Z_L_M     = 0x08,

      DLM_OUT_X_H_M     = 0x03,
      DLM_OUT_X_L_M     = 0x04,
      DLM_OUT_Z_H_M     = 0x05,
      DLM_OUT_Z_L_M     = 0x06,
      DLM_OUT_Y_H_M     = 0x07,
      DLM_OUT_Y_L_M     = 0x08,

      DLHC_OUT_X_H_M    = 0x03,
      DLHC_OUT_X_L_M    = 0x04,
      DLHC_OUT_Z_H_M    = 0x05,
      DLHC_OUT_Z_L_M    = 0x06,
      DLHC_OUT_Y_H_M    = 0x07,
      DLHC_OUT_Y_L_M    = 0x08,

      D_OUT_X_L_M       = 0x08,
      D_OUT_X_H_M       = 0x09,
      D_OUT_Y_L_M       = 0x0A,
      D_OUT_Y_H_M       = 0x0B,
      D_OUT_Z_L_M       = 0x0C,
      D_OUT_Z_H_M       = 0x0D
    };

    Vector<int16_t> m_acc {}; // accelerometer readings
    Vector<int16_t> m_mag {}; // magnetometer readings

    // FIXME: These values are never modified!
    Vector<int16_t> m_mag_max { INT16_MAX }; // maximum magnetometer values, used for calibration
    Vector<int16_t> m_mag_min { INT16_MIN }; // minimum magnetometer values, used for calibration

    uint8_t m_last_status {}; // status of last I2C transmission

    LSM303(void);

    bool init(DeviceTypeE device = DEVICE_AUTO, Sa0StateE sa0 = SA0_AUTO);
    DeviceTypeE getDeviceType(void) const
    {
      return m_deviceType;
    }
    bool isInitialized(void)
    {
      return m_deviceType != DEVICE_AUTO;
    }

    void enableDefault(void);

    void writeAccReg(uint8_t reg, uint8_t value);
    byte readAccReg(uint8_t reg);
    void writeMagReg(uint8_t reg, uint8_t value);
    byte readMagReg(int16_t reg);

    void writeReg(uint8_t reg, uint8_t value);
    byte readReg(int16_t reg);

    void readAcc(void);
    void readMag(void);
    void read(void);

    void setTimeout(uint16_t timeout);
    uint16_t getTimeout(void);
    bool timeoutOccurred(void);

    float heading(void);

    template <typename T>
    float heading(Vector<T> from);

  private:
    DeviceTypeE m_deviceType { DEVICE_AUTO }; // chip type (D, DLHC, DLM, or DLH)
    uint8_t m_acc_address {};
    uint8_t m_mag_address {};

    static const int16_t m_dummy_reg_count = 6;
    RegAddrE m_translated_regs[m_dummy_reg_count + 1] {}; // index 0 not used

    uint16_t m_io_timeout {};
    bool m_did_timeout { false };

    int16_t testReg(byte address, RegAddrE reg);
};

/*
Returns the angular difference in the horizontal plane between the
"from" vector and north, in degrees.

Description of heading algorithm:
Shift and scale the magnetic reading based on calibration data to find
the North vector. Use the acceleration readings to determine the Up
vector (gravity is measured as an upward acceleration). The cross
product of North and Up vectors is East. The vectors East and North
form a basis for the horizontal plane. The From vector is projected
into the horizontal plane and the angle between the projected vector
and horizontal north is returned.
*/
template <typename T>
float LSM303::heading(Vector<T> from)
{
  // Subtract offset (average of min and max) from magnetometer readings
  Vector<int32_t> temp_m = static_cast<Vector<int32_t>>(m_mag) -
      (static_cast<Vector<int32_t>>(m_mag_min) +
       static_cast<Vector<int32_t>>(m_mag_max)) / 2;

  // Compute E and N
  Vector<float> E = Vector<float>::cross(temp_m, m_acc);
  Vector<float>::normalize(E);
  Vector<float> N = Vector<float>::cross(m_acc, E);
  Vector<float>::normalize(N);

  // Compute heading
  float headingValue =
      atan2(Vector<float>::dot(E, from), Vector<float>::dot(N, from))
      * 180 / M_PI;
  if (headingValue < 0)
  {
    headingValue += 360;
  }

  return headingValue;
}
