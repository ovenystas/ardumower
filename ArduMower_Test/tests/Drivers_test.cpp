/*
 * Drivers_test.cpp
 *
 *  Created on: May 12, 2020
 *      Author: ove
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "Drivers.h"
#include "EEPROM.h"
#include "HardwareSerial.h"

TEST_GROUP(Drivers)
{
  void setup()
  {
  }

  void teardown()
  {
    mock().clear();
  }

  void mockSetup_I2CreadFrom(uint8_t device, uint8_t address, uint8_t num,
      uint8_t* buf_p, int retryCount)
  {
    for (int j = 0; j < retryCount + 1; j++)
    {
      mock().expectOneCall("TwoWire::beginTransmission")
          .withParameter("address", device);
      mock().expectOneCall("TwoWire::write")
          .withParameter("data", address);
      mock().expectOneCall("TwoWire::endTransmission")
          .andReturnValue(0);
      mock().expectOneCall("TwoWire::requestFrom")
          .withParameter("address", device)
          .withParameter("quantity", num)
          .andReturnValue(0);

      for (int i = 0; i < num; ++i)
      {
        mock().expectOneCall("TwoWire::available")
            .andReturnValue(num - i);
        mock().expectOneCall("TwoWire::read")
            .andReturnValue(buf_p[i]);
      }

      mock().expectOneCall("TwoWire::available")
          .andReturnValue(0);

      if (j != retryCount)
      {
        mock().expectOneCall("delay")
            .withParameter("ms", 3);
      }
    }
  }

  void mockSetup_I2CreadFrom_commError(uint8_t device, uint8_t address,
      uint8_t num, int retryCount)
  {
    for (int j = 0; j < retryCount + 1; j++)
    {
      mock().expectOneCall("TwoWire::beginTransmission")
          .withParameter("address", device);
      mock().expectOneCall("TwoWire::write")
          .withParameter("data", address);
      mock().expectOneCall("TwoWire::endTransmission")
          .andReturnValue(0);
      mock().expectOneCall("TwoWire::requestFrom")
          .withParameter("address", device)
          .withParameter("quantity", num)
          .andReturnValue(0);

      mock().expectOneCall("TwoWire::available")
          .andReturnValue(0);

      if (j != retryCount)
      {
        mock().expectOneCall("delay")
            .withParameter("ms", 3);
      }
    }
  }

  void mockSetup_I2CwriteTo(uint8_t device, uint8_t address, int num,
      const uint8_t* buf_p)
  {
    mock().expectOneCall("TwoWire::beginTransmission")
        .withParameter("address", device);
    mock().expectOneCall("TwoWire::write")
        .withParameter("data", address);

    for (int i = 0; i < num; i++)
    {
      mock().expectOneCall("TwoWire::write")
          .withParameter("data", buf_p[i]);

    }
    mock().expectOneCall("TwoWire::endTransmission")
        .andReturnValue(0);
  }
};

TEST(Drivers, sign)
{
  LONGS_EQUAL(-1, sign(INT64_MIN));
  LONGS_EQUAL(-1, sign(-1));
  LONGS_EQUAL( 0, sign( 0));
  LONGS_EQUAL(+1, sign(+1));
  LONGS_EQUAL(+1, sign(INT64_MAX));

  LONGS_EQUAL(-1, sign(-1.0e+38f));
  LONGS_EQUAL(-1, sign(-1.0e-38f));
  LONGS_EQUAL( 0, sign( 0.0f));
  LONGS_EQUAL(+1, sign(+1.0e-38f));
  LONGS_EQUAL(+1, sign(+1.0e+38f));
}

TEST(Drivers, eewrite_uint8_t)
{
  const uint16_t startIdx = 200;
  const uint8_t val = 12;

  uint16_t idx = startIdx;

  mock().expectOneCall("EEPROM::write")
      .withParameter("idx", startIdx)
      .withParameter("val", val);

  eewrite(idx, val);

  UNSIGNED_LONGS_EQUAL(startIdx + sizeof(val), idx);

  mock().checkExpectations();
}

TEST(Drivers, eewrite_uint32_t)
{
  const uint16_t startIdx = 200;
  const uint32_t val = 0xdeadbeef;

  uint16_t idx = startIdx;

  mock().expectOneCall("EEPROM::write")
      .withParameter("idx", startIdx)
      .withParameter("val", 0xef);
  mock().expectOneCall("EEPROM::write")
      .withParameter("idx", startIdx + 1)
      .withParameter("val", 0xbe);
  mock().expectOneCall("EEPROM::write")
      .withParameter("idx", startIdx + 2)
      .withParameter("val", 0xad);
  mock().expectOneCall("EEPROM::write")
      .withParameter("idx", startIdx + 3)
      .withParameter("val", 0xde);

  eewrite(idx, val);

  UNSIGNED_LONGS_EQUAL(startIdx + sizeof(val), idx);

  mock().checkExpectations();
}

TEST(Drivers, eeread_uint8_t)
{
  const uint16_t startIdx = 200;
  uint8_t val = 0;

  uint16_t idx = startIdx;

  mock().expectOneCall("EEPROM::read")
      .withParameter("idx", startIdx)
      .andReturnValue(12);

  eeread(idx, val);

  UNSIGNED_LONGS_EQUAL(startIdx + sizeof(val), idx);
  UNSIGNED_LONGS_EQUAL(12, val);

  mock().checkExpectations();
}

TEST(Drivers, eeread_uint32_t)
{
  const uint16_t startIdx = 200;
  uint32_t val = 0;

  uint16_t idx = startIdx;

  mock().expectOneCall("EEPROM::read")
      .withParameter("idx", startIdx)
      .andReturnValue(0xef);
  mock().expectOneCall("EEPROM::read")
      .withParameter("idx", startIdx + 1)
      .andReturnValue(0xbe);
  mock().expectOneCall("EEPROM::read")
      .withParameter("idx", startIdx + 2)
      .andReturnValue(0xad);
  mock().expectOneCall("EEPROM::read")
      .withParameter("idx", startIdx + 3)
      .andReturnValue(0xde);

  eeread(idx, val);

  UNSIGNED_LONGS_EQUAL(startIdx + sizeof(val), idx);
  UNSIGNED_LONGS_EQUAL(0xdeadbeef, val);

  mock().checkExpectations();
}

TEST(Drivers, eereadwrite_readUint8_t)
{
  const uint16_t startIdx = 200;
  uint8_t val = 0;

  uint16_t idx = startIdx;

  mock().expectOneCall("EEPROM::read")
      .withParameter("idx", startIdx)
      .andReturnValue(12);

  eereadwrite(true, idx, val);

  UNSIGNED_LONGS_EQUAL(startIdx + sizeof(val), idx);
  UNSIGNED_LONGS_EQUAL(12, val);

  mock().checkExpectations();
}

TEST(Drivers, eereadwrite_writeUint8_t)
{
  const uint16_t startIdx = 200;
  uint8_t val = 12;

  uint16_t idx = startIdx;

  mock().expectOneCall("EEPROM::write")
      .withParameter("idx", startIdx)
      .withParameter("val", val);

  eereadwrite(false, idx, val);

  UNSIGNED_LONGS_EQUAL(startIdx + sizeof(val), idx);

  mock().checkExpectations();
}

TEST(Drivers, scalePI)
{
  DOUBLES_EQUAL( -PI*0.5f , scalePI( -PI*2.5f ), 0.001);
  DOUBLES_EQUAL( +PI*0.5f , scalePI( -PI*1.5f ), 0.001);
  DOUBLES_EQUAL( -PI*0.75f, scalePI( -PI*0.75f), 0.001);
  DOUBLES_EQUAL( -PI*0.5f , scalePI( -PI*0.5f ), 0.001);
  DOUBLES_EQUAL( -PI*0.25f, scalePI( -PI*0.25f), 0.001);
  DOUBLES_EQUAL(  0.0f    , scalePI(  0.0f    ), 0.001);
  DOUBLES_EQUAL( +PI*0.25f, scalePI( +PI*0.25f), 0.001);
  DOUBLES_EQUAL( +PI*0.5f , scalePI( +PI*0.5f ), 0.001);
  DOUBLES_EQUAL( +PI*0.75f, scalePI( +PI*0.75f), 0.001);
  DOUBLES_EQUAL( -PI*0.5f , scalePI( +PI*1.5f ), 0.001);
  DOUBLES_EQUAL( +PI*0.5f , scalePI( +PI*2.5f ), 0.001);
}

TEST(Drivers, distancePI)
{
  DOUBLES_EQUAL(    0.0f , distancePI(+PI*4.25f, -PI*3.75f), 0.001);
  DOUBLES_EQUAL(-PI*0.5f , distancePI(+PI*4.25f, +PI*3.75f), 0.001);
  DOUBLES_EQUAL(-PI*0.5f , distancePI(+PI*0.75f, +PI*0.25f), 0.001);
  DOUBLES_EQUAL(-PI*0.5f , distancePI(-PI*0.75f, +PI*0.75f), 0.001);
  DOUBLES_EQUAL(+PI*0.5f , distancePI(-PI*0.25f, +PI*0.25f), 0.001);
  DOUBLES_EQUAL(+PI*0.25f, distancePI(     0.0f, +PI*0.25f), 0.001);
  DOUBLES_EQUAL(-PI*0.25f, distancePI(+PI*0.25f,      0.0f), 0.001);
  DOUBLES_EQUAL(-PI*0.5f , distancePI(+PI*0.25f, -PI*0.25f), 0.001);
  DOUBLES_EQUAL(+PI*0.5f , distancePI(+PI*0.75f, -PI*0.75f), 0.001);
  DOUBLES_EQUAL(+PI*0.5f , distancePI(-PI*0.75f, -PI*0.25f), 0.001);
  DOUBLES_EQUAL(+PI*0.5f , distancePI(-PI*4.25f, -PI*3.75f), 0.001);
  DOUBLES_EQUAL(    0.0f , distancePI(-PI*4.25f, +PI*3.75f), 0.001);
}

IGNORE_TEST(Drivers, freeRam)
{
}

TEST(Drivers, StreamPrint_progmem)
{
  mock().expectOneCall("Print::print")
      .onObject(&Serial1)
      .withParameter("str", "123, -24, 3.14");

  StreamPrint_progmem(Serial1, F("%u, %d, %.2f"), 123u, -24, PI);

  mock().checkExpectations();
}

TEST(Drivers, Serialprint)
{
  mock().expectOneCall("Print::print")
      .onObject(&Serial)
      .withParameter("str", "123, -24, 3.14");

  Serialprint("%u, %d, %.2f", 123u, -24, PI);

  mock().checkExpectations();
}

TEST(Drivers, Streamprint)
{
  mock().expectOneCall("Print::print")
      .onObject(&Serial2)
      .withParameter("str", "123, -24, 3.14");

  Streamprint(Serial2, "%u, %d, %.2f", 123u, -24, PI);

  mock().checkExpectations();
}

TEST(Drivers, minutes2time)
{
  timehm_t time;

  minutes2time(0, time);
  UNSIGNED_LONGS_EQUAL(0, time.hour);
  UNSIGNED_LONGS_EQUAL(0, time.minute);

  minutes2time(1, time);
  UNSIGNED_LONGS_EQUAL(0, time.hour);
  UNSIGNED_LONGS_EQUAL(1, time.minute);

  minutes2time(59, time);
  UNSIGNED_LONGS_EQUAL(0, time.hour);
  UNSIGNED_LONGS_EQUAL(59, time.minute);

  minutes2time(60, time);
  UNSIGNED_LONGS_EQUAL(1, time.hour);
  UNSIGNED_LONGS_EQUAL(0, time.minute);

  minutes2time(61, time);
  UNSIGNED_LONGS_EQUAL(1, time.hour);
  UNSIGNED_LONGS_EQUAL(1, time.minute);

  minutes2time(1439, time);
  UNSIGNED_LONGS_EQUAL(23, time.hour);
  UNSIGNED_LONGS_EQUAL(59, time.minute);
}

TEST(Drivers, time2minutes)
{
  timehm_t time;

  time.hour = 0, time.minute = 0;
  LONGS_EQUAL(0, time2minutes(time));

  time.hour = 0, time.minute = 1;
  LONGS_EQUAL(1, time2minutes(time));

  time.hour = 0, time.minute = 59;
  LONGS_EQUAL(59, time2minutes(time));

  time.hour = 1, time.minute = 0;
  LONGS_EQUAL(60, time2minutes(time));

  time.hour = 1, time.minute = 1;
  LONGS_EQUAL(61, time2minutes(time));

  time.hour = 23, time.minute = 59;
  LONGS_EQUAL(1439, time2minutes(time));
}

TEST(Drivers, time2str)
{
  timehm_t time;

  time.hour = 0, time.minute = 0;
  STRCMP_EQUAL("00:00", time2str(time).c_str());

  time.hour = 0, time.minute = 1;
  STRCMP_EQUAL("00:01", time2str(time).c_str());

  time.hour = 0, time.minute = 59;
  STRCMP_EQUAL("00:59", time2str(time).c_str());

  time.hour = 1, time.minute = 0;
  STRCMP_EQUAL("01:00", time2str(time).c_str());

  time.hour = 1, time.minute = 1;
  STRCMP_EQUAL("01:01", time2str(time).c_str());

  time.hour = 23, time.minute = 59;
  STRCMP_EQUAL("23:59", time2str(time).c_str());
}

TEST(Drivers, date2str)
{
  date_t date;

  date.year = 1000, date.month = 1, date.day = 1, date.dayOfWeek = 0;
  STRCMP_EQUAL("Mon 1000-01-01", date2str(date).c_str());

  date.year = 1999, date.month = 10, date.day = 10, date.dayOfWeek = 1;
  STRCMP_EQUAL("Tue 1999-10-10", date2str(date).c_str());

  date.year = 2000, date.month = 2, date.day = 28, date.dayOfWeek = 2;
  STRCMP_EQUAL("Wed 2000-02-28", date2str(date).c_str());

  date.year = 2020, date.month = 2, date.day = 29, date.dayOfWeek = 3;
  STRCMP_EQUAL("Thu 2020-02-29", date2str(date).c_str());

  date.year = 2021, date.month = 3, date.day = 31, date.dayOfWeek = 4;
  STRCMP_EQUAL("Fri 2021-03-31", date2str(date).c_str());

  date.year = 2022, date.month = 7, date.day = 8, date.dayOfWeek = 5;
  STRCMP_EQUAL("Sat 2022-07-08", date2str(date).c_str());

  date.year = 9999, date.month = 12, date.day = 31, date.dayOfWeek = 6;
  STRCMP_EQUAL("Sun 9999-12-31", date2str(date).c_str());
}

TEST(Drivers, getDayOfWeek)
{
  UNSIGNED_LONGS_EQUAL(Weekdays::Wednesday, getDayOfWeek(1800, 1, 1));
  UNSIGNED_LONGS_EQUAL(Weekdays::Monday,    getDayOfWeek(1974, 10, 7));
  UNSIGNED_LONGS_EQUAL(Weekdays::Wednesday, getDayOfWeek(2020, 5, 13));
  UNSIGNED_LONGS_EQUAL(Weekdays::Monday,    getDayOfWeek(1974 + 50, 10, 7));
  UNSIGNED_LONGS_EQUAL(Weekdays::Sunday,    getDayOfWeek(1974 + 100, 10, 7));

  mock().checkExpectations();
}

TEST(Drivers, I2CwriteTo_0_0_0)
{
  mock().expectOneCall("TwoWire::beginTransmission")
      .withParameter("address", 0);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 0);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 0);
  mock().expectOneCall("TwoWire::endTransmission")
      .andReturnValue(0);

  I2CwriteTo(0, 0, 0);

  mock().checkExpectations();
}

TEST(Drivers, I2CwriteTo_255_255_255)
{
  mock().expectOneCall("TwoWire::beginTransmission")
      .withParameter("address", 255);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 255);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 255);
  mock().expectOneCall("TwoWire::endTransmission")
      .andReturnValue(0);

  I2CwriteTo(255, 255, 255);

  mock().checkExpectations();
}

TEST(Drivers, I2CwriteTo_1_2_0_buf)
{
  const uint8_t buf[2] =
  {
      static_cast<uint8_t>(123u),
      static_cast<uint8_t>(321u)
  };

  mock().expectOneCall("TwoWire::beginTransmission")
      .withParameter("address", 1);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 2);
  mock().expectOneCall("TwoWire::endTransmission")
      .andReturnValue(0);

  I2CwriteTo(1, 2, 0, buf);

  mock().checkExpectations();
}

TEST(Drivers, I2CwriteTo_1_2_1_buf)
{
  const uint8_t buf[2] =
  {
      static_cast<uint8_t>(123u),
      static_cast<uint8_t>(222u)
  };

  mock().expectOneCall("TwoWire::beginTransmission")
      .withParameter("address", 1);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 2);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 123);
  mock().expectOneCall("TwoWire::endTransmission")
      .andReturnValue(0);

  I2CwriteTo(1, 2, 1, buf);

  mock().checkExpectations();
}

TEST(Drivers, I2CwriteTo_1_2_2_buf)
{
  const uint8_t buf[2] =
  {
      static_cast<uint8_t>(123u),
      static_cast<uint8_t>(222u)
  };

  mock().expectOneCall("TwoWire::beginTransmission")
      .withParameter("address", 1);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 2);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 123);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 222u);
  mock().expectOneCall("TwoWire::endTransmission")
      .andReturnValue(0);

  I2CwriteTo(1, 2, 2, buf);

  mock().checkExpectations();
}

TEST(Drivers, I2CreadFrom_neverTry)
{
  uint8_t buf[2] = { 0 };

  LONGS_EQUAL(0, I2CreadFrom(1, 2, 2, buf, -1));
  UNSIGNED_LONGS_EQUAL(0, buf[0]);
  UNSIGNED_LONGS_EQUAL(0, buf[1]);

  mock().checkExpectations();
}

TEST(Drivers, I2CreadFrom_read0bytes_retry0)
{
  mock().expectOneCall("TwoWire::beginTransmission")
      .withParameter("address", 1);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 2);
  mock().expectOneCall("TwoWire::endTransmission")
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::requestFrom")
      .withParameter("address", 1)
      .withParameter("quantity", 0)
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(0);

  uint8_t buf[2] = { 0 };

  LONGS_EQUAL(0, I2CreadFrom(1, 2, 0, buf, 0));
  UNSIGNED_LONGS_EQUAL(0, buf[0]);
  UNSIGNED_LONGS_EQUAL(0, buf[1]);

  mock().checkExpectations();
}

TEST(Drivers, I2CreadFrom_read1byte_retry0)
{
  mock().expectOneCall("TwoWire::beginTransmission")
      .withParameter("address", 1);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 2);
  mock().expectOneCall("TwoWire::endTransmission")
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::requestFrom")
      .withParameter("address", 1)
      .withParameter("quantity", 1)
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(1);
  mock().expectOneCall("TwoWire::read")
      .andReturnValue(123);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(0);

  uint8_t buf[2] = { 0 };

  LONGS_EQUAL(1, I2CreadFrom(1, 2, 1, buf, 0));
  UNSIGNED_LONGS_EQUAL(123, buf[0]);
  UNSIGNED_LONGS_EQUAL(0, buf[1]);

  mock().checkExpectations();
}

TEST(Drivers, I2CreadFrom_read2bytes_retry0_available2)
{
  mock().expectOneCall("TwoWire::beginTransmission")
      .withParameter("address", 1);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 2);
  mock().expectOneCall("TwoWire::endTransmission")
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::requestFrom")
      .withParameter("address", 1)
      .withParameter("quantity", 2)
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(2);
  mock().expectOneCall("TwoWire::read")
      .andReturnValue(123);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(1);
  mock().expectOneCall("TwoWire::read")
      .andReturnValue(222);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(0);

  uint8_t buf[2] = { 0 };

  LONGS_EQUAL(2, I2CreadFrom(1, 2, 2, buf, 0));
  UNSIGNED_LONGS_EQUAL(123, buf[0]);
  UNSIGNED_LONGS_EQUAL(222, buf[1]);

  mock().checkExpectations();
}

TEST(Drivers, I2CreadFrom_read2bytes_retry0_available1)
{
  mock().expectOneCall("TwoWire::beginTransmission")
      .withParameter("address", 1);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 2);
  mock().expectOneCall("TwoWire::endTransmission")
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::requestFrom")
      .withParameter("address", 1)
      .withParameter("quantity", 2)
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(2);
  mock().expectOneCall("TwoWire::read")
      .andReturnValue(123);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(1);
  mock().expectOneCall("TwoWire::read")
      .andReturnValue(222);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(0);

  uint8_t buf[2] = { 0 };

  LONGS_EQUAL(2, I2CreadFrom(1, 2, 2, buf, 0));
  UNSIGNED_LONGS_EQUAL(123, buf[0]);
  UNSIGNED_LONGS_EQUAL(222, buf[1]);

  mock().checkExpectations();
}

TEST(Drivers, I2CreadFrom_read2bytes_retry1_available1)
{
  mock().expectOneCall("TwoWire::beginTransmission")
      .withParameter("address", 1);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 2);
  mock().expectOneCall("TwoWire::endTransmission")
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::requestFrom")
      .withParameter("address", 1)
      .withParameter("quantity", 2)
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(1);
  mock().expectOneCall("TwoWire::read")
      .andReturnValue(123);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(0);
  mock().expectOneCall("delay")
      .withParameter("ms", 3);

  // Retry
  mock().expectOneCall("TwoWire::beginTransmission")
      .withParameter("address", 1);
  mock().expectOneCall("TwoWire::write")
      .withParameter("data", 2);
  mock().expectOneCall("TwoWire::endTransmission")
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::requestFrom")
      .withParameter("address", 1)
      .withParameter("quantity", 2)
      .andReturnValue(0);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(2);
  mock().expectOneCall("TwoWire::read")
      .andReturnValue(123);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(1);
  mock().expectOneCall("TwoWire::read")
      .andReturnValue(222);
  mock().expectOneCall("TwoWire::available")
      .andReturnValue(0);

  uint8_t buf[2] = { 0 };

  LONGS_EQUAL(2, I2CreadFrom(1, 2, 2, buf, 1));
  UNSIGNED_LONGS_EQUAL(123, buf[0]);
  UNSIGNED_LONGS_EQUAL(222, buf[1]);

  mock().checkExpectations();
}

TEST(Drivers, readDS1307)
{
  uint8_t rBuf[8] =
  {
      // Mon 2020-03-02 23:59
      B01111111,
      0x59,
      0x23,
      static_cast<uint8_t>(Weekdays::Monday),
      0x02,
      0x03,
      0x20,
      0x00
  };

  datetime_t dt;

  mockSetup_I2CreadFrom(DS1307_ADDRESS, 0, 8, rBuf, 0);

  CHECK_TRUE(readDS1307(dt));

  UNSIGNED_LONGS_EQUAL(static_cast<uint8_t>(Weekdays::Monday), dt.date.dayOfWeek);
  UNSIGNED_LONGS_EQUAL(2020, dt.date.year);
  UNSIGNED_LONGS_EQUAL(3, dt.date.month);
  UNSIGNED_LONGS_EQUAL(2, dt.date.day);
  UNSIGNED_LONGS_EQUAL(23, dt.time.hour);
  UNSIGNED_LONGS_EQUAL(59, dt.time.minute);

  mock().checkExpectations();
}

TEST(Drivers, readDS1307_commError)
{
  datetime_t dt;

  mockSetup_I2CreadFrom_commError(DS1307_ADDRESS, 0, 8, 3);
  mock().expectOneCall("Print::println")
      .withParameter("c", "DS1307 comm error");

  CHECK_FALSE(readDS1307(dt));

  mock().checkExpectations();
}

TEST(Drivers, readDS1307_dataError)
{
  uint8_t rBuf[8] =
  {
      // Mon 2020-03-02 23:59
      B01111111,
      0x59,
      0x23,
      static_cast<uint8_t>(Weekdays::Monday),
      0x02,
      0x03,
      0x20,
      0xFF
  };

  datetime_t dt;

  mockSetup_I2CreadFrom(DS1307_ADDRESS, 0, 8, rBuf, 0);
  mock().expectOneCall("Print::println")
      .withParameter("c", "DS1307 fata1 error");

  CHECK_FALSE(readDS1307(dt));

  mock().checkExpectations();
}

TEST(Drivers, readDS1307_dateTimeError)
{
  uint8_t rBuf[8] =
  {
      // Mon 2020-03-02 23:59
      B01111111,
      0x60,
      0x23,
      static_cast<uint8_t>(Weekdays::Monday),
      0x02,
      0x03,
      0x20,
      0x00
  };

  datetime_t dt;

  mockSetup_I2CreadFrom(DS1307_ADDRESS, 0, 8, rBuf, 0);
  mock().expectOneCall("Print::println")
      .withParameter("c", "DS1307 data2 error");

  CHECK_FALSE(readDS1307(dt));

  mock().checkExpectations();
}

TEST(Drivers, setDS1307)
{
  const datetime_t dt =
  {
      // Mon 2020-03-02 23:59
      { 23, 59 },
      { 0, 2, 3, 2020 }
  };

  uint8_t rBuf[7] = { 0 };
  uint8_t wBuf[7] =
  {
      0x00,
      0x59,
      0x23,
      static_cast<uint8_t>(Weekdays::Monday),
      0x02,
      0x03,
      0x20
  };

  mockSetup_I2CreadFrom(DS1307_ADDRESS, 0, 7, rBuf, 0);
  mockSetup_I2CwriteTo(DS1307_ADDRESS, 0, 7, wBuf);

  CHECK_TRUE(setDS1307(dt));

  mock().checkExpectations();
}

TEST(Drivers, setDS1307_commError)
{
  const datetime_t dt =
  {
      // Mon 2020-03-02 23:59
      { 23, 59 },
      { 0, 2, 3, 2020 }
  };

  mockSetup_I2CreadFrom_commError(DS1307_ADDRESS, 0, 7, 3);
  mock().expectOneCall("Print::println")
      .withParameter("c", "DS1307 comm error");

  CHECK_FALSE(setDS1307(dt));

  mock().checkExpectations();
}
