#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "BluetoothConfig.h"
#include "HardwareSerial.h"
#include "Drivers.h"

// BluetoothConfigInit ------------------------------------------------------------

TEST_GROUP(BluetoothConfigInit)
{
  BluetoothConfig* bt_p;

  void setup()
  {
  }

  void teardown()
  {
    if (bt_p)
    {
      delete bt_p;
    }
    mock().clear();
  }
};

TEST(BluetoothConfigInit, init)
{
  bt_p = new BluetoothConfig();

  STRCMP_EQUAL("", bt_p->m_btResult.c_str());

  mock().checkExpectations();
}

// BluetoothConfig ------------------------------------------------------------

TEST_GROUP(BluetoothConfig)
{
  BluetoothConfig* bt_p;

  void setup()
  {
    mock().disable();
    bt_p = new BluetoothConfig();
    mock().enable();
  }

  void teardown()
  {
    if (bt_p)
    {
      delete bt_p;
    }
    mock().clear();
  }

  void mockSetup_writeBT(const String s)
  {
    mock().expectOneCall("print")
        .onObject(&Console)
        .withParameter("val", "Send: ");
    mock().expectOneCall("print")
        .onObject(&Console)
        .withParameter("val", s);
    mock().expectOneCall("print")
        .onObject(&Bluetooth)
        .withParameter("val", s);
  }

  void mockSetup_readBT(const String s)
  {
    mock().expectOneCall("available")
        .onObject(&Bluetooth)
        .andReturnValue(static_cast<int>(s.length()));

    if (s.length() > 0)
    {
      mock().expectOneCall("print")
          .onObject(&Console)
          .withParameter("val", ", received: ");

      for (unsigned int i = 0; i < s.length(); ++i)
      {
        mock().expectOneCall("read")
            .onObject(&Bluetooth)
            .andReturnValue(s[i]);
        mock().expectOneCall("available")
            .onObject(&Bluetooth)
            .andReturnValue(static_cast<int>(s.length() - i - 1));
      }

      mock().expectOneCall("println")
          .onObject(&Console)
          .withParameter("val", s);
    }
  }

  void mockSetup_writeReadBT(const String s, const String r)
  {
    mockSetup_writeBT(s);
    mock().expectOneCall("delay")
        .withParameter("ms", 2000);
    mockSetup_readBT(r);
    mock().expectOneCall("println")
        .onObject(&Console);
  }
};

TEST(BluetoothConfig, printSuccess)
{
  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "=>success");

  bt_p->printSuccess();

  mock().checkExpectations();
}


TEST(BluetoothConfig, baudrateToN_valid)
{
  UNSIGNED_LONGS_EQUAL(1, bt_p->baudrateToN(1200));
  UNSIGNED_LONGS_EQUAL(2, bt_p->baudrateToN(2400));
  UNSIGNED_LONGS_EQUAL(3, bt_p->baudrateToN(4800));
  UNSIGNED_LONGS_EQUAL(4, bt_p->baudrateToN(9600));
  UNSIGNED_LONGS_EQUAL(5, bt_p->baudrateToN(19200));
  UNSIGNED_LONGS_EQUAL(6, bt_p->baudrateToN(38400));
  UNSIGNED_LONGS_EQUAL(7, bt_p->baudrateToN(57600));
  UNSIGNED_LONGS_EQUAL(8, bt_p->baudrateToN(115200));

  mock().checkExpectations();
}

TEST(BluetoothConfig, baudrateToN_invalid)
{
  mock().expectOneCall("print")
      .onObject(&Console)
      .withParameter("val", "Invalid baudrate: ");
  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", 0);

  UNSIGNED_LONGS_EQUAL(4, bt_p->baudrateToN(0));

  mock().checkExpectations();
}

TEST(BluetoothConfig, writeBT)
{
  const String sendStr = "Hello world!";

  mockSetup_writeBT(sendStr);

  bt_p->writeBT(sendStr);

  mock().checkExpectations();
}

TEST(BluetoothConfig, readBT_unavailable)
{
  const String receiveStr = "";

  mockSetup_readBT(receiveStr);

  bt_p->readBT();

  STRCMP_EQUAL(receiveStr.c_str(), bt_p->m_btResult.c_str());

  mock().checkExpectations();
}

TEST(BluetoothConfig, readBT_available)
{
  const String receiveStr = "ab";

  mockSetup_readBT(receiveStr);

  bt_p->readBT();

  STRCMP_EQUAL(receiveStr.c_str(), bt_p->m_btResult.c_str());

  mock().checkExpectations();
}

TEST(BluetoothConfig, writeReadBT_ok)
{
  const String sendStr = "Hello world!";
  const String receiveStr = "xyz";

  mockSetup_writeReadBT(sendStr, receiveStr);

  bt_p->writeReadBT(sendStr);

  STRCMP_EQUAL(receiveStr.c_str(), bt_p->m_btResult.c_str());

  mock().checkExpectations();
}

TEST(BluetoothConfig, writeReadBT_retryOnError)
{
  const String sendStr = "Hello world!";
  const String receiveStr = "ERROR";

  for (uint8_t i = 0; i < 4; ++i)
  {
    mockSetup_writeBT(sendStr);
    mock().expectOneCall("delay")
        .withParameter("ms", 2000);
    mockSetup_readBT(receiveStr);
  }
  mock().expectOneCall("println")
      .onObject(&Console);

  bt_p->writeReadBT(sendStr);

  STRCMP_EQUAL(receiveStr.c_str(), bt_p->m_btResult.c_str());

  mock().checkExpectations();
}

TEST(BluetoothConfig, detectBaudrate_quick_9600_8N1_HC05)
{
  mock().expectOneCall("println")
      .onObject(&Console);
  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "Detecting baudrate...");

  mock().expectOneCall("print")
      .onObject(&Console)
      .withParameter("val", "Trying baudrate ");
  mock().expectOneCall("print")
      .onObject(&Console)
      .withParameter("val", 9600);
  mock().expectOneCall("print")
      .onObject(&Console)
      .withParameter("val", " config ");
  mock().expectOneCall("print")
      .onObject(&Console)
      .withParameter("val", 0);
  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "...");

  mock().expectOneCall("begin")
      .onObject(&Bluetooth)
      .withParameter("baud", 9600)
      .withParameter("config", SERIAL_8N1);

  mockSetup_writeReadBT("AT", "");
  mockSetup_writeReadBT("AT\r\n", "OK");

  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "=>success");

  CHECK_TRUE(bt_p->detectBaudrate(true));

  mock().checkExpectations();
}

TEST(BluetoothConfig, detectModuleType_LINVOR_HC06)
{
  mock().expectOneCall("println")
      .onObject(&Console);
  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "Detecting BT type...");

  mockSetup_writeReadBT("AT+VERSION", "OKlinvor");

  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "=>it's a linvor/HC06");

  ENUMS_EQUAL_INT(BluetoothType::LINVOR_HC06, bt_p->detectModuleType());

  mock().checkExpectations();
}

TEST(BluetoothConfig, detectModuleType_HC05)
{
  mock().expectOneCall("println")
      .onObject(&Console);
  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "Detecting BT type...");

  mockSetup_writeReadBT("AT+VERSION", "");
  mockSetup_writeReadBT("AT+VERSION?\r\n", "OK");

  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "=>must be a HC03/04/05 ?");

  ENUMS_EQUAL_INT(BluetoothType::HC05, bt_p->detectModuleType());

  mock().checkExpectations();
}

TEST(BluetoothConfig, detectModuleType_FBT06_MBTV4)
{
  mock().expectOneCall("println")
      .onObject(&Console);
  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "Detecting BT type...");

  mockSetup_writeReadBT("AT+VERSION", "");
  mockSetup_writeReadBT("AT+VERSION?\r\n", "");
  mockSetup_writeReadBT("AT+VERSION\r\n", "ModiaTek");

  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "=>it's a FBT06/MBTV4");

  ENUMS_EQUAL_INT(BluetoothType::FBT06_MBTV4, bt_p->detectModuleType());

  mock().checkExpectations();
}

TEST(BluetoothConfig, detectModuleType_UNKNOWN)
{
  mock().expectOneCall("println")
      .onObject(&Console);
  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "Detecting BT type...");

  mockSetup_writeReadBT("AT+VERSION", "");
  mockSetup_writeReadBT("AT+VERSION?\r\n", "");
  mockSetup_writeReadBT("AT+VERSION\r\n", "");

  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "=>unknown");

  ENUMS_EQUAL_INT(BluetoothType::UNKNOWN, bt_p->detectModuleType());

  mock().checkExpectations();
}

TEST(BluetoothConfig, setName_HC05)
{
  const String name = "AwesomeBT";

  mock().expectOneCall("println")
      .onObject(&Console);
  mock().expectOneCall("print")
      .onObject(&Console)
      .withParameter("val", "Setting name ");
  mock().expectOneCall("print")
      .onObject(&Console)
      .withParameter("val", name);
  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "...");

  mockSetup_writeReadBT("AT+NAME=" + name + "\r\n", "OK");

  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "=>success");

  bt_p->setName(name, BluetoothType::HC05);

  mock().checkExpectations();
}

TEST(BluetoothConfig, setPinCode_HC05)
{
  const uint32_t pinCode = 1234;

  mock().expectOneCall("println")
      .onObject(&Console);
  mock().expectOneCall("print")
      .onObject(&Console)
      .withParameter("val", "Setting pin code ");
  mock().expectOneCall("print")
      .onObject(&Console)
      .withParameter("val", pinCode);
  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "...");

  mockSetup_writeReadBT("AT+PSWD=1234\r\n", "OK");

  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "=>success");

  bt_p->setPinCode(pinCode, BluetoothType::HC05);

  mock().checkExpectations();
}

TEST(BluetoothConfig, setBaudrate_HC05)
{
  const uint32_t baudrate = 9600;

  mock().expectOneCall("println")
      .onObject(&Console);
  mock().expectOneCall("print")
      .onObject(&Console)
      .withParameter("val", "Setting baudrate ");
  mock().expectOneCall("print")
      .onObject(&Console)
      .withParameter("val", baudrate);
  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "...");

  mockSetup_writeReadBT("AT+UART=9600,0,0\r\n", "OK");

  mock().expectOneCall("println")
      .onObject(&Console)
      .withParameter("val", "=>success");

  bt_p->setBaudrate(baudrate, BluetoothType::HC05);

  mock().checkExpectations();
}
