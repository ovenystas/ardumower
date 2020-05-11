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

    Console.setMockPrintFunctions(false);
    Console.setMockPrintToStdout(false);
    Console.startCapture();

    Bluetooth.setMockPrintFunctions(false);
    Bluetooth.setMockPrintToStdout(false);
    Bluetooth.startCapture();
}

  void teardown()
  {
    if (bt_p)
    {
      delete bt_p;
    }
    mock().clear();

    Console.stopCapture();
    Console.setMockPrintFunctions(true);
    Console.setMockPrintToStdout(false);

    Bluetooth.stopCapture();
    Bluetooth.setMockPrintFunctions(true);
    Bluetooth.setMockPrintToStdout(false);
}

  void mockSetup_readBT(const String s)
  {
    mock().expectOneCall("HardwareSerial::available")
        .onObject(&Bluetooth)
        .andReturnValue(static_cast<int>(s.length()));

    if (s.length() > 0)
    {
      for (unsigned int i = 0; i < s.length(); ++i)
      {
        mock().expectOneCall("HardwareSerial::read")
            .onObject(&Bluetooth)
            .andReturnValue(s[i]);
        mock().expectOneCall("HardwareSerial::available")
            .onObject(&Bluetooth)
            .andReturnValue(static_cast<int>(s.length() - i - 1));
      }
    }
  }

  void mockSetup_writeReadBT(const String r)
  {
    mock().expectOneCall("delay")
        .withParameter("ms", 2000);
    mockSetup_readBT(r);
  }
};

TEST(BluetoothConfig, printSuccess)
{
  bt_p->printSuccess();

  STRCMP_EQUAL("  =>success\r\n", Console.getMockOutString());

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
  UNSIGNED_LONGS_EQUAL(4, bt_p->baudrateToN(0));

  STRCMP_EQUAL("Invalid baudrate: 0\r\n", Console.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, writeBT)
{
  const String sendStr = "Hello world!";

  bt_p->writeBT(sendStr);

  STRCMP_EQUAL("  Send: Hello world!", Console.getMockOutString());

  STRCMP_EQUAL(sendStr.c_str(), Bluetooth.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, readBT_unavailable)
{
  const String receiveStr = "";

  mockSetup_readBT(receiveStr);

  bt_p->readBT();

  STRCMP_EQUAL(receiveStr.c_str(), bt_p->m_btResult.c_str());

  STRCMP_EQUAL("", Console.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, readBT_available)
{
  const String receiveStr = "ab";

  mockSetup_readBT(receiveStr);

  bt_p->readBT();

  STRCMP_EQUAL(receiveStr.c_str(), bt_p->m_btResult.c_str());

  STRCMP_EQUAL(
      "  Received: ab\r\n",
      Console.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, writeReadBT_ok)
{
  const String sendStr = "Hello world!";
  const String receiveStr = "xyz";

  mockSetup_writeReadBT(receiveStr);

  bt_p->writeReadBT(sendStr);

  STRCMP_EQUAL(receiveStr.c_str(), bt_p->m_btResult.c_str());

  STRCMP_EQUAL(
      "  Send: Hello world!  Received: xyz\r\n",
      Console.getMockOutString());

  STRCMP_EQUAL(
      sendStr.c_str(),
      Bluetooth.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, writeReadBT_retryOnError)
{
  const String sendStr = "Hello world!";
  const String receiveStr = "ERROR";

  for (uint8_t i = 0; i < 4; ++i)
  {
    mock().expectOneCall("delay")
        .withParameter("ms", 2000);
    mockSetup_readBT(receiveStr);
  }

  bt_p->writeReadBT(sendStr);

  STRCMP_EQUAL(receiveStr.c_str(), bt_p->m_btResult.c_str());

  STRCMP_EQUAL(String(
      "  Send: " + sendStr + "  Received: " + receiveStr + "\r\n" +
      "  Send: " + sendStr + "  Received: " + receiveStr + "\r\n" +
      "  Send: " + sendStr + "  Received: " + receiveStr + "\r\n" +
      "  Send: " + sendStr + "  Received: " + receiveStr + "\r\n").c_str(),
      Console.getMockOutString());

  STRCMP_EQUAL(
      (sendStr + sendStr + sendStr + sendStr).c_str(),
      Bluetooth.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, detectBaudrate_quick_9600_8N1_HC05)
{
  mock().expectOneCall("HardwareSerial::begin")
      .onObject(&Bluetooth)
      .withParameter("baud", 9600)
      .withParameter("config", SERIAL_8N1);

  mockSetup_writeReadBT("");
  mockSetup_writeReadBT("OK");

  CHECK_TRUE(bt_p->detectBaudrate(true));

  STRCMP_EQUAL(
      "Detecting baudrate...\r\n"
      "  Trying baudrate 9600 config 0...\r\n"
      "  Send: AT  Send: AT\r\n"
      "  Received: OK\r\n"
      "  =>success\r\n",
      Console.getMockOutString());

  STRCMP_EQUAL(
      "ATAT\r\n",
      Bluetooth.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, detectModuleType_LINVOR_HC06)
{
  mockSetup_writeReadBT("OKlinvor");

  ENUMS_EQUAL_INT(BluetoothType::LINVOR_HC06, bt_p->detectModuleType());

  STRCMP_EQUAL(
     "Detecting BT type...\r\n"
      "  Send: AT+VERSION  Received: OKlinvor\r\n"
      "  =>it's a linvor/HC06\r\n",
      Console.getMockOutString());

  STRCMP_EQUAL(
      "AT+VERSION",
      Bluetooth.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, detectModuleType_HC05)
{
  mockSetup_writeReadBT("");
  mockSetup_writeReadBT("OK");

  ENUMS_EQUAL_INT(BluetoothType::HC05, bt_p->detectModuleType());

  STRCMP_EQUAL(
      "Detecting BT type...\r\n"
      "  Send: AT+VERSION  Send: AT+VERSION?\r\n"
      "  Received: OK\r\n"
      "  =>must be a HC03/04/05 ?\r\n",
      Console.getMockOutString());

  STRCMP_EQUAL(
      "AT+VERSIONAT+VERSION?\r\n",
      Bluetooth.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, detectModuleType_FBT06_MBTV4)
{
  mockSetup_writeReadBT("");
  mockSetup_writeReadBT("");
  mockSetup_writeReadBT("ModiaTek");

  ENUMS_EQUAL_INT(BluetoothType::FBT06_MBTV4, bt_p->detectModuleType());

  STRCMP_EQUAL(
      "Detecting BT type...\r\n"
      "  Send: AT+VERSION  Send: AT+VERSION?\r\n"
      "  Send: AT+VERSION\r\n"
      "  Received: ModiaTek\r\n"
      "  =>it's a FBT06/MBTV4\r\n",
      Console.getMockOutString());

  STRCMP_EQUAL(
      "AT+VERSIONAT+VERSION?\r\nAT+VERSION\r\n",
      Bluetooth.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, detectModuleType_UNKNOWN)
{
  mockSetup_writeReadBT("");
  mockSetup_writeReadBT("");
  mockSetup_writeReadBT("");

  ENUMS_EQUAL_INT(BluetoothType::UNKNOWN, bt_p->detectModuleType());

  STRCMP_EQUAL(
      "Detecting BT type...\r\n"
      "  Send: AT+VERSION  Send: AT+VERSION?\r\n"
      "  Send: AT+VERSION\r\n"
      "  =>unknown\r\n",
      Console.getMockOutString());
  STRCMP_EQUAL(
      "AT+VERSIONAT+VERSION?\r\nAT+VERSION\r\n",
      Bluetooth.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, setName_HC05)
{
  const String name = "AwesomeBT";

  mockSetup_writeReadBT("OK");

  bt_p->setName(name, BluetoothType::HC05);

  STRCMP_EQUAL(
      "Setting name AwesomeBT...\r\n"
      "  Send: AT+NAME=AwesomeBT\r\n"
      "  Received: OK\r\n"
      "  =>success\r\n",
      Console.getMockOutString());

  STRCMP_EQUAL(
      "AT+NAME=AwesomeBT\r\n",
      Bluetooth.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, setPinCode_HC05)
{
  const uint32_t pinCode = 1234;

  mockSetup_writeReadBT("OK");

  bt_p->setPinCode(pinCode, BluetoothType::HC05);

  STRCMP_EQUAL(
      "Setting pin code 1234...\r\n"
      "  Send: AT+PSWD=1234\r\n"
      "  Received: OK\r\n"
      "  =>success\r\n",
      Console.getMockOutString());

  STRCMP_EQUAL(
      "AT+PSWD=1234\r\n",
      Bluetooth.getMockOutString());

  mock().checkExpectations();
}

TEST(BluetoothConfig, setBaudrate_HC05)
{
  const uint32_t baudrate = 9600;

  mockSetup_writeReadBT("OK");

  bt_p->setBaudrate(baudrate, BluetoothType::HC05);

  STRCMP_EQUAL(
      "Setting baudrate 9600...\r\n"
      "  Send: AT+UART=9600,0,0\r\n"
      "  Received: OK\r\n"
      "  =>success\r\n",
      Console.getMockOutString());

  STRCMP_EQUAL(
      "AT+UART=9600,0,0\r\n",
      Bluetooth.getMockOutString());

  mock().checkExpectations();
}
