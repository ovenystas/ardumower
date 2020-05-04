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
        mock().expectOneCall("print")
            .onObject(&Console)
            .withParameter("val", s[i]);
        mock().expectOneCall("available")
            .onObject(&Bluetooth)
            .andReturnValue(static_cast<int>(s.length() - i - 1));
      }
    }
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

  mockSetup_writeBT(sendStr);
  mock().expectOneCall("delay")
      .withParameter("ms", 2000);
  mockSetup_readBT(receiveStr);
  mock().expectOneCall("println")
      .onObject(&Console);

  bt_p->writeReadBT(sendStr);

  STRCMP_EQUAL(receiveStr.c_str(), bt_p->m_btResult.c_str());

  mock().checkExpectations();
}
