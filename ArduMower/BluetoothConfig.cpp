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

#include "BluetoothConfig.h"

#include "Drivers.h"

void BluetoothConfig::writeBT(String s)
{
  Console.print(F("Send: "));
  Console.print(s);

  Bluetooth.print(s);
}

void BluetoothConfig::readBT()
{
  m_btResult = "";

  if (Bluetooth.available())
  {
    Console.print(F(", received: "));

    do
    {
      char btData = static_cast<char>(Bluetooth.read());
      m_btResult += btData;
      Console.print(btData);
    } while (Bluetooth.available());
  }
  //Console.print("btResult=");
  //Console.println(btResult);
}

void BluetoothConfig::writeReadBT(String s)
{
  uint8_t counter = 0;
  do
  {
    writeBT(s);
    delay(2000);
    readBT();
  } while (m_btResult.startsWith("ERROR") && counter++ < 4);

  Console.println();
}

void BluetoothConfig::setName(String name, BluetoothType btType)
{
  Console.println();
  Console.print(F("Setting name "));
  Console.print(name);
  Console.println("...");

  bool res = false;

  switch (btType)
  {
    case BluetoothType::LINVOR_HC06:
      writeReadBT("AT+NAME" + name);
      res = m_btResult.startsWith("OKsetname");
      break;

    case BluetoothType::HC05:
      writeReadBT("AT+NAME=" + name + "\r\n");
      res = m_btResult.indexOf("OK") != -1;
      break;

    case BluetoothType::FBT06_MBTV4:
      writeReadBT("AT+NAME" + name + "\r\n");
      res = m_btResult.indexOf("OK") != -1;
      break;

    case BluetoothType::UNKNOWN:
      break;

    default:
      break;
  }

  if (res)
  {
    printSuccess();
  }
  else
  {
    Console.println(F("=>error setting name"));
  }
}

void BluetoothConfig::setPinCode(uint16_t pinCode, BluetoothType btType)
{
  Console.println();
  Console.print(F("Setting pin code "));
  Console.print(pinCode);
  Console.println(F("..."));

  bool res = false;

  switch (btType)
  {
    case BluetoothType::LINVOR_HC06:
      writeReadBT("AT+PIN" + String(pinCode));
      res = m_btResult.startsWith("OKsetPIN");
      break;

    case BluetoothType::HC05:
      writeReadBT("AT+PSWD=" + String(pinCode) + "\r\n");
      res = m_btResult.indexOf("OK") != -1;
      break;

    case BluetoothType::FBT06_MBTV4:
      writeReadBT("AT+PIN" + String(pinCode) + "\r\n");
      res = (m_btResult.indexOf("OK") != -1);
      break;

    case BluetoothType::UNKNOWN:
      break;

    default:
      break;
  }

  if (res)
  {
    printSuccess();
  }
  else
  {
    Console.println(F("=>error setting pin"));
  }
}

uint8_t BluetoothConfig::baudrateToN(uint32_t baudrate)
{
  switch (baudrate)
  {
    case 1200:   return 1;
    case 2400:   return 2;
    case 4800:   return 3;
    case 9600:   return 4;
    case 19200:  return 5;
    case 38400:  return 6;
    case 57600:  return 7;
    case 115200: return 8;
    default:
      Console.print(F("Invalid baudrate: "));
      Console.println(baudrate);
      return 4; // Defaulting to 9600 baud
  }
}

void BluetoothConfig::printSuccess()
{
  Console.println(F("=>success"));
}

void BluetoothConfig::setBaudrate(uint32_t baudrate, BluetoothType btType)
{
  Console.println();
  Console.print(F("Setting baudrate "));
  Console.print(baudrate);
  Console.println(F("..."));

  bool res = false;

  switch (btType)
  {
    case BluetoothType::LINVOR_HC06:
    {
      uint8_t n = baudrateToN(baudrate);

      writeReadBT("AT+PN"); // no parity
      writeReadBT("AT+BAUD" + String(n));

      res = m_btResult.startsWith("OK" + String(baudrate));
      break;
    }

    case BluetoothType::HC05:
      writeReadBT("AT+UART=" + String(baudrate) + ",0,0\r\n");
      res = m_btResult.indexOf("OK") != -1;
      break;

    case BluetoothType::FBT06_MBTV4:
    {
      uint8_t n = baudrateToN(baudrate);

      writeReadBT("AT+BAUD" + String(n) + "\r\n");

      res = m_btResult.indexOf("OK") != -1;
      break;
    }

    case BluetoothType::UNKNOWN:
      break;

    default:
      break;
  }

  if (res)
  {
    printSuccess();
  }
  else
  {
    Console.println(F("=>error setting baudrate"));
  }
}

bool BluetoothConfig::detectBaudrate(bool quickBaudScan)
{
  const uint32_t testBaudrates[] =
      { 9600, 38400, 19200, 57600, 115200, 4800, 2400, 1200 };

  const uint8_t testConfigs[]
  {
      SERIAL_8N1, SERIAL_5N1, SERIAL_6N1, SERIAL_7N1,
      SERIAL_5N2, SERIAL_6N2, SERIAL_7N2, SERIAL_8N2,
      SERIAL_5E1, SERIAL_6E1, SERIAL_7E1, SERIAL_8E1,
      SERIAL_5E2, SERIAL_6E2, SERIAL_7E2, SERIAL_8E2,
      SERIAL_5O1, SERIAL_6O1, SERIAL_7O1, SERIAL_8O1,
      SERIAL_5O2, SERIAL_6O2, SERIAL_7O2, SERIAL_8O2
  };

  Console.println();
  Console.println(F("Detecting baudrate..."));

  for (uint32_t testBaudrate : testBaudrates)
  {
    uint8_t j = 0;
    for (uint8_t testConfig : testConfigs)
    {
      Console.print(F("Trying baudrate "));
      Console.print(testBaudrate);
      Console.print(F(" config "));
      Console.print(j++);
      Console.println(F("..."));

      Bluetooth.begin(testBaudrate, testConfig);

      writeReadBT("AT");  // linvor/HC06 does not want a terminator!
      if (m_btResult.startsWith("OK"))
      {
        printSuccess();
        return true;
      }

      writeReadBT("AT\r\n");  // HC05 wants a terminator!
      if (m_btResult.startsWith("OK"))
      {
        printSuccess();
        return true;
      }

      if (quickBaudScan)
      {
        break;
      }
    }
    //writeReadBT("ATI1\n"); // BTM info
    //writeReadBT("ATZ0\n"); // BTM factory
    //writeReadBT("ATC0\r\nATQ1\r\nATI1\r\n"); // BTM
  }
  Console.println(F("=>error detecting baudrate"));
  return false;
}

BluetoothType BluetoothConfig::detectModuleType()
{
  Console.println();
  Console.println(F("Detecting BT type..."));

  writeReadBT(F("AT+VERSION"));
  if (m_btResult.startsWith("OKlinvor"))
  {
    Console.println(F("=>it's a linvor/HC06"));
    return BluetoothType::LINVOR_HC06;
  }

  writeReadBT(F("AT+VERSION?\r\n"));
  if (m_btResult.indexOf("OK") != -1)
  {
    Console.println(F("=>must be a HC03/04/05 ?"));
    return BluetoothType::HC05;
  }

  writeReadBT(F("AT+VERSION\r\n"));
  if (m_btResult.indexOf("ModiaTek") != -1)
  {
    Console.println(F("=>it's a FBT06/MBTV4"));
    return BluetoothType::FBT06_MBTV4;
  }

  return BluetoothType::UNKNOWN;
}

void BluetoothConfig::setParams(String name, uint16_t pinCode, uint32_t baudrate,
    bool quickBaudScan)
{
  Console.println(F("HC-03/04/05/06/linvor/ModiaTek "
                    "Bluetooth config programmer"));

  Console.println(F("NOTE for HC05: Connect KEY pin to 3.3V!"));

  Console.println(F("NOTE for HC06/linvor: Do NOT pair/connect "
                    "(LED must be blinking)"));

  Console.println(F("NOTE for FBT06/MBTV4: First you have to solder "
                    "the PIO11 pin to VCC (PIN 12) which is 3.3 Volts "
                    "using a thin wire."));

  if (detectBaudrate(quickBaudScan))
  {
    BluetoothType btType = detectModuleType();
    if (btType != BluetoothType::UNKNOWN)
    {
      setName(name, btType);
      setPinCode(pinCode, btType);
      setBaudrate(baudrate, btType);
      Console.println(F("You may restart BT module now!"));
    }
  }
}
