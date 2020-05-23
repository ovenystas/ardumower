/*
 * Motor_test.cpp
 *
 *  Created on: May 22, 2020
 *      Author: ove
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#define protected public
#include "MotorMock.h"

// MotorInit ------------------------------------------------------------

TEST_GROUP(MotorInit)
{
  MotorMock* motor_p;

  void setup()
  {
  }

  void teardown()
  {
    if (motor_p)
    {
      delete motor_p;
    }
    mock().clear();
  }
};

TEST(MotorInit, init)
{
  motor_p = new MotorMock();
}

// Motor ------------------------------------------------------------

TEST_GROUP(Motor)
{
  MotorMock* motDrv_p;

  void setup()
  {
    motDrv_p = new MotorMock();
  }

  void teardown()
  {
    if (motDrv_p)
    {
      delete motDrv_p;
    }
    mock().clear();
  }
};

TEST(Motor, getAverageSenseAdc_stopped)
{
  LONGS_EQUAL(0, motDrv_p->getAverageSenseAdc());
}

TEST(Motor, getAverageCurrent_stopped)
{
  LONGS_EQUAL(0, motDrv_p->getAverageCurrent());
}

TEST(Motor, calcPower_stopped_batVoltAsFloat)
{
  float batVolt_V = 10.0f;
  motDrv_p->calcPower(batVolt_V);
  LONGS_EQUAL(0, motDrv_p->m_powerMeas);
}

TEST(Motor, calcPower_stopped_batMilliVoltAsInt)
{
  int16_t batVolt_mV = 10000;
  motDrv_p->calcPower(batVolt_mV);
  LONGS_EQUAL(0, motDrv_p->m_powerMeas);
}

TEST(Motor, set_and_get_filterAlpha)
{
  DOUBLES_EQUAL(1.0f, motDrv_p->getFilterAlpha(), 0.01);
  motDrv_p->setFilterAlpha(0.67f);
  DOUBLES_EQUAL(0.67f, motDrv_p->getFilterAlpha(), 0.01);
}

TEST(Motor, setChannel)
{
  UNSIGNED_LONGS_EQUAL(0, motDrv_p->m_channel);
  motDrv_p->setChannel(3);
  UNSIGNED_LONGS_EQUAL(3, motDrv_p->m_channel);
}

TEST(Motor, set_and_get_scale)
{
  DOUBLES_EQUAL(3.25839f, motDrv_p->getScale(), 0.0001);
  motDrv_p->setScale(0.67f);
  DOUBLES_EQUAL(0.67f, motDrv_p->getScale(), 0.0001);
}

