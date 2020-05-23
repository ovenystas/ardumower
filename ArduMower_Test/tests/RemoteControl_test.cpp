/*
 * RemoteControl_test.cpp
 *
 *  Created on: May 19, 2020
 *      Author: ove
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "RemoteControl.h"
#include "RobotMock.h"

// RemoteControlInit ------------------------------------------------------------

TEST_GROUP(RemoteControlInit)
{
  RobotMock* robot_p;
  RemoteControl* rc_p;

  void setup()
  {
  }

  void teardown()
  {
    if (rc_p)
    {
      delete rc_p;
    }
    mock().clear();
  }
};

//TEST(RemoteControlInit, init)
//{
//  robot_p = new RobotMock();
//  rc_p = new RemoteControl(robot_p);
//}
