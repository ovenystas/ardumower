#include "Pid.h"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

static Pid pid;

TEST_GROUP(PidGroup)
{
  void setup()
  {
    mock().expectOneCall("micros").andReturnValue(0ul);
    pid_setup(1.0f, 0.0f, 0.0f, -1.0f, 1.0f, 1.0f, &pid);
    mock().checkExpectations();
  }

  void teardown()
  {
      mock().clear();
  }
};

unsigned long micros()
{
    mock().actualCall("micros");
    return mock().unsignedLongIntReturnValue();
}

TEST(PidGroup, Setup)
{
  DOUBLES_EQUAL(1.0, pid.settings.Kp, 0.01);
  DOUBLES_EQUAL(0.0, pid.settings.Ki, 0.01);
  DOUBLES_EQUAL(0.0, pid.settings.Kd, 0.01);
  DOUBLES_EQUAL(-1.0, pid.yMin, 0.01);
  DOUBLES_EQUAL(1.0, pid.yMax, 0.01);
  DOUBLES_EQUAL(1.0, pid.maxOutput, 0.01);
  DOUBLES_EQUAL(0.0, pid.errorOld, 0.01);
  DOUBLES_EQUAL(0.0, pid.errorSum, 0.01);
  DOUBLES_EQUAL(0.0, pid.setPoint, 0.01);
}

TEST(PidGroup, Compute_pvStep)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);
  pid_compute(1.0, &pid);
  //DOUBLES_EQUAL(-1.0, pid_compute(1.0, &pid), 0.01);
  mock().checkExpectations();
}

TEST(PidGroup, Compute_setPointStep)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);

  pid.setPoint = 1.0;

  DOUBLES_EQUAL(1.0, pid_compute(0.0, &pid), 0.01);
  mock().checkExpectations();
}

TEST(PidGroup, Compute_yMinlimit)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);

  pid.setPoint = -2.0;

  DOUBLES_EQUAL(-1.0, pid_compute(0.0, &pid), 0.01);
  mock().checkExpectations();
}

TEST(PidGroup, Compute_yMaxlimit)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);

  pid.setPoint = 2.0;

  DOUBLES_EQUAL(1.0, pid_compute(0.0, &pid), 0.01);
  mock().checkExpectations();
}

