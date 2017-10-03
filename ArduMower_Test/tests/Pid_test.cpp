#include "Pid.h"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

static Pid pid;

// PidPGroup ------------------------------------------------------------------

TEST_GROUP(PidPGroup)
{
  void setup()
  {
    mock().expectOneCall("micros").andReturnValue(0ul);
    pid_setup(1.0, 0.0, 0.0, -1.0, 1.0, 1.0, &pid);
    mock().checkExpectations();
  }

  void teardown()
  {
      mock().clear();
  }
};

TEST(PidPGroup, Setup)
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

TEST(PidPGroup, Compute_firstStepTooLarge)
{
  mock().expectOneCall("micros").andReturnValue(2000000ul);
  DOUBLES_EQUAL(-1.0, pid_compute(1.0, &pid), 0.01);
  mock().checkExpectations();
}

TEST(PidPGroup, Compute_pvStep)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);
  DOUBLES_EQUAL(-1.0, pid_compute(1.0, &pid), 0.01);
  mock().checkExpectations();
}

TEST(PidPGroup, Compute_setPointStep)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);

  pid.setPoint = 1.0;

  DOUBLES_EQUAL(1.0, pid_compute(0.0, &pid), 0.01);
  mock().checkExpectations();
}

TEST(PidPGroup, Compute_yMinlimit)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);

  pid.setPoint = -2.0;

  DOUBLES_EQUAL(-1.0, pid_compute(0.0, &pid), 0.01);
  mock().checkExpectations();
}

TEST(PidPGroup, Compute_yMaxlimit)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);

  pid.setPoint = 2.0;

  DOUBLES_EQUAL(1.0, pid_compute(0.0, &pid), 0.01);
  mock().checkExpectations();
}

// PidPIGroup -----------------------------------------------------------------

TEST_GROUP(PidPIGroup)
{
  void setup()
  {
    mock().expectOneCall("micros").andReturnValue(0ul);
    pid_setup(1.0, 1.0, 0.0, -10.0, 10.0, 5.0, &pid);
    mock().checkExpectations();
  }

  void teardown()
  {
      mock().clear();
  }
};

TEST(PidPIGroup, Setup)
{
  DOUBLES_EQUAL(1.0, pid.settings.Kp, 0.01);
  DOUBLES_EQUAL(1.0, pid.settings.Ki, 0.01);
  DOUBLES_EQUAL(0.0, pid.settings.Kd, 0.01);
  DOUBLES_EQUAL(-10.0, pid.yMin, 0.01);
  DOUBLES_EQUAL(10.0, pid.yMax, 0.01);
  DOUBLES_EQUAL(5.0, pid.maxOutput, 0.01);
  DOUBLES_EQUAL(0.0, pid.errorOld, 0.01);
  DOUBLES_EQUAL(0.0, pid.errorSum, 0.01);
  DOUBLES_EQUAL(0.0, pid.setPoint, 0.01);
}

TEST(PidPIGroup, Compute_pvStep)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);
  DOUBLES_EQUAL(-2.0, pid_compute(1.0, &pid), 0.01);

  mock().expectOneCall("micros").andReturnValue(2000000ul);
  DOUBLES_EQUAL(-3.0, pid_compute(1.0, &pid), 0.01);

  mock().checkExpectations();
}

TEST(PidPIGroup, Compute_setPointStep)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);

  pid.setPoint = 1.0;

  DOUBLES_EQUAL(2.0, pid_compute(0.0, &pid), 0.01);

  mock().expectOneCall("micros").andReturnValue(2000000ul);
  DOUBLES_EQUAL(3.0, pid_compute(0.0, &pid), 0.01);

  mock().checkExpectations();
}

TEST(PidPIGroup, AntiWindUpMax)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);

  pid.setPoint = 2.0;

  DOUBLES_EQUAL(4.0, pid_compute(0.0, &pid), 0.01);

  mock().expectOneCall("micros").andReturnValue(2000000ul);
  DOUBLES_EQUAL(6.0, pid_compute(0.0, &pid), 0.01);

  mock().expectOneCall("micros").andReturnValue(3000000ul);
  DOUBLES_EQUAL(7.0, pid_compute(0.0, &pid), 0.01);

  mock().expectOneCall("micros").andReturnValue(4000000ul);
  DOUBLES_EQUAL(7.0, pid_compute(0.0, &pid), 0.01);

  mock().checkExpectations();
}

TEST(PidPIGroup, AntiWindUpMin)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);

  pid.setPoint = -2.0;

  DOUBLES_EQUAL(-4.0, pid_compute(0.0, &pid), 0.01);

  mock().expectOneCall("micros").andReturnValue(2000000ul);
  DOUBLES_EQUAL(-6.0, pid_compute(0.0, &pid), 0.01);

  mock().expectOneCall("micros").andReturnValue(3000000ul);
  DOUBLES_EQUAL(-7.0, pid_compute(0.0, &pid), 0.01);

  mock().expectOneCall("micros").andReturnValue(4000000ul);
  DOUBLES_EQUAL(-7.0, pid_compute(0.0, &pid), 0.01);

  mock().checkExpectations();
}

// PidPIDGroup ----------------------------------------------------------------

TEST_GROUP(PidPIDGroup)
{
  void setup()
  {
    mock().expectOneCall("micros").andReturnValue(0ul);
    pid_setup(1.0, 1.0, 1.0, -10.0, 10.0, 10.0, &pid);
    mock().checkExpectations();
  }

  void teardown()
  {
      mock().clear();
  }
};

TEST(PidPIDGroup, Setup)
{
  DOUBLES_EQUAL(1.0, pid.settings.Kp, 0.01);
  DOUBLES_EQUAL(1.0, pid.settings.Ki, 0.01);
  DOUBLES_EQUAL(1.0, pid.settings.Kd, 0.01);
  DOUBLES_EQUAL(-10.0, pid.yMin, 0.01);
  DOUBLES_EQUAL(10.0, pid.yMax, 0.01);
  DOUBLES_EQUAL(10.0, pid.maxOutput, 0.01);
  DOUBLES_EQUAL(0.0, pid.errorOld, 0.01);
  DOUBLES_EQUAL(0.0, pid.errorSum, 0.01);
  DOUBLES_EQUAL(0.0, pid.setPoint, 0.01);
}

TEST(PidPIDGroup, Compute_pvStep)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);
  DOUBLES_EQUAL(-3.0, pid_compute(1.0, &pid), 0.01);

  mock().expectOneCall("micros").andReturnValue(2000000ul);
  DOUBLES_EQUAL(-3.0, pid_compute(1.0, &pid), 0.01);

  mock().checkExpectations();
}

TEST(PidPIDGroup, Compute_setPointStep)
{
  mock().expectOneCall("micros").andReturnValue(1000000ul);

  pid.setPoint = 1.0;

  DOUBLES_EQUAL(3.0, pid_compute(0.0, &pid), 0.01);

  mock().expectOneCall("micros").andReturnValue(2000000ul);
  DOUBLES_EQUAL(3.0, pid_compute(0.0, &pid), 0.01);

  mock().checkExpectations();
}

