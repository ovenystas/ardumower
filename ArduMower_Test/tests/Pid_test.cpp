#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Pid.h"

// PidPInit ------------------------------------------------------------------

TEST_GROUP(PidPInit)
{
  Pid* pid_p;

  void setup()
  {
  }

  void teardown()
  {
    if (pid_p)
    {
      delete pid_p;
    }
      mock().clear();
  }
};

TEST(PidPInit, defaultInit_setup)
{
  mock().expectOneCall("micros")
      .andReturnValue(0u);

  pid_p = new Pid();
  pid_p->setup(1.0, 0.0, 0.0, -1.0, 1.0, 1.0);

  PidSettings* pidSettings_p = pid_p->getSettings();

  DOUBLES_EQUAL(1.0, pidSettings_p->Kp.value, 0.01);
  DOUBLES_EQUAL(0.0, pidSettings_p->Ki.value, 0.01);
  DOUBLES_EQUAL(0.0, pidSettings_p->Kd.value, 0.01);

  DOUBLES_EQUAL(-1.0, pid_p->m_yMin, 0.01);
  DOUBLES_EQUAL(1.0, pid_p->m_yMax, 0.01);
  DOUBLES_EQUAL(1.0, pid_p->m_maxOutput, 0.01);
  DOUBLES_EQUAL(0.0, pid_p->m_errorOld, 0.01);
  DOUBLES_EQUAL(0.0, pid_p->m_errorSum, 0.01);
  DOUBLES_EQUAL(0.0, pid_p->m_setPoint, 0.01);

  mock().checkExpectations();
}

TEST(PidPInit, parameterizedInit)
{
  mock().expectOneCall("micros")
      .andReturnValue(0u);

  pid_p = new Pid(1.0, 0.0, 0.0, -1.0, 1.0, 1.0);

  PidSettings* pidSettings_p = pid_p->getSettings();

  DOUBLES_EQUAL(1.0, pidSettings_p->Kp.value, 0.01);
  DOUBLES_EQUAL(0.0, pidSettings_p->Ki.value, 0.01);
  DOUBLES_EQUAL(0.0, pidSettings_p->Kd.value, 0.01);

  DOUBLES_EQUAL(-1.0, pid_p->m_yMin, 0.01);
  DOUBLES_EQUAL(1.0, pid_p->m_yMax, 0.01);
  DOUBLES_EQUAL(1.0, pid_p->m_maxOutput, 0.01);
  DOUBLES_EQUAL(0.0, pid_p->m_errorOld, 0.01);
  DOUBLES_EQUAL(0.0, pid_p->m_errorSum, 0.01);
  DOUBLES_EQUAL(0.0, pid_p->m_setPoint, 0.01);

  mock().checkExpectations();
}

// PidP ------------------------------------------------------------

TEST_GROUP(PidP)
{
  Pid* pid_p;

  void setup()
  {
    mock().disable();
    pid_p = new Pid(1.0, 0.0, 0.0, -1.0, 1.0, 1.0);
    mock().enable();
  }

  void teardown()
  {
    if (pid_p)
    {
      delete pid_p;
    }
    mock().clear();
  }
};

TEST(PidP, Compute_firstStepTooLarge)
{
  mock().expectOneCall("micros")
      .andReturnValue(2000000u);

  DOUBLES_EQUAL(-1.0, pid_p->compute(1.0), 0.01);

  mock().checkExpectations();
}

TEST(PidP, Compute_pvStep)
{
  mock().expectOneCall("micros")
      .andReturnValue(1000000u);

  DOUBLES_EQUAL(-1.0, pid_p->compute(1.0), 0.01);

  mock().checkExpectations();
}

TEST(PidP, Compute_setPointStep)
{
  mock().expectOneCall("micros")
      .andReturnValue(1000000u);

  pid_p->setSetPoint(1.0);

  DOUBLES_EQUAL(1.0, pid_p->compute(0.0), 0.01);

  mock().checkExpectations();
}

TEST(PidP, Compute_yMinlimit)
{
  mock().expectOneCall("micros")
      .andReturnValue(1000000u);

  pid_p->setSetPoint(-2.0);

  DOUBLES_EQUAL(-1.0, pid_p->compute(0.0), 0.01);

  mock().checkExpectations();
}

TEST(PidP, Compute_yMaxlimit)
{
  mock().expectOneCall("micros")
      .andReturnValue(1000000u);

  pid_p->setSetPoint(2.0);

  DOUBLES_EQUAL(1.0, pid_p->compute(0.0), 0.01);

  mock().checkExpectations();
}

// PidPI -----------------------------------------------------------------

TEST_GROUP(PidPI)
{
  Pid* pid_p;

  void setup()
  {
    mock().disable();
    pid_p = new Pid(1.0, 1.0, 0.0, -10.0, 10.0, 5.0);
    mock().enable();
  }

  void teardown()
  {
    if (pid_p)
    {
      delete pid_p;
    }
    mock().clear();
  }
};

TEST(PidPI, Compute_pvStep)
{
  mock().expectOneCall("micros")
      .andReturnValue(1000000u);
  mock().expectOneCall("micros")
      .andReturnValue(2000000u);

  DOUBLES_EQUAL(-2.0, pid_p->compute(1.0), 0.01);
  DOUBLES_EQUAL(-3.0, pid_p->compute(1.0), 0.01);

  mock().checkExpectations();
}

TEST(PidPI, Compute_setPointStep)
{
  mock().expectOneCall("micros")
      .andReturnValue(1000000u);
  mock().expectOneCall("micros")
      .andReturnValue(2000000u);

  pid_p->setSetPoint(1.0);

  DOUBLES_EQUAL(2.0, pid_p->compute(0.0), 0.01);
  DOUBLES_EQUAL(3.0, pid_p->compute(0.0), 0.01);

  mock().checkExpectations();
}

TEST(PidPI, AntiWindUpMax)
{
  mock().expectOneCall("micros")
      .andReturnValue(1000000u);
  mock().expectOneCall("micros")
      .andReturnValue(2000000u);
  mock().expectOneCall("micros")
      .andReturnValue(3000000u);
  mock().expectOneCall("micros")
      .andReturnValue(4000000u);

  pid_p->setSetPoint(2.0);

  DOUBLES_EQUAL(4.0, pid_p->compute(0.0), 0.01);
  DOUBLES_EQUAL(6.0, pid_p->compute(0.0), 0.01);
  DOUBLES_EQUAL(7.0, pid_p->compute(0.0), 0.01);
  DOUBLES_EQUAL(7.0, pid_p->compute(0.0), 0.01);

  mock().checkExpectations();
}

TEST(PidPI, AntiWindUpMin)
{
  mock().expectOneCall("micros")
      .andReturnValue(1000000u);
  mock().expectOneCall("micros")
      .andReturnValue(2000000u);
  mock().expectOneCall("micros")
      .andReturnValue(3000000u);
  mock().expectOneCall("micros")
      .andReturnValue(4000000u);

  pid_p->setSetPoint(-2.0);

  DOUBLES_EQUAL(-4.0, pid_p->compute(0.0), 0.01);
  DOUBLES_EQUAL(-6.0, pid_p->compute(0.0), 0.01);
  DOUBLES_EQUAL(-7.0, pid_p->compute(0.0), 0.01);
  DOUBLES_EQUAL(-7.0, pid_p->compute(0.0), 0.01);

  mock().checkExpectations();
}

// PidPID ----------------------------------------------------------------

TEST_GROUP(PidPID)
{
  Pid* pid_p;

  void setup()
  {
    mock().disable();
    pid_p = new Pid(1.0, 1.0, 1.0, -10.0, 10.0, 10.0);
    mock().enable();
  }

  void teardown()
  {
    if (pid_p)
    {
      delete pid_p;
    }
    mock().clear();
  }
};

TEST(PidPID, Compute_pvStep)
{
  mock().expectOneCall("micros")
      .andReturnValue(1000000u);
  mock().expectOneCall("micros")
      .andReturnValue(2000000u);

  DOUBLES_EQUAL(-3.0, pid_p->compute(1.0), 0.01);
  DOUBLES_EQUAL(-3.0, pid_p->compute(1.0), 0.01);

  mock().checkExpectations();
}

TEST(PidPID, Compute_setPointStep)
{
  mock().expectOneCall("micros")
      .andReturnValue(1000000u);
  mock().expectOneCall("micros")
      .andReturnValue(2000000u);

  pid_p->setSetPoint(1.0);

  DOUBLES_EQUAL(3.0, pid_p->compute(0.0), 0.01);
  DOUBLES_EQUAL(3.0, pid_p->compute(0.0), 0.01);

  mock().checkExpectations();
}
