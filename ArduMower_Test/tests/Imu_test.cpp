#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Imu.h"

// ImuInit ------------------------------------------------------------

TEST_GROUP(ImuInit)
{
  Imu* imu_p;

  void setup()
  {
  }

  void teardown()
  {
    if (imu_p)
    {
      delete imu_p;
    }
    mock().clear();
  }
};

TEST(ImuInit, init)
{
  mock().expectOneCall("LSM303::LSM303");

  imu_p = new Imu();

  mock().checkExpectations();
}

// Imu ------------------------------------------------------------

TEST_GROUP(Imu)
{
  Imu* imu_p;

  void setup()
  {
    mock().disable();
    imu_p = new Imu();
    mock().enable();
  }

  void teardown()
  {
    if (imu_p)
    {
      delete imu_p;
    }
    mock().clear();
  }
};

TEST(Imu, getAndClearCallCounter)
{
  imu_p->m_callCounter = 12345;

  UNSIGNED_LONGS_EQUAL(12345, imu_p->getAndClearCallCounter());
  UNSIGNED_LONGS_EQUAL(0, imu_p->m_callCounter);

  mock().checkExpectations();
}
