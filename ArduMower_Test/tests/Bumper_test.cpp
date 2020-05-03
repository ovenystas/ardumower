#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Bumper.h"

// BumperInit ------------------------------------------------------------

TEST_GROUP(BumperInit)
{
  Bumper* bumper_p;

  void setup()
  {
  }

  void teardown()
  {
    if (bumper_p)
    {
      delete bumper_p;
    }
    mock().clear();
  }
};

TEST(BumperInit, defaultInit)
{
  bumper_p = new Bumper();

  UNSIGNED_LONGS_EQUAL(0, bumper_p->m_pin);
  CHECK(!bumper_p->m_hit);
  UNSIGNED_LONGS_EQUAL(0, bumper_p->m_counter);
}

TEST(BumperInit, defaultInit_setup)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT_PULLUP);

  bumper_p = new Bumper();
  bumper_p->setup(1);

  CHECK_EQUAL(1, bumper_p->m_pin);
  CHECK_EQUAL(false, bumper_p->m_hit);
  CHECK_EQUAL(0, bumper_p->m_counter);

  mock().checkExpectations();
}

TEST(BumperInit, parameterizedInit)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT_PULLUP);

  bumper_p = new Bumper(1);

  CHECK_EQUAL(1, bumper_p->m_pin);
  CHECK_EQUAL(false, bumper_p->m_hit);
  CHECK_EQUAL(0, bumper_p->m_counter);

  mock().checkExpectations();
}

// Bumper ------------------------------------------------------------

TEST_GROUP(Bumper)
{
  Bumper* bumper_p;

  void setup()
  {
    mock().disable();
    bumper_p = new Bumper(1);
    mock().enable();
  }

  void teardown()
  {
    if (bumper_p)
    {
      delete bumper_p;
    }
    mock().clear();
  }
};

TEST(Bumper, CheckOnceHigh)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(HIGH);

  bumper_p->check();
  CHECK_EQUAL(false, bumper_p->isHit());
  CHECK_EQUAL(0, bumper_p->getCounter());

  mock().checkExpectations();
}

TEST(Bumper, CheckOnceLow)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(LOW);

  bumper_p->check();
  CHECK_EQUAL(true, bumper_p->isHit());
  CHECK_EQUAL(1, bumper_p->getCounter());

  mock().checkExpectations();
}

TEST(Bumper, CheckTwiceHigh)
{
  mock().expectNCalls(2, "digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(HIGH);

  bumper_p->check();
  CHECK_EQUAL(false, bumper_p->isHit());
  CHECK_EQUAL(0, bumper_p->getCounter());

  bumper_p->check();
  CHECK_EQUAL(false, bumper_p->isHit());
  CHECK_EQUAL(0, bumper_p->getCounter());

  mock().checkExpectations();
}

TEST(Bumper, CheckTwiceLow)
{
  mock().expectNCalls(2, "digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(LOW);

  bumper_p->check();
  CHECK_EQUAL(true, bumper_p->isHit());
  CHECK_EQUAL(1, bumper_p->getCounter());

  bumper_p->check();
  CHECK_EQUAL(true, bumper_p->isHit());
  CHECK_EQUAL(2, bumper_p->getCounter());

  mock().checkExpectations();
}

TEST(Bumper, CheckTwiceSim)
{
  bumper_p->simHit();
  CHECK_EQUAL(true, bumper_p->isHit());
  CHECK_EQUAL(1, bumper_p->getCounter());

  bumper_p->simHit();
  CHECK_EQUAL(true, bumper_p->isHit());
  CHECK_EQUAL(2, bumper_p->getCounter());
}

TEST(Bumper, clearHit)
{
  bumper_p->simHit();
  bumper_p->simHit();
  CHECK_EQUAL(true, bumper_p->isHit());
  CHECK_EQUAL(2, bumper_p->getCounter());

  bumper_p->clearHit();
  CHECK_EQUAL(false, bumper_p->isHit());
  CHECK_EQUAL(2, bumper_p->getCounter());

  bumper_p->simHit();
  CHECK_EQUAL(true, bumper_p->isHit());
  CHECK_EQUAL(3, bumper_p->getCounter());

  bumper_p->clearHit();
  CHECK_EQUAL(false, bumper_p->isHit());
  CHECK_EQUAL(3, bumper_p->getCounter());
}

// BumpersInit ------------------------------------------------------------

const uint8_t BUMPERS_NUM = 2;
const uint8_t bumperPins[BUMPERS_NUM] = { 1, 2 };

TEST_GROUP(BumpersInit)
{
  Bumper bumperArray[BUMPERS_NUM];
  Bumpers* bumpers_p;

  void setup()
  {
  }

  void teardown()
  {
    if (bumpers_p)
    {
      delete bumpers_p;
    }
    mock().clear();
  }
};


TEST(BumpersInit, defaultInit)
{
  bumpers_p = new Bumpers;

  CHECK(!bumpers_p->m_use);
  BYTES_EQUAL(0, bumpers_p->m_len);
  POINTERS_EQUAL(nullptr, bumpers_p->m_bumperArray_p);
}

TEST(BumpersInit, parameterizedInit)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT_PULLUP);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 2)
      .withParameter("mode", INPUT_PULLUP);

  bumpers_p = new Bumpers(bumperPins, bumperArray, BUMPERS_NUM);

  CHECK(!bumpers_p->m_use);
  BYTES_EQUAL(BUMPERS_NUM, bumpers_p->m_len);
  CHECK(bumpers_p->m_bumperArray_p != nullptr);

  BYTES_EQUAL(1, bumpers_p->m_bumperArray_p[0].m_pin);
  CHECK(!bumpers_p->m_bumperArray_p[0].m_hit);
  UNSIGNED_LONGS_EQUAL(0, bumpers_p->m_bumperArray_p[0].m_counter);

  BYTES_EQUAL(2, bumpers_p->m_bumperArray_p[1].m_pin);
  CHECK(!bumpers_p->m_bumperArray_p[1].m_hit);
  UNSIGNED_LONGS_EQUAL(0, bumpers_p->m_bumperArray_p[1].m_counter);

  mock().checkExpectations();
}

// Bumpers ------------------------------------------------------------

TEST_GROUP(Bumpers)
{
  Bumper bumperArray[BUMPERS_NUM];
  Bumpers* bumpers_p;

  void setup()
  {
    mock().expectOneCall("pinMode")
        .withParameter("pin", 1)
        .withParameter("mode", INPUT_PULLUP);
    mock().expectOneCall("pinMode")
        .withParameter("pin", 2)
        .withParameter("mode", INPUT_PULLUP);

    bumpers_p = new Bumpers(bumperPins, bumperArray, BUMPERS_NUM);

    mock().checkExpectations();
  }

  void teardown()
  {
    if (bumpers_p)
    {
      delete bumpers_p;
    }
    mock().clear();
  }
};

TEST(Bumpers, checkNoHit)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 2)
      .andReturnValue(HIGH);

  bumpers_p->check();

  CHECK(!bumpers_p->isAnyHit());
  CHECK(!bumpers_p->m_bumperArray_p[0].isHit());
  CHECK(!bumpers_p->m_bumperArray_p[1].isHit());
  UNSIGNED_LONGS_EQUAL(0, bumpers_p->m_bumperArray_p[0].getCounter());
  UNSIGNED_LONGS_EQUAL(0, bumpers_p->m_bumperArray_p[1].getCounter());

  mock().checkExpectations();
}

TEST(Bumpers, checkOneHit)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 2)
      .andReturnValue(LOW);

  bumpers_p->check();

  CHECK(bumpers_p->isAnyHit());
  CHECK(!bumpers_p->m_bumperArray_p[0].isHit());
  CHECK(bumpers_p->m_bumperArray_p[1].isHit());
  UNSIGNED_LONGS_EQUAL(0, bumpers_p->m_bumperArray_p[0].getCounter());
  UNSIGNED_LONGS_EQUAL(1, bumpers_p->m_bumperArray_p[1].getCounter());

  mock().checkExpectations();
}

TEST(Bumpers, checkBothHit)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(LOW);
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 2)
      .andReturnValue(LOW);

  bumpers_p->check();

  CHECK(bumpers_p->isAnyHit());
  CHECK(bumpers_p->m_bumperArray_p[0].isHit());
  CHECK(bumpers_p->m_bumperArray_p[1].isHit());
  UNSIGNED_LONGS_EQUAL(1, bumpers_p->m_bumperArray_p[0].getCounter());
  UNSIGNED_LONGS_EQUAL(1, bumpers_p->m_bumperArray_p[1].getCounter());

  mock().checkExpectations();
}

TEST(Bumpers, clearHit)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(LOW);
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 2)
      .andReturnValue(LOW);

  bumpers_p->check();
  bumpers_p->clearHit();

  CHECK(!bumpers_p->isAnyHit());
  CHECK(!bumpers_p->m_bumperArray_p[0].isHit());
  CHECK(!bumpers_p->m_bumperArray_p[1].isHit());
  UNSIGNED_LONGS_EQUAL(1, bumpers_p->m_bumperArray_p[0].getCounter());
  UNSIGNED_LONGS_EQUAL(1, bumpers_p->m_bumperArray_p[1].getCounter());

  mock().checkExpectations();
}
