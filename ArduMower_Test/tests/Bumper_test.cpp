#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Bumper.h"

// BumperInit ------------------------------------------------------------

TEST_GROUP(BumperInit)
{
  Bumper* p_bumper;

  void setup()
  {
  }

  void teardown()
  {
    if (p_bumper)
    {
      delete p_bumper;
    }
    mock().clear();
  }
};

TEST(BumperInit, defaultInit)
{
  p_bumper = new Bumper();

  UNSIGNED_LONGS_EQUAL(0, p_bumper->m_pin);
  CHECK(!p_bumper->m_hit);
  UNSIGNED_LONGS_EQUAL(0, p_bumper->m_counter);
}

TEST(BumperInit, defaultInit_setup)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT_PULLUP);

  p_bumper = new Bumper();
  p_bumper->setup(1);

  CHECK_EQUAL(1, p_bumper->m_pin);
  CHECK_EQUAL(false, p_bumper->m_hit);
  CHECK_EQUAL(0, p_bumper->m_counter);

  mock().checkExpectations();
}

TEST(BumperInit, parameterizedInit)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT_PULLUP);

  p_bumper = new Bumper(1);

  CHECK_EQUAL(1, p_bumper->m_pin);
  CHECK_EQUAL(false, p_bumper->m_hit);
  CHECK_EQUAL(0, p_bumper->m_counter);

  mock().checkExpectations();
}

// Bumper ------------------------------------------------------------

TEST_GROUP(Bumper)
{
  Bumper* p_bumper;

  void setup()
  {
    mock().expectOneCall("pinMode")
        .withParameter("pin", 1)
        .withParameter("mode", INPUT_PULLUP);

    p_bumper = new Bumper(1);

    mock().checkExpectations();
  }

  void teardown()
  {
    if (p_bumper)
    {
      delete p_bumper;
    }
    mock().clear();
  }
};

TEST(Bumper, CheckOnceHigh)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(HIGH);

  p_bumper->check();
  CHECK_EQUAL(false, p_bumper->isHit());
  CHECK_EQUAL(0, p_bumper->getCounter());

  mock().checkExpectations();
}

TEST(Bumper, CheckOnceLow)
{
  mock().expectOneCall("digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(LOW);

  p_bumper->check();
  CHECK_EQUAL(true, p_bumper->isHit());
  CHECK_EQUAL(1, p_bumper->getCounter());

  mock().checkExpectations();
}

TEST(Bumper, CheckTwiceHigh)
{
  mock().expectNCalls(2, "digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(HIGH);

  p_bumper->check();
  CHECK_EQUAL(false, p_bumper->isHit());
  CHECK_EQUAL(0, p_bumper->getCounter());

  p_bumper->check();
  CHECK_EQUAL(false, p_bumper->isHit());
  CHECK_EQUAL(0, p_bumper->getCounter());

  mock().checkExpectations();
}

TEST(Bumper, CheckTwiceLow)
{
  mock().expectNCalls(2, "digitalRead")
      .withParameter("pin", 1)
      .andReturnValue(LOW);

  p_bumper->check();
  CHECK_EQUAL(true, p_bumper->isHit());
  CHECK_EQUAL(1, p_bumper->getCounter());

  p_bumper->check();
  CHECK_EQUAL(true, p_bumper->isHit());
  CHECK_EQUAL(2, p_bumper->getCounter());

  mock().checkExpectations();
}

TEST(Bumper, CheckTwiceSim)
{
  p_bumper->simHit();
  CHECK_EQUAL(true, p_bumper->isHit());
  CHECK_EQUAL(1, p_bumper->getCounter());

  p_bumper->simHit();
  CHECK_EQUAL(true, p_bumper->isHit());
  CHECK_EQUAL(2, p_bumper->getCounter());
}

TEST(Bumper, clearHit)
{
  p_bumper->simHit();
  p_bumper->simHit();
  CHECK_EQUAL(true, p_bumper->isHit());
  CHECK_EQUAL(2, p_bumper->getCounter());

  p_bumper->clearHit();
  CHECK_EQUAL(false, p_bumper->isHit());
  CHECK_EQUAL(2, p_bumper->getCounter());

  p_bumper->simHit();
  CHECK_EQUAL(true, p_bumper->isHit());
  CHECK_EQUAL(3, p_bumper->getCounter());

  p_bumper->clearHit();
  CHECK_EQUAL(false, p_bumper->isHit());
  CHECK_EQUAL(3, p_bumper->getCounter());
}

// BumpersInit ------------------------------------------------------------

const uint8_t BUMPERS_NUM = 2;
const uint8_t bumperPins[BUMPERS_NUM] = { 1, 2 };

TEST_GROUP(BumpersInit)
{
  Bumper bumperArray[BUMPERS_NUM];
  Bumpers* p_bumpers;

  void setup()
  {
  }

  void teardown()
  {
    if (p_bumpers)
    {
      delete p_bumpers;
    }
    mock().clear();
  }
};


TEST(BumpersInit, defaultInit)
{
  p_bumpers = new Bumpers;

  CHECK(!p_bumpers->m_use);
  BYTES_EQUAL(0, p_bumpers->m_len);
  POINTERS_EQUAL(nullptr, p_bumpers->m_bumperArray_p);
}

TEST(BumpersInit, defaultInit_setup)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT_PULLUP);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 2)
      .withParameter("mode", INPUT_PULLUP);

  p_bumpers = new Bumpers;
  p_bumpers->setup(bumperPins, bumperArray, BUMPERS_NUM);

  CHECK(!p_bumpers->m_use);
  BYTES_EQUAL(BUMPERS_NUM, p_bumpers->m_len);
  CHECK(p_bumpers->m_bumperArray_p != nullptr);

  BYTES_EQUAL(1, p_bumpers->m_bumperArray_p[0].m_pin);
  CHECK(!p_bumpers->m_bumperArray_p[0].m_hit);
  UNSIGNED_LONGS_EQUAL(0, p_bumpers->m_bumperArray_p[0].m_counter);

  BYTES_EQUAL(2, p_bumpers->m_bumperArray_p[1].m_pin);
  CHECK(!p_bumpers->m_bumperArray_p[1].m_hit);
  UNSIGNED_LONGS_EQUAL(0, p_bumpers->m_bumperArray_p[1].m_counter);

  mock().checkExpectations();
}

TEST(BumpersInit, parameterizedInit)
{
  mock().expectOneCall("pinMode")
      .withParameter("pin", 1)
      .withParameter("mode", INPUT_PULLUP);
  mock().expectOneCall("pinMode")
      .withParameter("pin", 2)
      .withParameter("mode", INPUT_PULLUP);

  p_bumpers = new Bumpers(bumperPins, bumperArray, BUMPERS_NUM);

  CHECK(!p_bumpers->m_use);
  BYTES_EQUAL(BUMPERS_NUM, p_bumpers->m_len);
  CHECK(p_bumpers->m_bumperArray_p != nullptr);

  BYTES_EQUAL(1, p_bumpers->m_bumperArray_p[0].m_pin);
  CHECK(!p_bumpers->m_bumperArray_p[0].m_hit);
  UNSIGNED_LONGS_EQUAL(0, p_bumpers->m_bumperArray_p[0].m_counter);

  BYTES_EQUAL(2, p_bumpers->m_bumperArray_p[1].m_pin);
  CHECK(!p_bumpers->m_bumperArray_p[1].m_hit);
  UNSIGNED_LONGS_EQUAL(0, p_bumpers->m_bumperArray_p[1].m_counter);

  mock().checkExpectations();
}

// Bumpers ------------------------------------------------------------

TEST_GROUP(Bumpers)
{
  Bumper bumperArray[BUMPERS_NUM];
  Bumpers* p_bumpers;

  void setup()
  {
    mock().expectOneCall("pinMode")
        .withParameter("pin", 1)
        .withParameter("mode", INPUT_PULLUP);
    mock().expectOneCall("pinMode")
        .withParameter("pin", 2)
        .withParameter("mode", INPUT_PULLUP);

    p_bumpers = new Bumpers(bumperPins, bumperArray, BUMPERS_NUM);

    mock().checkExpectations();
  }

  void teardown()
  {
    if (p_bumpers)
    {
      delete p_bumpers;
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

  p_bumpers->check();

  CHECK(!p_bumpers->isAnyHit());
  CHECK(!p_bumpers->m_bumperArray_p[0].isHit());
  CHECK(!p_bumpers->m_bumperArray_p[1].isHit());
  UNSIGNED_LONGS_EQUAL(0, p_bumpers->m_bumperArray_p[0].getCounter());
  UNSIGNED_LONGS_EQUAL(0, p_bumpers->m_bumperArray_p[1].getCounter());

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

  p_bumpers->check();

  CHECK(p_bumpers->isAnyHit());
  CHECK(!p_bumpers->m_bumperArray_p[0].isHit());
  CHECK(p_bumpers->m_bumperArray_p[1].isHit());
  UNSIGNED_LONGS_EQUAL(0, p_bumpers->m_bumperArray_p[0].getCounter());
  UNSIGNED_LONGS_EQUAL(1, p_bumpers->m_bumperArray_p[1].getCounter());

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

  p_bumpers->check();

  CHECK(p_bumpers->isAnyHit());
  CHECK(p_bumpers->m_bumperArray_p[0].isHit());
  CHECK(p_bumpers->m_bumperArray_p[1].isHit());
  UNSIGNED_LONGS_EQUAL(1, p_bumpers->m_bumperArray_p[0].getCounter());
  UNSIGNED_LONGS_EQUAL(1, p_bumpers->m_bumperArray_p[1].getCounter());

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

  p_bumpers->check();
  p_bumpers->clearHit();

  CHECK(!p_bumpers->isAnyHit());
  CHECK(!p_bumpers->m_bumperArray_p[0].isHit());
  CHECK(!p_bumpers->m_bumperArray_p[1].isHit());
  UNSIGNED_LONGS_EQUAL(1, p_bumpers->m_bumperArray_p[0].getCounter());
  UNSIGNED_LONGS_EQUAL(1, p_bumpers->m_bumperArray_p[1].getCounter());

  mock().checkExpectations();
}
