#include "Bumper.h"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

Bumper bumper;

// BumperGroup ------------------------------------------------------------

TEST_GROUP(BumperGroup)
{
  void setup()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", INPUT_PULLUP);
    bumper_setup(1, &bumper);
    mock().checkExpectations();
  }

  void teardown()
  {
    mock().clear();
  }
};

TEST(BumperGroup, Setup)
{
  CHECK_EQUAL(1, bumper.pin);
  CHECK_EQUAL(false, bumper.hit);
  CHECK_EQUAL(0, bumper.counter);
}

TEST(BumperGroup, CheckOnceHigh)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  bumper_check(&bumper);
  CHECK_EQUAL(false, bumper_isHit(&bumper));
  CHECK_EQUAL(0, bumper_getCounter(&bumper));

  mock().checkExpectations();
}

TEST(BumperGroup, CheckOnceLow)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  bumper_check(&bumper);
  CHECK_EQUAL(true, bumper_isHit(&bumper));
  CHECK_EQUAL(1, bumper_getCounter(&bumper));

  mock().checkExpectations();
}

TEST(BumperGroup, CheckTwiceHigh)
{
  mock().expectNCalls(2, "digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);

  bumper_check(&bumper);
  CHECK_EQUAL(false, bumper_isHit(&bumper));
  CHECK_EQUAL(0, bumper_getCounter(&bumper));

  bumper_check(&bumper);
  CHECK_EQUAL(false, bumper_isHit(&bumper));
  CHECK_EQUAL(0, bumper_getCounter(&bumper));

  mock().checkExpectations();
}

TEST(BumperGroup, CheckTwiceLow)
{
  mock().expectNCalls(2, "digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);

  bumper_check(&bumper);
  CHECK_EQUAL(true, bumper_isHit(&bumper));
  CHECK_EQUAL(1, bumper_getCounter(&bumper));

  bumper_check(&bumper);
  CHECK_EQUAL(true, bumper_isHit(&bumper));
  CHECK_EQUAL(2, bumper_getCounter(&bumper));

  mock().checkExpectations();
}

TEST(BumperGroup, CheckTwiceSim)
{
  bumper_simHit(&bumper);
  CHECK_EQUAL(true, bumper_isHit(&bumper));
  CHECK_EQUAL(1, bumper_getCounter(&bumper));

  bumper_simHit(&bumper);
  CHECK_EQUAL(true, bumper_isHit(&bumper));
  CHECK_EQUAL(2, bumper_getCounter(&bumper));
}

TEST(BumperGroup, clearHit)
{
  bumper_simHit(&bumper);
  bumper_simHit(&bumper);
  CHECK_EQUAL(true, bumper_isHit(&bumper));
  CHECK_EQUAL(2, bumper_getCounter(&bumper));

  bumper_clearHit(&bumper);
  CHECK_EQUAL(false, bumper_isHit(&bumper));
  CHECK_EQUAL(2, bumper_getCounter(&bumper));

  bumper_simHit(&bumper);
  CHECK_EQUAL(true, bumper_isHit(&bumper));
  CHECK_EQUAL(3, bumper_getCounter(&bumper));

  bumper_clearHit(&bumper);
  CHECK_EQUAL(false, bumper_isHit(&bumper));
  CHECK_EQUAL(3, bumper_getCounter(&bumper));
}

// BumperArrayGroup ------------------------------------------------------------

const uint8_t BUMPERS_NUM = 2;
const uint8_t bumperPins[BUMPERS_NUM] = { 1, 2 };
Bumper bumperArray[BUMPERS_NUM];
Bumpers bumpers;

TEST_GROUP(BumperArrayGroup)
{
  void setup()
  {
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 1)
        .withIntParameter("mode", INPUT_PULLUP);
    mock().expectOneCall("pinMode")
        .withIntParameter("pin", 2)
        .withIntParameter("mode", INPUT_PULLUP);

    bumpers_setup(bumperPins, bumperArray, &bumpers, BUMPERS_NUM);

    mock().checkExpectations();
  }

  void teardown()
  {
    mock().clear();
  }
};


TEST(BumperArrayGroup, setup)
{
  CHECK_EQUAL(false, bumpers.use);
  CHECK_EQUAL(BUMPERS_NUM, bumpers.len);

  CHECK_EQUAL(1, bumpers.bumperArray_p[0].pin);
  CHECK_EQUAL(false, bumpers.bumperArray_p[0].hit);
  CHECK_EQUAL(0, bumpers.bumperArray_p[0].counter);

  CHECK_EQUAL(2, bumpers.bumperArray_p[1].pin);
  CHECK_EQUAL(false, bumpers.bumperArray_p[1].hit);
  CHECK_EQUAL(0, bumpers.bumperArray_p[1].counter);
}

TEST(BumperArrayGroup, checkNoHit)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 2)
      .andReturnValue(HIGH);

  bumpers_check(&bumpers);

  CHECK_EQUAL(false, bumpers_isAnyHit(&bumpers));
  CHECK_EQUAL(false, bumper_isHit(&bumpers.bumperArray_p[0]));
  CHECK_EQUAL(false, bumper_isHit(&bumpers.bumperArray_p[1]));
  CHECK_EQUAL(0, bumper_getCounter(&bumpers.bumperArray_p[0]));
  CHECK_EQUAL(0, bumper_getCounter(&bumpers.bumperArray_p[1]));

  mock().checkExpectations();
}

TEST(BumperArrayGroup, checkOneHit)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(HIGH);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 2)
      .andReturnValue(LOW);

  bumpers_check(&bumpers);

  CHECK_EQUAL(true, bumpers_isAnyHit(&bumpers));
  CHECK_EQUAL(false, bumper_isHit(&bumpers.bumperArray_p[0]));
  CHECK_EQUAL(true, bumper_isHit(&bumpers.bumperArray_p[1]));
  CHECK_EQUAL(0, bumper_getCounter(&bumpers.bumperArray_p[0]));
  CHECK_EQUAL(1, bumper_getCounter(&bumpers.bumperArray_p[1]));

  mock().checkExpectations();
}

TEST(BumperArrayGroup, checkBothHit)
{
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 1)
      .andReturnValue(LOW);
  mock().expectOneCall("digitalRead")
      .withIntParameter("pin", 2)
      .andReturnValue(LOW);

  bumpers_check(&bumpers);

  CHECK_EQUAL(true, bumpers_isAnyHit(&bumpers));
  CHECK_EQUAL(true, bumper_isHit(&bumpers.bumperArray_p[0]));
  CHECK_EQUAL(true, bumper_isHit(&bumpers.bumperArray_p[1]));
  CHECK_EQUAL(1, bumper_getCounter(&bumpers.bumperArray_p[0]));
  CHECK_EQUAL(1, bumper_getCounter(&bumpers.bumperArray_p[1]));

  mock().checkExpectations();
}
