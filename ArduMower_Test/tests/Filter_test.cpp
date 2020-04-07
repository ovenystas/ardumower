#include "CppUTest/TestHarness.h"

// Get access to private class members in SUT
#define private public
#include "Filter.h"

TEST_GROUP(FilterGroup32)
{
  FilterEmaI32* filter_p;

  void setup()
  {
  }

  void teardown()
  {
    if (filter_p)
    {
      delete filter_p;
    }
  }
};

TEST(FilterGroup32, setGetAlpha)
{
  filter_p = new FilterEmaI32(0.0);
  DOUBLES_EQUAL(0.0, filter_p->getAlpha(), 0.00001);

  filter_p->setAlpha(1.0);
  DOUBLES_EQUAL(1.0, filter_p->getAlpha(), 0.00001);

  filter_p->setAlpha(0.5);
  DOUBLES_EQUAL(0.5, filter_p->getAlpha(), 0.00001);
}

TEST(FilterGroup32, Alpha32_00)
{
  filter_p = new FilterEmaI32(0.0);

  UNSIGNED_LONGS_EQUAL(0, filter_p->m_alpha);
  LONGS_EQUAL(0, filter_p->m_average);

  filter_p->addValue(0);
  LONGS_EQUAL(0, filter_p->getAverage());

  filter_p->addValue(10000);
  LONGS_EQUAL(0, filter_p->getAverage());

  filter_p->addValue(100000);
  LONGS_EQUAL(0, filter_p->getAverage());

  filter_p->addValue(-200000);
  LONGS_EQUAL(0, filter_p->getAverage());

}

TEST(FilterGroup32, Alpha32_10)
{
  filter_p = new FilterEmaI32(1.0);

  UNSIGNED_LONGS_EQUAL(65535, filter_p->m_alpha);
  LONGS_EQUAL(0, filter_p->m_average);

  filter_p->addValue(0);
  LONGS_EQUAL(0, filter_p->getAverage());

  filter_p->addValue(10000);
  LONGS_EQUAL(10000, filter_p->getAverage());

  filter_p->addValue(100000);
  LONGS_EQUAL(100000, filter_p->getAverage());

  filter_p->addValue(-200000);
  LONGS_EQUAL(-200000, filter_p->getAverage());
}

TEST(FilterGroup32, Alpha32_05)
{
  filter_p = new FilterEmaI32(0.5);

  UNSIGNED_LONGS_EQUAL(32767, filter_p->m_alpha);
  LONGS_EQUAL(0, filter_p->m_average);

  filter_p->addValue(0);
  LONGS_EQUAL(0, filter_p->getAverage());

  filter_p->addValue(32767);
  LONGS_EQUAL(16383, filter_p->getAverage());

  filter_p->addValue(65535);
  LONGS_EQUAL(40959, filter_p->getAverage());
}

TEST_GROUP(FilterGroup16)
{
  FilterEmaI16* filter_p;

  void setup()
  {
  }

  void teardown()
  {
    if (filter_p)
    {
      delete filter_p;
    }
  }
};

TEST(FilterGroup16, setGetAlpha)
{
  filter_p = new FilterEmaI16(0.0);
  DOUBLES_EQUAL(0.0, filter_p->getAlpha(), 0.002);

  filter_p->setAlpha(1.0);
  DOUBLES_EQUAL(1.0, filter_p->getAlpha(), 0.002);

  filter_p->setAlpha(0.5);
  DOUBLES_EQUAL(0.5, filter_p->getAlpha(), 0.002);
}

TEST(FilterGroup16, Alpha16_00)
{
  filter_p = new FilterEmaI16(0.0);

  UNSIGNED_LONGS_EQUAL(0, filter_p->m_alpha);
  LONGS_EQUAL(0, filter_p->m_average);

  filter_p->addValue(0);
  LONGS_EQUAL(0, filter_p->getAverage());

  filter_p->addValue(127);
  LONGS_EQUAL(0, filter_p->getAverage());

  filter_p->addValue(255);
  LONGS_EQUAL(0, filter_p->getAverage());
}

TEST(FilterGroup16, Alpha16_10)
{
  filter_p = new FilterEmaI16(1.0);

  UNSIGNED_LONGS_EQUAL(255, filter_p->m_alpha);
  LONGS_EQUAL(0, filter_p->m_average);

  filter_p->addValue(0);
  LONGS_EQUAL(0, filter_p->getAverage());

  filter_p->addValue(100);
  LONGS_EQUAL(100, filter_p->getAverage());

  filter_p->addValue(10000);
  LONGS_EQUAL(10000, filter_p->getAverage());

  filter_p->addValue(-20000);
  LONGS_EQUAL(-20000, filter_p->getAverage());
}

TEST(FilterGroup16, Alpha16_05)
{
  filter_p = new FilterEmaI16(0.5);

  UNSIGNED_LONGS_EQUAL(127, filter_p->m_alpha);
  LONGS_EQUAL(0, filter_p->m_average);

  filter_p->addValue(0);
  LONGS_EQUAL(0, filter_p->getAverage());

  filter_p->addValue(100);
  LONGS_EQUAL(50, filter_p->getAverage());

  filter_p->addValue(100);
  LONGS_EQUAL(75, filter_p->getAverage());

  filter_p->addValue(100);
  LONGS_EQUAL(87, filter_p->getAverage());

  filter_p->addValue(10000);
  LONGS_EQUAL(5024, filter_p->getAverage());

  filter_p->addValue(-20000);
  LONGS_EQUAL(-7439, filter_p->getAverage());
}
