#include "Filter.h"

#include "CppUTest/TestHarness.h"

TEST_GROUP(FilterGroup32)
{
};

TEST(FilterGroup32, Alpha32_00)
{
  FilterEmaI32 filter;

  FilterEmaI32_init(0.0, &filter);
  CHECK_EQUAL(0, filter.alpha);
  CHECK_EQUAL(0, filter.average);

  FilterEmaI32_addValue(0, &filter);
  CHECK_EQUAL(0, FilterEmaI32_getAverage(&filter));

  FilterEmaI32_addValue(10000, &filter);
  CHECK_EQUAL(0, FilterEmaI32_getAverage(&filter));

  FilterEmaI32_addValue(100000, &filter);
  CHECK_EQUAL(0, FilterEmaI32_getAverage(&filter));

  FilterEmaI32_addValue(-200000, &filter);
  CHECK_EQUAL(0, FilterEmaI32_getAverage(&filter));

}

TEST(FilterGroup32, Alpha32_10)
{
  FilterEmaI32 filter;

  FilterEmaI32_init(1.0, &filter);
  CHECK_EQUAL(65535, filter.alpha);
  CHECK_EQUAL(0, filter.average);

  FilterEmaI32_addValue(0, &filter);
  CHECK_EQUAL(0, FilterEmaI32_getAverage(&filter));

  FilterEmaI32_addValue(10000, &filter);
  CHECK_EQUAL(10000, FilterEmaI32_getAverage(&filter));

  FilterEmaI32_addValue(100000, &filter);
  CHECK_EQUAL(100000, FilterEmaI32_getAverage(&filter));

  FilterEmaI32_addValue(-200000, &filter);
  CHECK_EQUAL(-200000, FilterEmaI32_getAverage(&filter));
}

TEST(FilterGroup32, Alpha32_05)
{
  FilterEmaI32 filter;
  FilterEmaI32_init(0.5, &filter);
  CHECK_EQUAL(32767, filter.alpha);
  CHECK_EQUAL(0, filter.average);

  FilterEmaI32_addValue(0, &filter);
  CHECK_EQUAL(0, FilterEmaI32_getAverage(&filter));

  FilterEmaI32_addValue(32767, &filter);
  CHECK_EQUAL(16383, FilterEmaI32_getAverage(&filter));

  FilterEmaI32_addValue(65535, &filter);
  CHECK_EQUAL(40959, FilterEmaI32_getAverage(&filter));
}

TEST_GROUP(FilterGroup16)
{
};

TEST(FilterGroup16, Alpha16_00)
{
  FilterEmaI16 filter;

  FilterEmaI16_init(0.0, &filter);
  CHECK_EQUAL(0, filter.alpha);
  CHECK_EQUAL(0, filter.average);

  FilterEmaI16_addValue(0, &filter);
  CHECK_EQUAL(0, FilterEmaI16_getAverage(&filter));

  FilterEmaI16_addValue(127, &filter);
  CHECK_EQUAL(0, FilterEmaI16_getAverage(&filter));

  FilterEmaI16_addValue(255, &filter);
  CHECK_EQUAL(0, FilterEmaI16_getAverage(&filter));
}

TEST(FilterGroup16, Alpha16_10)
{
  FilterEmaI16 filter;

  FilterEmaI16_init(1.0, &filter);
  CHECK_EQUAL(255, filter.alpha);
  CHECK_EQUAL(0, filter.average);

  FilterEmaI16_addValue(0, &filter);
  CHECK_EQUAL(0, FilterEmaI16_getAverage(&filter));

  FilterEmaI16_addValue(100, &filter);
  CHECK_EQUAL(100, FilterEmaI16_getAverage(&filter));

  FilterEmaI16_addValue(10000, &filter);
  CHECK_EQUAL(10000, FilterEmaI16_getAverage(&filter));

  FilterEmaI16_addValue(-20000, &filter);
  CHECK_EQUAL(-20000, FilterEmaI16_getAverage(&filter));
}

TEST(FilterGroup16, Alpha16_05)
{
  FilterEmaI16 filter;

  FilterEmaI16_init(0.5, &filter);
  CHECK_EQUAL(127, filter.alpha);
  CHECK_EQUAL(0, filter.average);

  FilterEmaI16_addValue(0, &filter);
  CHECK_EQUAL(0, FilterEmaI16_getAverage(&filter));

  FilterEmaI16_addValue(100, &filter);
  CHECK_EQUAL(50, FilterEmaI16_getAverage(&filter));

  FilterEmaI16_addValue(100, &filter);
  CHECK_EQUAL(75, FilterEmaI16_getAverage(&filter));

  FilterEmaI16_addValue(100, &filter);
  CHECK_EQUAL(87, FilterEmaI16_getAverage(&filter));

  FilterEmaI16_addValue(10000, &filter);
  CHECK_EQUAL(5024, FilterEmaI16_getAverage(&filter));

  FilterEmaI16_addValue(-20000, &filter);
  CHECK_EQUAL(-7439, FilterEmaI16_getAverage(&filter));
}

TEST(FilterGroup32, Alpha32_x1)
{
  FilterEmaI32 filter =
  { 0, 0 };

  FilterEmaI32_addValue(INT16_MAX, &filter);
  CHECK_EQUAL(0, FilterEmaI32_getAverage(&filter));
}

TEST(FilterGroup32, Alpha32_x2)
{
  FilterEmaI32 filter =
  { 0, 32767 };

  FilterEmaI32_addValue(1000000, &filter);
  CHECK_EQUAL(499992, FilterEmaI32_getAverage(&filter));
}

