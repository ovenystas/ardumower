#include "CppUTest/TestHarness.h"
//#include "CppUTestExt/MockSupport.h"

// Get access to private class members in SUT
#define private public
#include "Vector.h"

// Vector ------------------------------------------------------------

TEST_GROUP(Vector)
{
};

TEST(Vector, int_defaultConstructor)
{
  Vector<int> v;

  LONGS_EQUAL(0, v.x);
  LONGS_EQUAL(0, v.y);
  LONGS_EQUAL(0, v.z);
}

TEST(Vector, int_singleValueConstructor)
{
  Vector<int> v(-123);

  LONGS_EQUAL(-123, v.x);
  LONGS_EQUAL(-123, v.y);
  LONGS_EQUAL(-123, v.z);
}

TEST(Vector, int_multiValueConstructor)
{
  Vector<int> v(-12, 34, -56);

  LONGS_EQUAL(-12, v.x);
  LONGS_EQUAL( 34, v.y);
  LONGS_EQUAL(-56, v.z);
}

TEST(Vector, int_copyConstructor)
{
  const Vector<int> v_orig(-78, 90, -12);
  Vector<int> v(v_orig);

  LONGS_EQUAL(-78, v.x);
  LONGS_EQUAL( 90, v.y);
  LONGS_EQUAL(-12, v.z);
}

TEST(Vector, int_isZero)
{
  Vector<int> v;

  CHECK_TRUE(v.isZero());
}

TEST(Vector, float_isZero)
{
  Vector<float> v;

  CHECK_TRUE(v.isZero());
}

TEST(Vector, int_isNotZeroX)
{
  Vector<int> v(1, 0, 0);

  CHECK_FALSE(v.isZero());
}

TEST(Vector, int_isNotZeroY)
{
  Vector<int> v(0, 1, 0);

  CHECK_FALSE(v.isZero());
}

TEST(Vector, int_isNotZeroZ)
{
  Vector<int> v(0, 0, 1);

  CHECK_FALSE(v.isZero());
}

TEST(Vector, float_isNotZero)
{
  Vector<float> v(0.0f, 0.1f, 0.0f);

  CHECK_FALSE(v.isZero());
}

TEST(Vector, int_isEqual1)
{
  Vector<int> v1;
  const Vector<int> v2;

  CHECK_TRUE(v1 == v2);
}

TEST(Vector, int_isEqual2)
{
  Vector<int> v1(-1, 0, 1);
  const Vector<int> v2(-1, 0, 1);

  CHECK_TRUE(v1 == v2);
}

TEST(Vector, int_isNotEqualX)
{
  Vector<int> v1(1, 2, 3);
  const Vector<int> v2(-1, 2, 3);

  CHECK_FALSE(v1 == v2);
}

TEST(Vector, int_isNotEqualY)
{
  Vector<int> v1(1, 2, 3);
  const Vector<int> v2(1, -2, 3);

  CHECK_FALSE(v1 == v2);
}

TEST(Vector, int_isNotEqualZ)
{
  Vector<int> v1(1, 2, 3);
  const Vector<int> v2(1, 2, -3);

  CHECK_FALSE(v1 == v2);
}

TEST(Vector, float_isUnequal)
{
  Vector<float> v1(0.0f, 0.1f, 0.0f);
  const Vector<float> v2(0.1f, 0.0f, 0.0f);

  CHECK_TRUE(v1 != v2);
}

TEST(Vector, int_assignment)
{
  Vector<int> v1;
  const Vector<int> v2(-1, 2, 1);

  v1 = v2;

  CHECK_TRUE(v1 == v2);
}

TEST(Vector, int_assignmentToItself)
{
  Vector<int> v(-1, 2, 1);
  const Vector<int> expected(-1, 2, 1);

  v = v;

  CHECK_TRUE(v == expected);
}

TEST(Vector, int_addition)
{
  const Vector<int> v1(-1, 0, 1);
  const Vector<int> v2(3, -2, 1);
  const Vector<int> expected(2, -2, 2);

  CHECK_TRUE(v1 + v2 == expected);
}

TEST(Vector, int_subtraction)
{
  const Vector<int> v1(-1, 0, 1);
  const Vector<int> v2(3, -2, 1);
  const Vector<int> expected(-4, 2, 0);

  CHECK_TRUE(v1 - v2 == expected);
}

TEST(Vector, int_multiplicationByValueAssignment)
{
  Vector<int> v1(3, -2, 1);
  const Vector<int> expected(12, -8, 4);

  v1 *= 4;

  CHECK_TRUE(v1 == expected);
}

TEST(Vector, int_divisionByValueAssignment)
{
  Vector<int> v1(24, -4, 12);
  const Vector<int> expected(6, -1, 3);

  v1 /= 4;

  CHECK_TRUE(v1 == expected);
}

TEST(Vector, int_negation)
{
  const Vector<int> v1(5, -6, 12);
  const Vector<int> expected(-5, 6, -12);

  CHECK_TRUE(-v1 == expected);
}

TEST(Vector, int_additionAssignment)
{
  Vector<int> v1(5, -6, 12);
  const Vector<int> v2(3, -2, 1);
  const Vector<int> expected(8, -8, 13);

  v1 += v2;

  CHECK_TRUE(v1 == expected);
}

TEST(Vector, int_subtractionAssignment)
{
  Vector<int> v1(5, -6, 12);
  const Vector<int> v2(3, -2, 1);
  const Vector<int> expected(2, -4, 11);

  v1 -= v2;

  CHECK_TRUE(v1 == expected);
}

TEST(Vector, int_multiplicationByValue)
{
  Vector<int> v1(5, -6, 12);
  const Vector<int> expected(10, -12, 24);

  CHECK_TRUE(v1 * 2 == expected);
}

TEST(Vector, int_divisionByValue)
{
  Vector<int> v1(4, -6, 12);
  const Vector<int> expected(2, -3, 6);

  CHECK_TRUE(v1 / 2 == expected);
}

TEST(Vector, int_castToFloat)
{
  const Vector<int> v(1, 2, 3);
  const Vector<float> expected(1, 2, 3);

  Vector<float> res = static_cast<Vector<float>>(v);

  CHECK_TRUE(res == expected);
}

TEST(Vector, int_cross)
{
  Vector<int> v1(1, 2, 3);
  const Vector<int> v2(4, 5, 6);
  const Vector<int> expected(-3, 6, -3);

  Vector<int> res = Vector<int>::cross(v1, v2);

  CHECK_TRUE(res == expected);
}

TEST(Vector, dot_int_int)
{
  const Vector<int> v1(1, 2, 3);
  const Vector<int> v2(4, 5, 6);
  const int expected = 32;

  int res = Vector<int>::dot(v1, v2);

  CHECK_TRUE(res == expected);
}

TEST(Vector, dot_int_float)
{
  const Vector<int> v1(1, 2, 3);
  const Vector<float> v2(0.4f, 0.5f, 0.6f);
  const float expected = 3.2f;

  float res = Vector<float>::dot(v1, v2);

  CHECK_TRUE(res == expected);
}

TEST(Vector, dot_float_int)
{
  const Vector<float> v1(0.1f, 0.2f, 0.3f);
  const Vector<int> v2(4, 5, 6);
  const float expected = 3.2f;

  float res = Vector<float>::dot(v1, v2);

  CHECK_TRUE(res == expected);
}

TEST(Vector, dot_float_float)
{
  const Vector<float> v1(0.1f, 0.2f, 0.3f);
  const Vector<float> v2(0.4f, 0.5f, 0.6f);
  const float expected = 0.32f;

  float res = Vector<float>::dot(v1, v2);

  CHECK_TRUE(res == expected);
}

TEST(Vector, int_normalize)
{
  Vector<float> v(4.0f, 5.0f, 6.0f);
  const Vector<float> expected(0.455842316f, 0.56980288f, 0.683763444f);

  Vector<float>::normalize(v);

  CHECK_TRUE(v == expected);
}

TEST(Vector, castBeforeAddition)
{
  const Vector<int16_t> v1(32000, -32000, 0);
  const Vector<int16_t> v2(32000, -32000, 0);

  Vector<int32_t> res =
      static_cast<Vector<int32_t>>(v1) + static_cast<Vector<int32_t>>(v2);

  LONGS_EQUAL(64000, res.x);
  LONGS_EQUAL(-64000, res.y);
  LONGS_EQUAL(0, res.z);
}

TEST(Vector, elementWiseMultiplication)
{
  const Vector<int> v1(1, 2, 3);
  const Vector<int> v2(4, 5, 6);

  Vector<int> res = v1 * v2;

  LONGS_EQUAL(4, res.x);
  LONGS_EQUAL(10, res.y);
  LONGS_EQUAL(18, res.z);
}

TEST(Vector, elementWiseDivision)
{
  const Vector<int> v1(12, 10, 24);
  const Vector<int> v2(4, 5, 6);

  Vector<int> res = v1 / v2;

  LONGS_EQUAL(3, res.x);
  LONGS_EQUAL(2, res.y);
  LONGS_EQUAL(4, res.z);
}
