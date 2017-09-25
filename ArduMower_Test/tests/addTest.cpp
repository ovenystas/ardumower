#include "add.h"

#include "CppUTest/TestHarness.h"

TEST_GROUP(AddGroup)
{
};

TEST(AddGroup, AddZeroes)
{
    CHECK_EQUAL(0, add(0, 0));
}

TEST(AddGroup, AddPositives)
{
    CHECK_EQUAL(59, add(56, 3));
}

TEST(AddGroup, AddNegatives)
{
    CHECK_EQUAL(-59, add(-56, -3));
}
