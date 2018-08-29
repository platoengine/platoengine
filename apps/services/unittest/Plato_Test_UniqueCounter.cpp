/*
 * Plato_Test_UniqueCounter.cpp
 */

#include "gtest/gtest.h"

#include "Plato_UniqueCounter.hpp"

namespace Plato
{

TEST(UniqueCounter,simple)
{
    // make counter
    UniqueCounter counter;

    // mark some values, expect new
    EXPECT_EQ(0, counter.mark(1u));
    EXPECT_EQ(0, counter.mark(3u));
    EXPECT_EQ(0, counter.mark(5u));
    // expect remember
    EXPECT_EQ(1, counter.mark(1u));

    // get some unique values
    EXPECT_EQ(0u, counter.assign_next_unique());
    EXPECT_EQ(2u, counter.assign_next_unique());
    EXPECT_EQ(4u, counter.assign_next_unique());
    EXPECT_EQ(6u, counter.assign_next_unique());
    EXPECT_EQ(7u, counter.assign_next_unique());

    // (re)-mark already assigned index
    EXPECT_EQ(1, counter.mark(6u));
    // mark, expect new
    EXPECT_EQ(0, counter.mark(256u));
}

}
