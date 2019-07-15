/*
 * Plato_Test_UniqueCounter.cpp
 */

#include "gtest/gtest.h"

#include "Plato_UniqueCounter.hpp"

namespace Plato
{

TEST(UniqueCounter,simple)
{
    // make tCounter
    Plato::UniqueCounter tCounter;

    // mark some values, expect new
    ASSERT_EQ(0, tCounter.mark(1u));
    ASSERT_EQ(0, tCounter.mark(3u));
    ASSERT_EQ(0, tCounter.mark(5u));
    // expect remember
    ASSERT_EQ(1, tCounter.mark(1u));

    // get some unique values
    ASSERT_EQ(0u, tCounter.assignNextUnique());
    ASSERT_EQ(2u, tCounter.assignNextUnique());
    ASSERT_EQ(4u, tCounter.assignNextUnique());
    ASSERT_EQ(6u, tCounter.assignNextUnique());
    ASSERT_EQ(7u, tCounter.assignNextUnique());

    // (re)-mark already assigned index
    ASSERT_EQ(1, tCounter.mark(6u));
    // mark, expect new
    ASSERT_EQ(0, tCounter.mark(256u));

    std::vector<size_t> tAssignIndices = tCounter.list();
    std::vector<size_t> tGold = {0, 1, 2, 3, 4, 5, 6, 7, 256};
    ASSERT_EQ(tGold.size(), tAssignIndices.size());

    for(size_t tIndex = 0; tIndex < tGold.size(); tIndex++)
    {
        ASSERT_EQ(tGold[tIndex], tAssignIndices[tIndex]);
    }
}

}
