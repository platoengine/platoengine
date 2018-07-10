/*
 * Plato_DataFactoryTest.cpp
 *
 *  Created on: Oct 21, 2017
 */

#include "gtest/gtest.h"

#include "Plato_DataFactory.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"

namespace PlatoTest
{

TEST(PlatoTest, DualDataFactory)
{
    Plato::DataFactory<double, size_t> tFactoryOne;

    // Test Factories for Dual Data
    size_t tNumVectors = 2;
    size_t tNumElements = 8;
    tFactoryOne.allocateDual(tNumElements, tNumVectors);

    size_t tGoldNumVectors = 2;
    EXPECT_EQ(tFactoryOne.dual().getNumVectors(), tGoldNumVectors);

    size_t tGoldNumElements = 8;
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryOne.dual(tIndex).size(), tGoldNumElements);
    }

    Plato::StandardVectorReductionOperations<double,size_t> tInterface;
    tFactoryOne.allocateDualReductionOperations(tInterface);

    std::vector<double> tStandardVector = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double,size_t> tPlatoVector(tStandardVector);
    double tSum = tFactoryOne.getDualReductionOperations().sum(tPlatoVector);
    double tGold = 55;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tSum, tGold, tTolerance);

    // Test Second Factory for Dual Data
    Plato::DataFactory<double,size_t> tFactoryTwo;
    Plato::StandardMultiVector<double,size_t> tMultiVector(tNumVectors, tPlatoVector);
    tFactoryTwo.allocateDual(tMultiVector);

    tGoldNumElements = 10;
    EXPECT_EQ(tFactoryTwo.dual().getNumVectors(), tGoldNumVectors);
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryTwo.dual(tIndex).size(), tGoldNumElements);
    }

    // Test Third Factory for Dual Data
    Plato::DataFactory<double,size_t> tFactoryThree;
    tFactoryThree.allocateDual(tPlatoVector, tNumVectors);

    EXPECT_EQ(tFactoryThree.dual().getNumVectors(), tGoldNumVectors);
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryThree.dual(tIndex).size(), tGoldNumElements);
    }

    // Test Fourth Factory for Dual Data (Default NumVectors = 1)
    Plato::DataFactory<double,size_t> tFactoryFour;
    tFactoryFour.allocateDual(tPlatoVector);

    tNumVectors = 1;
    tGoldNumVectors = 1;
    EXPECT_EQ(tFactoryFour.dual().getNumVectors(), tGoldNumVectors);
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryFour.dual(tIndex).size(), tGoldNumElements);
    }
}

TEST(PlatoTest, StateDataFactory)
{
    Plato::DataFactory<double,size_t> tFactoryOne;

    // Test Factories for State Data
    size_t tNumVectors = 2;
    size_t tNumElements = 8;
    tFactoryOne.allocateState(tNumElements, tNumVectors);

    size_t tGoldNumVectors = 2;
    EXPECT_EQ(tFactoryOne.state().getNumVectors(), tGoldNumVectors);

    size_t tGoldNumElements = 8;
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryOne.state(tIndex).size(), tGoldNumElements);
    }

    Plato::StandardVectorReductionOperations<double,size_t> tInterface;
    tFactoryOne.allocateStateReductionOperations(tInterface);

    std::vector<double> tStandardVector = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double,size_t> tPlatoVector(tStandardVector);
    double tSum = tFactoryOne.getStateReductionOperations().sum(tPlatoVector);
    double tGold = 55;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tSum, tGold, tTolerance);

    // Test Second Factory for State Data
    Plato::DataFactory<double,size_t> tFactoryTwo;
    Plato::StandardMultiVector<double,size_t> tMultiVector(tNumVectors, tPlatoVector);
    tFactoryTwo.allocateState(tMultiVector);

    tGoldNumElements = 10;
    EXPECT_EQ(tFactoryTwo.state().getNumVectors(), tGoldNumVectors);
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryTwo.state(tIndex).size(), tGoldNumElements);
    }

    // Test Third Factory for State Data
    Plato::DataFactory<double,size_t> tFactoryThree;
    tFactoryThree.allocateState(tPlatoVector, tNumVectors);

    EXPECT_EQ(tFactoryThree.state().getNumVectors(), tGoldNumVectors);
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryThree.state(tIndex).size(), tGoldNumElements);
    }

    // Test Fourth Factory for State Data (Default NumVectors = 1)
    Plato::DataFactory<double,size_t> tFactoryFour;
    tFactoryFour.allocateState(tPlatoVector);

    tNumVectors = 1;
    tGoldNumVectors = 1;
    EXPECT_EQ(tFactoryFour.state().getNumVectors(), tGoldNumVectors);
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryFour.state(tIndex).size(), tGoldNumElements);
    }
}

TEST(PlatoTest, ControlDataFactory)
{
    Plato::DataFactory<double,size_t> tFactoryOne;

    // ********* Test Factories for Control Data *********
    size_t tNumVectors = 2;
    size_t tNumElements = 8;
    tFactoryOne.allocateControl(tNumElements, tNumVectors);

    size_t tGoldNumVectors = 2;
    EXPECT_EQ(tFactoryOne.control().getNumVectors(), tGoldNumVectors);

    size_t tGoldNumElements = 8;
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryOne.control(tIndex).size(), tGoldNumElements);
    }

    Plato::StandardVectorReductionOperations<double,size_t> tInterface;
    tFactoryOne.allocateControlReductionOperations(tInterface);

    std::vector<double> tStandardVector = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double,size_t> tPlatoVector(tStandardVector);
    double tSum = tFactoryOne.getControlReductionOperations().sum(tPlatoVector);
    double tGold = 55;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tSum, tGold, tTolerance);

    // ********* Test Second Factory for Control Data *********
    Plato::DataFactory<double,size_t> tFactoryTwo;
    Plato::StandardMultiVector<double,size_t> tMultiVector(tNumVectors, tPlatoVector);
    tFactoryTwo.allocateControl(tMultiVector);

    tGoldNumElements = 10;
    EXPECT_EQ(tFactoryTwo.control().getNumVectors(), tGoldNumVectors);
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryTwo.control(tIndex).size(), tGoldNumElements);
    }

    // ********* Test Third Factory for Control Data *********
    Plato::DataFactory<double,size_t> tFactoryThree;
    tFactoryThree.allocateControl(tPlatoVector, tNumVectors);

    EXPECT_EQ(tFactoryThree.control().getNumVectors(), tGoldNumVectors);
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryThree.control(tIndex).size(), tGoldNumElements);
    }

    // ********* Test Fourth Factory for Control Data (Default NumVectors = 1) *********
    Plato::DataFactory<double,size_t> tFactoryFour;
    tFactoryFour.allocateControl(tPlatoVector);

    tNumVectors = 1;
    tGoldNumVectors = 1;
    EXPECT_EQ(tFactoryFour.control().getNumVectors(), tGoldNumVectors);
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        EXPECT_EQ(tFactoryFour.control(tIndex).size(), tGoldNumElements);
    }
}

} // namespace PlatoTest
