/*
 * Plato_LinearAlgebraTest.cpp
 *
 *  Created on: Oct 21, 2017
 */

#include "gtest/gtest.h"

#include <limits>

#include "Plato_UnitTestUtils.hpp"

#include "Plato_StandardVector.hpp"
#include "Plato_DistributedVector.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_DistributedReductionOperations.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"

namespace PlatoTest
{

TEST(PlatoTest, size)
{
    const double tBaseValue = 1;
    const size_t tNumElements = 10;
    std::vector<double> tTemplateVector(tNumElements, tBaseValue);

    Plato::StandardVector<double> tPlatoVector(tTemplateVector);

    const size_t tGold = 10;
    EXPECT_EQ(tPlatoVector.size(), tGold);
}

TEST(PlatoTest, scale)
{
    const double tBaseValue = 1;
    const int tNumElements = 10;
    Plato::StandardVector<double, int> tPlatoVector(tNumElements, tBaseValue);

    double tScaleValue = 2;
    tPlatoVector.scale(tScaleValue);

    double tGold = 2;
    double tTolerance = 1e-6;
    for(int tIndex = 0; tIndex < tPlatoVector.size(); tIndex++)
    {
        EXPECT_NEAR(tPlatoVector[tIndex], tGold, tTolerance);
    }
}

TEST(PlatoTest, entryWiseProduct)
{
    std::vector<double> tTemplateVector =
        { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double, size_t> tPlatoVector1(tTemplateVector);
    Plato::StandardVector<double, size_t> tPlatoVector2(tTemplateVector);

    tPlatoVector1.entryWiseProduct(tPlatoVector2);

    double tTolerance = 1e-6;
    std::vector<double> tGold =
        { 1, 4, 9, 16, 25, 36, 49, 64, 81, 100 };
    for(size_t tIndex = 0; tIndex < tPlatoVector1.size(); tIndex++)
    {
        EXPECT_NEAR(tPlatoVector1[tIndex], tGold[tIndex], tTolerance);
    }
}

TEST(PlatoTest, modulus)
{
    std::vector<double> tTemplateVector =
        { -1, 2, -3, 4, 5, -6, -7, 8, -9, -10 };
    Plato::StandardVector<double> tPlatoVector(tTemplateVector);

    tPlatoVector.modulus();

    double tTolerance = 1e-6;
    std::vector<double> tGold =
        { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    for(size_t tIndex = 0; tIndex < tPlatoVector.size(); tIndex++)
    {
        EXPECT_NEAR(tPlatoVector[tIndex], tGold[tIndex], tTolerance);
    }
}

TEST(PlatoTest, dot)
{
    std::vector<double> tTemplateVector1 =
        { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double> tPlatoVector1(tTemplateVector1);
    std::vector<double> tTemplateVector2 =
        { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 };
    Plato::StandardVector<double> tPlatoVector2(tTemplateVector2);

    double tDot = tPlatoVector1.dot(tPlatoVector2);

    double tGold = 110;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tDot, tGold, tTolerance);
}

TEST(PlatoTest, data)
{
    std::vector<double> tData = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double> tVector(tData);

    double tTolerance = 1e-6;
    for(size_t tIndex = 0; tIndex < tVector.size(); tIndex++)
    {
        EXPECT_NEAR(tData[tIndex], tVector.data()[tIndex], tTolerance);
    }
}

TEST(PlatoTest, fill)
{
    std::vector<double> tTemplateVector =
        { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double> tPlatoVector(tTemplateVector);

    double tFillValue = 3;
    tPlatoVector.fill(tFillValue);

    double tGold = 3.;
    double tTolerance = 1e-6;
    for(size_t tIndex = 0; tIndex < tPlatoVector.size(); tIndex++)
    {
        EXPECT_NEAR(tPlatoVector[tIndex], tGold, tTolerance);
    }
}

TEST(PlatoTest, create)
{
    std::vector<double> tTemplateVector =
        { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double> tPlatoVector(tTemplateVector);

// TEST ONE: CREATE COPY OF BASE CONTAINER WITH THE SAME NUMBER OF ELEMENTS AS THE BASE VECTOR AND FILL IT WITH ZEROS
    std::shared_ptr<Plato::Vector<double>> tCopy1 = tPlatoVector.create();

    size_t tGoldSize1 = 10;
    EXPECT_EQ(tCopy1->size(), tGoldSize1);
    EXPECT_TRUE(tCopy1->size() == tPlatoVector.size());

    double tGoldDot1 = 0;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tCopy1->dot(tPlatoVector), tGoldDot1, tTolerance);
}

TEST(PlatoTest, MultiVector)
{
    size_t tNumVectors = 8;
    std::vector<double> tVectorGold =
        { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double> tPlatoVector(tVectorGold);
    // Default for second template typename is OrdinalType = size_t
    Plato::StandardMultiVector<double> tMultiVector1(tNumVectors, tPlatoVector);

    size_t tGoldNumVectors = 8;
    EXPECT_EQ(tMultiVector1.getNumVectors(), tGoldNumVectors);

    double tGoldSum = 0;
    size_t tGoldSize = 10;

    double tTolerance = 1e-6;
    // Default for second template typename is OrdinalType = size_t
    Plato::StandardVectorReductionOperations<double> tInterface;
    for(size_t tIndex = 0; tIndex < tMultiVector1.getNumVectors(); tIndex++)
    {
        EXPECT_EQ(tMultiVector1[tIndex].size(), tGoldSize);
        double tSumValue = tInterface.sum(tMultiVector1[tIndex]);
        EXPECT_NEAR(tSumValue, tGoldSum, tTolerance);
    }

    std::vector<std::shared_ptr<Plato::Vector<double>>>tMultiVectorTemplate(tNumVectors);
    for(size_t tIndex = 0; tIndex < tNumVectors; tIndex++)
    {
        tMultiVectorTemplate[tIndex] = tPlatoVector.create();
        tMultiVectorTemplate[tIndex]->update(static_cast<double>(1.), tPlatoVector, static_cast<double>(0.));
    }

    // Default for second template typename is OrdinalType = size_t
    tGoldSum = 55;
    Plato::StandardMultiVector<double> tMultiVector2(tMultiVectorTemplate);
    for(size_t tVectorIndex = 0; tVectorIndex < tMultiVector2.getNumVectors(); tVectorIndex++)
    {
        EXPECT_EQ(tMultiVector2[tVectorIndex].size(), tGoldSize);
        for(size_t tElementIndex = 0; tElementIndex < tMultiVector2[tVectorIndex].size(); tElementIndex++)
        {
            EXPECT_NEAR(tMultiVector2(tVectorIndex, tElementIndex), tVectorGold[tElementIndex], tTolerance);
        }
        double tSumValue = tInterface.sum(tMultiVector2[tVectorIndex]);
        EXPECT_NEAR(tSumValue, tGoldSum, tTolerance);
    }
}

TEST(PlatoTest, StandardVectorReductionOperations)
{
    std::vector<double> tTemplateVector =
        { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double, size_t> tPlatoVector(tTemplateVector);

    Plato::StandardVectorReductionOperations<double, size_t> tInterface;

    // Test MAX
    double tMaxValue = tInterface.max(tPlatoVector);
    double tTolerance = 1e-6;
    double tGoldMaxValue = 10;
    EXPECT_NEAR(tMaxValue, tGoldMaxValue, tTolerance);

    // Test MIN
    double tMinValue = tInterface.min(tPlatoVector);
    double tGoldMinValue = 1.;
    EXPECT_NEAR(tMinValue, tGoldMinValue, tTolerance);

    // Test SUM
    double tSum = tInterface.sum(tPlatoVector);
    double tGold = 55;
    EXPECT_NEAR(tSum, tGold, tTolerance);
}

TEST(PlatoTest, DistributedReductionOperations)
{
    std::vector<double> tTemplateVector =
        { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double, size_t> tPlatoVector(tTemplateVector);

    Plato::DistributedReductionOperations<double, size_t> tReductionOperations;

    int tGold = std::numeric_limits<int>::max();
    MPI_Comm_size(MPI_COMM_WORLD, &tGold);
    size_t tNumRanks = tReductionOperations.getNumRanks();

    EXPECT_EQ(static_cast<size_t>(tGold), tNumRanks);

    double tTolerance = 1e-6;
    double tSum = tReductionOperations.sum(tPlatoVector);
    double tGoldSum = static_cast<double>(tNumRanks) * 55.;
    EXPECT_NEAR(tSum, tGoldSum, tTolerance);

    double tGoldMax = 10;
    double tMax = tReductionOperations.max(tPlatoVector);
    EXPECT_NEAR(tMax, tGoldMax, tTolerance);

    double tGoldMin = 1;
    double tMin = tReductionOperations.min(tPlatoVector);
    EXPECT_NEAR(tMin, tGoldMin, tTolerance);

    // NOTE: Default OrdinalType = size_t
    std::shared_ptr<Plato::ReductionOperations<double>> tReductionOperationsCopy = tReductionOperations.create();
    double tSumCopy = tReductionOperationsCopy->sum(tPlatoVector);
    EXPECT_NEAR(tSumCopy, tGoldSum, tTolerance);
}

TEST(PlatoTest, DistributedVector)
{
    std::vector<double> tLocalData = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::DistributedVector<double> tVectorA(MPI_COMM_WORLD, tLocalData);

    // **************** TEST: LOCAL SIZE ****************
    size_t tOrdinalValue = 10;
    EXPECT_EQ(tOrdinalValue, tVectorA.size());

    // **************** TEST: DATA FUNCTION ****************
    const double tTolerance = 1e-6;
    for(size_t tIndex = 0; tIndex < tVectorA.size(); tIndex++)
    {
        EXPECT_NEAR(tLocalData[tIndex], tVectorA.data()[tIndex], tTolerance);
    }

    // **************** TEST: SCALE FUNCTION ****************
    double tScalarValue = 3;
    tVectorA.scale(tScalarValue);
    tLocalData = { 3, 6, 9, 12, 15, 18, 21, 24, 27, 30 };
    Plato::DistributedVector<double> tVectorB(MPI_COMM_WORLD, tLocalData);
    PlatoTest::checkVectorData(tVectorA, tVectorB);

    // **************** TEST: ENTRY WISE PRODUCT FUNCTION ****************
    tVectorA.entryWiseProduct(tVectorB);
    tVectorB[0] = 9;
    tVectorB[1] = 36;
    tVectorB[2] = 81;
    tVectorB[3] = 144;
    tVectorB[4] = 225;
    tVectorB[5] = 324;
    tVectorB[6] = 441;
    tVectorB[7] = 576;
    tVectorB[8] = 729;
    tVectorB[9] = 900;
    PlatoTest::checkVectorData(tVectorA, tVectorB);

    // **************** TEST: FILL FUNCTION ****************
    tScalarValue = 2;
    tVectorA.fill(tScalarValue);
    tVectorB.fill(tScalarValue);
    PlatoTest::checkVectorData(tVectorA, tVectorB);

    // **************** TEST: MODULUS FUNCTION ****************
    tScalarValue = -2;
    tVectorA.fill(tScalarValue);
    tVectorA.modulus();
    tScalarValue = 2;
    tVectorB.fill(tScalarValue);
    PlatoTest::checkVectorData(tVectorA, tVectorB);

    // **************** TEST: DOT FUNCTION ****************
    tScalarValue = 1;
    tVectorA.fill(tScalarValue);
    tVectorB.fill(tScalarValue);
    double tInnerProduct = tVectorA.dot(tVectorB);
    int tNumRanks = 0;
    MPI_Comm_size(MPI_COMM_WORLD, &tNumRanks);
    tScalarValue = static_cast<double>(tNumRanks) * static_cast<double>(tVectorA.size());
    EXPECT_NEAR(tInnerProduct, tScalarValue, tTolerance);

    // **************** TEST: UPDATE FUNCTION ****************
    tVectorA.update(1., tVectorB, 1.);
    tScalarValue = 2;
    tVectorB.fill(tScalarValue);
    PlatoTest::checkVectorData(tVectorA, tVectorB);

    tVectorA.update(1., tVectorB, 0.);
    tScalarValue = 2;
    tVectorB.fill(tScalarValue);
    PlatoTest::checkVectorData(tVectorA, tVectorB);

    tScalarValue = 4;
    tVectorA.fill(tScalarValue);
    tVectorA.update(0., tVectorB, 1.);
    tVectorB.fill(tScalarValue);
    PlatoTest::checkVectorData(tVectorA, tVectorB);

    // **************** TEST: CREATE FUNCTION ****************
    std::shared_ptr<Plato::Vector<double>> tVectorC = tVectorA.create();
    EXPECT_EQ(tVectorC->size(), tVectorB.size());
    tInnerProduct = tVectorC->dot(tVectorB);
    tScalarValue = 0;
    EXPECT_NEAR(tInnerProduct, tScalarValue, tTolerance);
}

} // namespace PlatoTest
