#include <PSL_OrthogonalGridUtilities.hpp>
#include "PSL_FreeHelpers.hpp"
#include <iostream>
#include <limits>
#include <cmath>
#include <iterator>

namespace PlatoSubproblemLibrary
{

std::vector<size_t> OrthogonalGridUtilities::computeNumElementsInEachDirection(const Vector& aMaxUVWCoords, const Vector& aMinUVWCoords, const double& aTargetEdgeLength) const
{
    double tULength = aMaxUVWCoords(0) - aMinUVWCoords(0);
    double tVLength = aMaxUVWCoords(1) - aMinUVWCoords(1);
    double tWLength = aMaxUVWCoords(2) - aMinUVWCoords(2);

    if(aTargetEdgeLength <= 0.0)
        throw(std::domain_error("OrthogonalGridUtilities: target edge length must be greater than zero"));

    std::vector<size_t> tNumElements;

    tNumElements.push_back(aTargetEdgeLength > tULength ? 1 : (size_t) tULength/aTargetEdgeLength);
    tNumElements.push_back(aTargetEdgeLength > tVLength ? 1 : (size_t) tVLength/aTargetEdgeLength);
    tNumElements.push_back(aTargetEdgeLength > tWLength ? 1 : (size_t) tWLength/aTargetEdgeLength);

    return tNumElements;
}

size_t OrthogonalGridUtilities::getSerializedIndex(const size_t& i, const size_t& j, const size_t& k) const
{
    if(i < 0 || j < 0 || k < 0 || i > mNumElementsInEachDirection[0] || j > mNumElementsInEachDirection[1] || k > mNumElementsInEachDirection[2])
    {
        throw(std::out_of_range("OrthogonalGridUtilities: Index in each direction must be between zero and number of grid elements"));
    }

    return i + j*(mNumElementsInEachDirection[0]+1) + k*(mNumElementsInEachDirection[0]+1)*(mNumElementsInEachDirection[1]+1);
}

size_t OrthogonalGridUtilities::getSerializedIndex(const std::vector<size_t>& aIndex) const
{
    if(aIndex.size() != 3)
    {
        throw(std::domain_error("OrthogonalGridUtilities: Index must have 3 entries"));
    }

    return getSerializedIndex(aIndex.at(0), aIndex.at(1), aIndex.at(2));
}    

std::vector<std::vector<size_t>> OrthogonalGridUtilities::getSupportIndices(const size_t& i, const size_t& j, const size_t& k) const
{
    auto tDimensions = getGridDimensions();
    std::vector<std::vector<size_t>> tIndices;

    std::vector<size_t> tFirstIndexSupports;
    std::vector<size_t> tSecondIndexSupports;
    std::vector<size_t> tThirdIndexSupports;

        tFirstIndexSupports.push_back(i);
        if(i > 0)
            tFirstIndexSupports.push_back(i-1);
        if(i < tDimensions[0] - 1)
            tFirstIndexSupports.push_back(i+1);

        tSecondIndexSupports.push_back(j);
        if(j > 0)
            tSecondIndexSupports.push_back(j-1);
        if(j < tDimensions[1] - 1)
            tSecondIndexSupports.push_back(j+1);

        if(k > 0)
            tThirdIndexSupports.push_back(k-1);

        for(auto tFirst : tFirstIndexSupports)
        {
            for(auto tSecond : tSecondIndexSupports)
            {
                for(auto tThird : tThirdIndexSupports)
                {
                    if(tFirst == i || tSecond == j)
                        tIndices.push_back({tFirst,tSecond,tThird});
                }
            }
        }
                
    return tIndices;
}

std::vector<std::vector<size_t>> OrthogonalGridUtilities::getSupportIndices(const std::vector<size_t>& aIndex) const
{
    if(aIndex.size() != 3)
    {
        throw(std::domain_error("OrthogonalGridUtilities: Index must have 3 entries"));
    }

    return getSupportIndices(aIndex.at(0), aIndex.at(1), aIndex.at(2));
}

void OrthogonalGridUtilities::computeGridXYZCoordinates(std::vector<Vector>& aXYZCoordinates) const
{
    auto tDimension = getGridDimensions();

    aXYZCoordinates.resize(tDimension[0] * tDimension[1] * tDimension[2]);

    for(size_t i = 0; i < tDimension[0]; ++i)
    {
        for(size_t j = 0; j < tDimension[1]; ++j)
        {
            for(size_t k = 0; k < tDimension[2]; ++k)
            {
                aXYZCoordinates[getSerializedIndex(i,j,k)] = computeGridPointXYZCoordinates(i,j,k);
            }
        }
    }
}

Vector OrthogonalGridUtilities::computeGridPointXYZCoordinates(const std::vector<size_t>& aIndex) const
{
    if(aIndex.size() != 3)
        throw(std::domain_error("OrthogonalGridUtilities::computeGridPointXYZCoordinates: Index must have 3 dimensions"));

    double tULength = mMaxUVWCoords(0) - mMinUVWCoords(0);
    double tVLength = mMaxUVWCoords(1) - mMinUVWCoords(1);
    double tWLength = mMaxUVWCoords(2) - mMinUVWCoords(2);

    std::vector<double> tLength = {tULength,tVLength,tWLength};

    std::vector<Vector> tBasis;
    tBasis.push_back(mUBasisVector);
    tBasis.push_back(mVBasisVector);
    tBasis.push_back(mWBasisVector);

    Vector tXYZCoordinates({0.0,0.0,0.0});

    for(size_t tTempIndex = 0; tTempIndex < 3; ++tTempIndex)
    {
        double tUVWCoordinate = mMinUVWCoords(tTempIndex) + aIndex[tTempIndex]*tLength[tTempIndex]/mNumElementsInEachDirection[tTempIndex]; 
        tXYZCoordinates.set(0,tXYZCoordinates(0)+tUVWCoordinate*tBasis[tTempIndex](0));
        tXYZCoordinates.set(1,tXYZCoordinates(1)+tUVWCoordinate*tBasis[tTempIndex](1));
        tXYZCoordinates.set(2,tXYZCoordinates(2)+tUVWCoordinate*tBasis[tTempIndex](2));
    }

    return tXYZCoordinates;
}

Vector OrthogonalGridUtilities::computeGridPointXYZCoordinates(const size_t& i, const size_t& j, const size_t& k) const
{
    std::vector<size_t> tIndex({i,j,k});
    return computeGridPointXYZCoordinates(tIndex);
}

void OrthogonalGridUtilities::checkBasis(const Vector& aUBasisVector,
                                         const Vector& aVBasisVector,
                                         const Vector& aWBasisVector) const
{
    if(fabs(aUBasisVector.euclideanNorm() - 1) > 1e-12 || fabs(aVBasisVector.euclideanNorm() -1) > 1e-12 || fabs(aWBasisVector.euclideanNorm() - 1) > 1e-12)
    {
        throw(std::domain_error("OrthogonalGridUtilities: provided basis not unit length"));
    }

    if(dot_product(aUBasisVector,aVBasisVector) > 1e-12 || dot_product(aUBasisVector,aWBasisVector) > 1e-12 || dot_product(aVBasisVector,aWBasisVector) > 1e-12)
    {
        throw(std::domain_error("OrthogonalGridUtilities: provided basis is not orthogonal"));
    }

    if(dot_product(cross_product(aUBasisVector,aVBasisVector),aWBasisVector) < 0)
    {
        throw(std::domain_error("OrthogonalGridUtilities: provided basis is not positively oriented"));
    }
}

void OrthogonalGridUtilities::checkBounds(const Vector& aMaxUVWCoords,
                                          const Vector& aMinUVWCoords) const
{
    for(size_t i = 0; i < 3; ++i)
    {
        if(aMaxUVWCoords(i) <= aMinUVWCoords(i))
            throw(std::domain_error("OrthogonalGridUtilities: Max coordinates not strictly greater than min coordinates"));
    }
}

void OrthogonalGridUtilities::checkTargetEdgeLength(const double& aTargetEdgeLength) const
{
    if(aTargetEdgeLength <= 0)
        throw(std::domain_error("OrthogonalGridUtilities: Target edge length is not positive"));
}

void OrthogonalGridUtilities::checkNumElementsInEachDirection(const std::vector<size_t>& aNumElementsInEachDirection) const
{
    if(aNumElementsInEachDirection.size() != 3u)
        throw(std::domain_error("OrthogonalGridUtilities: Grid dimension must be 3"));

    for(size_t i = 0; i < 3; ++i)
    {
        if(aNumElementsInEachDirection[i] <= 0)
            throw(std::domain_error("OrthogonalGridUtilities: Number of elements in each direction must be positive"));
    }
}

std::vector<std::vector<size_t>> OrthogonalGridUtilities::getContainingGridElement(const Vector& aPoint) const
{
    std::vector<std::vector<size_t>> tGridIndicies;

    std::vector<size_t> tUIndices = getSurroundingIndices(0,aPoint);
    std::vector<size_t> tVIndices = getSurroundingIndices(1,aPoint);
    std::vector<size_t> tWIndices = getSurroundingIndices(2,aPoint);

    if(tUIndices.size() == 2u && tVIndices.size() == 2u && tWIndices.size() == 2)
    {
        tGridIndicies.push_back({tUIndices[0],tVIndices[0],tWIndices[0]});
        tGridIndicies.push_back({tUIndices[1],tVIndices[0],tWIndices[0]});
        tGridIndicies.push_back({tUIndices[0],tVIndices[1],tWIndices[0]});
        tGridIndicies.push_back({tUIndices[1],tVIndices[1],tWIndices[0]});
        tGridIndicies.push_back({tUIndices[0],tVIndices[0],tWIndices[1]});
        tGridIndicies.push_back({tUIndices[1],tVIndices[0],tWIndices[1]});
        tGridIndicies.push_back({tUIndices[0],tVIndices[1],tWIndices[1]});
        tGridIndicies.push_back({tUIndices[1],tVIndices[1],tWIndices[1]});
    }
    else
        throw(std::domain_error("Didn't find point"));

    return tGridIndicies;
}

std::vector<size_t> OrthogonalGridUtilities::getSurroundingIndices(const size_t& aDim, const Vector& aXYZPoint) const
{
    if(aDim < 0 || aDim > 2)
        throw(std::domain_error("OrthogonalGridUtilities: Provided index must be between 0 and 2"));

    Vector tUVWCoordinates = computePointUVWCoordinates(aXYZPoint);
    double tCoordinate = tUVWCoordinates(aDim);
    double tLength = mMaxUVWCoords(aDim) - mMinUVWCoords(aDim);
    size_t tMaxIndex = mNumElementsInEachDirection[aDim];
    double tGridSegmentLength = tLength/tMaxIndex;
    double tModifiedCoordinate = tCoordinate - mMinUVWCoords(aDim);

    // guess value for tIndex
    size_t tIndex, tNextIndex;
    double tIndexGuess = std::floor(tModifiedCoordinate / tGridSegmentLength);

    bool found_point = false;

    if(tIndexGuess <= 0.0)
    {
        tIndex = 0u;
        tNextIndex = 1u;
    }
    else if(tIndexGuess >= ((double) tMaxIndex) - 1.0)
    {
        tNextIndex = tMaxIndex;
        tIndex = tMaxIndex - 1u;
    }
    else
    {
        size_t tStart = ((size_t) tIndexGuess) - 1u;
        for(size_t i = tStart; i < tStart + 3u; ++i)
        {
            double tFloor = mMinUVWCoords(aDim) + (double) (i*tLength) / (double) tMaxIndex;
            double tCeiling = mMinUVWCoords(aDim) + (double) ((i+1)*tLength) / (double) tMaxIndex;

            if(tFloor <= tCoordinate && tCoordinate <= tCeiling)
            {
                tIndex = i;
                tNextIndex = i + 1u;
                found_point = true;
                break;
            }
        }

        if(!found_point)
            throw(std::runtime_error("OrthogonalGridUtilities::getSurroundingIndices: Couldn't find surrounding grid indices"));
    }

    std::vector<size_t> tSurroundingIndices;
    tSurroundingIndices.push_back(tIndex);
    tSurroundingIndices.push_back(tNextIndex);

    return tSurroundingIndices;
}

Vector OrthogonalGridUtilities::computePointUVWCoordinates(const Vector& aXYZPoint) const
{
    //use cramers rule to compute change of basis for input point
    double tDenominator = determinant3X3(mUBasisVector,mVBasisVector,mWBasisVector);

    double tU = determinant3X3(aXYZPoint,mVBasisVector,mWBasisVector)/tDenominator;
    double tV = determinant3X3(mUBasisVector,aXYZPoint,mWBasisVector)/tDenominator;
    double tW = determinant3X3(mUBasisVector,mVBasisVector,aXYZPoint)/tDenominator;

    return Vector({tU,tV,tW});
}

void OrthogonalGridUtilities::checkIndexFormat(const std::vector<std::vector<size_t>>& aContainingElementIndicies) const
{
    if(aContainingElementIndicies.size() != 8u)
        throw(std::domain_error("OrthogonalGridUtilities: Incorrect number of indices provided"));

    for(auto tIndex : aContainingElementIndicies)
    {
        if(tIndex.size() != 3)
            throw(std::domain_error("OrthogonalGridUtilities: Incorrect number of dimensions in provided index"));
    }

    std::vector<size_t> tIndex0 = aContainingElementIndicies[0];

    if(tIndex0[0] < 0 || tIndex0[0] >= mNumElementsInEachDirection[0]
       || tIndex0[1] < 0 || tIndex0[1] >= mNumElementsInEachDirection[1]
       || tIndex0[2] < 0 || tIndex0[2] >= mNumElementsInEachDirection[2])
    {
        throw(std::out_of_range("OrthogonalGridUtilities: provided indices must be between zero and the number of grid elements in each dimension"));
    }

    //provided indices must have the following ordering
    //{a,b,c};
    //{a+1,b,c};
    //{a,b+1,c};
    //{a+1,b+1,c};
    //{a,b,c+1};
    //{a+1,b,c+1};
    //{a,b+1,c+1};
    //{a+1,b+1,c+1};
    
    std::vector<std::vector<size_t>> tTestIndices;
    
    for(auto tIndex : aContainingElementIndicies)
    {
        tTestIndices.push_back({tIndex[0] - tIndex0[0], tIndex[1] - tIndex0[1], tIndex[2] - tIndex0[2]});
    }

    if(tTestIndices[0][0] != 0 || tTestIndices[0][1] != 0 || tTestIndices[0][2] != 0
    || tTestIndices[1][0] != 1 || tTestIndices[1][1] != 0 || tTestIndices[1][2] != 0
    || tTestIndices[2][0] != 0 || tTestIndices[2][1] != 1 || tTestIndices[2][2] != 0
    || tTestIndices[3][0] != 1 || tTestIndices[3][1] != 1 || tTestIndices[3][2] != 0
    || tTestIndices[4][0] != 0 || tTestIndices[4][1] != 0 || tTestIndices[4][2] != 1
    || tTestIndices[5][0] != 1 || tTestIndices[5][1] != 0 || tTestIndices[5][2] != 1
    || tTestIndices[6][0] != 0 || tTestIndices[6][1] != 1 || tTestIndices[6][2] != 1
    || tTestIndices[7][0] != 1 || tTestIndices[7][1] != 1 || tTestIndices[7][2] != 1)
    {
        throw(std::domain_error("OrthogonalGridUtilities: Indices not formatted correctly"));
    }

}

double OrthogonalGridUtilities::interpolateScalar(const std::vector<std::vector<size_t>>& aContainingElementIndicies,
                                                  const std::vector<double>& aScalarValues,
                                                  const Vector& aPoint) const
{
    checkIndexFormat(aContainingElementIndicies);

    if(aScalarValues.size() != 8u)
        throw(std::domain_error("OrthogonalGridUtilities::interpolateScalar: 8 scalar values must be provided"));

    std::vector<Vector> tElementCoordinates;
    for(auto tIndex : aContainingElementIndicies)
        tElementCoordinates.push_back(computeGridPointUVWCoordinates(tIndex));

    Vector tMaxUVWCoords;
    Vector tMinUVWCoords;
    computeBoundingBox(tElementCoordinates,tMinUVWCoords,tMaxUVWCoords);

    RegularHex8 tHex(tMinUVWCoords,tMaxUVWCoords);

    Vector tUVWPoint = computePointUVWCoordinates(aPoint);

    double tVal = tHex.interpolateScalar(tUVWPoint,aScalarValues);

    return tVal;
}

Vector OrthogonalGridUtilities::computeGridPointUVWCoordinates(const std::vector<size_t>& aIndex) const
{
    std::vector<size_t> tDimensions = getGridDimensions();
    
    for(size_t i = 0; i < 3; ++i)
    {
        if(aIndex[i] < 0 || aIndex[i] > tDimensions[i] - 1)
            throw(std::out_of_range("OrthogonalGridUtilities::computeGridPointUVWCoordinates: Index must be between zero and number of elements in each direction"));
    }
    
    double tULength = mMaxUVWCoords(0) - mMinUVWCoords(0);
    double tVLength = mMaxUVWCoords(1) - mMinUVWCoords(1);
    double tWLength = mMaxUVWCoords(2) - mMinUVWCoords(2);

    double tU = mMinUVWCoords(0) + (aIndex[0]*tULength)/mNumElementsInEachDirection[0];
    double tV = mMinUVWCoords(1) + (aIndex[1]*tVLength)/mNumElementsInEachDirection[1];
    double tW = mMinUVWCoords(2) + (aIndex[2]*tWLength)/mNumElementsInEachDirection[2];

    Vector tUVWCoordinates({tU,tV,tW});

    return tUVWCoordinates;
}

Vector OrthogonalGridUtilities::computeGridPointUVWCoordinates(const size_t& i, const size_t& j, const size_t& k) const
{
    std::vector<size_t> tIndex({i,j,k});
    return computeGridPointUVWCoordinates(tIndex);
}

}
