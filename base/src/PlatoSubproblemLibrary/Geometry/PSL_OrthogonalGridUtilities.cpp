#include <PSL_OrthogonalGridUtilities.hpp>
#include "PSL_FreeHelpers.hpp"
#include <iostream>
#include <limits>
#include <cmath>

namespace PlatoSubproblemLibrary
{

std::vector<int> OrthogonalGridUtilities::computeNumElementsInEachDirection(const Vector& aMaxUVWCoords, const Vector& aMinUVWCoords, const double& aTargetEdgeLength) const
{
    double tULength = aMaxUVWCoords(0) - aMinUVWCoords(0);
    double tVLength = aMaxUVWCoords(1) - aMinUVWCoords(1);
    double tWLength = aMaxUVWCoords(2) - aMinUVWCoords(2);

    if(aTargetEdgeLength <= 0.0)
        throw(std::domain_error("OrthogonalGridUtilities: target edge length must be greater than zero"));

    std::vector<int> tNumElements;

    tNumElements.push_back(aTargetEdgeLength > tULength ? 1 : (int) tULength/aTargetEdgeLength);
    tNumElements.push_back(aTargetEdgeLength > tVLength ? 1 : (int) tVLength/aTargetEdgeLength);
    tNumElements.push_back(aTargetEdgeLength > tWLength ? 1 : (int) tWLength/aTargetEdgeLength);

    return tNumElements;
}

int OrthogonalGridUtilities::getSerializedIndex(const int& i, const int& j, const int& k) const
{
    if(i < 0 || j < 0 || k < 0 || i > mNumElementsInEachDirection[0] || j > mNumElementsInEachDirection[1] || k > mNumElementsInEachDirection[2])
    {
        throw(std::out_of_range("OrthogonalGridUtilities: Index in each direction must be between zero and number of grid points"));
    }

    return i + j*(mNumElementsInEachDirection[0]+1) + k*(mNumElementsInEachDirection[0]+1)*(mNumElementsInEachDirection[1]+1);
}

int OrthogonalGridUtilities::getSerializedIndex(const std::vector<int>& aIndex) const
{
    if(aIndex.size() != 3)
    {
        throw(std::domain_error("OrthogonalGridUtilities: Index must have 3 entries"));
    }

    return getSerializedIndex(aIndex.at(0), aIndex.at(1), aIndex.at(2));
}

// void computeGridXYZCoordinates(const Vector& aUBasisVector,
//                                  const Vector& aVBasisVector,
//                                  const Vector& aBuildDirection,
//                                  const Vector& aMaxUVWCoords,
//                                  const Vector& aMinUVWCoords,
//                                  const std::vector<int>& aNumElements,
//                                  std::vector<Vector>& aXYZCoordinates)
// {
//     double tULength = aMaxUVWCoords(0) - aMinUVWCoords(0);
//     double tVLength = aMaxUVWCoords(1) - aMinUVWCoords(1);
//     double tWLength = aMaxUVWCoords(2) - aMinUVWCoords(2);

//     std::vector<double> tLength = {tULength,tVLength,tWLength};
//     std::vector<Vector> tBasis = {aUBasisVector,aVBasisVector,aBuildDirection};

//     if(tULength < 0 || tVLength < 0 || tWLength < 0)
//         throw(std::domain_error("OrthogonalGridUtilities::computeGridXYZCoordinates: Max UVW coordinates expected to be greater than Min UVW coordinates"));

//     if(aNumElements[0] <= 0 || aNumElements[1] <= 0 || aNumElements[2] <= 0)
//         throw(std::domain_error("OrthogonalGridUtilities::computeGridXYZCoordinates: Number of elements in each direction must be greater than zero"));

//     aXYZCoordinates.resize((aNumElements[0]+1) * (aNumElements[1]+1) * (aNumElements[2]+1));


//     for(int i = 0; i <= aNumElements[0]; ++i)
//     {
//         for(int j = 0; j <= aNumElements[1]; ++j)
//         {
//             for(int k = 0; k <= aNumElements[2]; ++k)
//             {

//                 Vector tXYZCoordinates({0.0,0.0,0.0});
//                 Vector tUVWCoordinates({0.0,0.0,0.0});

//                 std::vector<int> tIndex = {i,j,k};

//                 for(int tTempIndex = 0; tTempIndex < 3; ++tTempIndex)
//                 {
//                     double tUVWCoordinate = aMinUVWCoords(tTempIndex) + tIndex[tTempIndex]*tLength[tTempIndex]/aNumElements[tTempIndex]; 
//                     tUVWCoordinates.set(tTempIndex,tUVWCoordinate);
//                 }

//                 tXYZCoordinates = tUVWCoordinates(0)*aUBasisVector + tUVWCoordinates(1)*aVBasisVector + tUVWCoordinates(2)*aBuildDirection;

//                 aXYZCoordinates[getSerializedIndex(aNumElements,i,j,k)] = tXYZCoordinates;
//             }
//         }
//     }
// }

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
    for(int i = 0; i < 3; ++i)
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

void OrthogonalGridUtilities::checkNumElementsInEachDirection(const std::vector<int>& aNumElementsInEachDirection) const
{
    if(aNumElementsInEachDirection.size() != 3u)
        throw(std::domain_error("OrthogonalGridUtilities: Grid dimension must be 3"));

    for(int i = 0; i < 3; ++i)
    {
        if(aNumElementsInEachDirection[i] <= 0)
            throw(std::domain_error("OrthogonalGridUtilities: Number of elements in each direction must be positive"));
    }
}

}
