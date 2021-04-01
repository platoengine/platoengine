#pragma once

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <cassert>
#include <tuple>
#include <set>
#include <map>

#include <PSL_Vector.hpp>
#include <PSL_RegularHex8.hpp>

namespace PlatoSubproblemLibrary
{

class OrthogonalGridUtilities
{
    public:
        OrthogonalGridUtilities(const Vector& aUBasisVector,
                                const Vector& aVBasisVector,
                                const Vector& aWBasisVector,
                                const Vector& aMaxUVWCoords,
                                const Vector& aMinUVWCoords,
                                const double& aTargetEdgeLength)
            :mUBasisVector(aUBasisVector),
             mVBasisVector(aVBasisVector),
             mWBasisVector(aWBasisVector),
             mMaxUVWCoords(aMaxUVWCoords),
             mMinUVWCoords(aMinUVWCoords)
        {
            checkBasis(aUBasisVector,aVBasisVector,aWBasisVector);
            checkBounds(aMaxUVWCoords,aMinUVWCoords);
            checkTargetEdgeLength(aTargetEdgeLength);

            mNumElementsInEachDirection = computeNumElementsInEachDirection(aMaxUVWCoords,aMinUVWCoords,aTargetEdgeLength);
        }

        OrthogonalGridUtilities(const Vector& aUBasisVector,
                                const Vector& aVBasisVector,
                                const Vector& aWBasisVector,
                                const Vector& aMaxUVWCoords,
                                const Vector& aMinUVWCoords,
                                const std::vector<int>& aNumElementsInEachDirection)
            :mUBasisVector(aUBasisVector),
             mVBasisVector(aVBasisVector),
             mWBasisVector(aWBasisVector),
             mMaxUVWCoords(aMaxUVWCoords),
             mMinUVWCoords(aMinUVWCoords),
             mNumElementsInEachDirection(aNumElementsInEachDirection)
        {
            checkBasis(aUBasisVector,aVBasisVector,aWBasisVector);
            checkBounds(aMaxUVWCoords,aMinUVWCoords);
            checkNumElementsInEachDirection(aNumElementsInEachDirection);
        }

        std::vector<int> getGridDimensions() const {return std::vector<int>({mNumElementsInEachDirection[0]+1,
                                                                             mNumElementsInEachDirection[1]+1,
                                                                             mNumElementsInEachDirection[2]+1});}

        void computeGridXYZCoordinates(std::vector<Vector>& aXYZCoordinates) const;
        Vector computeGridPointXYZCoordinates(const std::vector<int>& aIndex) const;
        Vector computeGridPointXYZCoordinates(const int& i, const int& j, const int& k) const;
        std::vector<std::vector<int>> getContainingGridElement(const Vector& aPoint) const;

        int getSerializedIndex(const int& i, const int& j, const int& k) const;
        int getSerializedIndex(const std::vector<int>& aIndex) const;

        std::vector<std::vector<int>> getSupportIndices(const int& i, const int& j, const int& k) const;
        std::vector<std::vector<int>> getSupportIndices(const std::vector<int>& aIndex) const;

        std::vector<int> getSurroundingIndices(const int& aDim, const Vector& aPoint) const;

        double interpolateScalar(const std::vector<std::vector<int>>& aContainingElementIndicies,
                                 const std::vector<double>& aScalarValues,
                                 const Vector& aPoint) const;
    private:

        void checkBasis(const Vector& aUBasisVector,
                        const Vector& aVBasisVector,
                        const Vector& aWBasisVector) const;

        void checkBounds(const Vector& aMaxUVWCoords,
                         const Vector& aMinUVWCoords) const;

        void checkTargetEdgeLength(const double& aTargetEdgeLength) const;

        void checkNumElementsInEachDirection(const std::vector<int>& aNumElementsInEachDirection) const;

        std::vector<int> computeNumElementsInEachDirection(const Vector& aMaxUVWCoords,
                                                           const Vector& aMinUVWCoords,
                                                           const double& aTargetEdgeLength) const;

        void checkIndexFormat(const std::vector<std::vector<int>>& aContainingElementIndicies) const;

        const Vector& mUBasisVector;
        const Vector& mVBasisVector;
        const Vector& mWBasisVector;
        const Vector& mMaxUVWCoords;
        const Vector& mMinUVWCoords;

        std::vector<int> mNumElementsInEachDirection;

};


}