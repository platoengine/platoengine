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
        OrthogonalGridUtilities(const Vector aUBasisVector,
                                const Vector aVBasisVector,
                                const Vector aWBasisVector,
                                const Vector aMaxUVWCoords,
                                const Vector aMinUVWCoords,
                                const double aTargetEdgeLength)
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
                                const std::vector<size_t>& aNumElementsInEachDirection)
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

        std::vector<size_t> getGridDimensions() const {return std::vector<size_t>({mNumElementsInEachDirection[0]+1,
                                                                                   mNumElementsInEachDirection[1]+1,
                                                                                   mNumElementsInEachDirection[2]+1});}

        void computeGridXYZCoordinates(std::vector<Vector>& aXYZCoordinates) const;
        Vector computeGridPointXYZCoordinates(const std::vector<size_t>& aIndex) const;
        Vector computeGridPointXYZCoordinates(const size_t& i, const size_t& j, const size_t& k) const;
        Vector computeGridPointUVWCoordinates(const std::vector<size_t>& aIndex) const;
        Vector computeGridPointUVWCoordinates(const size_t& i, const size_t& j, const size_t& k) const;
        Vector computePointUVWCoordinates(const Vector& aPoint) const;
        std::vector<std::vector<size_t>> getContainingGridElement(const Vector& aPoint) const;

        size_t getSerializedIndex(const size_t& i, const size_t& j, const size_t& k) const;
        size_t getSerializedIndex(const std::vector<size_t>& aIndex) const;

        std::vector<std::vector<size_t>> getSupportIndices(const size_t& i, const size_t& j, const size_t& k) const;
        std::vector<std::vector<size_t>> getSupportIndices(const std::vector<size_t>& aIndex) const;

        std::vector<size_t> getSurroundingIndices(const size_t& aDim, const Vector& aPoint) const;

        double interpolateScalar(const std::vector<std::vector<size_t>>& aContainingElementIndicies,
                                 const std::vector<double>& aScalarValues,
                                 const Vector& aPoint) const;

    private:

        void checkBasis(const Vector& aUBasisVector,
                        const Vector& aVBasisVector,
                        const Vector& aWBasisVector) const;

        void checkBounds(const Vector& aMaxUVWCoords,
                         const Vector& aMinUVWCoords) const;

        void checkTargetEdgeLength(const double& aTargetEdgeLength) const;

        void checkNumElementsInEachDirection(const std::vector<size_t>& aNumElementsInEachDirection) const;

        std::vector<size_t> computeNumElementsInEachDirection(const Vector& aMaxUVWCoords,
                                                           const Vector& aMinUVWCoords,
                                                           const double& aTargetEdgeLength) const;

        void checkIndexFormat(const std::vector<std::vector<size_t>>& aContainingElementIndicies) const;

        const Vector mUBasisVector;
        const Vector mVBasisVector;
        const Vector mWBasisVector;
        const Vector mMaxUVWCoords;
        const Vector mMinUVWCoords;

        std::vector<size_t> mNumElementsInEachDirection;

};


}
