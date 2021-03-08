#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <cassert>
#include <tuple>
#include <set>
#include <map>
#include <PSL_Vector.hpp>


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
        }
        
    private:

        void checkBasis(const Vector& aUBasisVector,
                        const Vector& aVBasisVector,
                        const Vector& aWBasisVector) const;

        void checkBounds(const Vector& aMaxUVWCoords,
                         const Vector& aMinUVWCoords) const;

        void checkTargetEdgeLength(const double& aTargetEdgeLength) const;

        const Vector& mUBasisVector;
        const Vector& mVBasisVector;
        const Vector& mWBasisVector;
        const Vector& mMaxUVWCoords;
        const Vector& mMinUVWCoords;

};

std::vector<int> computeNumElementsInEachDirection(const Vector& aMaxUVWCoords,
        const Vector& aMinUVWCoords,
        const double& aTargetEdgeLength);

void computeGridXYZCoordinates(const Vector& aUBasisVector,
        const Vector& aVBasisVector,
        const Vector& aBuildDirection,
        const Vector& aMaxUVWCoords,
        const Vector& aMinUVWCoords,
        const std::vector<int>& aNumElements,
        std::vector<Vector>& aXYZCoordinates);

int getSerializedIndex(const std::vector<int>& aNumElementsInEachDirection, const int& i, const int& j, const int& k);
int getSerializedIndex(const std::vector<int>& aNumElementsInEachDirection, const std::vector<int>& aIndex);

}
