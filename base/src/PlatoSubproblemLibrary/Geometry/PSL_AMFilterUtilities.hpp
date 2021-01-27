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

class AMFilterUtilities
{
    public:
        AMFilterUtilities(const std::vector<std::vector<double>>& aCoordinates,
                           const std::vector<std::vector<int>>& aConnectivity,
                           Vector aUBasisVector,
                           Vector aVBasisVector,
                           Vector aBuildDirection,
                           const std::vector<int>& aBaseLayer) 
            :mCoordinates(aCoordinates),
             mConnectivity(aConnectivity),
             mUBasisVector(aUBasisVector),
             mVBasisVector(aVBasisVector),
             mBuildDirection(aBuildDirection),
             mBaseLayer(aBaseLayer)
        {
            checkInput();
        }

        ~AMFilterUtilities(){}

        void computeBoundingBox(Vector& aMaxUVWCoords, Vector& aMinUVWCoords) const;

        double computeMinEdgeLength() const;

        bool pointInTetrahedron(const std::vector<int>& aTet, const Vector& aPoint) const;

    private:

        void checkInput() const;

        bool sameSide(const int& v1, const int& v2, const int& v3, const int& v4, const Vector& aPoint) const;

    private:
        const std::vector<std::vector<double>>& mCoordinates;
        const std::vector<std::vector<int>>& mConnectivity;
        Vector mUBasisVector;
        Vector mVBasisVector;
        Vector mBuildDirection;
        const std::vector<int>& mBaseLayer;
};

std::vector<int> computeNumElementsInEachDirection(const Vector& aMaxUVWCoords,
                                                   const Vector& aMinUVWCoords,
                                                   const double& aTargetEdgeLength);

Vector computeGridXYZCoordinates(const Vector& aUBasisVector,
                                 const Vector& aVBasisVector,
                                 const Vector& aBuildDirection,
                                 const Vector& aMaxUVWCoords,
                                 const Vector& aMinUVWCoords,
                                 const std::vector<int>& aNumElements,
                                 const std::vector<int>& aIndex);

}
