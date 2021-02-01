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
                           Vector aBuildDirection)
            :mCoordinates(aCoordinates),
             mConnectivity(aConnectivity),
             mUBasisVector(aUBasisVector),
             mVBasisVector(aVBasisVector),
             mBuildDirection(aBuildDirection)
        {
            checkInput();
        }

        ~AMFilterUtilities(){}

        void computeBoundingBox(Vector& aMaxUVWCoords, Vector& aMinUVWCoords) const;

        double computeMinEdgeLength() const;

        bool isPointInTetrahedron(const std::vector<int>& aTet, const Vector& aPoint) const;

        std::vector<double> computeBarycentricCoordinates(const std::vector<int>& aTet, const Vector& aPoint) const;


    private:

        void checkInput() const;

        bool sameSide(const int& v1, const int& v2, const int& v3, const int& v4, const Vector& aPoint) const;

    private:
        const std::vector<std::vector<double>>& mCoordinates;
        const std::vector<std::vector<int>>& mConnectivity;
        Vector mUBasisVector;
        Vector mVBasisVector;
        Vector mBuildDirection;
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


double determinant3X3(const Vector& aRow1,
                      const Vector& aRow2,
                      const Vector& aRow3);

}
