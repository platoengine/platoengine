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
                           // const double& aCriticalPrintAngle,
                           Vector aUBasisVector,
                           Vector aVBasisVector,
                           Vector aBuildDirection,
                           const std::vector<int>& aBaseLayer) 
            :mCoordinates(aCoordinates),
             mConnectivity(aConnectivity),
             // mCriticalPrintAngle(aCriticalPrintAngle),
             mUBasisVector(aUBasisVector),
             mVBasisVector(aVBasisVector),
             mBuildDirection(aBuildDirection),
             mBaseLayer(aBaseLayer)
        {
            checkInput();
            setBuildDirectionAndUVWBasis(aBuildDirection);
        }

        ~AMFilterUtilities(){}

        void getBoundingBox(Vector& tMaxUVWCoords, Vector& tMinUVWCoords) const;

    private:

        void checkInput() const;

        void setBuildDirectionAndUVWBasis(const PlatoSubproblemLibrary::Vector& aVector)
        {
            mBuildDirection = aVector;
            mBuildDirection.normalize();

            Vector tXUnit({1.0,0.0,0.0});
            Vector tYUnit({0.0,1.0,0.0});
            Vector tZUnit({0.0,0.0,1.0});

            if(fabs(dot_product(tXUnit,aVector)) < 0.8)
            {
                mUBasisVector = cross_product(mBuildDirection,tXUnit);
                mUBasisVector.normalize();
                mVBasisVector = cross_product(mBuildDirection,mUBasisVector);
                mVBasisVector.normalize();
            }
            else if(fabs(dot_product(tYUnit,aVector)) < 0.8)
            {
                mUBasisVector = cross_product(mBuildDirection,tYUnit);
                mUBasisVector.normalize();
                mVBasisVector = cross_product(mBuildDirection,mUBasisVector);
                mVBasisVector.normalize();
            }
            else if(fabs(dot_product(tZUnit,aVector)) < 0.8)
            {
                mUBasisVector = cross_product(mBuildDirection,tZUnit);
                mUBasisVector.normalize();
                mVBasisVector = cross_product(mBuildDirection,mUBasisVector);
                mVBasisVector.normalize();
            }
        }

    private:
        const std::vector<std::vector<double>>& mCoordinates;
        const std::vector<std::vector<int>>& mConnectivity;
        // const double& mCriticalPrintAngle;
        Vector mUBasisVector;
        Vector mVBasisVector;
        Vector mBuildDirection;
        const std::vector<int>& mBaseLayer;
};

}
