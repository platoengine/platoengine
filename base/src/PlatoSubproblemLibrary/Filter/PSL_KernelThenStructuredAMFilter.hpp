// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* Class: Kernel then AM filter for density method topology optimization.
*
* Smooth a field, and then apply an AM Filter to ensure printability of the materials.
*/

#include "PSL_Filter.hpp"
#include "PSL_AbstractKernelThenFilter.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Vector.hpp"
#include "PSL_Point.hpp"
#include "PSL_AMFilterUtilities.hpp"


#include <vector>
#include <set>
#include <map>
#include <memory>
#include <cstddef>
#include <cassert>
#include <iostream>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{
class PointCloud;
class ParallelVector;
class ParallelExchanger;
}
class ParameterData;
class AbstractAuthority;
class KernelFilter;

class KernelThenStructuredAMFilter : public AbstractKernelThenFilter
{
public:
    KernelThenStructuredAMFilter(AbstractAuthority* authority,
                              ParameterData* data,
                              AbstractInterface::PointCloud* points,
                              AbstractInterface::ParallelExchanger* exchanger)
                            : AbstractKernelThenFilter(authority, data, points, exchanger),
                              mInputData(data){}

    virtual ~KernelThenStructuredAMFilter()
    {
        mInputData = NULL;
        // delete mUtilities;
        // mUtilities = NULL;
    }

    void init(const std::vector<std::vector<double>>& aCoordinates,
              const std::vector<std::vector<int>>&    aConnectivity,
              const PlatoSubproblemLibrary::Vector&   aBuildDirection)
    {
        setCoordinates(aCoordinates);
        setConnectivity(aConnectivity);
        setBuildDirectionAndUVWBasis(aBuildDirection);

        mBuildDirection = aBuildDirection;
        mBuildDirection.normalize();

        buildStructuredGrid();
    }


private:

    void buildStructuredGrid();

    void internal_apply(AbstractInterface::ParallelVector* aBlueprintDensity);
    void internal_gradient(AbstractInterface::ParallelVector* const aBlueprintDensity, AbstractInterface::ParallelVector* aGradient) const;

    void setCoordinates(const std::vector<std::vector<double>>& aCoordinates)
    {
        mCoordinates = aCoordinates;
    }

    void setConnectivity(const std::vector<std::vector<int>>& aConnectivity)
    {
        mConnectivity = aConnectivity;
    }

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

    void computePrintableDensity(AbstractInterface::ParallelVector* const aBlueprintDensity);

    void getTetIDForEachGridPoint(std::vector<int>& aTetIDs) const;

    int getSerializedIndex(const int& i, const int& j, const int& k) const;

    int getContainingTetID(const int& i, const int& j, const int& k) const;

    std::vector<std::vector<double>> mCoordinates;
    std::vector<std::vector<int>> mConnectivity;

    std::vector<int> mNumElementsInEachDirection;
    std::vector<int> mContainingTetID;

    Vector mUBasisVector;
    Vector mVBasisVector;
    Vector mBuildDirection;

    Vector mMaxUVWCoords;
    Vector mMinUVWCoords;

    double mTargetEdgeLength;

    bool mSupportDensityHasBeenComputed = false;
    bool mFilterBuilt = false;

    ParameterData* mInputData;

    // AMFilterUtilities* mUtilities;
    std::unique_ptr<AMFilterUtilities> mUtilities;
};

}
