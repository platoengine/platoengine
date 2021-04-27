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
#include "PSL_TetMeshUtilities.hpp"
#include "PSL_OrthogonalGridUtilities.hpp"
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
        // delete mAMFilterUtilities;
        // mAMFilterUtilities = NULL;
        // delete mGridUtilities;
        // mGridUtilities = NULL;
        // delete mTetUtilities;
        // mTetUtilities = NULL;
    }

    void init(const std::vector<std::vector<double>>& aCoordinates,
              const std::vector<std::vector<int>>&    aConnectivity,
              const PlatoSubproblemLibrary::Vector&   aBuildDirection)
    {
        setBuildDirectionAndUVWBasis(aBuildDirection);
        buildStructuredGrid(aCoordinates, aConnectivity);
    }


private:

    void buildStructuredGrid(const std::vector<std::vector<double>>& aCoordinates, const std::vector<std::vector<int>>& aConnectivity);

    void internal_apply(AbstractInterface::ParallelVector* aDensity);
    void internal_gradient(AbstractInterface::ParallelVector* const aBlueprintDensity, AbstractInterface::ParallelVector* aGradient) const;

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

    Vector mUBasisVector;
    Vector mVBasisVector;
    Vector mBuildDirection;

    bool mFilterBuilt = false;

    ParameterData* mInputData;

    std::unique_ptr<TetMeshUtilities> mTetUtilities;
    std::unique_ptr<OrthogonalGridUtilities> mGridUtilities;
    std::unique_ptr<AMFilterUtilities> mAMFilterUtilities;
    // TetMeshUtilities* mTetUtilities;
    // OrthogonalGridUtilities* mGridUtilities;
    // AMFilterUtilities* mAMFilterUtilities;
};

}
