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
    }

    void init(const std::vector<std::vector<double>>& aCoordinates,
              const std::vector<std::vector<int>>&    aConnectivity,
              const std::vector<int>&                 aBaseLayer,
              const PlatoSubproblemLibrary::Vector&   aBuildDirection,
              const double                            aCriticalAngle)
    {
        setCoordinates(aCoordinates);
        setConnectivity(aConnectivity);
        setBaseLayer(aBaseLayer);
        setCriticalPrintAngle(aCriticalAngle);

        mBuildDirection = aBuildDirection;
        mBuildDirection.normalize();

        buildStructuredGrid();
    }


private:

    void buildStructuredGrid();

    double internal_apply(AbstractInterface::ParallelVector* const aBlueprintDensity, const int& aNodeIndex);
    double internal_gradient(AbstractInterface::ParallelVector* const aBlueprintDensity, const int& aNodeIndex) const;

    void setCoordinates(const std::vector<std::vector<double>>& aCoordinates)
    {
        mCoordinates = aCoordinates;
    }

    void setConnectivity(const std::vector<std::vector<int>>& aConnectivity)
    {
        mConnectivity = aConnectivity;
    }

    void setBaseLayer(const std::vector<int>& aBaseLayer)
    {
        mBaseLayer = aBaseLayer;
    }
    

    void setCriticalPrintAngle(const double aCriticalAngle)
    {
        mCriticalPrintAngle = aCriticalAngle;
    }

    void computeSupportDensity(AbstractInterface::ParallelVector* const aBlueprintDensity);

    std::vector<std::vector<double>> mCoordinates;
    std::vector<std::vector<int>> mConnectivity;

    std::vector<int> mBaseLayer;

    Vector mBuildDirection;

    double mCriticalPrintAngle;
    bool mSupportDensityHasBeenComputed = false;
    bool mFilterBuilt = false;

    ParameterData* mInputData;
};

}
