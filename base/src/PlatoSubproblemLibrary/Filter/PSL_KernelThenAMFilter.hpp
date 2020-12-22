// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* Class: Kernel then AM filter for density method topology optimization.
*
* Smooth a field, and then apply an AM Filter to ensure printability of the materials.
*/

#include "PSL_Filter.hpp"
#include "PSL_AbstractKernelThenFilter.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Vector.hpp"
#include "PSL_Point.hpp"
#include "PSL_PseudoLayerBuilder.hpp"

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

class KernelThenAMFilter : public AbstractKernelThenFilter
{
public:
    KernelThenAMFilter(AbstractAuthority* authority,
                              ParameterData* data,
                              AbstractInterface::PointCloud* points,
                              AbstractInterface::ParallelExchanger* exchanger)
                            : AbstractKernelThenFilter(authority, data, points, exchanger)
{
}

    virtual ~KernelThenAMFilter(){}

    void init(const std::vector<std::vector<double>>& aCoordinates,
              const std::vector<std::vector<int>>&    aConnectivity,
              const std::vector<int>&                 aBaseLayer,
              const PlatoSubproblemLibrary::Vector&   aBuildDirection,
              const double                            aCriticalAngle)
    {
        setCoordinates(aCoordinates);
        setConnectivity(aConnectivity);
        setBaseLayer(aBaseLayer);
        setBuildDirection(aBuildDirection);
        setCriticalPrintAngle(aCriticalAngle);
        buildPseudoLayers();
    }


private:

    void buildPseudoLayers();

    double projection_apply(const double& beta, const double& input);
    double projection_gradient(const double& beta, const double& input);

    void setCoordinates(const std::vector<std::vector<double>>& aCoordinates)
    {
        mCoordinates = aCoordinates;
        mPseudoLayers.resize(mCoordinates.size());
        mOrderedNodes.resize(mCoordinates.size());
        mSupportSet.resize(mCoordinates.size());
    }

    void setConnectivity(const std::vector<std::vector<int>>& aConnectivity)
    {
        mConnectivity = aConnectivity;
    }

    void setBaseLayer(const std::vector<int>& aBaseLayer)
    {
        mBaseLayer = aBaseLayer;
    }
    
    void setBuildDirection(const PlatoSubproblemLibrary::Vector& aVector)
    {
        mBuildDirection = aVector;
        mBuildDirection.normalize();
    }

    void setCriticalPrintAngle(const double aCriticalAngle)
    {
        mCriticalPrintAngle = aCriticalAngle;
    }

    std::vector<std::vector<double>> mCoordinates;
    std::vector<std::vector<int>> mConnectivity;

    std::vector<std::set<SupportPointData>> mSupportSet;
    std::map<SupportPointData,std::vector<double>> mSupportCoefficients;

    std::vector<int> mBaseLayer;

    std::vector<int> mPseudoLayers; // stores the pseudo layer id for each node
    std::vector<int> mOrderedNodes; // vector of nodes ordered in build direction

    Vector mBuildDirection;
    double mCriticalPrintAngle;
};

}
