// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* Class: Kernel then AM filter for density method topology optimization.
*
* Smooth a field, and then apply an AM Filter to ensure printability of the materials.
*/

#include "PSL_Filter.hpp"
#include "PSL_AbstractKernelThenFilter.hpp"
#include "PSL_ParameterDataEnums.hpp"

#include <vector>
#include <set>
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

    void setCoordinates(const std::vector<std::vector<double>>& aCoordinates)
    {
        for(auto tNode : aCoordinates)
        {
            mCoordinates.push_back(tNode);
        }
    }

    void setConnectivity(const std::vector<std::vector<int>>& aConnectivity)
    {
        for(auto tElement : aConnectivity)
            mConnectivity.push_back(tElement);
    }

    void buildPseudoLayers();

private:

    double projection_apply(const double& beta, const double& input);
    double projection_gradient(const double& beta, const double& input);
    
    std::set<int> getNeighbors(const int& aNodeID) const; 

    std::vector<std::vector<double>> mCoordinates;
    std::vector<std::vector<int>> mConnectivity;
};

}
