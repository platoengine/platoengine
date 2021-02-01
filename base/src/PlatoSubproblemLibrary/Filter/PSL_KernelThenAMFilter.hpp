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
#include "PSL_PseudoLayerBuilder.hpp"
#include "PSL_BoundarySupportPoint.hpp"


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
                            : AbstractKernelThenFilter(authority, data, points, exchanger),
                              mInputData(data){}

    virtual ~KernelThenAMFilter()
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
        setBuildDirection(aBuildDirection);
        setCriticalPrintAngle(aCriticalAngle);

        buildPseudoLayers();


        // mSmoothMaxPNorm =  mInputData->get_smooth_max_p_norm();
    }


private:

    void buildPseudoLayers();

    double internal_apply(AbstractInterface::ParallelVector* const aBlueprintDensity, const int& aNodeIndex);
    double internal_gradient(AbstractInterface::ParallelVector* const aBlueprintDensity, const int& aNodeIndex) const;

    void setCoordinates(const std::vector<std::vector<double>>& aCoordinates)
    {
        mCoordinates = aCoordinates;
        mPseudoLayers.resize(mCoordinates.size());
        mOrderedNodes.resize(mCoordinates.size());
        mBoundarySupportSet.resize(mCoordinates.size());
        mSupportDensity.resize(mCoordinates.size());
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

    void computeSupportDensity(AbstractInterface::ParallelVector* const aBlueprintDensity);

    std::vector<std::vector<double>> mCoordinates;
    std::vector<std::vector<int>> mConnectivity;

    std::vector<std::set<BoundarySupportPoint>> mBoundarySupportSet;

    std::map<std::pair<int,int>,std::vector<std::vector<double>>> mInteriorSupportPointCoefficients; // input is argument is node, element pair, 
                                                                                                     // output is vector of coefficients for each interior support point
                                                                                                     // coefficient order is determined by the element connectivity
    std::map<BoundarySupportPoint,std::vector<double>> mBoundarySupportCoefficients; // output is coefficients in the same order as second member of BoundarySupportPoint

    std::vector<int> mBaseLayer;

    std::vector<int> mPseudoLayers; // stores the pseudo layer id for each node
    std::vector<int> mOrderedNodes; // vector of nodes ordered in build direction
    std::vector<double> mSupportDensity;

    Vector mBuildDirection;
    double mCriticalPrintAngle;
    bool mSupportDensityHasBeenComputed = false;
    bool mFilterBuilt = false;

    double mSmoothMaxPNorm = 20.0;
    double mX0 = 0.5;

    ParameterData* mInputData;
};

}
