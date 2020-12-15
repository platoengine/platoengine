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
                            : AbstractKernelThenFilter(authority, data, points, exchanger),
                              mInitialized(false),
                              mLessThanFunctor(*this)
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

        mInitialized = true;
    }

    void buildPseudoLayers();

private:

    double projection_apply(const double& beta, const double& input);
    double projection_gradient(const double& beta, const double& input);

    void orderNodesInBuildDirection();
    void setBaseLayerIDToZeroAndOthersToMinusOne();
    void computeNeighborsBelow();
    void addElementNeighborsBelow(const std::vector<int>& aElement);

    void assignNodeToPseudoLayer(const int& aNode);
    std::set<int> getNeighborsBelow(const int& aNodeID) const; 

    void setCoordinates(const std::vector<std::vector<double>>& aCoordinates)
    {
        mCoordinates.clear();

        for(auto tNode : aCoordinates)
        {
            mCoordinates.push_back(tNode);
        }

        mPseudoLayers.resize(mCoordinates.size());
        mOrderedNodes.resize(mCoordinates.size());
        mNeighborsBelow.resize(mCoordinates.size());
    }

    void setConnectivity(const std::vector<std::vector<int>>& aConnectivity)
    {
        mConnectivity.clear();

        for(auto tElement : aConnectivity)
            mConnectivity.push_back(tElement);
    }

    void setBaseLayer(const std::vector<int>& aBaseLayer)
    {
        mBaseLayer.clear();

        for(auto tNode : aBaseLayer)
            mBaseLayer.push_back(tNode);
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

    std::vector<std::set<int>> mNeighborsBelow;

    std::vector<int> mBaseLayer;

    std::vector<int> mPseudoLayers; // stores the pseudo layer id for each node
    std::vector<int> mOrderedNodes; // vector of nodes ordered in build direction

    int mNumPseudoLayers = 0;

    Vector mBuildDirection;
    double mCriticalPrintAngle;

    bool mInitialized;

    // functor to sort in build direction
    class LessThanInBuildDirection
    {
        public:
            LessThanInBuildDirection(KernelThenAMFilter& aFilter) :mFilter(aFilter) {};
            ~LessThanInBuildDirection(){}

            bool operator()(int aIndex1, int aIndex2) const
            {
                if(aIndex1 > (int) mFilter.mCoordinates.size() || aIndex2 > (int) mFilter.mCoordinates.size())
                {
                    throw(std::out_of_range("Indices must be betweeen zero and number of nodes in mesh to compare distance of node in build direction"));
                }

                Vector tVec1(mFilter.mCoordinates[aIndex1]);
                Vector tVec2(mFilter.mCoordinates[aIndex2]);

                double tSignedDistanceFromOrigin1 = PlatoSubproblemLibrary::dot_product(tVec1,mFilter.mBuildDirection);
                double tSignedDistanceFromOrigin2 = PlatoSubproblemLibrary::dot_product(tVec2,mFilter.mBuildDirection);

                return tSignedDistanceFromOrigin1 < tSignedDistanceFromOrigin2;
            }

        private:

            KernelThenAMFilter& mFilter;
    };

    LessThanInBuildDirection mLessThanFunctor;

};

}
