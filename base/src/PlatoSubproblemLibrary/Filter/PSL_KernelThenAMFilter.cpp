// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_KernelThenAMFilter.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Point.hpp"
#include "PSL_Vector.hpp"


namespace PlatoSubproblemLibrary
{

double KernelThenAMFilter::projection_apply(const double& beta, const double& input)
{
  return AM_apply(beta,input);
}
double KernelThenAMFilter::projection_gradient(const double& beta, const double& input)
{
  return AM_gradient(beta,input);
}

void KernelThenAMFilter::buildPseudoLayers()
{
    if(!mInitialized)
        throw(std::runtime_error("AM filter not initialized"));

    orderNodesInBuildDirection();
    setBaseLayerIDToZeroAndOthersToMinusOne();
    computeNeighborsBelow();

    for(auto tNode : mOrderedNodes)
    {
        assignNodeToPseudoLayer(tNode);
    }
}

void KernelThenAMFilter::orderNodesInBuildDirection()
{
    for(int i = 0; i < (int) mCoordinates.size(); ++i)
    {
        mOrderedNodes[i] = i;
    }

    std::sort(mOrderedNodes.begin(), mOrderedNodes.end(), mLessThanFunctor);
}

void KernelThenAMFilter::setBaseLayerIDToZeroAndOthersToMinusOne()
{
    for(int i = 0; i < (int) mCoordinates.size(); ++i)
    {
        mPseudoLayers[i] = -1;
    }

    for(auto tNode : mBaseLayer)
    {
        mPseudoLayers[tNode] = 0;
    }

    ++mNumPseudoLayers;
}

void KernelThenAMFilter::assignNodeToPseudoLayer(const int& aNode)
{
    std::set<int> tNeighborIDs = mNeighborsBelow[aNode];

}

void KernelThenAMFilter::computeNeighborsBelow()
{
    for(auto tElement : mConnectivity)
    {
        addElementNeighborsBelow(tElement);
    }
}

void KernelThenAMFilter::addElementNeighborsBelow(const std::vector<int>& aElement)
{
    for(size_t i = 0; i < aElement.size(); ++i)
    {
        int tNode = aElement[i];
        for(size_t j = 0; j < aElement.size()-1; ++j)
        {
            int tNeighbor = aElement[(i+j)%aElement.size()];
            if(mLessThanFunctor(tNeighbor,tNode))
                mNeighborsBelow[tNode].insert(tNeighbor);
        }
    }
}

}
