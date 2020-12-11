// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_KernelThenAMFilter.hpp"
#include "PSL_FreeHelpers.hpp"


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

std::set<int> KernelThenAMFilter::getNeighbors(const int& aNodeID) const
{
    std::set<int> tNeighborIDs;

    for(auto tElement : mConnectivity)
    {
        for(size_t i = 0; i < tElement.size(); ++i)
        {
            if(tElement[i] == aNodeID)
            {
                tNeighborIDs.insert(tElement[(i+1)%4]);
                tNeighborIDs.insert(tElement[(i+2)%4]);
                tNeighborIDs.insert(tElement[(i+3)%4]);
            }
        }
    }

    return tNeighborIDs;
}

void KernelThenAMFilter::buildPseudoLayers()
{
    // we assume a positive z build direction for now.
    
    auto tNeighbors = getNeighbors(1);
    for(auto tNeighbor : tNeighbors)
    {
        std::cout << "Neighbor ID: " << tNeighbor << std::endl;
    }
}


}
