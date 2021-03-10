#include <PSL_AMFilterUtilities.hpp>
#include "PSL_Abstract_ParallelVector.hpp"

namespace PlatoSubproblemLibrary
{

double AMFilterUtilities::computeGridPointBlueprintDensity(const int& i, const int& j, const int&k, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const
{
    auto tConnectivity = mTetUtilities.getConnectivity();

    auto tTet = tConnectivity[mContainingTetID[mGridUtilities.getSerializedIndex(i,j,k)]];
    Vector tGridPoint = mGridPointCoordinates[mGridUtilities.getSerializedIndex(i,j,k)];

    std::vector<double> tBaryCentricCoordinates = mTetUtilities.computeBarycentricCoordinates(tTet, tGridPoint);

    double tGridPointDensity = 0;
    for(int tNodeIndex = 0; tNodeIndex < (int) tTet.size(); ++tNodeIndex)
    {
       tGridPointDensity += tBaryCentricCoordinates[tNodeIndex]*(aTetMeshBlueprintDensity->get_value(tTet[tNodeIndex])); 
    }

    return tGridPointDensity;
}

// double KernelThenStructuredAMFilter::computeGridSupportDensity(AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity, std::vector<double>& aGridSupportDensity) const
// {
//     // use grid point blueprint density to compute grid support density
//     // this has to be done bottom to top so it must be stored
//     return 0;
// }

// double KernelThenStructuredAMFilter::getGridPointSupportDensity(const int& i, const int& j, const int&k) const
// {
// }

// double AMFilterUtilities::computeGridPointPrintableDensity(const int& i, const int& j, const int& k, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const
// {
//     // use grid point support density on points below to compute printable density
//     return 0;
// }

// double AMFilterUtilities::computeTetNodePrintableDensity(const int& aTetNodeIndex, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const

// {
//     // get grid element that tet node belongs to
//     // use printable density at each grid point to compute tet node printable density
//     return 0;
// }

// void AMFilterUtilities::computePrintableDensity(AbstractInterface::ParallelVector* aDensity)
// {
//     // compute and store grid support density

//     for(size_t i = 0; i < aDensity->get_length(); ++i)
//     {
//         aDensity->set_value(i, computeTetNodePrintableDensity(i, aDensity));
//     }
// }

}
