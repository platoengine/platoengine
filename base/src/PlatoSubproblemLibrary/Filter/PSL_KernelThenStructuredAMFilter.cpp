// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_KernelThenStructuredAMFilter.hpp"
#include "PSL_Abstract_ParallelVector.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Point.hpp"
#include "PSL_Vector.hpp"
#include <iostream>

namespace PlatoSubproblemLibrary
{

double KernelThenStructuredAMFilter::internal_apply(AbstractInterface::ParallelVector* const aBlueprintDensity, const int& aNodeIndex)
{
    if(!mFilterBuilt)
        throw(std::runtime_error("Pseudo Layers not built before attempting to apply filter"));

    if(aBlueprintDensity->get_length() != mCoordinates.size())
        throw(std::domain_error("Provided density field does not match the mesh size"));

    // if(!mSupportDensityHasBeenComputed)
    // {
    //     computeSupportDensity(aBlueprintDensity);
    //     mSupportDensityHasBeenComputed = true;
    // }

    return aBlueprintDensity->get_value(aNodeIndex);
}

double KernelThenStructuredAMFilter::internal_gradient(AbstractInterface::ParallelVector* const aBlueprintDensity, const int& aNodeIndex) const
{
    return 1.0;
}

void KernelThenStructuredAMFilter::buildStructuredGrid()
{
    // Vector tMaxUVWCoords, tMinUVWCoords;

    // AMFilterUtilities tUtilities(mCoordinates,mConnectivity,mCriticalPrintAngle,mBuildDirection,mBaseLayer);

    // tUtilities.getBoundingBox(tMaxUVWCoords, tMinUVWCoords);

}

// void KernelThenStructuredAMFilter::computeSupportDensity(AbstractInterface::ParallelVector* const aBlueprintDensity)
// {
//     for(auto tNode : mBaseLayer)
//     {
//         mSupportDensity[tNode] = 1.0;
//     }

//     for(auto tNode : mOrderedNodes)
//     {
//         std::set<BoundarySupportPoint> tBoundarySupportPointDataSet = mBoundarySupportSet[tNode];

//         double tSupportDensity = 0;

//         if(tBoundarySupportPointDataSet.size() == 0)
//         {
//             if(mPseudoLayers[tNode] != 0)
//                 throw(std::runtime_error("only nodes on the base layer should have no support points"));
//             continue;
//         }

//         for(auto tSupportPointData : tBoundarySupportPointDataSet)
//         {
//             std::set<int> tSupportingNodes = tSupportPointData.getSupportingNodeIndices();
//             std::vector<double> tBoundarySupportCoefficients = mBoundarySupportCoefficients[tSupportPointData];
//             double tSupportPointDensity = 0;

//             auto tCoefficientIterator = tBoundarySupportCoefficients.begin();

//             for(auto tSupportNode : tSupportingNodes)
//             {
//                 tSupportPointDensity += aBlueprintDensity->get_value(tSupportNode) * (*tCoefficientIterator);
//                 ++tCoefficientIterator;
//             }

//             tSupportDensity += std::pow(tSupportPointDensity, mSmoothMaxPNorm);
//         }

//         // double tSmoothMaxQ = mSmoothMaxPNorm + (std::log((double) tSupportPointDataSet.size()) / std::log(mX0));
//         double tSmoothMaxQ = mSmoothMaxPNorm;

//         tSupportDensity = std::pow(tSupportDensity, 1.0/tSmoothMaxQ);

//         mSupportDensity[tNode] = tSupportDensity;
//     }

// }

}

