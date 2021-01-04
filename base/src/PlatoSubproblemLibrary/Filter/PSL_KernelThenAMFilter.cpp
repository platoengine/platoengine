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
    PseudoLayerBuilder tBuilder(mCoordinates, mConnectivity, mCriticalPrintAngle, mBuildDirection, mBaseLayer);

    mOrderedNodes = tBuilder.orderNodesInBuildDirection();
    mPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();
    tBuilder.computeSupportSetAndCoefficients(mSupportSet, mSupportCoefficients);

    for(auto tNode : mOrderedNodes)
    {
        mPseudoLayers[tNode] = tBuilder.assignNodeToPseudoLayer(tNode, mPseudoLayers, mSupportSet[tNode]);
        mSupportSet[tNode] = tBuilder.pruneSupportSet(tNode, mPseudoLayers, mSupportSet[tNode], mSupportCoefficients);
    }
}

}

