// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_KernelThenAMFilter.hpp"
#include "PSL_Abstract_ParallelVector.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Point.hpp"
#include "PSL_Vector.hpp"

namespace PlatoSubproblemLibrary
{

double KernelThenAMFilter::internal_apply(AbstractInterface::ParallelVector* const field, const int& i) const
{
  return field->get_value(i);
}

double KernelThenAMFilter::internal_gradient(AbstractInterface::ParallelVector* const field, const int& i) const
{
  return 1.0;
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

