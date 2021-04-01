// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_KernelThenStructuredAMFilter.hpp"
#include "PSL_Abstract_ParallelVector.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Point.hpp"
#include "PSL_Vector.hpp"
#include <iostream>
#include <memory>

namespace PlatoSubproblemLibrary
{

void KernelThenStructuredAMFilter::internal_apply(AbstractInterface::ParallelVector* aDensity)
{
    if(!mFilterBuilt)
        throw(std::runtime_error("KernelThenStructuredAMFilter: Filter not built before attempting to apply filter"));

    auto tCoordinates = mTetUtilities->getCoordinates();

    if(aDensity->get_length() != tCoordinates.size())
        throw(std::domain_error("Provided density field does not match the mesh size"));

    std::vector<double> tGridBlueprintDensity;
    mAMFilterUtilities->computeGridBlueprintDensity(aDensity,tGridBlueprintDensity); 
    std::vector<double> tGridSupportDensity;
    mAMFilterUtilities->computeGridSupportDensity(tGridBlueprintDensity,tGridSupportDensity);
    std::vector<double> tGridPrintableDensity;
    mAMFilterUtilities->computeGridPrintableDensity(tGridBlueprintDensity,tGridSupportDensity,tGridPrintableDensity);
    mAMFilterUtilities->computeTetMeshPrintableDensity(tGridPrintableDensity,aDensity);
}

void KernelThenStructuredAMFilter::internal_gradient(AbstractInterface::ParallelVector* const aBlueprintDensity, AbstractInterface::ParallelVector* aGradient) const
{
    //apply chain rule to 3 transformations - T2G, AMFilterGrid, G2T 
    //i.e. printableDensity = computeTetMeshPrintableDensity(computeGridPrintableDensity(computeGridSupportDensity(computeGridBlueprintDensity(aBlueprintDensity))))
}

void KernelThenStructuredAMFilter::buildStructuredGrid(const std::vector<std::vector<double>>& aCoordinates, const std::vector<std::vector<int>>& aConnectivity)
{
    mTetUtilities = std::unique_ptr<TetMeshUtilities>(new TetMeshUtilities(aCoordinates,aConnectivity));

    Vector aMaxUVWCoords, aMinUVWCoords;
    mTetUtilities->computeBoundingBox(mUBasisVector,mVBasisVector,mBuildDirection,aMaxUVWCoords,aMinUVWCoords);
    double aTargetEdgeLength = mTetUtilities->computeMinEdgeLength()/2.0;

    mGridUtilities = std::unique_ptr<OrthogonalGridUtilities>(new OrthogonalGridUtilities(mUBasisVector,mVBasisVector,mBuildDirection,aMaxUVWCoords,aMinUVWCoords,aTargetEdgeLength));

    double tPNorm = mInputData->get_smooth_max_p_norm();

    mAMFilterUtilities = std::unique_ptr<AMFilterUtilities>(new AMFilterUtilities( *(mTetUtilities.get()) , *(mGridUtilities.get()), tPNorm));

    mFilterBuilt = true;
}

}
