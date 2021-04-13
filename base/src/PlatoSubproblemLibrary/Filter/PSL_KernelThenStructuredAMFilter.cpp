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

    const std::vector<std::vector<double>>& tCoordinates = mTetUtilities->getCoordinates();

    if(aDensity->get_length() != tCoordinates.size())
        throw(std::domain_error("Provided density field does not match the mesh size"));

    std::cout << "Compute Grid Blueprint Density" << std::endl;
    std::vector<double> tGridBlueprintDensity;
    mAMFilterUtilities->computeGridBlueprintDensity(aDensity,tGridBlueprintDensity); 

    // std::cout << "Grid Density: " << std::endl;

    // for(auto val : tGridBlueprintDensity)
    // {
    //     std::cout << " " << val;
    //     if(val < 0 - 1e-14 || val > 1 + 1e-14)
    //         throw(std::runtime_error("Bad value"));
    // }
    // std::cout << std::endl;

    // std::cout << "Compute Grid Support Density" << std::endl;
    // std::vector<double> tGridSupportDensity;
    // mAMFilterUtilities->computeGridSupportDensity(tGridBlueprintDensity,tGridSupportDensity);
    // std::cout << "Compute Grid Printable Density" << std::endl;
    // std::vector<double> tGridPrintableDensity;
    // mAMFilterUtilities->computeGridPrintableDensity(tGridBlueprintDensity,tGridSupportDensity,tGridPrintableDensity);
    // std::cout << "Compute Tet Mesh Printable Density" << std::endl;
    // mAMFilterUtilities->computeTetMeshPrintableDensity(tGridPrintableDensity,aDensity);

    // mGridUtilities->tempFunction(tTest);

    // std::cout << "Printable Densities: ";
    // for(auto value : tTest)
    //     std::cout << " " << value;
    // std::cout << std::endl;

    // std::cin.get();

    // std::fill(tTest.begin(),tTest.end(),1);
    //
    // std::vector<int> tGridDimensions = mGridUtilities->getGridDimensions();
   
    // std::vector<double> tTest(tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2]);
    // for(int i = 0; i < tGridDimensions[0]; ++i)
    // {
    //     for(int j = 0; j < tGridDimensions[1]; ++j)
    //     {
    //         for(int k = 0; k < tGridDimensions[2]; ++k)
    //         {
    //             // aGridBlueprintDensity[mGridUtilities.getSerializedIndex(i,j,k)] = computeGridPointBlueprintDensity(i,j,k,aTetMeshBlueprintDensity);
    //             tTest[mGridUtilities->getSerializedIndex(i,j,k)] = (double) k / (double) (tGridDimensions[2]-1);
    //         }
    //     }
    // }

    // mAMFilterUtilities->computeTetMeshPrintableDensity(tTest,aDensity);

    mAMFilterUtilities->computeTetMeshPrintableDensity(tGridBlueprintDensity,aDensity);
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

