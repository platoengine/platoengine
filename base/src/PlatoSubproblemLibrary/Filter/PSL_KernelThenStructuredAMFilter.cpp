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

void KernelThenStructuredAMFilter::internal_apply(AbstractInterface::ParallelVector* aBlueprintDensity)
{
    if(!mFilterBuilt)
        throw(std::runtime_error("Pseudo Layers not built before attempting to apply filter"));

    if(aBlueprintDensity->get_length() != mCoordinates.size())
        throw(std::domain_error("Provided density field does not match the mesh size"));

    // computePrintableDensity(aBlueprintDensity);
}

void KernelThenStructuredAMFilter::internal_gradient(AbstractInterface::ParallelVector* const aBlueprintDensity, AbstractInterface::ParallelVector* aGradient) const
{
    ;
}

void KernelThenStructuredAMFilter::buildStructuredGrid()
{
    mUtilities = std::unique_ptr<AMFilterUtilities>(new AMFilterUtilities(mCoordinates,mConnectivity,mUBasisVector,mVBasisVector,mBuildDirection));

    mUtilities->computeBoundingBox(mMaxUVWCoords,mMinUVWCoords);
    mTargetEdgeLength = mUtilities->computeMinEdgeLength()/2.0;
    mNumElementsInEachDirection = computeNumElementsInEachDirection(mMaxUVWCoords,mMinUVWCoords,mTargetEdgeLength);

    // mNumElementsInEachDirection = {5,5,5};

    // mUtilities->getTetIDForEachGridPoint(mNumElementsInEachDirection,mContainingTetID);

    mFilterBuilt = true;
}


// void KernelThenStructuredAMFilter::getTetIDForEachGridPoint(std::vector<int>& aTetIDs) const
// {
//     aTetIDs.resize((mNumElementsInEachDirection[0]+1) * (mNumElementsInEachDirection[1]+1) * (mNumElementsInEachDirection[2]+1));
//     for(int i = 0; i <= mNumElementsInEachDirection[0]; ++i)
//     {
//         for(int j = 0; j <= mNumElementsInEachDirection[1]; ++j)
//         {
//             for(int k = 0; k <= mNumElementsInEachDirection[2]; ++k)
//             {
//                 aTetIDs[getSerializedIndex(i,j,k)] = getContainingTetID(i,j,k);
//             }
//         }
//     }
// }

// int KernelThenStructuredAMFilter::getContainingTetID(const int& i, const int& j, const int& k) const
// {
//     std::vector<int> tIndex({i,j,k});
//     Vector tPoint = computeGridXYZCoordinates(mUBasisVector,mVBasisVector,mBuildDirection,mMaxUVWCoords,mMinUVWCoords,mNumElementsInEachDirection,tIndex);

//     for(int tTetIndex = 0; tTetIndex < (int) mConnectivity.size(); ++tTetIndex)
//     {
//         auto tTet = mConnectivity[tTetIndex];

//         if(mUtilities->isPointInTetrahedron(tTet,tPoint)) 
//             return tTetIndex;
//     }

//     return -1;
// }

// double KernelThenStructuredAMFilter::computeGridPointBlueprintDensity(const int& i, const int& j, const int&k, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const
// {
//     auto tTet = mConnectivity[mContainingTetID[getSerializedIndex(i,j,k)]];
//     std::vector<int> tIndex = {i,j,k};

//     Vector tGridPoint = computeGridXYZCoordinates(mUBasisVector,mVBasisVector,mBuildDirection,mMaxUVWCoords,mMinUVWCoords,mNumElementsInEachDirection,tIndex);
//     std::vector<double> tBaryCentricCoordinates = mUtilities->computeBarycentricCoordinates(tTet, tGridPoint);

//     double tGridPointDensity = 0;
//     for(int tNodeIndex = 0; tNodeIndex < (int) tTet.size(); ++tNodeIndex)
//     {
//        tGridPointDensity += tBaryCentricCoordinates[tNodeIndex]*(aTetMeshBlueprintDensity->get_value(tTet[tNodeIndex])); 
//     }

//     return tGridPointDensity;
// }

// double KernelThenStructuredAMFilter::computeGridSupportDensity(AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const
// {
//     // use grid point blueprint density to compute support density
//     // this has to be done bottom to top so it must be stored
//     return 0;
// }

// double KernelThenStructuredAMFilter::getGridPointSupportDensity(const int& i, const int& j, const int&k) const
// {
//     // use grid point blueprint density to compute support density
//     // this has to be done bottom to top so it must be stored
//     return 0;
// }

// double KernelThenStructuredAMFilter::computeGridPointPrintableDensity(const int& i const int& j, const int& k, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const
// {
//     // use grid point support density on points below to compute printable density
//     return 0;
// }

// double KernelThenStructuredAMFilter::computeTetNodePrintableDensity(const int& aTetNodeIndex, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const
// {
//     // get grid element that tet node belongs to
//     // use printable density at each grid point to compute tet node printable density
//     return 0;
// }

// void KernelThenStructuredAMFilter::computePrintableDensity(AbstractInterface::ParallelVector* const aBlueprintDensity)
// {
    // computeGridSupportDensity(aBlueprintDensity);

    // for(size_t i = 0; i < aBlueprintDensity->get_length(); ++i)
    // {
    //     aBlueprintDensity->set_value(i, computeTetNodePrintableDensity(i, aBlueprintDensity));
    // }
// }

}

