#pragma once

#include "PSL_TetMeshUtilities.hpp"
#include "PSL_OrthogonalGridUtilities.hpp"

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{
class ParallelVector;
}

class AMFilterUtilities
{
public:
    AMFilterUtilities(const TetMeshUtilities& aTetUtilities,
                      const OrthogonalGridUtilities& aGridUtilities)
                    :mTetUtilities(aTetUtilities),
                     mGridUtilities(aGridUtilities)
    {
        mGridUtilities.computeGridXYZCoordinates(mGridPointCoordinates);
        mTetUtilities.getTetIDForEachPoint(mGridPointCoordinates,mContainingTetID);
    }

    // void computePrintableDensity(AbstractInterface::ParallelVector* aDensity) const;
    double computeGridPointBlueprintDensity(const int& i, const int& j, const int&k, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const;

private:

    const TetMeshUtilities& mTetUtilities;
    const OrthogonalGridUtilities& mGridUtilities;

    std::vector<int> mContainingTetID;
    std::vector<Vector> mGridPointCoordinates;
};


}
