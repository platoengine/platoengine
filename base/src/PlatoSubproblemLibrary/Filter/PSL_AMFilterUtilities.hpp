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
                      const OrthogonalGridUtilities& aGridUtilities,
                      const double& aPNorm)
                    :mTetUtilities(aTetUtilities),
                     mGridUtilities(aGridUtilities),
                     mPNorm(aPNorm)
    {
        if(aPNorm < 1)
            throw(std::domain_error("AMFilterUtilities: P norm must be greater than 1"));

        mGridUtilities.computeGridXYZCoordinates(mGridPointCoordinates);
        mTetUtilities.getTetIDForEachPoint(mGridPointCoordinates,mContainingTetID);
    }

    double computeGridPointBlueprintDensity(const int& i, const int& j, const int&k, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const;
    double computeGridPointBlueprintDensity(const std::vector<int>& aIndex, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const;
    void computeGridSupportDensity(AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity, std::vector<double>& aGridSupportDensity) const;
    double computeGridPointPrintableDensity(const int& i, const int& j, const int& k, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity, const std::vector<double>& aGridSupportDensity) const;
    double computeGridPointPrintableDensity(const std::vector<int>& aIndex, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity, const std::vector<double>& aGridSupportDensity) const;


private:

    const TetMeshUtilities& mTetUtilities;
    const OrthogonalGridUtilities& mGridUtilities;
    const double& mPNorm;

    std::vector<int> mContainingTetID;
    std::vector<Vector> mGridPointCoordinates;
};

double smax(const std::vector<double>& aArguments, const double& aPNorm);
double smin(const double& aArg1, const double& aArg2, double aEps = std::numeric_limits<double>::epsilon());

}
