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

    void computeGridBlueprintDensity(AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity, std::vector<double>& aGridBlueprintDensity) const;
    double computeGridPointBlueprintDensity(const int& i, const int& j, const int&k, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const;
    double computeGridPointBlueprintDensity(const std::vector<int>& aIndex, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const;
    void computeGridSupportDensity(const std::vector<double>& aGridBlueprintDensity, std::vector<double>& aGridSupportDensity) const;

    double computeGridPointPrintableDensity(const int& i, const int& j, const int& k, const std::vector<double>& aGridBlueprintDensity, const std::vector<double>& aGridSupportDensity) const;
    double computeGridPointPrintableDensity(const std::vector<int>& aIndex, const std::vector<double>& aGridBlueprintDensity, const std::vector<double>& aGridSupportDensity) const;

    void computeGridPrintableDensity(const std::vector<double>& tGridBlueprintDensity, const std::vector<double>& tGridSupportDensity, std::vector<double>& tGridPrintableDensity) const;

    void computeTetMeshPrintableDensity(const std::vector<double>& aGridPrintableDensity, AbstractInterface::ParallelVector* aDensity) const;
    double computeTetNodePrintableDensity(const int& aTetNodeIndex,
            const std::vector<double>& aGridPrintableDensity,
            AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const;


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
