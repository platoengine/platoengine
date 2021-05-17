#pragma once

#include "PSL_TetMeshUtilities.hpp"
#include "PSL_OrthogonalGridUtilities.hpp"

// This class implements the AM filter functions from the 2016 paper
// "An additive manufacturing filter for topology optimization of print-ready designs"
// by Matthajs Langelaar along with functions to transfer topology between 
// a tetrahedral mesh onto a regular grid as well as corresponding gradients

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
                      double aPNorm)
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

    void computeGridLayerSupportDensity(const size_t& k,
                                        const std::vector<double>& aGridPrintableDensity,
                                        std::vector<double>& aGridSupportDensity) const;

    double computeGridPointSupportDensity(const size_t& i,
                                          const size_t& j,
                                          const size_t& k,
                                          const std::vector<double>& aGridPrintableDensity) const;

    void computeGridLayerPrintableDensity(const size_t& k,
                                          const std::vector<double>& aGridBlueprintDensity,
                                          const std::vector<double>& aGridSupportDensity,
                                          std::vector<double>& aGridPrintableDensity) const;

    void computeGridPrintableDensity(const std::vector<double>& aGridBlueprintDensity, std::vector<double>& aGridPrintableDensity) const;

    void computeTetMeshPrintableDensity(const std::vector<double>& aGridPrintableDensity, AbstractInterface::ParallelVector* aDensity) const;

    void postMultiplyTetMeshPrintableDensityGradient(AbstractInterface::ParallelVector* const aGradient,
                                                     std::vector<double>& aGridGradient) const;

    void postMultiplyGridPrintableDensityGradient(const std::vector<double>& tGridBlueprintDensity, std::vector<double>& tGridGradient) const;

    void postMultiplyGridBlueprintDensityGradient(const std::vector<double>& aInputGridGradient,
                                                  AbstractInterface::ParallelVector* aOutputGradient) const;

    double computeTetNodePrintableDensity(const int& aTetNodeIndex,
                                          const std::vector<double>& aGridPrintableDensity) const;

    const std::vector<Vector>& getGridPointCoordinates() const {return mGridPointCoordinates;}

    void computeLambda(const std::vector<double>& aGridBlueprintDensity,
                       const std::vector<double>& aGridPrintableDensity,
                       const std::vector<double>& aGridGradient,
                       const size_t aLayerIndex,
                       std::vector<double>& aLambda) const;

    void postMultiplyLambdaByGradientWRTPreviousLayerPrintableDensity(const std::vector<double>& aGridBlueprintDensity,
                                                                      const std::vector<double>& aGridPrintableDensity,
                                                                      const size_t aLayerIndex,
                                                                      std::vector<double>& aLambda) const;

    void postMultiplyLambdaByGradientWRTCurrentLayerBlueprintDensity(const std::vector<double>& aGridBlueprintDensity,
                                                                     const std::vector<double>& aGridPrintableDensity,
                                                                     const size_t& tLayerIndex,
                                                                     const std::vector<double>& aLambda,
                                                                     std::vector<double>& aGridGradient) const;

private:

    const TetMeshUtilities& mTetUtilities;
    const OrthogonalGridUtilities& mGridUtilities;
    const double mPNorm;

    std::vector<int> mContainingTetID;
    std::vector<Vector> mGridPointCoordinates;

    double mX0 = 0.5;
};

double smax(const std::vector<double>& aArguments, const double& aPNorm, const double& aX0);
void smax_gradient(const std::vector<double>& aArguments, const double& aPNorm, const double& aX0, std::vector<double>& aGradient);
double smin(const double& aArg1, const double& aArg2, double aEps = std::numeric_limits<double>::epsilon());
double smin_gradient1(const double& aArg1, const double& aArg2, double aEps = std::numeric_limits<double>::epsilon());
double smin_gradient2(const double& aArg1, const double& aArg2, double aEps = std::numeric_limits<double>::epsilon());

}
