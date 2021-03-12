#include <PSL_AMFilterUtilities.hpp>
#include "PSL_Abstract_ParallelVector.hpp"

namespace PlatoSubproblemLibrary
{

double AMFilterUtilities::computeGridPointBlueprintDensity(const int& i, const int& j, const int&k, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const
{
    auto tConnectivity = mTetUtilities.getConnectivity();

    int tContainingTetID = mContainingTetID[mGridUtilities.getSerializedIndex(i,j,k)];
    
    if(tContainingTetID == -1)
        return 0;

    auto tTet = tConnectivity[tContainingTetID];
    Vector tGridPoint = mGridPointCoordinates[mGridUtilities.getSerializedIndex(i,j,k)];

    std::vector<double> tBaryCentricCoordinates = mTetUtilities.computeBarycentricCoordinates(tTet, tGridPoint);

    double tGridPointDensity = 0;
    for(int tNodeIndex = 0; tNodeIndex < (int) tTet.size(); ++tNodeIndex)
    {
       tGridPointDensity += tBaryCentricCoordinates[tNodeIndex]*(aTetMeshBlueprintDensity->get_value(tTet[tNodeIndex])); 
    }

    return tGridPointDensity;
}

double AMFilterUtilities::computeGridPointBlueprintDensity(const std::vector<int>& aIndex, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const
{
    if(aIndex.size() != 3u)
        throw(std::domain_error("AMFilterUtilities: Grid point index must have 3 entries"));

    return computeGridPointBlueprintDensity(aIndex[0], aIndex[1], aIndex[2], aTetMeshBlueprintDensity);
}

void AMFilterUtilities::computeGridSupportDensity(AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity, std::vector<double>& aGridSupportDensity) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();

    aGridSupportDensity.resize(tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2]);

    for(int i = 0; i < tGridDimensions[0]; ++i)
    {
        for(int j = 0; j < tGridDimensions[1]; ++j)
        {
            for(int k = 0; k < tGridDimensions[2]; ++k)
            {
                if(k == 0)
                {
                    aGridSupportDensity[mGridUtilities.getSerializedIndex(i,j,k)] = 1;
                }
                else
                {
                    auto tSupportIndices = mGridUtilities.getSupportIndices(i,j,k);
                    std::vector<double> tSupportDensityBelow;
                    for(auto tSupportIndex : tSupportIndices)
                    {
                        tSupportDensityBelow.push_back(computeGridPointBlueprintDensity(tSupportIndex,aTetMeshBlueprintDensity));
                    }
                    aGridSupportDensity[mGridUtilities.getSerializedIndex(i,j,k)] = smax(tSupportDensityBelow,mPNorm);
                }
            }
        }
    }
}

double AMFilterUtilities::computeGridPointPrintableDensity(const int& i, const int& j, const int& k, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity, const std::vector<double>& aGridSupportDensity) const
{
    if(aGridSupportDensity.size() != mGridPointCoordinates.size())
        throw(std::domain_error("AMFilterUtilities: Grid support density vector does not match grid size"));

    double tSupportDensity = aGridSupportDensity[mGridUtilities.getSerializedIndex(i,j,k)];
    double tBluePrintDensity = computeGridPointBlueprintDensity(i,j,k,aTetMeshBlueprintDensity);

    return smin(tSupportDensity,tBluePrintDensity);
}

double AMFilterUtilities::computeGridPointPrintableDensity(const std::vector<int>& aIndex, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity, const std::vector<double>& aGridSupportDensity) const
{
    if(aIndex.size() != 3u)
        throw(std::domain_error("AMFilterUtilities: Grid point index must have 3 entries"));

    return computeGridPointPrintableDensity(aIndex[0],aIndex[1],aIndex[2],aTetMeshBlueprintDensity,aGridSupportDensity);
}

double AMFilterUtilities::computeTetNodePrintableDensity(const int& aTetNodeIndex,
                                                         AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity,
                                                         const std::vector<double>& aGridSupportDensity) const
{
    auto tCoordinates = mTetUtilities.getCoordinates();
    std::vector<std::vector<int>> tGridIndicies = mGridUtilities.getContainingGridElement(tCoordinates[aTetNodeIndex]);
    // trilinear interpolation of density at grid points gives printable density
    return 0;
}

void AMFilterUtilities::computePrintableDensity(AbstractInterface::ParallelVector* aDensity) const
{
    std::vector<double> tGridSupportDensity;
    computeGridSupportDensity(aDensity,tGridSupportDensity);

    for(size_t i = 0; i < aDensity->get_length(); ++i)
    {
        aDensity->set_value(i, computeTetNodePrintableDensity(i, aDensity, tGridSupportDensity));
    }
}

double smax(const std::vector<double>& aArguments, const double& aPNorm)
{
    double tSmax = 0;
    double aQNorm = aPNorm + std::log(aArguments.size())/std::log(0.5);

    for(auto tArgument : aArguments)
    {
        tSmax += std::pow(std::abs(tArgument),aPNorm);
    }

    tSmax = std::pow(tSmax,1.0/aQNorm);

    return tSmax;
}

double smin(const double& aArg1, const double& aArg2, double aEps)
{
    return 0.5*(aArg1 + aArg2 - std::pow(std::pow((aArg1 - aArg2),2) + aEps,0.5) + std::sqrt(aEps));
}

}
