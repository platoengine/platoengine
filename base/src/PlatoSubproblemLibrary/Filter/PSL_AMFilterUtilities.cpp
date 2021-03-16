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

void AMFilterUtilities::computeGridBlueprintDensity(AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity, std::vector<double>& aGridBlueprintDensity) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();
    aGridBlueprintDensity.resize(tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2]);

    for(int i = 0; i < tGridDimensions[0]; ++i)
    {
        for(int j = 0; j < tGridDimensions[1]; ++j)
        {
            for(int k = 0; k < tGridDimensions[2]; ++k)
            {
                aGridBlueprintDensity[mGridUtilities.getSerializedIndex(i,j,k)] = computeGridPointBlueprintDensity(i,j,k,aTetMeshBlueprintDensity);
            }
        }
    }
}

void AMFilterUtilities::computeGridSupportDensity(const std::vector<double>& aGridBlueprintDensity, std::vector<double>& aGridSupportDensity) const
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
                        tSupportDensityBelow.push_back(aGridBlueprintDensity[mGridUtilities.getSerializedIndex(tSupportIndex)]);
                    }
                    aGridSupportDensity[mGridUtilities.getSerializedIndex(i,j,k)] = smax(tSupportDensityBelow,mPNorm);
                }
            }
        }
    }
}

double AMFilterUtilities::computeGridPointPrintableDensity(const int& i, const int& j, const int& k, const std::vector<double>& aGridBlueprintDensity, const std::vector<double>& aGridSupportDensity) const
{
    if(aGridSupportDensity.size() != mGridPointCoordinates.size())
        throw(std::domain_error("AMFilterUtilities: Grid support density vector does not match grid size"));

    if(aGridBlueprintDensity.size() != mGridPointCoordinates.size())
        throw(std::domain_error("AMFilterUtilities: Grid blueprint density vector does not match grid size"));

    double tSupportDensity = aGridSupportDensity[mGridUtilities.getSerializedIndex(i,j,k)];
    double tBlueprintDensity = aGridBlueprintDensity[mGridUtilities.getSerializedIndex(i,j,k)];

    return smin(tSupportDensity,tBlueprintDensity);
}

double AMFilterUtilities::computeGridPointPrintableDensity(const std::vector<int>& aIndex, const std::vector<double>& aGridBlueprintDensity, const std::vector<double>& aGridSupportDensity) const
{
    if(aIndex.size() != 3u)
        throw(std::domain_error("AMFilterUtilities: Grid point index must have 3 entries"));

    return computeGridPointPrintableDensity(aIndex[0],aIndex[1],aIndex[2],aGridBlueprintDensity,aGridSupportDensity);
}

void AMFilterUtilities::computeGridPrintableDensity(const std::vector<double>& aGridBlueprintDensity, const std::vector<double>& aGridSupportDensity, std::vector<double>& aGridPrintableDensity) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();

    aGridPrintableDensity.resize(tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2]);

    for(int i = 0; i < tGridDimensions[0]; ++i)
    {
        for(int j = 0; j < tGridDimensions[1]; ++j)
        {
            for(int k = 0; k < tGridDimensions[2]; ++k)
            {
                aGridPrintableDensity[mGridUtilities.getSerializedIndex(i,j,k)] = computeGridPointPrintableDensity(i,j,k,aGridBlueprintDensity,aGridSupportDensity);
            }
        }
    }
}

double AMFilterUtilities::computeTetNodePrintableDensity(const int& aTetNodeIndex,
                                                         const std::vector<double>& aGridPrintableDensity) const
{
    auto tCoordinates = mTetUtilities.getCoordinates();

    if(aTetNodeIndex < 0 || aTetNodeIndex >= (int) tCoordinates.size())
        throw(std::domain_error("AMFilterUtilities: Index must be between 0 and number of nodes on tet mesh"));

    std::vector<std::vector<int>> tContainingElementIndicies = mGridUtilities.getContainingGridElement(tCoordinates[aTetNodeIndex]);

    std::vector<double> tContainingElementDensities;

    for(auto tIndex : tContainingElementIndicies)
    {
        tContainingElementDensities.push_back(aGridPrintableDensity[mGridUtilities.getSerializedIndex(tIndex)]);
    }

    return mGridUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,Vector(tCoordinates[aTetNodeIndex]));;
}

void AMFilterUtilities::computeTetMeshPrintableDensity(const std::vector<double>& aGridPrintableDensity, AbstractInterface::ParallelVector* aDensity) const
{
    if(aDensity->get_length() != mTetUtilities.getCoordinates().size())
        throw(std::domain_error("AMFilterUtilities: Tet mesh density vector does not match the mesh size"));

    for(size_t i = 0; i < aDensity->get_length(); ++i)
    {
        aDensity->set_value(i, computeTetNodePrintableDensity(i, aGridPrintableDensity));
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
