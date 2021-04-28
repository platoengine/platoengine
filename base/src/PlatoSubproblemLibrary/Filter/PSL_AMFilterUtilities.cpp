#include <PSL_AMFilterUtilities.hpp>
#include "PSL_Abstract_ParallelVector.hpp"

namespace PlatoSubproblemLibrary
{

double AMFilterUtilities::computeGridPointBlueprintDensity(const int& i, const int& j, const int&k, AbstractInterface::ParallelVector* const aTetMeshBlueprintDensity) const
{


    const std::vector<std::vector<int>>& tConnectivity = mTetUtilities.getConnectivity();

    int tContainingTetID = mContainingTetID[mGridUtilities.getSerializedIndex(i,j,k)];
    
    if(tContainingTetID == -1)
        return 0;

    auto tTet = tConnectivity[tContainingTetID];
    Vector tGridPoint = mGridPointCoordinates[mGridUtilities.getSerializedIndex(i,j,k)];

    std::vector<double> tBaryCentricCoordinates = mTetUtilities.computeBarycentricCoordinates(tTet, tGridPoint);

    if(tBaryCentricCoordinates.size() != 4)
        throw(std::runtime_error("Incorrect barycentric coordinates"));

    for(auto tCoordinate : tBaryCentricCoordinates)
        if(tCoordinate > 1 + 1e-14 || tCoordinate < 0 - 1e-14)
            throw(std::runtime_error("Grid point outside of TET"));

    double tGridPointDensity = 0;
    for(int tNodeIndex = 0; tNodeIndex < (int) tTet.size(); ++tNodeIndex)
    {
       tGridPointDensity += tBaryCentricCoordinates[tNodeIndex]*(aTetMeshBlueprintDensity->get_value(tTet[tNodeIndex])); 
    }

    if(tGridPointDensity < 0)
        tGridPointDensity = 0;

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

    for(size_t i = 0; i < tGridDimensions[0]; ++i)
    {
        for(size_t j = 0; j < tGridDimensions[1]; ++j)
        {
            for(size_t k = 0; k < tGridDimensions[2]; ++k)
            {
                aGridBlueprintDensity[mGridUtilities.getSerializedIndex(i,j,k)] = computeGridPointBlueprintDensity(i,j,k,aTetMeshBlueprintDensity);
            }
        }
    }
}

void AMFilterUtilities::computeGridLayerSupportDensity(const int& k,
                                                       const std::vector<double>& aGridPrintableDensity,
                                                       std::vector<double>& aGridSupportDensity) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();
    size_t tGridSize = tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2];

    if(aGridPrintableDensity.size() != tGridSize || aGridSupportDensity.size() != tGridSize)
        throw(std::domain_error("AMFilterUtilities::computeGridLayerSupportDensity: Density vectors do not match grid size"));

    for(size_t i = 0; i < tGridDimensions[0]; ++i)
    {
        for(size_t j = 0; j < tGridDimensions[1]; ++j)
        {
            if(k == 0)
            {
                aGridSupportDensity[mGridUtilities.getSerializedIndex(i,j,k)] = 1.0;
            }
            else
            {
                auto tSupportIndices = mGridUtilities.getSupportIndices(i,j,k);
                std::vector<double> tSupportDensityBelow;
                for(auto tSupportIndex : tSupportIndices)
                {
                    tSupportDensityBelow.push_back(aGridPrintableDensity[mGridUtilities.getSerializedIndex(tSupportIndex)]);
                }

                double tVal = smax(tSupportDensityBelow,mPNorm);

                aGridSupportDensity[mGridUtilities.getSerializedIndex(i,j,k)] = tVal;
            }
        }
    }
}

void AMFilterUtilities::computeGridLayerPrintableDensity(const int& k,
                                                         const std::vector<double>& aGridBlueprintDensity,
                                                         const std::vector<double>& aGridSupportDensity,
                                                         std::vector<double>& aGridPrintableDensity) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();
    size_t tGridSize = tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2];

    if(aGridBlueprintDensity.size() != tGridSize || aGridPrintableDensity.size() != tGridSize || aGridSupportDensity.size() != tGridSize)
        throw(std::domain_error("AMFilterUtilities::computeGridLayerPrintableDensity: Density vectors do not match grid size"));

    for(size_t i = 0; i < tGridDimensions[0]; ++i)
    {
        for(size_t j = 0; j < tGridDimensions[1]; ++j)
        {
            size_t tSerializedIndex = mGridUtilities.getSerializedIndex(i,j,k);
            aGridPrintableDensity[tSerializedIndex] = smin(aGridBlueprintDensity[tSerializedIndex],aGridSupportDensity[tSerializedIndex]);
        }
    }
}

void AMFilterUtilities::computeGridPrintableDensity(const std::vector<double>& aGridBlueprintDensity, std::vector<double>& aGridPrintableDensity) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();

    aGridPrintableDensity.resize(tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2]);

    std::vector<double> tGridSupportDensity(aGridPrintableDensity.size());
    for(size_t k = 0; k < tGridDimensions[2]; ++k)
    {
        computeGridLayerSupportDensity(k,aGridPrintableDensity,tGridSupportDensity);
        computeGridLayerPrintableDensity(k,aGridBlueprintDensity,tGridSupportDensity,aGridPrintableDensity);
    }
}

double AMFilterUtilities::computeTetNodePrintableDensity(const int& aTetNodeIndex,
                                                         const std::vector<double>& aGridPrintableDensity) const
{
    const std::vector<std::vector<double>>& tCoordinates = mTetUtilities.getCoordinates();

    if(aTetNodeIndex < 0 || aTetNodeIndex >= (int) tCoordinates.size())
        throw(std::out_of_range("AMFilterUtilities: Index must be between 0 and number of nodes on tet mesh"));
    if(aGridPrintableDensity.size() != mGridPointCoordinates.size())
        throw(std::domain_error("AMFilterUtilities: Provided grid density vector does not match grid size"));

    std::vector<std::vector<size_t>> tContainingElementIndicies = mGridUtilities.getContainingGridElement(tCoordinates[aTetNodeIndex]);

    std::vector<double> tContainingElementDensities;

    for(auto tIndex : tContainingElementIndicies)
    {
        tContainingElementDensities.push_back(aGridPrintableDensity[mGridUtilities.getSerializedIndex(tIndex)]);
    }

    double tVal = mGridUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,Vector(tCoordinates[aTetNodeIndex]));

    return tVal;
}

void AMFilterUtilities::computeTetMeshPrintableDensity(const std::vector<double>& aGridPrintableDensity, AbstractInterface::ParallelVector* aDensity) const
{
    const std::vector<std::vector<double>>& tCoordinates = mTetUtilities.getCoordinates();
    if(aDensity->get_length() != tCoordinates.size())
        throw(std::domain_error("AMFilterUtilities: Tet mesh density vector does not match the mesh size"));

    for(size_t i = 0; i < aDensity->get_length(); ++i)
    {
        double tVal = computeTetNodePrintableDensity(i, aGridPrintableDensity);
        aDensity->set_value(i, tVal);
    }
}

double smax(const std::vector<double>& aArguments, const double& aPNorm)
{
    double tSmax = 0;
    double aQNorm = aPNorm + std::log(aArguments.size())/std::log(0.5);

    for(auto tArgument : aArguments)
    {
        if(tArgument < 0)
            throw(std::domain_error("AMFilterUtilities: Smooth max arguments must be positive"));
        tSmax += std::pow(std::abs(tArgument),aPNorm);
    }

    tSmax = std::pow(tSmax,1.0/aQNorm);

    return tSmax;
}

double smin(const double& aArg1, const double& aArg2, double aEps)
{
    double tVal = 0.5*(aArg1 + aArg2 - std::pow(std::pow((aArg1 - aArg2),2) + aEps,0.5) + std::sqrt(aEps));

    return tVal;
}

}
