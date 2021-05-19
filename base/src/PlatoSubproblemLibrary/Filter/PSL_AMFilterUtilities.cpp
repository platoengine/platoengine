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

void AMFilterUtilities::computeGridLayerSupportDensity(const size_t& k,
                                                       const std::vector<double>& aGridPrintableDensity,
                                                       std::vector<double>& aGridSupportDensity) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();
    size_t tGridSize = tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2];

    if(aGridPrintableDensity.size() != tGridSize || aGridSupportDensity.size() != tGridSize)
        throw(std::domain_error("AMFilterUtilities::computeGridLayerSupportDensity: Density vectors do not match grid size"));

    for(size_t i = 0; i < tGridDimensions[0]; ++i)
        for(size_t j = 0; j < tGridDimensions[1]; ++j)
            aGridSupportDensity[mGridUtilities.getSerializedIndex(i,j,k)] = computeGridPointSupportDensity(i,j,k,aGridPrintableDensity);
}

double AMFilterUtilities::computeGridPointSupportDensity(const size_t& i,
                                                         const size_t& j,
                                                         const size_t& k,
                                                         const std::vector<double>& aGridPrintableDensity) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();
    size_t tGridSize = tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2];

    if(aGridPrintableDensity.size() != tGridSize)
        throw(std::domain_error("AMFilterUtilities::computeGridPointSupportDensity: Density vector does not match grid size"));

    if(i >= tGridDimensions[0] || j >= tGridDimensions[1] || k >= tGridDimensions[2])
        throw(std::out_of_range("AMFilterUtilities::computeGridPointSupportDensity: Index out of range"));

    if(k == 0)
        return 1.0;
    else
    {
        auto tSupportIndices = mGridUtilities.getSupportIndices(i,j,k);
        std::vector<double> tSupportDensityBelow;
        for(auto tSupportIndex : tSupportIndices)
        {
            tSupportDensityBelow.push_back(aGridPrintableDensity[mGridUtilities.getSerializedIndex(tSupportIndex)]);
        }

        return smax(tSupportDensityBelow,mPNorm);
    }
}

void AMFilterUtilities::computeGridLayerPrintableDensity(const size_t& k,
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

void AMFilterUtilities::postMultiplyTetMeshPrintableDensityGradient(AbstractInterface::ParallelVector* const aInputGradient,
                                                                    std::vector<double>& aGridGradient) const
{
    const std::vector<std::vector<double>>& tCoordinates = mTetUtilities.getCoordinates();
    auto tGridDimensions = mGridUtilities.getGridDimensions();
    size_t tNumGridElements = tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2];

    aGridGradient.resize(tNumGridElements);
    std::fill(aGridGradient.begin(), aGridGradient.end(), 0.0);

    if(aInputGradient->get_length() != tCoordinates.size())
        throw(std::domain_error("AMFilterUtilities::postMultiplyTetMeshPrintableDensityGradient: Input gradient does not match tet mesh size"));

    for(size_t tTetIndex = 0u; tTetIndex < tCoordinates.size(); ++tTetIndex)
    {
        Vector tPoint(tCoordinates[tTetIndex]);
        std::vector<std::vector<size_t>> tContainingGridElement;
        std::vector<double> tLocalGradientValues;
        mGridUtilities.computeGradientOfDensityWRTGridDensity(tPoint,tLocalGradientValues,tContainingGridElement);

        for(size_t tLocalIndex = 0u; tLocalIndex < 8u; ++tLocalIndex)
        {
            auto tIndex = tContainingGridElement[tLocalIndex];
            aGridGradient[mGridUtilities.getSerializedIndex(tIndex)] += aInputGradient->get_value(tTetIndex)*tLocalGradientValues[tLocalIndex];
        }
    }
}

void AMFilterUtilities::postMultiplyGridPrintableDensityGradient(const std::vector<double>& aGridBlueprintDensity, std::vector<double>& aGridGradient) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();
    size_t tNumGridElements = tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2];

    if(aGridBlueprintDensity.size() != tNumGridElements)
        throw(std::domain_error("AMFilterUtilities::postMultiplyGridPrintableDensityGradient: Input density vector does not match grid size"));
    if(aGridGradient.size() != tNumGridElements)
        throw(std::domain_error("AMFilterUtilities::postMultiplyGridPrintableDensityGradient: Input gradient does not match grid size"));

    std::vector<double> aGridPrintableDensity;
    computeGridPrintableDensity(aGridBlueprintDensity,aGridPrintableDensity);

    std::vector<double> tLambda;

    for(size_t tAuxiliaryIndex = 1; tAuxiliaryIndex <= tGridDimensions[2]; ++tAuxiliaryIndex)
    {
        size_t tLayerIndex = tGridDimensions[2] - tAuxiliaryIndex;

        computeLambda(aGridBlueprintDensity,aGridPrintableDensity,aGridGradient,tLayerIndex,tLambda);
        postMultiplyLambdaByGradientWRTCurrentLayerBlueprintDensity(aGridBlueprintDensity,aGridPrintableDensity,tLayerIndex,tLambda,aGridGradient);
    }
}

void AMFilterUtilities::postMultiplyLambdaByGradientWRTCurrentLayerBlueprintDensity(const std::vector<double>& aGridBlueprintDensity,
                                                                                    const std::vector<double>& aGridPrintableDensity,
                                                                                    const size_t& tLayerIndex,
                                                                                    const std::vector<double>& aLambda,
                                                                                    std::vector<double>& aGridGradient) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();
    size_t tNumGridElements = tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2];

    if(aGridBlueprintDensity.size() != tNumGridElements)
        throw(std::domain_error("AMFilterUtilities::postMultiplyLambdaByGradientWRTCurrentLayerBlueprintDensity: Input density vector does not match grid size"));
    if(aGridPrintableDensity.size() != tNumGridElements)
        throw(std::domain_error("AMFilterUtilities::postMultiplyLambdaByGradientWRTCurrentLayerBlueprintDensity: Input density vector does not match grid size"));
    if(aGridGradient.size() != tNumGridElements)
        throw(std::domain_error("AMFilterUtilities::postMultiplyLambdaByGradientWRTCurrentLayerBlueprintDensity: Input gradient does not match grid size"));
    if(aLambda.size() != tGridDimensions[0]*tGridDimensions[1])
        throw(std::domain_error("AMFilterUtilities::postMultiplyLambdaByGradientWRTCurrentLayerBlueprintDensity: Input lagrange multiplier does not match grid layer size"));
    if(tLayerIndex >= tGridDimensions[2])
        throw(std::out_of_range("AMFilterUtilities::postMultiplyLambdaByGradientWRTCurrentLayerBlueprintDensity: Input layer index out of range"));

    for(size_t i = 0; i < tGridDimensions[0]; ++i)
    {
        for(size_t j = 0; j < tGridDimensions[1]; ++j)
        {
            double tSupportDensity = computeGridPointSupportDensity(i,j,tLayerIndex,aGridPrintableDensity);
            double tSminGradientWRTBlueprintDensity = smin_gradient1(aGridBlueprintDensity[mGridUtilities.getSerializedIndex(i,j,tLayerIndex)], tSupportDensity);
            aGridGradient[mGridUtilities.getSerializedIndex(i,j,tLayerIndex)] = aLambda[mGridUtilities.getSerializedIndexWithinLayer(i,j)]*tSminGradientWRTBlueprintDensity;
        }
    }
}

void AMFilterUtilities::computeLambda(const std::vector<double>& aGridBlueprintDensity,
                                      const std::vector<double>& aGridPrintableDensity,
                                      const std::vector<double>& aGridGradient,
                                      const size_t aLayerIndex,
                                      std::vector<double>& aLambda) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();
    size_t tNumGridElements = tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2];

    if(aGridBlueprintDensity.size() != tNumGridElements)
        throw(std::domain_error("AMFilterUtilities::computeLambda: Input density vector does not match grid size"));
    if(aGridPrintableDensity.size() != tNumGridElements)
        throw(std::domain_error("AMFilterUtilities::computeLambda: Input density vector does not match grid size"));
    if(aGridGradient.size() != tNumGridElements)
        throw(std::domain_error("AMFilterUtilities::computeLambda: Input gradient does not match grid size"));
    if(aLayerIndex >= tGridDimensions[2])
        throw(std::out_of_range("AMFilterUtilities::computeLambda: Grid layer index out of range"));


    if(aLayerIndex == tGridDimensions[2] - 1)
    {
        aLambda.resize(tGridDimensions[0]*tGridDimensions[1]);
        for(size_t i = 0; i < tGridDimensions[0]; ++i)
            for(size_t j = 0; j < tGridDimensions[1]; ++j)
                aLambda[mGridUtilities.getSerializedIndexWithinLayer(i,j)] = aGridGradient[mGridUtilities.getSerializedIndex(i,j,aLayerIndex)];
    }
    else
    {
        if(aLambda.size() != tGridDimensions[0]*tGridDimensions[1])
            throw(std::domain_error("AMFilterUtilities::computeLambda: input Lambda value incorrect size"));

        postMultiplyLambdaByGradientWRTPreviousLayerPrintableDensity(aGridBlueprintDensity,aGridPrintableDensity,aLayerIndex,aLambda); 

        for(size_t i = 0; i < tGridDimensions[0]; ++i)
            for(size_t j = 0; j < tGridDimensions[1]; ++j)
                aLambda[mGridUtilities.getSerializedIndexWithinLayer(i,j)] += aGridGradient[mGridUtilities.getSerializedIndex(i,j,aLayerIndex)];
    }
}

void AMFilterUtilities::postMultiplyLambdaByGradientWRTPreviousLayerPrintableDensity(const std::vector<double>& aGridBlueprintDensity,
                                                                                     const std::vector<double>& aGridPrintableDensity,
                                                                                     const size_t aLayerIndex,
                                                                                     std::vector<double>& aLambda) const
{
    auto tGridDimensions = mGridUtilities.getGridDimensions();
    size_t tNumGridElements = tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2];

    if(aGridBlueprintDensity.size() != tNumGridElements)
        throw(std::domain_error("AMFilterUtilities::postMultiplyLambdaByGradientWRTPreviousLayerPrintableDensity: Input density vector does not match grid size"));
    if(aGridPrintableDensity.size() != tNumGridElements)
        throw(std::domain_error("AMFilterUtilities::postMultiplyLambdaByGradientWRTPreviousLayerPrintableDensity: Input density vector does not match grid size"));
    if(aLayerIndex >= tGridDimensions[2])
        throw(std::out_of_range("AMFilterUtilities::postMultiplyLambdaByGradientWRTPreviousLayerPrintableDensity: Grid layer index out of range"));
    if(aLambda.size() != tGridDimensions[0]*tGridDimensions[1])
        throw(std::domain_error("AMFilterUtilities::postMultiplyLambdaByGradientWRTPreviousLayerPrintableDensity: input Lambda value incorrect size"));

    for(size_t i = 0; i < tGridDimensions[0]; ++i)
    {
        for(size_t j = 0; j < tGridDimensions[1]; ++j)
        {
            double tSupportDensity = computeGridPointSupportDensity(i,j,aLayerIndex,aGridPrintableDensity);
            double tSminGradientWRTSupportDensity = smin_gradient2(aGridBlueprintDensity[mGridUtilities.getSerializedIndex(i,j,aLayerIndex)], tSupportDensity);
            aLambda[mGridUtilities.getSerializedIndexWithinLayer(i,j)] *= tSminGradientWRTSupportDensity;
        }
    }

    std::vector<double> tNewLambda(aLambda.size(),0.0);
    for(size_t i = 0; i < tGridDimensions[0]; ++i)
    {
        for(size_t j = 0; j < tGridDimensions[1]; ++j)
        {
            double tCurrentLambda = aLambda[mGridUtilities.getSerializedIndexWithinLayer(i,j)];

            std::vector<std::vector<size_t>> tSupportIndices = mGridUtilities.getSupportIndices(i,j,aLayerIndex);
            std::vector<double> tSupportSetPrintableDensities;
            for(auto tSupportIndex : tSupportIndices)
            {
                tSupportSetPrintableDensities.push_back(aGridPrintableDensity[mGridUtilities.getSerializedIndex(tSupportIndex)]);
            }

            std::vector<double> tSupportDensityGradientWRTPreviousLayer;
            smax_gradient(tSupportSetPrintableDensities,mPNorm,tSupportDensityGradientWRTPreviousLayer);
            
            for(size_t tSumIndex = 0; tSumIndex < tSupportIndices.size(); ++tSumIndex)
            {
                size_t tSupportIndexI = tSupportIndices[tSumIndex][0];
                size_t tSupportIndexJ = tSupportIndices[tSumIndex][1];
                tNewLambda[mGridUtilities.getSerializedIndexWithinLayer(tSupportIndexI,tSupportIndexJ)] += tCurrentLambda * tSupportDensityGradientWRTPreviousLayer[tSumIndex];
            }
        }
    }

    aLambda = tNewLambda;
}

void AMFilterUtilities::postMultiplyGridBlueprintDensityGradient(const std::vector<double>& aInputGridGradient,
                                                                 AbstractInterface::ParallelVector* aOutputGradient) const
{
    const std::vector<std::vector<double>>& tCoordinates = mTetUtilities.getCoordinates();
    auto tGridDimensions = mGridUtilities.getGridDimensions();
    size_t tNumGridElements = tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2];

    if(aOutputGradient->get_length() != tCoordinates.size())
        throw(std::domain_error("AMFilterUtilities::postMultiplyGridBlueprintDensityGradient: Output gradient does not match tet mesh size"));

    if(aInputGridGradient.size() != tNumGridElements)
        throw(std::domain_error("AMFilterUtilities::postMultiplyGridBlueprintDensityGradient: Input gradient does not match grid size"));

    for(size_t tGridIndex = 0u; tGridIndex < tNumGridElements; ++tGridIndex)
    {
        Vector tGridPoint = mGridPointCoordinates[tGridIndex];
        int tContainingTetID = mContainingTetID[tGridIndex];

        if(tContainingTetID == -1)
            continue;

        std::vector<double> tLocalGradientValues = mTetUtilities.computeGradientOfDensityWRTTetNodeDensity(tGridPoint,tContainingTetID);

        for(size_t tLocalIndex = 0u; tLocalIndex < 4u; ++tLocalIndex)
        {
            auto tTet = mTetUtilities.getTet(tContainingTetID);
            double tVal = aOutputGradient->get_value(tTet[tLocalIndex]);
            tVal += aInputGridGradient.at(tGridIndex)*tLocalGradientValues[tLocalIndex];
            aOutputGradient->set_value(tTet[tLocalIndex], tVal);
        }
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

void smax_gradient(const std::vector<double>& aArguments, const double& aPNorm, std::vector<double>& aGradient)
{
    aGradient.resize(aArguments.size());

    double tQ = aPNorm + std::log(aArguments.size()) / std::log(0.5);

    for(size_t i = 0; i < aArguments.size(); ++i)
    {
        double tSum = 0;
        for(size_t j = 0; j < aArguments.size(); ++j)
            tSum += std::pow(aArguments.at(j),aPNorm);
        tSum = std::pow(tSum,1/tQ - 1);
        aGradient[i] = aPNorm*std::pow(aArguments.at(i),aPNorm - 1)/tQ * tSum;
    }
}

double smin(const double& aArg1, const double& aArg2, double aEps)
{
    double tVal = 0.5*(aArg1 + aArg2 - std::pow(std::pow((aArg1 - aArg2),2) + aEps,0.5) + std::sqrt(aEps));

    return tVal;
}

double smin_gradient1(const double& aArg1, const double& aArg2, double aEps)
{
    return 0.5*(1.0-(aArg1 - aArg2)*std::pow((std::pow(aArg1 - aArg2,2.0) + aEps),-0.5));
}

double smin_gradient2(const double& aArg1, const double& aArg2, double aEps)
{
    return 1.0 - smin_gradient1(aArg1,aArg2,aEps);
}

}
