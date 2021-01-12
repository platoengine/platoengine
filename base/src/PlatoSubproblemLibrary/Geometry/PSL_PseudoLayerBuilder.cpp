#include <PSL_PseudoLayerBuilder.hpp>
#include <iostream>

namespace PlatoSubproblemLibrary
{

// functor to sort in build direction
class LessThanInBuildDirection
{
    public:
        LessThanInBuildDirection(const std::vector<std::vector<double>>& aCoordinates,
                                 const PlatoSubproblemLibrary::Vector& aBuildDirection)
                                        :mCoordinates(aCoordinates),
                                         mBuildDirection(aBuildDirection)
        {}
        ~LessThanInBuildDirection(){}

        bool operator()(int aIndex1, int aIndex2) const
        {
            if(aIndex1 > (int) mCoordinates.size() || aIndex2 > (int) mCoordinates.size())
            {
                throw(std::out_of_range("Indices must be betweeen zero and number of nodes in mesh to compare distance of node in build direction"));
            }

            PlatoSubproblemLibrary::Vector tVec1(mCoordinates[aIndex1]);
            PlatoSubproblemLibrary::Vector tVec2(mCoordinates[aIndex2]);

            double tSignedDistanceFromOrigin1 = PlatoSubproblemLibrary::dot_product(tVec1,mBuildDirection);
            double tSignedDistanceFromOrigin2 = PlatoSubproblemLibrary::dot_product(tVec2,mBuildDirection);

            return tSignedDistanceFromOrigin1 < tSignedDistanceFromOrigin2;
        }

        const std::vector<std::vector<double>>& mCoordinates;
        const PlatoSubproblemLibrary::Vector& mBuildDirection;
};

std::vector<int> PseudoLayerBuilder::orderNodesInBuildDirection() const
{

    LessThanInBuildDirection tLessThanFunctor(mCoordinates, mBuildDirection);

    std::vector<int> tOrderedNodes(mCoordinates.size());
    for(int i = 0; i < (int) mCoordinates.size(); ++i)
    {
        tOrderedNodes[i] = i;
    }

    std::sort(tOrderedNodes.begin(), tOrderedNodes.end(), tLessThanFunctor);
    
    return tOrderedNodes;
}

double PseudoLayerBuilder::intersectionResidual(const double& aSuperUnkown,
                            const PlatoSubproblemLibrary::Vector& aV0,
                            const PlatoSubproblemLibrary::Vector& aV1,
                            const PlatoSubproblemLibrary::Vector& aV2) const
{
     PlatoSubproblemLibrary::Vector tVec = aSuperUnkown*aV1 + (1-aSuperUnkown)*aV2 - aV0;

     return cos(mCriticalPrintAngle)*sqrt(PlatoSubproblemLibrary::dot_product(tVec,tVec))*sqrt(PlatoSubproblemLibrary::dot_product(mBuildDirection,mBuildDirection)) 
             + PlatoSubproblemLibrary::dot_product(tVec,mBuildDirection);
}

double PseudoLayerBuilder::intersectionResidualDerivative(const double& aSuperUnkown,
                                      const PlatoSubproblemLibrary::Vector& aV0,
                                      const PlatoSubproblemLibrary::Vector& aV1,
                                      const PlatoSubproblemLibrary::Vector& aV2) const
{
    PlatoSubproblemLibrary::Vector tVec = aSuperUnkown*aV1 + (1-aSuperUnkown)*aV2 - aV0;
    PlatoSubproblemLibrary::Vector tVecPrime = aV1 - aV2;
    return cos(mCriticalPrintAngle)*pow(PlatoSubproblemLibrary::dot_product(tVec,tVec),-0.5)
            *PlatoSubproblemLibrary::dot_product(tVec,tVecPrime)*sqrt(PlatoSubproblemLibrary::dot_product(mBuildDirection,mBuildDirection))
            + PlatoSubproblemLibrary::dot_product(tVecPrime,mBuildDirection);
}

std::vector<int> PseudoLayerBuilder::setBaseLayerIDToZeroAndOthersToMinusOne() const
{
    std::vector<int> tPseudoLayers(mCoordinates.size());
    for(int i = 0; i < (int) mCoordinates.size(); ++i)
    {
        tPseudoLayers[i] = -1;
    }

    for(auto tNode : mBaseLayer)
    {
        tPseudoLayers[tNode] = 0;
    }

    return tPseudoLayers;
}

void PseudoLayerBuilder::computeSupportSetAndCoefficients(std::vector<std::set<BoundarySupportPoint>>& aBoundarySupportSet) const
{
    // we loop over elements for efficiency since we only have element to node connectivity
    // stored explicitly
    for(int tElementIndex = 0; tElementIndex < (int) mConnectivity.size(); ++tElementIndex)
    {
        std::vector<int> tElement = mConnectivity[tElementIndex];
        assert(tElement.size() == 4);

        for(size_t i = 0; i < 4; ++i)
        {
            computeBoundarySupportPointsAndCoefficients(i, tElement, aBoundarySupportSet);
        }
    }
}

void PseudoLayerBuilder::computeBoundarySupportPointsAndCoefficients(size_t& i,
                                                      std::vector<int>& aElement,
                                                      std::vector<std::set<BoundarySupportPoint>>& aBoundarySupportSet) const
{
    int tNode = aElement[i];
    LessThanInBuildDirection tLessThanFunctor(mCoordinates,mBuildDirection);

    for(size_t j = 1; j < 4; ++j)
    {
        int tNeighbor = aElement[(i+j)%4];

        if(tLessThanFunctor(tNeighbor,tNode))
        {
            if(isNeighborInCriticalWindow(tNode,tNeighbor))
            {
                BoundarySupportPoint tSupportPoint(tNode, {tNeighbor}, {1.0});
                aBoundarySupportSet[tNode].insert(tSupportPoint);
            }
            else // tNeighbor is not in the critical window
            {
                for(int k = 1; k < 4; ++k)
                {
                    if((i+j+k)%4 != i)
                    {
                        int tOtherNeighbor = aElement[(i+j+k)%4];
                        if(tLessThanFunctor(tOtherNeighbor,tNode))
                        {
                            if(isNeighborInCriticalWindow(tNode,tOtherNeighbor))
                            {
                                BoundarySupportPoint tSupportPoint(tNode, {tNeighbor,tOtherNeighbor}, computeSupportCoefficients(tNode, {tNeighbor, tOtherNeighbor}));
                                aBoundarySupportSet[tNode].insert(tSupportPoint);
                            }
                            else // tOtherNeighbor is also outside the critical window
                            {
                                //add both points to the support set along with coefficients
                            }
                        }
                    }
                }
            }
        }
    }
}

bool PseudoLayerBuilder::isNeighborInCriticalWindow(const int& aNode, const int& aNeighbor) const
{
    PlatoSubproblemLibrary::Vector tNodeVector(mCoordinates[aNode]);
    PlatoSubproblemLibrary::Vector tNeighborVector(mCoordinates[aNeighbor]);

    PlatoSubproblemLibrary::Vector tVec = tNeighborVector - tNodeVector;

    return PlatoSubproblemLibrary::angle_between(tVec,-1*mBuildDirection) < mCriticalPrintAngle;
}

std::vector<double> PseudoLayerBuilder::computeSupportCoefficients(const int& aNode, const std::set<int>& aSupportDependencyNodes) const
{
    if(aSupportDependencyNodes.size() != 2)
        throw(std::length_error("Expecting support point to be dependent on two nodes"));

    auto tIterator = aSupportDependencyNodes.begin();
    int tNeighbor1 = *tIterator;
    ++tIterator;
    int tNeighbor2 = *tIterator;

    if((isNeighborInCriticalWindow(aNode,tNeighbor1) && isNeighborInCriticalWindow(aNode,tNeighbor2))
    || (!isNeighborInCriticalWindow(aNode,tNeighbor1) && !isNeighborInCriticalWindow(aNode,tNeighbor2)))
        throw(std::runtime_error("Expecting neighbor exactly one support dependency node to be outside the critical window"));

    PlatoSubproblemLibrary::Vector tNodeVec(mCoordinates[aNode]);
    PlatoSubproblemLibrary::Vector tNeighbor1Vec(mCoordinates[tNeighbor1]);
    PlatoSubproblemLibrary::Vector tNeighbor2Vec(mCoordinates[tNeighbor2]);

    double tCoefficient1 = computeFirstCoefficient(tNodeVec, tNeighbor1Vec, tNeighbor2Vec);
    double tCoefficient2 = 1 - tCoefficient1;

    std::vector<double> tSupportCoefficients = {tCoefficient1, tCoefficient2};
    return tSupportCoefficients;
}

double PseudoLayerBuilder::computeFirstCoefficient(const PlatoSubproblemLibrary::Vector& aV0,
                                                   const PlatoSubproblemLibrary::Vector& aV1,
                                                   const PlatoSubproblemLibrary::Vector& aV2) const
{
    double tAnswer = 0.5;
    double tTolerance = 1e-5;
    double tResidual = 1; //fake starting value just to start the while loop
    int tMaxIterations = 10000;
        
    while(fabs(tResidual) > tTolerance && tMaxIterations > 0)
    {
        tResidual = intersectionResidual(tAnswer,aV0,aV1,aV2);
        double tDerivative = intersectionResidualDerivative(tAnswer,aV0,aV1,aV2);
        tAnswer -= tResidual / tDerivative;
        --tMaxIterations;
    }

    if(fabs(tResidual) > tTolerance)
        throw(std::runtime_error("Max iterations reached but newton iteration did not converge"));

    return tAnswer;
}

int PseudoLayerBuilder::assignNodeToPseudoLayer(const int& aNode, 
                                                const std::vector<int>& aPseudoLayers,
                                                const std::set<BoundarySupportPoint>& aSupportSet) const
{
    // TODO:  Really I should just use the same function to determine the support density, but assuming a density of one at each node,
    // to determine which layer contributes the most
    if(aPseudoLayers[aNode] == 0)
        return 0;

    if(aPseudoLayers[aNode] == -1)
    {
        std::set<int> tSupportingNeighbors = getSupportingNeighbors(aNode, aSupportSet);
        std::set<int> tConnectedPseudoLayers = determineConnectedPseudoLayers(aNode, aPseudoLayers, tSupportingNeighbors);
        int tSupportingPseudoLayer = determineSupportingPseudoLayer(aNode, tSupportingNeighbors, aPseudoLayers, tConnectedPseudoLayers);

        return tSupportingPseudoLayer + 1;
    }
    else
    {
        throw(std::runtime_error("Pseudo Layer already assigned"));
    }

}

std::set<int> PseudoLayerBuilder::getSupportingNeighbors(const int& aNode, const std::set<BoundarySupportPoint>& aSupportSet) const
{
    std::set<int> tSupportingNeighbors;

    for(auto tSupportPoint : aSupportSet)
    {
        std::set<int> tSupportingNeighborsForThisPoint = tSupportPoint.getSupportingNodeIndices();
        for(auto tNeighbor : tSupportingNeighborsForThisPoint)
        {
            tSupportingNeighbors.insert(tNeighbor);
        }
    }

    return tSupportingNeighbors;
}

std::set<int> PseudoLayerBuilder::determineConnectedPseudoLayers(const int& aNode, 
                                                                 const std::vector<int>& aPseudoLayers,
                                                                 const std::set<int>& aNeighbors) const
{
    std::set<int> tConnectedPseudoLayers;

    for(auto tNeighbor : aNeighbors)
    {
        if(aPseudoLayers[tNeighbor] == -1)
            throw(std::runtime_error("All nodes below the current node should already be assigned to a pseudo layer"));

        tConnectedPseudoLayers.insert(aPseudoLayers[tNeighbor]);
    }

    return tConnectedPseudoLayers;
}

int PseudoLayerBuilder::determineSupportingPseudoLayer(const int& aNode,
                                                       const std::set<int>& aNeighbors,
                                                       const std::vector<int>& aPseudoLayers,
                                                       const std::set<int>& aConnectedPseudoLayers) const
{
    int tSupportingPseudoLayer;
    std::vector<double> tSupportScores;

    for(auto tPseudoLayer : aConnectedPseudoLayers)
    {
        double tLayerSupportScore = 0;
        for(auto tNeighbor : aNeighbors)
        {
            if(aPseudoLayers[tNeighbor] == tPseudoLayer)
            {
                PlatoSubproblemLibrary::Vector tNodeVec(mCoordinates[aNode]);
                PlatoSubproblemLibrary::Vector tNeighborVector(mCoordinates[tNeighbor]);
                PlatoSubproblemLibrary::Vector tVec = tNeighborVector - tNodeVec;
                double tDistanceBetweeen = tVec.euclideanNorm();
                double tAngleBetween = PlatoSubproblemLibrary::angle_between(tVec, -1*mBuildDirection);
                tLayerSupportScore += (1/(1+tDistanceBetweeen))*cos(tAngleBetween);
            }
        }

        tSupportScores.push_back(tLayerSupportScore);
    }

    double tHighestScore = 0;
    for(int i = 0; i < (int) tSupportScores.size(); ++i)
    {
        double tScore = tSupportScores[i];
        if(tScore > tHighestScore)
        {
            tHighestScore = tScore;
            // set the ith layer as the supporting pseudo layer
            tSupportingPseudoLayer = *std::next(aConnectedPseudoLayers.begin(), i);
        }
    }

    return tSupportingPseudoLayer;
}

int getOtherSupportingNodeIndex(const BoundarySupportPoint& aSupportPoint, const int& aNeighbor)
{
    std::set<int> tNeighbors = aSupportPoint.getSupportingNodeIndices();

    if(tNeighbors.size() == 1u)
        throw(std::runtime_error("Support point depends on only one node"));

    for(auto tNeighbor : tNeighbors)
    {
        if(tNeighbor != aNeighbor)
            return tNeighbor;
    }

    throw(std::runtime_error("Invalid Support point input data"));
}

std::set<BoundarySupportPoint> PseudoLayerBuilder::pruneSupportSet(const int& aNode,
                                                               const std::vector<int>& aPseudoLayers,
                                                               const std::set<BoundarySupportPoint>& aSupportSet) const
{
    int tSupportingPseudoLayer = aPseudoLayers[aNode] - 1;

    std::set<BoundarySupportPoint> tPrunedSupportSet;
    
    for (auto tSupportPoint : aSupportSet)
    {
        bool tRemainsInSupportSet = true;
        for(auto tNeighbor : tSupportPoint.getSupportingNodeIndices())
        {
            if(aPseudoLayers[tNeighbor] != tSupportingPseudoLayer)
            {
                if(isNeighborInCriticalWindow(aNode,tNeighbor))
                {
                    if(tSupportPoint.getSupportingNodeIndices().size() == 1)
                    {
                        tRemainsInSupportSet = false;
                        break;
                    }
                    else
                    {
                        auto tOtherNeighbor = getOtherSupportingNodeIndex(tSupportPoint, tNeighbor);
                        if(aPseudoLayers[tOtherNeighbor] != tSupportingPseudoLayer)
                        {
                            tRemainsInSupportSet = false;
                            break;
                        }
                        else
                        {
                            std::vector<double> tModifiedCoefficient = {tSupportPoint.getCorrespondingCoefficient(tOtherNeighbor)};
                            BoundarySupportPoint tModifiedSupportPoint(tSupportPoint.getSupportedNodeIndex(),{tOtherNeighbor}, tModifiedCoefficient);

                            tPrunedSupportSet.insert(tModifiedSupportPoint);

                            tRemainsInSupportSet = false;
                            break;
                        }
                    }
                }
                else
                {
                    tRemainsInSupportSet = false;
                    break;
                }
            }
        }
        if(tRemainsInSupportSet)
        {
            tPrunedSupportSet.insert(tSupportPoint);
        }
    }

    return tPrunedSupportSet;
}

void PseudoLayerBuilder::checkInput() const
{
    int tMaxNodeID = 0;
    int tMinNodeID = 0;
    for(auto tElement : mConnectivity)
    {
        for(auto tNodeID : tElement)
        {
            if(tNodeID > tMaxNodeID)
            {
                tMaxNodeID = tNodeID;
            }
            if(tNodeID < tMinNodeID)
            {
                tMinNodeID = tNodeID;
            }
        }
    }

    for(auto tNodeID : mBaseLayer)
    {
        if(tNodeID > tMaxNodeID)
        {
            tMaxNodeID = tNodeID;
        }
        if(tNodeID < tMinNodeID)
        {
            tMinNodeID = tNodeID;
        }
    }

    if(tMinNodeID < 0 || tMaxNodeID >= (int) mCoordinates.size())
    {
        std::string tError = std::string("Node IDs must be between zero and ") + std::to_string(mCoordinates.size() - 1);
        throw(std::out_of_range(tError));
    }

    if(mCriticalPrintAngle <= 0 || mCriticalPrintAngle >= M_PI/2)
        throw(std::out_of_range("Critical print angle should be between zero and Pi/2"));
}

PlatoSubproblemLibrary::Vector getVectorToSupportPoint(const BoundarySupportPoint& aSupportPoint,
                                                       const std::vector<std::vector<double>>& aCoordinates)
{
    std::vector<double> tCoefficients = aSupportPoint.getCoefficients();
    std::set<int> tSupportingNodes = aSupportPoint.getSupportingNodeIndices();

    PlatoSubproblemLibrary::Vector tVec;
    int tCoefficientIndex = 0;
    for(int tSupportingNode : tSupportingNodes)
    {
        double tCoefficient = tCoefficients[tCoefficientIndex];

        if(tSupportingNode < 0 || tSupportingNode > (int) aCoordinates.size())
            throw(std::out_of_range("Node index must lie within 0 and the number of nodes"));

        tVec = tVec + tCoefficient*PlatoSubproblemLibrary::Vector(aCoordinates[tSupportingNode]);
        ++tCoefficientIndex;
    }

    return tVec;
}

}
