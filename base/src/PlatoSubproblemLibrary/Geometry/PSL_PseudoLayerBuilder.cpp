#include <PSL_PseudoLayerBuilder.hpp>
#include <iostream>

namespace PlatoSubproblemLibrary
{

// functor to sort in build direction
class LessThanInBuildDirection
{
    public:
        LessThanInBuildDirection(const std::vector<std::vector<double>>& aCoordinates,
                                 const Vector& aBuildDirection)
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

            Vector tVec1(mCoordinates[aIndex1]);
            Vector tVec2(mCoordinates[aIndex2]);

            double tSignedDistanceFromOrigin1 = dot_product(tVec1,mBuildDirection);
            double tSignedDistanceFromOrigin2 = dot_product(tVec2,mBuildDirection);

            return tSignedDistanceFromOrigin1 < tSignedDistanceFromOrigin2;
        }

        const std::vector<std::vector<double>>& mCoordinates;
        const Vector& mBuildDirection;
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

double PseudoLayerBuilder::angleObjective(const double& aSuperUnkown,
                                          const Vector& aV0,
                                          const Vector& aV1,
                                          const Vector& aV2) const
{
     Vector tVec = aSuperUnkown*aV1 + (1-aSuperUnkown)*aV2 - aV0;
     double tCosineDenominator = sqrt(dot_product(tVec,tVec))*sqrt(dot_product(mBuildDirection,mBuildDirection));

     double tCosine = -1*dot_product(tVec,mBuildDirection)/(tCosineDenominator);

     double tSine = sqrt(1-tCosine*tCosine);
    
     return tSine;
}

double PseudoLayerBuilder::angleObjectiveDerivative(const double& aSuperUnkown,
                                                    const Vector& aV0,
                                                    const Vector& aV1,
                                                    const Vector& aV2) const
{
    Vector tVec = aSuperUnkown*aV1 + (1-aSuperUnkown)*aV2 - aV0;
    Vector tVecPrime = aV1 - aV2;

    double tCosineDenominator = sqrt(dot_product(tVec,tVec))*sqrt(dot_product(mBuildDirection,mBuildDirection));
    double tCosine = -1*dot_product(tVec,mBuildDirection)/(tCosineDenominator);

    double tTerm1 = -1*tCosineDenominator*(dot_product(tVecPrime,mBuildDirection));
    double tTerm2 = dot_product(tVec,mBuildDirection)*pow(dot_product(tVec,tVec),-0.5)*sqrt(dot_product(mBuildDirection,mBuildDirection))*dot_product(tVec,tVecPrime);
    double tCosinePrimeNumerator = tTerm1 + tTerm2;
    double tCosinePrimeDenominator = tCosineDenominator*tCosineDenominator;

    double tCosinePrime = tCosinePrimeNumerator / tCosinePrimeDenominator;

    double tSinePrime = -1*pow(1-tCosine*tCosine,-0.5)*tCosine*tCosinePrime;

    return tSinePrime;
}

double PseudoLayerBuilder::intersectionResidual(const double& aSuperUnkown,
                                                const Vector& aV0,
                                                const Vector& aV1,
                                                const Vector& aV2) const
{
     Vector tVec = aSuperUnkown*aV1 + (1-aSuperUnkown)*aV2 - aV0;

     return cos(mCriticalPrintAngle)*sqrt(dot_product(tVec,tVec))*sqrt(dot_product(mBuildDirection,mBuildDirection)) 
             + dot_product(tVec,mBuildDirection);
}

double PseudoLayerBuilder::intersectionResidualDerivative(const double& aSuperUnkown,
                                                          const Vector& aV0,
                                                          const Vector& aV1,
                                                          const Vector& aV2) const
{
    Vector tVec = aSuperUnkown*aV1 + (1-aSuperUnkown)*aV2 - aV0;
    Vector tVecPrime = aV1 - aV2;
    return cos(mCriticalPrintAngle)*pow(dot_product(tVec,tVec),-0.5)
            *dot_product(tVec,tVecPrime)*sqrt(dot_product(mBuildDirection,mBuildDirection))
            + dot_product(tVecPrime,mBuildDirection);
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

void PseudoLayerBuilder::computeBoundarySupportPointsAndCoefficients(size_t& aLocalIndexOnElement,
                                                                     std::vector<int>& aElement,
                                                                     std::vector<std::set<BoundarySupportPoint>>& aBoundarySupportSet) const
{
    int tNode = aElement[aLocalIndexOnElement];
    LessThanInBuildDirection tLessThanFunctor(mCoordinates,mBuildDirection);

    for(size_t j = 1; j < 4; ++j)
    {
        int tNeighbor = aElement[(aLocalIndexOnElement+j)%4];

        if(tLessThanFunctor(tNeighbor,tNode))
        {
            if(isNeighborInCriticalWindow(tNode,tNeighbor))
            {
                BoundarySupportPoint tSupportPoint(tNode, {tNeighbor}, std::vector<double>({1.0}));
                aBoundarySupportSet[tNode].insert(tSupportPoint);
            }
            else // tNeighbor is not in the critical window
            {
                for(int k = 1; k < 4; ++k) // for each of the other neighbors
                {
                    if((aLocalIndexOnElement+j+k)%4 != aLocalIndexOnElement)
                    {
                        int tOtherNeighbor = aElement[(aLocalIndexOnElement+j+k)%4];
                        if(tLessThanFunctor(tOtherNeighbor,tNode))
                        {
                            if(isNeighborInCriticalWindow(tNode,tOtherNeighbor))
                            {
                                BoundarySupportPoint tSupportPoint(tNode, {tNeighbor,tOtherNeighbor}, computeSupportCoefficients(tNode, {tNeighbor, tOtherNeighbor}));
                                aBoundarySupportSet[tNode].insert(tSupportPoint);
                            }
                            else // both tNeighbor and tOtherNeighbor are outside the critical window
                            {
                                double tFirstCoefficient = 0;
                                if(criticalWindowIntersectsEdge(tNode, tNeighbor, tOtherNeighbor, tFirstCoefficient))
                                {
                                    std::cout << "Edge crossing" << std::endl;
                                    //add both points to the support set along with coefficients
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

bool PseudoLayerBuilder::criticalWindowIntersectsEdge(const int& aNode, const int& aNeighbor0, const int& aNeighbor1, double& aFirstCoefficient) const
{
    Vector tNodeVector(mCoordinates[aNode]);
    Vector tNeighborVector0(mCoordinates[aNeighbor0]);
    Vector tNeighborVector1(mCoordinates[aNeighbor1]);

    aFirstCoefficient = determineVectorWithMinimalAngleBetweenEdgeAndBuildDirection(tNodeVector, tNeighborVector0, tNeighborVector1);

    Vector tVec = aFirstCoefficient*tNeighborVector0 + (1-aFirstCoefficient)*tNeighborVector1 - tNodeVector;

    if(angle_between(tVec, -1*mBuildDirection) < mCriticalPrintAngle)
    {
        if(aFirstCoefficient < 0 || aFirstCoefficient > 1)
        {
            std::cout << "tVec: " << tVec << std::endl;
            std::cout << "aFirstCoefficient: " << aFirstCoefficient << std::endl;
            std::cout << "Angle Between: " << angle_between(tVec, -1*mBuildDirection) << std::endl;
            throw(std::runtime_error("PseudoLayerBuilder::criticalWindowIntersectsEdge: Coefficient expected to be between 0 and 1"));
        }
        return true;
    }
    else
        return false;
}

double PseudoLayerBuilder::determineVectorWithMinimalAngleBetweenEdgeAndBuildDirection(const Vector& aV0,
                                                                                       const Vector& aV1,
                                                                                       const Vector& aV2) const
{
    double tAnswer = 0.5;
    double tPrevAnswer = 0;
    double tTolerance = 1e-12;
    int tMaxIterations = 100000;
    double tDerivative = 1; //fake starting value just to start the while loop
    double tPrevDerivative = 1;
        
    while(fabs(tDerivative) > tTolerance && tMaxIterations > 0)
    {
        tPrevDerivative = tDerivative;
        tDerivative = angleObjectiveDerivative(tAnswer,aV0,aV1,aV2);
        double tStepSize = (tAnswer - tPrevAnswer)*(tDerivative - tPrevDerivative)/((tDerivative-tPrevDerivative)*(tDerivative-tPrevDerivative));
        tPrevAnswer = tAnswer;
        tAnswer -= tDerivative*tStepSize;
        --tMaxIterations;
    }

    if(fabs(tDerivative) > tTolerance)
        throw(std::runtime_error("Max iterations reached but newton iteration did not converge"));

    return tAnswer;
}

bool PseudoLayerBuilder::isPointInCriticalWindow(const int& aNode, const Vector& aCoordinateVec) const
{
    Vector tNodeVector(mCoordinates[aNode]);
    Vector tVec = aCoordinateVec - tNodeVector;

    return angle_between(tVec, -1*mBuildDirection) < mCriticalPrintAngle;
}

bool PseudoLayerBuilder::isNeighborInCriticalWindow(const int& aNode, const int& aNeighbor) const
{
    Vector tNodeVector(mCoordinates[aNode]);
    Vector tNeighborVector(mCoordinates[aNeighbor]);

    Vector tVec = tNeighborVector - tNodeVector;

    return angle_between(tVec,-1*mBuildDirection) < mCriticalPrintAngle;
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
        throw(std::runtime_error("Expecting exactly one support dependency node to be outside the critical window"));

    Vector tNodeVec(mCoordinates[aNode]);
    Vector tNeighbor1Vec(mCoordinates[tNeighbor1]);
    Vector tNeighbor2Vec(mCoordinates[tNeighbor2]);

    double tInitialGuess = getInitialGuess(aNode, tNeighbor1, tNeighbor2);

    double tCoefficient1 = computeFirstCoefficient(tInitialGuess, tNodeVec, tNeighbor1Vec, tNeighbor2Vec);
    double tCoefficient2 = 1 - tCoefficient1;

    std::vector<double> tSupportCoefficients = {tCoefficient1, tCoefficient2};
    return tSupportCoefficients;
}

double PseudoLayerBuilder::getInitialGuess(const int& aNode,
                                           const int& aNeighbor1,
                                           const int& aNeighbor2) const
{
    double tInitialGuess = 0.5;
    double tStep = 0.25;
    int tMaxIterations = 10;
    double tMultiplier;

    if(isNeighborInCriticalWindow(aNode,aNeighbor1) && !isNeighborInCriticalWindow(aNode,aNeighbor2))
    {
        tMultiplier = 1.0;
    }
    else if(!isNeighborInCriticalWindow(aNode,aNeighbor1) && isNeighborInCriticalWindow(aNode,aNeighbor2))
    {
        tMultiplier = -1.0;
    }
    else
    {
        throw(std::runtime_error("PseudoLayerBuilder::getInitialGuess: Expecting exactly one support dependency node to be inside the critical window"));
    }

    for(int i = 0; i < tMaxIterations; ++i)
    {
        Vector tPoint = tInitialGuess*mCoordinates[aNeighbor1]+(1-tInitialGuess)*mCoordinates[aNeighbor2];
        if(isPointInCriticalWindow(aNode,tPoint))
        {
            tInitialGuess -= tMultiplier*(tStep);
        }
        else
        {
            tInitialGuess += tMultiplier*(tStep);
        }
        tStep = tStep/2.0;
    }

    return tInitialGuess;
}

double PseudoLayerBuilder::computeFirstCoefficient(const double& aInitialGuess,
                                                   const Vector& aV0,
                                                   const Vector& aV1,
                                                   const Vector& aV2) const
{
    double tAnswer = aInitialGuess;
    double tPrevAnswer = -1.0;
    double tTolerance = 1e-12;
    double tResidual = 0;
    int tMaxIterations = 100000;
        
    while(fabs(tAnswer - tPrevAnswer) > tTolerance && tMaxIterations > 0)
    {
        tPrevAnswer = tAnswer;
        tResidual = intersectionResidual(tAnswer,aV0,aV1,aV2);
        double tDerivative = intersectionResidualDerivative(tAnswer,aV0,aV1,aV2);
        tAnswer -= tResidual / tDerivative;
        --tMaxIterations;
    }

    if(fabs(tAnswer - tPrevAnswer) > tTolerance)
        throw(std::runtime_error("PseudoLayerBuilder::computeFirstCoefficient: Max iterations reached but newton iteration did not converge"));


    if(tAnswer < 0.0 || tAnswer > 1.0)
        throw(std::runtime_error("PseudoLayerBuilder::computeFirstCoefficient: Expecting value to be between 0 and 1"));

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
                Vector tNodeVec(mCoordinates[aNode]);
                Vector tNeighborVector(mCoordinates[tNeighbor]);
                Vector tVec = tNeighborVector - tNodeVec;
                double tDistanceBetweeen = tVec.euclideanNorm();
                double tAngleBetween = angle_between(tVec, -1*mBuildDirection);
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

Vector getVectorToSupportPoint(const BoundarySupportPoint& aSupportPoint,
                                                       const std::vector<std::vector<double>>& aCoordinates)
{
    std::vector<double> tCoefficients = aSupportPoint.getCoefficients();
    std::set<int> tSupportingNodes = aSupportPoint.getSupportingNodeIndices();

    Vector tVec;
    int tCoefficientIndex = 0;
    for(int tSupportingNode : tSupportingNodes)
    {
        double tCoefficient = tCoefficients[tCoefficientIndex];

        if(tSupportingNode < 0 || tSupportingNode > (int) aCoordinates.size())
            throw(std::out_of_range("Node index must lie within 0 and the number of nodes"));

        tVec = tVec + tCoefficient*Vector(aCoordinates[tSupportingNode]);
        ++tCoefficientIndex;
    }

    return tVec;
}

}
