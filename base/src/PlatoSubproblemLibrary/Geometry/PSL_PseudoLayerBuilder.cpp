#include <PSL_PseudoLayerBuilder.hpp>

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

void PseudoLayerBuilder::computeSupportSetAndCoefficients(std::vector<std::set<SupportPointData>>& aSupportSet,
                                                          std::map<SupportPointData,std::vector<double>>& aSupportCoefficients) const
{
    // we loop over elements for efficiency since we only have element to node connectivity
    // stored explicitly
    for(int tElementIndex = 0; tElementIndex < (int) mConnectivity.size(); ++tElementIndex)
    {
        std::vector<int> tElement = mConnectivity[tElementIndex];
        assert(tElement.size() == 4);

        for(size_t i = 0; i < 4; ++i)
        {
            int tNode = tElement[i];

            // add support points of tNode on tElement
            for(size_t j = 1; j < 4; ++j)
            {
                int tNeighbor = tElement[(i+j)%4];
                LessThanInBuildDirection tLessThanFunctor(mCoordinates,mBuildDirection);
                if(tLessThanFunctor(tNeighbor,tNode))
                {
                    if(isNeighborInCriticalWindow(tNode,tNeighbor))
                    {
                        SupportPointData tSupportPoint = {tNode, {tNeighbor}};
                        aSupportSet[tNode].insert(tSupportPoint);
                        aSupportCoefficients[tSupportPoint] = {1.0};
                    }
                    else
                    {
                        for(int k = 1; k < 4; ++k)
                        {
                            if((i+j+k)%4 != i)
                            {
                                int tOtherNeighbor = tElement[(i+j+k)%4];
                                if(tLessThanFunctor(tOtherNeighbor,tNode) && isNeighborInCriticalWindow(tNode,tOtherNeighbor))
                                {
                                    SupportPointData tSupportPoint = {tNode, {tNeighbor,tOtherNeighbor}};
                                    aSupportSet[tNode].insert(tSupportPoint);
                                    aSupportCoefficients[tSupportPoint] = computeSupportCoefficients(tSupportPoint);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

bool PseudoLayerBuilder::isNeighborInCriticalWindow(const int& aNode, 
                                const int& aNeighbor) const
{
    PlatoSubproblemLibrary::Vector tNodeVector(mCoordinates[aNode]);
    PlatoSubproblemLibrary::Vector tNeighborVector(mCoordinates[aNeighbor]);

    PlatoSubproblemLibrary::Vector tVec = tNeighborVector - tNodeVector;

    return PlatoSubproblemLibrary::angle_between(tVec,-1*mBuildDirection) < mCriticalPrintAngle;
}

std::vector<double> PseudoLayerBuilder::computeSupportCoefficients(const SupportPointData& aSupportPoint) const
{
    const int tNode = aSupportPoint.first;
    const std::set<int> tSupportDependencyNodes = aSupportPoint.second;

    if(tSupportDependencyNodes.size() != 2)
        throw(std::length_error("Expecting support point to be dependent on two nodes"));

    auto tIterator = tSupportDependencyNodes.begin();
    int tNeighbor1 = *tIterator;
    ++tIterator;
    int tNeighbor2 = *tIterator;

    if((isNeighborInCriticalWindow(tNode,tNeighbor1) && isNeighborInCriticalWindow(tNode,tNeighbor2))
    || (!isNeighborInCriticalWindow(tNode,tNeighbor1) && !isNeighborInCriticalWindow(tNode,tNeighbor2)))
        throw(std::runtime_error("Expecting neighbor exactly one support dependency node to be outside the critical window"));

    PlatoSubproblemLibrary::Vector tNodeVec(mCoordinates[tNode]);
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
        
    while(fabs(tResidual/tAnswer) > tTolerance && tMaxIterations > 0)
    {
        tResidual = intersectionResidual(tAnswer,aV0,aV1,aV2);
        double tDerivative = intersectionResidualDerivative(tAnswer,aV0,aV1,aV2);
        tAnswer -= tResidual / tDerivative;
        --tMaxIterations;
    }

    if(fabs(tResidual/tAnswer) > tTolerance)
        throw(std::runtime_error("Max iterations reached but newton iteration did not converge"));

    return tAnswer;
}

int PseudoLayerBuilder::assignNodeToPseudoLayer(const int& aNode, 
                                                const std::vector<int>& aPseudoLayers,
                                                const std::set<SupportPointData>& aSupportSet) const
{
    std::set<int> tSupportingNeighbors = getSupportingNeighbors(aNode, aSupportSet);
    std::set<int> tConnectedPseudoLayers = determineConnectedPseudoLayers(aNode, aPseudoLayers, tSupportingNeighbors);
    int tSupportingPseudoLayer = determineSupportingPseudoLayer(aNode, tSupportingNeighbors, aPseudoLayers, tConnectedPseudoLayers);

    return tSupportingPseudoLayer + 1;
}

std::set<int> PseudoLayerBuilder::getSupportingNeighbors(const int& aNode, const std::set<SupportPointData>& aSupportSet) const
{
    std::set<int> tSupportingNeighbors;

    for(auto tSupportPoint : aSupportSet)
    {
        std::set<int> tSupportingNeighborsForThisPoint = tSupportPoint.second;
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

std::set<SupportPointData> PseudoLayerBuilder::pruneSupportSet(const int& aNode, const std::vector<int>& aPseudoLayers, const std::set<SupportPointData>& aSupportSet) const
{
    int tSupportingPseudoLayer = aPseudoLayers[aNode] - 1;

    std::set<SupportPointData> tNodeSupportSet = aSupportSet;

    for (auto tIterator = tNodeSupportSet.begin(); tIterator != tNodeSupportSet.end(); )
    {
        std::set<int> tSupportPoint = tIterator->second;
        bool tErased = false;
        for(auto tNeighbor : tSupportPoint)
        {
            if(aPseudoLayers[tNeighbor] != tSupportingPseudoLayer)
            {
                tIterator = tNodeSupportSet.erase(tIterator);
                tErased = true;
                break;
            }
        }
        if(!tErased)
        {
            ++tIterator;
        }
    }

    return tNodeSupportSet;
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

PlatoSubproblemLibrary::Vector PseudoLayerBuilder::getVectorToSupportPoint(const SupportPointData& aSupportPoint,
                                                       const std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>>& aSupportCoefficients,
                                                       const std::vector<std::vector<double>>& aCoordinates) const
{
    std::vector<double> tCoefficients = aSupportCoefficients.at(aSupportPoint);
    std::set<int> tSupportingNodes = aSupportPoint.second;
    PlatoSubproblemLibrary::Vector tVec;
    int tCoefficientIndex = 0;
    for(int tSupportingNode : tSupportingNodes)
    {
        double tCoefficient = tCoefficients[tCoefficientIndex];
        tVec = tVec + tCoefficient*PlatoSubproblemLibrary::Vector(aCoordinates[tSupportingNode]);
        ++tCoefficientIndex;
    }

    return tVec;
}

// void PseudoLayerBuilder::constructNodeToElementsMap()
// {
//     for(int i = 0; i < (int) mConnectivity.size(); ++i)
//     {
//         auto tElement = mConnectivity[i];
//         for(auto tNode : tElement)
//         {
//             mNodeToElementsMap[tNode].insert(i);
//         }
//     }
// }

}
