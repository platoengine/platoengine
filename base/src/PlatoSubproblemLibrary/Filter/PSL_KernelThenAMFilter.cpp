// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_KernelThenAMFilter.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Point.hpp"
#include "PSL_Vector.hpp"


namespace PlatoSubproblemLibrary
{

double KernelThenAMFilter::projection_apply(const double& beta, const double& input)
{
  return AM_apply(beta,input);
}
double KernelThenAMFilter::projection_gradient(const double& beta, const double& input)
{
  return AM_gradient(beta,input);
}

void KernelThenAMFilter::buildPseudoLayers()
{
    if(!mInitialized)
        throw(std::runtime_error("AM filter not initialized"));

    orderNodesInBuildDirection();
    setBaseLayerIDToZeroAndOthersToMinusOne();
    computeSupportSet();

    for(auto tNode : mOrderedNodes)
    {
        assignNodeToPseudoLayerAndPruneSupportSet(tNode);
    }
}

void KernelThenAMFilter::orderNodesInBuildDirection()
{
    for(int i = 0; i < (int) mCoordinates.size(); ++i)
    {
        mOrderedNodes[i] = i;
    }

    std::sort(mOrderedNodes.begin(), mOrderedNodes.end(), mLessThanFunctor);
}

void KernelThenAMFilter::setBaseLayerIDToZeroAndOthersToMinusOne()
{
    for(int i = 0; i < (int) mCoordinates.size(); ++i)
    {
        mPseudoLayers[i] = -1;
    }

    for(auto tNode : mBaseLayer)
    {
        mPseudoLayers[tNode] = 0;
    }

    ++mNumPseudoLayers;
}

void KernelThenAMFilter::computeSupportSet()
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
                if(mLessThanFunctor(tNeighbor,tNode))
                {
                    if(isNeighborInCriticalWindow(tNode,tNeighbor))
                    {
                        mSupportSet[tNode].insert(SupportPointData({tNode, {tNeighbor}}));
                    }
                    else
                    {
                        for(int k = 1; k < 4; ++k)
                        {
                            if((i+j+k)%4 != i)
                            {
                                int tOtherNeighbor = tElement[(i+j+k)%4];
                                if(mLessThanFunctor(tOtherNeighbor,tNode) && isNeighborInCriticalWindow(tNode,tOtherNeighbor))
                                {
                                    mSupportSet[tNode].insert(SupportPointData({tNode, {tNeighbor,tOtherNeighbor}}));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

bool KernelThenAMFilter::isNeighborInCriticalWindow(const int& aNode, const int& aNeighbor) const
{
    PlatoSubproblemLibrary::Vector tNodeVector(mCoordinates[aNode]);
    PlatoSubproblemLibrary::Vector tNeighborVector(mCoordinates[aNeighbor]);

    PlatoSubproblemLibrary::Vector tVec = tNeighborVector - tNodeVector;

    return PlatoSubproblemLibrary::angle_between(tVec,-1*mBuildDirection) < mCriticalPrintAngle;
}

void KernelThenAMFilter::assignNodeToPseudoLayerAndPruneSupportSet(const int& aNode)
{
    std::set<int> tSupportingNeighbors = getSupportingNeighbors(aNode);
    std::set<int> tConnectedPseudoLayers = determineConnectedPseudoLayers(aNode, tSupportingNeighbors);
    int tSupportingPseudoLayer = determineSupportingPseudoLayer(aNode, tSupportingNeighbors, tConnectedPseudoLayers);

    assignNodeToPseudoLayer(aNode, tSupportingPseudoLayer);
}

std::set<int> KernelThenAMFilter::getSupportingNeighbors(const int& aNode) const
{
    std::set<int> tSupportingNeighbors;

    std::set<SupportPointData> tSupportSet = mSupportSet[aNode];

    for(auto tSupportPoint : tSupportSet)
    {
        std::set<int> tSupportingNeighborsForThisPoint = tSupportPoint.second;
        for(auto tNeighbor : tSupportingNeighborsForThisPoint)
        {
            tSupportingNeighbors.insert(tNeighbor);
        }
    }

    return tSupportingNeighbors;
}

std::set<int> KernelThenAMFilter::determineConnectedPseudoLayers(const int& aNode, const std::set<int>& aNeighbors) const
{
    std::set<int> tConnectedPseudoLayers;

    for(auto tNeighbor : aNeighbors)
    {
        if(mPseudoLayers[tNeighbor] == -1)
            throw(std::runtime_error("All nodes below the current node should already be assigned to a pseudo layer"));

        tConnectedPseudoLayers.insert(mPseudoLayers[tNeighbor]);
    }

    return tConnectedPseudoLayers;
}

int KernelThenAMFilter::determineSupportingPseudoLayer(const int& aNode, const std::set<int>& aNeighbors, const std::set<int>& aConnectedPseudoLayers) const
{
    int tSupportingPseudoLayer;
    std::vector<double> tSupportScores;

    for(auto tPseudoLayer : aConnectedPseudoLayers)
    {
        double tLayerSupportScore = 0;
        for(auto tNeighbor : aNeighbors)
        {
            if(mPseudoLayers[tNeighbor] == tPseudoLayer)
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

void KernelThenAMFilter::assignNodeToPseudoLayer(const int& aNode, const int& aSupportingPseudoLayer)
{
    mPseudoLayers[aNode] = aSupportingPseudoLayer + 1;
}

}
