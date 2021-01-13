#include <PSL_BoundarySupportPoint.hpp>

namespace PlatoSubproblemLibrary
{

bool BoundarySupportPoint::operator<(const BoundarySupportPoint& aSupportPoint) const
{
    std::tuple<int, std::set<int>, int> tThisSupportPointData = {this->getSupportedNodeIndex(), this->getSupportingNodeIndices(), this->getIndexOnEdge()};
    std::tuple<int, std::set<int>, int> tInputSupportPointData = {aSupportPoint.getSupportedNodeIndex(), aSupportPoint.getSupportingNodeIndices(), aSupportPoint.getIndexOnEdge()};
    return tThisSupportPointData < tInputSupportPointData;
}

bool BoundarySupportPoint::operator==(const BoundarySupportPoint& aSupportPoint) const
{
    std::tuple<int, std::set<int>, int> tThisSupportPointData = {this->getSupportedNodeIndex(), this->getSupportingNodeIndices(), this->getIndexOnEdge()};
    std::tuple<int, std::set<int>, int> tInputSupportPointData = {aSupportPoint.getSupportedNodeIndex(), aSupportPoint.getSupportingNodeIndices(), aSupportPoint.getIndexOnEdge()};
    return tThisSupportPointData == tInputSupportPointData;
}

bool BoundarySupportPoint::operator!=(const BoundarySupportPoint& aSupportPoint) const
{
    std::tuple<int, std::set<int>, int> tThisSupportPointData = {this->getSupportedNodeIndex(), this->getSupportingNodeIndices(), this->getIndexOnEdge()};
    std::tuple<int, std::set<int>, int> tInputSupportPointData = {aSupportPoint.getSupportedNodeIndex(), aSupportPoint.getSupportingNodeIndices(), aSupportPoint.getIndexOnEdge()};
    return tThisSupportPointData != tInputSupportPointData;
}

double BoundarySupportPoint::getCorrespondingCoefficient(const int& aNeighbor) const
{
    if(!mCoefficientsSet)
    {
        throw(std::runtime_error("BoundarySupportPoint: Coefficients not yet set"));
    }
    auto tNeighborIterator = mSupportingNodesAsGiven.begin();
    auto tCoefficientIterator = mCoefficients.begin();

    for(; tNeighborIterator != mSupportingNodesAsGiven.end(); ++tNeighborIterator)
    {
        if(*tNeighborIterator == aNeighbor)
            return *tCoefficientIterator;

        ++tCoefficientIterator;
    }
    
    throw(std::domain_error("Support point does not depend on specified node"));
}

std::ostream & operator<< (std::ostream &out, const BoundarySupportPoint &aSupportPoint)
{
    std::pair<int, std::set<int>> tThisSupportPointData = {aSupportPoint.getSupportedNodeIndex(), aSupportPoint.getSupportingNodeIndices()};
    out << "{" << aSupportPoint.getSupportedNodeIndex() << ", {";
    
    auto tIndices = aSupportPoint.getSupportingNodeIndices();
    if(tIndices.size() == 1)
    {
        out << *(tIndices.begin());
    }
    else if(tIndices.size() == 2)
    {
        out << *(tIndices.begin()) << "," << *(++(tIndices.begin()));
    }
    else
    {
        throw(std::runtime_error("BoundarySupportPoint: Support point must depend on 1 or 2 nodes"));
    }

    out << "}}";

    return out;
}

}
