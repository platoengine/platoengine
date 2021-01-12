#include <PSL_BoundarySupportPoint.hpp>

namespace PlatoSubproblemLibrary
{

bool BoundarySupportPoint::operator<(const BoundarySupportPoint& aSupportPoint) const
{
    std::pair<int, std::set<int>> tThisSupportPointData = {this->getSupportedNodeIndex(), this->getSupportingNodeIndices()};
    std::pair<int, std::set<int>> tInputSupportPointData = {aSupportPoint.getSupportedNodeIndex(), aSupportPoint.getSupportingNodeIndices()};
    return tThisSupportPointData < tInputSupportPointData;
}

bool BoundarySupportPoint::operator==(const BoundarySupportPoint& aSupportPoint) const
{
    std::pair<int, std::set<int>> tThisSupportPointData = {this->getSupportedNodeIndex(), this->getSupportingNodeIndices()};
    std::pair<int, std::set<int>> tInputSupportPointData = {aSupportPoint.getSupportedNodeIndex(), aSupportPoint.getSupportingNodeIndices()};
    return tThisSupportPointData == tInputSupportPointData;
}

double BoundarySupportPoint::getCorrespondingCoefficient(const int& aNeighbor) const
{
    auto tNeighborIterator = mSupportingNodeIndices.begin();
    auto tCoefficientIterator = mCoefficients.begin();

    for(; tNeighborIterator != mSupportingNodeIndices.end(); ++tNeighborIterator)
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
    out << "{" << aSupportPoint.getSupportedNodeIndex() << ",{";
    
    for(int i : aSupportPoint.getSupportingNodeIndices())
    {
        out << i << ",";
    }

    out << '\b';
    out << "}}" << std::endl;

    return out;
}

}
