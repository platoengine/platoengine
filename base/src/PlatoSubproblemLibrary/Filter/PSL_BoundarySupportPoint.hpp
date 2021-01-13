// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* Class: Kernel then AM filter for density method topology optimization.
*
* Smooth a field, and then apply an AM Filter to ensure printability of the materials.
*/

// #include "PSL_Filter.hpp"
// #include "PSL_AbstractKernelThenFilter.hpp"
// #include "PSL_ParameterData.hpp"
// #include "PSL_ParameterDataEnums.hpp"
// #include "PSL_Vector.hpp"
// #include "PSL_Point.hpp"
// #include "PSL_PseudoLayerBuilder.hpp"


#include <vector>
#include <set>
#include <stdexcept>
// #include <map>
// #include <cstddef>
// #include <cassert>
#include <iostream>

namespace PlatoSubproblemLibrary
{

class BoundarySupportPoint
{
public:
    BoundarySupportPoint(int aSupportedNodeIndex,
                         std::vector<int> aSupportingNodeIndices,
                         std::vector<double> aCoefficients,
                         int aMetaIndex = 0)
                         : mSupportedNodeIndex(aSupportedNodeIndex),
                           mSupportingNodesAsGiven(aSupportingNodeIndices),
                           mCoefficients(aCoefficients),
                           mMetaIndex(aMetaIndex)
    
    {
        if(aSupportingNodeIndices.size() != aCoefficients.size())
        {
            throw(std::domain_error("BoundarySupportPoint: Number of supporting nodes must match the number of coefficients"));
        }
        if(aSupportingNodeIndices.size() == 0 || aSupportingNodeIndices.size() > 2)
        {
            throw(std::domain_error("BoundarySupportPoint: A boundary support point must depend on one or 2 nodes"));
        }
        if(aMetaIndex != 0 && aMetaIndex != 1)
        {
            throw(std::domain_error("BoundarySupportPoint: Index on edge must be 0 or 1"));
        }
        if(aMetaIndex != 0 && aSupportingNodeIndices.size() < 2)
        {
            throw(std::domain_error("BoundarySupportPoint: Index on edge can only be non-zero for support points on an edge"));
        }
            
        addToSupportingNodeSet(aSupportingNodeIndices);
        mCoefficientsSet = true;
    }

    BoundarySupportPoint(int aSupportedNodeIndex,
                         std::vector<int> aSupportingNodeIndices,
                         int aMetaIndex = 0)
                         : mSupportedNodeIndex(aSupportedNodeIndex),
                           mSupportingNodesAsGiven(aSupportingNodeIndices),
                           mMetaIndex(aMetaIndex)
    {
        if(aSupportingNodeIndices.size() == 0 || aSupportingNodeIndices.size() > 2)
        {
            throw(std::domain_error("BoundarySupportPoint: A boundary support point must depend on one or 2 nodes"));
        }
        if(aMetaIndex != 0 && aMetaIndex != 1)
        {
            throw(std::domain_error("BoundarySupportPoint: Index on edge must be 0 or 1 "));
        }
        if(aMetaIndex != 0 && aSupportingNodeIndices.size() < 2)
        {
            throw(std::domain_error("BoundarySupportPoint: Index on edge can only be non-zero for support points on an edge "));
        }
            
        addToSupportingNodeSet(aSupportingNodeIndices);
    }

    BoundarySupportPoint() = delete;
    ~BoundarySupportPoint(){}

    int getSupportedNodeIndex() const {return mSupportedNodeIndex;}
    std::set<int> getSupportingNodeIndices() const {return mSupportingNodeIndices;}
    std::vector<double> getCoefficients() const {if(mCoefficientsSet) return mCoefficients; else throw(std::runtime_error("BoundarySupportPoint: Coefficients not yet set"));}
    int getIndexOnEdge() const {return mMetaIndex;}
    void setCoefficients(std::vector<double> aCoefficients)
    {
        if(mSupportingNodeIndices.size() != aCoefficients.size())
        {
            throw(std::domain_error("BoundarySupportPoint: Number of supporting nodes must match the number of coefficients"));
        }
        mCoefficients = aCoefficients;
        mCoefficientsSet = true;
    }

    double getCorrespondingCoefficient(const int& aNeighbor) const;

    bool operator<(const BoundarySupportPoint& aSupportPoint) const;
    friend std::ostream & operator<< (std::ostream &out, const BoundarySupportPoint &c);
    bool operator==(const BoundarySupportPoint& aSupportPoint) const;
    bool operator!=(const BoundarySupportPoint& aSupportPoint) const;

private:

    void addToSupportingNodeSet(std::vector<int>& aSupportingNodeIndices)
    {
        for(auto tIndex : aSupportingNodeIndices)
        {
            if(!mSupportingNodeIndices.insert(tIndex).second)
            {
                throw(std::domain_error("BoundarySupportPoint: Input supporting node indices are not unique"));
            }
            if(tIndex == mSupportedNodeIndex)
            {
                throw(std::domain_error("BoundarySupportPoint: A node cannot support itself"));
            }
        }
    }

    int mSupportedNodeIndex;
    std::set<int> mSupportingNodeIndices; // set used to identify unique BoundarySupportPoints

    std::vector<int> mSupportingNodesAsGiven; // vector used to retrieve coefficients in same order as they were given
    std::vector<double> mCoefficients;
    bool mCoefficientsSet = false;

    int mMetaIndex = 0; // may be set to 1 in order to distinguish the second support points when the critical window crosses the edge twice
};

}
