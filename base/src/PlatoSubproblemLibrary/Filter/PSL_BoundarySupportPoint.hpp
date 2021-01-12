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
                         std::set<int> aSupportingNodeIndices,
                         std::vector<double> aCoefficients,
                         int aMetaIndex = 0)
                         : mSupportedNodeIndex(aSupportedNodeIndex),
                           mSupportingNodeIndices(aSupportingNodeIndices),
                           mCoefficients(aCoefficients)
    
    {}

    BoundarySupportPoint(int aSupportedNodeIndex,
                         std::set<int> aSupportingNodeIndices)
                         : mSupportedNodeIndex(aSupportedNodeIndex),
                           mSupportingNodeIndices(aSupportingNodeIndices)
    
    {
        mCoefficients.clear();
    }

    BoundarySupportPoint()
    {
        mSupportedNodeIndex = 0;
        mSupportingNodeIndices.clear();
        mCoefficients.clear();
    }

    ~BoundarySupportPoint(){}

    int getSupportedNodeIndex() const {return mSupportedNodeIndex;}
    std::set<int> getSupportingNodeIndices() const {return mSupportingNodeIndices;}
    std::vector<double> getCoefficients() const {return mCoefficients;}

    double getCorrespondingCoefficient(const int& aNeighbor) const;

    bool operator<(const BoundarySupportPoint& aSupportPoint) const;
    friend std::ostream & operator<< (std::ostream &out, const BoundarySupportPoint &c);
    bool operator==(const BoundarySupportPoint& aSupportPoint) const;

private:

    int mSupportedNodeIndex;
    std::set<int> mSupportingNodeIndices;
    std::vector<double> mCoefficients;

};

}
