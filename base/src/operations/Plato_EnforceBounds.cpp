/*
 * Plato_EnforceBounds.cpp
 *
 *  Created on: Jun 29, 2019
 */

#include <algorithm>

#include "PlatoApp.hpp"
#include "Plato_InputData.hpp"
#include "Plato_EnforceBounds.hpp"

namespace Plato
{

EnforceBounds::EnforceBounds(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp)
{
    mLowerBoundVectorFieldName = "Lower Bound Vector";
    mUpperBoundVectorFieldName = "Upper Bound Vector";
    mTopologyFieldName = "Topology";
}

void EnforceBounds::operator()()
{
    // Get the output field
    double* tOutputData;
    double* tLowerBoundData;
    double* tUpperBoundData;
    int tDataLenth = 0;

    auto& tOutputField = *(mPlatoApp->getNodeField(mTopologyFieldName));
    tOutputField.ExtractView(&tOutputData);
    tDataLenth = tOutputField.MyLength();

    auto& tLowerBoundField = *(mPlatoApp->getNodeField(mLowerBoundVectorFieldName));
    tLowerBoundField.ExtractView(&tLowerBoundData);

    auto& tUpperBoundField = *(mPlatoApp->getNodeField(mUpperBoundVectorFieldName));
    tUpperBoundField.ExtractView(&tUpperBoundData);

    for(int tIndex = 0; tIndex < tDataLenth; tIndex++)
    {
        tOutputData[tIndex] = std::max(tOutputData[tIndex], tLowerBoundData[tIndex]);
        tOutputData[tIndex] = std::min(tOutputData[tIndex], tUpperBoundData[tIndex]);
    }
}

void EnforceBounds::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg(Plato::data::layout_t::SCALAR_FIELD, mLowerBoundVectorFieldName));
    aLocalArgs.push_back(Plato::LocalArg(Plato::data::layout_t::SCALAR_FIELD, mUpperBoundVectorFieldName));
    aLocalArgs.push_back(Plato::LocalArg(Plato::data::layout_t::SCALAR_FIELD, mTopologyFieldName));
}

}
// namespace Plato
