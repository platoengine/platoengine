/*
 * Plato_Roughness.cpp
 *
 *  Created on: Jun 30, 2019
 */

#include "mesh_services.hpp"

#include "PlatoApp.hpp"
#include "Plato_Roughness.hpp"
#include "Plato_InputData.hpp"

namespace Plato
{

Roughness::Roughness(PlatoApp* aPlatoAppp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoAppp),
        mTopologyName("Topology"),
        mRoughnessName("Roughness"),
        mGradientName("Roughness Gradient")
{
}

void Roughness::operator()()
{
    // Input:  Topology
    // Output: Roughness, Roughness Gradient

    // get local topology
    auto& tTopology = *(mPlatoApp->getNodeField(mTopologyName));

    // get local roughness
    std::vector<double>* tData = mPlatoApp->getValue(mRoughnessName);
    double& tRoughnessValue = (*tData)[0];

    // get local gradient
    auto& tRoughnessGradient = *(mPlatoApp->getNodeField(mGradientName));
    mPlatoApp->getMeshServices()->getRoughness(tTopology, tRoughnessValue, tRoughnessGradient);

    return;
}

void Roughness::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg
        { Plato::data::layout_t::SCALAR_FIELD, mTopologyName });
    aLocalArgs.push_back(Plato::LocalArg
        { Plato::data::layout_t::SCALAR, mRoughnessName,/*length=*/1 });
    aLocalArgs.push_back(Plato::LocalArg
        { Plato::data::layout_t::SCALAR_FIELD, mGradientName });
}

}
// namespace Plato
