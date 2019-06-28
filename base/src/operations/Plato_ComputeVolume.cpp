/*
 * Plato_ComputeVolume.cpp
 *
 *  Created on: Jun 28, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_InputData.hpp"
#include "Plato_PenaltyModel.hpp"
#include "Plato_ComputeVolume.hpp"

namespace Plato
{

ComputeVolume::ComputeVolume(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp),
        mTopologyName("Topology"),
        mVolumeName("Volume"),
        mGradientName("Volume Gradient"),
        mPenaltyModel(nullptr)
{
    Plato::PenaltyModelFactory tPenaltyModelFactory;
    mPenaltyModel = tPenaltyModelFactory.create(aNode);
}

ComputeVolume::~ComputeVolume()
{
    if(mPenaltyModel)
    {
        delete mPenaltyModel;
        mPenaltyModel = nullptr;
    }
}

void ComputeVolume::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg {Plato::data::layout_t::SCALAR_FIELD, mTopologyName});
    aLocalArgs.push_back(Plato::LocalArg {Plato::data::layout_t::SCALAR, mVolumeName,/*length=*/1});
    aLocalArgs.push_back(Plato::LocalArg {Plato::data::layout_t::SCALAR_FIELD, mGradientName});
}

void ComputeVolume::operator()()
{
    // Input:  Topology
    // Output: Volume, Volume Gradient

    // get local topology
    auto& tTopology = *(mPlatoApp->getNodeField(mTopologyName));

    // get local volume
    std::vector<double>* tData = mPlatoApp->getValue(mVolumeName);
    double& tVolumeValue = (*tData)[0];

    // get local gradient
    auto& tVolumeGradient = *(mPlatoApp->getNodeField(mGradientName));

    mPlatoApp->getMeshServices()->getCurrentVolume(tTopology, tVolumeValue, tVolumeGradient);

    return;
}

}
// namespace Plato
