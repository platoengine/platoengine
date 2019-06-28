/*
 * Plato_DesignVolume.cpp
 *
 *  Created on: Jun 28, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_InputData.hpp"
#include "Plato_DesignVolume.hpp"

namespace Plato
{

DesignVolume::DesignVolume(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp),
        mOutValueName("Design Volume")
{
}

void DesignVolume::operator()()
{
    double tVolume = mPlatoApp->getMeshServices()->getTotalVolume();
    std::vector<double>* tData = mPlatoApp->getValue(mOutValueName);
    (*tData)[0] = tVolume;

    return;
}

void DesignVolume::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg { Plato::data::layout_t::SCALAR, mOutValueName,/*length=*/1 });
}

}
// namespace Plato
