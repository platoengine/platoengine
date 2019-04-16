/*
 * Plato_GradFreeRocketObjective.cpp
 *
 *  Created on: Aug 30, 2018
 */

#include <map>
#include <string>
#include <numeric>
#include <cassert>

#include "Plato_GradFreeRocketObjective.hpp"

namespace Plato
{

GradFreeRocketObjective::GradFreeRocketObjective(const Plato::AlgebraicRocketInputs<double>& aRocketInputs,
                                                 const std::shared_ptr<Plato::GeometryModel<double>>& aChamberGeom) :
        PlatoSubproblemLibrary::DiscreteObjective(),
        mNormTargetValues(0),
        mTargetThrustProfile(),
        mNumEvaluationsPerDim(),
        mRocketModel(aRocketInputs, aChamberGeom),
        mBounds()
{
}

GradFreeRocketObjective::~GradFreeRocketObjective()
{
}

void GradFreeRocketObjective::setOptimizationInputs(const std::vector<int>& aNumEvaluationsPerDim,
                                                    const std::pair<std::vector<double>, std::vector<double>>& aBounds /* <lower,upper> */)
{
    mBounds = aBounds;
    mNumEvaluationsPerDim = aNumEvaluationsPerDim;
}

std::vector<double> GradFreeRocketObjective::getThrustProfile() const
{
    return (mRocketModel.getThrustProfile());
}

void GradFreeRocketObjective::get_domain(std::vector<double>& aLowerBounds,
                                         std::vector<double>& aUpperBounds,
                                         std::vector<int>& aNumEvaluationsPerDim)
{
    aLowerBounds = mBounds.first;
    aUpperBounds = mBounds.second;
    aNumEvaluationsPerDim = mNumEvaluationsPerDim;
}

double GradFreeRocketObjective::evaluate(const std::vector<double>& aControls)
{
    this->update(aControls);

    mRocketModel.solve();
    std::vector<double> tSimulationThrustProfile = mRocketModel.getThrustProfile();
    assert(tSimulationThrustProfile.size() == mTargetThrustProfile.size());

    double tObjectiveValue = 0;
    for(size_t tIndex = 0; tIndex < mTargetThrustProfile.size(); tIndex++)
    {
        double tDeltaThrust = tSimulationThrustProfile[tIndex] - mTargetThrustProfile[tIndex];
        tObjectiveValue += tDeltaThrust * tDeltaThrust;
    }
    const double tNumElements = mTargetThrustProfile.size();
    tObjectiveValue = static_cast<double>(1.0 / (2.0 * tNumElements * mNormTargetValues)) * tObjectiveValue;

    return tObjectiveValue;
}

void GradFreeRocketObjective::initialize(const std::vector<double>& aTarget)
{
    mTargetThrustProfile = aTarget;
    const double tBaseValue = 0;
    mNormTargetValues =
    std::inner_product(mTargetThrustProfile.begin(), mTargetThrustProfile.end(), mTargetThrustProfile.begin(), tBaseValue);
    mRocketModel.disableOutput();
}

void GradFreeRocketObjective::update(const std::vector<double>& aControls)
{
    std::map<std::string, double> tSimParam;
    tSimParam.insert(std::pair<std::string, double>("RefBurnRate", aControls[1]));
    mRocketModel.updateSimulation(tSimParam);
    std::map<std::string, double> tChamberGeomParam;
    tChamberGeomParam.insert(std::pair<std::string, double>("Radius", aControls[0]));
    tChamberGeomParam.insert(std::pair<std::string, double>("Configuration", Plato::Configuration::INITIAL));
    mRocketModel.updateInitialChamberGeometry(tChamberGeomParam);
}

} // namespace Plato
