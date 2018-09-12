/*
 * Plato_GradFreeSimpleRocketObjective.cpp
 *
 *  Created on: Aug 30, 2018
 */

#include <map>
#include <string>
#include <numeric>
#include <cassert>

#include "Plato_GradFreeSimpleRocketObjective.hpp"

namespace Plato
{

GradFreeSimpleRocketObjective::GradFreeSimpleRocketObjective(const Plato::SimpleRocketInuts<double>& aRocketInputs,
                                                             const std::shared_ptr<Plato::GeometryModel<double>>& aChamberGeom) :
        PlatoSubproblemLibrary::DiscreteObjective(),
        mNormTargetValues(0),
        mRocketModel(aRocketInputs, aChamberGeom),
        mTargetThrustProfile(),
        mNumEvaluationsPerDim(),
        mBounds()
{
    this->initialize();
}

GradFreeSimpleRocketObjective::~GradFreeSimpleRocketObjective()
{
}

void GradFreeSimpleRocketObjective::setOptimizationInputs(const std::vector<int>& aNumEvaluationsPerDim,
                                                          const std::pair<std::vector<double>, std::vector<double>>& aBounds /* <lower,upper> */)
{
    mBounds = aBounds;
    mNumEvaluationsPerDim = aNumEvaluationsPerDim;
}

std::vector<double> GradFreeSimpleRocketObjective::getThrustProfile() const
{
    return (mRocketModel.getThrustProfile());
}

void GradFreeSimpleRocketObjective::get_domain(std::vector<double>& aLowerBounds,
                                               std::vector<double>& aUpperBounds,
                                               std::vector<int>& aNumEvaluationsPerDim)
{
    aLowerBounds = mBounds.first;
    aUpperBounds = mBounds.second;
    aNumEvaluationsPerDim = mNumEvaluationsPerDim;
}

double GradFreeSimpleRocketObjective::evaluate(const std::vector<double>& aControls)
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

void GradFreeSimpleRocketObjective::initialize()
{
    mTargetThrustProfile =
    {   0, 1656714.377766964, 1684717.520617273, 1713123.001583093, 1741935.586049868,
        1771160.083875437, 1800801.349693849, 1830864.28322051, 1861353.829558637,
        1892274.979507048, 1923632.769869272, 1955432.283763989, 1987678.650936801,
        2020377.048073344, 2053532.699113719, 2087150.875568287, 2121236.896834771,
        2155796.130516737, 2190833.992743404, 2226355.948490792, 2262367.511904243,
        2298874.246622283, 2335881.766101836, 2373395.733944806, 2411421.864226017,
        2449965.921822503, 2489033.722744186, 2528631.134465915, 2568764.076260844,
        2609438.519535244, 2650660.488164633, 2692436.058831303, 2734771.361363255,
        2777672.579074459, 2821145.949106557, 2865197.762771913, 2909834.365898075,
        2955062.159173611, 3000887.598495364, 3047317.195317072, 3094357.516999425,
        3142015.18716148, 3190296.886033527, 3239209.350811319, 3288759.376011737,
        3338953.813829865, 3389799.574497465, 3441303.626642879, 3493472.997652346,
        3546314.774032734, 3599836.101775718, 3654044.186723352, 3708946.294935087,
        3764549.753056224, 3820861.948687783, 3877890.330757833, 3935642.409894215,
        3994125.758798767, 4053348.012622938, 4113316.869344868, 4174040.090147917,
        4235525.499800648, 4297780.987038235, 4360814.504945371, 4424634.071340578,
        4489247.76916203, 4554663.746854796, 4620890.218759571, 4687935.465502855,
        4755807.834388626, 4824515.739791448, 4894067.663551098, 4964472.155368621,
        5035737.83320389, 5107873.383674653, 5180887.562457044, 5254789.194687578,
        5329587.175366664, 5405290.469763565, 5481908.11382287, 5559449.214572486,
        5637922.950533082, 5717338.572129052, 5797705.402100981, 5879032.835919643,
        5961330.342201422, 6044607.46312535, 6128873.814851565, 6214139.087941348,
        6300413.047778608, 6387705.534992979, 6476026.465884338, 6565385.832848894,
        6655793.704806847, 6747260.227631442, 6839795.624579719, 6933410.196724654,
        7028114.32338894, 7123918.462580209, 7220833.151427887};

    const double tBaseValue = 0;
    mNormTargetValues =
    std::inner_product(mTargetThrustProfile.begin(), mTargetThrustProfile.end(), mTargetThrustProfile.begin(), tBaseValue);
    mRocketModel.disableOutput();
}

void GradFreeSimpleRocketObjective::update(const std::vector<double>& aControls)
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
