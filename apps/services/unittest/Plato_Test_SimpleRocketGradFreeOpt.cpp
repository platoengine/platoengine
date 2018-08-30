/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

/*
 * Plato_Test_SimpleRocketGradFreeOpt.cpp
 *
 *  Created on: Aug 29, 2018
 */

#include "gtest/gtest.h"

#include <map>
#include <vector>
#include <memory>
#include <utility>
#include <cassert>
#include <cstddef>

#include "Plato_SimpleRocket.hpp"
#include "Plato_CircularCylinder.hpp"

#include "PSL_DiscreteObjective.hpp"

namespace Plato
{

class SimpleRocketObjectiveGradFree : public PlatoSubproblemLibrary::DiscreteObjective
{
public:
    SimpleRocketObjectiveGradFree(const Plato::SimpleRocketInuts<double>& aRocketInputs,
                                  const std::shared_ptr<Plato::GeometryModel<double>>& aGeomModel) :
            PlatoSubproblemLibrary::DiscreteObjective(),
            mRocketModel(aRocketInputs, aGeomModel),
            mTargetThrustProfile(),
            mNumEvaluationsPerDim(),
            mBounds()
    {
        this->initialize();
    }

    virtual ~SimpleRocketObjectiveGradFree()
    {
    }

    void setOptimizationInputs(const std::vector<int>& aNumEvaluationsPerDim,
                               const std::pair<std::vector<double>, std::vector<double>>& aBounds /* <lower,upper> */)
    {
        mBounds = aBounds;
        mNumEvaluationsPerDim = aNumEvaluationsPerDim;
    }

    void get_domain(std::vector<double>& aLowerBounds, std::vector<double>& aUpperBounds, std::vector<int>& aNumEvaluationsPerDim)
    {
        aLowerBounds = mBounds.first;
        aUpperBounds = mBounds.second;
        aNumEvaluationsPerDim = mNumEvaluationsPerDim;
    }

    double evaluate(const std::vector<double>& aControls)
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
        tObjectiveValue = static_cast<double>(0.5) * tObjectiveValue;

        return tObjectiveValue;
    }

private:
    /******************************************************************************//**
     * @brief Set target data and disable console output.
     **********************************************************************************/
    void initialize()
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
        mRocketModel.disableOutput();
    }

    /******************************************************************************//**
     * @brief update parameters (e.g. design variables) for simulation.
     * @param aControls design variables
     **********************************************************************************/
    void update(const std::vector<double>& aControls)
    {
        std::map<std::string, double> tSimParam;
        tSimParam.insert(std::pair<std::string, double>("RefBurnRate",aControls[1]));
        mRocketModel.updateSimulation(tSimParam);
        std::map<std::string, double> tChamberGeomParam;
        tChamberGeomParam.insert(std::pair<std::string, double>("Radius",aControls[0]));
        tChamberGeomParam.insert(std::pair<std::string, double>("Configuration", Plato::Configuration::INITIAL));
        mRocketModel.updateInitialChamberGeometry(tChamberGeomParam);
    }

private:
    Plato::SimpleRocket<double> mRocketModel;
    std::vector<double> mTargetThrustProfile;

    std::vector<int> mNumEvaluationsPerDim;
    std::pair<std::vector<double>, std::vector<double>> mBounds; /* <lower,upper> */
};
// class SimpleRocketObjectiveGradFree

} // namespace Plato

namespace PlatoTest
{

TEST(PlatoTest, SimpleRocketObjectiveGradFree)
{
    // allocate problem inputs - use default parameters
    Plato::SimpleRocketInuts<double> tRocketInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
            std::make_shared<Plato::CircularCylinder<double>>(tRocketInputs.mChamberRadius, tRocketInputs.mChamberLength);

    // domain dimension = 10x10=100
    std::vector<int> tNumEvaluationsPerDim = {100, 100};

    /* {chamber_radius_lb, ref_burn_rate_lb},  {chamber_radius_ub, ref_burn_rate_ub} */
    std::pair<std::vector<double>, std::vector<double>> aBounds =
            std::make_pair<std::vector<double>, std::vector<double>>({0.07, 0.004},{0.08, 0.006});
    Plato::SimpleRocketObjectiveGradFree tObjective(tRocketInputs, tGeomModel);
    tObjective.setOptimizationInputs(tNumEvaluationsPerDim, aBounds);

    /* {chamber_radius, ref_burn_rate} */
    std::vector<double> tControls = {0.075, 0.005};
    double tValue = tObjective.evaluate(tControls);

    // test objective function evaluation
    double tGold = 0;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tValue, tGold, tTolerance);

    // test inputs for gradient-free algorithm
    std::vector<double> tLowerBounds;
    std::vector<double> tUpperBounds;
    std::vector<int> tEvaluationsPerDim;
    tObjective.get_domain(tLowerBounds, tUpperBounds, tEvaluationsPerDim);

    for(size_t tIndex = 0; tIndex < tNumEvaluationsPerDim.size(); tIndex++)
    {
        ASSERT_DOUBLE_EQ(tLowerBounds[tIndex], aBounds.first[tIndex]);
        ASSERT_DOUBLE_EQ(tUpperBounds[tIndex], aBounds.second[tIndex]);
        EXPECT_EQ(tEvaluationsPerDim[tIndex], tNumEvaluationsPerDim[tIndex]);
    }
}

TEST(PlatoTest, CircularCylinder)
{
    // allocate problem inputs - use default parameters
    const double tRadius = 1;
    const double tLength = 2;
    Plato::CircularCylinder<double> tCylinder(tRadius, tLength);

    // test area calculation
    double tTolerance = 1e-6;
    double tArea = tCylinder.area();
    EXPECT_NEAR(tArea, 12.566370614359172, tTolerance);

    // test gradient
    const size_t tNumControls = 2;
    std::vector<double> tGrad(tNumControls);
    tCylinder.gradient(tGrad);
    EXPECT_NEAR(tGrad[0], 12.56637061435917, tTolerance);
    EXPECT_NEAR(tGrad[1], 6.283185307179586, tTolerance);

    // test update initial configuration
    std::map<std::string, double> tParam;
    tParam.insert(std::pair<std::string, double>("Radius", 2));
    tParam.insert(std::pair<std::string, double>("Configuration", Plato::Configuration::INITIAL));
    tCylinder.update(tParam);
    tArea = tCylinder.area();
    EXPECT_NEAR(tArea, 25.132741228718345, tTolerance);

    // test update dynamics configuration
    tParam.insert(std::pair<std::string, double>("BurnRate", 10));
    tParam.insert(std::pair<std::string, double>("DeltaTime", 0.1));
    tParam.find("Configuration")->second = Plato::Configuration::DYNAMIC;
    tCylinder.update(tParam);
    tArea = tCylinder.area();
    EXPECT_NEAR(tArea, 37.699111843077520, tTolerance);
}

} // namespace PlatoTest
