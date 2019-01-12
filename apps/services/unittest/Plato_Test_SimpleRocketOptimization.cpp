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
 * Plato_Test_SimpleRocketOptimization.cpp
 *
 *  Created on: Aug 29, 2018
 */

#include "gtest/gtest.h"

#include "Plato_GradFreeRocketObjective.hpp"

#include "Plato_StandardVector.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_GradBasedRocketObjective.hpp"
#include "Plato_KelleySachsBoundLightInterface.hpp"

#include "PSL_Random.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_IterativeSelection.hpp"

namespace PlatoTest
{

/******************************************************************************//**
 * @brief Return target thrust profile for gradient-based unit test
 * @return standard vector with target thrust profile
**********************************************************************************/
std::vector<double> get_target_thrust_profile()
{
    std::vector<double> tTargetThrustProfile =
        { 0, 1656714.377766964, 1684717.520617273, 1713123.001583093, 1741935.586049868, 1771160.083875437,
                1800801.349693849, 1830864.28322051, 1861353.829558637, 1892274.979507048, 1923632.769869272,
                1955432.283763989, 1987678.650936801, 2020377.048073344, 2053532.699113719, 2087150.875568287,
                2121236.896834771, 2155796.130516737, 2190833.992743404, 2226355.948490792, 2262367.511904243,
                2298874.246622283, 2335881.766101836, 2373395.733944806, 2411421.864226017, 2449965.921822503,
                2489033.722744186, 2528631.134465915, 2568764.076260844, 2609438.519535244, 2650660.488164633,
                2692436.058831303, 2734771.361363255, 2777672.579074459, 2821145.949106557, 2865197.762771913,
                2909834.365898075, 2955062.159173611, 3000887.598495364, 3047317.195317072, 3094357.516999425,
                3142015.18716148, 3190296.886033527, 3239209.350811319, 3288759.376011737, 3338953.813829865,
                3389799.574497465, 3441303.626642879, 3493472.997652346, 3546314.774032734, 3599836.101775718,
                3654044.186723352, 3708946.294935087, 3764549.753056224, 3820861.948687783, 3877890.330757833,
                3935642.409894215, 3994125.758798767, 4053348.012622938, 4113316.869344868, 4174040.090147917,
                4235525.499800648, 4297780.987038235, 4360814.504945371, 4424634.071340578, 4489247.76916203,
                4554663.746854796, 4620890.218759571, 4687935.465502855, 4755807.834388626, 4824515.739791448,
                4894067.663551098, 4964472.155368621, 5035737.83320389, 5107873.383674653, 5180887.562457044,
                5254789.194687578, 5329587.175366664, 5405290.469763565, 5481908.11382287, 5559449.214572486,
                5637922.950533082, 5717338.572129052, 5797705.402100981, 5879032.835919643, 5961330.342201422,
                6044607.46312535, 6128873.814851565, 6214139.087941348, 6300413.047778608, 6387705.534992979,
                6476026.465884338, 6565385.832848894, 6655793.704806847, 6747260.227631442, 6839795.624579719,
                6933410.196724654, 7028114.32338894, 7123918.462580209, 7220833.151427887 };

    return (tTargetThrustProfile);
}
// function get_target_thrust_profile

TEST(PlatoTest, GradBasedRocketObjectiveGradFree)
{
    // allocate problem inputs - use default parameters
    Plato::AlgebraicRocketInputs<double> tRocketInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
            std::make_shared<Plato::Cylinder<double>>(tRocketInputs.mChamberRadius, tRocketInputs.mChamberLength);

    // domain dimension = 10x10=100
    std::vector<int> tNumEvaluationsPerDim = {100, 100};

    /* {chamber_radius_lb, ref_burn_rate_lb},  {chamber_radius_ub, ref_burn_rate_ub} */
    std::pair<std::vector<double>, std::vector<double>> tBounds =
            std::make_pair<std::vector<double>, std::vector<double>>({0.07, 0.004},{0.08, 0.006});
    Plato::GradFreeRocketObjective tObjective(tRocketInputs, tGeomModel);
    tObjective.setOptimizationInputs(tNumEvaluationsPerDim, tBounds);

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
        ASSERT_DOUBLE_EQ(tLowerBounds[tIndex], tBounds.first[tIndex]);
        ASSERT_DOUBLE_EQ(tUpperBounds[tIndex], tBounds.second[tIndex]);
        EXPECT_EQ(tEvaluationsPerDim[tIndex], tNumEvaluationsPerDim[tIndex]);
    }
}

TEST(PlatoTest, GradBasedRocketObjective)
{
    // allocate problem inputs - use default parameters
    Plato::AlgebraicRocketInputs<double> tRocketInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
            std::make_shared<Plato::Cylinder<double>>(tRocketInputs.mChamberRadius, tRocketInputs.mChamberLength);

    // set normalization constants
    std::vector<double> tTargetThrustProfile = PlatoTest::get_target_thrust_profile();
    Plato::GradBasedRocketObjective<double> tObjective(tTargetThrustProfile, tRocketInputs, tGeomModel);
    tObjective.disableObjectiveNormalization();
    std::vector<double> tUpperBounds = {0.09, 0.007}; /* {chamber_radius_ub, ref_burn_rate_ub} */
    tObjective.setNormalizationConstants(tUpperBounds);

    // evaluate criterion
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls);
    // set control multi-vector
    const size_t tVECTOR_INDEX = 0;
    tControl(tVECTOR_INDEX, 0) = static_cast<double>(0.075) / tUpperBounds[0];
    tControl(tVECTOR_INDEX, 1) = static_cast<double>(0.005) / tUpperBounds[1];
    const double tObjValue = tObjective.value(tControl);

    // test objective function evaluation
    double tGold = 0;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tObjValue, tGold, tTolerance);

    // evaluate gradient
    Plato::StandardMultiVector<double> tGradient(tNumVectors, tNumControls);
    tControl(tVECTOR_INDEX, 0) = static_cast<double>(0.085) / tUpperBounds[0];
    tControl(tVECTOR_INDEX, 1) = static_cast<double>(0.004) / tUpperBounds[1];
    tObjective.setPerturbationParameter(1e-4);
    tObjective.gradient(tControl, tGradient);
    // test gradient evaluation
    EXPECT_NEAR(tGradient(tVECTOR_INDEX,0), -0.0048183366343682423, tTolerance);
    EXPECT_NEAR(tGradient(tVECTOR_INDEX,1), -0.0057808715527571993, tTolerance);

    // evaluate hessian
    double tValue = 1;
    Plato::StandardMultiVector<double> tHessian(tNumVectors, tNumControls);
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls, tValue);
    tObjective.hessian(tControl, tVector, tHessian);
    // test hessian evaluation
    EXPECT_NEAR(tHessian(tVECTOR_INDEX,0), 1., tTolerance);
    EXPECT_NEAR(tHessian(tVECTOR_INDEX,1), 1., tTolerance);
}

TEST(PlatoTest, Cylinder)
{
    // allocate problem inputs - use default parameters
    const double tRadius = 1;
    const double tLength = 2;
    Plato::Cylinder<double> tCylinder(tRadius, tLength);

    // test area calculation
    double tTolerance = 1e-6;
    double tArea = tCylinder.area();
    EXPECT_NEAR(tArea, 12.566370614359172, tTolerance);

    // test gradient
    const size_t tNumControls = 2;
    std::vector<double> tGrad(tNumControls);
    tCylinder.gradient(tGrad);
    EXPECT_NEAR(tGrad[0], 12.56637061435917, tTolerance);EXPECT_NEAR(tGrad[1], 6.283185307179586, tTolerance);

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

TEST(PlatoTest, GradFreeSimpleRocketOptimization)
{
    // define objective
    Plato::AlgebraicRocketInputs<double> tRocketInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
    std::make_shared<Plato::Cylinder<double>>(tRocketInputs.mChamberRadius, tRocketInputs.mChamberLength);
    Plato::GradFreeRocketObjective tObjective(tRocketInputs, tGeomModel);;

    // set inputs for optimization problem
    std::vector<int> tNumEvaluationsPerDim = {100, 100}; // domain dimension = 100x100=10000
    /* {chamber_radius_lb, ref_burn_rate_lb},  {chamber_radius_ub, ref_burn_rate_ub} */
    std::pair<std::vector<double>, std::vector<double>> aBounds =
    std::make_pair<std::vector<double>, std::vector<double>>( {0.07, 0.004}, {0.08, 0.006});
    tObjective.setOptimizationInputs(tNumEvaluationsPerDim, aBounds);

    // define searcher
    PlatoSubproblemLibrary::AbstractAuthority tAuthority;
    PlatoSubproblemLibrary::IterativeSelection tSearcher(&tAuthority);
    tSearcher.set_objective(&tObjective);

    // find a minimum
    std::vector<double> tBestParameters;
    PlatoSubproblemLibrary::set_rand_seed();
    const double tBestObjectiveValue = tSearcher.find_min(tBestParameters);
    std::cout << "BestObjectiveValue = " << tBestObjectiveValue << "\n";
    std::cout << "NumFunctionEvaluations = " << tSearcher.get_number_of_evaluations() << "\n";
    std::cout << "Best1: " << tBestParameters[0] << ", Best2: " << tBestParameters[1] << std::endl;

    // equal by determinism of objective
    EXPECT_FLOAT_EQ(tBestObjectiveValue, tObjective.evaluate(tBestParameters));
}

TEST(PlatoTest, GradBasedSimpleRocketOptimizationWithLightInterface)
{
    // ********* SET NORMALIZATION CONSTANTS *********
    const size_t tNumControls = 2;
    std::vector<double> tNormalizationConstants(tNumControls);
    tNormalizationConstants[0] = 0.08; tNormalizationConstants[1] = 0.006;

    // ********* ALLOCATE OBJECTIVE AND SET TARGET THRSUT PROFILE *********
    Plato::AlgebraicRocketInputs<double> tRocketInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
            std::make_shared<Plato::Cylinder<double>>(tRocketInputs.mChamberRadius, tRocketInputs.mChamberLength);
    std::shared_ptr<Plato::GradBasedRocketObjective<double>> tThrustMisfitObjective =
            std::make_shared<Plato::GradBasedRocketObjective<double>>(tRocketInputs, tGeomModel);
    tThrustMisfitObjective->setNormalizationConstants(tNormalizationConstants);
    std::vector<double> tTargetThrustProfile = PlatoTest::get_target_thrust_profile();
    tThrustMisfitObjective->setTargetThrustProfile(tTargetThrustProfile);

    // ********* SET LIST OF OBJECTIVE FUNCTIONS *********
    std::shared_ptr<Plato::CriterionList<double>> tMyObjective = std::make_shared<Plato::CriterionList<double>>();
    tMyObjective->add(tThrustMisfitObjective);

    // ********* SET OPTIMIZATION ALGORITHM INPUTS *********
    const size_t tNumVectors = 1;
    Plato::AlgorithmInputsKSBC<double> tInputs;
    tInputs.mHessianMethod = Plato::Hessian::DISABLED;
    tInputs.mMinTrustRegionRadius = 1e-6;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls);
    (*tInputs.mLowerBounds)(0,0) = 0.06 / tNormalizationConstants[0]; 
    (*tInputs.mLowerBounds)(0,1) = 0.003 / tNormalizationConstants[1];
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 1.0 /* base value */);

    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls);
    (*tInputs.mInitialGuess)(0,0) = 0.074 / tNormalizationConstants[0]; 
    (*tInputs.mInitialGuess)(0,1) = 0.0055 / tNormalizationConstants[1];

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::AlgorithmOutputsKSBC<double> tOutputs;
    Plato::solve_ksbc<double, size_t>(tMyObjective, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    const double tBest1 = (*tOutputs.mSolution)(0,0) * tNormalizationConstants[0];
    const double tBest2 = (*tOutputs.mSolution)(0,1) * tNormalizationConstants[1];
    EXPECT_NEAR(tBest1, 0.0749614, tTolerance);
    EXPECT_NEAR(tBest2, 0.00500168, tTolerance);

    // ********* OUTPUT TO TERMINAL *********
    std::cout << "NumIterationsDone = " << tOutputs.mNumOuterIter << std::endl;
    std::cout << "NumFunctionEvaluations = " << tThrustMisfitObjective->getNumFunctionEvaluations() << std::endl;
    std::cout << "BestObjectiveValue = " << tOutputs.mObjFuncValue << std::endl;
    std::cout << "BestValue 1 = " << tBest1 << ", BestValue 2 = " << tBest2 << std::endl;
    std::cout << "StoppingCriterion = " << tOutputs.mStopCriterion.c_str() << std::endl;
}

TEST(PlatoTest, GradBasedSimpleRocketOptimizationWithLightInterface_LBFGS)
{
    // ********* SET NORMALIZATION CONSTANTS *********
    const size_t tNumControls = 2;
    std::vector<double> tNormalizationConstants(tNumControls);
    tNormalizationConstants[0] = 0.08; tNormalizationConstants[1] = 0.006;

    // ********* ALLOCATE OBJECTIVE AND SET TARGET THRSUT PROFILE *********
    Plato::AlgebraicRocketInputs<double> tRocketInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
            std::make_shared<Plato::Cylinder<double>>(tRocketInputs.mChamberRadius, tRocketInputs.mChamberLength);
    std::shared_ptr<Plato::GradBasedRocketObjective<double>> tThrustMisfitObjective =
            std::make_shared<Plato::GradBasedRocketObjective<double>>(tRocketInputs, tGeomModel);
    tThrustMisfitObjective->setNormalizationConstants(tNormalizationConstants);
    std::vector<double> tTargetThrustProfile = PlatoTest::get_target_thrust_profile();
    tThrustMisfitObjective->setTargetThrustProfile(tTargetThrustProfile);

    // ********* SET LIST OF OBJECTIVE FUNCTIONS *********
    std::shared_ptr<Plato::CriterionList<double>> tMyObjective = std::make_shared<Plato::CriterionList<double>>();
    tMyObjective->add(tThrustMisfitObjective);

    // ********* SET OPTIMIZATION ALGORITHM INPUTS *********
    const size_t tNumVectors = 1;
    Plato::AlgorithmInputsKSBC<double> tInputs;
    tInputs.mDisablePostSmoothing = true;
    tInputs.mHessianMethod = Plato::Hessian::LBFGS;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls);
    (*tInputs.mLowerBounds)(0,0) = 0.06 / tNormalizationConstants[0];
    (*tInputs.mLowerBounds)(0,1) = 0.003 / tNormalizationConstants[1];
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 1.0 /* base value */);

    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls);
    (*tInputs.mInitialGuess)(0,0) = 0.074 / tNormalizationConstants[0];
    (*tInputs.mInitialGuess)(0,1) = 0.0055 / tNormalizationConstants[1];

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::AlgorithmOutputsKSBC<double> tOutputs;
    Plato::solve_ksbc<double, size_t>(tMyObjective, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    const double tBest1 = (*tOutputs.mSolution)(0,0) * tNormalizationConstants[0];
    EXPECT_EQ(19u, tOutputs.mNumOuterIter);
    EXPECT_EQ(82u, tOutputs.mNumObjFuncEval);
    EXPECT_EQ(20u, tOutputs.mNumObjGradEval);
    const double tBest2 = (*tOutputs.mSolution)(0,1) * tNormalizationConstants[1];
    EXPECT_NEAR(tBest1, 0.0750002, tTolerance);
    EXPECT_NEAR(tBest2, 0.00499999, tTolerance);

    // ********* OUTPUT TO TERMINAL *********
    std::cout << "NumIterationsDone = " << tOutputs.mNumOuterIter << std::endl;
    std::cout << "NumFunctionEvaluations = " << tThrustMisfitObjective->getNumFunctionEvaluations() << std::endl;
    std::cout << "BestObjectiveValue = " << tOutputs.mObjFuncValue << std::endl;
    std::cout << "BestValue 1 = " << tBest1 << ", BestValue 2 = " << tBest2 << std::endl;
    std::cout << "StoppingCriterion = " << tOutputs.mStopCriterion.c_str() << std::endl;
}

} // namespace PlatoTest
