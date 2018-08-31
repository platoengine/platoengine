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

#include "Plato_SimpleRocket.hpp"
#include "Plato_CircularCylinder.hpp"
#include "Plato_GradFreeSimpleRocketObjective.hpp"

#include "Plato_DataFactory.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_SimpleRocketObjective.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_KelleySachsBoundConstrained.hpp"
#include "Plato_ReducedSpaceTrustRegionStageMng.hpp"

#include "PSL_Random.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_IterativeSelection.hpp"

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
    std::pair<std::vector<double>, std::vector<double>> tBounds =
            std::make_pair<std::vector<double>, std::vector<double>>({0.07, 0.004},{0.08, 0.006});
    Plato::GradFreeSimpleRocketObjective tObjective(tRocketInputs, tGeomModel);
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

TEST(PlatoTest, SimpleRocketObjective)
{
    // allocate problem inputs - use default parameters
    Plato::SimpleRocketInuts<double> tRocketInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
            std::make_shared<Plato::CircularCylinder<double>>(tRocketInputs.mChamberRadius, tRocketInputs.mChamberLength);

    // set normalization constants
    Plato::SimpleRocketObjective<double> tObjective(tRocketInputs, tGeomModel);
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
    PlatoSubproblemLibrary::AbstractAuthority tAuthority;

    // define objective
    Plato::SimpleRocketInuts<double> tRocketInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
    std::make_shared<Plato::CircularCylinder<double>>(tRocketInputs.mChamberRadius, tRocketInputs.mChamberLength);
    Plato::GradFreeSimpleRocketObjective tObjective(tRocketInputs, tGeomModel);;

    // set inputs for optimization problem
    std::vector<int> tNumEvaluationsPerDim = {100, 100}; // domain dimension = 100x100=10000
    /* {chamber_radius_lb, ref_burn_rate_lb},  {chamber_radius_ub, ref_burn_rate_ub} */
    std::pair<std::vector<double>, std::vector<double>> aBounds =
    std::make_pair<std::vector<double>, std::vector<double>>( {0.07, 0.004}, {0.08, 0.006});
    tObjective.setOptimizationInputs(tNumEvaluationsPerDim, aBounds);

    // define searcher
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

TEST(PlatoTest, GradBasedSimpleRocketOptimization)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    tDataFactory->allocateControl(tNumControls);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<double>> tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<double>>(tDataFactory);

    // ********* SET CONTROL PARAMETERS *********
    const size_t tVECTOR_INDEX = 0;
    std::vector<double> tNormalizationConstants(tNumControls);
    tNormalizationConstants[0] = 0.08; tNormalizationConstants[1] = 0.006;

    Plato::StandardVector<double> tLowerBounds(tNumControls);
    tLowerBounds[0] = 0.06 / tNormalizationConstants[0]; tLowerBounds[1] = 0.003 / tNormalizationConstants[1];
    tDataMng->setControlLowerBounds(tVECTOR_INDEX, tLowerBounds);

    Plato::StandardVector<double> tUpperBounds(tNumControls);
    tUpperBounds[0] = 1.0; tUpperBounds[1] = 1.0;
    tDataMng->setControlUpperBounds(tVECTOR_INDEX, tUpperBounds);

    Plato::StandardVector<double> tInitialGuess(tNumControls);
    tInitialGuess[0] = 0.074 / tNormalizationConstants[0]; tInitialGuess[1] = 0.0055 / tNormalizationConstants[1];
    tDataMng->setInitialGuess(tVECTOR_INDEX, tInitialGuess);

    // ********* ALLOCATE OBJECTIVE *********
    Plato::SimpleRocketInuts<double> tRocketInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
            std::make_shared<Plato::CircularCylinder<double>>(tRocketInputs.mChamberRadius, tRocketInputs.mChamberLength);
    std::shared_ptr<Plato::SimpleRocketObjective<double>> tMyObjective =
            std::make_shared<Plato::SimpleRocketObjective<double>>(tRocketInputs, tGeomModel);

    // ********* SET NORMALIZATION PARAMETERS *********
    tMyObjective->setNormalizationConstants(tNormalizationConstants);

    // ********* SET OBJECTIVE FUNCTION LIST *********
    std::shared_ptr<Plato::CriterionList<double>> tObjectiveList = std::make_shared<Plato::CriterionList<double>>();
    tObjectiveList->add(tMyObjective);

    // ********* NEWTON ALGORITHM'S REDUCED SPACE STAGE MANAGER *********
    std::shared_ptr<Plato::ReducedSpaceTrustRegionStageMng<double>> tStageMng =
            std::make_shared<Plato::ReducedSpaceTrustRegionStageMng<double>>(tDataFactory, tObjectiveList);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::KelleySachsBoundConstrained<double> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    tAlgorithm.setMaxNumIterations(500);
    tAlgorithm.setMaxTrustRegionRadius(1e1);
    tAlgorithm.setStationarityTolerance(1e-12);
    tAlgorithm.setStagnationTolerance(1e-12);
    tAlgorithm.setGradientTolerance(1e-12);
    tAlgorithm.setActualReductionTolerance(1e-12);
    tAlgorithm.solve();

    // ********* EQUAL BY DETERMINISM OF OBJECTIVE *********
    EXPECT_FLOAT_EQ(tDataMng->getCurrentObjectiveFunctionValue(), tMyObjective->value(tDataMng->getCurrentControl()));

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    const double tBest1 = tDataMng->getCurrentControl(tVECTOR_INDEX)[0] * tNormalizationConstants[0];
    const double tBest2 = tDataMng->getCurrentControl(tVECTOR_INDEX)[1] * tNormalizationConstants[1];
    EXPECT_NEAR(tBest1, 0.0749672, tTolerance);
    EXPECT_NEAR(tBest2, 0.00500131, tTolerance);

    // ********* OUTPUT TO CONSOLE *********
    std::cout << "NumIterationsDone = " << tAlgorithm.getNumIterationsDone() << std::endl;
    std::cout << "NumFunctionEvaluations = " << tMyObjective->getNumFunctionEvaluations() << std::endl;
    std::cout << "BestObjectiveValue = " << tDataMng->getCurrentObjectiveFunctionValue() << std::endl;
    std::cout << "Best1: " << tBest1 << ", Best2: " << tBest2 << std::endl;
    std::ostringstream tMessage;
    Plato::getStopCriterion(tAlgorithm.getStoppingCriterion(), tMessage);
    std::cout << "StoppingCriterion = " << tMessage.str().c_str() << std::endl;
}

} // namespace PlatoTest
