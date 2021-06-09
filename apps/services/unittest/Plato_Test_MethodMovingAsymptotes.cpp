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
 * Plato_Test_MethodMovingAsymptotes.cpp
 *
 *  Created on: Jul 21, 2019
 */

#include <gtest/gtest.h>

#include "Plato_Radius.hpp"
#include "Plato_Circle.hpp"
#include "Plato_Rosenbrock.hpp"
#include "Plato_Himmelblau.hpp"
#include "Plato_GoldsteinPrice.hpp"
#include "Plato_ShiftedEllipse.hpp"
#include "Plato_CcsaTestObjective.hpp"
#include "Plato_CcsaTestInequality.hpp"

#include "Plato_DataFactory.hpp"
#include "Plato_ProxyVolume.hpp"
#include "Plato_ProxyCompliance.hpp"
#include "Plato_EpetraSerialDenseVector.hpp"
#include "Plato_EpetraSerialDenseMultiVector.hpp"
#include "Plato_StructuralTopologyOptimization.hpp"

#include "Plato_Diagnostics.hpp"
#include "Plato_MethodMovingAsymptotesParser.hpp"
#include "Plato_MethodMovingAsymptotesInterface.hpp"

#include "Plato_UnitTestUtils.hpp"

// namespace Plato

namespace PlatoTest
{

std::vector<double> get_topology_optimization_gold()
{
    std::vector<double> tData = {1.000000, 1.000000, 0.054416, 0.010000, 0.010000, 0.010000, 0.010000, 0.107172, 1.000000,
                                 0.995753, 1.000000, 1.000000, 0.222840, 0.011774, 0.010000, 0.011193, 0.017450, 0.099040,
                                 1.000000, 0.995839, 1.000000, 0.995062, 0.994558, 0.145738, 0.010000, 0.010000, 0.010000,
                                 0.100141, 1.000000, 0.992882, 1.000000, 0.423631, 0.949310, 0.884680, 0.104899, 0.010000,
                                 0.010000, 0.100928, 1.000000, 0.987589, 1.000000, 0.267570, 0.208984, 1.000000, 0.683408,
                                 0.055787, 0.010000, 0.100133, 1.000000, 1.000000, 1.000000, 0.250357, 0.010000, 0.320634,
                                 1.000000, 0.474707, 0.021041, 0.102702, 1.000000, 1.000000, 1.000000, 0.249412, 0.010000,
                                 0.028060, 0.533048, 1.000000, 0.277356, 0.114534, 1.000000, 1.000000, 1.000000, 0.253342,
                                 0.010000, 0.010000, 0.071676, 0.757942, 1.000000, 0.269741, 1.000000, 1.000000, 1.000000,
                                 0.258540, 0.010000, 0.010000, 0.010000, 0.122981, 0.949194, 0.927986, 1.000000, 1.000000,
                                 1.000000, 0.265321, 0.010000, 0.010000, 0.010000, 0.010000, 0.172308, 1.000000, 1.000000,
                                 1.000000, 1.000000, 0.273426, 0.010000, 0.010000, 0.010000, 0.010000, 0.010000, 0.217226,
                                 1.000000, 1.000000, 1.000000, 0.283754, 0.010000, 0.010000, 0.010000, 0.010000, 0.013950,
                                 0.353984, 1.000000, 1.000000, 1.000000, 0.296464, 0.010000, 0.010000, 0.010000, 0.010000,
                                 0.174673, 1.000000, 0.724970, 0.879559, 1.000000, 0.312293, 0.010000, 0.010000, 0.010000,
                                 0.104787, 0.932868, 0.865799, 0.174877, 0.883984, 1.000000, 0.332625, 0.010000, 0.010000,
                                 0.064205, 0.716493, 0.999894, 0.160022, 0.065007, 0.882556, 1.000000, 0.361010, 0.012093,
                                 0.026129, 0.528020, 1.000000, 0.267766, 0.010000, 0.064538, 0.880948, 1.000000, 0.405741,
                                 0.025328, 0.294324, 1.000000, 0.510514, 0.022451, 0.010000, 0.063489, 0.879414, 1.000000,
                                 0.458781, 0.227690, 1.000000, 0.710788, 0.065053, 0.010000, 0.010000, 0.063364, 0.878329,
                                 1.000000, 0.459109, 0.837643, 0.952140, 0.118016, 0.010000, 0.010000, 0.010000, 0.063441,
                                 0.877578, 1.000000, 0.688623, 1.000000, 0.178189, 0.010000, 0.010000, 0.010000, 0.010000,
                                 0.063215, 0.877164, 1.000000, 1.000000, 0.466642, 0.028261, 0.010000, 0.010000, 0.010000,
                                 0.010000, 0.063249, 0.877080, 1.000000, 0.749128, 0.102255, 0.010000, 0.010000, 0.010000,
                                 0.010000, 0.010000, 0.063339, 0.877342, 0.370998, 1.000000, 0.499144, 0.053033, 0.010000,
                                 0.010000, 0.010000, 0.010000, 0.063584, 0.877876, 0.015374, 0.276432, 1.000000, 0.547670,
                                 0.063342, 0.010000, 0.010000, 0.010000, 0.064030, 0.878854, 0.017543, 0.010000, 0.262921,
                                 1.000000, 0.590471, 0.072003, 0.010000, 0.010000, 0.064773, 0.879917, 0.010000, 0.010000,
                                 0.010000, 0.235337, 1.000000, 0.630965, 0.084735, 0.010000, 0.066888, 0.882083, 0.010000,
                                 0.010000, 0.010000, 0.010000, 0.212622, 1.000000, 0.672914, 0.095532, 0.070270, 0.881922,
                                 0.010000, 0.010000, 0.010000, 0.010000, 0.010000, 0.192335, 1.000000, 0.699664, 0.198865,
                                 0.876913, 0.010000, 0.010000, 0.010000, 0.010000, 0.010000, 0.010000, 0.174371, 1.000000,
                                 0.754778, 0.789327, 0.014449, 0.010000, 0.010000, 0.010000, 0.010000, 0.012695, 0.010000,
                                 0.172408, 0.999518, 1.000000};
    return tData;
}

TEST(PlatoTest, MethodMovingAsymptotes_Parser)
{
    Plato::InputData tInputData;
    EXPECT_TRUE(tInputData.empty());
    ASSERT_STREQ("Input Data", tInputData.name().c_str());

    Plato::MethodMovingAsymptotesParser<double> tParser;
    Plato::AlgorithmInputsMMA<double> tInputsOne;
    tParser.parse(tInputData, tInputsOne);

    // ********* TEST: OPTIONS NODE NOT DEFINE -> USE DEFAULT PARAMETERS *********
    EXPECT_FALSE(tInputsOne.mPrintMMADiagnostics);
    EXPECT_EQ(0u, tInputsOne.mUpdateFrequency);
    EXPECT_EQ(500u, tInputsOne.mMaxNumSolverIter);
    EXPECT_EQ(1u, tInputsOne.mNumControlVectors);
    EXPECT_EQ(50u, tInputsOne.mMaxNumTrustRegionIter);
    EXPECT_EQ(50u, tInputsOne.mMaxNumSubProblemIter);
    EXPECT_EQ(Plato::MemorySpace::HOST, tInputsOne.mMemorySpace);

    const double tTolerance = 1e-6;
    EXPECT_TRUE(tInputsOne.mConstraintNormalizationMultipliers.empty());
    EXPECT_NEAR(0.5, tInputsOne.mMoveLimit, tTolerance);
    EXPECT_NEAR(1.2, tInputsOne.mAsymptoteExpansion, tTolerance);
    EXPECT_NEAR(0.7, tInputsOne.mAsymptoteContraction, tTolerance);
    EXPECT_NEAR(0.0015, tInputsOne.mInitialAugLagPenalty, tTolerance);
    EXPECT_NEAR(0.5, tInputsOne.mInitialAymptoteScaling, tTolerance);
    EXPECT_NEAR(0.1, tInputsOne.mSubProblemBoundsScaling, tTolerance);
    EXPECT_NEAR(1e-6, tInputsOne.mOptimalityTolerance, tTolerance);
    EXPECT_NEAR(1e-4, tInputsOne.mFeasibilityTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsOne.mControlStagnationTolerance, tTolerance);
    EXPECT_NEAR(1e-8, tInputsOne.mObjectiveStagnationTolerance, tTolerance);

    // ********* TEST: OPTIONS NODE DEFINED, BUT PARAMETERS NOT SPECIFIED -> USE DEFAULT PARAMETERS *********
    Plato::InputData tOptions("Options");
    EXPECT_TRUE(tOptions.empty());
    tParser.parse(tOptions, tInputsOne);

    EXPECT_FALSE(tInputsOne.mPrintMMADiagnostics);
    EXPECT_EQ(0u, tInputsOne.mUpdateFrequency);
    EXPECT_EQ(500u, tInputsOne.mMaxNumSolverIter);
    EXPECT_EQ(1u, tInputsOne.mNumControlVectors);
    EXPECT_EQ(50u, tInputsOne.mMaxNumTrustRegionIter);
    EXPECT_EQ(50u, tInputsOne.mMaxNumSubProblemIter);
    EXPECT_EQ(Plato::MemorySpace::HOST, tInputsOne.mMemorySpace);

    EXPECT_NEAR(0.5, tInputsOne.mMoveLimit, tTolerance);
    EXPECT_NEAR(1.2, tInputsOne.mAsymptoteExpansion, tTolerance);
    EXPECT_NEAR(0.7, tInputsOne.mAsymptoteContraction, tTolerance);
    EXPECT_NEAR(0.0015, tInputsOne.mInitialAugLagPenalty, tTolerance);
    EXPECT_NEAR(0.5, tInputsOne.mInitialAymptoteScaling, tTolerance);
    EXPECT_NEAR(0.1, tInputsOne.mSubProblemBoundsScaling, tTolerance);
    EXPECT_NEAR(1e-6, tInputsOne.mOptimalityTolerance, tTolerance);
    EXPECT_NEAR(1e-4, tInputsOne.mFeasibilityTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsOne.mControlStagnationTolerance, tTolerance);
    EXPECT_NEAR(1e-8, tInputsOne.mObjectiveStagnationTolerance, tTolerance);
    EXPECT_TRUE(tInputsOne.mConstraintNormalizationMultipliers.empty());

    // ********* TEST: SET PARAMETERS AND ADD OPTIONS NODE TO OPTIMIZER'S NODE -> NON-DEFAULT VALUES ARE EXPECTED *********
    tOptions.add<std::string>("MemorySpace", "Device");
    tOptions.add<std::string>("OutputDiagnosticsToFile", "false");
    tOptions.add<std::string>("UpdateFrequency", "5");
    tOptions.add<std::string>("NumControlVectors", "2");
    tOptions.add<std::string>("MaxNumSubProblemIter", "200");
    tOptions.add<std::string>("MaxNumTrustRegionIter", "25");
    tOptions.add<std::string>("MaxNumOuterIterations", "100");
    tOptions.add<std::string>("MoveLimit", "0.55");
    tOptions.add<std::string>("InitialAugLagPenalty", "2.0");
    tOptions.add<std::string>("AsymptoteExpansion", "1.3");
    tOptions.add<std::string>("AsymptoteContraction", "0.85");
    tOptions.add<std::string>("InitialAymptoteScaling", "0.25");
    tOptions.add<std::string>("SubProblemBoundsScaling", "0.2");
    tOptions.add<std::string>("OptimalityTolerance", "1e-2");
    tOptions.add<std::string>("FeasibilityTolerance", "1e-3");
    tOptions.add<std::string>("ControlStagnationTolerance", "1e-5");
    tOptions.add<std::string>("ObjectiveStagnationTolerance", "1e-4");
    tOptions.add<std::string>("ConstraintNormalizationMultipliers", "2");
    Plato::InputData tOptimizerNodeOne("OptimizerNode");
    tOptimizerNodeOne.add<Plato::InputData>("Options", tOptions);

    tParser.parse(tOptimizerNodeOne, tInputsOne);

    EXPECT_FALSE(tInputsOne.mPrintMMADiagnostics);
    EXPECT_FALSE(tInputsOne.mPrintAugLagSubProbDiagnostics);
    EXPECT_EQ(5u, tInputsOne.mUpdateFrequency);
    EXPECT_EQ(100u, tInputsOne.mMaxNumSolverIter);
    EXPECT_EQ(2u, tInputsOne.mNumControlVectors);
    EXPECT_EQ(25u, tInputsOne.mMaxNumTrustRegionIter);
    EXPECT_EQ(200u, tInputsOne.mMaxNumSubProblemIter);
    EXPECT_EQ(Plato::MemorySpace::DEVICE, tInputsOne.mMemorySpace);

    EXPECT_NEAR(0.55, tInputsOne.mMoveLimit, tTolerance);
    EXPECT_NEAR(1.3, tInputsOne.mAsymptoteExpansion, tTolerance);
    EXPECT_NEAR(0.85, tInputsOne.mAsymptoteContraction, tTolerance);
    EXPECT_NEAR(0.0015, tInputsOne.mInitialAugLagPenalty, tTolerance);
    EXPECT_NEAR(0.25, tInputsOne.mInitialAymptoteScaling, tTolerance);
    EXPECT_NEAR(0.2, tInputsOne.mSubProblemBoundsScaling, tTolerance);
    EXPECT_NEAR(1e-2, tInputsOne.mOptimalityTolerance, tTolerance);
    EXPECT_NEAR(1e-3, tInputsOne.mFeasibilityTolerance, tTolerance);
    EXPECT_NEAR(1e-5, tInputsOne.mControlStagnationTolerance, tTolerance);
    EXPECT_NEAR(1e-4, tInputsOne.mObjectiveStagnationTolerance, tTolerance);
    EXPECT_FALSE(tInputsOne.mConstraintNormalizationMultipliers.empty());
    EXPECT_NEAR(2.0, tInputsOne.mConstraintNormalizationMultipliers[0], tTolerance);

    // ********* TEST: SET A HANDFULL OF PARAMETERS AND ADD OPTIONS NODE TO OPTIMIZER'S NODE -> A FEW DEFAULT VALUES ARE EXPECTED *********
    Plato::InputData tOptionsTwo("Options");
    tOptionsTwo.add<std::string>("OutputDiagnosticsToFile", "false");
    tOptionsTwo.add<std::string>("UpdateFrequency", "5");
    tOptionsTwo.add<std::string>("MaxNumTrustRegionIter", "25");
    tOptionsTwo.add<std::string>("MaxNumOuterIterations", "100");
    tOptionsTwo.add<std::string>("MoveLimit", "0.55");
    tOptionsTwo.add<std::string>("InitialAugLagPenalty", "2.0");
    tOptionsTwo.add<std::string>("ConstraintNormalizationMultipliers", "2");
    tOptionsTwo.add<std::string>("ConstraintNormalizationMultipliers", "3");
    Plato::InputData tOptimizerNodeTwo("OptimizerNode");
    tOptimizerNodeTwo.add<Plato::InputData>("Options", tOptionsTwo);
    Plato::AlgorithmInputsMMA<double> tInputsTwo;

    tParser.parse(tOptimizerNodeTwo, tInputsTwo);

    EXPECT_FALSE(tInputsTwo.mPrintMMADiagnostics);
    EXPECT_EQ(5u, tInputsTwo.mUpdateFrequency);
    EXPECT_EQ(1u, tInputsTwo.mNumControlVectors);
    EXPECT_EQ(100u, tInputsTwo.mMaxNumSolverIter);
    EXPECT_EQ(25u, tInputsTwo.mMaxNumTrustRegionIter);
    EXPECT_EQ(25u, tInputsTwo.mMaxNumSubProblemIter);
    EXPECT_EQ(Plato::MemorySpace::HOST, tInputsTwo.mMemorySpace);

    EXPECT_NEAR(0.55, tInputsTwo.mMoveLimit, tTolerance);
    EXPECT_NEAR(1.2, tInputsTwo.mAsymptoteExpansion, tTolerance);
    EXPECT_NEAR(0.7, tInputsTwo.mAsymptoteContraction, tTolerance);
    EXPECT_NEAR(0.0015, tInputsTwo.mInitialAugLagPenalty, tTolerance);
    EXPECT_NEAR(0.5, tInputsTwo.mInitialAymptoteScaling, tTolerance);
    EXPECT_NEAR(0.1, tInputsTwo.mSubProblemBoundsScaling, tTolerance);
    EXPECT_NEAR(1e-6, tInputsTwo.mOptimalityTolerance, tTolerance);
    EXPECT_NEAR(1e-4, tInputsTwo.mFeasibilityTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsTwo.mControlStagnationTolerance, tTolerance);
    EXPECT_NEAR(1e-8, tInputsTwo.mObjectiveStagnationTolerance, tTolerance);
    EXPECT_FALSE(tInputsTwo.mConstraintNormalizationMultipliers.empty());
    EXPECT_EQ(2u, tInputsTwo.mConstraintNormalizationMultipliers.size());
    EXPECT_NEAR(2.0, tInputsTwo.mConstraintNormalizationMultipliers[0], tTolerance);
    EXPECT_NEAR(3.0, tInputsTwo.mConstraintNormalizationMultipliers[1], tTolerance);
}

TEST(PlatoTest, MethodMovingAsymptotes_PrintDiagnosticsOneConstraints)
{
    Plato::CommWrapper tComm(MPI_COMM_WORLD);
    if (tComm.myProcID() == static_cast<int>(0))
    {
        std::ofstream tWriteFile;
        tWriteFile.open("MyFile1.txt");
        Plato::OutputDataMMA<double> tData;
        tData.mNumIter = 0;
        tData.mObjFuncCount = 1;
        tData.mObjFuncValue = 1.0;
        tData.mNormObjFuncGrad = 4.5656e-3;
        tData.mControlStagnationMeasure = 1.2345678e6;
        tData.mObjectiveStagnationMeasure = std::numeric_limits<double>::max();
        const size_t tNumConstraints = 1;
        tData.mConstraints.resize(tNumConstraints);
        tData.mConstraints[0] = 1.23e-5;

        ASSERT_NO_THROW(Plato::print_mma_diagnostics_header(tData, tWriteFile));
        ASSERT_NO_THROW(Plato::print_mma_diagnostics(tData, tWriteFile));
        tData.mNumIter = 1;
        tData.mObjFuncCount = 3;
        tData.mObjFuncValue = 0.298736;
        tData.mNormObjFuncGrad = 3.45656e-1;
        tData.mControlStagnationMeasure = 0.18743;
        tData.mObjectiveStagnationMeasure = 0.7109;
        tData.mConstraints[0] = 8.23e-2;
        ASSERT_NO_THROW(Plato::print_mma_diagnostics(tData, tWriteFile));
        tWriteFile.close();
        std::ifstream tReadFile;
        tReadFile.open("MyFile1.txt");
        std::string tInputString;
        std::stringstream tReadData;
        while (tReadFile >> tInputString)
        {
            tReadData << tInputString.c_str();
        }
        tReadFile.close();
        Plato::system("rm -f MyFile1.txt");

        std::stringstream tGold;
        tGold << "IterF-countF(X)Norm(F')H1(X)abs(dX)abs(dF)";
        tGold << "011.000000e+004.565600e-031.230000e-051.234568e+061.797693e+308";
        tGold << "132.987360e-013.456560e-018.230000e-021.874300e-017.109000e-01";
        ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
    }
}

TEST(PlatoTest, MethodMovingAsymptotes_PrintDiagnosticsTwoConstraints)
{
    Plato::CommWrapper tComm(MPI_COMM_WORLD);
    if (tComm.myProcID() == static_cast<int>(0))
    {
        std::ofstream tWriteFile;
        tWriteFile.open("MyFile1.txt");
        Plato::OutputDataMMA<double> tData;
        tData.mNumIter = 0;
        tData.mObjFuncCount = 1;
        tData.mObjFuncValue = 1.0;
        tData.mNormObjFuncGrad = 4.5656e-3;
        tData.mControlStagnationMeasure = 1.2345678e6;
        tData.mObjectiveStagnationMeasure = std::numeric_limits<double>::max();
        const size_t tNumConstraints = 2;
        tData.mConstraints.resize(tNumConstraints);
        tData.mConstraints[0] = 1.23e-5;
        tData.mConstraints[1] = 3.33e-3;

        ASSERT_NO_THROW(Plato::print_mma_diagnostics_header(tData, tWriteFile));
        ASSERT_NO_THROW(Plato::print_mma_diagnostics(tData, tWriteFile));
        tData.mNumIter = 1;
        tData.mObjFuncCount = 3;
        tData.mObjFuncValue = 0.298736;
        tData.mNormObjFuncGrad = 3.45656e-1;
        tData.mControlStagnationMeasure = 0.18743;
        tData.mObjectiveStagnationMeasure = 0.7109;
        tData.mConstraints[0] = 8.23e-2;
        tData.mConstraints[1] = 8.33e-5;
        ASSERT_NO_THROW(Plato::print_mma_diagnostics(tData, tWriteFile));
        tWriteFile.close();

         std::ifstream tReadFile;
         tReadFile.open("MyFile1.txt");
         std::string tInputString;
         std::stringstream tReadData;
         while(tReadFile >> tInputString)
         {
         tReadData << tInputString.c_str();
         }
         tReadFile.close();
         Plato::system("rm -f MyFile1.txt");

         std::stringstream tGold;
         tGold << "IterF-countF(X)Norm(F')H1(X)H2(X)abs(dX)abs(dF)";
         tGold << "011.000000e+004.565600e-031.230000e-053.330000e-031.234568e+061.797693e+308";
         tGold << "132.987360e-013.456560e-018.230000e-028.330000e-051.874300e-017.109000e-01";
         ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
    }
}

TEST(PlatoTest, MethodMovingAsymptotes_PrintStoppingCriterion)
{
    std::string tDescription;
    Plato::algorithm::stop_t tFlag = Plato::algorithm::NOT_CONVERGED;
    Plato::print_mma_stop_criterion(tFlag, tDescription);
    std::string tGold("\n\n****** Optimization algorithm did not converge. ******\n\n");
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::OPTIMALITY_AND_FEASIBILITY;
    Plato::print_mma_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to optimality and feasibility tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::MAX_NUMBER_ITERATIONS;
    Plato::print_mma_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to exceeding maximum number of iterations. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::CONTROL_STAGNATION;
    Plato::print_mma_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to control (i.e. design variable) stagnation. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::algorithm::OBJECTIVE_STAGNATION;
    Plato::print_mma_stop_criterion(tFlag, tDescription);
    tGold.clear();
    tGold = "\n\n****** Optimization stopping due to objective stagnation. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());
}

TEST(PlatoTest, MethodMovingAsymptotesCriterion_Objective)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    Plato::ApproximationFunctionData<double> tAppxFuncCoreData(tDataFactory);
    Plato::fill(0.1, *tAppxFuncCoreData.mAppxFunctionP);
    Plato::fill(0.11, *tAppxFuncCoreData.mAppxFunctionQ);
    Plato::fill(1.0, *tAppxFuncCoreData.mCurrentControls);
    Plato::fill(5.0, *tAppxFuncCoreData.mUpperAsymptotes);
    Plato::fill(-5.0, *tAppxFuncCoreData.mLowerAsymptotes);
    tAppxFuncCoreData.mCurrentNormalizedCriterionValue = 1;

    Plato::MethodMovingAsymptotesCriterion<double> tAppxFunction(tDataFactory);
    tAppxFunction.setConstraintAppxFuncMultiplier(0.0);
    tAppxFunction.update(tAppxFuncCoreData);

    // TEST GRADIENT
    std::ostringstream tGradDiagnosticMsg;
    Plato::Diagnostics<double> tDiagnostics;
    ASSERT_NO_THROW(tDiagnostics.checkCriterionGradient(tAppxFunction, *tAppxFuncCoreData.mCurrentControls, tGradDiagnosticMsg));
    ASSERT_TRUE(tDiagnostics.didGradientTestPassed());
    Plato::CommWrapper tComm(MPI_COMM_WORLD);
    if(tComm.myProcID() == static_cast<int>(0))
    {
        std::cout << tGradDiagnosticMsg.str().c_str();
    }

    // TEST HESSIAN
    std::ostringstream tHessDiagnosticMsg;
    ASSERT_NO_THROW(tDiagnostics.checkCriterionHessian(tAppxFunction, *tAppxFuncCoreData.mCurrentControls, tHessDiagnosticMsg));
    ASSERT_TRUE(tDiagnostics.didHessianTestPassed());
    if(tComm.myProcID() == static_cast<int>(0))
    {
        std::cout << tHessDiagnosticMsg.str().c_str();
    }
}

TEST(PlatoTest, MethodMovingAsymptotesCriterion_Constraint)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    Plato::ApproximationFunctionData<double> tAppxFuncCoreData(tDataFactory);
    Plato::fill(0.1, *tAppxFuncCoreData.mAppxFunctionP);
    Plato::fill(0.11, *tAppxFuncCoreData.mAppxFunctionQ);
    Plato::fill(1.0, *tAppxFuncCoreData.mCurrentControls);
    Plato::fill(5.0, *tAppxFuncCoreData.mUpperAsymptotes);
    Plato::fill(-5.0, *tAppxFuncCoreData.mLowerAsymptotes);
    tAppxFuncCoreData.mCurrentNormalizedCriterionValue = 1;

    Plato::MethodMovingAsymptotesCriterion<double> tAppxFunction(tDataFactory);
    tAppxFunction.update(tAppxFuncCoreData);

    // TEST GRADIENT
    std::ostringstream tGradDiagnosticMsg;
    Plato::Diagnostics<double> tDiagnostics;
    ASSERT_NO_THROW(tDiagnostics.checkCriterionGradient(tAppxFunction, *tAppxFuncCoreData.mCurrentControls, tGradDiagnosticMsg));
    ASSERT_TRUE(tDiagnostics.didGradientTestPassed());
    Plato::CommWrapper tComm(MPI_COMM_WORLD);
    if(tComm.myProcID() == static_cast<int>(0))
    {
        std::cout << tGradDiagnosticMsg.str().c_str();
    }

    // TEST HESSIAN
    std::ostringstream tHessDiagnosticMsg;
    ASSERT_NO_THROW(tDiagnostics.checkCriterionHessian(tAppxFunction, *tAppxFuncCoreData.mCurrentControls, tHessDiagnosticMsg));
    ASSERT_TRUE(tDiagnostics.didHessianTestPassed());
    if(tComm.myProcID() == static_cast<int>(0))
    {
        std::cout << tHessDiagnosticMsg.str().c_str();
    }
}

TEST(PlatoTest, MethodMovingAsymptotesOperations_initialize)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CALL FUNCTION
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.75);
    tOperations.getUpperMinusLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tData, tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesOperations_updateInitialAsymptotes)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CALL FUNCTION
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateInitialAsymptotes(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.625);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tGold);
    Plato::fill(1.375, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesOperations_updateCurrentAsymptotesMultipliers)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.75, tData);
    tDataMng.setPreviousControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousControls(), tData);
    Plato::fill(0.65, tData);
    tDataMng.setAntepenultimateControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getAntepenultimateControls(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateCurrentAsymptotesMultipliers(tDataMng);

    // TEST OUTPUT
    tOperations.getCurrentAsymptotesMultipliers(tData);
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 1.2);
    PlatoTest::checkMultiVectorData(tData, tGold);

    // CASE 2: CONTRACTION PARAMETER IS CHOSEN
    Plato::fill(0.55, tData);
    tDataMng.setPreviousControls(tData);
    tOperations.updateCurrentAsymptotesMultipliers(tDataMng);

    // TEST OUTPUT
    tOperations.getCurrentAsymptotesMultipliers(tData);
    Plato::fill(0.7, tGold);
    PlatoTest::checkMultiVectorData(tData, tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesOperations_updateCurrentAsymptotes)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.75, tData);
    tDataMng.setPreviousControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousControls(), tData);
    Plato::fill(0.65, tData);
    tDataMng.setAntepenultimateControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getAntepenultimateControls(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateCurrentAsymptotesMultipliers(tDataMng);
    tOperations.updateCurrentAsymptotes(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.85);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tGold);
    Plato::fill(1.75, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesOperations_updateSubProblemBounds)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateSubProblemBounds(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.6625);
    PlatoTest::checkMultiVectorData(tDataMng.getSubProblemControlLowerBounds(), tGold);
    Plato::fill(1., tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getSubProblemControlUpperBounds(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesOperations_updateObjectiveApproximationFunctionData)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // CREATE MEANINGFUL CRITERION
    Plato::CcsaTestObjective<double> tCriterion;
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);
    double tObjFuncValue = tCriterion.value(tData);
    Plato::StandardMultiVector<double> tGradient(1 /* number of vectors */, tNumControls);
    tCriterion.gradient(tData, tGradient);

    // SET DATA
    Plato::MethodMovingAsymptotesDataMng<double> tDataMng(tDataFactory);
    tDataMng.setCurrentObjectiveValue(tObjFuncValue);
    const double tTolerance = 1e-6;
    ASSERT_NEAR(0.312, tDataMng.getCurrentObjectiveValue(), tTolerance);
    tDataMng.setCurrentObjectiveGradient(tGradient);
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.0624 /* initial value */);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentObjectiveGradient(), tGold);

    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateObjectiveApproximationFunctionData(tDataMng);

    // TEST OUTPUT
    Plato::fill(0.028155, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getObjFuncAppxFunctionP(), tGold);
    Plato::fill(3e-5, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getObjFuncAppxFunctionQ(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesOperations_updateConstraintApproximationFunctionsData)
{
    const size_t tNumControls = 5;
    const size_t tNumConstraints = 2;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateDual(tNumConstraints);
    tDataFactory->allocateControl(tNumControls);

    // CREATE MEANINGFUL CRITERION
    Plato::CcsaTestObjective<double> tCriterion;
    Plato::MethodMovingAsymptotesDataMng<double> tDataMng(tDataFactory);
    ASSERT_EQ(2u, tDataMng.getNumConstraints());
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);

    // SET DATA FOR CONSTRAINT 1
    size_t tContraintIndex = 0;
    double tConstraintValue = tCriterion.value(tData);
    tDataMng.setCurrentConstraintValue(tContraintIndex, tConstraintValue);
    tDataMng.setConstraintNormalization(tContraintIndex, 1);
    const double tTolerance = 1e-6;
    ASSERT_NEAR(0.312, tDataMng.getCurrentConstraintValue(tContraintIndex), tTolerance);

    Plato::StandardMultiVector<double> tConstraintGradient(1 /* number of vectors */, tNumControls);
    tCriterion.gradient(tData, tConstraintGradient);
    tDataMng.setCurrentConstraintGradient(tContraintIndex, tConstraintGradient);
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.0624 /* initial value */);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentConstraintGradient(tContraintIndex), tGold);

    // SET DATA FOR CONSTRAINT 2
    tContraintIndex = 1;
    tCriterion.setWeightConstant(0.624);
    tConstraintValue = tCriterion.value(tData);
    tDataMng.setConstraintNormalization(tContraintIndex, 1);
    tDataMng.setCurrentConstraintValue(tContraintIndex, tConstraintValue);
    ASSERT_NEAR(3.12, tDataMng.getCurrentConstraintValue(tContraintIndex), tTolerance);

    tCriterion.gradient(tData, tConstraintGradient);
    tDataMng.setCurrentConstraintGradient(tContraintIndex, tConstraintGradient);
    Plato::fill(0.624, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentConstraintGradient(tContraintIndex), tGold);

    // SET OTHER DATA
    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateConstraintApproximationFunctionsData(tDataMng);

    // TEST OUTPUT FOR FIRST CONSTRAINT
    tContraintIndex = 0;
    Plato::fill(0.00878565, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionP(tContraintIndex), tGold);
    Plato::fill(0.00001065, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionQ(tContraintIndex), tGold);

    // TEST OUTPUT FOR SECOND CONSTRAINT
    tContraintIndex = 1;
    Plato::fill(0.087839625, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionP(tContraintIndex), tGold);
    Plato::fill(0.000089625, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionQ(tContraintIndex), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotes_5Bars)
{
    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::Criterion<double>> tObjective = std::make_shared<Plato::CcsaTestObjective<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    std::shared_ptr<Plato::CcsaTestInequality<double>> tConstraint = std::make_shared<Plato::CcsaTestInequality<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const size_t tNumVectors = 1;
    const size_t tNumControls = 5;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsMMA<double> tInputs;
    //tInputs.mPrintMMADiagnostics = true;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 1.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 10.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 5.0 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 1.0 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(9u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(1.33996, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 6.016020337; tGold(0,1) = 5.309110081; tGold(0,2) = 4.49438973; tGold(0,3) = 3.501469461; tGold(0,4) = 2.15267005;
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(*tOutputs.mSolution);
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

TEST(PlatoTest, MethodMovingAsymptotes_RosenbrockRadius)
{
    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::Rosenbrock<double>> tObjective = std::make_shared<Plato::Rosenbrock<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    std::shared_ptr<Plato::Radius<double>> tConstraint = std::make_shared<Plato::Radius<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsMMA<double> tInputs;
    //tInputs.mPrintMMADiagnostics = true;
    tInputs.mMoveLimit = 0.1;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 2.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 1.0 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 2.0 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(77u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(0.0456748, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 0.7864153996; tGold(0,1) = 0.6176982996;
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(*tOutputs.mSolution);
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

TEST(PlatoTest, MethodMovingAsymptotes_HimmelblauShiftedEllipse)
{
    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::Himmelblau<double>> tObjective = std::make_shared<Plato::Himmelblau<double>>();
    std::shared_ptr<Plato::ShiftedEllipse<double>> tConstraint = std::make_shared<Plato::ShiftedEllipse<double>>();
    tConstraint->specify(-2., 2., -3., 3.);
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsMMA<double> tInputs;
    //tInputs.mPrintMMADiagnostics = true;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -5.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -1.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -2.0 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 2 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 5e-4;
    ASSERT_EQ(166u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(2.40362, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = -3.984397039; tGold(0,1) = -3.369040142;
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(*tOutputs.mSolution);
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

TEST(PlatoTest, MethodMovingAsymptotes_GoldsteinPriceShiftedEllipse)
{
    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::GoldsteinPrice<double>> tObjective = std::make_shared<Plato::GoldsteinPrice<double>>();
    std::shared_ptr<Plato::ShiftedEllipse<double>> tConstraint = std::make_shared<Plato::ShiftedEllipse<double>>();
    tConstraint->specify(0., 1., .5, 1.5);
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsMMA<double> tInputs;
    //tInputs.mPrintMMADiagnostics = true;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -3.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -0.4 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 2 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(8u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(3, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 0.0; tGold(0,1) = -1.0;
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution, tTolerance);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(*tOutputs.mSolution);
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

TEST(PlatoTest, MethodMovingAsymptotes_CircleRadius)
{
    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::Circle<double>> tObjective = std::make_shared<Plato::Circle<double>>();
    std::shared_ptr<Plato::Radius<double>> tConstraint = std::make_shared<Plato::Radius<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    Plato::AlgorithmInputsMMA<double> tInputs;
    //tInputs.mPrintMMADiagnostics = true;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 2.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.5 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 0.5 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(8u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(2.678, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) =  0.3115704953; tGold(0,1) = 0.9502230404;
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution, tTolerance);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(*tOutputs.mSolution);
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

TEST(DISABLED_PlatoTest, MethodMovingAsymptotes_MinComplianceVolumeConstraint)
{
    // ************** ALLOCATE SIMPLE STRUCTURAL TOPOLOGY OPTIMIZATION SOLVER **************
    const int tNumElementsXDir = 30;
    const int tNumElementsYDir = 10;
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    std::shared_ptr<Plato::StructuralTopologyOptimization> tPDE =
            std::make_shared <Plato::StructuralTopologyOptimization>(tPoissonRatio, tElasticModulus, tNumElementsXDir, tNumElementsYDir);
    tPDE->setFilterRadius(1.1);

    // ************** SET FORCE VECTOR **************
    const int tGlobalNumDofs = tPDE->getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tGlobalNumDofs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE->setForceVector(tForce);

    // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tDofs = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tDofs.data(), tDofs.size());
    tPDE->setFixedDOFs(tFixedDOFs);

    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::ProxyVolume<double, int>> tVolume = std::make_shared<Plato::ProxyVolume<double, int>>(tPDE);
    std::shared_ptr<Plato::ProxyCompliance<double, int>> tCompliance = std::make_shared<Plato::ProxyCompliance<double, int>>(tPDE);
    std::shared_ptr<Plato::CriterionList<double, int>> tConstraintList = std::make_shared<Plato::CriterionList<double, int>>();
    tConstraintList->add(tVolume);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    const int tNumVectors = 1;
    const int tNumConstraints = 1;
    const int tNumControls = tPDE->getNumDesignVariables();
    Plato::AlgorithmInputsMMA<double, int> tInputs;
    const double tTargetVolume = tPDE->getVolumeFraction();
    tInputs.mMaxNumSolverIter = 50;
    tInputs.mInitialGuess = std::make_shared<Plato::EpetraSerialDenseMultiVector<double, int>>(tNumVectors, tNumControls, tTargetVolume /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::EpetraSerialDenseMultiVector<double, int>>(tNumVectors, tNumControls, 1.0 /* values */);
    tInputs.mLowerBounds = std::make_shared<Plato::EpetraSerialDenseMultiVector<double, int>>(tNumVectors, tNumControls, 1e-2 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::EpetraSerialDenseVector<double, int>>(tNumConstraints, 4.0 /* values */);
    Plato::AlgorithmOutputsMMA<double, int> tOutputs;
    Plato::solve_mma<double, int>(tCompliance, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(50, tOutputs.mNumSolverIter);
    ASSERT_NEAR(0.143372, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    std::vector<double> tGoldData = PlatoTest::get_topology_optimization_gold();
    Plato::StandardMultiVector<double, int> tGold(tNumVectors, tGoldData);
    int tIndex = 0;
    tGold.setData(tIndex, tGoldData);
    //PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution, tTolerance);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

}
