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
    std::vector<double> tData = { 1, 1, 0.09963763906, 0.01, 0.01, 0.01, 0.02981858214, 0.122176848, 1, 1, 1, 1, 0.2407984809, 0.0159862795, 0.01, 0.01,
        0.02993454668, 0.1204176079, 0.9984638161, 1, 1, 0.8792774721, 1, 0.1527013574, 0.01, 0.01, 0.03110600992, 0.116003526, 0.99789355, 1, 1, 0.3621251213,
        0.869480942, 0.9222532171, 0.1059821443, 0.01, 0.02726839263, 0.1093306959, 1, 1, 1, 0.229465817, 0.177670247, 0.9608359435, 0.7573956306,
        0.07856082129, 0.01613082746, 0.1038487414, 1, 0.9987520709, 1, 0.213310992, 0.09880097021, 0.4230699414, 0.8618869651, 0.6385901013, 0.05527002214,
        0.09962157642, 0.9991862169, 0.9873343721, 0.9936766576, 0.21332468, 0.0888907914, 0.06665360106, 0.6708894614, 0.8500175685, 0.4072074949,
        0.1368029677, 1, 1, 1, 0.2086912518, 0.07991180519, 0.01759975628, 0.1035920092, 0.7906481264, 0.9033673742, 0.3404269023, 0.9982450947, 1,
        0.9954508096, 0.2127764606, 0.07411368624, 0.01, 0.01, 0.1524722234, 0.9224332142, 0.8073510223, 0.9127129638, 1, 1, 0.2146435157, 0.06006097549, 0.01,
        0.01, 0.01295802044, 0.2550278407, 0.965270516, 0.905787634, 0.9939075568, 0.9945545856, 0.2230887416, 0.05064233306, 0.01, 0.01, 0.01, 0.01774828622,
        0.3203253654, 0.9965550133, 0.9934764039, 0.9981459117, 0.2293810086, 0.04568875546, 0.01634284154, 0.01, 0.01, 0.01044056529, 0.2528440393, 1, 1,
        0.99727598, 0.2413115806, 0.03164221735, 0.01262831739, 0.01841103037, 0.01, 0.1774915334, 0.9692749415, 0.7129325521, 0.8632292503, 1, 0.2580872933,
        0.02539650312, 0.01986806615, 0.01006649374, 0.1385087242, 0.8632392614, 0.8557574523, 0.1701971433, 0.8751502435, 0.9966404439, 0.2888995111,
        0.0168106666, 0.01875154327, 0.112405472, 0.7739732449, 0.893669012, 0.1467913411, 0.07236252735, 0.8757534631, 0.9992574255, 0.3394110602,
        0.01372413809, 0.09592103627, 0.7157707903, 0.815507893, 0.369117658, 0.01567082462, 0.06453770917, 0.8747390771, 1, 0.416004086, 0.1156031579,
        0.6132942877, 0.813874354, 0.5883956644, 0.04770494721, 0.01592927583, 0.06346413713, 0.873894403, 0.9945048142, 0.4007816515, 0.520010458,
        0.8460529553, 0.7240971586, 0.07553824292, 0.01, 0.01, 0.06352301774, 0.873194224, 0.9934189319, 0.5218459518, 0.750271282, 0.8378993636, 0.1098522469,
        0.01257494755, 0.01, 0.01, 0.06361849477, 0.8725756646, 0.9916137817, 0.7458641095, 0.8546970895, 0.1701835009, 0.01735174789, 0.01, 0.01, 0.01,
        0.06371955664, 0.8719892236, 0.9735136467, 0.8210040525, 0.4429160497, 0.02614668387, 0.01338563026, 0.01, 0.01, 0.01, 0.06386002518, 0.8713635522,
        0.8493975383, 0.8719407676, 0.19617933, 0.01490080135, 0.01, 0.01, 0.01, 0.01168024438, 0.06407551131, 0.8706381416, 0.1372458288, 0.881936745,
        0.8051764521, 0.1475906557, 0.01, 0.01317893402, 0.01, 0.01100125746, 0.06447658074, 0.8694408448, 0.01160002611, 0.1483743016, 0.8454816649,
        0.8001792584, 0.1476388708, 0.01, 0.01, 0.01, 0.06523647754, 0.8674728199, 0.01, 0.01262313208, 0.1633494583, 0.8378953242, 0.7977748154, 0.1468819449,
        0.01, 0.01611682195, 0.06690406329, 0.8640400233, 0.01, 0.01, 0.01469643042, 0.167958588, 0.8368900078, 0.7937936722, 0.1471738997, 0.01, 0.07009093675,
        0.8606060962, 0.01, 0.01, 0.01, 0.01002830186, 0.1701030503, 0.8393064961, 0.7847495731, 0.1444929369, 0.07748326246, 0.8597340432, 0.01, 0.01,
        0.02057088789, 0.01, 0.0123472956, 0.1664332417, 0.8605738334, 0.7747448051, 0.2566614261, 0.8579196479, 0.01, 0.01, 0.01, 0.01454491462, 0.01, 0.01,
        0.1540314408, 0.922222503, 0.7825272514, 0.7965536156, 0.01444850231, 0.01, 0.01, 0.01, 0.01, 0.01016863972, 0.01175750541, 0.1550800289, 0.9349465177,
        1 };
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
    EXPECT_FALSE(tInputsOne.mPrintDiagnostics);
    EXPECT_EQ(0u, tInputsOne.mUpdateFrequency);
    EXPECT_EQ(500u, tInputsOne.mMaxNumSolverIter);
    EXPECT_EQ(1u, tInputsOne.mNumControlVectors);
    EXPECT_EQ(50u, tInputsOne.mMaxNumTrustRegionIter);
    EXPECT_EQ(100u, tInputsOne.mMaxNumSubProblemIter);
    EXPECT_EQ(Plato::MemorySpace::HOST, tInputsOne.mMemorySpace);

    const double tTolerance = 1e-6;
    EXPECT_TRUE(tInputsOne.mConstraintNormalizationMultipliers.empty());
    EXPECT_NEAR(0.5, tInputsOne.mMoveLimit, tTolerance);
    EXPECT_NEAR(1.2, tInputsOne.mAsymptoteExpansion, tTolerance);
    EXPECT_NEAR(0.7, tInputsOne.mAsymptoteContraction, tTolerance);
    EXPECT_NEAR(1.0, tInputsOne.mInitialAugLagPenalty, tTolerance);
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

    EXPECT_FALSE(tInputsOne.mPrintDiagnostics);
    EXPECT_EQ(0u, tInputsOne.mUpdateFrequency);
    EXPECT_EQ(500u, tInputsOne.mMaxNumSolverIter);
    EXPECT_EQ(1u, tInputsOne.mNumControlVectors);
    EXPECT_EQ(50u, tInputsOne.mMaxNumTrustRegionIter);
    EXPECT_EQ(100u, tInputsOne.mMaxNumSubProblemIter);
    EXPECT_EQ(Plato::MemorySpace::HOST, tInputsOne.mMemorySpace);

    EXPECT_NEAR(0.5, tInputsOne.mMoveLimit, tTolerance);
    EXPECT_NEAR(1.2, tInputsOne.mAsymptoteExpansion, tTolerance);
    EXPECT_NEAR(0.7, tInputsOne.mAsymptoteContraction, tTolerance);
    EXPECT_NEAR(1.0, tInputsOne.mInitialAugLagPenalty, tTolerance);
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

    EXPECT_FALSE(tInputsOne.mPrintDiagnostics);
    EXPECT_EQ(5u, tInputsOne.mUpdateFrequency);
    EXPECT_EQ(100u, tInputsOne.mMaxNumSolverIter);
    EXPECT_EQ(2u, tInputsOne.mNumControlVectors);
    EXPECT_EQ(25u, tInputsOne.mMaxNumTrustRegionIter);
    EXPECT_EQ(200u, tInputsOne.mMaxNumSubProblemIter);
    EXPECT_EQ(Plato::MemorySpace::DEVICE, tInputsOne.mMemorySpace);

    EXPECT_NEAR(0.55, tInputsOne.mMoveLimit, tTolerance);
    EXPECT_NEAR(1.3, tInputsOne.mAsymptoteExpansion, tTolerance);
    EXPECT_NEAR(0.85, tInputsOne.mAsymptoteContraction, tTolerance);
    EXPECT_NEAR(2.0, tInputsOne.mInitialAugLagPenalty, tTolerance);
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

    EXPECT_FALSE(tInputsTwo.mPrintDiagnostics);
    EXPECT_EQ(5u, tInputsTwo.mUpdateFrequency);
    EXPECT_EQ(1u, tInputsTwo.mNumControlVectors);
    EXPECT_EQ(100u, tInputsTwo.mMaxNumSolverIter);
    EXPECT_EQ(25u, tInputsTwo.mMaxNumTrustRegionIter);
    EXPECT_EQ(25u, tInputsTwo.mMaxNumSubProblemIter);
    EXPECT_EQ(Plato::MemorySpace::HOST, tInputsTwo.mMemorySpace);

    EXPECT_NEAR(0.55, tInputsTwo.mMoveLimit, tTolerance);
    EXPECT_NEAR(1.01, tInputsTwo.mAsymptoteExpansion, tTolerance);
    EXPECT_NEAR(0.99, tInputsTwo.mAsymptoteContraction, tTolerance);
    EXPECT_NEAR(2.0, tInputsTwo.mInitialAugLagPenalty, tTolerance);
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
        std::system("rm -f MyFile1.txt");

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
         std::system("rm -f MyFile1.txt");

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
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 1.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 10.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 5.0 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 1.0 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(28u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(1.33996, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 6.007921063; tGold(0,1) = 5.309376913; tGold(0,2) = 4.497699841; tGold(0,3) = 3.505355435; tGold(0,4) = 2.15340137;
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
    tInputs.mMoveLimit = 0.05;
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 2.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 1.0 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 2.0 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(36u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(0.0456735, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 0.7864850239; tGold(0,1) = 0.617637575;
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
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -5.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -1.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -2.0 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 0.5 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(27u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(11.8039164, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = -3.99832364; tGold(0,1) = -2.878447094;
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
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -3.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, -0.4 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 0.5 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(19u, tOutputs.mNumSolverIter);
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
    tInputs.mLowerBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.0 /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 2.0 /* values */);
    tInputs.mInitialGuess = std::make_shared<Plato::StandardMultiVector<double>>(tNumVectors, tNumControls, 0.5 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::StandardVector<double>>(tNumConstraints, 0.5 /* values */);
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tObjective, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(19u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(2.677976067, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 0.3129487804; tGold(0,1) = 0.9497764126;
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution, tTolerance);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(*tOutputs.mSolution);
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

TEST(PlatoTest, MethodMovingAsymptotes_MinComplianceVolumeConstraint)
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
    tInputs.mAsymptoteExpansion = 1.001;
    tInputs.mAsymptoteContraction = 0.999;
    tInputs.mInitialGuess = std::make_shared<Plato::EpetraSerialDenseMultiVector<double, int>>(tNumVectors, tNumControls, tTargetVolume /* values */);
    tInputs.mUpperBounds = std::make_shared<Plato::EpetraSerialDenseMultiVector<double, int>>(tNumVectors, tNumControls, 1.0 /* values */);
    tInputs.mLowerBounds = std::make_shared<Plato::EpetraSerialDenseMultiVector<double, int>>(tNumVectors, tNumControls, 1e-2 /* values */);
    tInputs.mConstraintNormalizationParams = std::make_shared<Plato::EpetraSerialDenseVector<double, int>>(tNumConstraints, 4.0 /* values */);
    Plato::AlgorithmOutputsMMA<double, int> tOutputs;
    Plato::solve_mma<double, int>(tCompliance, tConstraintList, tInputs, tOutputs);

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-2;
    ASSERT_EQ(50u, tOutputs.mNumSolverIter);
    ASSERT_NEAR(0.1485850316, tOutputs.mObjFuncValue, tTolerance);
    ASSERT_TRUE(std::abs((*tOutputs.mConstraints)[0]) < tTolerance);
    std::vector<double> tGoldData = PlatoTest::get_topology_optimization_gold();
    Plato::StandardMultiVector<double, int> tGold(tNumVectors, tGoldData);
    tGold.setData(0, tGoldData);
    PlatoTest::checkMultiVectorData(tGold, *tOutputs.mSolution, tTolerance);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tOutputs.mNumSolverIter << "\n" << std::flush;
    std::cout << "BEST OBJECTIVE VALUE = " << tOutputs.mObjFuncValue << "\n" << std::flush;
    std::cout << "BEST CONSTRAINT VALUE = " << (*tOutputs.mConstraints)[0] << "\n" << std::flush;
    std::cout << tOutputs.mStopCriterion.c_str() << "\n" << std::flush;
}

}
