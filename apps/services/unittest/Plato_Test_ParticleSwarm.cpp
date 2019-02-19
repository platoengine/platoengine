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
 //S
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
 * Plato_Test_ParticleSwarm.cpp
 *
 *  Created on: Dec 26, 2018
 */

#include <mpi.h>
#include <gtest/gtest.h>

#include "Plato_GradFreeCircle.hpp"
#include "Plato_GradFreeRadius.hpp"
#include "Plato_GradFreeHimmelblau.hpp"
#include "Plato_GradFreeRosenbrock.hpp"
#include "Plato_GradFreeRocketObjFunc.hpp"
#include "Plato_GradFreeShiftedEllipse.hpp"
#include "Plato_GradFreeGoldsteinPrice.hpp"

#include "Plato_ParticleSwarmInterfaceBCPSO.hpp"
#include "Plato_ParticleSwarmInterfaceALPSO.hpp"
#include "Plato_ParticleSwarmParser.hpp"

#include "Plato_UnitTestUtils.hpp"

namespace ParticleSwarmTest
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

TEST(PlatoTest, PSO_DataMng)
{
    // ********* Allocate Core Optimization Data Templates *********
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    // TEST setInitialParticles
    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
    EXPECT_EQ(20u, tDataMng.getNumParticles());
    Plato::StandardVector<double> tVector(tNumControls);
    tVector.fill(5);
    tDataMng.setUpperBounds(tVector);
    tVector.fill(-5);
    tDataMng.setLowerBounds(tVector);
    tDataMng.setInitialParticles();
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentParticles(), tDataMng.getCurrentVelocities());

    // TEST cachePreviousVelocities
    tDataMng.cachePreviousVelocities();
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentVelocities(), tDataMng.getPreviousVelocities());
}

TEST(PlatoTest, PSO_IsFileOpenExeption)
{
    int tMySize = 0;
    MPI_Comm_size(MPI_COMM_WORLD, &tMySize);
    if(tMySize <= 1)
    {
        std::ofstream tFile;
        ASSERT_THROW(Plato::pso::is_file_open(tFile), std::invalid_argument);

        tFile.open("MyFile.txt");
        ASSERT_NO_THROW(Plato::pso::is_file_open(tFile));
        tFile.close();
        std::system("rm -f MyFile.txt");
    }
}

TEST(PlatoTest, PSO_IsVectorEmpty)
{
    std::vector<double> tVector;
    ASSERT_THROW(Plato::pso::is_vector_empty(tVector), std::invalid_argument);

    const size_t tLength = 1;
    tVector.resize(tLength);
    ASSERT_NO_THROW(Plato::pso::is_vector_empty(tVector));
}

TEST(PlatoTest, PSO_PrintStoppingCriterion)
{
    std::string tDescription;
    Plato::particle_swarm::stop_t tFlag = Plato::particle_swarm::DID_NOT_CONVERGE;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    std::string tGold("\n\n****** Optimization algorithm did not converge. ******\n\n");
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::particle_swarm::MAX_NUMBER_ITERATIONS;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to exceeding maximum number of iterations. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::particle_swarm::TRUE_OBJECTIVE_TOLERANCE;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to global best objective function tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::particle_swarm::MEAN_OBJECTIVE_TOLERANCE;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to mean objective function tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::particle_swarm::STDDEV_OBJECTIVE_TOLERANCE;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to standard deviation tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());
}

TEST(PlatoTest, PSO_PrintDiagnosticsInvalidArgumentsPSO)
{
    std::ofstream tFile1;
    Plato::DiagnosticsBCPSO<double> tData;
    ASSERT_THROW(Plato::pso::print_bcpso_diagnostics_header(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_bcpso_diagnostics_header(tData, tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_bcpso_diagnostics(tData, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_bcpso_diagnostics(tData, tFile1, true /* print message */), std::invalid_argument);
}

TEST(PlatoTest, PSO_PrintDiagnostics)
{
    int tMySize = 0;
    MPI_Comm_size(MPI_COMM_WORLD, &tMySize);
    if(tMySize <= 1)
    {
        std::ofstream tWriteFile;
        tWriteFile.open("MyFile1.txt");
        Plato::DiagnosticsBCPSO<double> tData;
        tData.mNumIter = 1;
        tData.mObjFuncCount = 20;
        tData.mNumConstraints = 0;
        tData.mCurrentGlobalBestObjFuncValue = 4.2321;
        tData.mMeanCurrentBestObjFuncValues = 8.2321;
        tData.mStdDevCurrentBestObjFuncValues = 2.2321;
        tData.mTrustRegionMultiplier = 1.0;
        ASSERT_NO_THROW(Plato::pso::print_bcpso_diagnostics_header(tData, tWriteFile));
        ASSERT_NO_THROW(Plato::pso::print_bcpso_diagnostics(tData, tWriteFile));

        tData.mNumIter = 2;
        tData.mObjFuncCount = 40;
        tData.mCurrentGlobalBestObjFuncValue = 2.2321;
        tData.mMeanCurrentBestObjFuncValues = 7.2321;
        tData.mStdDevCurrentBestObjFuncValues = 2.4321;
        tData.mTrustRegionMultiplier = 1.0;
        ASSERT_NO_THROW(Plato::pso::print_bcpso_diagnostics(tData, tWriteFile));
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
        tGold << "IterF-countBest(F)Mean(F)StdDev(F)TR-Radius";
        tGold << "1204.232100e+008.232100e+002.232100e+001.000000e+00";
        tGold << "2402.232100e+007.232100e+002.432100e+001.000000e+00";
        ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
    }
}

TEST(PlatoTest, PSO_PrintSolution)
{
    int tMySize = 0;
    MPI_Comm_size(MPI_COMM_WORLD, &tMySize);
    if(tMySize <= 1)
    {
        // SOLUTION PRINTED
        size_t tNumElem = 3;
        Plato::StandardVector<double> tBest(tNumElem);
        tBest[0] = 1; tBest[1] = 2; tBest[2] = 3;
        Plato::StandardVector<double> tMean(tNumElem);
        tMean[0] = 0.9; tMean[1] = 1.9; tMean[2] = 3.1;
        Plato::StandardVector<double> tStdDev(tNumElem);
        tStdDev[0] = 0.1; tStdDev[1] = 0.2; tStdDev[2] = 0.3;
        EXPECT_TRUE(Plato::pso::print_solution(tBest, tMean, tStdDev));

        std::ifstream tReadFile;
        tReadFile.open("plato_pso_solution.txt");
        std::string tInputString;
        std::stringstream tReadData;
        while(tReadFile >> tInputString)
        {
            tReadData << tInputString.c_str();
        }
        tReadFile.close();
        std::system("rm -f plato_pso_solution.txt");

        std::stringstream tGold;
        tGold << "Best(X)Mean(X)StdDev(X)";
        tGold << "1.000000e+009.000000e-011.000000e-01";
        tGold << "2.000000e+001.900000e+002.000000e-01";
        tGold << "3.000000e+003.100000e+003.000000e-01";
        ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());

        Plato::StandardVector<double> tEmptyBest;
        Plato::StandardVector<double> tEmptyMean;
        Plato::StandardVector<double> tEmptyStdDev;
        EXPECT_FALSE(Plato::pso::print_solution(tEmptyBest, tEmptyMean, tEmptyStdDev));
    }
}

TEST(PlatoTest, PSO_PrintDiagnosticsInvalidArgumentsALPSO)
{
    std::ofstream tFile1;
    Plato::DiagnosticsALPSO<double> tDataALPSO;
    ASSERT_THROW(Plato::pso::print_alpso_diagnostics_header(tDataALPSO, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_alpso_diagnostics_header(tDataALPSO, tFile1, true /* print message */), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tFile1, true /* print message */), std::invalid_argument);

    Plato::DiagnosticsBCPSO<double> tDataPSO;
    ASSERT_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tFile1), std::invalid_argument);
    ASSERT_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tFile1, true /* print message */), std::invalid_argument);
}

TEST(PlatoTest, PSO_PrintDiagnosticsALPSO)
{
    int tMySize = 0;
    MPI_Comm_size(MPI_COMM_WORLD, &tMySize);
    if(tMySize <= 1)
    {
        std::ofstream tWriteFile;
        tWriteFile.open("MyFile.txt");
        Plato::DiagnosticsBCPSO<double> tDataPSO;
        const size_t tNumConstraints = 2;
        Plato::DiagnosticsALPSO<double> tDataALPSO(tNumConstraints);
        tDataPSO.mNumConstraints = tNumConstraints;

        // **** AUGMENTED LAGRANGIAN OUTPUT ****
        tDataPSO.mNumIter = 0;
        tDataPSO.mObjFuncCount = 1;
        tDataPSO.mCurrentGlobalBestObjFuncValue = 1;
        tDataPSO.mMeanCurrentBestObjFuncValues = 1.5;
        tDataPSO.mStdDevCurrentBestObjFuncValues = 2.34e-2;
        tDataPSO.mTrustRegionMultiplier = 0.5;
        ASSERT_NO_THROW(Plato::pso::print_alpso_diagnostics_header(tDataALPSO, tWriteFile));
        ASSERT_NO_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tWriteFile));

        tDataPSO.mNumIter = 1;
        tDataPSO.mObjFuncCount = 10;
        tDataPSO.mCurrentGlobalBestObjFuncValue = 0.1435;
        tDataPSO.mMeanCurrentBestObjFuncValues = 0.78;
        tDataPSO.mStdDevCurrentBestObjFuncValues = 0.298736;
        tDataPSO.mTrustRegionMultiplier = 3.45656e-1;
        ASSERT_NO_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tWriteFile));

        // **** AUGMENTED LAGRANGIAN OUTPUT ****
        tDataALPSO.mNumIter = 1;
        tDataALPSO.mAugLagFuncCount = 10;
        tDataALPSO.mCurrentGlobalBestAugLagValue = 1.2359e-1;
        tDataALPSO.mMeanCurrentBestAugLagValues = 3.2359e-1;
        tDataALPSO.mStdDevCurrentBestAugLagValues = 3.2359e-2;
        tDataALPSO.mCurrentGlobalBestObjFuncValue = 8.2359e-2;
        tDataALPSO.mMeanCurrentBestObjFuncValues = 9.2359e-2;
        tDataALPSO.mStdDevCurrentBestObjFuncValues = 2.2359e-2;
        tDataALPSO.mCurrentGlobalBestConstraintValues[0] = 1.23e-5;
        tDataALPSO.mCurrentGlobalBestConstraintValues[1] = 3.65e-3;
        tDataALPSO.mMeanCurrentBestConstraintValues[0] = 4.23e-5;
        tDataALPSO.mMeanCurrentBestConstraintValues[1] = 6.65e-3;
        tDataALPSO.mStdDevCurrentBestConstraintValues[0] = 1.23e-5;
        tDataALPSO.mStdDevCurrentBestConstraintValues[1] = 8.65e-4;
        tDataALPSO.mMeanCurrentPenaltyMultipliers[0] = 1;
        tDataALPSO.mMeanCurrentPenaltyMultipliers[1] = 2;
        tDataALPSO.mStdDevCurrentPenaltyMultipliers[0] = 0.25;
        tDataALPSO.mStdDevCurrentPenaltyMultipliers[1] = 0.1;
        tDataALPSO.mMeanCurrentLagrangeMultipliers[0] = 1.23e-2;
        tDataALPSO.mMeanCurrentLagrangeMultipliers[1] = 8.65e-1;
        tDataALPSO.mStdDevCurrentLagrangeMultipliers[0] = 9.23e-3;
        tDataALPSO.mStdDevCurrentLagrangeMultipliers[1] = 5.65e-1;
        ASSERT_NO_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tWriteFile));
        tDataPSO.mNumIter = 1;
        tDataPSO.mObjFuncCount = 10;
        ASSERT_NO_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tWriteFile));
        tDataPSO.mNumIter = 2;
        tDataPSO.mObjFuncCount = 20;
        ASSERT_NO_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tWriteFile));
        tDataPSO.mNumIter = 3;
        tDataPSO.mObjFuncCount = 30;
        ASSERT_NO_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tWriteFile));
        tDataALPSO.mNumIter = 2;
        tDataALPSO.mAugLagFuncCount = 40;
        ASSERT_NO_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tWriteFile));
        tDataPSO.mNumIter = 1;
        tDataPSO.mObjFuncCount = 10;
        ASSERT_NO_THROW(Plato::pso::print_alpso_inner_diagnostics(tDataPSO, tWriteFile));
        tDataALPSO.mNumIter = 3;
        tDataALPSO.mAugLagFuncCount = 50;
        ASSERT_NO_THROW(Plato::pso::print_alpso_outer_diagnostics(tDataALPSO, tWriteFile));
        tWriteFile.close();

        std::ifstream tReadFile;
        tReadFile.open("MyFile.txt");
        std::string tInputString;
        std::stringstream tReadData;
        while(tReadFile >> tInputString)
        {
            tReadData << tInputString.c_str();
        }
        tReadFile.close();
        std::system("rm -f MyFile.txt");

        std::stringstream tGold;
        tGold << "IterF-countBest(L)Mean(L)StdDev(L)Best(F)Mean(F)StdDev(F)TR-RadiusBest(H1)Mean(H1)StdDev(H1)Mean(P1)StdDev(P1)Mean(l1)StdDev(l1)";
        tGold << "Best(H2)Mean(H2)StdDev(H2)Mean(P2)StdDev(P2)Mean(l2)StdDev(l2)";
        tGold << "000.000000e+000.000000e+000.000000e+000.000000e+000.000000e+000.000000e+00*0.000000e+000.000000e+00";
        tGold << "0.000000e+000.000000e+000.000000e+000.000000e+000.000000e+00";
        tGold << "0.000000e+000.000000e+000.000000e+000.000000e+000.000000e+000.000000e+000.000000e+00";
        tGold << "1*1.435000e-017.800000e-012.987360e-01***3.456560e-01**************";
        tGold << "1101.235900e-013.235900e-013.235900e-028.235900e-029.235900e-022.235900e-02*1.230000e-054.230000e-051.230000e-051.000000e+002.500000e-01";
        tGold << "1.230000e-029.230000e-033.650000e-036.650000e-038.650000e-042.000000e+001.000000e-018.650000e-015.650000e-01";
        tGold << "1*1.435000e-017.800000e-012.987360e-01***3.456560e-01**************";
        tGold << "2*1.435000e-017.800000e-012.987360e-01***3.456560e-01**************";
        tGold << "3*1.435000e-017.800000e-012.987360e-01***3.456560e-01**************";
        tGold << "2401.235900e-013.235900e-013.235900e-028.235900e-029.235900e-022.235900e-02*1.230000e-054.230000e-051.230000e-051.000000e+002.500000e-01";
        tGold << "1.230000e-029.230000e-033.650000e-036.650000e-038.650000e-042.000000e+001.000000e-018.650000e-015.650000e-01";
        tGold << "1*1.435000e-017.800000e-012.987360e-01***3.456560e-01**************";
        tGold << "3501.235900e-013.235900e-013.235900e-028.235900e-029.235900e-022.235900e-02*1.230000e-054.230000e-051.230000e-051.000000e+002.500000e-01";
        tGold << "1.230000e-029.230000e-033.650000e-036.650000e-038.650000e-042.000000e+001.000000e-018.650000e-015.650000e-01";
        ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
    }
}

TEST(PlatoTest, PSO_checkInertiaMultiplier)
{
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    Plato::ParticleSwarmOperations<double> tOperations(tFactory);
    EXPECT_FALSE(tOperations.checkInertiaMultiplier());

    // MULTIPLIER ABOVE UPPER BOUND = 1, SET VALUE TO DEFAULT = 0.9
    tOperations.setInertiaMultiplier(2);
    EXPECT_TRUE(tOperations.checkInertiaMultiplier());
    const double tTolerance = 1e-6;
    EXPECT_NEAR(0.9, tOperations.getInertiaMultiplier(), tTolerance);

    // MULTIPLIER BELOW LOWER BOUND = -0.2, SET VALUE TO DEFAULT = 0.9
    tOperations.setInertiaMultiplier(-0.3);
    EXPECT_TRUE(tOperations.checkInertiaMultiplier());
    EXPECT_NEAR(0.9, tOperations.getInertiaMultiplier(), tTolerance);
}

TEST(PlatoTest, PSO_updateTrustRegionMultiplier)
{
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);
    Plato::ParticleSwarmOperations<double> tOperations(tFactory);

    // MULTIPLIERS STAYS UNCHANGED
    tOperations.updateTrustRegionMultiplier();
    const double tTolerance = 1e-6;
    EXPECT_NEAR(1.0, tOperations.getTrustRegionMultiplier(), tTolerance);

    // MULTIPLIERS - CONTRACTED
    tOperations.setNumConsecutiveFailures(10 /* default limit */);
    tOperations.updateTrustRegionMultiplier();
    EXPECT_NEAR(0.75, tOperations.getTrustRegionMultiplier(), tTolerance);

    // MULTIPLIERS - EXPANDED
    tOperations.setNumConsecutiveFailures(0);
    tOperations.setNumConsecutiveSuccesses(10 /* default limit */);
    tOperations.updateTrustRegionMultiplier();
    EXPECT_NEAR(3.0, tOperations.getTrustRegionMultiplier(), tTolerance);
}

TEST(PlatoTest, PSO_computeBestObjFunStatistics)
{
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    std::vector<double> tData =
        { 2.47714e-10, 1.85455e-10, 6.77601e-09, 1.31141e-09, 0.00147344, 4.75417e-07, 4.00712e-09, 2.52841e-10,
                3.88818e-10, 0.000664043, 5.2746e-10, 3.68332e-07, 1.21143e-09, 8.75453e-10, 1.81673e-10, 1.42615e-08,
                5.58984e-10, 2.70975e-08, 3.36991e-10, 1.55175e-09 };
    Plato::StandardVector<double> tBestObjFuncValues(tData);
    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
    tDataMng.setCurrentBestObjFuncValues(tBestObjFuncValues);
    tDataMng.computeCurrentBestObjFuncStatistics();

    const double tTolerance = 1e-6;
    EXPECT_NEAR(0.00010692, tDataMng.getMeanCurrentBestObjFuncValues(), tTolerance);
    EXPECT_NEAR(0.000354175, tDataMng.getStdDevCurrentBestObjFuncValues(), tTolerance);
}

TEST(PlatoTest, PSO_findBestParticlePositions_BoundConstrainedStageMng)
{
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 5;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
    tDataMng.setCurrentGlobalBestObjFunValue(0.000423009);
    std::vector<double> tData = { 0.00044607, 0.0639247, 3.9283e-05, 0.0318453, 0.000420515 };
    Plato::StandardVector<double> tCurrentObjFuncValues(tData);
    tDataMng.setCurrentObjFuncValues(tCurrentObjFuncValues);

    tData = { 0.000423009, 0.0008654, 0.00174032, 0.000871822, 0.000426448 };
    Plato::StandardVector<double> tBestObjFuncValues(tData);
    tDataMng.setCurrentBestObjFuncValues(tBestObjFuncValues);

    Plato::StandardMultiVector<double> tControls(tNumParticles, tNumControls);
    tControls(0, 0) = 1.02069; tControls(0, 1) = 1.04138;
    tControls(1, 0) = 1.03309; tControls(1, 1) = 1.0422;
    tControls(2, 0) = 0.998152; tControls(2, 1) = 0.996907;
    tControls(3, 0) = 1.01857; tControls(3, 1) = 1.05524;
    tControls(4, 0) = 1.0205; tControls(4, 1) = 1.04138;
    tDataMng.setCurrentParticles(tControls);

    tControls(0, 0) = 1.02056; tControls(0, 1) = 1.0415;
    tControls(1, 0) = 1.02941; tControls(1, 1) = 1.05975;
    tControls(2, 0) = 0.98662; tControls(2, 1) = 0.97737;
    tControls(3, 0) = 1.02876; tControls(3, 1) = 1.05767;
    tControls(4, 0) = 1.02065; tControls(4, 1) = 1.04175;
    tDataMng.setBestParticlePositions(tControls);

    // FIND BEST PARTICLE POSITIONS
    tDataMng.cacheGlobalBestObjFunValue();
    tDataMng.updateBestParticlesData();
    tDataMng.findGlobalBestParticle();

    const double tTolerance = 1e-6;
    EXPECT_NEAR(3.9283e-05, tDataMng.getCurrentGlobalBestObjFuncValue(), tTolerance);
    EXPECT_NEAR(0.000423009, tDataMng.getPreviousGlobalBestObjFuncValue(), tTolerance);
    EXPECT_NEAR(0.998152, tDataMng.getGlobalBestParticlePosition()[0], tTolerance);
    EXPECT_NEAR(0.996907, tDataMng.getGlobalBestParticlePosition()[1], tTolerance);
    // CHECK CURRENT BEST PARTICLE POSITIONS
    EXPECT_NEAR(1.02056, tDataMng.getBestParticlePosition(0)[0], tTolerance);
    EXPECT_NEAR(1.0415, tDataMng.getBestParticlePosition(0)[1], tTolerance);
    EXPECT_NEAR(1.02941, tDataMng.getBestParticlePosition(1)[0], tTolerance);
    EXPECT_NEAR(1.05975, tDataMng.getBestParticlePosition(1)[1], tTolerance);
    EXPECT_NEAR(0.998152, tDataMng.getBestParticlePosition(2)[0], tTolerance);
    EXPECT_NEAR(0.996907, tDataMng.getBestParticlePosition(2)[1], tTolerance);
    EXPECT_NEAR(1.02876, tDataMng.getBestParticlePosition(3)[0], tTolerance);
    EXPECT_NEAR(1.05767, tDataMng.getBestParticlePosition(3)[1], tTolerance);
    EXPECT_NEAR(1.0205, tDataMng.getBestParticlePosition(4)[0], tTolerance);
    EXPECT_NEAR(1.04138, tDataMng.getBestParticlePosition(4)[1], tTolerance);
    // CHECK CURRENT BEST OBJECTIVE FUNCTION VALUES
    EXPECT_NEAR(0.000423009, tDataMng.getCurrentBestObjFuncValues()[0], tTolerance);
    EXPECT_NEAR(0.0008654, tDataMng.getCurrentBestObjFuncValues()[1], tTolerance);
    EXPECT_NEAR(3.9283e-05, tDataMng.getCurrentBestObjFuncValues()[2], tTolerance);
    EXPECT_NEAR(0.000871822, tDataMng.getCurrentBestObjFuncValues()[3], tTolerance);
    EXPECT_NEAR(0.000420515, tDataMng.getCurrentBestObjFuncValues()[4], tTolerance);
}

TEST(PlatoTest, PSO_BoundConstrainedStageMng)
{
    const size_t tNumControls = 2;
    const size_t tNumParticles = 1;
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
    tDataMng.setCurrentParticle(2.0);

    std::shared_ptr<Plato::GradFreeRosenbrock<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();
    Plato::ParticleSwarmStageMngBCPSO<double> tStageMng(tFactory, tObjective);
    tStageMng.evaluateObjective(tDataMng);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(401, tDataMng.getCurrentObjFuncValue(0 /* particle index */), tTolerance);
}

TEST(PlatoTest, PSO_AugmentedLagrangianStageMng)
{
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 2;
    const size_t tNumConstraints = 1;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateDual(tNumParticles, tNumConstraints);
    tFactory->allocateControl(tNumControls, tNumParticles);

    std::shared_ptr<Plato::GradFreeRosenbrock<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();
    std::shared_ptr<Plato::GradFreeRadius<double>> tConstraint = std::make_shared<Plato::GradFreeRadius<double>>();
    std::shared_ptr<Plato::GradFreeCriteriaList<double>> tConstraintList = std::make_shared<Plato::GradFreeCriteriaList<double>>();
    tConstraintList->add(tConstraint);

    Plato::ParticleSwarmStageMngALPSO<double> tStageMng(tFactory, tObjective, tConstraintList);

    // FIRST AUGMENTED LAGRANGIAN FUNCTION EVALUATION
    Plato::StandardVector<double> tLagrangianValues(tNumParticles);
    Plato::StandardMultiVector<double> tControl(tNumParticles, tNumControls);
    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
    tDataMng.setCurrentParticle(0 /* particle index */, 0.725 /* scalar value */);
    tDataMng.setCurrentParticle(1 /* particle index */, 0.705 /* scalar value */);
    tStageMng.evaluateObjective(tDataMng);
    const double tTolerance = 1e-6;
    EXPECT_NEAR(4.053290625, tDataMng.getCurrentObjFuncValue(0 /* particle index */), tTolerance); // augmented Lagrangian
    EXPECT_NEAR(4.4123850625, tDataMng.getCurrentObjFuncValue(1 /* particle index */), tTolerance); // augmented Lagrangian

    // FIND CURRENT SET OF BEST PARTICLES
    tStageMng.findBestParticlePositions(tDataMng);
    EXPECT_NEAR(4.0506640625, tStageMng.getCurrentBestObjFuncValue(0 /* particle index */), tTolerance); // objective function
    EXPECT_NEAR(4.4123850625, tStageMng.getCurrentBestObjFuncValues()[1 /* particle index */], tTolerance); // objective function

    // UPDATE PENALTY MULTIPLIERS BASED ON BEST CONSTRAINT VALUES
    tStageMng.updatePenaltyMultipliers();
    EXPECT_NEAR(2, tStageMng.getPenaltyMultipliers(0  /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0.5, tStageMng.getPenaltyMultiplier(0  /* constraint index */, 1  /* particle index */), tTolerance);

    // UPDATE LAGRANGE MULTIPLIERS BASED ON BEST CONSTRAINT VALUES
    tStageMng.updateLagrangeMultipliers();
    EXPECT_NEAR(0.205, tStageMng.getLagrangeMultipliers(0  /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0, tStageMng.getLagrangeMultiplier(0  /* constraint index */, 1  /* particle index */), tTolerance);

    tStageMng.computeCriteriaStatistics();
    EXPECT_NEAR(4.0506640625, tStageMng.getCurrentGlobalBestObjFuncValue(), tTolerance);
    EXPECT_NEAR(4.2315245625, tStageMng.getMeanCurrentBestObjFuncValues(), tTolerance);
    EXPECT_NEAR(0.255775372, tStageMng.getStdDevCurrentBestObjFuncValues(), tTolerance);
    EXPECT_NEAR(0.02265, tStageMng.getMeanCurrentBestConstraintValues(0  /* constraint index */), tTolerance);
    EXPECT_NEAR(0.04044650788, tStageMng.getStdDevCurrentBestConstraintValues(0  /* constraint index */), tTolerance);
    EXPECT_NEAR(-0.00595, tStageMng.getCurrentGlobalBestConstraintValue(0  /* constraint index */), tTolerance);

    // SECOND CALL: EVALUATE AUGMENTED LAGRANGIAN FUNCTION
    tDataMng.setCurrentParticle(0  /* particle index */, 0.715  /* scalar value */);
    tDataMng.setCurrentParticle(1  /* particle index */, 0.695  /* scalar value */);
    tStageMng.evaluateObjective(tDataMng);
    EXPECT_NEAR(4.2392603175, tDataMng.getCurrentObjFuncValue(0  /* particle index */), tTolerance); // augmented Lagrangian
    EXPECT_NEAR(4.5863650625, tDataMng.getCurrentObjFuncValue(1  /* particle index */), tTolerance); // augmented Lagrangian
    EXPECT_NEAR(4.2336500625, tStageMng.getCurrentObjFuncValue(0  /* particle index */), tTolerance); // objective function
    EXPECT_NEAR(4.5863650625, tStageMng.getCurrentObjFuncValues()[1 /* particle index */], tTolerance); // objective function
    EXPECT_NEAR(4.0506640625, tStageMng.getPreviousBestObjFuncValue(0  /* particle index */), tTolerance); // objective function
    EXPECT_NEAR(4.4123850625, tStageMng.getPreviousBestObjFuncValues()[1 /* particle index */], tTolerance); // objective function
    EXPECT_NEAR(0.02245, tStageMng.getCurrentConstraintValue(0  /* constraint index */, 0  /* particle index */), tTolerance);
    EXPECT_NEAR(-0.03395, tStageMng.getCurrentConstraintValues(0  /* constraint index */)[1], tTolerance);

    // FIND CURRENT SET OF BEST PARTICLES
    tStageMng.findBestParticlePositions(tDataMng);
    EXPECT_NEAR(4.0506640625, tStageMng.getPreviousBestObjFuncValue(0  /* particle index */), tTolerance); // objective function
    EXPECT_NEAR(4.4123850625, tStageMng.getPreviousBestObjFuncValues()[1 /* particle index */], tTolerance); // objective function
    EXPECT_NEAR(0.05125, tStageMng.getPreviousBestConstraintValue(0  /* constraint index */, 0  /* particle index */), tTolerance);
    EXPECT_NEAR(-0.00595, tStageMng.getPreviousBestConstraintValues(0  /* constraint index */)[1], tTolerance);
    EXPECT_NEAR(0.05125, tStageMng.getCurrentBestConstraintValue(0  /* constraint index */, 0  /* particle index */), tTolerance);
    EXPECT_NEAR(-0.00595, tStageMng.getCurrentBestConstraintValues(0  /* constraint index */)[1 /* particle index */], tTolerance);

    tStageMng.updatePenaltyMultipliers();
    EXPECT_NEAR(22.63846284534, tStageMng.getPenaltyMultipliers(0  /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0.25, tStageMng.getPenaltyMultipliers(0  /* constraint index */)[1], tTolerance);

    tStageMng.updateLagrangeMultipliers();
    EXPECT_NEAR(2.5254424416477, tStageMng.getLagrangeMultipliers(0  /* constraint index */)[0], tTolerance);
    EXPECT_NEAR(0, tStageMng.getLagrangeMultipliers(0  /* constraint index */)[1], tTolerance);

    tStageMng.computeCriteriaStatistics();
    EXPECT_NEAR(4.0506640625, tStageMng.getCurrentGlobalBestObjFuncValue(), tTolerance);
    EXPECT_NEAR(4.2315245625, tStageMng.getMeanCurrentBestObjFuncValues(), tTolerance);
    EXPECT_NEAR(0.255775372, tStageMng.getStdDevCurrentBestObjFuncValues(), tTolerance);
    EXPECT_NEAR(0.02265, tStageMng.getMeanCurrentBestConstraintValues(0  /* constraint index */), tTolerance);
    EXPECT_NEAR(0.04044650788, tStageMng.getStdDevCurrentBestConstraintValues(0  /* constraint index */), tTolerance);
    EXPECT_NEAR(-0.00595, tStageMng.getCurrentGlobalBestConstraintValue(0  /* constraint index */), tTolerance);
}

TEST(PlatoTest, PSO_ParserBCPSO)
{
    Plato::InputData tInputData;
    EXPECT_TRUE(tInputData.empty());
    ASSERT_STREQ("Input Data", tInputData.name().c_str());

    Plato::ParticleSwarmParser<double> tParserPSO;
    Plato::InputDataBCPSO<double> tInputsPSO;
    tParserPSO.parse(tInputData, tInputsPSO);

    // ********* TEST: OPTIONS NODE NOT DEFINE -> USE DEFAULT PARAMETERS *********
    EXPECT_FALSE(tInputsPSO.mOutputSolution);
    EXPECT_FALSE(tInputsPSO.mOutputDiagnostics);
    EXPECT_FALSE(tInputsPSO.mDisableStdDevStoppingTol);
    EXPECT_EQ(10u, tInputsPSO.mNumParticles);
    EXPECT_EQ(1000u, tInputsPSO.mMaxNumIterations);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveSuccesses);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(1.0, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.9, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(5e-4, tInputsPSO.mMeanBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mStdDevBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-10, tInputsPSO.mGlobalBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(4.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.75, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);

    // ********* TEST: OPTIONS NODE DEFINED, BUT PARAMETERS NOT SPECIFIED -> USE DEFAULT PARAMETERS *********
    Plato::InputData tOptions("Options");
    EXPECT_TRUE(tOptions.empty());
    tParserPSO.parse(tOptions, tInputsPSO);
    EXPECT_FALSE(tInputsPSO.mOutputSolution);
    EXPECT_FALSE(tInputsPSO.mOutputDiagnostics);
    EXPECT_FALSE(tInputsPSO.mDisableStdDevStoppingTol);
    EXPECT_EQ(10u, tInputsPSO.mNumParticles);
    EXPECT_EQ(1000u, tInputsPSO.mMaxNumIterations);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveSuccesses);

    EXPECT_NEAR(1.0, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.9, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(5e-4, tInputsPSO.mMeanBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mStdDevBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-10, tInputsPSO.mGlobalBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(4.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.75, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);

    // ********* TEST: SET PARAMETERS AND ADD OPTIONS NODE TO OPTIMIZER'S NODE -> NON-DEFAULT VALUES ARE EXPECTED *********
    tOptions.add<std::string>("NumParticles", "20");
    tOptions.add<std::string>("MaxNumOuterIterations", "100");
    tOptions.add<std::string>("MaxNumConsecutiveFailures", "7");
    tOptions.add<std::string>("MaxNumConsecutiveSuccesses", "8");
    tOptions.add<std::string>("OutputSolution", "true");
    tOptions.add<std::string>("OutputDiagnosticsToFile", "false");
    tOptions.add<std::string>("DisableStdDevStoppingTolerance", "true");
    tOptions.add<std::string>("InertiaMultiplier", "0.55");
    tOptions.add<std::string>("ParticleVelocityTimeStep", "0.75");
    tOptions.add<std::string>("SocialBehaviorMultiplier", "0.85");
    tOptions.add<std::string>("MeanBestObjFuncTolerance", "0.001");
    tOptions.add<std::string>("CognitiveBehaviorMultiplier", "0.6");
    tOptions.add<std::string>("StdDevBestObjFuncTolerance", "0.0001");
    tOptions.add<std::string>("GlobalBestObjFuncTolerance", "0.000001");
    tOptions.add<std::string>("TrustRegionExpansionMultiplier", "2");
    tOptions.add<std::string>("TrustRegionContractionMultiplier", "0.5");
    Plato::InputData tOptimizerNodeOne("OptimizerNode");
    tOptimizerNodeOne.add<Plato::InputData>("Options", tOptions);

    tParserPSO.parse(tOptimizerNodeOne, tInputsPSO);
    EXPECT_FALSE(tInputsPSO.mOutputDiagnostics);
    EXPECT_TRUE(tInputsPSO.mOutputSolution);
    EXPECT_TRUE(tInputsPSO.mDisableStdDevStoppingTol);
    EXPECT_EQ(20u, tInputsPSO.mNumParticles);
    EXPECT_EQ(100u, tInputsPSO.mMaxNumIterations);
    EXPECT_EQ(7u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(8u, tInputsPSO.mMaxNumConsecutiveSuccesses);

    EXPECT_NEAR(0.75, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.55, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.85, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-3, tInputsPSO.mMeanBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(0.6, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-4, tInputsPSO.mStdDevBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mGlobalBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(2.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.5, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);

    // ********* TEST: SET A HANDFULL OF PARAMETERS AND ADD OPTIONS NODE TO OPTIMIZER'S NODE -> A FEW DEFAULT VALUES ARE EXPECTED *********
    Plato::InputData tOptionsTwo("Options");
    tOptionsTwo.add<std::string>("NumParticles", "20");
    tOptionsTwo.add<std::string>("MaxNumConsecutiveFailures", "7");
    tOptionsTwo.add<std::string>("MaxNumConsecutiveSuccesses", "8");
    tOptionsTwo.add<std::string>("InertiaMultiplier", "0.55");
    tOptionsTwo.add<std::string>("ParticleVelocityTimeStep", "0.75");
    tOptionsTwo.add<std::string>("MeanBestObjFuncTolerance", "0.001");
    tOptionsTwo.add<std::string>("CognitiveBehaviorMultiplier", "0.6");
    tOptionsTwo.add<std::string>("TrustRegionContractionMultiplier", "0.5");
    Plato::InputData tOptimizerNodeTwo("OptimizerNode");
    tOptimizerNodeTwo.add<Plato::InputData>("Options", tOptionsTwo);
    Plato::InputDataBCPSO<double> tInputsTwoPSO;
    tParserPSO.parse(tOptimizerNodeTwo, tInputsTwoPSO);

    EXPECT_FALSE(tInputsTwoPSO.mOutputSolution);
    EXPECT_TRUE(tInputsTwoPSO.mOutputDiagnostics);
    EXPECT_FALSE(tInputsTwoPSO.mDisableStdDevStoppingTol);
    EXPECT_EQ(20u, tInputsTwoPSO.mNumParticles);
    EXPECT_EQ(1000u, tInputsTwoPSO.mMaxNumIterations);
    EXPECT_EQ(7u, tInputsTwoPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(8u, tInputsTwoPSO.mMaxNumConsecutiveSuccesses);

    EXPECT_NEAR(0.75, tInputsTwoPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.55, tInputsTwoPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsTwoPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(0.001, tInputsTwoPSO.mMeanBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(0.6, tInputsTwoPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-6, tInputsTwoPSO.mStdDevBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-10, tInputsTwoPSO.mGlobalBestObjFuncTolerance, tTolerance);
    EXPECT_NEAR(4.0, tInputsTwoPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.5, tInputsTwoPSO.mTrustRegionContractionMultiplier, tTolerance);
}

TEST(PlatoTest, PSO_ParserALPSO)
{
    Plato::InputData tInputData;
    EXPECT_TRUE(tInputData.empty());
    ASSERT_STREQ("Input Data", tInputData.name().c_str());

    Plato::ParticleSwarmParser<double> tParserPSO;
    Plato::InputDataALPSO<double> tInputsPSO;
    tParserPSO.parse(tInputData, tInputsPSO);

    // ********* TEST: OPTIONS NODE NOT DEFINE -> USE DEFAULT PARAMETERS *********
    EXPECT_FALSE(tInputsPSO.mOutputSolution);
    EXPECT_FALSE(tInputsPSO.mOutputDiagnostics);
    EXPECT_FALSE(tInputsPSO.mDisableStdDevStoppingTol);
    EXPECT_TRUE(tInputsPSO.mConstraintTypes.empty());
    EXPECT_EQ(10u, tInputsPSO.mNumParticles);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumInnerIter);
    EXPECT_EQ(1000u, tInputsPSO.mMaxNumOuterIter);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveSuccesses);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(1.0, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.9, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(5e-4, tInputsPSO.mMeanBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(2.0, tInputsPSO.mPenaltyExpansionMultiplier, tTolerance);
    EXPECT_NEAR(100, tInputsPSO.mPenaltyMultiplierUpperBound, tTolerance);
    EXPECT_NEAR(0.5, tInputsPSO.mPenaltyContractionMultiplier, tTolerance);
    EXPECT_NEAR(1e-4, tInputsPSO.mFeasibilityInexactnessTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mStdDevBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-10, tInputsPSO.mGlobalBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(4.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.75, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);

    // ********* TEST: OPTIONS NODE DEFINED, BUT PARAMETERS NOT SPECIFIED -> USE DEFAULT PARAMETERS *********
    Plato::InputData tOptions("Options");
    EXPECT_TRUE(tOptions.empty());
    tParserPSO.parse(tOptions, tInputsPSO);
    EXPECT_FALSE(tInputsPSO.mOutputSolution);
    EXPECT_FALSE(tInputsPSO.mOutputDiagnostics);
    EXPECT_FALSE(tInputsPSO.mDisableStdDevStoppingTol);
    EXPECT_TRUE(tInputsPSO.mConstraintTypes.empty());
    EXPECT_EQ(10u, tInputsPSO.mNumParticles);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumInnerIter);
    EXPECT_EQ(1000u, tInputsPSO.mMaxNumOuterIter);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(10u, tInputsPSO.mMaxNumConsecutiveSuccesses);

    EXPECT_NEAR(1.0, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.9, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(5e-4, tInputsPSO.mMeanBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(2.0, tInputsPSO.mPenaltyExpansionMultiplier, tTolerance);
    EXPECT_NEAR(100, tInputsPSO.mPenaltyMultiplierUpperBound, tTolerance);
    EXPECT_NEAR(0.5, tInputsPSO.mPenaltyContractionMultiplier, tTolerance);
    EXPECT_NEAR(1e-4, tInputsPSO.mFeasibilityInexactnessTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mStdDevBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-10, tInputsPSO.mGlobalBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(4.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.75, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);

    // ********* TEST: SET PARAMETERS AND ADD OPTIONS NODE TO OPTIMIZER'S NODE -> NON-DEFAULT VALUES ARE EXPECTED *********
    tOptions.add<std::string>("NumParticles", "20");
    tOptions.add<std::string>("MaxNumOuterIterations", "100");
    tOptions.add<std::string>("MaxNumConsecutiveFailures", "7");
    tOptions.add<std::string>("MaxNumConsecutiveSuccesses", "8");
    tOptions.add<std::string>("OutputSolution", "true");
    tOptions.add<std::string>("OutputDiagnosticsToFile", "false");
    tOptions.add<std::string>("DisableStdDevStoppingTolerance", "true");
    tOptions.add<std::string>("InertiaMultiplier", "0.55");
    tOptions.add<std::string>("ParticleVelocityTimeStep", "0.75");
    tOptions.add<std::string>("SocialBehaviorMultiplier", "0.85");
    tOptions.add<std::string>("MeanBestAugLagFuncTolerance", "0.001");
    tOptions.add<std::string>("CognitiveBehaviorMultiplier", "0.6");
    tOptions.add<std::string>("StdDevBestAugLagFuncTolerance", "0.0001");
    tOptions.add<std::string>("GlobalBestAugLagFuncTolerance", "0.000001");
    tOptions.add<std::string>("TrustRegionExpansionMultiplier", "2");
    tOptions.add<std::string>("TrustRegionContractionMultiplier", "0.5");
    tOptions.add<std::string>("PenaltyExpansionMultiplier", "4");
    tOptions.add<std::string>("PenaltyMultiplierUpperBound", "10");
    tOptions.add<std::string>("PenaltyContractionMultiplier", "0.25");
    tOptions.add<std::string>("FeasibilityInexactnessTolerance", "0.01");
    Plato::InputData tOptimizerNode("OptimizerNode");
    tOptimizerNode.add<Plato::InputData>("Options", tOptions);

    Plato::InputData tConstraint("Constraint");
    tConstraint.add<std::string>("ValueStageName", "ConstraintEvaluation");
    tConstraint.add<std::string>("ReferenceValue", "1.0");
    tConstraint.add<std::string>("TargetValue", "0.0");
    tOptimizerNode.add<Plato::InputData>("Constraint", tConstraint);

    tParserPSO.parse(tOptimizerNode, tInputsPSO);

    EXPECT_TRUE(tInputsPSO.mOutputSolution);
    EXPECT_FALSE(tInputsPSO.mOutputDiagnostics);
    EXPECT_FALSE(tInputsPSO.mConstraintTypes.empty());
    EXPECT_TRUE(tInputsPSO.mDisableStdDevStoppingTol);
    EXPECT_EQ(20u, tInputsPSO.mNumParticles);
    EXPECT_EQ(100u, tInputsPSO.mMaxNumOuterIter);
    EXPECT_EQ(7u, tInputsPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(8u, tInputsPSO.mMaxNumConsecutiveSuccesses);
    EXPECT_EQ(1u, tInputsPSO.mConstraintTypes.size());
    EXPECT_EQ(Plato::particle_swarm::INEQUALITY, tInputsPSO.mConstraintTypes[0]);

    EXPECT_NEAR(0.75, tInputsPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.55, tInputsPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.85, tInputsPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-3, tInputsPSO.mMeanBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(0.6, tInputsPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-4, tInputsPSO.mStdDevBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsPSO.mGlobalBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(2.0, tInputsPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.5, tInputsPSO.mTrustRegionContractionMultiplier, tTolerance);
    EXPECT_NEAR(4.0, tInputsPSO.mPenaltyExpansionMultiplier, tTolerance);
    EXPECT_NEAR(10, tInputsPSO.mPenaltyMultiplierUpperBound, tTolerance);
    EXPECT_NEAR(0.25, tInputsPSO.mPenaltyContractionMultiplier, tTolerance);
    EXPECT_NEAR(1e-2, tInputsPSO.mFeasibilityInexactnessTolerance, tTolerance);

    // ********* TEST: SET A HANDFULL OF PARAMETERS AND ADD OPTIONS NODE TO OPTIMIZER'S NODE -> A FEW DEFAULT VALUES ARE EXPECTED *********
    Plato::InputData tOptionsTwo("Options");
    tOptionsTwo.add<std::string>("NumParticles", "20");
    tOptionsTwo.add<std::string>("MaxNumConsecutiveFailures", "7");
    tOptionsTwo.add<std::string>("MaxNumConsecutiveSuccesses", "8");
    tOptionsTwo.add<std::string>("InertiaMultiplier", "0.55");
    tOptionsTwo.add<std::string>("ParticleVelocityTimeStep", "0.75");
    tOptionsTwo.add<std::string>("MeanBestAugLagFuncTolerance", "0.001");
    tOptionsTwo.add<std::string>("CognitiveBehaviorMultiplier", "0.6");
    tOptionsTwo.add<std::string>("TrustRegionContractionMultiplier", "0.5");
    Plato::InputData tOptimizerNodeTwo("OptimizerNode");
    tOptimizerNodeTwo.add<Plato::InputData>("Options", tOptionsTwo);
    Plato::InputDataALPSO<double> tInputsTwoPSO;

    Plato::InputData tConstraint1("Constraint1");
    tConstraint.add<std::string>("ValueStageName", "ConstraintEvaluation1");
    tConstraint.add<std::string>("ReferenceValue", "1.0");
    tConstraint.add<std::string>("TargetValue", "0.0");
    tOptimizerNodeTwo.add<Plato::InputData>("Constraint", tConstraint1);
    Plato::InputData tConstraint2("Constraint2");
    tConstraint2.add<std::string>("ValueStageName", "ConstraintEvaluation2");
    tConstraint2.add<std::string>("ReferenceValue", "1.0");
    tConstraint2.add<std::string>("TargetValue", "0.0");
    tConstraint2.add<std::string>("Type", "Equality");
    tOptimizerNodeTwo.add<Plato::InputData>("Constraint", tConstraint2);

    tParserPSO.parse(tOptimizerNodeTwo, tInputsTwoPSO);

    EXPECT_FALSE(tInputsTwoPSO.mOutputSolution);
    EXPECT_TRUE(tInputsTwoPSO.mOutputDiagnostics);
    EXPECT_FALSE(tInputsTwoPSO.mDisableStdDevStoppingTol);

    EXPECT_EQ(2u, tInputsTwoPSO.mConstraintTypes.size());
    EXPECT_EQ(Plato::particle_swarm::INEQUALITY, tInputsTwoPSO.mConstraintTypes[0]);
    EXPECT_EQ(Plato::particle_swarm::EQUALITY, tInputsTwoPSO.mConstraintTypes[1]);

    EXPECT_EQ(20u, tInputsTwoPSO.mNumParticles);
    EXPECT_EQ(10u, tInputsTwoPSO.mMaxNumInnerIter);
    EXPECT_EQ(1000u, tInputsTwoPSO.mMaxNumOuterIter);
    EXPECT_EQ(7u, tInputsTwoPSO.mMaxNumConsecutiveFailures);
    EXPECT_EQ(8u, tInputsTwoPSO.mMaxNumConsecutiveSuccesses);

    EXPECT_NEAR(0.75, tInputsTwoPSO.mTimeStep, tTolerance);
    EXPECT_NEAR(0.55, tInputsTwoPSO.mInertiaMultiplier, tTolerance);
    EXPECT_NEAR(0.8, tInputsTwoPSO.mSocialBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(0.6, tInputsTwoPSO.mCognitiveBehaviorMultiplier, tTolerance);
    EXPECT_NEAR(1e-3, tInputsTwoPSO.mMeanBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(2.0, tInputsTwoPSO.mPenaltyExpansionMultiplier, tTolerance);
    EXPECT_NEAR(100, tInputsTwoPSO.mPenaltyMultiplierUpperBound, tTolerance);
    EXPECT_NEAR(0.5, tInputsTwoPSO.mPenaltyContractionMultiplier, tTolerance);
    EXPECT_NEAR(1e-4, tInputsTwoPSO.mFeasibilityInexactnessTolerance, tTolerance);
    EXPECT_NEAR(1e-6, tInputsTwoPSO.mStdDevBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(1e-10, tInputsTwoPSO.mGlobalBestAugLagFuncTolerance, tTolerance);
    EXPECT_NEAR(4.0, tInputsTwoPSO.mTrustRegionExpansionMultiplier, tTolerance);
    EXPECT_NEAR(0.5, tInputsTwoPSO.mTrustRegionContractionMultiplier, tTolerance);
}

TEST(PlatoTest, PSO_ParserOutputStageName)
{
    Plato::ParticleSwarmParser<double> tParserPSO;
    Plato::InputDataALPSO<double> tInputsPSO;
    Plato::InputData tOptimizerNode("OptimizerNode");
    tParserPSO.parse(tOptimizerNode, tInputsPSO);
    EXPECT_TRUE(tInputsPSO.mOutputStageName.empty());

    Plato::InputData tOutput("Output Data");
    ASSERT_STREQ("Output Data", tOutput.name().c_str());
    tOutput.add<std::string>("OutputStage", "Output To File");
    tOptimizerNode.add<Plato::InputData>("Output", tOutput);
    tParserPSO.parse(tOptimizerNode, tInputsPSO);
    ASSERT_STREQ("Output To File", tInputsPSO.mOutputStageName.c_str());
}

TEST(PlatoTest, PSO_ParserCriteriaData)
{
    Plato::InputData tObjective("Objective");
    tObjective.add<std::string>("ValueStageName", "Mass");

    Plato::InputData tConstraintOne("Constraint");
    tConstraintOne.add<std::string>("ValueStageName", "Volume");
    tConstraintOne.add<std::string>("ReferenceValue", "2.0");
    tConstraintOne.add<std::string>("TargetValue", "1.0");

    Plato::InputData tConstraintTwo("Constraint");
    tConstraintTwo.add<std::string>("ValueStageName", "Stress");

    Plato::InputData tOptimizerNode("OptimizerNode");
    tOptimizerNode.add<Plato::InputData>("Objective", tObjective);
    tOptimizerNode.add<Plato::InputData>("Constraint", tConstraintOne);
    tOptimizerNode.add<Plato::InputData>("Constraint", tConstraintTwo);

    Plato::ParticleSwarmParser<double> Parser;
    std::string tObjFuncName = Parser.getObjectiveStageName(tOptimizerNode);
    ASSERT_STREQ("Mass", tObjFuncName.c_str());

    std::vector<std::string> tConstraintStageNames = Parser.getConstraintStageNames(tOptimizerNode);
    EXPECT_EQ(2u, tConstraintStageNames.size());
    ASSERT_STREQ("Volume", tConstraintStageNames[0].c_str());
    ASSERT_STREQ("Stress", tConstraintStageNames[1].c_str());

    std::vector<double> tConstraintReferenceValues = Parser.getConstraintReferenceValues(tOptimizerNode);
    EXPECT_EQ(2u, tConstraintReferenceValues.size());
    const double tTolerance = 1e-4;
    EXPECT_NEAR(2.0, tConstraintReferenceValues[0], tTolerance);
    EXPECT_NEAR(1.0, tConstraintReferenceValues[1], tTolerance);

    std::vector<double> tConstraintTargetValues = Parser.getConstraintTargetValues(tOptimizerNode);
    EXPECT_EQ(2u, tConstraintTargetValues.size());
    EXPECT_NEAR(1.0, tConstraintTargetValues[0], tTolerance);
    EXPECT_NEAR(0.0, tConstraintTargetValues[1], tTolerance);
}

TEST(PlatoTest, PSO_SolveBCPSO_Rosenbrock)
{
    // ********* ALLOCATE CRITERION *********
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 10;
    Plato::InputDataBCPSO<double> tInputs;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-5);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(5);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::OutputDataBCPSO<double> tOutputs;
    Plato::solve_bcpso<double>(tObjective, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tOutputs.mGlobalBestObjFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestObjFuncValue << ", MEAN = "
            << tOutputs.mMeanBestObjFuncValue << ", STDDEV = " << tOutputs.mStdDevBestObjFuncValue << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

TEST(PlatoTest, PSO_SolveBCPSO_GoldsteinPrice)
{
    // ********* ALLOCATE CRITERION *********
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective =
            std::make_shared<Plato::GradFreeGoldsteinPrice<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    Plato::InputDataBCPSO<double> tInputs;
    tInputs.mStdDevBestObjFuncTolerance = 1e-4;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-2);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(2);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::OutputDataBCPSO<double> tOutputs;
    Plato::solve_bcpso<double>(tObjective, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-2;
    EXPECT_NEAR(3, tOutputs.mGlobalBestObjFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestObjFuncValue << ", MEAN = "
            << tOutputs.mMeanBestObjFuncValue << ", STDDEV = " << tOutputs.mStdDevBestObjFuncValue << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

TEST(PlatoTest, PSO_SolveBCPSO_Himmelblau)
{
    // ********* ALLOCATE CRITERION *********
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective =
            std::make_shared<Plato::GradFreeHimmelblau<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 10;
    Plato::InputDataBCPSO<double> tInputs;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-6);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(6);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::OutputDataBCPSO<double> tOutputs;
    Plato::solve_bcpso<double>(tObjective, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-3;
    EXPECT_NEAR(0, tOutputs.mGlobalBestObjFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestObjFuncValue << ", MEAN = "
            << tOutputs.mMeanBestObjFuncValue << ", STDDEV = " << tOutputs.mStdDevBestObjFuncValue << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

TEST(PlatoTest, PSO_SolveBCPSO_Circle)
{
    // ********* ALLOCATE CRITERION *********
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective =
            std::make_shared<Plato::GradFreeCircle<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 10;
    Plato::InputDataBCPSO<double> tInputs;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-6);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(6);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::OutputDataBCPSO<double> tOutputs;
    Plato::solve_bcpso<double>(tObjective, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-3;
    EXPECT_NEAR(0, tOutputs.mGlobalBestObjFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestObjFuncValue << ", MEAN = "
            << tOutputs.mMeanBestObjFuncValue << ", STDDEV = " << tOutputs.mStdDevBestObjFuncValue << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

TEST(PlatoTest, PSO_SolveBCPSO_Rocket)
{
    // ********* Allocate Core Optimization Data Templates *********
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 15;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    // ********* ALLOCATE NORMALIZATION CONSTANTS *********
    Plato::StandardVector<double> tVector(tNumControls);
    tVector[0] = 0.08; tVector[1] = 0.006;

    // ********* ALLOCATE OBJECTIVE FUNCTION *********
    Plato::AlgebraicRocketInputs<double> tRocketInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
            std::make_shared<Plato::Cylinder<double>>(tRocketInputs.mChamberRadius, tRocketInputs.mChamberLength);
    std::shared_ptr<Plato::GradFreeRocketObjFunc<double>> tObjective =
            std::make_shared<Plato::GradFreeRocketObjFunc<double>>(tVector, tRocketInputs, tGeomModel);
    tObjective->disableOutput();

    // ********* SET TARGET THRUST PROFILE *********
    std::vector<double> tData = ParticleSwarmTest::get_target_thrust_profile();
    Plato::StandardVector<double> tTargetThrustProfile(tData);
    tObjective->setTargetThrustProfile(tTargetThrustProfile);

    // ********* TEST ALGORITHM *********
    Plato::ParticleSwarmAlgorithmBCPSO<double> tAlgorithm(tFactory, tObjective);
    tVector[0] = 1; tVector[1] = 1;
    tAlgorithm.setUpperBounds(tVector);  /* bounds are normalized */
    tVector[0] = 0.06 / tVector[0]; tVector[1] = 0.003 / tVector[1];
    tAlgorithm.setLowerBounds(tVector);  /* bounds are normalized */
    tAlgorithm.setMeanBestObjFuncTolerance(1e-6);
    tAlgorithm.solve();

    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tAlgorithm.getCurrentGlobalBestObjFuncValue(), tTolerance);

    // ********* OUTPUT DIAGNOSTICS *********
    std::cout << "NUM ITERATIONS = " << tAlgorithm.getNumIterations() << "\n";
    std::cout << "OBJECTIVE: BEST = " << tAlgorithm.getCurrentGlobalBestObjFuncValue() << ", MEAN = "
            << tAlgorithm.getMeanCurrentBestObjFuncValues() << ", STDDEV = "
            << tAlgorithm.getStdDevCurrentBestObjFuncValues() << "\n";
    std::cout << tAlgorithm.getStoppingCriterion() << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = "
                << tAlgorithm.getDataMng().getGlobalBestParticlePosition()[tIndex]
                        * tObjective->getNormalizationConstants()[tIndex]
                << ", MEAN = "
                << tAlgorithm.getDataMng().getMeanParticlePositions()[tIndex]
                        * tObjective->getNormalizationConstants()[tIndex]
                << ", STDDEV = "
                << tAlgorithm.getDataMng().getStdDevParticlePositions()[tIndex]
                        * tObjective->getNormalizationConstants()[tIndex] << "\n";
    }

    // ********* TEST THRUST PROFILE SOLUTION *********
    Plato::StandardVector<double> tBestThrustProfileSolution(tData.size());
    tObjective->solve(tAlgorithm.getDataMng().getGlobalBestParticlePosition(), tBestThrustProfileSolution);
    const double tMultiplier = 1.0 / tObjective->getNormTargetThrustProfile();
    tTargetThrustProfile.scale(tMultiplier);
    tBestThrustProfileSolution.scale(tMultiplier);
    PlatoTest::checkVectorData(tTargetThrustProfile, tBestThrustProfileSolution);
}

TEST(PlatoTest, PSO_SolveALPSO_RosenbrockObj_RadiusConstr)
{
    // ********* DEFINE CRITERIA *********
    const double tBound = 2;
    std::shared_ptr<Plato::GradFreeCriterion<double>> tConstraintOne = std::make_shared<Plato::GradFreeRadius<double>>(tBound);
    std::shared_ptr<Plato::GradFreeCriteriaList<double>> tConstraints = std::make_shared<Plato::GradFreeCriteriaList<double>>();
    tConstraints->add(tConstraintOne);
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    const size_t tNumConstraints = 1;
    Plato::InputDataALPSO<double> tInputs;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-5);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(5);
    tInputs.mDual = std::make_shared<Plato::StandardMultiVector<double>>(tNumConstraints, tNumParticles);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::OutputDataALPSO<double> tOutputs;
    Plato::solve_alpso<double>(tObjective, tConstraints, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tOutputs.mGlobalBestAugLagFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestAugLagFuncValue << ", MEAN = "
            << tOutputs.mMeanBestAugLagFuncValue << ", STDDEV = " << tOutputs.mStdDevBestAugLagFuncValue << "\n";

    std::cout << "\nCONSTRAINT #0: BEST = " << (*tOutputs.mGlobalBestConstraintValues)[0] << ", MEAN = "
            << (*tOutputs.mMeanBestConstraintValues)[0] << ", STDDEV = " << (*tOutputs.mStdDevBestConstraintValues)[0]
            << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

TEST(PlatoTest, PSO_SolveALPSO_HimmelblauObj_ShiftedEllipseConstr)
{
    // ********* DEFINE CRITERIA *********
    std::shared_ptr<Plato::GradFreeShiftedEllipse<double>> tMyConstraint =
            std::make_shared<Plato::GradFreeShiftedEllipse<double>>();
    tMyConstraint->define(-2., 2., -3., 3.);
    std::shared_ptr<Plato::GradFreeCriteriaList<double>> tConstraints =
            std::make_shared<Plato::GradFreeCriteriaList<double>>();
    tConstraints->add(tMyConstraint);
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective =
            std::make_shared<Plato::GradFreeHimmelblau<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    const size_t tNumConstraints = 1;
    Plato::InputDataALPSO<double> tInputs;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(-5);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(5);
    tInputs.mDual = std::make_shared<Plato::StandardMultiVector<double>>(tNumConstraints, tNumParticles);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::OutputDataALPSO<double> tOutputs;
    Plato::solve_alpso<double>(tObjective, tConstraints, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tOutputs.mGlobalBestAugLagFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestAugLagFuncValue << ", MEAN = "
            << tOutputs.mMeanBestAugLagFuncValue << ", STDDEV = " << tOutputs.mStdDevBestAugLagFuncValue << "\n";

    std::cout << "\nCONSTRAINT #0: BEST = " << (*tOutputs.mGlobalBestConstraintValues)[0] << ", MEAN = "
            << (*tOutputs.mMeanBestConstraintValues)[0] << ", STDDEV = " << (*tOutputs.mStdDevBestConstraintValues)[0]
            << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    std::vector<double> tGold = { -3.77931, -3.28319 };
    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
        EXPECT_NEAR(tGold[tIndex], (*tOutputs.mGlobalBestParticles)[tIndex], tTolerance);
    }
}

TEST(PlatoTest, PSO_SolveALPSO_CircleObj_RadiusConstr)
{
    // ********* DEFINE CRITERIA *********
    std::shared_ptr<Plato::GradFreeRadius<double>> tMyConstraint = std::make_shared<Plato::GradFreeRadius<double>>();
    std::shared_ptr<Plato::GradFreeCriteriaList<double>> tConstraints =
            std::make_shared<Plato::GradFreeCriteriaList<double>>();
    tConstraints->add(tMyConstraint);
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective = std::make_shared<Plato::GradFreeCircle<double>>();

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 10;
    const size_t tNumConstraints = 1;
    Plato::InputDataALPSO<double> tInputs;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(0);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(1);
    tInputs.mDual = std::make_shared<Plato::StandardMultiVector<double>>(tNumConstraints, tNumParticles);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::OutputDataALPSO<double> tOutputs;
    Plato::solve_alpso<double>(tObjective, tConstraints, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-2;
    EXPECT_NEAR(0, tOutputs.mGlobalBestAugLagFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestAugLagFuncValue << ", MEAN = "
            << tOutputs.mMeanBestAugLagFuncValue << ", STDDEV = " << tOutputs.mStdDevBestAugLagFuncValue << "\n";

    std::cout << "\nCONSTRAINT #0: BEST = " << (*tOutputs.mGlobalBestConstraintValues)[0] << ", MEAN = "
            << (*tOutputs.mMeanBestConstraintValues)[0] << ", STDDEV = " << (*tOutputs.mStdDevBestConstraintValues)[0]
            << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

TEST(PlatoTest, PSO_SolveALPSO_CircleObj_RadiusEqConstr)
{
    // ********* DEFINE CRITERIA *********
    std::shared_ptr<Plato::GradFreeRadius<double>> tMyConstraint = std::make_shared<Plato::GradFreeRadius<double>>();
    std::shared_ptr<Plato::GradFreeCriteriaList<double>> tConstraints =
            std::make_shared<Plato::GradFreeCriteriaList<double>>();
    tConstraints->add(tMyConstraint, Plato::particle_swarm::EQUALITY);
    std::shared_ptr<Plato::GradFreeCriterion<double>> tObjective = std::make_shared<Plato::GradFreeCircle<double>>(1.0, 2.0);

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    const size_t tNumConstraints = 1;
    Plato::InputDataALPSO<double> tInputs;
    tInputs.mOutputDiagnostics = true;
    tInputs.mPenaltyMultiplierUpperBound = 1e3;
    tInputs.mStdDevBestAugLagFuncTolerance = 5e-4;
    tInputs.mCriteriaEvals = std::make_shared<Plato::StandardVector<double>>(tNumParticles);
    tInputs.mParticlesLowerBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesLowerBounds->fill(0);
    tInputs.mParticlesUpperBounds = std::make_shared<Plato::StandardVector<double>>(tNumControls);
    tInputs.mParticlesUpperBounds->fill(2);
    tInputs.mDual = std::make_shared<Plato::StandardMultiVector<double>>(tNumConstraints, tNumParticles);
    tInputs.mParticles = std::make_shared<Plato::StandardMultiVector<double>>(tNumParticles, tNumControls);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::OutputDataALPSO<double> tOutputs;
    Plato::solve_alpso<double>(tObjective, tConstraints, tInputs, tOutputs);

    // ********* DIAGNOSTICS *********
    const double tTolerance = 1e-2;
    EXPECT_NEAR(2.67805, tOutputs.mGlobalBestAugLagFuncValue, tTolerance);

    std::cout << "\nNUM ITERATIONS = " << tOutputs.mNumOuterIter << "\n";
    std::cout << "\nOBJECTIVE: BEST = " << tOutputs.mGlobalBestAugLagFuncValue << ", MEAN = "
            << tOutputs.mMeanBestAugLagFuncValue << ", STDDEV = " << tOutputs.mStdDevBestAugLagFuncValue << "\n";

    std::cout << "\nCONSTRAINT #0: BEST = " << (*tOutputs.mGlobalBestConstraintValues)[0] << ", MEAN = "
            << (*tOutputs.mMeanBestConstraintValues)[0] << ", STDDEV = " << (*tOutputs.mStdDevBestConstraintValues)[0]
            << "\n";

    std::cout << tOutputs.mStopCriterion << "\n";

    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = " << (*tOutputs.mGlobalBestParticles)[tIndex] <<
                ", MEAN = " << (*tOutputs.mMeanBestParticles)[tIndex] << ", STDDEV = "
                << (*tOutputs.mStdDevBestParticles)[tIndex] << "\n";
    }
}

} // ParticleSwarmTest
