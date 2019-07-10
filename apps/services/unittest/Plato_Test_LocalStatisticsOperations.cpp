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
 * Plato_Test_LocalStatisticsOperations.cpp
 *
 *  Created on: Jun 27, 2019
 */

#include <gtest/gtest.h>

#include "PlatoApp.hpp"
#include "Plato_InputData.hpp"
#include "Plato_MeanPlusVarianceMeasure.hpp"
#include "Plato_MeanPlusVarianceGradient.hpp"

namespace MeanPlusVarianceMeasureTest
{

TEST(PlatoTest, MeanPlusVarianceMeasure)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDev");
    tOperations.add<std::string>("Name", "Stochastic Objective Value");
    tOperations.add<std::string>("Layout", "Scalar");

    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tOperations.add<Plato::InputData>("Input", tInput1);
    Plato::InputData tInput2("Input");
    tInput2.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective");
    tInput2.add<std::string>("Probability", "0.25");
    tOperations.add<Plato::InputData>("Input", tInput2);
    Plato::InputData tInput3("Input");
    tInput3.add<std::string>("ArgumentName", "sierra_sd1_lc3_objective");
    tInput3.add<std::string>("Probability", "0.25");
    tOperations.add<Plato::InputData>("Input", tInput3);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_Mean");
    tOperations.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_Std_Dev");
    tOperations.add<Plato::InputData>("Output", tOutput2);
    Plato::InputData tOutput3("Output");
    tOutput3.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput3.add<std::string>("ArgumentName", "objective_Mean_Plus_1_StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput3);
    Plato::InputData tOutput4("Output");
    tOutput4.add<std::string>("Statistic", "mean_plus_5_std_dev");
    tOutput4.add<std::string>("ArgumentName", "objective_Mean_Plus_5_StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput4);

    // TEST THAT INPUT DATA IS PARSED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    Plato::MeanPlusVarianceMeasure tOperation(&tPlatoApp, tOperations);

    // TEST DATA
    ASSERT_EQ(Plato::data::SCALAR, tOperation.getDataLayout());
    ASSERT_STREQ("Stochastic Objective Value", tOperation.getOperationName().c_str());
    ASSERT_STREQ("MeanPlusStdDev", tOperation.getFunctionIdentifier().c_str());

    std::vector<Plato::LocalArg> tLocalArguments;
    tOperation.getArguments(tLocalArguments);
    ASSERT_EQ(7u, tLocalArguments.size());
    std::vector<std::string> tArgumentNames =
        { "sierra_sd1_lc1_objective", "sierra_sd1_lc2_objective", "sierra_sd1_lc3_objective", "objective_Mean",
                "objective_Std_Dev", "objective_Mean_Plus_1_StdDev", "objective_Mean_Plus_5_StdDev" };
    for(size_t tIndex = 0; tIndex < tArgumentNames.size(); tIndex++)
    {
        bool tFoundGoldValue = std::find(tArgumentNames.begin(), tArgumentNames.end(), tLocalArguments[tIndex].mName)
                != tArgumentNames.end();
        ASSERT_TRUE(tFoundGoldValue);
    }

    double tTolerance = 1e-6;
    ASSERT_NEAR(0.5, tOperation.getProbability("sierra_sd1_lc1_objective"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getProbability("sierra_sd1_lc2_objective"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getProbability("sierra_sd1_lc3_objective"), tTolerance);
    ASSERT_THROW(tOperation.getProbability("sierra_sd1_lc4_objective"), std::runtime_error);

    std::vector<double> tMultipliers = tOperation.getStandardDeviationMultipliers();
    ASSERT_EQ(2u, tMultipliers.size());
    ASSERT_NEAR(1.0, tMultipliers[0], tTolerance);
    ASSERT_NEAR(5.0, tMultipliers[1], tTolerance);

    ASSERT_STREQ("objective_Mean", tOperation.getOutputArgument("MEAN").c_str());
    ASSERT_STREQ("objective_Std_Dev", tOperation.getOutputArgument("STD_DEV").c_str());
    ASSERT_STREQ("objective_Mean_Plus_1_StdDev", tOperation.getOutputArgument("MEAN_PLUS_1_STD_DEV").c_str());
    ASSERT_STREQ("objective_Mean_Plus_5_StdDev", tOperation.getOutputArgument("MEAN_PLUS_5_STD_DEV").c_str());
    ASSERT_THROW(tOperation.getOutputArgument("MEAN_PLUS_2_STDDEV"), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");

    Plato::InputData tCriterionValue("CriterionValue");
    tCriterionValue.add<std::string>("Layout", "Scalar");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tCriterionValue.add<Plato::InputData>("Input", tInput1);
    Plato::InputData tInput2("Input");
    tInput2.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective");
    tInput2.add<std::string>("Probability", "0.25");
    tCriterionValue.add<Plato::InputData>("Input", tInput2);
    Plato::InputData tInput3("Input");
    tInput3.add<std::string>("ArgumentName", "sierra_sd1_lc3_objective");
    tInput3.add<std::string>("Probability", "0.25");
    tCriterionValue.add<Plato::InputData>("Input", tInput3);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriterionValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriterionValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriterionValue", tCriterionValue);

    Plato::InputData tCriterionGradient("CriterionGradient");
    tCriterionGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    tCriterionGradient.add<Plato::InputData>("Input", tInput11);
    Plato::InputData tInput12("Input");
    tInput12.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective_gradient");
    tInput12.add<std::string>("Probability", "0.25");
    tCriterionGradient.add<Plato::InputData>("Input", tInput12);
    Plato::InputData tInput13("Input");
    tInput13.add<std::string>("ArgumentName", "sierra_sd1_lc3_objective_gradient");
    tInput13.add<std::string>("Probability", "0.25");
    tCriterionGradient.add<Plato::InputData>("Input", tInput13);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriterionGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriterionGradient", tCriterionGradient);

    // TEST THAT INPUT DATA IS PARSED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations);

    // TEST DATA
    ASSERT_EQ(Plato::data::SCALAR, tOperation.getCriterionValueDataLayout());
    ASSERT_STREQ("Stochastic Criterion Gradient", tOperation.getOperationName().c_str()); // using default function name
    ASSERT_STREQ("MeanPlusStdDevGradient", tOperation.getFunctionIdentifier().c_str());

    std::vector<Plato::LocalArg> tLocalArguments;
    tOperation.getArguments(tLocalArguments);
    ASSERT_EQ(9u, tLocalArguments.size());
    std::vector<std::string> tArgumentNames =
        { "sierra_sd1_lc1_objective", "sierra_sd1_lc2_objective", "sierra_sd1_lc3_objective", "objective_mean",
                "objective_std_dev", "sierra_sd1_lc1_objective_gradient", "sierra_sd1_lc2_objective_gradient",
                "sierra_sd1_lc3_objective_gradient", "objective_mean_plus_1_std_dev_gradient" };
    for(size_t tIndex = 0; tIndex < tArgumentNames.size(); tIndex++)
    {
        bool tFoundGoldValue = std::find(tArgumentNames.begin(), tArgumentNames.end(), tLocalArguments[tIndex].mName)
                != tArgumentNames.end();
        ASSERT_TRUE(tFoundGoldValue);
    }

    double tTolerance = 1e-6;
    ASSERT_NEAR(0.5, tOperation.getCriterionValueProbability("sierra_sd1_lc1_objective"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getCriterionValueProbability("sierra_sd1_lc2_objective"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getCriterionValueProbability("sierra_sd1_lc3_objective"), tTolerance);
    ASSERT_THROW(tOperation.getCriterionValueProbability("sierra_sd1_lc4_objective"), std::runtime_error);
    ASSERT_THROW(tOperation.getCriterionValueProbability("sierra_sd1_lc1_objective_gradient"), std::runtime_error);
    ASSERT_NEAR(0.5, tOperation.getCriterionGradientProbability("sierra_sd1_lc1_objective_gradient"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getCriterionGradientProbability("sierra_sd1_lc2_objective_gradient"), tTolerance);
    ASSERT_NEAR(0.25, tOperation.getCriterionGradientProbability("sierra_sd1_lc3_objective_gradient"), tTolerance);
    ASSERT_THROW(tOperation.getCriterionGradientProbability("sierra_sd1_lc4_objective_gradient"), std::runtime_error);
    ASSERT_THROW(tOperation.getCriterionGradientProbability("sierra_sd1_lc1_objective"), std::runtime_error);

    double tMultiplier = tOperation.getStandardDeviationMultipliers();
    ASSERT_NEAR(1.0, tMultiplier, tTolerance);

    ASSERT_STREQ("objective_mean", tOperation.getCriterionValueOutputArgument("MEAN").c_str());
    ASSERT_STREQ("objective_std_dev", tOperation.getCriterionValueOutputArgument("STD_DEV").c_str());
    ASSERT_THROW(tOperation.getCriterionValueOutputArgument("MEAN_PLUS_2_STDDEV"), std::runtime_error);
    ASSERT_STREQ("objective_mean_plus_1_std_dev_gradient", tOperation.getCriterionGradientOutputArgument("MEAN_PLUS_1_STD_DEV").c_str());
    ASSERT_THROW(tOperation.getCriterionGradientOutputArgument("MEAN_PLUS_2_STDDEV"), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_checkInputProbabilityValues_Error1)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    Plato::InputData tCriterionValue("CriterionValue");
    tCriterionValue.add<std::string>("Layout", "Scalar");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tCriterionValue.add<Plato::InputData>("Input", tInput1);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriterionValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriterionValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriterionValue", tCriterionValue);

    Plato::InputData tCriterionGradient("CriterionGradient");
    tCriterionGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.45");
    tCriterionGradient.add<Plato::InputData>("Input", tInput11);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriterionGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriterionGradient", tCriterionGradient);

    // TEST ERROR - SAMPLE PROBABILITIES DO NOT MATCH. CRITERIA VALUE AND GRADIENT PROBABILITIES ARE EXPECTED TO BE THE SAME
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_checkInputProbabilityValues_Error2)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    Plato::InputData tCriterionValue("CriterionValue");
    tCriterionValue.add<std::string>("Layout", "Scalar");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tCriterionValue.add<Plato::InputData>("Input", tInput1);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriterionValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriterionValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriterionValue", tCriterionValue);

    Plato::InputData tCriterionGradient("CriterionGradient");
    tCriterionGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    Plato::InputData tInput12("Input");
    tInput12.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective_gradient");
    tInput12.add<std::string>("Probability", "0.25");
    tCriterionGradient.add<Plato::InputData>("Input", tInput12);
    tCriterionGradient.add<Plato::InputData>("Input", tInput11);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriterionGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriterionGradient", tCriterionGradient);

    // TEST ERROR - MISMATCH IN NUMBER OF CRITERIA VALUE AND CRITERIA GRADIENT SAMPLES. THE NUMBER OF SAMPLES SHOULD BE THE SAME
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_parseFunction_Error)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    // TEST ERROR - FUNCTION NAME IS NOT DEFINED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_parseCriterionValueDataLayout_Error)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");

    Plato::InputData tCriterionValue("CriterionValue");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tCriterionValue.add<Plato::InputData>("Input", tInput1);
    Plato::InputData tInput2("Input");
    tInput2.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective");
    tInput2.add<std::string>("Probability", "0.25");
    tCriterionValue.add<Plato::InputData>("Input", tInput2);
    Plato::InputData tInput3("Input");
    tInput3.add<std::string>("ArgumentName", "sierra_sd1_lc3_objective");
    tInput3.add<std::string>("Probability", "0.25");
    tCriterionValue.add<Plato::InputData>("Input", tInput3);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriterionValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriterionValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriterionValue", tCriterionValue);

    Plato::InputData tCriterionGradient("CriterionGradient");
    tCriterionGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    tCriterionGradient.add<Plato::InputData>("Input", tInput11);
    Plato::InputData tInput12("Input");
    tInput12.add<std::string>("ArgumentName", "sierra_sd1_lc2_objective_gradient");
    tInput12.add<std::string>("Probability", "0.25");
    tCriterionGradient.add<Plato::InputData>("Input", tInput12);
    Plato::InputData tInput13("Input");
    tInput13.add<std::string>("ArgumentName", "sierra_sd1_lc3_objective_gradient");
    tInput13.add<std::string>("Probability", "0.25");
    tCriterionGradient.add<Plato::InputData>("Input", tInput13);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriterionGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriterionGradient", tCriterionGradient);

    // TEST ERROR - CRITERION VALUE DATA LAYOUT IS NOT DEFINED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_getMyProbability_Error)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    Plato::InputData tCriterionValue("CriterionValue");
    tCriterionValue.add<std::string>("Layout", "Scalar");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "-0.5");
    tCriterionValue.add<Plato::InputData>("Input", tInput1);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriterionValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriterionValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriterionValue", tCriterionValue);

    Plato::InputData tCriterionGradient("CriterionGradient");
    tCriterionGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    tCriterionGradient.add<Plato::InputData>("Input", tInput11);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriterionGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriterionGradient", tCriterionGradient);

    // TEST ERROR - NEGATIVE PROBABILITY
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_addArgument_Error)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    Plato::InputData tCriterionValue("CriterionValue");
    tCriterionValue.add<std::string>("Layout", "Scalar");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "");
    tInput1.add<std::string>("Probability", "-0.5");
    tCriterionValue.add<Plato::InputData>("Input", tInput1);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriterionValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriterionValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriterionValue", tCriterionValue);

    Plato::InputData tCriterionGradient("CriterionGradient");
    tCriterionGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    tCriterionGradient.add<Plato::InputData>("Input", tInput11);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriterionGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriterionGradient", tCriterionGradient);

    // TEST ERROR - INPUT ARGUMENT NOT DEFINED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_getDataLayout_Error)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    Plato::InputData tCriterionValue("CriterionValue");
    tCriterionValue.add<std::string>("Layout", "");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tCriterionValue.add<Plato::InputData>("Input", tInput1);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriterionValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriterionValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriterionValue", tCriterionValue);

    Plato::InputData tCriterionGradient("CriterionGradient");
    tCriterionGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    tCriterionGradient.add<Plato::InputData>("Input", tInput11);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriterionGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriterionGradient", tCriterionGradient);

    // TEST ERROR - DATA LAYOUT NOT DEFINED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_getOutputArgument_Error)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    Plato::InputData tCriterionValue("CriterionValue");
    tCriterionValue.add<std::string>("Layout", "Scalar");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tCriterionValue.add<Plato::InputData>("Input", tInput1);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "mean");
    tOutput1.add<std::string>("ArgumentName", "");
    tCriterionValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriterionValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriterionValue", tCriterionValue);

    Plato::InputData tCriterionGradient("CriterionGradient");
    tCriterionGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    tCriterionGradient.add<Plato::InputData>("Input", tInput11);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriterionGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriterionGradient", tCriterionGradient);

    // TEST ERROR - OUTPUT ARGUMENT NOT DEFINED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

TEST(PlatoTest, MeanPlusVarianceGradient_getStatisticMeasure_Error)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDevGradient");
    tOperations.add<std::string>("Name", "Stochastic Objective Gradient");

    Plato::InputData tCriterionValue("CriterionValue");
    tCriterionValue.add<std::string>("Layout", "Scalar");
    Plato::InputData tInput1("Input");
    tInput1.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective");
    tInput1.add<std::string>("Probability", "0.5");
    tCriterionValue.add<Plato::InputData>("Input", tInput1);

    Plato::InputData tOutput1("Output");
    tOutput1.add<std::string>("Statistic", "");
    tOutput1.add<std::string>("ArgumentName", "objective_mean");
    tCriterionValue.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("Statistic", "std_dev");
    tOutput2.add<std::string>("ArgumentName", "objective_std_dev");
    tCriterionValue.add<Plato::InputData>("Output", tOutput2);
    tOperations.add<Plato::InputData>("CriterionValue", tCriterionValue);

    Plato::InputData tCriterionGradient("CriterionGradient");
    tCriterionGradient.add<std::string>("Layout", "Nodal Field");
    Plato::InputData tInput11("Input");
    tInput11.add<std::string>("ArgumentName", "sierra_sd1_lc1_objective_gradient");
    tInput11.add<std::string>("Probability", "0.5");
    tCriterionGradient.add<Plato::InputData>("Input", tInput11);

    Plato::InputData tOutput11("Output");
    tOutput11.add<std::string>("Statistic", "mean_plus_1_std_dev");
    tOutput11.add<std::string>("ArgumentName", "objective_mean_plus_1_std_dev_gradient");
    tCriterionGradient.add<Plato::InputData>("Output", tOutput11);
    tOperations.add<Plato::InputData>("CriterionGradient", tCriterionGradient);

    // TEST ERROR - STATISTIC MEASURE NOT DEFINED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    ASSERT_THROW(Plato::MeanPlusVarianceGradient tOperation(&tPlatoApp, tOperations), std::runtime_error);
}

}
// namespace MeanPlusVarianceMeasureTest
