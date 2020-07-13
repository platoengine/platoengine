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

TEST(PlatoTest, IsZero)
{
    ASSERT_TRUE(Plato::equal(0.0, 0.0));
    ASSERT_FALSE(Plato::equal(1.0, 0.0));
}

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
    Plato::InputData tOutput5("Output");
    tOutput5.add<std::string>("Statistic", "mean_plus_-1_std_dev");
    tOutput5.add<std::string>("ArgumentName", "objective_Mean_Plus_-1_StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput5);
    Plato::InputData tOutput6("Output");
    tOutput6.add<std::string>("Statistic", "mean_plus_-5_std_dev");
    tOutput6.add<std::string>("ArgumentName", "objective_Mean_Plus_-5_StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput6);

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
    ASSERT_EQ(9u, tLocalArguments.size());
    std::vector<std::string> tArgumentNames = { "sierra_sd1_lc1_objective", "sierra_sd1_lc2_objective", "sierra_sd1_lc3_objective", "objective_Mean", // @suppress("Invalid arguments")
        "objective_Std_Dev", "objective_Mean_Plus_1_StdDev", "objective_Mean_Plus_5_StdDev", "objective_Mean_Plus_-1_StdDev", "objective_Mean_Plus_-5_StdDev" };
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
    ASSERT_EQ(4u, tMultipliers.size());
    ASSERT_NEAR(-5.0, tMultipliers[0], tTolerance);
    ASSERT_NEAR(-1.0, tMultipliers[1], tTolerance);
    ASSERT_NEAR(1.0, tMultipliers[2], tTolerance);
    ASSERT_NEAR(5.0, tMultipliers[3], tTolerance);

    ASSERT_STREQ("objective_Mean", tOperation.getOutputArgument("MEAN").c_str());
    ASSERT_STREQ("objective_Std_Dev", tOperation.getOutputArgument("STD_DEV").c_str());
    ASSERT_STREQ("objective_Mean_Plus_1_StdDev", tOperation.getOutputArgument("MEAN_PLUS_1_STD_DEV").c_str());
    ASSERT_STREQ("objective_Mean_Plus_5_StdDev", tOperation.getOutputArgument("MEAN_PLUS_5_STD_DEV").c_str());
    ASSERT_STREQ("objective_Mean_Plus_-1_StdDev", tOperation.getOutputArgument("MEAN_PLUS_-1_STD_DEV").c_str());
    ASSERT_STREQ("objective_Mean_Plus_-5_StdDev", tOperation.getOutputArgument("MEAN_PLUS_-5_STD_DEV").c_str());
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
    std::vector<std::string> tArgumentNames = // @suppress("Invalid arguments")
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

TEST(PlatoTest, compute_sample_set_mean_global_value)
{
    std::vector<Plato::SampleProbPair<double, double>> tPairs;
    tPairs.push_back(Plato::SampleProbPair<double, double>{});
    tPairs[0].mLength = 1;
    tPairs[0].mProbability = 0.2;
    tPairs[0].mSample = 5;
    tPairs.push_back(Plato::SampleProbPair<double, double>{});
    tPairs[1].mLength = 1;
    tPairs[1].mProbability = 0.1;
    tPairs[1].mSample = 2;
    tPairs.push_back(Plato::SampleProbPair<double, double>{});
    tPairs[2].mLength = 1;
    tPairs[2].mProbability = 0.3;
    tPairs[2].mSample = 3;
    tPairs.push_back(Plato::SampleProbPair<double, double>{});
    tPairs[3].mLength = 1;
    tPairs[3].mProbability = 0.15;
    tPairs[3].mSample = 4;
    tPairs.push_back(Plato::SampleProbPair<double, double>{});
    tPairs[4].mLength = 1;
    tPairs[4].mProbability = 0.25;
    tPairs[4].mSample = 1;

    double tTolerance = 1e-6;
    auto tMean = Plato::compute_sample_set_mean(tPairs);
    ASSERT_NEAR(2.95, tMean, tTolerance);
}

TEST(PlatoTest, compute_sample_set_mean_field_value)
{
    std::vector<Plato::SampleProbPair<double*, double>> tPairs;
    tPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tPairs[0].mLength = 2;
    tPairs[0].mProbability = 0.2;
    std::vector<double> tSample1 = {1, 3};
    tPairs[0].mSample = tSample1.data();
    tPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tPairs[1].mLength = 2;
    tPairs[1].mProbability = 0.1;
    std::vector<double> tSample2 = {2, 3};
    tPairs[1].mSample = tSample2.data();
    tPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tPairs[2].mLength = 2;
    tPairs[2].mProbability = 0.3;
    std::vector<double> tSample3 = {4, 5};
    tPairs[2].mSample = tSample3.data();
    tPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tPairs[3].mLength = 2;
    tPairs[3].mProbability = 0.15;
    std::vector<double> tSample4 = {3, 8};
    tPairs[3].mSample = tSample4.data();
    tPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tPairs[4].mLength = 2;
    tPairs[4].mProbability = 0.25;
    std::vector<double> tSample5 = {8, 11};
    tPairs[4].mSample = tSample5.data();

    double tTolerance = 1e-6;
    std::vector<double> tMean = {8, 11};
    Plato::compute_sample_set_mean(tPairs, tMean.data());
    ASSERT_NEAR(4.05, tMean[0], tTolerance);
    ASSERT_NEAR(6.35, tMean[1], tTolerance);
}

TEST(PlatoTest, compute_sample_set_standard_deviation_global_value_1)
{
    std::vector<Plato::SampleProbPair<double, double>> tPairs;
    Plato::SampleProbPair<double, double> tSample1;
    tSample1.mLength = 1;
    tSample1.mProbability = 0.36112468067266212;
    tSample1.mSample = 6.0821175463902577 ;
    tPairs.push_back(tSample1);
    Plato::SampleProbPair<double, double> tSample2;
    tSample2.mLength = 1;
    tSample2.mProbability = 0.63887286897558715;
    tSample2.mSample = 6.1586066842967053;
    tPairs.push_back(tSample2);

    double tTolerance = 1e-6;
    auto tMean = Plato::compute_sample_set_mean(tPairs);
    EXPECT_NEAR(6.1309694780426414, tMean, tTolerance);
    auto tStdDev = Plato::compute_sample_set_standard_deviation(tMean, tPairs);
    EXPECT_NEAR(0.036739745415510963, tStdDev, tTolerance);
}

TEST(PlatoTest, compute_sample_set_standard_deviation_global_value_2)
{
    std::vector<Plato::SampleProbPair<double, double>> tPairs;
    tPairs.push_back(Plato::SampleProbPair<double, double>{});
    tPairs[0].mLength = 1;
    tPairs[0].mProbability = 0.2;
    tPairs[0].mSample = 5;
    tPairs.push_back(Plato::SampleProbPair<double, double>{});
    tPairs[1].mLength = 1;
    tPairs[1].mProbability = 0.1;
    tPairs[1].mSample = 2;
    tPairs.push_back(Plato::SampleProbPair<double, double>{});
    tPairs[2].mLength = 1;
    tPairs[2].mProbability = 0.3;
    tPairs[2].mSample = 3;
    tPairs.push_back(Plato::SampleProbPair<double, double>{});
    tPairs[3].mLength = 1;
    tPairs[3].mProbability = 0.15;
    tPairs[3].mSample = 4;
    tPairs.push_back(Plato::SampleProbPair<double, double>{});
    tPairs[4].mLength = 1;
    tPairs[4].mProbability = 0.25;
    tPairs[4].mSample = 1;

    double tTolerance = 1e-6;
    auto tMean = Plato::compute_sample_set_mean(tPairs);
    ASSERT_NEAR(2.95, tMean, tTolerance);
    auto tStdDev = Plato::compute_sample_set_standard_deviation(tMean, tPairs);
    ASSERT_NEAR(1.430908802125419, tStdDev, tTolerance);
}

TEST(PlatoTest, compute_sample_set_standard_deviation_field_value)
{
    std::vector<Plato::SampleProbPair<double*, double>> tPairs;
    tPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tPairs[0].mLength = 2;
    tPairs[0].mProbability = 0.2;
    std::vector<double> tSample1 = {1, 3};
    tPairs[0].mSample = tSample1.data();
    tPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tPairs[1].mLength = 2;
    tPairs[1].mProbability = 0.1;
    std::vector<double> tSample2 = {2, 3};
    tPairs[1].mSample = tSample2.data();
    tPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tPairs[2].mLength = 2;
    tPairs[2].mProbability = 0.3;
    std::vector<double> tSample3 = {4, 5};
    tPairs[2].mSample = tSample3.data();
    tPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tPairs[3].mLength = 2;
    tPairs[3].mProbability = 0.15;
    std::vector<double> tSample4 = {3, 8};
    tPairs[3].mSample = tSample4.data();
    tPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tPairs[4].mLength = 2;
    tPairs[4].mProbability = 0.25;
    std::vector<double> tSample5 = {8, 11};
    tPairs[4].mSample = tSample5.data();

    double tTolerance = 1e-6;
    std::vector<double> tMean = {0, 0};
    Plato::compute_sample_set_mean(tPairs, tMean.data());
    ASSERT_NEAR(4.05, tMean[0], tTolerance);
    ASSERT_NEAR(6.35, tMean[1], tTolerance);
    std::vector<double> tStdDev = {0, 0};
    Plato::compute_sample_set_standard_deviation(tMean.data(), tPairs, tStdDev.data());
    ASSERT_NEAR(2.519424537468824, tStdDev[0], tTolerance);
    ASSERT_NEAR(3.118894034750139, tStdDev[1], tTolerance);
}

TEST(PlatoTest, compute_sample_set_mean_plus_std_dev_gradient)
{
    // SET OBJCTIVE PAIRS
    std::vector<Plato::SampleProbPair<double, double>> tFvalPairs;
    tFvalPairs.push_back(Plato::SampleProbPair<double, double>{});
    tFvalPairs[0].mLength = 1;
    tFvalPairs[0].mProbability = 0.2;
    tFvalPairs[0].mSample = 5;
    tFvalPairs.push_back(Plato::SampleProbPair<double, double>{});
    tFvalPairs[1].mLength = 1;
    tFvalPairs[1].mProbability = 0.1;
    tFvalPairs[1].mSample = 2;
    tFvalPairs.push_back(Plato::SampleProbPair<double, double>{});
    tFvalPairs[2].mLength = 1;
    tFvalPairs[2].mProbability = 0.3;
    tFvalPairs[2].mSample = 3;
    tFvalPairs.push_back(Plato::SampleProbPair<double, double>{});
    tFvalPairs[3].mLength = 1;
    tFvalPairs[3].mProbability = 0.15;
    tFvalPairs[3].mSample = 4;
    tFvalPairs.push_back(Plato::SampleProbPair<double, double>{});
    tFvalPairs[4].mLength = 1;
    tFvalPairs[4].mProbability = 0.25;
    tFvalPairs[4].mSample = 1;

    // SET GRADIENT PAIRS
    std::vector<Plato::SampleProbPair<double*, double>> tGradPairs;
    tGradPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tGradPairs[0].mLength = 2;
    tGradPairs[0].mProbability = 0.2;
    std::vector<double> tSample1 = {1, 3};
    tGradPairs[0].mSample = tSample1.data();
    tGradPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tGradPairs[1].mLength = 2;
    tGradPairs[1].mProbability = 0.1;
    std::vector<double> tSample2 = {2, 3};
    tGradPairs[1].mSample = tSample2.data();
    tGradPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tGradPairs[2].mLength = 2;
    tGradPairs[2].mProbability = 0.3;
    std::vector<double> tSample3 = {4, 5};
    tGradPairs[2].mSample = tSample3.data();
    tGradPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tGradPairs[3].mLength = 2;
    tGradPairs[3].mProbability = 0.15;
    std::vector<double> tSample4 = {3, 8};
    tGradPairs[3].mSample = tSample4.data();
    tGradPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tGradPairs[4].mLength = 2;
    tGradPairs[4].mProbability = 0.25;
    std::vector<double> tSample5 = {8, 11};
    tGradPairs[4].mSample = tSample5.data();

    const double tTolerance = 1e-4;
    const double tStdDevMultiplier = 2;
    auto tFvalMean = Plato::compute_sample_set_mean(tFvalPairs);
    auto tFvalStdDev = Plato::compute_sample_set_standard_deviation(tFvalMean, tFvalPairs);
    std::vector<double> tGradient = {0, 0};
    Plato::compute_sample_set_mean_plus_std_dev_gradient(tFvalMean,
                                                         tFvalStdDev,
                                                         tStdDevMultiplier,
                                                         tFvalPairs,
                                                         tGradPairs,
                                                         tGradient.data());
    ASSERT_NEAR(-0.34930203144298277, tGradient[0], tTolerance);
    ASSERT_NEAR(2.041549321072917, tGradient[1], tTolerance);
}

TEST(PlatoTest, compute_sample_set_mean_global_value_error)
{
    // EMPTY INPUT
    std::vector<Plato::SampleProbPair<double, double>> tPairs;
    ASSERT_THROW(Plato::compute_sample_set_mean(tPairs), std::runtime_error);
}

TEST(PlatoTest, compute_sample_set_standard_deviation_global_value_error)
{
    // EMPTY INPUT
    double tMean = 1;
    std::vector<Plato::SampleProbPair<double, double>> tPairs;
    ASSERT_THROW(Plato::compute_sample_set_standard_deviation(tMean, tPairs), std::runtime_error);

    // NON-FINITE MEAN
    tMean = std::numeric_limits<double>::quiet_NaN();
    ASSERT_THROW(Plato::compute_sample_set_standard_deviation(tMean, tPairs), std::runtime_error);
}

TEST(PlatoTest, compute_sample_set_standard_deviation_field_value_error)
{
    // EMPTY INPUT
    std::vector<double> tMean = {1, 2};
    std::vector<double> tStdDev = {0, 0};
    std::vector<Plato::SampleProbPair<double*, double>> tPairs;
    ASSERT_THROW(Plato::compute_sample_set_standard_deviation(tMean.data(), tPairs, tStdDev.data()), std::runtime_error);
}

TEST(PlatoTest, compute_sample_set_mean_plus_std_dev_gradient_error)
{
    // SET INPUT
    double tFvalMean = 1;
    double tFvalStdDev = 1;
    double tStdDevMultiplier = 1;
    std::vector<double> tGradient = {0, 0};
    std::vector<Plato::SampleProbPair<double, double>> tFvalPairs;
    std::vector<Plato::SampleProbPair<double*, double>> tGradPairs;

    // 1. INPUT CONTAINER OF CRITERION VALUE SAMPLE-PROBABILITY PAIRS IS EMPTY
    ASSERT_THROW(Plato::compute_sample_set_mean_plus_std_dev_gradient(tFvalMean,
                                                                      tFvalStdDev,
                                                                      tStdDevMultiplier,
                                                                      tFvalPairs,
                                                                      tGradPairs,
                                                                      tGradient.data()),
                 std::runtime_error);

    // 2. INPUT CONTAINER OF CRITERION GRADIENT SAMPLE-PROBABILITY PAIRS IS EMPTY
    tFvalPairs.push_back(Plato::SampleProbPair<double, double>{});
    tFvalPairs[0].mLength = 1;
    tFvalPairs[0].mProbability = 0.2;
    tFvalPairs[0].mSample = 5;
    ASSERT_THROW(Plato::compute_sample_set_mean_plus_std_dev_gradient(tFvalMean,
                                                                      tFvalStdDev,
                                                                      tStdDevMultiplier,
                                                                      tFvalPairs,
                                                                      tGradPairs,
                                                                      tGradient.data()),
                 std::runtime_error);

    // 3. MEAN IS NOT A FINITE NUMBER
    tGradPairs.push_back(Plato::SampleProbPair<double*, double>{});
    tGradPairs[0].mLength = 2;
    tGradPairs[0].mProbability = 0.2;
    std::vector<double> tSample1 = {1, 3};
    tGradPairs[0].mSample = tSample1.data();
    tFvalMean = std::numeric_limits<double>::quiet_NaN();
    ASSERT_THROW(Plato::compute_sample_set_mean_plus_std_dev_gradient(tFvalMean,
                                                                      tFvalStdDev,
                                                                      tStdDevMultiplier,
                                                                      tFvalPairs,
                                                                      tGradPairs,
                                                                      tGradient.data()),
                 std::runtime_error);

    // 4. STANDARD DEVIATION IS NOT A FINITE NUMBER
    tFvalMean = 1;
    tFvalStdDev = std::numeric_limits<double>::quiet_NaN();
    ASSERT_THROW(Plato::compute_sample_set_mean_plus_std_dev_gradient(tFvalMean,
                                                                      tFvalStdDev,
                                                                      tStdDevMultiplier,
                                                                      tFvalPairs,
                                                                      tGradPairs,
                                                                      tGradient.data()),
                 std::runtime_error);

    // 5. STANDARD DEVIATION MULTIPLIER IS NOT A FINITE NUMBER
    tFvalStdDev = 1;
    tStdDevMultiplier = std::numeric_limits<double>::quiet_NaN();
    ASSERT_THROW(Plato::compute_sample_set_mean_plus_std_dev_gradient(tFvalMean,
                                                                      tFvalStdDev,
                                                                      tStdDevMultiplier,
                                                                      tFvalPairs,
                                                                      tGradPairs,
                                                                      tGradient.data()),
                 std::runtime_error);

    // 6. CRITERION VALUE AND CRITERION GRADIENT CONTAINERS SHOULD HAVE THE SAME SIZE
    tFvalPairs.push_back(Plato::SampleProbPair<double, double>{});
    tFvalPairs[1].mLength = 1;
    tFvalPairs[1].mProbability = 0.1;
    tFvalPairs[1].mSample = 2;
    ASSERT_THROW(Plato::compute_sample_set_mean_plus_std_dev_gradient(tFvalMean,
                                                                      tFvalStdDev,
                                                                      tStdDevMultiplier,
                                                                      tFvalPairs,
                                                                      tGradPairs,
                                                                      tGradient.data()),
                 std::runtime_error);
}

}
// namespace MeanPlusVarianceMeasureTest
