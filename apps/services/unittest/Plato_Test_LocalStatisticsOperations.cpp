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

namespace MeanPlusVarianceMeasureTest
{

TEST(PlatoTest, MeanPlusVarianceMeasure)
{
    Plato::InputData tOperations("Operation");
    tOperations.add<std::string>("Function", "MeanPlusStdDev");
    tOperations.add<std::string>("Name", "Objective Statistics");
    tOperations.add<std::string>("Layout", "Scalar");
    tOperations.add<std::string>("Alias", "Objective");

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
    tOutput1.add<std::string>("ArgumentName", "Mean");
    tOperations.add<Plato::InputData>("Output", tOutput1);
    Plato::InputData tOutput2("Output");
    tOutput2.add<std::string>("ArgumentName", "StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput2);
    Plato::InputData tOutput3("Output");
    tOutput3.add<std::string>("ArgumentName", "Mean_Plus_1_StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput3);
    Plato::InputData tOutput4("Output");
    tOutput4.add<std::string>("ArgumentName", "Mean_Plus_5_StdDev");
    tOperations.add<Plato::InputData>("Output", tOutput4);

    // TEST THAT INPUT DATA IS PARSED
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    Plato::MeanPlusVarianceMeasure tOperation(&tPlatoApp, tOperations);

    // TEST DATA
    ASSERT_EQ(Plato::data::SCALAR, tOperation.getDataLayout());
    ASSERT_STREQ("Objective", tOperation.getOutputAlias().c_str());
    ASSERT_STREQ("Objective Statistics", tOperation.getOperationName().c_str());
    ASSERT_STREQ("MeanPlusStdDev", tOperation.getFunctionIdentifier().c_str());

    std::vector<Plato::LocalArg> tLocalArguments;
    tOperation.getArguments(tLocalArguments);
    ASSERT_EQ(7u, tLocalArguments.size());
    std::vector<std::string> tArgumentNames =
        { "sierra_sd1_lc1_objective", "sierra_sd1_lc2_objective", "sierra_sd1_lc3_objective", "Objective_MEAN",
                "Objective_STDDEV", "Objective_MEAN_PLUS_1_STDDEV", "Objective_MEAN_PLUS_5_STDDEV" };
    for(size_t tIndex = 0; tIndex < tArgumentNames.size(); tIndex++)
    {
        bool tFoundGoldValue =
                std::find(tArgumentNames.begin(), tArgumentNames.end(), tLocalArguments[tIndex].mName) != tArgumentNames.end();
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

    ASSERT_STREQ("Objective_MEAN", tOperation.getOutputArgument("MEAN").c_str());
    ASSERT_STREQ("Objective_STDDEV", tOperation.getOutputArgument("STDDEV").c_str());
    ASSERT_STREQ("Objective_MEAN_PLUS_1_STDDEV", tOperation.getOutputArgument("MEAN_PLUS_1_STDDEV").c_str());
    ASSERT_STREQ("Objective_MEAN_PLUS_5_STDDEV", tOperation.getOutputArgument("MEAN_PLUS_5_STDDEV").c_str());
    ASSERT_THROW(tOperation.getOutputArgument("MEAN_PLUS_2_STDDEV"), std::runtime_error);
}

}
// namespace MeanPlusVarianceMeasureTest
