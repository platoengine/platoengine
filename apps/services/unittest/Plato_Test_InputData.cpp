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
 * Plato_Test_InputData.cpp
 *
 *  Created on: Oct 5, 2017
 */

#include <gtest/gtest.h>

#include <map>
#include <vector>
#include <string>
#include <utility>
#include <cassert>

#include "Plato_StageInputDataMng.hpp"
#include "Plato_OperationInputDataMng.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace PlatoStageInputDataTest
{

TEST(PlatoTest, SingleOperationInputDataTest)
{
    Plato::OperationInputDataMng tOperationInputData;

    std::string tPerformerName("PlatoMain");
    std::string tOperationName("FilterControl");
    std::vector<std::string> tInputArgumentName;
    tInputArgumentName.push_back("Field");
    std::vector<std::string> tInputSharedDataName;
    tInputSharedDataName.push_back("Optimization DOFs");
    tOperationInputData.addInputs(tPerformerName, tOperationName, tInputSharedDataName, tInputArgumentName);

    int tOperationIndex = 0;
    EXPECT_STREQ(tOperationName.c_str(), tOperationInputData.getOperationName(tPerformerName).c_str());
    EXPECT_STREQ(tOperationName.c_str(), tOperationInputData.getOperationName(tOperationIndex).c_str());
    EXPECT_STREQ(tPerformerName.c_str(), tOperationInputData.getPerformerName(tOperationIndex).c_str());
    int tNumOperations = 1;
    EXPECT_EQ(tNumOperations, tOperationInputData.getNumOperations());
    int tNumPerformers = 1;
    EXPECT_EQ(tNumPerformers, tOperationInputData.getNumPerformers());

    int tNumInputs = 1;
    EXPECT_EQ(tNumInputs, tOperationInputData.getNumInputs(tPerformerName));
    EXPECT_EQ(tNumInputs, tOperationInputData.getNumInputs(tOperationIndex));
    int tDataIndex = 0;
    EXPECT_STREQ(tInputArgumentName[tDataIndex].c_str(), tOperationInputData.getInputArgument(tPerformerName, tDataIndex).c_str());
    EXPECT_STREQ(tInputArgumentName[tDataIndex].c_str(), tOperationInputData.getInputArgument(tOperationIndex, tDataIndex).c_str());
    EXPECT_STREQ(tInputSharedDataName[tDataIndex].c_str(), tOperationInputData.getInputSharedData(tPerformerName, tDataIndex).c_str());
    EXPECT_STREQ(tInputSharedDataName[tDataIndex].c_str(), tOperationInputData.getInputSharedData(tOperationIndex, tDataIndex).c_str());
    int tNumOutputs = 0;
    EXPECT_EQ(tNumOutputs, tOperationInputData.getNumOutputs(tPerformerName));
    EXPECT_EQ(tNumOutputs, tOperationInputData.getNumOutputs(tOperationIndex));

    std::vector<std::string> tOutputArgumentName;
    tOutputArgumentName.push_back("Filtered Field");
    std::vector<std::string> tOutputSharedDataName;
    tOutputSharedDataName.push_back("Topology");
    tOperationInputData.addOutputs(tPerformerName, tOperationName, tOutputSharedDataName, tOutputArgumentName);

    tNumOutputs = 1;
    EXPECT_EQ(tNumOutputs, tOperationInputData.getNumOutputs(tPerformerName));
    EXPECT_EQ(tNumOutputs, tOperationInputData.getNumOutputs(tOperationIndex));
    EXPECT_STREQ(tOutputArgumentName[tDataIndex].c_str(), tOperationInputData.getOutputArgument(tPerformerName, tDataIndex).c_str());
    EXPECT_STREQ(tOutputArgumentName[tDataIndex].c_str(), tOperationInputData.getOutputArgument(tOperationIndex, tDataIndex).c_str());
    EXPECT_STREQ(tOutputSharedDataName[tDataIndex].c_str(), tOperationInputData.getOutputSharedData(tPerformerName, tDataIndex).c_str());
    EXPECT_STREQ(tOutputSharedDataName[tDataIndex].c_str(), tOperationInputData.getOutputSharedData(tOperationIndex, tDataIndex).c_str());

    EXPECT_FALSE(tOperationInputData.hasSubOperations());
}

TEST(PlatoTest, MultipleOperationsInputDataTest)
{
    Plato::OperationInputDataMng tOperationInputData;

    // STAGE 1 *************** ADD FIRST OUTPUT TO FIRST STAGE ***************
    std::string tPerformerName("LightMP_static_load1");
    std::string tOperationName("Compute Internal Energy");
    std::vector<std::string> tOutputArgumentNames;
    tOutputArgumentNames.push_back("Internal Energy");
    std::vector<std::string> tOutputSharedDataNames;
    tOutputSharedDataNames.push_back("Internal Energy 1");
    tOperationInputData.addOutputs(tPerformerName, tOperationName, tOutputSharedDataNames, tOutputArgumentNames);

    int tNumInputs = 0;
    int tOperationIndex = 0;
    EXPECT_EQ(tNumInputs, tOperationInputData.getNumInputs(tPerformerName));
    EXPECT_EQ(tNumInputs, tOperationInputData.getNumInputs(tOperationIndex));

    EXPECT_STREQ(tOperationName.c_str(), tOperationInputData.getOperationName(tPerformerName).c_str());
    EXPECT_STREQ(tOperationName.c_str(), tOperationInputData.getOperationName(tOperationIndex).c_str());
    EXPECT_STREQ(tPerformerName.c_str(), tOperationInputData.getPerformerName(tOperationIndex).c_str());
    int tNumOperations = 1;
    EXPECT_EQ(tNumOperations, tOperationInputData.getNumOperations());
    int tNumPerformers = 1;
    EXPECT_EQ(tNumPerformers, tOperationInputData.getNumPerformers());

    int tNumOutputs = 1;
    int tOutputDataIndex = 0;
    EXPECT_EQ(tNumOutputs, tOperationInputData.getNumOutputs(tPerformerName));
    EXPECT_EQ(tNumOutputs, tOperationInputData.getNumOutputs(tOperationIndex));
    EXPECT_STREQ(tOutputArgumentNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputArgument(tPerformerName, tOutputDataIndex).c_str());
    EXPECT_STREQ(tOutputArgumentNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputArgument(tOperationIndex, tOutputDataIndex).c_str());
    EXPECT_STREQ(tOutputSharedDataNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputSharedData(tPerformerName, tOutputDataIndex).c_str());
    EXPECT_STREQ(tOutputSharedDataNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputSharedData(tOperationIndex, tOutputDataIndex).c_str());

    // STAGE 1 *************** ADD SECOND OUTPUT TO FIRST STAGE ***************
    std::string tSecondOutputArgumentName("Internal Energy Gradient");
    std::string tSecondOutputSharedDataName("Internal Energy 1 Gradient");
    tOperationInputData.addOutput(tPerformerName, tOperationName, tSecondOutputSharedDataName, tSecondOutputArgumentName);
    tNumOutputs = 2;
    EXPECT_EQ(tNumOutputs, tOperationInputData.getNumOutputs(tPerformerName));
    EXPECT_EQ(tNumOutputs, tOperationInputData.getNumOutputs(tOperationIndex));
    tOutputDataIndex = 1;
    EXPECT_STREQ(tSecondOutputArgumentName.c_str(), tOperationInputData.getOutputArgument(tPerformerName, tOutputDataIndex).c_str());
    EXPECT_STREQ(tSecondOutputArgumentName.c_str(), tOperationInputData.getOutputArgument(tOperationIndex, tOutputDataIndex).c_str());
    EXPECT_STREQ(tSecondOutputSharedDataName.c_str(), tOperationInputData.getOutputSharedData(tPerformerName, tOutputDataIndex).c_str());
    EXPECT_STREQ(tSecondOutputSharedDataName.c_str(), tOperationInputData.getOutputSharedData(tOperationIndex, tOutputDataIndex).c_str());

    // STAGE 1 *************** ADD FIRST AND ONLY INPUT TO FIRST STAGE ***************
    std::string tInputArgumentName("Topology");
    std::string tInputSharedDataName("Topology");
    tOperationInputData.addInput(tPerformerName, tOperationName, tInputSharedDataName, tInputArgumentName);

    tNumInputs = 1;
    EXPECT_EQ(tNumInputs, tOperationInputData.getNumInputs(tPerformerName));
    EXPECT_EQ(tNumInputs, tOperationInputData.getNumInputs(tOperationIndex));

    int tInputDataIndex = 0;
    EXPECT_STREQ(tInputArgumentName.c_str(), tOperationInputData.getInputArgument(tPerformerName, tInputDataIndex).c_str());
    EXPECT_STREQ(tInputArgumentName.c_str(), tOperationInputData.getInputArgument(tOperationIndex, tInputDataIndex).c_str());
    EXPECT_STREQ(tInputSharedDataName.c_str(), tOperationInputData.getInputSharedData(tPerformerName, tInputDataIndex).c_str());
    EXPECT_STREQ(tInputSharedDataName.c_str(), tOperationInputData.getInputSharedData(tOperationIndex, tInputDataIndex).c_str());

    // STAGE 2 *************** ADD OUTPUTS TO SECOND STAGE ***************
    tPerformerName.clear();
    tPerformerName.assign("LightMP_static_load2");
    tOperationName.clear();
    tOperationName.assign("Compute Internal Energy");
    tOutputArgumentNames.clear();
    tOutputArgumentNames.push_back("Internal Energy");
    tOutputArgumentNames.push_back("Internal Energy Gradient");
    tOutputSharedDataNames.clear();
    tOutputSharedDataNames.push_back("Internal Energy 2");
    tOutputSharedDataNames.push_back("Internal Energy 2 Gradient");
    tOperationInputData.addOutputs(tPerformerName, tOperationName, tOutputSharedDataNames, tOutputArgumentNames);

    tNumInputs = 0;
    tOperationIndex = 1;
    EXPECT_EQ(tNumInputs, tOperationInputData.getNumInputs(tPerformerName));
    EXPECT_EQ(tNumInputs, tOperationInputData.getNumInputs(tOperationIndex));
    tNumOutputs = 2;
    EXPECT_EQ(tNumOutputs, tOperationInputData.getNumOutputs(tPerformerName));
    EXPECT_EQ(tNumOutputs, tOperationInputData.getNumOutputs(tOperationIndex));

    EXPECT_STREQ(tOperationName.c_str(), tOperationInputData.getOperationName(tPerformerName).c_str());
    EXPECT_STREQ(tOperationName.c_str(), tOperationInputData.getOperationName(tOperationIndex).c_str());
    EXPECT_STREQ(tPerformerName.c_str(), tOperationInputData.getPerformerName(tOperationIndex).c_str());
    tNumOperations = 2;
    EXPECT_EQ(tNumOperations, tOperationInputData.getNumOperations());
    tNumPerformers = 2;
    EXPECT_EQ(tNumPerformers, tOperationInputData.getNumPerformers());

    tOutputDataIndex = 0;
    EXPECT_STREQ(tOutputArgumentNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputArgument(tPerformerName, tOutputDataIndex).c_str());
    EXPECT_STREQ(tOutputArgumentNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputArgument(tOperationIndex, tOutputDataIndex).c_str());
    EXPECT_STREQ(tOutputSharedDataNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputSharedData(tPerformerName, tOutputDataIndex).c_str());
    EXPECT_STREQ(tOutputSharedDataNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputSharedData(tOperationIndex, tOutputDataIndex).c_str());

    tOutputDataIndex = 1;
    EXPECT_STREQ(tOutputArgumentNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputArgument(tPerformerName, tOutputDataIndex).c_str());
    EXPECT_STREQ(tOutputArgumentNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputArgument(tOperationIndex, tOutputDataIndex).c_str());
    EXPECT_STREQ(tOutputSharedDataNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputSharedData(tPerformerName, tOutputDataIndex).c_str());
    EXPECT_STREQ(tOutputSharedDataNames[tOutputDataIndex].c_str(), tOperationInputData.getOutputSharedData(tOperationIndex, tOutputDataIndex).c_str());

    // STAGE 2 *************** ADD INPUTS TO SECOND STAGE ***************
    tInputArgumentName.clear();
    tInputArgumentName.assign("Topology");
    tInputSharedDataName.clear();
    tInputSharedDataName.assign("Topology");
    tOperationInputData.addInput(tPerformerName, tOperationName, tInputSharedDataName, tInputArgumentName);
    tNumInputs = 1;
    EXPECT_EQ(tNumInputs, tOperationInputData.getNumInputs(tPerformerName));
    EXPECT_EQ(tNumInputs, tOperationInputData.getNumInputs(tOperationIndex));

    tInputDataIndex = 0;
    EXPECT_STREQ(tInputArgumentName.c_str(), tOperationInputData.getInputArgument(tPerformerName, tInputDataIndex).c_str());
    EXPECT_STREQ(tInputArgumentName.c_str(), tOperationInputData.getInputArgument(tOperationIndex, tInputDataIndex).c_str());
    EXPECT_STREQ(tInputSharedDataName.c_str(), tOperationInputData.getInputSharedData(tPerformerName, tInputDataIndex).c_str());
    EXPECT_STREQ(tInputSharedDataName.c_str(), tOperationInputData.getInputSharedData(tOperationIndex, tInputDataIndex).c_str());

    EXPECT_TRUE(tOperationInputData.hasSubOperations());
}

TEST(PlatoTest, StageInputDataTest)
{
    Plato::StageInputDataMng tStageInputData;

    // *********** SET STAGE ONE NAME AND TEST ***********
    int tOrdinalGold = 0;
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumStages());
    std::string tStageName("Design Volume");
    tStageInputData.add(tStageName);
    tOrdinalGold = 1;
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumStages());
    int tStageIndex = 0;
    EXPECT_STREQ(tStageName.c_str(), tStageInputData.getStageName(tStageIndex).c_str());

    // *********** SET OUTPUT LIST FOR A STAGE ONE AND TEST ***********
    tOrdinalGold = 0;
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumOutputs(tStageName));
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumOutputs(tStageIndex));

    std::string tSharedDataName("Design Volume");
    std::vector<std::string> tOutputs;
    tOutputs.push_back(tSharedDataName);
    tStageInputData.addOutputs(tStageName, tOutputs);
    int tOutputIndex = 0;
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getOutputs(tStageName)[tOutputIndex].c_str());
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getOutputs(tStageIndex)[tOutputIndex].c_str());
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getOutput(tStageIndex, tOutputIndex).c_str());
    tOrdinalGold = 1;
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumOutputs(tStageName));
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumOutputs(tStageIndex));
    tOrdinalGold = 0;
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumInputs(tStageName));
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumInputs(tStageIndex));

    // *********** SET STAGE TWO NAME, INPUTS AND OUTPUTS & TEST ***********
    tStageName.clear();
    tStageName.assign("Volume");
    tOutputs.clear();
    EXPECT_TRUE(tOutputs.empty());
    tOutputs.push_back("Volume");
    tOutputs.push_back("Volume Gradient");

    std::vector<std::string> tInputs;
    tInputs.push_back("Optimization DOFs");
    tStageInputData.add(tStageName, tInputs, tOutputs);
    tOrdinalGold = 2;
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumStages());

    tOrdinalGold = 2;
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumOutputs(tStageName));
    tStageIndex = 1;
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumOutputs(tStageIndex));
    tOrdinalGold = 1;
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumInputs(tStageName));
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumInputs(tStageIndex));

    // *********** TEST THAT OUTPUTS FOR THE SECOND STAGE WERE PROPERLY PARSED ***********
    tStageIndex = 1;
    tOutputIndex = 0;
    tSharedDataName.clear();
    tSharedDataName.assign("Volume");
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getOutput(tStageName, tOutputIndex).c_str());
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getOutput(tStageIndex, tOutputIndex).c_str());
    tOutputIndex = 1;
    tSharedDataName.clear();
    tSharedDataName.assign("Volume Gradient");
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getOutput(tStageName, tOutputIndex).c_str());
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getOutput(tStageIndex, tOutputIndex).c_str());

    // *********** TEST THAT INPUTS FOR THE SECOND STAGE WERE PROPERLY PARSED ***********
    int tInputIndex = 0;
    tSharedDataName.clear();
    tSharedDataName.assign("Optimization DOFs");
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getInputs(tStageName)[tInputIndex].c_str());
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getInputs(tStageIndex)[tInputIndex].c_str());
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getInput(tStageName, tInputIndex).c_str());
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getInput(tStageIndex, tInputIndex).c_str());

    // *********** ADD AN INPUT TO VOLUME STAGE (I.E. SECOND STAGE) AND RESET INPUT DATA ***********
    tStageIndex = 1;
    tInputs.push_back("Material Density");
    tStageInputData.addInputs(tStageName, tInputs);
    tOrdinalGold = 2;
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumStages());

    int tNumInputs = 2;
    EXPECT_EQ(tNumInputs, tStageInputData.getNumInputs(tStageName));
    EXPECT_EQ(tNumInputs, tStageInputData.getNumInputs(tStageIndex));

    tInputIndex = 0;
    tSharedDataName.clear();
    tSharedDataName.assign("Optimization DOFs");
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getInput(tStageName, tInputIndex).c_str());
    tInputIndex = 1;
    tSharedDataName.clear();
    tSharedDataName.assign("Material Density");
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getInput(tStageName, tInputIndex).c_str());

    // *********** ADD AN OUTPUT TO VOLUME STAGE (I.E. SECOND STAGE) AND RESET OUTPUT DATA ***********
    tStageIndex = 1;
    tOutputs.push_back("Mass");
    tStageInputData.addOutputs(tStageName, tOutputs);
    tOrdinalGold = 2;
    EXPECT_EQ(tOrdinalGold, tStageInputData.getNumStages());

    int tNumOutputs = 3;
    EXPECT_EQ(tNumOutputs, tStageInputData.getNumOutputs(tStageName));
    EXPECT_EQ(tNumOutputs, tStageInputData.getNumOutputs(tStageIndex));

    tOutputIndex = 0;
    tSharedDataName.clear();
    tSharedDataName.assign("Volume");
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getOutput(tStageName, tOutputIndex).c_str());
    tOutputIndex = 1;
    tSharedDataName.clear();
    tSharedDataName.assign("Volume Gradient");
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getOutput(tStageName, tOutputIndex).c_str());
    tOutputIndex = 2;
    tSharedDataName.clear();
    tSharedDataName.assign("Mass");
    EXPECT_STREQ(tSharedDataName.c_str(), tStageInputData.getOutput(tStageName, tOutputIndex).c_str());

    // *********** ADD AN OPERATION INPUT DATA COLLECTOR TO STAGE MANAGER ***********
    Plato::OperationInputDataMng tOperationInputData;

    std::string tPerformerName("PlatoMain");
    std::string tOperationName("FilterControl");
    std::vector<std::string> tInputArgumentName;
    tInputArgumentName.push_back("Field");
    std::vector<std::string> tInputSharedDataName;
    tInputSharedDataName.push_back("Optimization DOFs");
    tOperationInputData.addInputs(tPerformerName, tOperationName, tInputSharedDataName, tInputArgumentName);

    std::vector<std::string> tOutputArgumentName;
    tOutputArgumentName.push_back("Filtered Field");
    std::vector<std::string> tOutputSharedDataName;
    tOutputSharedDataName.push_back("Topology");
    tOperationInputData.addOutputs(tPerformerName, tOperationName, tOutputSharedDataName, tOutputArgumentName);

    tStageInputData.addOperationInputData(tStageName, tOperationInputData);

    // *********** TEST OPERATION DATA COLLECTOR IN STAGE INPUT DATA MANAGER ***********
    const int tOperationIndex = 0;
    const Plato::OperationInputDataMng & tOperationData = tStageInputData.getOperationInputData(tStageName, tOperationIndex);
    int tNumOperations = 1;
    EXPECT_EQ(tNumOperations, tOperationData.getNumOperations());
    EXPECT_STREQ(tOperationName.c_str(), tOperationData.getOperationName(tPerformerName).c_str());
    EXPECT_STREQ(tOperationName.c_str(), tOperationData.getOperationName(tOperationIndex).c_str());
    EXPECT_STREQ(tPerformerName.c_str(), tOperationData.getPerformerName(tOperationIndex).c_str());

    tNumInputs = 1;
    EXPECT_EQ(tNumInputs, tOperationData.getNumInputs(tPerformerName));
    EXPECT_EQ(tNumInputs, tOperationData.getNumInputs(tOperationIndex));
    int tDataIndex = 0;
    EXPECT_STREQ(tInputArgumentName[tDataIndex].c_str(), tOperationData.getInputArgument(tPerformerName, tDataIndex).c_str());
    EXPECT_STREQ(tInputArgumentName[tDataIndex].c_str(), tOperationData.getInputArgument(tOperationIndex, tDataIndex).c_str());
    EXPECT_STREQ(tInputSharedDataName[tDataIndex].c_str(), tOperationData.getInputSharedData(tPerformerName, tDataIndex).c_str());
    EXPECT_STREQ(tInputSharedDataName[tDataIndex].c_str(), tOperationData.getInputSharedData(tOperationIndex, tDataIndex).c_str());

    tNumOutputs = 1;
    EXPECT_EQ(tNumOutputs, tOperationData.getNumOutputs(tPerformerName));
    EXPECT_EQ(tNumOutputs, tOperationData.getNumOutputs(tOperationIndex));
    EXPECT_STREQ(tOutputArgumentName[tDataIndex].c_str(), tOperationData.getOutputArgument(tPerformerName, tDataIndex).c_str());
    EXPECT_STREQ(tOutputArgumentName[tDataIndex].c_str(), tOperationData.getOutputArgument(tOperationIndex, tDataIndex).c_str());
    EXPECT_STREQ(tOutputSharedDataName[tDataIndex].c_str(), tOperationData.getOutputSharedData(tPerformerName, tDataIndex).c_str());
    EXPECT_STREQ(tOutputSharedDataName[tDataIndex].c_str(), tOperationData.getOutputSharedData(tOperationIndex, tDataIndex).c_str());

    EXPECT_FALSE(tOperationData.hasSubOperations());
}

TEST(PlatoTest, OptimalityCriteriaEngineStageData)
{
    Plato::OptimizerEngineStageData tOptimizerData;

    // **************** TEST: CONTROL NAMES FUNCTIONS ****************
    std::vector<std::string> tStringVector = { "MaterialOne", "MaterialTwo" };
    tOptimizerData.setControlNames(tStringVector);
    size_t tOrdinalValue = tStringVector.size();
    EXPECT_EQ(tOrdinalValue, tOptimizerData.getNumControlVectors());

    std::string tStringValue("MaterialOne");
    tOrdinalValue = 0;
    EXPECT_STREQ(tStringValue.c_str(), tOptimizerData.getControlName(tOrdinalValue).c_str());

    tStringValue.clear();
    tStringValue.assign("MaterialTwo");
    tOrdinalValue = 1;
    EXPECT_STREQ(tStringValue.c_str(), tOptimizerData.getControlName(tOrdinalValue).c_str());

    tStringValue.clear();
    tStringValue.assign("MaterialThree");
    tOptimizerData.addControlName(tStringValue);
    tOrdinalValue = 3;
    EXPECT_EQ(tOrdinalValue, tOptimizerData.getNumControlVectors());
    tOrdinalValue = 2;
    EXPECT_STREQ(tStringValue.c_str(), tOptimizerData.getControlName(tOrdinalValue).c_str());

    // **************** TEST: OUTPUT STAGE NAME FUNCTIONS ****************
    tStringValue.clear();
    tStringValue.assign("Output To File");
    tOptimizerData.setOutputStageName(tStringValue);
    EXPECT_STREQ(tStringValue.c_str(), tOptimizerData.getOutputStageName().c_str());

    // **************** TEST: OBJECTIVE VALUE NAME FUNCTIONS ****************
    tStringValue.clear();
    tStringValue.assign("Internal Energy");
    tOptimizerData.setObjectiveValueOutputName(tStringValue);
    EXPECT_STREQ(tStringValue.c_str(), tOptimizerData.getObjectiveValueOutputName().c_str());

    // **************** TEST: OBJECTIVE VALUE NAME FUNCTIONS ****************
    tStringValue.clear();
    tStringValue.assign("Internal Energy Gradient");
    tOptimizerData.setObjectiveGradientOutputName(tStringValue);
    EXPECT_STREQ(tStringValue.c_str(), tOptimizerData.getObjectiveGradientOutputName().c_str());

    // **************** TEST: CONSTRAINT VALUE NAME FUNCTIONS ****************
    std::vector<std::string> tConstraintNames = {"ConstraintOne", "ConstraintTwo"};
    tOptimizerData.setConstraintValueNames(tConstraintNames);
    for(size_t tIndex = 0; tIndex < tConstraintNames.size(); tIndex++)
    {
        EXPECT_STREQ(tConstraintNames[tIndex].c_str(), tOptimizerData.getConstraintValueName(tIndex).c_str());
    }
    tConstraintNames.push_back("ConstraintThree");
    tOptimizerData.addConstraintValueName("ConstraintThree");
    std::vector<std::string> tMyConstraintNames = tOptimizerData.getConstraintValueNames();
    for(size_t tIndex = 0; tIndex < tConstraintNames.size(); tIndex++)
    {
        EXPECT_STREQ(tConstraintNames[tIndex].c_str(), tOptimizerData.getConstraintValueName(tIndex).c_str());
    }
    tOrdinalValue = tConstraintNames.size();
    EXPECT_EQ(tOrdinalValue, tOptimizerData.getNumConstraints());

    // **************** TEST: CONSTRAINT TARGET VALUES FUNCTIONS ****************
    std::vector<double> tConstraintTargetValues = { 0.1, 0.13, 0.11 };
    tOptimizerData.addConstraintNormalizedTargetValue(tConstraintNames[0], tConstraintTargetValues[0]);
    tOptimizerData.addConstraintNormalizedTargetValue(tConstraintNames[1], tConstraintTargetValues[1]);
    tOptimizerData.addConstraintNormalizedTargetValue(tConstraintNames[2], tConstraintTargetValues[2]);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(tConstraintTargetValues[0], tOptimizerData.getConstraintNormalizedTargetValue(0), tTolerance);
    EXPECT_NEAR(tConstraintTargetValues[1], tOptimizerData.getConstraintNormalizedTargetValue("ConstraintTwo"), tTolerance);
    EXPECT_NEAR(tConstraintTargetValues[2], tOptimizerData.getConstraintNormalizedTargetValue(2), tTolerance);

    // **************** TEST: CONSTRAINT REFERENCE VALUES FUNCTIONS ****************
    std::vector<double> tConstraintReferenceValues = { 0.2, 0.23, 0.21 };
    tOptimizerData.addConstraintReferenceValue(tConstraintNames[0], tConstraintReferenceValues[0]);
    tOptimizerData.addConstraintReferenceValue(tConstraintNames[1], tConstraintReferenceValues[1]);
    tOptimizerData.addConstraintReferenceValue(tConstraintNames[2], tConstraintReferenceValues[2]);

    EXPECT_NEAR(tConstraintReferenceValues[0], tOptimizerData.getConstraintReferenceValue(0), tTolerance);
    EXPECT_NEAR(tConstraintReferenceValues[1], tOptimizerData.getConstraintReferenceValue("ConstraintTwo"), tTolerance);
    EXPECT_NEAR(tConstraintReferenceValues[2], tOptimizerData.getConstraintReferenceValue(2), tTolerance);

    // **************** TEST: CONSTRAINT REFERENCE VALUE NAMES FUNCTIONS ****************
    std::vector<std::string> tConstraintReferenceNames = { "ConstraintRefOne", "ConstraintRefTwo", "ConstraintRefThree" };
    tOptimizerData.addConstraintReferenceValueName(tConstraintNames[0], tConstraintReferenceNames[0]);
    tOptimizerData.addConstraintReferenceValueName(tConstraintNames[1], tConstraintReferenceNames[1]);
    tOptimizerData.addConstraintReferenceValueName(tConstraintNames[2], tConstraintReferenceNames[2]);

    EXPECT_STREQ(tConstraintReferenceNames[0].c_str(), tOptimizerData.getConstraintReferenceValueName(0).c_str());
    EXPECT_STREQ(tConstraintReferenceNames[1].c_str(), tOptimizerData.getConstraintReferenceValueName("ConstraintTwo").c_str());
    EXPECT_STREQ(tConstraintReferenceNames[2].c_str(), tOptimizerData.getConstraintReferenceValueName(2).c_str());

    // **************** TEST: CONSTRAINT GRADIENT NAMES FUNCTIONS ****************
    std::vector<std::string> tConstraintGradNames = { "ConstraintGradOne", "ConstraintGradTwo", "ConstraintGradThree" };
    tOptimizerData.addConstraintGradientName(tConstraintNames[0], tConstraintGradNames[0]);
    tOptimizerData.addConstraintGradientName(tConstraintNames[1], tConstraintGradNames[1]);
    tOptimizerData.addConstraintGradientName(tConstraintNames[2], tConstraintGradNames[2]);

    EXPECT_STREQ(tConstraintGradNames[0].c_str(), tOptimizerData.getConstraintGradientName(0).c_str());
    EXPECT_STREQ(tConstraintGradNames[1].c_str(), tOptimizerData.getConstraintGradientName("ConstraintTwo").c_str());
    EXPECT_STREQ(tConstraintGradNames[2].c_str(), tOptimizerData.getConstraintGradientName(2).c_str());

    // **************** TEST: CONSTRAINT HESSIAN NAMES FUNCTIONS ****************
    std::vector<std::string> tConstraintHessNames = { "ConstraintHessOne", "ConstraintHessTwo", "ConstraintHessThree" };
    tOptimizerData.addConstraintHessianName(tConstraintNames[0], tConstraintHessNames[0]);
    tOptimizerData.addConstraintHessianName(tConstraintNames[1], tConstraintHessNames[1]);
    tOptimizerData.addConstraintHessianName(tConstraintNames[2], tConstraintHessNames[2]);

    EXPECT_STREQ(tConstraintHessNames[0].c_str(), tOptimizerData.getConstraintHessianName(0).c_str());
    EXPECT_STREQ(tConstraintHessNames[1].c_str(), tOptimizerData.getConstraintHessianName("ConstraintTwo").c_str());
    EXPECT_STREQ(tConstraintHessNames[2].c_str(), tOptimizerData.getConstraintHessianName(2).c_str());
}

} // namespace PlatoTest
