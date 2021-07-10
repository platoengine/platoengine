/*
 * XMLGeneratorParseMetaData_UnitTester.cpp
 *
 *  Created on: Jun 16, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorParseOutput.hpp"
#include "XMLGeneratorParseScenario.hpp"
#include "XMLGeneratorParseServices.hpp"
#include "XMLGeneratorParseCriteria.hpp"
#include "XMLGeneratorParseMaterial.hpp"
#include "XMLGeneratorParseObjective.hpp"
#include "XMLGeneratorParseConstraint.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorFixedBlockUtilities.hpp"


namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, CheckFixedBlocksData_Case1)
{
    // SET PROBLEM
    XMLGen::OptimizationParameters tMetadata;
    std::vector<std::string> tFixedBlockIds = {"1", "2", "3"};
    tMetadata.setFixedBlockIDs(tFixedBlockIds);
    XMLGen::FixedBlock::check_fixed_block_metadata(tMetadata);

    // TEST OUTPUT
    auto tMaterialStates = tMetadata.fixed_block_material_states();
    ASSERT_EQ(3u, tMaterialStates.size());
    for(auto& tState : tMaterialStates)
    {
        ASSERT_STREQ("solid", tState.c_str());
    }

    auto tDomainValues = tMetadata.fixed_block_domain_values();
    ASSERT_EQ(3u, tDomainValues.size());
    for(auto& tValue : tDomainValues)
    {
        ASSERT_STREQ("1.0", tValue.c_str());
    }

    auto tBoundaryValues = tMetadata.fixed_block_boundary_values();
    ASSERT_EQ(3u, tBoundaryValues.size());
    for(auto& tValue : tBoundaryValues)
    {
        ASSERT_STREQ("0.5001", tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, CheckFixedBlocksData_Case2)
{
    // SET PROBLEM
    XMLGen::OptimizationParameters tMetadata;
    std::vector<std::string> tFixedBlockIds = {"1", "2", "3"};
    tMetadata.setFixedBlockIDs(tFixedBlockIds);
    std::vector<std::string> tMaterialStates = {"solid", "fluid", "solid"};
    tMetadata.setFixedBlockMaterialStates(tMaterialStates);

    // CALL FUNCTION
    XMLGen::FixedBlock::check_fixed_block_metadata(tMetadata);
    
    // TEST OUTPUT
    std::vector<std::string> tMaterialStatesGold = {"solid", "fluid", "solid"};
    tMaterialStates = tMetadata.fixed_block_material_states();
    ASSERT_EQ(3u, tMaterialStates.size());
    for(auto& tState : tMaterialStates)
    {
        auto tIndex = &tState - &tMaterialStates[0];
        ASSERT_STREQ(tMaterialStatesGold[tIndex].c_str(), tState.c_str());
    }

    std::vector<std::string> tDomainValuesGold = {"0.0", "1.0", "0.0"};
    auto tDomainValues = tMetadata.fixed_block_domain_values();
    ASSERT_EQ(3u, tDomainValues.size());
    for(auto& tValue : tDomainValues)
    {
        auto tIndex = &tValue - &tDomainValues[0];
        ASSERT_STREQ(tDomainValuesGold[tIndex].c_str(), tValue.c_str());
    }

    std::vector<std::string> tBoundaryValuesGold = {"0.4999", "0.5001", "0.4999"};
    auto tBoundaryValues = tMetadata.fixed_block_boundary_values();
    ASSERT_EQ(3u, tBoundaryValues.size());
    for(auto& tValue : tBoundaryValues)
    {
        auto tIndex = &tValue - &tBoundaryValues[0];
        ASSERT_STREQ(tBoundaryValuesGold[tIndex].c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, CheckFixedBlocksData_Case3)
{
    // SET PROBLEM
    XMLGen::OptimizationParameters tMetadata;
    std::vector<std::string> tFixedBlockIds = {"1", "2", "3"};
    tMetadata.setFixedBlockIDs(tFixedBlockIds);
    std::vector<std::string> tDomainValues = {"0.9", "0.4", "0.8"};
    tMetadata.setFixedBlockDomainValues(tDomainValues);
    std::vector<std::string> tBoundaryValues = {"0.92", "0.43", "0.85"};
    tMetadata.setFixedBlockBoundaryValues(tBoundaryValues);

    // CALL FUNCTION
    XMLGen::FixedBlock::check_fixed_block_metadata(tMetadata);
    
    // TEST OUTPUT
    std::vector<std::string> tMaterialStatesGold = {"solid", "solid", "solid"};
    auto tMaterialStates = tMetadata.fixed_block_material_states();
    ASSERT_EQ(3u, tMaterialStates.size());
    for(auto& tState : tMaterialStates)
    {
        auto tIndex = &tState - &tMaterialStates[0];
        ASSERT_STREQ(tMaterialStatesGold[tIndex].c_str(), tState.c_str());
    }

    std::vector<std::string> tDomainValuesGold = {"0.9", "0.4", "0.8"};
    tDomainValues = tMetadata.fixed_block_domain_values();
    ASSERT_EQ(3u, tDomainValues.size());
    for(auto& tValue : tDomainValues)
    {
        auto tIndex = &tValue - &tDomainValues[0];
        ASSERT_STREQ(tDomainValuesGold[tIndex].c_str(), tValue.c_str());
    }

    std::vector<std::string> tBoundaryValuesGold = {"0.92", "0.43", "0.85"};
    tBoundaryValues = tMetadata.fixed_block_boundary_values();
    ASSERT_EQ(3u, tBoundaryValues.size());
    for(auto& tValue : tBoundaryValues)
    {
        auto tIndex = &tValue - &tBoundaryValues[0];
        ASSERT_STREQ(tBoundaryValuesGold[tIndex].c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, CheckFixedBlocksData_Case4)
{
    // SET PROBLEM
    XMLGen::OptimizationParameters tMetadata;
    std::vector<std::string> tFixedBlockIds = {"1", "2", "3"};
    tMetadata.setFixedBlockIDs(tFixedBlockIds);
    std::vector<std::string> tDomainValues = {"0.9", "0.4", "0.8"};
    tMetadata.setFixedBlockDomainValues(tDomainValues);
    std::vector<std::string> tBoundaryValues = {"0.92", "0.43", "0.85"};
    tMetadata.setFixedBlockBoundaryValues(tBoundaryValues);
    std::vector<std::string> tMaterialStates = {"solid", "fluid", "solid"};
    tMetadata.setFixedBlockMaterialStates(tMaterialStates);

    // CALL FUNCTION
    XMLGen::FixedBlock::check_fixed_block_metadata(tMetadata);
    
    // TEST OUTPUT
    std::vector<std::string> tMaterialStatesGold = {"solid", "fluid", "solid"};
    tMaterialStates = tMetadata.fixed_block_material_states();
    ASSERT_EQ(3u, tMaterialStates.size());
    for(auto& tState : tMaterialStates)
    {
        auto tIndex = &tState - &tMaterialStates[0];
        ASSERT_STREQ(tMaterialStatesGold[tIndex].c_str(), tState.c_str());
    }

    std::vector<std::string> tDomainValuesGold = {"0.9", "0.4", "0.8"};
    tDomainValues = tMetadata.fixed_block_domain_values();
    ASSERT_EQ(3u, tDomainValues.size());
    for(auto& tValue : tDomainValues)
    {
        auto tIndex = &tValue - &tDomainValues[0];
        ASSERT_STREQ(tDomainValuesGold[tIndex].c_str(), tValue.c_str());
    }

    std::vector<std::string> tBoundaryValuesGold = {"0.92", "0.43", "0.85"};
    tBoundaryValues = tMetadata.fixed_block_boundary_values();
    ASSERT_EQ(3u, tBoundaryValues.size());
    for(auto& tValue : tBoundaryValues)
    {
        auto tIndex = &tValue - &tBoundaryValues[0];
        ASSERT_STREQ(tBoundaryValuesGold[tIndex].c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorNoType)
{
    std::string tStringInput =
        "begin objective\n"
        "criteria 1\n"
        "services 1\n"
        "scenarios 1\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorNoCriteria)
{
    std::string tStringInput =
        "begin objective\n"
        "type single_criterion\n"
        "services 1\n"
        "scenarios 1\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorNoServices)
{
    std::string tStringInput =
        "begin objective\n"
        "type single_criterion\n"
        "criteria 1\n"
        "scenarios 1\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorNoScenarios)
{
    std::string tStringInput =
        "begin objective\n"
        "type single_criterion\n"
        "criteria 1\n"
        "services 1\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorMultipleCriteriaInSingleCriterionObjective)
{
    std::string tStringInput =
        "begin objective\n"
        "type single_criterion\n"
        "criteria 1 2\n"
        "services 1 2\n"
        "scenarios 1 2\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorWeightsForSingleCriterion)
{
    std::string tStringInput =
        "begin objective\n"
        "type single_criterion\n"
        "criteria 1 \n"
        "services 1 \n"
        "scenarios 1 \n"
        "weights 1 \n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorNoWeights)
{
    std::string tStringInput =
        "begin objective\n"
        "type weighted_sum\n"
        "criteria 1 2\n"
        "services 1 2\n"
        "scenarios 1 2\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_InvalidObjectiveType)
{
    std::string tStringInput =
        "begin objective\n"
        "type invalid_type\n"
        "criteria 1\n"
        "services 1\n"
        "scenarios 1\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_SingleCriterion)
{
    std::string tStringInput =
        "begin objective\n"
        "type single_criterion\n"
        "criteria 1\n"
        "services 1\n"
        "scenarios 1\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_NO_THROW(tObjectiveParser.parse(tInputSS));

    auto tObjectiveMetaData = tObjectiveParser.data();
    ASSERT_EQ("single_criterion", tObjectiveMetaData.type);
    ASSERT_STREQ("1", tObjectiveMetaData.criteriaIDs[0].c_str());
    ASSERT_STREQ("1", tObjectiveMetaData.serviceIDs[0].c_str());
    ASSERT_STREQ("1", tObjectiveMetaData.scenarioIDs[0].c_str());
}

TEST(PlatoTestXMLGenerator, ParseObjective_WeightedSum)
{
    std::string tStringInput =
        "begin objective\n"
        "type weighted_sum\n"
        "criteria 1 2\n"
        "services 1 2\n"
        "scenarios 1 2\n"
        "weights 1 1\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_NO_THROW(tObjectiveParser.parse(tInputSS));

    auto tObjectiveMetaData = tObjectiveParser.data();
    ASSERT_EQ("weighted_sum", tObjectiveMetaData.type);
    ASSERT_STREQ("1", tObjectiveMetaData.criteriaIDs[0].c_str());
    ASSERT_STREQ("2", tObjectiveMetaData.criteriaIDs[1].c_str());
    ASSERT_STREQ("1", tObjectiveMetaData.serviceIDs[0].c_str());
    ASSERT_STREQ("2", tObjectiveMetaData.serviceIDs[1].c_str());
    ASSERT_STREQ("1", tObjectiveMetaData.scenarioIDs[0].c_str());
    ASSERT_STREQ("2", tObjectiveMetaData.scenarioIDs[1].c_str());
    ASSERT_STREQ("1", tObjectiveMetaData.weights[0].c_str());
    ASSERT_STREQ("1", tObjectiveMetaData.weights[1].c_str());
}

TEST(PlatoTestXMLGenerator, ParseCriteria_ErrorNoCriteriaID)
{
    std::string tStringInput =
        "begin criterion\n"
        "type compliance\n"
        "end criterion\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseCriteria tCriteriaParser;
    ASSERT_THROW(tCriteriaParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseCriteria_ErrorDuplicateIDs)
{
    std::string tStringInput =
        "begin criterion 1\n"
        "type compliance\n"
        "end criterion\n"
        "begin criterion 1\n"
        "type volume\n"
        "end criterion\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseCriteria tCriteriaParser;
    ASSERT_THROW(tCriteriaParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseCriteria_InvalidCriterionType)
{
    std::string tStringInput =
        "begin criterion 1\n"
        "type invalid_type\n"
        "end criterion\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseCriteria tCriteriaParser;
    ASSERT_THROW(tCriteriaParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseCriteria_NoCriterionType)
{
    std::string tStringInput =
        "begin criterion 1\n"
        "end criterion\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseCriteria tCriteriaParser;
    ASSERT_THROW(tCriteriaParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseCriteria_Compliance)
{
    std::string tStringInput =
        "begin criterion 1\n"
        "type mechanical_compliance\n"
        "normalize true\n"
        "normalization_value 10\n"
        "end criterion\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseCriteria tCriteriaParser;
    ASSERT_NO_THROW(tCriteriaParser.parse(tInputSS));

    auto tCriterionMetaData = tCriteriaParser.data();
    ASSERT_EQ(1u, tCriterionMetaData.size());
    ASSERT_STREQ("1", tCriterionMetaData[0].id().c_str());
    ASSERT_STREQ("mechanical_compliance", tCriterionMetaData[0].type().c_str());
    ASSERT_STREQ("true", tCriterionMetaData[0].value("normalize").c_str());
    ASSERT_STREQ("10", tCriterionMetaData[0].value("normalization_value").c_str());

}

TEST(PlatoTestXMLGenerator, ParseCriteria_Volume)
{
    std::string tStringInput =
        "begin criterion 1\n"
        "type volume\n"
        "end criterion\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseCriteria tCriteriaParser;
    tCriteriaParser.parse(tInputSS);

    auto tCriterionMetaData = tCriteriaParser.data();
    ASSERT_EQ(1u, tCriterionMetaData.size());
    ASSERT_STREQ("1", tCriterionMetaData[0].id().c_str());
    ASSERT_STREQ("volume", tCriterionMetaData[0].type().c_str());
    ASSERT_STREQ("false", tCriterionMetaData[0].value("normalize").c_str());
    ASSERT_STREQ("1.0", tCriterionMetaData[0].value("normalization_value").c_str());

}

TEST(PlatoTestXMLGenerator, ParseCriteria_StressPNorm)
{
    std::string tStringInput =
        "begin criterion 1\n"
        "type stress_p-norm\n"
        "stress_p_norm_exponent 3\n"
        "end criterion\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseCriteria tCriteriaParser;
    ASSERT_NO_THROW(tCriteriaParser.parse(tInputSS));

    auto tCriterionMetaData = tCriteriaParser.data();
    ASSERT_EQ(1u, tCriterionMetaData.size());
    ASSERT_STREQ("1", tCriterionMetaData[0].id().c_str());
    ASSERT_STREQ("stress_p-norm", tCriterionMetaData[0].type().c_str());

    ASSERT_STREQ("3", tCriterionMetaData[0].value("stress_p_norm_exponent").c_str());
}

TEST(PlatoTestXMLGenerator, ParseCriteria_ThreeCriteria)
{
    std::string tStringInput =
        "begin criterion 1\n"
        "type stress_p-norm\n"
        "stress_p_norm_exponent 3\n"
        "end criterion\n"
        "begin criterion 2\n"
        "type mechanical_compliance\n"
        "end criterion\n"
        "begin criterion 3\n"
        "type volume\n"
        "end criterion\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseCriteria tCriteriaParser;
    tCriteriaParser.parse(tInputSS);

    auto tCriterionMetaData = tCriteriaParser.data();
    ASSERT_EQ(3u, tCriterionMetaData.size());
    ASSERT_STREQ("1", tCriterionMetaData[0].id().c_str());
    ASSERT_STREQ("2", tCriterionMetaData[1].id().c_str());
    ASSERT_STREQ("3", tCriterionMetaData[2].id().c_str());
    ASSERT_STREQ("stress_p-norm", tCriterionMetaData[0].type().c_str());
    ASSERT_STREQ("mechanical_compliance", tCriterionMetaData[1].type().c_str());
    ASSERT_STREQ("volume", tCriterionMetaData[2].type().c_str());

    ASSERT_STREQ("3", tCriterionMetaData[0].value("stress_p_norm_exponent").c_str());
}

TEST(PlatoTestXMLGenerator, ParseMaterial_ErrorEmptyMaterialBlock)
{
    std::string tStringInput =
        "begin material\n"
        "end material\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseMaterial tMaterialParser;
    ASSERT_THROW(tMaterialParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseMaterial_ErrorInvalidMaterialModel)
{
    std::string tStringInput =
        "begin material\n"
        "   material_model hippo\n"
        "end material\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseMaterial tMaterialParser;
    ASSERT_THROW(tMaterialParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseMaterial_ErrorEmptyMaterialProperties)
{
    std::string tStringInput =
        "begin material\n"
        "   material_model isotropic_linear_elastic\n"
        "end material\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseMaterial tMaterialParser;
    ASSERT_THROW(tMaterialParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseMaterial_NonUniqueIDs)
{
    std::string tStringInput =
        "begin material 1\n"
        "   material_model isotropic_linear_elastic\n"
        "   youngs_modulus 200e9\n"
        "   poissons_ratio 0.33\n"
        "end material\n"
        "begin material 1\n"
        "   material_model isotropic_linear_elastic\n"
        "   youngs_modulus 200e9\n"
        "   poissons_ratio 0.33\n"
        "end material\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseMaterial tMaterialParser;
    ASSERT_THROW(tMaterialParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseMaterial_OneMaterial)
{
    std::string tStringInput =
        "begin material 1\n"
        "   material_model isotropic_linear_elastic\n"
        "   youngs_modulus 200e9\n"
        "   poissons_ratio 0.33\n"
        "end material\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseMaterial tMaterialParser;
    ASSERT_NO_THROW(tMaterialParser.parse(tInputSS));

    auto tMaterialMetaData = tMaterialParser.data();
    ASSERT_EQ(1u, tMaterialMetaData.size());
    ASSERT_STREQ("1", tMaterialMetaData[0].id().c_str());
    auto tTags = tMaterialMetaData[0].tags();
    ASSERT_EQ(2u, tTags.size());
    ASSERT_STREQ("plato_analyze", tMaterialMetaData[0].code().c_str());
    ASSERT_STREQ("0.33", tMaterialMetaData[0].property("poissons_ratio").c_str());
    ASSERT_STREQ("200e9", tMaterialMetaData[0].property("youngs_modulus").c_str());
    ASSERT_STREQ("isotropic_linear_elastic", tMaterialMetaData[0].materialModel().c_str());
}

TEST(PlatoTestXMLGenerator, ParseMaterial_TwoMaterial)
{
    std::string tStringInput =
        "begin material 1\n"
        "   material_model isotropic_linear_elastic\n"
        "   youngs_modulus 200e9\n"
        "   poissons_ratio 0.33\n"
        "end material\n"
        "begin material\n"
        "   id 2\n"
        "   code sierra_sd\n"
        "   material_model isotropic_linear_elastic\n"
        "   youngs_modulus 100e9\n"
        "   poissons_ratio 0.35\n"
        "end material\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseMaterial tMaterialParser;
    ASSERT_NO_THROW(tMaterialParser.parse(tInputSS));

    auto tMaterialMetaData = tMaterialParser.data();
    ASSERT_EQ(2u, tMaterialMetaData.size());

    ASSERT_STREQ("1", tMaterialMetaData[0].id().c_str());
    ASSERT_STREQ("2", tMaterialMetaData[1].id().c_str());

    auto tTags = tMaterialMetaData[0].tags();
    ASSERT_EQ(2u, tTags.size());
    ASSERT_STREQ("plato_analyze", tMaterialMetaData[0].code().c_str());
    ASSERT_STREQ("0.33", tMaterialMetaData[0].property("poissons_ratio").c_str());
    ASSERT_STREQ("200e9", tMaterialMetaData[0].property("youngs_modulus").c_str());
    ASSERT_STREQ("isotropic_linear_elastic", tMaterialMetaData[0].materialModel().c_str());

    tTags = tMaterialMetaData[1].tags();
    ASSERT_EQ(3u, tTags.size());
    ASSERT_STREQ("sierra_sd", tMaterialMetaData[1].code().c_str());
    ASSERT_STREQ("0.35", tMaterialMetaData[1].property("poissons_ratio").c_str());
    ASSERT_STREQ("100e9", tMaterialMetaData[1].property("youngs_modulus").c_str());
    ASSERT_STREQ("3.0", tMaterialMetaData[1].property("penalty_exponent").c_str());
    ASSERT_STREQ("isotropic_linear_elastic", tMaterialMetaData[1].materialModel().c_str());
}

TEST(PlatoTestXMLGenerator, ParseConstraint_ErrorEmptyConstraintMetadata)
{
    std::string tStringInput =
        "begin constraint\n"
        "end constraint\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseConstraint tConstraintParser;
    ASSERT_THROW(tConstraintParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseConstraint_ErrorEmptyCriterion)
{
    std::string tStringInput =
        "begin constraint\n"
        "   service 1\n"
        "   scenario 1\n"
        "   relative_target 0.5\n"
        "end constraint\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseConstraint tConstraintParser;
    ASSERT_THROW(tConstraintParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseConstraint_ErrorEmptyService)
{
    std::string tStringInput =
        "begin constraint\n"
        "   criterion 1\n"
        "   scenario 1\n"
        "   relative_target 0.5\n"
        "end constraint\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseConstraint tConstraintParser;
    ASSERT_THROW(tConstraintParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseConstraint_EmptyTarget)
{
    std::string tStringInput =
        "begin constraint\n"
        "   criterion 1\n"
        "   service 1\n"
        "   scenario 1\n"
        "end constraint\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseConstraint tConstraintParser;
    ASSERT_NO_THROW(tConstraintParser.parse(tInputSS));
    auto tConstraintMetadata = tConstraintParser.data();

    for (auto& tConstraint : tConstraintMetadata)
    {
        ASSERT_STREQ("1", tConstraint.criterion().c_str());
        ASSERT_STREQ("1", tConstraint.service().c_str());
        ASSERT_STREQ("1", tConstraint.scenario().c_str());
        ASSERT_STREQ("", tConstraint.relativeTarget().c_str());
        ASSERT_STREQ("", tConstraint.absoluteTarget().c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseConstraint_Default)
{
    std::string tStringInput =
        "begin constraint\n"
        "   criterion 1\n"
        "   service 1\n"
        "   scenario 1\n"
        "   relative_target 0.5\n"
        "end constraint\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseConstraint tConstraintParser;
    ASSERT_NO_THROW(tConstraintParser.parse(tInputSS));
    auto tConstraintMetadata = tConstraintParser.data();

    for (auto& tConstraint : tConstraintMetadata)
    {
        ASSERT_STREQ("1", tConstraint.criterion().c_str());
        ASSERT_STREQ("1", tConstraint.service().c_str());
        ASSERT_STREQ("1", tConstraint.scenario().c_str());
        ASSERT_STREQ("0.5", tConstraint.relativeTarget().c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorInvalidPhysics)
{
    std::string tStringInput =
        "begin scenario\n"
        "   physics mechanics\n"
        "   dimensions 3\n"
        "   loads 1\n"
        "   boundary_conditions 1\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    ASSERT_THROW(tScenarioParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorInvalidDimensions)
{
    std::string tStringInput =
        "begin scenario\n"
        "   physics steady_state_mechanics\n"
        "   dimensions 1\n"
        "   loads 1\n"
        "   boundary_conditions 1\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    ASSERT_THROW(tScenarioParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorNoPhysics)
{
    std::string tStringInput =
        "begin scenario\n"
        "   dimensions 3\n"
        "   loads 1\n"
        "   boundary_conditions 1\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    ASSERT_THROW(tScenarioParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorNoDimensions)
{
    std::string tStringInput =
        "begin scenario\n"
        "   physics steady_state_mechanics\n"
        "   loads 1\n"
        "   boundary_conditions 1\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    ASSERT_THROW(tScenarioParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorNoLoads)
{
    std::string tStringInput =
        "begin scenario\n"
        "   physics steady_state_mechanics\n"
        "   dimensions 3\n"
        "   boundary_conditions 1\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    ASSERT_NO_THROW(tScenarioParser.parse(tInputSS));
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorNoBoundaryConditions)
{
    std::string tStringInput =
        "begin scenario\n"
        "   physics steady_state_mechanics\n"
        "   dimensions 3\n"
        "   loads 1\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    ASSERT_THROW(tScenarioParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorInvalidScenarioBlockID)
{
    // NOTE: examples for support of scenario ids are:
    // 1. begin scenario 1  GOOD!
    // 2. begin scenario name1_name2_name3  GOOD!
    // 3. begin scenario name1 name2 name3  BAD!
    std::string tStringInput =
        "begin scenario air_force one\n"
        "   physics steady_state_thermal\n"
        "   dimensions 2\n"
        "   material_penalty_exponent 1.0\n"
        "   minimum_ersatz_material_value 1e-6\n"
        "   loads 1\n"
        "   boundary_conditions 1\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    ASSERT_THROW(tScenarioParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseScenario)
{
    // NOTE: examples for support of scenario ids are:
    // 1. begin scenario 1  GOOD!
    // 2. begin scenario name1_name2_name3  GOOD!
    // 3. begin scenario name1 name2 name3  BAD!
    std::string tStringInput =
        "begin scenario air_force_one\n"
        "   physics steady_state_thermal\n"
        "   dimensions 2\n"
        "   material_penalty_exponent 1.0\n"
        "   minimum_ersatz_material_value 1e-6\n"
        "   loads 1 2 3\n"
        "   boundary_conditions 4 5 6\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    tScenarioParser.parse(tInputSS);
    auto tScenarios = tScenarioParser.data();
    for (auto &tScenario : tScenarios)
    {
        ASSERT_STREQ("steady_state_thermal", tScenario.value("physics").c_str());
        ASSERT_STREQ("air_force_one", tScenario.value("id").c_str());
        ASSERT_STREQ("2", tScenario.value("dimensions").c_str());
        ASSERT_STREQ("1.0", tScenario.value("material_penalty_exponent").c_str());
        ASSERT_STREQ("1e-6", tScenario.value("minimum_ersatz_material_value").c_str());
        ASSERT_STREQ("1", tScenario.loadIDs()[0].c_str());
        ASSERT_STREQ("2", tScenario.loadIDs()[1].c_str());
        ASSERT_STREQ("3", tScenario.loadIDs()[2].c_str());
        ASSERT_STREQ("4", tScenario.bcIDs()[0].c_str());
        ASSERT_STREQ("5", tScenario.bcIDs()[1].c_str());
        ASSERT_STREQ("6", tScenario.bcIDs()[2].c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseScenario_WithTimeAndSolverBlocks)
{
    // NOTE: examples for support of scenario ids are:
    // 1. begin scenario 1  GOOD!
    // 2. begin scenario name1_name2_name3  GOOD!
    // 3. begin scenario name1 name2 name3  BAD!
    std::string tStringInput =
        "begin scenario air_force_one\n"
        "   physics steady_state_thermal\n"
        "   dimensions 2\n"
        "   loads 1\n"
        "   boundary_conditions 1\n"
        "   material_penalty_exponent 1.0\n"
        "   minimum_ersatz_material_value 1e-6\n"
        "   begin time\n"
        "     time_step 0.1\n"
        "     number_time_steps 80\n"
        "     max_number_time_steps 160\n"
        "     time_step_expansion_multiplier 1.2\n"
        "     newmark_beta 0.5\n"
        "     newmark_gamma 0.25\n"
        "   end time\n"
        "   begin solver\n"
        "     newton_solver_tolerance 1e-10\n"
        "     max_number_iterations 20\n"
        "     convergence_criterion residual\n"
        "   end solver\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    tScenarioParser.parse(tInputSS);
    auto tScenarios = tScenarioParser.data();
    for (auto &tScenario : tScenarios)
    {
        ASSERT_STREQ("air_force_one", tScenario.value("id").c_str());
        ASSERT_STREQ("steady_state_thermal", tScenario.value("physics").c_str());
        ASSERT_STREQ("2", tScenario.value("dimensions").c_str());
        ASSERT_STREQ("1", tScenario.loadIDs()[0].c_str());
        ASSERT_STREQ("1", tScenario.bcIDs()[0].c_str());
        ASSERT_STREQ("1.0", tScenario.value("material_penalty_exponent").c_str());
        ASSERT_STREQ("1e-6", tScenario.value("minimum_ersatz_material_value").c_str());
        ASSERT_STREQ("80", tScenario.value("number_time_steps").c_str());
        ASSERT_STREQ("160", tScenario.value("max_number_time_steps").c_str());
        ASSERT_STREQ("1.2", tScenario.value("time_step_expansion_multiplier").c_str());
        ASSERT_STREQ("1e-10", tScenario.value("newton_solver_tolerance").c_str());
        ASSERT_STREQ("20", tScenario.value("max_number_iterations").c_str());
        ASSERT_STREQ("residual", tScenario.value("convergence_criterion").c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseScenario_DefaultMainValues)
{
    std::string tStringInput =
        "begin scenario\n"
        "   physics steady_state_mechanics\n"
        "   dimensions 3\n"
        "   loads 1\n"
        "   boundary_conditions 1\n"
        "end scenario\n"
        "begin scenario\n"
        "   physics steady_state_mechanics\n"
        "   dimensions 3\n"
        "   loads 1\n"
        "   boundary_conditions 1\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    tScenarioParser.parse(tInputSS);
    auto tScenarios = tScenarioParser.data();
    int id_counter = 1;
    for (auto& tScenario : tScenarios)
    {
        ASSERT_STREQ(std::string("steady_state_mechanics_" + std::to_string(id_counter++)).c_str(), tScenario.id().c_str());
        ASSERT_STREQ("steady_state_mechanics", tScenario.value("physics").c_str());
        ASSERT_STREQ("3", tScenario.value("dimensions").c_str());
        ASSERT_STREQ("1", tScenario.loadIDs()[0].c_str());
        ASSERT_STREQ("1", tScenario.bcIDs()[0].c_str());

        ASSERT_STREQ("simp", tScenario.value("material_penalty_model").c_str());
        ASSERT_STREQ("3.0", tScenario.value("material_penalty_exponent").c_str());

        ASSERT_STREQ("1.0", tScenario.value("time_step").c_str());
        ASSERT_STREQ("0.25", tScenario.value("newmark_beta").c_str());
        ASSERT_STREQ("0.5", tScenario.value("newmark_gamma").c_str());
        ASSERT_STREQ("40", tScenario.value("number_time_steps").c_str());
        ASSERT_STREQ("160", tScenario.value("max_number_time_steps").c_str());
        ASSERT_STREQ("1.25", tScenario.value("time_step_expansion_multiplier").c_str());

        ASSERT_STREQ("1e-8", tScenario.value("tolerance").c_str());
        ASSERT_STREQ("25", tScenario.value("max_number_iterations").c_str());
        ASSERT_STREQ("residual", tScenario.value("convergence_criterion").c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseService_ErrorInvalidCode)
{
    std::string tStringInput =
        "begin service\n"
        "   code dog\n"
        "end service\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseService tServiceParser;
    ASSERT_THROW(tServiceParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseService_NoCode)
{
    std::string tStringInput =
        "begin service\n"
        "end service\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseService tServiceParser;
    ASSERT_THROW(tServiceParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseService_InvalidID)
{
    std::string tStringInput =
        "begin service invalid id\n"
        "code plato_analyze\n"
        "end service\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseService tServiceParser;
    ASSERT_THROW(tServiceParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseService)
{
    std::string tStringInput =
        "begin service 1\n"
        "code plato_analyze\n"
        "cache_state false\n"
        "update_problem true\n"
        "additive_continuation true\n"
        "number_processors 10\n"
        "number_ranks 1\n"
        "end service\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseService tServiceParser;
    ASSERT_NO_THROW(tServiceParser.parse(tInputSS));
    auto tServices = tServiceParser.data();
    for (auto &tService : tServices)
    {
        ASSERT_STREQ("1", tService.value("id").c_str());
        ASSERT_STREQ("plato_analyze", tService.value("code").c_str());
        ASSERT_STREQ("false", tService.value("cache_state").c_str());
        ASSERT_STREQ("true", tService.value("update_problem").c_str());
        ASSERT_STREQ("true", tService.value("additive_continuation").c_str());
        ASSERT_STREQ("10", tService.value("number_processors").c_str());
        ASSERT_STREQ("1", tService.value("number_ranks").c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseService_Defaults)
{
    std::string tStringInput =
        "begin service\n"
        "code plato_analyze\n"
        "end service\n"
        "begin service\n"
        "code plato_analyze\n"
        "end service\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseService tServiceParser;
    ASSERT_NO_THROW(tServiceParser.parse(tInputSS));
    auto tServices = tServiceParser.data();
    int id_counter = 1;
    for (auto &tService : tServices)
    {
        ASSERT_STREQ(std::string("plato_analyze_" + std::to_string(id_counter++)).c_str(), tService.value("id").c_str());
        ASSERT_STREQ("plato_analyze", tService.value("code").c_str());
        ASSERT_STREQ("false", tService.value("cache_state").c_str());
        ASSERT_STREQ("false", tService.value("update_problem").c_str());
        ASSERT_STREQ("false", tService.value("additive_continuation").c_str());
        ASSERT_STREQ("1", tService.value("number_processors").c_str());
        ASSERT_STREQ("1", tService.value("number_ranks").c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseOutput_EmptyOutputMetadata)
{
    std::string tStringInput =
        "begin output\n"
        "   service 1\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));

    auto tOutputMetadata = tOutputParser.data();
    ASSERT_TRUE(tOutputMetadata[0].isRandomMapEmpty());
    ASSERT_TRUE(tOutputMetadata[0].isDeterministicMapEmpty());
}

TEST(PlatoTestXMLGenerator, ParseOutput_ErrorInvalidQoI)
{
    std::string tStringInput =
        "begin output\n"
        "   service 1\n"
        "   data dispx dispy dispz hippo\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_THROW(tOutputParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseOutput_ErrorInvalidRandomQoI)
{
    std::string tStringInput =
        "begin output\n"
        "   service 1\n"
        "   data dispx dispy dispz\n"
        "   statistics dispx dispy hippo dispz\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_THROW(tOutputParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseOutput_DeterministicOnly)
{
    std::string tStringInput =
        "begin output\n"
        "   service 1\n"
        "   data dispx dispy dispz temperature\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));
    auto tOutputMetadata = tOutputParser.data();
    ASSERT_TRUE(tOutputMetadata[0].isRandomMapEmpty());
    ASSERT_FALSE(tOutputMetadata[0].isDeterministicMapEmpty());

    std::vector<std::string> tGoldIDs = {"dispx", "dispy", "dispz", "temperature"};
    auto tIDs = tOutputMetadata[0].deterministicIDs();
    for(auto& tID : tIDs)
    {
        auto tItr = std::find(tGoldIDs.begin(), tGoldIDs.end(), tID);
        ASSERT_TRUE(tItr != tGoldIDs.end());
        ASSERT_STREQ(tItr->c_str(), tID.c_str());
        ASSERT_STREQ("Nodal Field", tOutputMetadata[0].deterministicLayout(tID).c_str());
        ASSERT_STREQ(tItr->c_str(), tOutputMetadata[0].deterministicArgumentName(tID).c_str());
        ASSERT_STREQ(tItr->c_str(), tOutputMetadata[0].deterministicSharedDataName(tID).c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseOutput_RandomOnly)
{
    std::string tStringInput =
        "begin output\n"
        "   service 1\n"
        "   statistics accumulated_plastic_strain temperature\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));
    auto tOutputMetadata = tOutputParser.data();
    ASSERT_FALSE(tOutputMetadata[0].isRandomMapEmpty());
    ASSERT_TRUE(tOutputMetadata[0].isDeterministicMapEmpty());

    std::vector<std::string> tGoldRandomID =
    {"accumulated_plastic_strain", "temperature"};
    std::vector<std::string> tGoldRandomArgumentName =
    {"accumulated_plastic_strain {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
     "temperature {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    std::vector<std::string> tGoldRandomSharedDataName =
    {"accumulated_plastic_strain {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
     "temperature {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    std::vector<std::string> tGoldRandomLayout = {"Element Field", "Nodal Field"};

    auto tRandomIDs = tOutputMetadata[0].randomIDs();
    for(auto& tID : tRandomIDs)
    {
        auto tItr = std::find(tGoldRandomID.begin(), tGoldRandomID.end(), tID);
        ASSERT_TRUE(tItr != tGoldRandomID.end());
        ASSERT_STREQ(tItr->c_str(), tID.c_str());

        auto tLayout = tOutputMetadata[0].randomLayout(tID);
        tItr = std::find(tGoldRandomLayout.begin(), tGoldRandomLayout.end(), tLayout);
        ASSERT_TRUE(tItr != tGoldRandomLayout.end());
        ASSERT_STREQ(tItr->c_str(), tLayout.c_str());

        auto tArgumentName = tOutputMetadata[0].randomArgumentName(tID);
        tItr = std::find(tGoldRandomArgumentName.begin(), tGoldRandomArgumentName.end(), tArgumentName);
        ASSERT_TRUE(tItr != tGoldRandomArgumentName.end());
        ASSERT_STREQ(tItr->c_str(), tArgumentName.c_str());

        auto tSharedDataName = tOutputMetadata[0].randomSharedDataName(tID);
        tItr = std::find(tGoldRandomSharedDataName.begin(), tGoldRandomSharedDataName.end(), tSharedDataName);
        ASSERT_TRUE(tItr != tGoldRandomSharedDataName.end());
        ASSERT_STREQ(tItr->c_str(), tSharedDataName.c_str());
    }
}

TEST(PlatoTestXMLGenerator, Split)
{
    std::string tInput("1 2 3 4 5 6 7 8");
    std::vector<std::string> tOutput;
    XMLGen::split(tInput, tOutput);
    ASSERT_EQ(8u, tOutput.size());

    std::vector<std::string> tGold = {"1", "2", "3", "4", "5", "6", "7", "8"};
    for(auto& tValue : tOutput)
    {
        auto tIndex = &tValue - &tOutput[0];
        ASSERT_STREQ(tGold[tIndex].c_str(), tValue.c_str());
    }
}

}
// namespace PlatoTestXMLGenerator
