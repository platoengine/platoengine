/*
 * XMLGeneratorParseMetaData_UnitTester.cpp
 *
 *  Created on: Jun 16, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorParseOutput.hpp"
#include "XMLGeneratorParseScenario.hpp"
#include "XMLGeneratorParseMaterial.hpp"
#include "XMLGeneratorParseObjective.hpp"
#include "XMLGeneratorParseConstraint.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace PlatoTestXMLGenerator
{

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
        "   material_model isotropic linear elastic\n"
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
        "   material_model isotropic linear elastic\n"
        "   youngs_modulus 200e9\n"
        "   poissons_ratio 0.33\n"
        "end material\n"
        "begin material 1\n"
        "   material_model isotropic linear elastic\n"
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
        "   material_model isotropic linear elastic\n"
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
    ASSERT_STREQ("isotropic linear elastic", tMaterialMetaData[0].category().c_str());
}

TEST(PlatoTestXMLGenerator, ParseMaterial_TwoMaterial)
{
    std::string tStringInput =
        "begin material 1\n"
        "   material_model isotropic linear elastic\n"
        "   youngs_modulus 200e9\n"
        "   poissons_ratio 0.33\n"
        "end material\n"
        "begin material\n"
        "   id 2\n"
        "   code sierra_sd\n"
        "   material_model isotropic linear elastic\n"
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
    ASSERT_STREQ("isotropic linear elastic", tMaterialMetaData[0].category().c_str());

    tTags = tMaterialMetaData[1].tags();
    ASSERT_EQ(3u, tTags.size());
    ASSERT_STREQ("sierra_sd", tMaterialMetaData[1].code().c_str());
    ASSERT_STREQ("0.35", tMaterialMetaData[1].property("poissons_ratio").c_str());
    ASSERT_STREQ("100e9", tMaterialMetaData[1].property("youngs_modulus").c_str());
    ASSERT_STREQ("3.0", tMaterialMetaData[1].property("penalty_exponent").c_str());
    ASSERT_STREQ("isotropic linear elastic", tMaterialMetaData[1].category().c_str());
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

TEST(PlatoTestXMLGenerator, ParseConstraint_ErrorInvalidConstraintType)
{
    std::string tStringInput =
        "begin constraint\n"
        "   type hippo\n"
        "end constraint\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseConstraint tConstraintParser;
    ASSERT_THROW(tConstraintParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseConstraint_ErrorEmptyTarget)
{
    std::string tStringInput =
        "begin constraint\n"
        "   type volume\n"
        "end constraint\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseConstraint tConstraintParser;
    ASSERT_THROW(tConstraintParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseConstraint_Default)
{
    std::string tStringInput =
        "begin constraint\n"
        "   code plato_analyze\n"
        "   type volume\n"
        "   volume fraction 0.2\n"
        "end constraint\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseConstraint tConstraintParser;
    ASSERT_NO_THROW(tConstraintParser.parse(tInputSS));
    auto tConstraintMetadata = tConstraintParser.data();

    for (auto& tConstraint : tConstraintMetadata)
    {
        ASSERT_TRUE(tConstraint.absoluteTarget().empty());
        ASSERT_STREQ("0.2", tConstraint.normalizedTarget().c_str());
        ASSERT_STREQ("volume", tConstraint.category().c_str());
        ASSERT_STREQ("plato_analyze", tConstraint.code().c_str());
        ASSERT_STREQ("3.0", tConstraint.materialPenaltyExponent().c_str());
        ASSERT_STREQ("1e-9", tConstraint.minErsatzMaterialConstant().c_str());
        ASSERT_STREQ("1", tConstraint.name().c_str());
        ASSERT_STREQ("plato_analyze", tConstraint.performer().c_str());
        ASSERT_STREQ("6.0", tConstraint.pnormExponent().c_str());
        ASSERT_STREQ("1.0", tConstraint.weight().c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseConstraint_OneConstraint)
{
    std::string tStringInput =
        "begin constraint 1\n"
        "   type volume\n"
        "   name dog\n"
        "   weight 2.0\n"
        "   performer sierra_sd_0\n"
        "   volume absolute 40.0\n"
        "   code sierra_sd\n"
        "   penalty power 1.0\n"
        "   minimum ersatz material value 1e-6\n"
        "end constraint\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseConstraint tConstraintParser;
    tConstraintParser.parse(tInputSS);
    auto tConstraintMetadata = tConstraintParser.data();

    for (auto& tConstraint : tConstraintMetadata)
    {
        ASSERT_TRUE(tConstraint.normalizedTarget().empty());
        ASSERT_STREQ("40.0", tConstraint.absoluteTarget().c_str());
        ASSERT_STREQ("volume", tConstraint.category().c_str());
        ASSERT_STREQ("sierra_sd", tConstraint.code().c_str());
        ASSERT_STREQ("1.0", tConstraint.materialPenaltyExponent().c_str());
        ASSERT_STREQ("1e-6", tConstraint.minErsatzMaterialConstant().c_str());
        ASSERT_STREQ("1", tConstraint.id().c_str());
        ASSERT_STREQ("dog", tConstraint.name().c_str());
        ASSERT_STREQ("sierra_sd_0", tConstraint.performer().c_str());
        ASSERT_STREQ("6.0", tConstraint.pnormExponent().c_str());
        ASSERT_STREQ("2.0", tConstraint.weight().c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseConstraint_TwoConstraint)
{
    std::string tStringInput =
        "begin constraint\n"
        "   code platomain\n"
        "   type volume\n"
        "   volume absolute 10.0\n"
        "end constraint\n"
        "begin constraint\n"
        "   code plato_analyze\n"
        "   type stress p-norm\n"
        "   target normalized 0.4\n"
        "   pnorm exponent 30.0\n"
        "end constraint\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseConstraint tConstraintParser;
    ASSERT_NO_THROW(tConstraintParser.parse(tInputSS));
    auto tConstraintMetadata = tConstraintParser.data();

    // Constraint One
    ASSERT_TRUE(tConstraintMetadata[0].normalizedTarget().empty());
    ASSERT_STREQ("1", tConstraintMetadata[0].name().c_str());
    ASSERT_STREQ("1.0", tConstraintMetadata[0].weight().c_str());
    ASSERT_STREQ("volume", tConstraintMetadata[0].category().c_str());
    ASSERT_STREQ("6.0", tConstraintMetadata[0].pnormExponent().c_str());
    ASSERT_STREQ("platomain", tConstraintMetadata[0].code().c_str());
    ASSERT_STREQ("10.0", tConstraintMetadata[0].absoluteTarget().c_str());
    ASSERT_STREQ("platomain", tConstraintMetadata[0].performer().c_str());
    ASSERT_STREQ("3.0", tConstraintMetadata[0].materialPenaltyExponent().c_str());
    ASSERT_STREQ("1e-9", tConstraintMetadata[0].minErsatzMaterialConstant().c_str());

    // Constraint Two
    ASSERT_TRUE(tConstraintMetadata[1].absoluteTarget().empty());
    ASSERT_STREQ("1", tConstraintMetadata[1].name().c_str());
    ASSERT_STREQ("1.0", tConstraintMetadata[1].weight().c_str());
    ASSERT_STREQ("stress p-norm", tConstraintMetadata[1].category().c_str());
    ASSERT_STREQ("30.0", tConstraintMetadata[1].pnormExponent().c_str());
    ASSERT_STREQ("plato_analyze", tConstraintMetadata[1].code().c_str());
    ASSERT_STREQ("0.4", tConstraintMetadata[1].normalizedTarget().c_str());
    ASSERT_STREQ("plato_analyze", tConstraintMetadata[1].performer().c_str());
    ASSERT_STREQ("3.0", tConstraintMetadata[1].materialPenaltyExponent().c_str());
    ASSERT_STREQ("1e-9", tConstraintMetadata[1].minErsatzMaterialConstant().c_str());
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorEmptyPhysicsMetadata)
{
    std::string tStringInput =
        "begin scenario\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    ASSERT_THROW(tScenarioParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorInvalidPhysics)
{
    std::string tStringInput =
        "begin scenario\n"
        "   physics mechanics\n"
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
        "   physics mechanical\n"
        "   code sierra_sd\n"
        "   dimensions 1\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    ASSERT_THROW(tScenarioParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorInvalidCode)
{
    std::string tStringInput =
        "begin scenario\n"
        "   physics mechanical\n"
        "   dimensions 3\n"
        "   code dog\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    ASSERT_THROW(tScenarioParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseScenario_DefaultMainValues)
{
    std::string tStringInput =
        "begin scenario\n"
        "   physics mechanical\n"
        "   dimensions 3\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    tScenarioParser.parse(tInputSS);
    auto tScenarios = tScenarioParser.data();
    for (auto& tScenario : tScenarios)
    {
        ASSERT_STREQ("false", tScenario.value("use_new_analyze_uq_workflow").c_str());
        ASSERT_STREQ("plato_analyze", tScenario.value("code").c_str());
        ASSERT_STREQ("mechanical", tScenario.value("physics").c_str());
        ASSERT_STREQ("plato_analyze_1", tScenario.value("performer").c_str());
        ASSERT_STREQ("plato_analyze_mechanical_1", tScenario.value("id").c_str());
        ASSERT_STREQ("3", tScenario.value("dimensions").c_str());
        ASSERT_STREQ("3.0", tScenario.value("material_penalty_exponent").c_str());
        ASSERT_STREQ("1e-9", tScenario.value("minimum_ersatz_material_value").c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorInvalidScenarioBlockID)
{
    // NOTE: examples for support of scenario ids are:
    // 1. begin scenario 1  GOOD!
    // 2. begin scenario name1_name2_name3  GOOD!
    // 3. begin scenario name1 name2 name3  BAD!
    std::string tStringInput =
        "begin scenario air_force one\n"
        "   physics thermal\n"
        "   use_new_analyze_uq_workflow true\n"
        "   dimensions 2\n"
        "   code sierra_sd\n"
        "   performer sierra_sd_0\n"
        "   material_penalty_exponent 1.0\n"
        "   minimum_ersatz_material_value 1e-6\n"
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
        "   physics thermal\n"
        "   use_new_analyze_uq_workflow true\n"
        "   dimensions 2\n"
        "   code sierra_sd\n"
        "   performer sierra_sd_0\n"
        "   material_penalty_exponent 1.0\n"
        "   minimum_ersatz_material_value 1e-6\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    tScenarioParser.parse(tInputSS);
    auto tScenarios = tScenarioParser.data();
    for (auto &tScenario : tScenarios)
    {
        ASSERT_STREQ("true", tScenario.value("use_new_analyze_uq_workflow").c_str());
        ASSERT_STREQ("sierra_sd", tScenario.value("code").c_str());
        ASSERT_STREQ("thermal", tScenario.value("physics").c_str());
        ASSERT_STREQ("sierra_sd_0", tScenario.value("performer").c_str());
        ASSERT_STREQ("air_force_one", tScenario.value("id").c_str());
        ASSERT_STREQ("2", tScenario.value("dimensions").c_str());
        ASSERT_STREQ("1.0", tScenario.value("material_penalty_exponent").c_str());
        ASSERT_STREQ("1e-6", tScenario.value("minimum_ersatz_material_value").c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseScenarioWithTimeAndSolverBlocks)
{
    std::string tStringInput =
        "begin scenario 1\n"
        "   physics plasticity\n"
        "   dimensions 3\n"
        "   code plato_analyze\n"
        "   begin time\n"
        "     number_time_steps 80\n"
        "     max_number_time_steps 160\n"
        "     time_step_expansion_multiplier 1.2\n"
        "   end time\n"
        "   begin solver\n"
        "     tolerance 1e-10\n"
        "     max_number_iterations 20\n"
        "   end solver\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    tScenarioParser.parse(tInputSS);
    auto tScenarios = tScenarioParser.data();
    for (auto &tScenario : tScenarios)
    {
        ASSERT_STREQ("false", tScenario.value("use_new_analyze_uq_workflow").c_str());
        ASSERT_STREQ("plato_analyze", tScenario.value("code").c_str());
        ASSERT_STREQ("plasticity", tScenario.value("physics").c_str());
        ASSERT_STREQ("plato_analyze_1", tScenario.value("performer").c_str());
        ASSERT_STREQ("1", tScenario.value("id").c_str());
        ASSERT_STREQ("3", tScenario.value("dimensions").c_str());
        ASSERT_STREQ("3.0", tScenario.value("material_penalty_exponent").c_str());
        ASSERT_STREQ("1e-9", tScenario.value("minimum_ersatz_material_value").c_str());
        ASSERT_STREQ("80", tScenario.value("number_time_steps").c_str());
        ASSERT_STREQ("160", tScenario.value("max_number_time_steps").c_str());
        ASSERT_STREQ("1.2", tScenario.value("time_step_expansion_multiplier").c_str());
        ASSERT_STREQ("1e-10", tScenario.value("tolerance").c_str());
        ASSERT_STREQ("20", tScenario.value("max_number_iterations").c_str());
        ASSERT_STREQ("residual", tScenario.value("convergence_criterion").c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseOutput_ErrorScenarionIdIsNotDefined)
{
    std::string tStringInput =
        "begin output\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_THROW(tOutputParser.parse(tInputSS), std::runtime_error);

    auto tOutputMetadata = tOutputParser.data();
    ASSERT_TRUE(tOutputMetadata.isRandomMapEmpty());
    ASSERT_TRUE(tOutputMetadata.isDeterministicMapEmpty());
}

TEST(PlatoTestXMLGenerator, ParseOutput_EmptyOutputMetadata)
{
    std::string tStringInput =
        "begin output\n"
        "   scenario 1\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));

    auto tOutputMetadata = tOutputParser.data();
    ASSERT_TRUE(tOutputMetadata.isRandomMapEmpty());
    ASSERT_TRUE(tOutputMetadata.isDeterministicMapEmpty());
}

TEST(PlatoTestXMLGenerator, ParseOutput_ErrorInvalidQoI)
{
    std::string tStringInput =
        "begin output\n"
        "   scenario 1\n"
        "   quantities_of_interest dispx dispy dispz hippo\n"
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
        "   scenario 1\n"
        "   quantities_of_interest dispx dispy dispz\n"
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
        "   scenario 1\n"
        "   quantities_of_interest dispx dispy dispz temperature\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));
    auto tOutputMetadata = tOutputParser.data();
    ASSERT_TRUE(tOutputMetadata.isRandomMapEmpty());
    ASSERT_FALSE(tOutputMetadata.isDeterministicMapEmpty());

    std::vector<std::string> tGoldIDs = {"dispx", "dispy", "dispz", "temperature"};
    auto tIDs = tOutputMetadata.deterministicIDs();
    for(auto& tID : tIDs)
    {
        auto tItr = std::find(tGoldIDs.begin(), tGoldIDs.end(), tID);
        ASSERT_TRUE(tItr != tGoldIDs.end());
        ASSERT_STREQ(tItr->c_str(), tID.c_str());
        ASSERT_STREQ("Nodal Field", tOutputMetadata.deterministicLayout(tID).c_str());
        ASSERT_STREQ(tItr->c_str(), tOutputMetadata.deterministicArgumentName(tID).c_str());
        ASSERT_STREQ(tItr->c_str(), tOutputMetadata.deterministicSharedDataName(tID).c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseOutput_RandomOnly)
{
    std::string tStringInput =
        "begin output\n"
        "   scenario 1\n"
        "   statistics accumulated_plastic_strain temperature\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));
    auto tOutputMetadata = tOutputParser.data();
    ASSERT_FALSE(tOutputMetadata.isRandomMapEmpty());
    ASSERT_TRUE(tOutputMetadata.isDeterministicMapEmpty());

    std::vector<std::string> tGoldRandomID =
    {"accumulated_plastic_strain", "temperature"};
    std::vector<std::string> tGoldRandomArgumentName =
    {"accumulated_plastic_strain {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
     "temperature {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    std::vector<std::string> tGoldRandomSharedDataName =
    {"accumulated_plastic_strain {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
     "temperature {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    std::vector<std::string> tGoldRandomLayout = {"Element Field", "Nodal Field"};

    auto tRandomIDs = tOutputMetadata.randomIDs();
    for(auto& tID : tRandomIDs)
    {
        auto tItr = std::find(tGoldRandomID.begin(), tGoldRandomID.end(), tID);
        ASSERT_TRUE(tItr != tGoldRandomID.end());
        ASSERT_STREQ(tItr->c_str(), tID.c_str());

        auto tLayout = tOutputMetadata.randomLayout(tID);
        tItr = std::find(tGoldRandomLayout.begin(), tGoldRandomLayout.end(), tLayout);
        ASSERT_TRUE(tItr != tGoldRandomLayout.end());
        ASSERT_STREQ(tItr->c_str(), tLayout.c_str());

        auto tArgumentName = tOutputMetadata.randomArgumentName(tID);
        tItr = std::find(tGoldRandomArgumentName.begin(), tGoldRandomArgumentName.end(), tArgumentName);
        ASSERT_TRUE(tItr != tGoldRandomArgumentName.end());
        ASSERT_STREQ(tItr->c_str(), tArgumentName.c_str());

        auto tSharedDataName = tOutputMetadata.randomSharedDataName(tID);
        tItr = std::find(tGoldRandomSharedDataName.begin(), tGoldRandomSharedDataName.end(), tSharedDataName);
        ASSERT_TRUE(tItr != tGoldRandomSharedDataName.end());
        ASSERT_STREQ(tItr->c_str(), tSharedDataName.c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseOutput_DeterministicPlusRandom)
{
    std::string tStringInput =
        "begin output\n"
        "   scenario 1\n"
        "   quantities_of_interest dispx dispy dispz temperature\n"
        "   statistics accumulated_plastic_strain temperature\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));
    auto tOutputMetadata = tOutputParser.data();
    ASSERT_FALSE(tOutputMetadata.isRandomMapEmpty());
    ASSERT_FALSE(tOutputMetadata.isDeterministicMapEmpty());

    std::vector<std::string> tGoldRandomID =
    {"accumulated_plastic_strain", "temperature"};
    std::vector<std::string> tGoldRandomArgumentName =
    {"accumulated_plastic_strain {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
     "temperature {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    std::vector<std::string> tGoldRandomSharedDataName =
    {"accumulated_plastic_strain {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
     "temperature {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    std::vector<std::string> tGoldRandomLayout = {"Element Field", "Nodal Field"};

    auto tRandomIDs = tOutputMetadata.randomIDs();
    for(auto& tID : tRandomIDs)
    {
        auto tItr = std::find(tGoldRandomID.begin(), tGoldRandomID.end(), tID);
        ASSERT_TRUE(tItr != tGoldRandomID.end());
        ASSERT_STREQ(tItr->c_str(), tID.c_str());

        auto tLayout = tOutputMetadata.randomLayout(tID);
        tItr = std::find(tGoldRandomLayout.begin(), tGoldRandomLayout.end(), tLayout);
        ASSERT_TRUE(tItr != tGoldRandomLayout.end());
        ASSERT_STREQ(tItr->c_str(), tLayout.c_str());

        auto tArgumentName = tOutputMetadata.randomArgumentName(tID);
        tItr = std::find(tGoldRandomArgumentName.begin(), tGoldRandomArgumentName.end(), tArgumentName);
        ASSERT_TRUE(tItr != tGoldRandomArgumentName.end());
        ASSERT_STREQ(tItr->c_str(), tArgumentName.c_str());

        auto tSharedDataName = tOutputMetadata.randomSharedDataName(tID);
        tItr = std::find(tGoldRandomSharedDataName.begin(), tGoldRandomSharedDataName.end(), tSharedDataName);
        ASSERT_TRUE(tItr != tGoldRandomSharedDataName.end());
        ASSERT_STREQ(tItr->c_str(), tSharedDataName.c_str());
    }

    std::vector<std::string> tGoldDeterministicIDs = {"dispx", "dispy", "dispz", "temperature"};
    auto tIDs = tOutputMetadata.deterministicIDs();
    for(auto& tID : tIDs)
    {
        auto tItr = std::find(tGoldDeterministicIDs.begin(), tGoldDeterministicIDs.end(), tID);
        ASSERT_TRUE(tItr != tGoldDeterministicIDs.end());
        ASSERT_STREQ(tItr->c_str(), tID.c_str());
        ASSERT_STREQ("Nodal Field", tOutputMetadata.deterministicLayout(tID).c_str());
        ASSERT_STREQ(tItr->c_str(), tOutputMetadata.deterministicArgumentName(tID).c_str());
        ASSERT_STREQ(tItr->c_str(), tOutputMetadata.deterministicSharedDataName(tID).c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorInvalidCriterion)
{
    std::string tStringInput =
        "begin objective\n"
        "   type hippo\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorDimMistmachNaturalBCArrays)
{
    std::string tStringInput =
        "begin objective\n"
        "   type stress p-norm\n"
        "   load ids 10\n"
        "   load case weights 1 2\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorInvalidCode)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code dog\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorInvalidOutputKey)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz bats\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorEmptyNaturalBCIDs)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorEmptyEssentialBCIDs)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorNumProcessorIsNotNumber)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "   distribute objective at most P3 processors\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_OneObjective)
{
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   performer ferrari\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    tObjectiveParser.parse(tInputSS);
    auto tObjectiveMetadata = tObjectiveParser.data();
    ASSERT_EQ(1u, tObjectiveMetadata.size());

    ASSERT_STREQ("maximize stiffness", tObjectiveMetadata[0].type.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_procs.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_ranks.c_str());
    ASSERT_STREQ("1.0", tObjectiveMetadata[0].weight.c_str());
    ASSERT_STREQ("3.0", tObjectiveMetadata[0].mPenaltyParam.c_str());
    ASSERT_STREQ("6.0", tObjectiveMetadata[0].mPnormExponent.c_str());
    ASSERT_STREQ("1e-9", tObjectiveMetadata[0].mMinimumErsatzValue.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[0].code_name.c_str());
    ASSERT_STREQ("ferrari", tObjectiveMetadata[0].mPerformerName.c_str());
    ASSERT_STREQ("true", tObjectiveMetadata[0].normalize_objective.c_str());
    ASSERT_STREQ("1e-7", tObjectiveMetadata[0].analysis_solver_tolerance.c_str());
    ASSERT_STREQ("false", tObjectiveMetadata[0].multi_load_case.c_str());
    ASSERT_STREQ("none", tObjectiveMetadata[0].distribute_objective_type.c_str());

    ASSERT_EQ(1u, tObjectiveMetadata[0].bc_ids.size());
    ASSERT_STREQ("11", tObjectiveMetadata[0].bc_ids[0].c_str());
    ASSERT_EQ(1u, tObjectiveMetadata[0].load_case_ids.size());
    ASSERT_STREQ("10", tObjectiveMetadata[0].load_case_ids[0].c_str());

    ASSERT_EQ(3u, tObjectiveMetadata[0].output_for_plotting.size());
    std::vector<std::string> tGoldOutputs = {"dispx", "dispy", "dispz"};
    for(auto& tValue : tObjectiveMetadata[0].output_for_plotting)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[0].output_for_plotting[0];
        ASSERT_STREQ(tGoldOutputs[tIndex].c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseObjective_TwoObjective)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n"
        "begin objective\n"
        "   type stress p-norm\n"
        "   normalize objective false\n"
        "   load ids 10 20\n"
        "   boundary condition ids 11 21\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dIspy dispz vonmises\n"
        "   distribute objective at most 3 processors\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    tObjectiveParser.parse(tInputSS);
    auto tObjectiveMetadata = tObjectiveParser.data();
    ASSERT_EQ(2u, tObjectiveMetadata.size());

    // TEST OBJECTIVE 1
    ASSERT_STREQ("compliance", tObjectiveMetadata[0].type.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_procs.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_ranks.c_str());
    ASSERT_STREQ("1.0", tObjectiveMetadata[0].weight.c_str());
    ASSERT_STREQ("3.0", tObjectiveMetadata[0].mPenaltyParam.c_str());
    ASSERT_STREQ("6.0", tObjectiveMetadata[0].mPnormExponent.c_str());
    ASSERT_STREQ("1e-9", tObjectiveMetadata[0].mMinimumErsatzValue.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[0].code_name.c_str());
    ASSERT_STREQ("plato_analyze_1", tObjectiveMetadata[0].mPerformerName.c_str());
    ASSERT_STREQ("true", tObjectiveMetadata[0].normalize_objective.c_str());
    ASSERT_STREQ("1e-7", tObjectiveMetadata[0].analysis_solver_tolerance.c_str());
    ASSERT_STREQ("false", tObjectiveMetadata[0].multi_load_case.c_str());
    ASSERT_STREQ("none", tObjectiveMetadata[0].distribute_objective_type.c_str());

    ASSERT_EQ(1u, tObjectiveMetadata[0].bc_ids.size());
    ASSERT_STREQ("11", tObjectiveMetadata[0].bc_ids[0].c_str());
    ASSERT_EQ(1u, tObjectiveMetadata[0].load_case_ids.size());
    ASSERT_STREQ("10", tObjectiveMetadata[0].load_case_ids[0].c_str());

    ASSERT_EQ(3u, tObjectiveMetadata[0].output_for_plotting.size());
    std::vector<std::string> tGoldOutputs = {"dispx", "dispy", "dispz"};
    for(auto& tValue : tObjectiveMetadata[0].output_for_plotting)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[0].output_for_plotting[0];
        ASSERT_STREQ(tGoldOutputs[tIndex].c_str(), tValue.c_str());
    }

    // TEST OBJECTIVE 2
    ASSERT_STREQ("stress p-norm", tObjectiveMetadata[1].type.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[1].num_procs.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[1].num_ranks.c_str());
    ASSERT_STREQ("1.0", tObjectiveMetadata[1].weight.c_str());
    ASSERT_STREQ("3.0", tObjectiveMetadata[1].mPenaltyParam.c_str());
    ASSERT_STREQ("6.0", tObjectiveMetadata[1].mPnormExponent.c_str());
    ASSERT_STREQ("1e-9", tObjectiveMetadata[1].mMinimumErsatzValue.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[1].code_name.c_str());
    ASSERT_STREQ("plato_analyze_2", tObjectiveMetadata[1].mPerformerName.c_str());
    ASSERT_STREQ("false", tObjectiveMetadata[1].normalize_objective.c_str());
    ASSERT_STREQ("1e-7", tObjectiveMetadata[1].analysis_solver_tolerance.c_str());
    ASSERT_STREQ("true", tObjectiveMetadata[1].multi_load_case.c_str());
    ASSERT_STREQ("3", tObjectiveMetadata[1].atmost_total_num_processors.c_str());
    ASSERT_STREQ("atmost", tObjectiveMetadata[1].distribute_objective_type.c_str());

    std::vector<std::string> tGoldEssentialBCIDs = {"11", "21"};
    ASSERT_EQ(2u, tObjectiveMetadata[1].bc_ids.size());
    for(auto& tValue : tObjectiveMetadata[1].bc_ids)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[1].bc_ids[0];
        ASSERT_STREQ(tGoldEssentialBCIDs[tIndex].c_str(), tValue.c_str());
    }

    std::vector<std::string> tGoldNaturalBCIDs = {"10", "20"};
    ASSERT_EQ(2u, tObjectiveMetadata[1].load_case_ids.size());
    for(auto& tValue : tObjectiveMetadata[1].load_case_ids)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[1].load_case_ids[0];
        ASSERT_STREQ(tGoldNaturalBCIDs[tIndex].c_str(), tValue.c_str());
    }

    ASSERT_EQ(4u, tObjectiveMetadata[1].output_for_plotting.size());
    tGoldOutputs = {"dispx", "dispy", "dispz", "vonmises"};
    for(auto& tValue : tObjectiveMetadata[0].output_for_plotting)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[0].output_for_plotting[0];
        ASSERT_STREQ(tGoldOutputs[tIndex].c_str(), tValue.c_str());
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
