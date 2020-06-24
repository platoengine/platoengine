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
        "   youngs modulus 200e9\n"
        "   poissons ratio 0.33\n"
        "end material\n"
        "begin material 1\n"
        "   material_model isotropic linear elastic\n"
        "   youngs modulus 200e9\n"
        "   poissons ratio 0.33\n"
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
        "   youngs modulus 200e9\n"
        "   poissons ratio 0.33\n"
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
    ASSERT_STREQ("0.33", tMaterialMetaData[0].property("poissons ratio").c_str());
    ASSERT_STREQ("200e9", tMaterialMetaData[0].property("youngs modulus").c_str());
    ASSERT_STREQ("isotropic linear elastic", tMaterialMetaData[0].category().c_str());
}

TEST(PlatoTestXMLGenerator, ParseMaterial_TwoMaterial)
{
    std::string tStringInput =
        "begin material 1\n"
        "   material_model isotropic linear elastic\n"
        "   youngs modulus 200e9\n"
        "   poissons ratio 0.33\n"
        "end material\n"
        "begin material\n"
        "   id 2\n"
        "   code sierra_sd\n"
        "   material_model isotropic linear elastic\n"
        "   youngs modulus 100e9\n"
        "   poissons ratio 0.35\n"
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
    ASSERT_STREQ("0.33", tMaterialMetaData[0].property("poissons ratio").c_str());
    ASSERT_STREQ("200e9", tMaterialMetaData[0].property("youngs modulus").c_str());
    ASSERT_STREQ("isotropic linear elastic", tMaterialMetaData[0].category().c_str());

    tTags = tMaterialMetaData[1].tags();
    ASSERT_EQ(3u, tTags.size());
    ASSERT_STREQ("sierra_sd", tMaterialMetaData[1].code().c_str());
    ASSERT_STREQ("0.35", tMaterialMetaData[1].property("poissons ratio").c_str());
    ASSERT_STREQ("100e9", tMaterialMetaData[1].property("youngs modulus").c_str());
    ASSERT_STREQ("3.0", tMaterialMetaData[1].property("penalty exponent").c_str());
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
        ASSERT_STREQ("plato_analyze_1", tConstraint.performer().c_str());
        ASSERT_STREQ("6.0", tConstraint.pnormExponent().c_str());
        ASSERT_STREQ("1.0", tConstraint.weight().c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseConstraint_OneConstraint)
{
    std::string tStringInput =
        "begin constraint\n"
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
        "   type volume\n"
        "   volume absolute 10.0\n"
        "end constraint\n"
        "begin constraint\n"
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
    ASSERT_STREQ("plato_analyze", tConstraintMetadata[0].code().c_str());
    ASSERT_STREQ("10.0", tConstraintMetadata[0].absoluteTarget().c_str());
    ASSERT_STREQ("plato_analyze_1", tConstraintMetadata[0].performer().c_str());
    ASSERT_STREQ("3.0", tConstraintMetadata[0].materialPenaltyExponent().c_str());
    ASSERT_STREQ("1e-9", tConstraintMetadata[0].minErsatzMaterialConstant().c_str());

    // Constraint Two
    ASSERT_TRUE(tConstraintMetadata[1].absoluteTarget().empty());
    ASSERT_STREQ("2", tConstraintMetadata[1].name().c_str());
    ASSERT_STREQ("1.0", tConstraintMetadata[1].weight().c_str());
    ASSERT_STREQ("stress p-norm", tConstraintMetadata[1].category().c_str());
    ASSERT_STREQ("30.0", tConstraintMetadata[1].pnormExponent().c_str());
    ASSERT_STREQ("plato_analyze", tConstraintMetadata[1].code().c_str());
    ASSERT_STREQ("0.4", tConstraintMetadata[1].normalizedTarget().c_str());
    ASSERT_STREQ("plato_analyze_1", tConstraintMetadata[1].performer().c_str());
    ASSERT_STREQ("3.0", tConstraintMetadata[1].materialPenaltyExponent().c_str());
    ASSERT_STREQ("1e-9", tConstraintMetadata[1].minErsatzMaterialConstant().c_str());
}

TEST(PlatoTestXMLGenerator, ParseScenario_ErrorEmptyScenarioMetadata)
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

TEST(PlatoTestXMLGenerator, ParseScenario_DefaultValues)
{
    std::string tStringInput =
        "begin scenario\n"
        "   physics mechanical\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    tScenarioParser.parse(tInputSS);
    auto tScenarioMetadata = tScenarioParser.data();

    ASSERT_FALSE(tScenarioMetadata.useNewAnalyzeUQWorkflow());
    ASSERT_STREQ("plato_analyze", tScenarioMetadata.code().c_str());
    ASSERT_STREQ("mechanical", tScenarioMetadata.physics().c_str());
    ASSERT_STREQ("plato_analyze_0", tScenarioMetadata.performer().c_str());
    ASSERT_STREQ("plato_analyze_mechanical_0", tScenarioMetadata.id().c_str());
    ASSERT_STREQ("3", tScenarioMetadata.dimensions().c_str());
    ASSERT_STREQ("3.0", tScenarioMetadata.materialPenaltyExponent().c_str());
    ASSERT_STREQ("1e-9", tScenarioMetadata.minErsatzMaterialConstant().c_str());
}

TEST(PlatoTestXMLGenerator, ParseScenario)
{
    std::string tStringInput =
        "begin scenario\n"
        "   physics thermal\n"
        "   use_new_analyze_uq_workflow true\n"
        "   dimensions 2\n"
        "   code sierra_sd\n"
        "   performer sierra_sd_0\n"
        "   material_penalty_exponent 1.0\n"
        "   minimum_ersatz_material_value 1e-6\n"
        "   scenario_id air_force_one\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    tScenarioParser.parse(tInputSS);
    auto tScenarioMetadata = tScenarioParser.data();

    ASSERT_TRUE(tScenarioMetadata.useNewAnalyzeUQWorkflow());
    ASSERT_STREQ("sierra_sd", tScenarioMetadata.code().c_str());
    ASSERT_STREQ("thermal", tScenarioMetadata.physics().c_str());
    ASSERT_STREQ("sierra_sd_0", tScenarioMetadata.performer().c_str());
    ASSERT_STREQ("air_force_one", tScenarioMetadata.id().c_str());
    ASSERT_STREQ("2", tScenarioMetadata.dimensions().c_str());
    ASSERT_STREQ("1.0", tScenarioMetadata.materialPenaltyExponent().c_str());
    ASSERT_STREQ("1e-6", tScenarioMetadata.minErsatzMaterialConstant().c_str());
}

TEST(PlatoTestXMLGenerator, ParseOutput_EmptyOutputMetadata)
{
    std::string tStringInput =
        "begin output\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));

    auto tOutputMetadata = tOutputParser.data();
    ASSERT_TRUE(tOutputMetadata.getRandomQoI().empty());
    ASSERT_TRUE(tOutputMetadata.getDeterminsiticQoI().empty());
}

TEST(PlatoTestXMLGenerator, ParseOutput_ErrorInvalidQoI)
{
    std::string tStringInput =
        "begin output\n"
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
        "   quantities_of_interest dispx dispy dispz\n"
        "   random_quantities_of_interest dispx dispy hippo dispz\n"
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
        "   quantities_of_interest dispx dispy dispz temperature\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));
    auto tOutputMetadata = tOutputParser.data();
    ASSERT_TRUE(tOutputMetadata.getRandomQoI().empty());
    ASSERT_FALSE(tOutputMetadata.getDeterminsiticQoI().empty());

    std::vector<std::string> tGoldDeterministic = {"dispx", "dispy", "dispz", "temperature"};
    for(auto& tValue : tOutputMetadata.getDeterminsiticQoI())
    {
        auto tItr = std::find(tGoldDeterministic.begin(), tGoldDeterministic.end(), tValue.first);
        ASSERT_TRUE(tItr != tGoldDeterministic.end());
        ASSERT_STREQ(tItr->c_str(), tValue.first.c_str());
        ASSERT_STREQ("nodal field", tValue.second.c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseOutput_RandomOnly)
{
    std::string tStringInput =
        "begin output\n"
        "   random_quantities_of_interest accumulated_plastic_strain temperature\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));
    auto tOutputMetadata = tOutputParser.data();
    ASSERT_FALSE(tOutputMetadata.getRandomQoI().empty());
    ASSERT_TRUE(tOutputMetadata.getDeterminsiticQoI().empty());

    std::vector<std::string> tGoldRandom = {"accumulated_plastic_strain", "temperature"};
    std::vector<std::string> tGoldRandomLayout = {"element field", "nodal field"};
    for(auto& tValue : tOutputMetadata.getRandomQoI())
    {
        auto tItr = std::find(tGoldRandom.begin(), tGoldRandom.end(), tValue.first);
        ASSERT_TRUE(tItr != tGoldRandom.end());
        ASSERT_STREQ(tItr->c_str(), tValue.first.c_str());

        tItr = std::find(tGoldRandomLayout.begin(), tGoldRandomLayout.end(), tValue.second);
        ASSERT_TRUE(tItr != tGoldRandomLayout.end());
        ASSERT_STREQ(tItr->c_str(), tValue.second.c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseOutput_DeterministicPlusRandom)
{
    std::string tStringInput =
        "begin output\n"
        "   quantities_of_interest dispx dispy dispz temperature\n"
        "   random_quantities_of_interest accumulated_plastic_strain temperature\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));
    auto tOutputMetadata = tOutputParser.data();
    ASSERT_FALSE(tOutputMetadata.getRandomQoI().empty());
    ASSERT_FALSE(tOutputMetadata.getDeterminsiticQoI().empty());

    std::vector<std::string> tGoldRandom = {"accumulated_plastic_strain", "temperature"};
    std::vector<std::string> tGoldRandomLayout = {"element field", "nodal field"};
    for(auto& tValue : tOutputMetadata.getRandomQoI())
    {
        auto tItr = std::find(tGoldRandom.begin(), tGoldRandom.end(), tValue.first);
        ASSERT_TRUE(tItr != tGoldRandom.end());
        ASSERT_STREQ(tItr->c_str(), tValue.first.c_str());

        tItr = std::find(tGoldRandomLayout.begin(), tGoldRandomLayout.end(), tValue.second);
        ASSERT_TRUE(tItr != tGoldRandomLayout.end());
        ASSERT_STREQ(tItr->c_str(), tValue.second.c_str());
    }
    std::vector<std::string> tGoldDeterministic = {"dispx", "dispy", "dispz", "temperature"};
    for(auto& tValue : tOutputMetadata.getDeterminsiticQoI())
    {
        auto tItr = std::find(tGoldDeterministic.begin(), tGoldDeterministic.end(), tValue.first);
        ASSERT_TRUE(tItr != tGoldDeterministic.end());
        ASSERT_STREQ(tItr->c_str(), tValue.first.c_str());
        ASSERT_STREQ("nodal field", tValue.second.c_str());
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
