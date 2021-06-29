#include <gtest/gtest.h>

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGenerator_UnitTester_Tools.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorSierraSDInputDeckUtilities.hpp"

namespace PlatoTestXMLGenerator {

XMLGen::InputData setUpMetaDataForExistingSDInputDeck(const char *input) {
    XMLGen::InputData aXMLMetaData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("sierra_sd");
    aXMLMetaData.append(tService);

    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.physics("steady_state_mechanics");
    aXMLMetaData.append(tScenario);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("mass");
    aXMLMetaData.append(tCriterion);

    XMLGen::Objective tObjective;
    tObjective.serviceIDs.push_back("1");
    tObjective.criteriaIDs.push_back("1");
    tObjective.scenarioIDs.push_back("1");
    tObjective.multi_load_case = "false";
    aXMLMetaData.objective = tObjective;

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("discretization", "density");
    tOptimizationParameters.append("optimization_type", "topology");
    aXMLMetaData.set(tOptimizationParameters);

    return aXMLMetaData;
}

TEST(PlatoTestXMLGenerator, SD_changeSolutionCase)
{
    constexpr char input[] =
    "SOLUTION\n"
    "  case '1'\n"
    "  statics\n"
    "  solver gdsw\n"
    "END\n";

    std::stringstream iDeck(input), oDeck;

    XMLGen::InputData aXMLMetaData = setUpMetaDataForExistingSDInputDeck(input);
    XMLGen::augment_sierra_sd_input_deck_with_plato_problem_description(aXMLMetaData, iDeck, oDeck);

    constexpr char expected_output[] =
    "SOLUTION\n"
    "  case '1'\n"
    "  topology_optimization\n"
    "  solver gdsw\n"
    "END\n"
    "TOPOLOGY-OPTIMIZATION\n"
    "  algorithm = plato_engine\n"
    "  volume_fraction = .314\n"
    "END\n";

    EXPECT_EQ(strcmp(oDeck.str().c_str(),expected_output),0);
}

TEST(PlatoTestXMLGenerator, SD_augmentMaterialSection)
{
    constexpr char input[] =
    "MATERIAL 1\n"
    "  isotropic\n"
    "  E = 1e9\n"
    "  nu = .33\n"
    "END\n";

    std::stringstream iDeck(input), oDeck;

    XMLGen::InputData aXMLMetaData = setUpMetaDataForExistingSDInputDeck(input);
    XMLGen::augment_sierra_sd_input_deck_with_plato_problem_description(aXMLMetaData, iDeck, oDeck);

    constexpr char expected_output[] =
    "MATERIAL 1\n"
    "  isotropic\n"
    "  E = 1e9\n"
    "  nu = .33\n"
    "  material_penalty_model = simp\n"
    "END\n"
    "TOPOLOGY-OPTIMIZATION\n"
    "  algorithm = plato_engine\n"
    "  volume_fraction = .314\n"
    "END\n";

    EXPECT_EQ(strcmp(oDeck.str().c_str(),expected_output),0);
}

TEST(PlatoTestXMLGenerator, SD_augmentOutputsSection)
{
    constexpr char input[] =
    "OUTPUTS\n"
    "  disp\n"
    "END\n";

    std::stringstream iDeck(input), oDeck;

    XMLGen::InputData aXMLMetaData = setUpMetaDataForExistingSDInputDeck(input);
    XMLGen::augment_sierra_sd_input_deck_with_plato_problem_description(aXMLMetaData, iDeck, oDeck);

    constexpr char expected_output[] =
    "OUTPUTS\n"
    "  disp\n"
    "  topology\n"
    "END\n"
    "TOPOLOGY-OPTIMIZATION\n"
    "  algorithm = plato_engine\n"
    "  volume_fraction = .314\n"
    "END\n";

    EXPECT_EQ(strcmp(oDeck.str().c_str(),expected_output),0);
}

TEST(PlatoTestXMLGenerator, SD_augmentEchoSection)
{
    constexpr char input[] =
    "ECHO\n"
    "  disp\n"
    "END\n";

    std::stringstream iDeck(input), oDeck;

    XMLGen::InputData aXMLMetaData = setUpMetaDataForExistingSDInputDeck(input);
    XMLGen::augment_sierra_sd_input_deck_with_plato_problem_description(aXMLMetaData, iDeck, oDeck);

    constexpr char expected_output[] =
    "ECHO\n"
    "  disp\n"
    "  topology\n"
    "END\n"
    "TOPOLOGY-OPTIMIZATION\n"
    "  algorithm = plato_engine\n"
    "  volume_fraction = .314\n"
    "END\n";

    EXPECT_EQ(strcmp(oDeck.str().c_str(),expected_output),0);
}

TEST(PlatoTestXMLGenerator, SD_deckWithManySections)
{
    constexpr char input[] =
    "SOLUTION\n"
    "  case '1'\n"
    "  statics\n"
    "  case '2'\n"
    "  statics\n"
    "  solver gdsw\n"
    "END\n"
    "MATERIAL 1\n"
    "  isotropic\n"
    "  E = 1e9\n"
    "  nu = .33\n"
    "END\n"
    "OUTPUTS\n"
    "  disp\n"
    "END\n"
    "ECHO\n"
    "  disp\n"
    "END\n";

    std::stringstream iDeck(input), oDeck;

    XMLGen::InputData aXMLMetaData = setUpMetaDataForExistingSDInputDeck(input);
    XMLGen::augment_sierra_sd_input_deck_with_plato_problem_description(aXMLMetaData, iDeck, oDeck);

    constexpr char expected_output[] =
    "SOLUTION\n"
    "  case '1'\n"
    "  topology_optimization\n"
    "  case '2'\n"
    "  topology_optimization\n"
    "  solver gdsw\n"
    "END\n"
    "MATERIAL 1\n"
    "  isotropic\n"
    "  E = 1e9\n"
    "  nu = .33\n"
    "  material_penalty_model = simp\n"
    "END\n"
    "OUTPUTS\n"
    "  disp\n"
    "  topology\n"
    "END\n"
    "ECHO\n"
    "  disp\n"
    "  topology\n"
    "END\n"
    "TOPOLOGY-OPTIMIZATION\n"
    "  algorithm = plato_engine\n"
    "  volume_fraction = .314\n"
    "END\n";

    EXPECT_EQ(strcmp(oDeck.str().c_str(),expected_output),0);
}


} // namespace PlatoTestXMLGenerator