#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "pugixml.hpp"
#include <Plato_FreeFunctions.hpp>
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorLaunchScriptUtilities.hpp"
#include "XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.hpp"

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, generateSummitLaunchScripts_non_analyze_performer)
{
  XMLGen::InputData tInputData;
  XMLGen::Objective tObjective;
  tObjective.code_name = "sierra_sd";
  tInputData.objectives.push_back(tObjective);

  EXPECT_THROW(XMLGen::generate_summit_launch_scripts(tInputData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, generateAnalyzeBashScript)
{
  XMLGen::generate_analyze_bash_script();
  
  auto tReadData = XMLGen::read_data_from_file("analyze.sh");
  auto tGold = std::string("exportPLATO_PERFORMER_ID=1exportPLATO_INTERFACE_FILE=interface.xml") +
               std::string("exportPLATO_APP_FILE=plato_analyze_operations.xmlanalyze_MPMD") +
               std::string("--input-config=plato_analyze_input_deck.xml"); 

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf analyze.sh");
}

TEST(PlatoTestXMLGenerator, appendAnalyzeMPIRunLines)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  tInputData.num_opt_processors = "10";
  tInputData.m_UncertaintyMetaData.numPerformers = 5;
  FILE* fp=fopen("appendEngineMPIRunLines.txt", "w");
  XMLGen::append_analyze_mpirun_lines(tInputData, fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendEngineMPIRunLines.txt");
  auto tGold = std::string(":-np5-xPLATO_PERFORMER_ID=1\\-xPLATO_INTERFACE_FILE=interface.xml\\-xPLATO_APP_FILE=plato_analyze_operations.xml\\analyze_MPMD--input-config=plato_analyze_input_deck.xml\\");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendEngineMPIRunLines.txt");
}

TEST(PlatoTestXMLGenerator, appendAnalyzeMPIRunLines_noPerformers)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  tInputData.num_opt_processors = "10";
  FILE* fp=fopen("appendEngineMPIRunLines.txt", "w");
  EXPECT_THROW(XMLGen::append_analyze_mpirun_lines(tInputData, fp),std::runtime_error);
  fclose(fp);
}

}