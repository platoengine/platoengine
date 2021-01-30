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

  XMLGen::Service tService1;
  tService1.id("1");
  tService1.code("sierra_sd");
  tInputData.append(tService1);

  tInputData.objective.serviceIDs.push_back("1");

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

  XMLGen::Service tService;
  tService.numberProcessors("10");
  tService.code("platomain");
  tService.id("1");
  tInputData.append(tService);
  tService.numberProcessors("1");
  tService.code("plato_analyze");
  tService.id("2");
  tInputData.append(tService);
  tInputData.mPerformerServices.push_back(tService);

  tInputData.m_UseLaunch = false;
  tInputData.mesh.run_name = "dummy_mesh.exo";
  tInputData.m_UncertaintyMetaData.numPerformers = 5;
  FILE* fp=fopen("appendEngineMPIRunLines.txt", "w");
  int tPerformerID = 1;
  XMLGen::append_analyze_mpirun_commands_robust_optimization_problems(tInputData, tPerformerID, fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendEngineMPIRunLines.txt");
  auto tGold = std::string(":-np5-xPLATO_PERFORMER_ID=1\\-xPLATO_INTERFACE_FILE=interface.xml\\-xPLATO_APP_FILE=plato_analyze_2_operations.xml\\analyze_MPMD--input-config=plato_analyze_2_input_deck.xml\\");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendEngineMPIRunLines.txt");
}

TEST(PlatoTestXMLGenerator, appendAnalyzeMPIRunLines_noPerformers)
{
  XMLGen::InputData tInputData;
  tInputData.mesh.run_name = "dummy_mesh.exo";

  XMLGen::Service tService;
  tService.numberProcessors("10");
  tService.code("platomain");
  tInputData.append(tService);

  FILE* fp=fopen("appendEngineMPIRunLines.txt", "w");
  int tPerformerID = 0;
  EXPECT_THROW(XMLGen::append_analyze_mpirun_commands_robust_optimization_problems(tInputData, tPerformerID, fp),std::runtime_error);
  fclose(fp);
}

}
