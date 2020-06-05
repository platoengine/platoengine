#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "pugixml.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, ComputeNumberOfNodesNeeded_invalidDenominator)
{
    ASSERT_THROW(XMLGen::compute_number_of_nodes_needed(10,0),std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ComputeNumberOfNodesNeeded)
{
    size_t tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,1);
    ASSERT_EQ(tNumNodesNeeded,10);
    tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,2);
    ASSERT_EQ(tNumNodesNeeded,5);
    tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,3);
    ASSERT_EQ(tNumNodesNeeded,4);
    tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,4);
    ASSERT_EQ(tNumNodesNeeded,3);
    tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,5);
    ASSERT_EQ(tNumNodesNeeded,2);
    tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,10);
    ASSERT_EQ(tNumNodesNeeded,1);
}

TEST(PlatoTestXMLGenerator, generateBatchScript)
{
  XMLGen::generate_batch_script(12);
  
  auto tReadData = XMLGen::read_data_from_file("plato.batch");
  auto tGold = std::string("#!/bin/bash#LSFDirectives#BSUB-P<PROJECT>#BSUB-W0:00#BSUB-nnodes2#BSUB-Jplato") +
               std::string("#BSUB-oplato.%Jcd<path/to/working/directory>datejsrun-Aeng-n1-a1-c1-g0jsrun-Aper-n12-a1-c1-g1jsrun-fjsrun.source");

  ASSERT_STREQ(tReadData.str().c_str(),tGold.c_str());
  std::system("rm -rf plato.batch");
}

TEST(PlatoTestXMLGenerator, generateJSRunScript)
{
  XMLGen::generate_jsrun_script(12);
  
  auto tReadData = XMLGen::read_data_from_file("jsrun.source");
  auto tGold = std::string("1:eng:bashengine.sh12:per:bashanalyze.sh");

  ASSERT_STREQ(tReadData.str().c_str(),tGold.c_str());
  std::system("rm -rf jsrun.source");
}

TEST(PlatoTestXMLGenerator, generateEngineBashScript)
{
  XMLGen::generate_engine_bash_script();
  
  auto tReadData = XMLGen::read_data_from_file("engine.sh");
  auto tGold = std::string("exportPLATO_PERFORMER_ID=0exportPLATO_INTERFACE_FILE=interface.xml") + 
               std::string("exportPLATO_APP_FILE=plato_main_operations.xmlPlatoMainplato_main_input_deck.xml");

  ASSERT_STREQ(tReadData.str().c_str(),tGold.c_str());
  std::system("rm -rf engine.sh");
}

TEST(PlatoTestXMLGenerator, generateAnalyzeBashScript)
{
  XMLGen::generate_analyze_bash_script();
  
  auto tReadData = XMLGen::read_data_from_file("analyze.sh");
  auto tGold = std::string("exportPLATO_PERFORMER_ID=1exportPLATO_INTERFACE_FILE=interface.xml") +
               std::string("exportPLATO_APP_FILE=plato_analyze_operations.xmlanalyze_MPMD") +
               std::string("--input-config=plato_analyze_input_deck.xml"); 

  ASSERT_STREQ(tReadData.str().c_str(),tGold.c_str());
  std::system("rm -rf analyze.sh");
}

TEST(PlatoTestXMLGenerator, generateSummitLaunchScripts_non_analyze_performer)
{
  XMLGen::InputData tInputData;
  XMLGen::Objective tObjective;
  tObjective.code_name = "sierra_sd";
  tInputData.objectives.push_back(tObjective);

  ASSERT_THROW(XMLGen::generate_summit_launch_scripts(tInputData), std::runtime_error);
}

}
