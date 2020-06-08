#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "pugixml.hpp"
#include <Plato_FreeFunctions.hpp>
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorLaunchScriptUtilities.hpp"

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, ComputeNumberOfNodesNeeded_invalidDenominator)
{
    EXPECT_THROW(XMLGen::compute_number_of_nodes_needed(10,0),std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ComputeNumberOfNodesNeeded)
{
    size_t tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,1);
    EXPECT_EQ(tNumNodesNeeded,10);
    tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,2);
    EXPECT_EQ(tNumNodesNeeded,5);
    tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,3);
    EXPECT_EQ(tNumNodesNeeded,4);
    tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,4);
    EXPECT_EQ(tNumNodesNeeded,3);
    tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,5);
    EXPECT_EQ(tNumNodesNeeded,2);
    tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(10,10);
    EXPECT_EQ(tNumNodesNeeded,1);
}

TEST(PlatoTestXMLGenerator, generateBatchScript)
{
  XMLGen::generate_batch_script(12);
  
  auto tReadData = XMLGen::read_data_from_file("plato.batch");
  auto tGold = std::string("#!/bin/bash#LSFDirectives#BSUB-P<PROJECT>#BSUB-W0:00#BSUB-nnodes2#BSUB-Jplato") +
               std::string("#BSUB-oplato.%Jcd<path/to/working/directory>datejsrun-Aeng-n1-a1-c1-g0jsrun-Aper-n12-a1-c1-g1jsrun-fjsrun.source");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf plato.batch");
}

TEST(PlatoTestXMLGenerator, generateJSRunScript)
{
  XMLGen::generate_jsrun_script(12);
  
  auto tReadData = XMLGen::read_data_from_file("jsrun.source");
  auto tGold = std::string("1:eng:bashengine.sh12:per:bashanalyze.sh");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf jsrun.source");
}

TEST(PlatoTestXMLGenerator, generateEngineBashScript)
{
  XMLGen::generate_engine_bash_script();
  
  auto tReadData = XMLGen::read_data_from_file("engine.sh");
  auto tGold = std::string("exportPLATO_PERFORMER_ID=0exportPLATO_INTERFACE_FILE=interface.xml") + 
               std::string("exportPLATO_APP_FILE=plato_main_operations.xmlPlatoMainplato_main_input_deck.xml");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf engine.sh");
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

TEST(PlatoTestXMLGenerator, generateSummitLaunchScripts_non_analyze_performer)
{
  XMLGen::InputData tInputData;
  XMLGen::Objective tObjective;
  tObjective.code_name = "sierra_sd";
  tInputData.objectives.push_back(tObjective);

  EXPECT_THROW(XMLGen::generate_summit_launch_scripts(tInputData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, getNumOptProcs_numProcsNotSpecified)
{
  XMLGen::InputData tInputData;
  std::string num_opt_processors = XMLGen::get_num_opt_processors(tInputData);
  EXPECT_STREQ("1", num_opt_processors.c_str());
}

TEST(PlatoTestXMLGenerator, getNumOptProcs_numProcsSpecified)
{
  XMLGen::InputData tInputData;
  tInputData.num_opt_processors = "10";
  std::string num_opt_processors = XMLGen::get_num_opt_processors(tInputData);
  EXPECT_STREQ("10", num_opt_processors.c_str());
}

TEST(PlatoTestXMLGenerator, getNumProcs_numProcsNotSpecified)
{
  XMLGen::Objective tObjective;
  std::string num_procs = XMLGen::get_num_procs(tObjective);
  EXPECT_STREQ("4", num_procs.c_str());
}

TEST(PlatoTestXMLGenerator, getNumProcs_numProcsSpecified)
{
  XMLGen::Objective tObjective;
  tObjective.num_procs = "10";
  std::string num_procs = XMLGen::get_num_procs(tObjective);
  EXPECT_STREQ("10", num_procs.c_str());
}

TEST(PlatoTestXMLGenerator, determineMPIEnvAndSeparationStrings)
{
  std::string envString, separationString;
  XMLGen::determine_mpi_env_and_separation_strings(envString, separationString);

#ifndef USING_OPEN_MPI
  EXPECT_STREQ("-env", envString.c_str());
  EXPECT_STREQ(" ", separationString.c_str());
#else
  EXPECT_STREQ("-x", envString.c_str());
  EXPECT_STREQ("=", separationString.c_str());
#endif
}

TEST(PlatoTestXMLGenerator, determineMPILaunchStrings_useLaunch)
{
  XMLGen::InputData tInputData;
  tInputData.m_UseLaunch = true;
  std::string tLaunchString, tNumProcsString;
  XMLGen::determine_mpi_launch_strings(tInputData, tLaunchString, tNumProcsString);

  EXPECT_STREQ("launch", tLaunchString.c_str());
  EXPECT_STREQ("-n", tNumProcsString.c_str());
}

TEST(PlatoTestXMLGenerator, determineMPILaunchStrings_dontUseLaunch)
{
  XMLGen::InputData tInputData;
  tInputData.m_UseLaunch = false;
  std::string tLaunchString, tNumProcsString;
  XMLGen::determine_mpi_launch_strings(tInputData, tLaunchString, tNumProcsString);

  EXPECT_STREQ("mpiexec", tLaunchString.c_str());
  EXPECT_STREQ("-np", tNumProcsString.c_str());
}

TEST(PlatoTestXMLGenerator, determineMPILaunchStrings_notSet)
{
  XMLGen::InputData tInputData;
  std::string tLaunchString, tNumProcsString;
  XMLGen::determine_mpi_launch_strings(tInputData, tLaunchString, tNumProcsString);

  EXPECT_STREQ("mpiexec", tLaunchString.c_str());
  EXPECT_STREQ("-np", tNumProcsString.c_str());
}

TEST(PlatoTestXMLGenerator, appendDecompLine)
{
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::string num_processors = "10";
  std::string mesh_file_name = "dummy_mesh.exo";
  XMLGen::append_decomp_line(fp, num_processors, mesh_file_name);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("decomp-p10dummy_mesh.exo");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForOptimizer)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  tInputData.initial_guess_filename = "dummy_initial_guess.exo";
  tInputData.num_opt_processors = "10";
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  XMLGen::append_decomp_lines_for_optimizer(tInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("decomp-p10dummy_mesh.exodecomp-p10dummy_initial_guess.exo");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForOptimizer_emptyMeshName)
{
  XMLGen::InputData tInputData;
  tInputData.num_opt_processors = "10";
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  EXPECT_THROW(XMLGen::append_decomp_lines_for_optimizer(tInputData, fp,
                  hasBeenDecompedForThisNumberOfProcessors), std::runtime_error);
  fclose(fp);

  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForOptimizer_invalidNumProcessors)
{
  XMLGen::InputData tInputData;
  tInputData.num_opt_processors = "0";
  tInputData.run_mesh_name = "dummy_mesh.exo";
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  EXPECT_THROW(XMLGen::append_decomp_lines_for_optimizer(tInputData, fp,
                  hasBeenDecompedForThisNumberOfProcessors), std::runtime_error);

  tInputData.num_opt_processors = "-10";
  EXPECT_THROW(XMLGen::append_decomp_lines_for_optimizer(tInputData, fp,
                  hasBeenDecompedForThisNumberOfProcessors), std::runtime_error);

  tInputData.num_opt_processors = "mmm food";
  EXPECT_THROW(XMLGen::append_decomp_lines_for_optimizer(tInputData, fp,
                  hasBeenDecompedForThisNumberOfProcessors), std::runtime_error);

  fclose(fp);

  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForOptimizer_emptyInitialGuess)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  tInputData.num_opt_processors = "10";
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  XMLGen::append_decomp_lines_for_optimizer(tInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("decomp-p10dummy_mesh.exo");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForOptimizer_noNeedToDecompose)
{
  XMLGen::InputData tInputData;
  tInputData.num_opt_processors = "1";
  tInputData.run_mesh_name = "dummy_mesh.exo";
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  XMLGen::append_decomp_lines_for_optimizer(tInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForOptimizer_hasBeenDecomped)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  tInputData.num_opt_processors = "10";
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  hasBeenDecompedForThisNumberOfProcessors["10"] = 1;
  XMLGen::append_decomp_lines_for_optimizer(tInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForPerformers_emptyMeshName)
{
  XMLGen::InputData tInputData;
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  EXPECT_THROW(XMLGen::append_decomp_lines_for_performers(tInputData, fp,
                                hasBeenDecompedForThisNumberOfProcessors), std::runtime_error);
  fclose(fp);
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForPerformers)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  XMLGen::Objective tObjective;
  tObjective.num_procs = "10";
  tObjective.ref_frf_file = "dummy_frf_file.exo";
  tInputData.objectives.push_back(tObjective);
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  XMLGen::append_decomp_lines_for_performers(tInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("decomp-p10dummy_mesh.exodecomp-p10dummy_frf_file.exo");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForPerformers_noObjectives)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  XMLGen::append_decomp_lines_for_performers(tInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForPerformers_noNeedToDecompose)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  XMLGen::Objective tObjective;
  tObjective.num_procs = "1";
  tObjective.ref_frf_file = "dummy_frf_file.exo";
  tInputData.objectives.push_back(tObjective);
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  XMLGen::append_decomp_lines_for_performers(tInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForPerformers_multipleObjectivesSameNumProcs)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  XMLGen::Objective tObjective;
  tObjective.num_procs = "10";
  tObjective.ref_frf_file = "dummy_frf_file.exo";
  tInputData.objectives.push_back(tObjective);
  XMLGen::Objective tObjective2;
  tObjective2.num_procs = "10";
  tObjective2.ref_frf_file = "dummy_frf_file2.exo";
  tInputData.objectives.push_back(tObjective2);
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  XMLGen::append_decomp_lines_for_performers(tInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("decomp-p10dummy_mesh.exodecomp-p10dummy_frf_file.exodecomp-p10dummy_frf_file2.exo");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForPerformers_hasBeenDecomposed)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  XMLGen::Objective tObjective;
  tObjective.num_procs = "10";
  tObjective.ref_frf_file = "dummy_frf_file.exo";
  tInputData.objectives.push_back(tObjective);
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  hasBeenDecompedForThisNumberOfProcessors["10"] = 1;
  XMLGen::append_decomp_lines_for_performers(tInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("decomp-p10dummy_frf_file.exo");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesToMPILaunchScript)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  XMLGen::Objective tObjective;
  tObjective.num_procs = "10";
  tObjective.ref_frf_file = "dummy_frf_file.exo";
  tInputData.objectives.push_back(tObjective);
  tInputData.num_opt_processors = "5";
  FILE* fp=fopen("appendDecompLine.txt", "w");
  XMLGen::append_decomp_lines_to_mpirun_launch_script(tInputData, fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("decomp-p5dummy_mesh.exodecomp-p10dummy_mesh.exodecomp-p10dummy_frf_file.exo");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

}
