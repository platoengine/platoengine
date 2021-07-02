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
    EXPECT_THROW(XMLGen::Internal::compute_number_of_nodes_needed(10,0),std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ComputeNumberOfNodesNeeded)
{
    size_t tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(10,1);
    EXPECT_EQ(tNumNodesNeeded,10u);
    tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(10,2);
    EXPECT_EQ(tNumNodesNeeded,5u);
    tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(10,3);
    EXPECT_EQ(tNumNodesNeeded,4u);
    tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(10,4);
    EXPECT_EQ(tNumNodesNeeded,3u);
    tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(10,5);
    EXPECT_EQ(tNumNodesNeeded,2u);
    tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(10,10);
    EXPECT_EQ(tNumNodesNeeded,1u);
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
  std::string performerName = "analyze";
  XMLGen::generate_jsrun_script(12, performerName);
  
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

TEST(PlatoTestXMLGenerator, getNumOptProcs_numProcsNotSpecified)
{
  XMLGen::InputData tInputData;
  std::string num_opt_processors = XMLGen::Internal::get_num_opt_processors(tInputData);
  EXPECT_STREQ("1", num_opt_processors.c_str());
}

TEST(PlatoTestXMLGenerator, getNumOptProcs_numProcsSpecified)
{
  XMLGen::InputData tInputData;
  XMLGen::Service tService;
  tService.numberProcessors("10");
  tService.code("platomain");
  tInputData.append(tService);
  std::string num_opt_processors = XMLGen::Internal::get_num_opt_processors(tInputData);
  EXPECT_STREQ("10", num_opt_processors.c_str());
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
  tInputData.mesh.run_name = "dummy_mesh.exo";
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("initial_guess_file_name", "dummy_initial_guess.exo");
  tInputData.set(tOptimizationParameters);
  XMLGen::Service tService;
  tService.numberProcessors("10");
  tService.code("platomain");
  tInputData.append(tService);
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
  XMLGen::Service tService;
  tService.numberProcessors("10");
  tService.code("platomain");
  tInputData.append(tService);
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
  XMLGen::Service tService;
  tService.numberProcessors("0");
  tService.code("platomain");
  tInputData.append(tService);
  tInputData.mesh.run_name = "dummy_mesh.exo";
  FILE* fp=fopen("appendDecompLine.txt", "w");
  std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;
  EXPECT_THROW(XMLGen::append_decomp_lines_for_optimizer(tInputData, fp,
                  hasBeenDecompedForThisNumberOfProcessors), std::runtime_error);

  std::vector<XMLGen::Service> tEmptyServiceVector;

  tInputData.set(tEmptyServiceVector);
  tService.numberProcessors("-10");
  tInputData.append(tService);
  EXPECT_THROW(XMLGen::append_decomp_lines_for_optimizer(tInputData, fp,
                  hasBeenDecompedForThisNumberOfProcessors), std::runtime_error);

  tInputData.set(tEmptyServiceVector);
  tService.numberProcessors("mmm food");
  tInputData.append(tService);
  EXPECT_THROW(XMLGen::append_decomp_lines_for_optimizer(tInputData, fp,
                  hasBeenDecompedForThisNumberOfProcessors), std::runtime_error);

  fclose(fp);

  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForOptimizer_emptyInitialGuess)
{
  XMLGen::InputData tInputData;
  tInputData.mesh.run_name = "dummy_mesh.exo";
  XMLGen::Service tService;
  tService.numberProcessors("10");
  tService.code("platomain");
  tInputData.append(tService);
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
  XMLGen::Service tService;
  tService.numberProcessors("1");
  tService.code("platomain");
  tInputData.append(tService);
  tInputData.mesh.run_name = "dummy_mesh.exo";
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
  tInputData.mesh.run_name = "dummy_mesh.exo";
  XMLGen::Service tService;
  tService.numberProcessors("10");
  tService.code("platomain");
  tInputData.append(tService);
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
  tInputData.mesh.run_name = "dummy_mesh.exo";

  XMLGen::Service tService;
  tService.id("1");
  tService.numberProcessors("10");
  tInputData.append(tService);

  XMLGen::Scenario tScenario;
  tScenario.id("1");
  tScenario.append("ref_data_file", "dummy_frf_file.exo");
  tInputData.append(tScenario);

  XMLGen::Objective tObjective;
  tObjective.serviceIDs.push_back("1");
  tObjective.scenarioIDs.push_back("1");
  tInputData.objective = tObjective;
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
  tInputData.mesh.run_name = "dummy_mesh.exo";
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
  tInputData.mesh.run_name = "dummy_mesh.exo";
  XMLGen::Service tService;
  tService.id("1");
  tService.numberProcessors("1");
  tInputData.append(tService);

  XMLGen::Scenario tScenario;
  tScenario.id("1");
  tScenario.append("ref_data_file", "dummy_frf_file.exo");
  tInputData.append(tScenario);

  XMLGen::Objective tObjective;
  tObjective.serviceIDs.push_back("1");
  tObjective.scenarioIDs.push_back("1");
  tInputData.objective = tObjective;

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
  tInputData.mesh.run_name = "dummy_mesh.exo";

  XMLGen::Service tService;
  tService.id("1");
  tService.numberProcessors("10");
  tInputData.append(tService);

  XMLGen::Service tService2;
  tService2.id("2");
  tService2.numberProcessors("10");
  tInputData.append(tService2);

  XMLGen::Scenario tScenario;
  tScenario.id("1");
  tScenario.append("ref_data_file", "dummy_frf_file.exo");
  tInputData.append(tScenario);

  XMLGen::Scenario tScenario2;
  tScenario2.id("2");
  tScenario2.append("ref_data_file", "dummy_frf_file2.exo");
  tInputData.append(tScenario2);

  XMLGen::Objective tObjective;
  tObjective.serviceIDs.push_back("1");
  tObjective.serviceIDs.push_back("2");
  tObjective.scenarioIDs.push_back("1");
  tObjective.scenarioIDs.push_back("2");
  tInputData.objective = tObjective;

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
  tInputData.mesh.run_name = "dummy_mesh.exo";

  XMLGen::Service tService;
  tService.id("1");
  tService.numberProcessors("10");
  tInputData.append(tService);

  XMLGen::Scenario tScenario;
  tScenario.id("1");
  tScenario.append("ref_data_file", "dummy_frf_file.exo");
  tInputData.append(tScenario);

  XMLGen::Objective tObjective;
  tObjective.serviceIDs.push_back("1");
  tObjective.scenarioIDs.push_back("1");
  tInputData.objective = tObjective;

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
  tInputData.mesh.run_name = "dummy_mesh.exo";

  XMLGen::Service tService;
  tService.id("1");
  tService.code("platomain");
  tService.numberProcessors("5");
  tInputData.append(tService);

  XMLGen::Service tService2;
  tService2.id("2");
  tService.code("plato_analyze");
  tService2.numberProcessors("10");
  tInputData.append(tService2);

  XMLGen::Scenario tScenario;
  tScenario.id("1");
  tScenario.append("ref_data_file", "dummy_frf_file.exo");
  tInputData.append(tScenario);

  XMLGen::Objective tObjective;
  tObjective.serviceIDs.push_back("2");
  tObjective.scenarioIDs.push_back("1");
  tInputData.objective = tObjective;

  FILE* fp=fopen("appendDecompLine.txt", "w");
  XMLGen::append_decomp_lines_to_mpirun_launch_script(tInputData, fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendDecompLine.txt");
  auto tGold = std::string("decomp-p5dummy_mesh.exodecomp-p10dummy_mesh.exodecomp-p10dummy_frf_file.exo");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendDecompLine.txt");
}

TEST(PlatoTestXMLGenerator, appendEngineMPIRunLines)
{
  XMLGen::InputData tInputData;
  tInputData.m_UseLaunch = false;
  tInputData.mesh.run_name = "dummy_mesh.exo";
  XMLGen::Service tService;
  tService.numberProcessors("10");
  tService.code("platomain");
  tInputData.append(tService);
  FILE* fp=fopen("appendEngineMPIRunLines.txt", "w");
  int tPerformerID = 0;
  XMLGen::append_engine_mpirun_lines(tInputData, tPerformerID, fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendEngineMPIRunLines.txt");
  auto tGold = std::string("mpiexec-np10-xPLATO_PERFORMER_ID=0\\-xPLATO_INTERFACE_FILE=interface.xml\\-xPLATO_APP_FILE=plato_main_operations.xml\\PlatoMainplato_main_input_deck.xml\\");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendEngineMPIRunLines.txt");
}

TEST(PlatoTestXMLGenerator, getNumBufferLayers)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("number_buffer_layers", "2");
  tInputData.set(tOptimizationParameters);

  std::string number_buffer_layers = XMLGen::Internal::get_num_buffer_layers(tInputData);
  EXPECT_STREQ(number_buffer_layers.c_str(), tInputData.optimization_parameters().number_buffer_layers().c_str());
}

TEST(PlatoTestXMLGenerator, getNumBufferLayers_invalidInput)
{
  XMLGen::InputData tInputData;

  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("number_buffer_layers", "foo");
  tInputData.set(tOptimizationParameters);
  EXPECT_THROW(XMLGen::Internal::get_num_buffer_layers(tInputData),std::runtime_error);

  tOptimizationParameters.append("number_buffer_layers", "-10");
  tInputData.set(tOptimizationParameters);
  EXPECT_THROW(XMLGen::Internal::get_num_buffer_layers(tInputData),std::runtime_error);

  tOptimizationParameters.append("number_buffer_layers", "12.2");
  tInputData.set(tOptimizationParameters);
  EXPECT_THROW(XMLGen::Internal::get_num_buffer_layers(tInputData),std::runtime_error);
}

TEST(PlatoTestXMLGenerator, getNumBufferLayers_emptyInput)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("number_buffer_layers", "");
  tInputData.set(tOptimizationParameters);

  std::string number_buffer_layers = XMLGen::Internal::get_num_buffer_layers(tInputData);
  EXPECT_STREQ(number_buffer_layers.c_str(), "2");
}

TEST(PlatoTestXMLGenerator, getPruneString)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;

  tOptimizationParameters.append("prune_mesh", "true");
  tInputData.set(tOptimizationParameters);
  std::string tPruneString = XMLGen::Internal::get_prune_string(tInputData);
  EXPECT_STREQ(tPruneString.c_str(),"1");

  tOptimizationParameters.append("prune_mesh", "false");
  tInputData.set(tOptimizationParameters);
  tPruneString = XMLGen::Internal::get_prune_string(tInputData);
  EXPECT_STREQ(tPruneString.c_str(),"0");

  tOptimizationParameters.append("prune_mesh", "some_invalid_string");
  tInputData.set(tOptimizationParameters);
  tPruneString = XMLGen::Internal::get_prune_string(tInputData);
  EXPECT_STREQ(tPruneString.c_str(),"0");
}

TEST(PlatoTestXMLGenerator, appendConcatenateMeshFileLines)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("number_prune_and_refine_processors", "5");
  tInputData.set(tOptimizationParameters);
  tInputData.mesh.run_name = "dummy.exo";
  FILE* fp = fopen("concatenate.txt", "w");

  XMLGen::append_concatenate_mesh_file_lines(tInputData,fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("concatenate.txt");
  std::string tGold = "epu-autodummy.exo.5.0";

  EXPECT_STREQ(tReadData.str().c_str(), tGold.c_str());

  Plato::system("rm -rf concatenate.txt");
}

TEST(PlatoTestXMLGenerator, appendConcatenateMeshFileLines_runMeshNameNotSet)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("number_prune_and_refine_processors", "5");
  tInputData.set(tOptimizationParameters);
  FILE* fp = fopen("concatenate.txt", "w");

  EXPECT_THROW(XMLGen::append_concatenate_mesh_file_lines(tInputData, fp),std::runtime_error);
  fclose(fp);
  Plato::system("rm -rf concatenate.txt");
}

TEST(PlatoTestXMLGenerator, getExtensionString)
{
  std::string tNumberPruneAndRefineProcsString = "10";
  std::string tExtensionString = XMLGen::Internal::get_extension_string(tNumberPruneAndRefineProcsString);

  std::string tGold = ".10.00";
  EXPECT_STREQ(tExtensionString.c_str(), tGold.c_str());

  tNumberPruneAndRefineProcsString = "5";
  tExtensionString = XMLGen::Internal::get_extension_string(tNumberPruneAndRefineProcsString);
  tGold = ".5.0";
  EXPECT_STREQ(tExtensionString.c_str(), tGold.c_str());

  tNumberPruneAndRefineProcsString = "245";
  tExtensionString = XMLGen::Internal::get_extension_string(tNumberPruneAndRefineProcsString);
  tGold = ".245.000";
  EXPECT_STREQ(tExtensionString.c_str(), tGold.c_str());
}

TEST(PlatoTestXMLGenerator, getNumberOfRefines_invalidInput)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;

  tOptimizationParameters.append("number_refines", "foo");
  tInputData.set(tOptimizationParameters);
  EXPECT_THROW(XMLGen::Internal::get_number_of_refines(tInputData),std::runtime_error);

  tOptimizationParameters.append("number_refines", "-10");
  tInputData.set(tOptimizationParameters);
  EXPECT_THROW(XMLGen::Internal::get_number_of_refines(tInputData),std::runtime_error);

  tOptimizationParameters.append("number_refines", "12.2");
  tInputData.set(tOptimizationParameters);
  EXPECT_THROW(XMLGen::Internal::get_number_of_refines(tInputData),std::runtime_error);
}

TEST(PlatoTestXMLGenerator, getNumberOfRefines)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("number_refines", "");
  tInputData.set(tOptimizationParameters);
  int tNumRefines = XMLGen::Internal::get_number_of_refines(tInputData);
  EXPECT_EQ(tNumRefines, 0);

  tOptimizationParameters.append("number_refines", "2");
  tInputData.set(tOptimizationParameters);
  tNumRefines = XMLGen::Internal::get_number_of_refines(tInputData);
  EXPECT_EQ(tNumRefines, 2);
}

TEST(PlatoTestXMLGenerator, getMaxNumberOfObjectiveProcs)
{
  XMLGen::InputData tInputData;

  XMLGen::Service tService1;
  tService1.numberProcessors("2");
  tService1.id("1");
  tInputData.append(tService1);
  XMLGen::Service tService2;
  tService2.numberProcessors("4");
  tService2.id("2");
  tInputData.append(tService2);

  tInputData.objective.serviceIDs.push_back("1");
  tInputData.objective.serviceIDs.push_back("2");

  int tMaxNumProcs = XMLGen::Internal::get_max_number_of_objective_procs(tInputData);

  EXPECT_EQ(tMaxNumProcs,4);

  std::vector<XMLGen::Service> tEmptyServiceVector;

  tInputData.set(tEmptyServiceVector);
  tService1.numberProcessors("");
  tService2.numberProcessors("");
  tInputData.append(tService1);
  tInputData.append(tService2);
  tMaxNumProcs = XMLGen::Internal::get_max_number_of_objective_procs(tInputData);
  EXPECT_EQ(tMaxNumProcs,0);

  tInputData.set(tEmptyServiceVector);
  tService1.numberProcessors("3");
  tService2.numberProcessors("1");
  tInputData.append(tService1);
  tInputData.append(tService2);
  tMaxNumProcs = XMLGen::Internal::get_max_number_of_objective_procs(tInputData);
  EXPECT_EQ(tMaxNumProcs,3);

  tInputData.set(tEmptyServiceVector);
  tService1.numberProcessors("3");
  tService2.numberProcessors("");
  tInputData.append(tService1);
  tInputData.append(tService2);
  tMaxNumProcs = XMLGen::Internal::get_max_number_of_objective_procs(tInputData);
  EXPECT_EQ(tMaxNumProcs,3);
}

TEST(PlatoTestXMLGenerator, getNumberOfPruneAndRefineProcs)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("number_prune_and_refine_processors", "3");
  tInputData.set(tOptimizationParameters);

  int tNumProcs = XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData);
  EXPECT_EQ(tNumProcs,3);

  tOptimizationParameters.append("number_prune_and_refine_processors", "");
  tInputData.set(tOptimizationParameters);
  tNumProcs = XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData);
  EXPECT_EQ(tNumProcs,0);

  tOptimizationParameters.append("number_prune_and_refine_processors", "0");
  tInputData.set(tOptimizationParameters);
  tNumProcs = XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData);
  EXPECT_EQ(tNumProcs,0);
}

TEST(PlatoTestXMLGenerator, getNumberOfPruneAndRefineProcs_invalidInput)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("number_prune_and_refine_processors", "foo");
  tInputData.set(tOptimizationParameters);
  EXPECT_THROW(XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData),std::runtime_error);

  tOptimizationParameters.append("number_prune_and_refine_processors", "-10");
  tInputData.set(tOptimizationParameters);
  EXPECT_THROW(XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData),std::runtime_error);

  tOptimizationParameters.append("number_prune_and_refine_processors", "12.2");
  tInputData.set(tOptimizationParameters);
  EXPECT_THROW(XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData),std::runtime_error);
}

TEST(PlatoTestXMLGenerator, appendPruneAndRefineCommand)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("prune_mesh", "true");
  tOptimizationParameters.append("number_refines", "2");
  tOptimizationParameters.append("number_buffer_layers", "2");
  tOptimizationParameters.append("number_prune_and_refine_processors", "10");
  tOptimizationParameters.append("initial_guess_file_name", "dummy_guess.exo");
  tOptimizationParameters.append("initial_guess_field_name", "badGuess");
  tInputData.set(tOptimizationParameters);
  tInputData.codepaths.prune_and_refine_path = "path/to/some/executable";
  tInputData.mesh.name = "dummy.exo";
  tInputData.mesh.run_name = "output.exo";
  tInputData.m_UseLaunch = false;
  FILE* fp = fopen("command.txt", "w");
  XMLGen::append_prune_and_refine_command(tInputData, fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("command.txt");
  std::string tGold = std::string("mpiexec-np10path/to/some/executable--mesh_with_variable=dummy_guess.exo") + 
      std::string("--mesh_to_be_pruned=dummy.exo--result_mesh=output.exo--field_name=badGuess") + 
      std::string("--number_of_refines=2--number_of_buffer_layers=2--prune_mesh=1");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());


  tOptimizationParameters.append("prune_mesh", "false");
  tOptimizationParameters.append("number_refines", "");
  tOptimizationParameters.append("number_buffer_layers", "");
  tOptimizationParameters.append("number_prune_and_refine_processors", "");
  tOptimizationParameters.append("initial_guess_file_name", "");
  tOptimizationParameters.append("initial_guess_field_name", "");
  tInputData.set(tOptimizationParameters);
  tInputData.codepaths.prune_and_refine_path = "";

  tInputData.mesh.name = "dummy.exo";
  tInputData.mesh.run_name = "output.exo";
  fp = fopen("command.txt", "w");
  XMLGen::append_prune_and_refine_command(tInputData, fp);
  fclose(fp);

  tReadData = XMLGen::read_data_from_file("command.txt");
  tGold = std::string("mpiexec-np0prune_and_refine--mesh_to_be_pruned=dummy.exo--result_mesh=output.exo") + 
          std::string("--number_of_refines=0--number_of_buffer_layers=2--prune_mesh=0");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf command.txt");
}

TEST(PlatoTestXMLGenerator, appendPruneAndRefineCommand_invalidInput)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("prune_mesh", "true");
  tOptimizationParameters.append("number_refines", "2");
  tOptimizationParameters.append("number_buffer_layers", "2");
  tOptimizationParameters.append("number_prune_and_refine_processors", "10");
  tOptimizationParameters.append("initial_guess_file_name", "dummy_guess.exo");
  tOptimizationParameters.append("initial_guess_field_name", "badGuess");
  tInputData.set(tOptimizationParameters);
  tInputData.codepaths.prune_and_refine_path = "path/to/some/executable";
  tInputData.mesh.name = "";
  tInputData.mesh.run_name = "output.exo";

  FILE* fp = fopen("command.txt", "w");
  EXPECT_THROW(XMLGen::append_prune_and_refine_command(tInputData, fp),std::runtime_error);
  fclose(fp);

  tInputData.mesh.name = "dummy.exo";
  tInputData.mesh.run_name = "";

  fp = fopen("command.txt", "w");
  EXPECT_THROW(XMLGen::append_prune_and_refine_command(tInputData, fp),std::runtime_error);
  fclose(fp);

  Plato::system("rm -rf command.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForPruneAndRefine)
{
  //no need to prune, transfer, or refine
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("number_refines", "0");
  tOptimizationParameters.append("number_prune_and_refine_processors", "2");
  tInputData.set(tOptimizationParameters);
  tInputData.mesh.name = "dummy.exo";
  FILE* fp = fopen("decomp.txt", "w");
  XMLGen::append_decomp_lines_for_prune_and_refine(tInputData, fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("decomp.txt");
  std::string tGold = "";

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());


  //no need to decompose
  tOptimizationParameters.append("number_refines", "2");
  tOptimizationParameters.append("number_prune_and_refine_processors", "1");
  tInputData.set(tOptimizationParameters);
  tInputData.mesh.name = "dummy.exo";
  fp = fopen("decomp.txt", "w");
  XMLGen::append_decomp_lines_for_prune_and_refine(tInputData, fp);
  fclose(fp);

  tReadData = XMLGen::read_data_from_file("decomp.txt");
  tGold = "";
  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());

  tOptimizationParameters.append("number_refines", "0");
  tOptimizationParameters.append("number_prune_and_refine_processors", "2");
  tOptimizationParameters.append("initial_guess_file_name", "dummy_guess.exo");
  tOptimizationParameters.append("initial_guess_field_name", "badGuess");
  tOptimizationParameters.isARestartRun(true);
  tInputData.set(tOptimizationParameters);
  tInputData.mesh.name = "dummy.exo";
  fp = fopen("decomp.txt", "w");
  XMLGen::append_decomp_lines_for_prune_and_refine(tInputData, fp);
  fclose(fp);

  tReadData = XMLGen::read_data_from_file("decomp.txt");
  tGold = "decomp-p2dummy.exodecomp-p2dummy_guess.exo";
  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());


  tOptimizationParameters.append("number_refines", "2");
  tOptimizationParameters.append("number_prune_and_refine_processors", "2");
  tOptimizationParameters.append("initial_guess_file_name", "");
  tOptimizationParameters.append("initial_guess_field_name", "");
  tOptimizationParameters.isARestartRun(true);
  tInputData.set(tOptimizationParameters);
  tInputData.mesh.name = "dummy.exo";
  fp = fopen("decomp.txt", "w");
  XMLGen::append_decomp_lines_for_prune_and_refine(tInputData, fp);
  fclose(fp);

  tReadData = XMLGen::read_data_from_file("decomp.txt");
  tGold = "decomp-p2dummy.exo";
  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());

  Plato::system("rm -rf decomp.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForPruneAndRefine_missingMeshName)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("number_refines", "2");
  tOptimizationParameters.append("number_prune_and_refine_processors", "2");
  tOptimizationParameters.isARestartRun(true);
  tInputData.set(tOptimizationParameters);
  FILE* fp = fopen("decomp.txt", "w");
  EXPECT_THROW(XMLGen::append_decomp_lines_for_prune_and_refine(tInputData, fp),std::runtime_error);
  fclose(fp);

  Plato::system("rm -rf decomp.txt");
}

TEST(PlatoTestXMLGenerator, appendPruneAndRefineLinesToMPIRunLaunchScript)
{
  XMLGen::InputData tInputData;
  XMLGen::OptimizationParameters tOptimizationParameters;
  tOptimizationParameters.append("prune_mesh", "true");
  tOptimizationParameters.append("number_refines", "2");
  tOptimizationParameters.append("number_buffer_layers", "2");
  tOptimizationParameters.append("number_prune_and_refine_processors", "10");
  tOptimizationParameters.append("initial_guess_file_name", "dummy_guess.exo");
  tOptimizationParameters.append("initial_guess_field_name", "badGuess");
  tOptimizationParameters.isARestartRun(true);
  tInputData.set(tOptimizationParameters);
  tInputData.codepaths.prune_and_refine_path = "path/to/some/executable";
  tInputData.mesh.name = "dummy.exo";
  tInputData.mesh.run_name = "output.exo";
  tInputData.m_UseLaunch = false;
  FILE* fp = fopen("pruneAndRefine.txt", "w");
  XMLGen::append_prune_and_refine_lines_to_mpirun_launch_script(tInputData, fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("pruneAndRefine.txt");
  std::string tGold = std::string("mpiexec-np10path/to/some/executable--mesh_with_variable=dummy_guess.exo") + 
      std::string("--mesh_to_be_pruned=dummy.exo--result_mesh=output.exo--field_name=badGuess") + 
      std::string("--number_of_refines=2--number_of_buffer_layers=2--prune_mesh=1") +
      std::string("epu-autooutput.exo.10.00");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());

  Plato::system("rm -rf pruneAndRefine.txt");
}

}
