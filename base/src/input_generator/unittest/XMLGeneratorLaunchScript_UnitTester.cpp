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
    EXPECT_EQ(tNumNodesNeeded,10);
    tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(10,2);
    EXPECT_EQ(tNumNodesNeeded,5);
    tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(10,3);
    EXPECT_EQ(tNumNodesNeeded,4);
    tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(10,4);
    EXPECT_EQ(tNumNodesNeeded,3);
    tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(10,5);
    EXPECT_EQ(tNumNodesNeeded,2);
    tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(10,10);
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
  tInputData.num_opt_processors = "10";
  std::string num_opt_processors = XMLGen::Internal::get_num_opt_processors(tInputData);
  EXPECT_STREQ("10", num_opt_processors.c_str());
}

TEST(PlatoTestXMLGenerator, getNumProcs_numProcsNotSpecified)
{
  XMLGen::Objective tObjective;
  std::string num_procs = XMLGen::Internal::get_num_procs(tObjective);
  EXPECT_STREQ("4", num_procs.c_str());
}

TEST(PlatoTestXMLGenerator, getNumProcs_numProcsSpecified)
{
  XMLGen::Objective tObjective;
  tObjective.num_procs = "10";
  std::string num_procs = XMLGen::Internal::get_num_procs(tObjective);
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

TEST(PlatoTestXMLGenerator, appendEngineMPIRunLines)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy_mesh.exo";
  tInputData.num_opt_processors = "10";
  FILE* fp=fopen("appendEngineMPIRunLines.txt", "w");
  XMLGen::append_engine_mpirun_lines(tInputData, fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("appendEngineMPIRunLines.txt");
  auto tGold = std::string("mpiexec-np10-xPLATO_PERFORMER_ID=0\\-xPLATO_INTERFACE_FILE=interface.xml\\-xPLATO_APP_FILE=plato_main_operations.xml\\plato_mainplato_main_input_deck.xml\\");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf appendEngineMPIRunLines.txt");
}

TEST(PlatoTestXMLGenerator, getNumBufferLayers)
{
  XMLGen::InputData tInputData;
  tInputData.number_buffer_layers = "2";

  std::string number_buffer_layers = XMLGen::Internal::get_num_buffer_layers(tInputData);
  EXPECT_STREQ(number_buffer_layers.c_str(), tInputData.number_buffer_layers.c_str());
}

TEST(PlatoTestXMLGenerator, getNumBufferLayers_invalidInput)
{
  XMLGen::InputData tInputData;

  tInputData.number_buffer_layers = "foo";
  EXPECT_THROW(XMLGen::Internal::get_num_buffer_layers(tInputData),std::runtime_error);

  tInputData.number_buffer_layers = "-10";
  EXPECT_THROW(XMLGen::Internal::get_num_buffer_layers(tInputData),std::runtime_error);

  tInputData.number_buffer_layers = "12.2";
  EXPECT_THROW(XMLGen::Internal::get_num_buffer_layers(tInputData),std::runtime_error);
}

TEST(PlatoTestXMLGenerator, getNumBufferLayers_emptyInput)
{
  XMLGen::InputData tInputData;
  tInputData.number_buffer_layers = "";

  std::string number_buffer_layers = XMLGen::Internal::get_num_buffer_layers(tInputData);
  EXPECT_STREQ(number_buffer_layers.c_str(), "2");
}

TEST(PlatoTestXMLGenerator, getPruneString)
{
  XMLGen::InputData tInputData;

  tInputData.prune_mesh = "true";
  std::string tPruneString = XMLGen::Internal::get_prune_string(tInputData);
  EXPECT_STREQ(tPruneString.c_str(),"1");

  tInputData.prune_mesh = "false";
  tPruneString = XMLGen::Internal::get_prune_string(tInputData);
  EXPECT_STREQ(tPruneString.c_str(),"0");

  tInputData.prune_mesh = "some_invalid_string";
  tPruneString = XMLGen::Internal::get_prune_string(tInputData);
  EXPECT_STREQ(tPruneString.c_str(),"0");
}

TEST(PlatoTestXMLGenerator, appendConcatenateMeshFileLines)
{
  XMLGen::InputData tInputData;
  tInputData.run_mesh_name = "dummy.exo";
  tInputData.number_prune_and_refine_processors = "5";
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
  tInputData.number_prune_and_refine_processors = "5";
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

  tInputData.number_refines = "foo";
  EXPECT_THROW(XMLGen::Internal::get_number_of_refines(tInputData),std::runtime_error);

  tInputData.number_refines = "-10";
  EXPECT_THROW(XMLGen::Internal::get_number_of_refines(tInputData),std::runtime_error);

  tInputData.number_refines = "12.2";
  EXPECT_THROW(XMLGen::Internal::get_number_of_refines(tInputData),std::runtime_error);
}

TEST(PlatoTestXMLGenerator, getNumberOfRefines)
{
  XMLGen::InputData tInputData;
  tInputData.number_refines = "";
  int tNumRefines = XMLGen::Internal::get_number_of_refines(tInputData);
  EXPECT_EQ(tNumRefines, 0);

  tInputData.number_refines = "2";
  tNumRefines = XMLGen::Internal::get_number_of_refines(tInputData);
  EXPECT_EQ(tNumRefines, 2);
}

TEST(PlatoTestXMLGenerator, getMaxNumberOfObjectiveProcs)
{
  XMLGen::InputData tInputData;
  XMLGen::Objective tObjective1;
  XMLGen::Objective tObjective2;
  tObjective1.num_procs = "2";
  tObjective2.num_procs = "4";
  tInputData.objectives.push_back(tObjective1);
  tInputData.objectives.push_back(tObjective2);
  int tMaxNumProcs = XMLGen::Internal::get_max_number_of_objective_procs(tInputData);

  EXPECT_EQ(tMaxNumProcs,4);

  tInputData.objectives[0].num_procs = "";
  tInputData.objectives[1].num_procs = "";
  tMaxNumProcs = XMLGen::Internal::get_max_number_of_objective_procs(tInputData);
  EXPECT_EQ(tMaxNumProcs,1);

  tInputData.objectives[0].num_procs = "3";
  tInputData.objectives[1].num_procs = "1";
  tMaxNumProcs = XMLGen::Internal::get_max_number_of_objective_procs(tInputData);
  EXPECT_EQ(tMaxNumProcs,3);

  tInputData.objectives[0].num_procs = "3";
  tInputData.objectives[1].num_procs = "";
  tMaxNumProcs = XMLGen::Internal::get_max_number_of_objective_procs(tInputData);
  EXPECT_EQ(tMaxNumProcs,3);
}

TEST(PlatoTestXMLGenerator, getNumberOfPruneAndRefineProcs)
{
  XMLGen::InputData tInputData;
  XMLGen::Objective tObjective1;
  XMLGen::Objective tObjective2;
  tObjective1.num_procs = "2";
  tObjective2.num_procs = "4";
  tInputData.objectives.push_back(tObjective1);
  tInputData.objectives.push_back(tObjective2);
  tInputData.number_prune_and_refine_processors = "3";

  int tNumProcs = XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData);
  EXPECT_EQ(tNumProcs,3);

  tInputData.number_prune_and_refine_processors = "";
  tNumProcs = XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData);
  EXPECT_EQ(tNumProcs,4);

  tInputData.number_prune_and_refine_processors = "0";
  tNumProcs = XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData);
  EXPECT_EQ(tNumProcs,4);
}

TEST(PlatoTestXMLGenerator, getNumberOfPruneAndRefineProcs_invalidInput)
{
  XMLGen::InputData tInputData;
  XMLGen::Objective tObjective1;
  XMLGen::Objective tObjective2;
  tObjective1.num_procs = "2";
  tObjective2.num_procs = "4";
  tInputData.objectives.push_back(tObjective1);
  tInputData.objectives.push_back(tObjective2);

  tInputData.number_prune_and_refine_processors = "foo";
  EXPECT_THROW(XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData),std::runtime_error);

  tInputData.number_prune_and_refine_processors = "-10";
  EXPECT_THROW(XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData),std::runtime_error);

  tInputData.number_prune_and_refine_processors = "12.2";
  EXPECT_THROW(XMLGen::Internal::get_number_of_prune_and_refine_procs(tInputData),std::runtime_error);
}

TEST(PlatoTestXMLGenerator, appendPruneAndRefineCommand)
{
  XMLGen::InputData tInputData;
  tInputData.prune_mesh = "true";
  tInputData.number_refines = "2";
  tInputData.number_buffer_layers = "2";
  tInputData.number_prune_and_refine_processors = "10";
  tInputData.prune_and_refine_path = "path/to/some/executable";
  tInputData.initial_guess_filename = "dummy_guess.exo";
  tInputData.initial_guess_field_name = "badGuess";
  tInputData.mesh_name = "dummy.exo";
  tInputData.run_mesh_name = "output.exo";
  FILE* fp = fopen("command.txt", "w");
  XMLGen::append_prune_and_refine_command(tInputData, fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("command.txt");
  std::string tGold = std::string("mpiexec-np10path/to/some/executable--mesh_with_variable=dummy_guess.exo") + 
      std::string("--mesh_to_be_pruned=dummy.exo--result_mesh=output.exo--field_name=badGuess") + 
      std::string("--number_of_refines=2--number_of_buffer_layers=2--prune_mesh=1");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());


  tInputData.prune_mesh = "false";
  tInputData.number_refines = "";
  tInputData.number_buffer_layers = "";
  tInputData.number_prune_and_refine_processors = "";
  tInputData.prune_and_refine_path = "";
  tInputData.initial_guess_filename = "";
  tInputData.initial_guess_field_name = "";

  tInputData.mesh_name = "dummy.exo";
  tInputData.run_mesh_name = "output.exo";
  fp = fopen("command.txt", "w");
  XMLGen::append_prune_and_refine_command(tInputData, fp);
  fclose(fp);

  tReadData = XMLGen::read_data_from_file("command.txt");
  tGold = std::string("mpiexec-np1prune_and_refine--mesh_to_be_pruned=dummy.exo--result_mesh=output.exo") + 
          std::string("--number_of_refines=0--number_of_buffer_layers=2--prune_mesh=0");

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());
  Plato::system("rm -rf command.txt");
}

TEST(PlatoTestXMLGenerator, appendPruneAndRefineCommand_invalidInput)
{
  XMLGen::InputData tInputData;
  tInputData.prune_mesh = "true";
  tInputData.number_refines = "2";
  tInputData.number_buffer_layers = "2";
  tInputData.number_prune_and_refine_processors = "10";
  tInputData.prune_and_refine_path = "path/to/some/executable";
  tInputData.initial_guess_filename = "dummy_guess.exo";
  tInputData.initial_guess_field_name = "badGuess";
  tInputData.mesh_name = "";
  tInputData.run_mesh_name = "output.exo";

  FILE* fp = fopen("command.txt", "w");
  EXPECT_THROW(XMLGen::append_prune_and_refine_command(tInputData, fp),std::runtime_error);
  fclose(fp);

  tInputData.mesh_name = "dummy.exo";
  tInputData.run_mesh_name = "";

  fp = fopen("command.txt", "w");
  EXPECT_THROW(XMLGen::append_prune_and_refine_command(tInputData, fp),std::runtime_error);
  fclose(fp);

  Plato::system("rm -rf command.txt");
}

TEST(PlatoTestXMLGenerator, appendDecompLinesForPruneAndRefine)
{
  //no need to prune, transfer, or refine
  XMLGen::InputData tInputData;
  tInputData.number_refines = "0";
  tInputData.number_prune_and_refine_processors = "2";
  tInputData.mesh_name = "dummy.exo";
  FILE* fp = fopen("decomp.txt", "w");
  XMLGen::append_decomp_lines_for_prune_and_refine(tInputData, fp);
  fclose(fp);

  auto tReadData = XMLGen::read_data_from_file("decomp.txt");
  std::string tGold = "";

  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());


  //no need to decompose
  tInputData.number_refines = "2";
  tInputData.number_prune_and_refine_processors = "1";
  tInputData.mesh_name = "dummy.exo";
  fp = fopen("decomp.txt", "w");
  XMLGen::append_decomp_lines_for_prune_and_refine(tInputData, fp);
  fclose(fp);

  tReadData = XMLGen::read_data_from_file("decomp.txt");
  tGold = "";
  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());

  tInputData.number_refines = "0";
  tInputData.number_prune_and_refine_processors = "2";
  tInputData.mesh_name = "dummy.exo";
  tInputData.initial_guess_filename = "dummy_guess.exo";
  tInputData.initial_guess_field_name = "badGuess";
  fp = fopen("decomp.txt", "w");
  XMLGen::append_decomp_lines_for_prune_and_refine(tInputData, fp);
  fclose(fp);

  tReadData = XMLGen::read_data_from_file("decomp.txt");
  tGold = "decomp-p2dummy.exodecomp-p2dummy_guess.exo";
  EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());


  tInputData.number_refines = "2";
  tInputData.number_prune_and_refine_processors = "2";
  tInputData.mesh_name = "dummy.exo";
  tInputData.initial_guess_filename = "";
  tInputData.initial_guess_field_name = "";
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
  tInputData.number_refines = "2";
  tInputData.number_prune_and_refine_processors = "2";
  FILE* fp = fopen("decomp.txt", "w");
  EXPECT_THROW(XMLGen::append_decomp_lines_for_prune_and_refine(tInputData, fp),std::runtime_error);
  fclose(fp);

  Plato::system("rm -rf decomp.txt");
}

TEST(PlatoTestXMLGenerator, appendPruneAndRefineLinesToMPIRunLaunchScript)
{
  XMLGen::InputData tInputData;
  tInputData.prune_mesh = "true";
  tInputData.number_refines = "2";
  tInputData.number_buffer_layers = "2";
  tInputData.number_prune_and_refine_processors = "10";
  tInputData.prune_and_refine_path = "path/to/some/executable";
  tInputData.initial_guess_filename = "dummy_guess.exo";
  tInputData.initial_guess_field_name = "badGuess";
  tInputData.mesh_name = "dummy.exo";
  tInputData.run_mesh_name = "output.exo";
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
