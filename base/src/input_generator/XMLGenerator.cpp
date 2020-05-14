/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

/*
 * XMLGenerator.cpp
 *
 *  Created on: Sep 28, 2017
 *
 */

#include <set>
#include <string>
#include <iostream>
#include <iostream>
#include <fstream>
#include <cstring>
#include <cassert>
#include <utility>
#include <string>
#include <map>

#include "Plato_SromXML.hpp"
#include "XMLGenerator.hpp"

#include "../optimize/Plato_SromXMLGenTools.hpp"
#include "XMLGeneratorParser.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "Plato_SolveUncertaintyProblem.hpp"
#include "Plato_UniqueCounter.hpp"
#include "Plato_Vector3DVariations.hpp"
#include "PlatoAnalyzeInputDeckWriter.hpp"
#include "SalinasInputDeckWriter.hpp"
#include "Plato_FreeFunctions.hpp"

#include "XMLG_Macros.hpp"
#include "DefaultInputGenerator.hpp"
#include "ComplianceMinTOPlatoAnalyzeInputGenerator.hpp"
#include "ComplianceMinTOPlatoAnalyzeUncertInputGenerator.hpp"

namespace XMLGen
{


const int MAX_CHARS_PER_LINE = 10000;
const int MAX_TOKENS_PER_LINE = 5000;
const char* const DELIMITER = " \t";

void PrintUnrecognizedTokens(const std::vector<std::string> & unrecognizedTokens)
{
       std::cout << "Did not recognize: ";
       for (size_t ind = 0 ; ind < unrecognizedTokens.size(); ++ind)
               std::cout << unrecognizedTokens[ind].c_str() << " " ;
        std::cout << "\n";
}

/******************************************************************************/
XMLGenerator::XMLGenerator(const std::string &input_filename, bool use_launch, const XMLGen::Arch& arch) :
        m_InputFilename(input_filename),
        m_InputData()
/******************************************************************************/
{
  m_InputData.m_UseLaunch = use_launch;
  m_InputData.m_Arch = arch;
  m_InputData.m_filterType_identity_generatorName = "identity";
  m_InputData.m_filterType_identity_XMLName = "Identity";
  m_InputData.m_filterType_kernel_generatorName = "kernel";
  m_InputData.m_filterType_kernel_XMLName = "Kernel";
  m_InputData.m_filterType_kernelThenHeaviside_generatorName = "kernel then heaviside";
  m_InputData.m_filterType_kernelThenHeaviside_XMLName = "KernelThenHeaviside";
  m_InputData.m_filterType_kernelThenTANH_generatorName = "kernel then tanh";
  m_InputData.m_filterType_kernelThenTANH_XMLName = "KernelThenTANH";
  m_InputData.m_HasUncertainties = false;
  m_InputData.m_RequestedVonMisesOutput = false;
  m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow = false;
}

/******************************************************************************/
XMLGenerator::~XMLGenerator() 
/******************************************************************************/
{
}

/******************************************************************************//**
 * \fn wirteInputFiles
 * \brief Write input files, i.e. write all the XML files needed by Plato.
**********************************************************************************/
void XMLGenerator::wirteInputFiles()
{

    if(m_InputData.input_generator_version == "old")
    {
        DefaultInputGenerator tGenerator(m_InputData);
        tGenerator.generateInputFiles();
    }
    else
    {
        ProblemType tProblemType = this->getProblemType();
        switch(tProblemType)
        {
            case COMPLIANCE_MINIMIZATION_TO_PLATO_ANLYZE:
            {
                ComplianceMinTOPlatoAnalyzeInputGenerator tGenerator(m_InputData);
                tGenerator.generateInputFiles();
                break;
            }
            case COMPLIANCE_MINIMIZATION_TO_PLATO_ANLYZE_WITH_UNCERTAINTIES:
            {
                ComplianceMinTOPlatoAnalyzeUncertInputGenerator tGenerator(m_InputData);
                tGenerator.generateInputFiles();
                break;
            }
            default:
            {
                DefaultInputGenerator tGenerator(m_InputData);
                tGenerator.generateInputFiles();
                break;
            }
        }
    }
}

/******************************************************************************/
bool XMLGenerator::generate()
/******************************************************************************/
{
    /////////////////////////////////////////////////
    // Parse input and gather various info
    /////////////////////////////////////////////////
    
    if(!parseFile())
    {
        PRINTERR("Failed to parse input file.")
        return false;
    }

    if(m_InputData.optimization_type == "shape" && m_InputData.csm_filename.length() > 0)
    {
        if(!parseCSMFile())
        {
            PRINTERR("Failed to parse CSM file.")
            return false;
        }
    }

    this->getUncertaintyFlags();

    if(!runSROMForUncertainVariables())
    {
        PRINTERR("Failed to expand uncertainties in file generation.")
        return false;
    }

    // NOTE: modifies objectives to resolves distribution
    if(!distributeObjectivesForGenerate())
    {
        PRINTERR("Failed to distribute objectives in file generation.")
        return false;
    }

    this->lookForPlatoAnalyzePerformers();
    this->wirteInputFiles();

    return true;
}

/******************************************************************************/
ProblemType XMLGenerator::getProblemType()
/******************************************************************************/
{
    ProblemType tProblemType = UNKNOWN;

    // Look for PA problem types.
    tProblemType = identifyPlatoAnalyzeProblemTypes();

    if(tProblemType == UNKNOWN)
    {
        // Look for other recognized problem types...
    }
    
    return tProblemType;
}

/******************************************************************************/
ProblemType XMLGenerator::identifyPlatoAnalyzeProblemTypes()
/******************************************************************************/
{
    ProblemType tProblemType = UNKNOWN;

    if(m_InputData.mAllPerformersArePlatoAnalyze)
    {
        if(m_InputData.optimization_type == "topology")
        {
            if(m_InputData.mAllObjectivesAreComplianceMinimization)
            {
                if(m_InputData.m_HasUncertainties == false)
                    tProblemType = COMPLIANCE_MINIMIZATION_TO_PLATO_ANLYZE;
                else
                    tProblemType = COMPLIANCE_MINIMIZATION_TO_PLATO_ANLYZE_WITH_UNCERTAINTIES;
            }
        }
    }

    return tProblemType;
}

/******************************************************************************/
bool XMLGenerator::runSROMForUncertainVariables()
/******************************************************************************/
{
    if(m_InputData.uncertainties.size() > 0)
    {
        if (m_InputData.objectives.size() > 1)
        {
            std::cout << "ERROR: Only one objective is supported for optimization under uncertainty problem." << std::endl;
            return false;
        }
        if (m_InputData.objectives[0].code_name == "plato_analyze" && !m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow)
        {
            if (m_InputData.objectives[0].atmost_total_num_processors < m_InputData.uncertainties[0].num_samples)
            {
                std::cout << "Number of processors must be equal to number of samples " << "with legacy plato analyze uncerainty workflow" << std::endl;
                return false;
            }
        }
        if (m_InputData.objectives[0].weight != "1")
        {
            std::cout << "Objective weight must be equal to 1 for optimization under uncertainty problem." << std::endl;
            return false;
        }

        Plato::srom::InputMetaData tSromInputs;
        Plato::srom::preprocess_srom_problem_inputs(m_InputData, tSromInputs);
        Plato::srom::OutputMetaData tSromOutputs;
        Plato::srom::build_sroms(tSromInputs, tSromOutputs);
        Plato::srom::postprocess_srom_problem_outputs(tSromOutputs, m_InputData);
        Plato::srom::check_output(m_InputData.mRandomMetaData);

        this->setNumPerformers();
    }

    return true;
}

/******************************************************************************/
void XMLGenerator::setNumPerformers()
/******************************************************************************/
{
    if (m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow)
    {
        m_InputData.m_UncertaintyMetaData.numPeformers = std::stoi(m_InputData.objectives[0].num_ranks);
    }
    else
    {
        m_InputData.m_UncertaintyMetaData.numPeformers = std::stoi(m_InputData.objectives[0].atmost_total_num_processors);
    }

    if (m_InputData.mRandomMetaData.numSamples() % m_InputData.m_UncertaintyMetaData.numPeformers != 0)
    {
        THROWERR("Set Number for Performers: Number of samples must divide evenly into number of processors.");
    }
}

/******************************************************************************/
bool XMLGenerator::parseCSMFile()
/******************************************************************************/
{
    bool tRet = true;

    std::ifstream tInputStream;
    tInputStream.open(m_InputData.csm_filename.c_str());
    if(tInputStream.good())
    {
        tRet = parseCSMFileFromStream(tInputStream);
        tInputStream.close();
    }
    else
        tRet = false;

    // Build a tesselation and exodus filenames from the csm filename
    size_t tPosition = m_InputData.csm_filename.find(".csm");
    if(tPosition != std::string::npos)
    {
        m_InputData.csm_tesselation_filename = m_InputData.csm_filename.substr(0, tPosition);
        m_InputData.csm_tesselation_filename += ".eto";
        m_InputData.csm_exodus_filename = m_InputData.csm_filename.substr(0, tPosition);
        m_InputData.csm_exodus_filename += ".exo";
    }
    else
    {
        std::cout << "\n\nError: CSM filename did not have .csm extension\n\n";
        tRet = false;
    }

    return tRet;
}

/******************************************************************************/
bool XMLGenerator::parseCSMFileFromStream(std::istream &aStream)
/******************************************************************************/
{
    bool tRet = true;

    char tBuffer[MAX_CHARS_PER_LINE];

    // read each line of the file (could optimize this to not read the whole file)
    while(!aStream.eof())
    {
        // read an entire line into memory
        aStream.getline(tBuffer, MAX_CHARS_PER_LINE);

        char *tCharPointer = std::strtok(tBuffer, " ");

        // skip comments
        if(tCharPointer && tCharPointer[0] == '#')
            continue;

        if(tCharPointer && std::strcmp(tCharPointer, "despmtr") == 0)
        {
            // Get the variable name
            tCharPointer = std::strtok(0, " ");
            if(!tCharPointer)
                break;
            // Get the variable value
            tCharPointer = std::strtok(0, " ");
            m_InputData.mShapeDesignVariableValues.push_back(tCharPointer);
            m_InputData.num_shape_design_variables++;
        }
    }

    return tRet;
}


/******************************************************************************/
void XMLGenerator::lookForPlatoAnalyzePerformers()
/******************************************************************************/
{
    m_InputData.mPlatoAnalyzePerformerExists = false;
    m_InputData.mAllPerformersArePlatoAnalyze = false;
    size_t tNumPlatoAnalyzePerformers = 0;
    size_t tNumComplianceMinimizationObjectives = 0;
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        if(m_InputData.objectives[i].code_name == "plato_analyze")
        {
            m_InputData.mPlatoAnalyzePerformerExists = true;
            tNumPlatoAnalyzePerformers++;
        }
        if(m_InputData.objectives[i].code_name == "plato_analyze")
        {
            tNumComplianceMinimizationObjectives++;
        }
    }
    if(tNumPlatoAnalyzePerformers == m_InputData.objectives.size())
        m_InputData.mAllPerformersArePlatoAnalyze = true;
    if(tNumComplianceMinimizationObjectives == m_InputData.objectives.size())
        m_InputData.mAllObjectivesAreComplianceMinimization = true;
}

/******************************************************************************/
bool XMLGenerator::distributeObjectivesForGenerate()
/******************************************************************************/
{
    // for each objective, consider if should distribute
    size_t objective_index = 0u;
    if(m_InputData.m_HasUncertainties && m_InputData.objectives.size() != 1)
    {
      std::cout << "Uncertainty problems only support one objective" << std::endl;
      return false;
    }
    while(objective_index < m_InputData.objectives.size())
    {
        const std::string thisObjective_distributeType = m_InputData.objectives[objective_index].distribute_objective_type;
        if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && thisObjective_distributeType != "")
        {
          std::cout << "Objective distribution is not supported with new plato analyze uncertainty workflow" << std::endl;
          return false;
        }

        if(thisObjective_distributeType == "")
        {
            // no distribute; nothing to do for this objective.
        }
        else if(thisObjective_distributeType == "atmost")
        {
            // distribute by "atmost" rule

            // get inputs to distributed
            const size_t total_number_of_tasks = m_InputData.objectives[objective_index].load_case_ids.size();
            const int num_processors_in_group = std::stoi(m_InputData.objectives[objective_index].num_procs);
            const int atmost_processor_count = std::stoi(m_InputData.objectives[objective_index].atmost_total_num_processors);
            if(num_processors_in_group <= 0 || atmost_processor_count <= 0)
            {
                std::cout << "ERROR:XMLGenerator:distributeObjectives: read a non-positive processor count.\n";
                return false;
            }

            // divide up distributed objective
            const size_t num_distributed_objectives = Plato::divide_up_atmost_processors(total_number_of_tasks,
                                                                                         num_processors_in_group,
                                                                                         atmost_processor_count);
            // store original load ids and weights
            const std::vector<std::string> orig_load_case_ids = m_InputData.objectives[objective_index].load_case_ids;
            const std::vector<std::string> orig_load_case_weights = m_InputData.objectives[objective_index].load_case_weights;

            const size_t num_loads_this_original_objective = orig_load_case_ids.size();
            if(num_loads_this_original_objective != orig_load_case_weights.size())
            {
                std::cout << "ERROR:XMLGenerator:distributeObjectives: "
                          << "Found length mismatch between load case ids and weights.\n";
                return false;
            }

            // clear load ids/weights in preparation for re-allocation
            m_InputData.objectives[objective_index].load_case_ids.clear();
            m_InputData.objectives[objective_index].load_case_weights.clear();

            // clear distribute type in preparation for completion
            m_InputData.objectives[objective_index].distribute_objective_type = "";
            m_InputData.objectives[objective_index].atmost_total_num_processors = "";

            // pushback indices for this distributed objective
            std::vector<size_t> fromOriginal_newDistributed_objectiveIndices(1u, objective_index);
            for(size_t distributed_index = 1u; distributed_index < num_distributed_objectives; distributed_index++)
            {
                fromOriginal_newDistributed_objectiveIndices.push_back(m_InputData.objectives.size());
                m_InputData.objectives.push_back(m_InputData.objectives[objective_index]);
            }

            // stride load case ids and weights across distributed objectives
            size_t strided_distributed_index = 0u;
            for(size_t abstract_load_id = 0u; abstract_load_id < num_loads_this_original_objective; abstract_load_id++)
            {
                // resolve strided index to objective index
                const size_t this_distributed_objectiveIndex =
                        fromOriginal_newDistributed_objectiveIndices[strided_distributed_index];

                // transfer ids/weights
                m_InputData.objectives[this_distributed_objectiveIndex].load_case_ids.push_back(orig_load_case_ids[abstract_load_id]);
                m_InputData.objectives[this_distributed_objectiveIndex].load_case_weights.push_back(orig_load_case_weights[abstract_load_id]);

                // remove name in distributed
                m_InputData.objectives[this_distributed_objectiveIndex].name="";
                m_InputData.objectives[this_distributed_objectiveIndex].performer_name="";

                // advance distributed index
                strided_distributed_index = (strided_distributed_index + 1) % num_distributed_objectives;
            }
        }

        // advance considered objective
        objective_index++;
    }

    // assign names
    const bool filling_did_pass = fillObjectiveAndPerfomerNames();
    if(!filling_did_pass)
    {
        std::cout << "ERROR:XMLGenerator:distributeObjectives: Failed to fill objective and performer names.\n";
        return false;
    }

    return true;
}

/******************************************************************************/
bool XMLGenerator::parseTokens(char *buffer, std::vector<std::string> &tokens)
/******************************************************************************/
{
    const char* token[MAX_TOKENS_PER_LINE] = {}; // initialize to 0
    int n = 0;

    // parse the line
    token[0] = strtok(buffer, DELIMITER); // first token

    // If there is a comment...
    if(token[0] && strlen(token[0]) > 1 && token[0][0] == '/' && token[0][1] == '/')
    {
        tokens.clear();
        return true;
    }

    if (token[0]) // zero if line is blank
    {
        for (n = 1; n < MAX_TOKENS_PER_LINE; n++)
        {
            token[n] = strtok(0, DELIMITER); // subsequent tokens
            if (!token[n])
                break; // no more tokens
        }
    }
    for(int i=0; i<n; ++i)
        tokens.push_back(token[i]);

    return true;
}

/******************************************************************************/
bool XMLGenerator::parseObjectives(std::istream &fin)
/******************************************************************************/
{
    std::vector<std::string> tInputStringList;
    char buf[MAX_CHARS_PER_LINE];
    std::vector<std::string> tokens;
    std::string tStringValue;

    // read each line of the file
    while (!fin.eof())
    {
        // read an entire line into memory
        fin.getline(buf, MAX_CHARS_PER_LINE);
        tokens.clear();
        parseTokens(buf, tokens);

        // process the tokens
        if(tokens.size() > 0)
        {
            for(size_t j=0; j<tokens.size(); ++j)
                tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"begin","objective"}, tStringValue))
            {
                XMLGen::Objective new_objective;
                new_objective.weight="1";
                new_objective.normalize_objective = "true";
                // found an objective. parse it.
                // parse the rest of the objective
                while (!fin.eof())
                {
                    tokens.clear();
                    fin.getline(buf, MAX_CHARS_PER_LINE);
                    parseTokens(buf, tokens);
                    // process the tokens
                    if(tokens.size() > 0)
                    {
                        std::vector<std::string> unlowered_tokens = tokens;

                        for(size_t j=0; j<tokens.size(); ++j)
                            tokens[j] = toLower(tokens[j]);

                        if(parseSingleValue(tokens, tInputStringList = {"end","objective"}, tStringValue))
                        {
                            break;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"type"}, tStringValue))
                        {
                            if(tokens.size() < 2)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No type specified after \"type\" keyword.\n";
                                return false;
                            }
                            new_objective.type = tokens[1];
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                new_objective.type += " ";
                                new_objective.type += tokens[j];
                            }
                            if((new_objective.type == "stress constrained mass minimization")  &&
                               new_objective.analysis_solver_tolerance.length() == 0)
                            {
                                // Set a default in case one isn't set by user.
                                new_objective.analysis_solver_tolerance = "1e-7";
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"name"}, tStringValue))
                        {
                            if(tokens.size() < 2)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No name specified after \"name\" keyword.\n";
                                return false;
                            }
                            new_objective.name = tokens[1];
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                new_objective.name += " ";
                                new_objective.name += tokens[j];
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"boundary","condition","ids"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"boundary condition ids\" keywords.\n";
                                return false;
                            }
                            for(size_t j=3; j<tokens.size(); ++j)
                            {
                                new_objective.bc_ids.push_back(tokens[j]);
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"stress","limit"}, tStringValue))
                        {
                            if(tokens.size() < 3)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"stress limit\" keywords.\n";
                                return false;
                            }
                            if(new_objective.relative_stress_limit != "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: Only specify \"relative stress limit\" or \"stress limit\".\n";
                                return false;
                            }
                            new_objective.stress_limit = tokens[2];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"relative","stress","limit"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"relative stress limit\" keywords.\n";
                                return false;
                            }
                            if(new_objective.stress_limit != "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: Only specify \"relative stress limit\" or \"stress limit\".\n";
                                return false;
                            }
                            new_objective.relative_stress_limit = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"scmm", "constraint", "exponent"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"scmm constraint exponent\" keywords.\n";
                                return false;
                            }
                            new_objective.scmm_constraint_exponent = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"scmm", "penalty", "expansion", "factor"}, tStringValue))
                        {
                            if(tokens.size() < 5)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"scmm penalty expansion factor\" keywords.\n";
                                return false;
                            }
                            new_objective.scmm_penalty_expansion_factor = tokens[4];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"scmm", "initial", "penalty"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"scmm initial penalty\" keywords.\n";
                                return false;
                            }
                            new_objective.scmm_initial_penalty = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"stress","p","norm","power"}, tStringValue))
                        {
                            if(tokens.size() < 5)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"stress p norm power\" keywords.\n";
                                return false;
                            }
                            new_objective.stress_p_norm_power = tokens[4];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"volume","misfit","target"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"volume misfit target\" keywords.\n";
                                return false;
                            }
                            new_objective.volume_misfit_target = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"stress","ramp","factor"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"stress ramp factor\" keywords.\n";
                                return false;
                            }
                            new_objective.stress_ramp_factor = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"limit","power","min"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"limit power min\" keywords.\n";
                                return false;
                            }
                            new_objective.limit_power_min = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"limit","power","max"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"limit power max\" keywords.\n";
                                return false;
                            }
                            new_objective.limit_power_max = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"limit","power","feasible","bias"}, tStringValue))
                        {
                            if(tokens.size() < 5)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"limit power feasible bias\" keywords.\n";
                                return false;
                            }
                            new_objective.limit_power_feasible_bias = tokens[4];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"limit","power","feasible","slope"}, tStringValue))
                        {
                            if(tokens.size() < 5)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"limit power feasible slope\" keywords.\n";
                                return false;
                            }
                            new_objective.limit_power_feasible_slope = tokens[4];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"limit","power","infeasible","bias"}, tStringValue))
                        {
                            if(tokens.size() < 5)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"limit power infeasible bias\" keywords.\n";
                                return false;
                            }
                            new_objective.limit_power_infeasible_bias = tokens[4];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"limit","power","infeasible","slope"}, tStringValue))
                        {
                            if(tokens.size() < 5)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"limit power infeasible slope\" keywords.\n";
                                return false;
                            }
                            new_objective.limit_power_infeasible_slope = tokens[4];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"limit","reset","subfrequency"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"limit reset subfrequency\" keywords.\n";
                                return false;
                            }
                            new_objective.limit_reset_subfrequency = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"limit","reset","count"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"limit reset count\" keywords.\n";
                                return false;
                            }
                            new_objective.limit_reset_count = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"inequality","allowable","feasibility","upper"}, tStringValue))
                        {
                            if(tokens.size() < 5)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"inequality allowable feasibility upper\" keywords.\n";
                                return false;
                            }
                            new_objective.inequality_allowable_feasiblity_upper = tokens[4];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"inequality","feasibility","scale"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"inequality feasibility scale\" keywords.\n";
                                return false;
                            }
                            new_objective.inequality_feasibility_scale = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"inequality","infeasibility","scale"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"inequality infeasibility scale\" keywords.\n";
                                return false;
                            }
                            new_objective.inequality_infeasibility_scale = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"stress","inequality","power"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"stress inequality power\" keywords.\n";
                                return false;
                            }
                            new_objective.stress_inequality_power = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"stress","favor","final"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"stress favor final\" keywords.\n";
                                return false;
                            }
                            new_objective.stress_favor_final = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"stress","favor","updates"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"stress favor updates\" keywords.\n";
                                return false;
                            }
                            new_objective.stress_favor_updates = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"volume","penalty","power"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"volume penalty power\" keywords.\n";
                                return false;
                            }
                            new_objective.volume_penalty_power = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"volume","penalty","divisor"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"volume penalty divisor\" keywords.\n";
                                return false;
                            }
                            new_objective.volume_penalty_divisor = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"volume","penalty","bias"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"volume penalty bias\" keywords.\n";
                                return false;
                            }
                            new_objective.volume_penalty_bias = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"load","ids"}, tStringValue))
                        {
                            if(tokens.size() < 3)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"load ids\" keywords.\n";
                                return false;
                            }
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                new_objective.load_case_ids.push_back(tokens[j]);
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"load","case","weights"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"load case weights\" keywords.\n";
                                return false;
                            }
                            for(size_t j=3; j<tokens.size(); ++j)
                            {
                                new_objective.load_case_weights.push_back(tokens[j]);
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"code"}, tStringValue))
                        {
                            if(tokens.size() < 2)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No code specified after \"code\" keyword.\n";
                                return false;
                            }
                            new_objective.code_name = tokens[1];
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                new_objective.code_name += " ";
                                new_objective.code_name += tokens[j];
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"complex","error","measure"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No complex error measure value specified after \"complex error measure\" keywords.\n";
                                return false;
                            }
                            new_objective.complex_error_measure = tokens[3];
                            for(size_t j=4; j<tokens.size(); ++j)
                            {
                                new_objective.complex_error_measure += " ";
                                new_objective.complex_error_measure += tokens[j];
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"output","for","plotting"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No outputs specified after \"output for plotting\" keywords.\n";
                                return false;
                            }
                            for(size_t j=3; j<tokens.size(); ++j)
                            {
                                new_objective.output_for_plotting.push_back(tokens[j]);
                            }
                        }
                        else if (parseSingleValue(tokens, tInputStringList = {"ls","tet","type"}, tStringValue))
                        {
                            new_objective.convert_to_tet10 = tokens[3];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"number","processors"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"number processors\" keywords.\n";
                                return false;
                            }
                            new_objective.num_procs = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"weight"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"weight\" keyword.\n";
                                return false;
                            }
                            new_objective.weight = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"multi","load","case"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"multi load case\" keywords.\n";
                                return false;
                            }
                            new_objective.multi_load_case = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"frf","match","nodesets"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"frf match nodesets\" keywords.\n";
                                return false;
                            }
                            for(size_t j=3; j<tokens.size(); ++j)
                                new_objective.frf_match_nodesets.push_back(tokens[j]);
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"weightmass","scale","factor"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"weightmass scale factor\" keywords.\n";
                                return false;
                            }
                            new_objective.wtmass_scale_factor = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"analysis","solver","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"analysis solver tolerance\" keywords.\n";
                                return false;
                            }
                            new_objective.analysis_solver_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"normalize","objective"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"normalize objective\" keywords.\n";
                                return false;
                            }
                            new_objective.normalize_objective = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"reference","frf","file"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"reference frf file\" keywords.\n";
                                return false;
                            }
                            new_objective.ref_frf_file = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"raleigh","damping","alpha"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"raleigh damping alpha\" keywords.\n";
                                return false;
                            }
                            new_objective.raleigh_damping_alpha = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"raleigh","damping","beta"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"raleigh damping beta\" keywords.\n";
                                return false;
                            }
                            new_objective.raleigh_damping_beta = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"distribute","objective"}, tStringValue))
                        {
                            if(tokens.size() < 3)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No case specified after \"repeat objective\" keywords.\n";
                                return false;
                            }
                            if(tokens[2]=="none")
                            {
                                // distribute objective none
                                // 0          1         2
                                new_objective.distribute_objective_type="";
                            }
                            else if(tokens[2]=="at")
                            {
                                // distribute objective at most {number} processors
                                // 0          1         2  3    4        5
                                if(tokens.size() < 6)
                                {
                                    std::cout << "ERROR:XMLGenerator:parseObjectives: Unmatched case specified after \"repeat objective\" keywords.\n";
                                    return false;
                                }
                                if(tokens[3]!="most" || tokens[5]!="processors")
                                {
                                    std::cout << "ERROR:XMLGenerator:parseObjectives: Unmatched case specified after \"repeat objective\" keywords.\n";
                                    return false;
                                }
                                new_objective.distribute_objective_type = "atmost";
                                new_objective.atmost_total_num_processors = tokens[4];
                            }
                            else
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: Unmatched case specified after \"repeat objective\" keywords.\n";
                                return false;
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"analyze","new","workflow"}, tStringValue))
                        {
                          if(tokens[3] == "true")
                            m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow = true;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"number", "ranks"}, tStringValue))
                        {
                          new_objective.num_ranks = tokens[2];
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"begin","frequency"}, tStringValue))
                        {
                            while (!fin.eof())
                            {
                                tokens.clear();
                                fin.getline(buf, MAX_CHARS_PER_LINE);
                                parseTokens(buf, tokens);
                                // process the tokens
                                if(tokens.size() > 0)
                                {
                                    for(size_t j=0; j<tokens.size(); ++j)
                                        tokens[j] = toLower(tokens[j]);

                                    if(parseSingleValue(tokens, tInputStringList = {"end","frequency"}, tStringValue))
                                    {
                                        if(new_objective.freq_min == "" ||
                                                new_objective.freq_max == "" ||
                                                new_objective.freq_step == "")
                                        {
                                            std::cout << "ERROR:XMLGenerator:parseObjectives: Insufficient info in \"frequency block\".\n";
                                            return false;
                                        }
                                        break;
                                    }
                                    else
                                    {
                                        if(parseSingleValue(tokens, tInputStringList = {"min"}, tStringValue))
                                        {
                                            new_objective.freq_min = tStringValue;
                                        }
                                        else if(parseSingleValue(tokens, tInputStringList = {"max"}, tStringValue))
                                        {
                                            new_objective.freq_max = tStringValue;
                                        }
                                        else if(parseSingleValue(tokens, tInputStringList = {"step"}, tStringValue))
                                        {
                                            new_objective.freq_step = tStringValue;
                                        }
                                        else
                                        {
                                            PrintUnrecognizedTokens(tokens);
                                            std::cout << "ERROR:XMLGenerator:parseObjectives: Unrecognized keyword in \"frequency block\".\n";
                                            return false;
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            PrintUnrecognizedTokens(tokens);
                            std::cout << "ERROR:XMLGenerator:parseObjectives: Unrecognized keywords.\n";
                            return false;
                        }
                    }
                }

                // do checking of this objective

                // check (number of load ids) versus (number of load weights)
                const size_t num_load_ids = new_objective.load_case_ids.size();
                const size_t num_load_case_weights = new_objective.load_case_weights.size();
                if(num_load_case_weights == 0u)
                {
                    // replace with constant weighting
                    const double constantWeightFraction = 1.0;
                    const std::string uniformWeightFraction_str = Plato::to_string(constantWeightFraction);
                    new_objective.load_case_weights.assign(num_load_ids, uniformWeightFraction_str);
                    // must be constant at 1.0 rather than (1.0/num_cases) to allow
                    // multi-load to be equivalent to multi-objective. We don't know within
                    // this objective how many other loads should be considered "equally".
                }
                else if(num_load_ids != num_load_case_weights)
                {
                    std::cout << "ERROR:XMLGenerator:parseObjectives: Length mismatch in load case ids and weights.\n";
                    return false;
                }

                // check that (distribute objective) must have (multi load case true)
                const bool have_distributed_objective =
                        (new_objective.distribute_objective_type != "");
                const bool have_multiLoadCaseTrue =
                        (new_objective.multi_load_case == "true");
                if(have_distributed_objective && !have_multiLoadCaseTrue)
                {
                    std::cout << "ERROR:XMLGenerator:parseObjectives: "
                            << "Parsed input is ambiguous.\n"
                            << "Distributed objectives must have \"multi load case true\".\n"
                            << "Or set \"distribute objective none\".\n";
                    return false;
                }

                // place objective in array
                m_InputData.objectives.push_back(new_objective);
            }
        }
    }

    // assign names
    const bool filling_did_pass = fillObjectiveAndPerfomerNames();
    if(!filling_did_pass)
    {
        std::cout << "ERROR:XMLGenerator:parseObjectives: Failed to fill objective and performer names.\n";
        return false;
    }

    return true;
}

/******************************************************************************/
bool XMLGenerator::fillObjectiveAndPerfomerNames()
/******************************************************************************/
{
    // assigns objective names to yet un-named objectives
    // assigns performer name to each objective

    char buf2[200];
    size_t num_objs = m_InputData.objectives.size();
    // If there were objectives without names add a default name
    for(size_t i=0; i<num_objs; ++i)
    {
        // For each code name we will make sure there are names set
        std::string cur_code_name = m_InputData.objectives[i].code_name;
        int num_cur_code_objs=0;
        for(size_t j=i; j<num_objs; ++j)
        {
            if(!m_InputData.objectives[j].code_name.compare(cur_code_name))
            {
                num_cur_code_objs++;
                if(m_InputData.objectives[j].name.empty())
                {
                    sprintf(buf2, "%d", num_cur_code_objs);
                    m_InputData.objectives[j].name = buf2;
                }
            }
        }
    }
    // Set the performer names
    for(size_t i=0; i<num_objs; ++i)
    {
        m_InputData.objectives[i].performer_name =
                m_InputData.objectives[i].code_name +
                "_" + m_InputData.objectives[i].name;
    }

    return true;
}

/******************************************************************************/
bool XMLGenerator::parseLoads(std::istream &fin)
/******************************************************************************/
{
    std::vector<std::string> tInputStringList;
    std::vector<std::string> tokens;
    std::string tStringValue;
    bool load_block_found = false;

    // read each line of the file
    while (!fin.eof())
    {
        getTokensFromLine(fin,tokens);

        // process the tokens
        if(tokens.size() > 0)
        {
            if(parseSingleValue(tokens, tInputStringList = {"begin","loads"}, tStringValue))
            {
              if(!parseLoadsBlock(fin))
                return false;
              load_block_found = true;
            }
        }
    }

    if(!load_block_found)
    {
      std::cout << "ERROR:XMLGenerator:parseLoads: No load block found \n";
      return false;
    }

    return true;
}

/******************************************************************************/
bool XMLGenerator::parseLoadsBlock(std::istream &fin)
/******************************************************************************/
{
  std::vector<std::string> tInputStringList;
  std::vector<std::string> tokens;
  std::string tStringValue;
  
  while (!fin.eof())
  {
    getTokensFromLine(fin,tokens);

    if(tokens.size() > 0)
    {
      for(size_t j=0; j<tokens.size(); ++j)
          tokens[j] = toLower(tokens[j]);
      if(parseSingleValue(tokens, tInputStringList = {"end","loads"}, tStringValue))
        break;
      else
        if(!parseLoadLine(tokens))
          return false;
    }
  }
  return true;
}

/******************************************************************************/
void XMLGenerator::getTokensFromLine(std::istream &fin, std::vector<std::string>& tokens)
/******************************************************************************/
{
    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    fin.getline(buf, MAX_CHARS_PER_LINE);
    parseTokens(buf, tokens);
}

/******************************************************************************/
bool XMLGenerator::parseLoadLine(std::vector<std::string>& tokens)
/******************************************************************************/
{
    XMLGen::Load new_load;
    new_load.type = tokens[0];
    bool return_status = true;

    if(!new_load.type.compare("traction"))
      return_status = parseTractionLoad(tokens,new_load);
    else if(!new_load.type.compare("pressure"))
      return_status = parsePressureLoad(tokens,new_load);
    else if(!new_load.type.compare("acceleration"))
      return_status = parseAccelerationLoad(tokens,new_load);
    else if(!new_load.type.compare("heat"))
      return_status = parseHeatFluxLoad(tokens,new_load);
    else if(!new_load.type.compare("force"))
      return_status = parseForceLoad(tokens,new_load);
    else
    {
        PrintUnrecognizedTokens(tokens);
        std::cout << "ERROR:XMLGenerator:parseLoads: Unrecognized load type.\n";
        return false;
    }

    if(return_status)
      putLoadInLoadCase(new_load);

    return return_status;
}

/******************************************************************************/
bool XMLGenerator::parseTractionLoad(std::vector<std::string>& tokens, XMLGen::Load& new_load)
/******************************************************************************/
{
    size_t tMin_parameters = 10;
    if(tokens.size() < tMin_parameters)
    {
      std::cout << "ERROR:XMLGenerator:parseLoads: Wrong number of parameters specified for \"traction\" load.\n";
      return false;
    }

    size_t tTokenIndex = 0;

    new_load.app_type = tokens[++tTokenIndex];

    if(parseMeshSetNameOrID(tTokenIndex,tokens,new_load))
    {
      if(!parseMeshSetNameOrID(tTokenIndex,tokens,new_load))
        --tTokenIndex;
    }
    else
    {
      new_load.app_name = "";
      new_load.app_id = tokens[tTokenIndex];
    }

    if(tokens[++tTokenIndex] != "value")
    {
      std::cout << "ERROR:XMLGenerator:parseLoads: \"value\" keyword not specified after sideset id.\n";
      return false;
    }

    new_load.values.push_back(tokens[++tTokenIndex]);
    new_load.values.push_back(tokens[++tTokenIndex]);
    new_load.values.push_back(tokens[++tTokenIndex]);

    if(tokens[++tTokenIndex] != "load" || tokens[++tTokenIndex] != "id")
    {
      std::cout << "ERROR:XMLGenerator:parseLoads: \"load id\" keywords not specified after value components.\n";
      return false;
    }

    new_load.load_id = tokens[++tTokenIndex];


    return true;
}

/******************************************************************************/
bool XMLGenerator::parseMeshSetNameOrID(size_t& aTokenIndex, std::vector<std::string>& tokens, XMLGen::Load& new_load)
/******************************************************************************/
{
    if(tokens[++aTokenIndex] == "id")
    {
      new_load.app_id = tokens[++aTokenIndex];
      return true;
    }
    else if(tokens[aTokenIndex] == "name")
    {
      new_load.app_name = tokens[++aTokenIndex];
      return true;
    }
    else
      return false;
}

/******************************************************************************/
bool XMLGenerator::parsePressureLoad(std::vector<std::string>& tokens, XMLGen::Load& new_load)
/******************************************************************************/
{
  size_t tMin_parameters = 8;
  if(tokens.size() < tMin_parameters)
  {
    std::cout << "ERROR:XMLGenerator:parseLoads: Wrong number of parameters specified for \"traction\" load.\n";
    return false;
  }

  size_t tTokenIndex = 0;
  new_load.app_type = tokens[++tTokenIndex];
  if(new_load.app_type != "sideset")
  {
      std::cout << "ERROR:XMLGenerator:parseLoads: Pressures can currently only be specified on sidesets.\n";
      return false;
  }

  if(parseMeshSetNameOrID(tTokenIndex,tokens,new_load))
  {
    if(!parseMeshSetNameOrID(tTokenIndex,tokens,new_load))
      --tTokenIndex;
  }
  else
  {
    new_load.app_name = "";
    new_load.app_id = tokens[tTokenIndex];
  }

  if(tokens[++tTokenIndex] != "value")
  {
      std::cout << "ERROR:XMLGenerator:parseLoads: \"value\" keyword not specified after sideset id.\n";
      return false;
  }
  new_load.values.push_back(tokens[++tTokenIndex]);
  if(tokens[++tTokenIndex] != "load" || tokens[++tTokenIndex] != "id")
  {
      std::cout << "ERROR:XMLGenerator:parseLoads: \"load id\" keywords not specified after value components.\n";
      return false;
  }
  new_load.load_id = tokens[++tTokenIndex];

  return true;
}

/******************************************************************************/
bool XMLGenerator::parseAccelerationLoad(std::vector<std::string>& tokens, XMLGen::Load& new_load)
/******************************************************************************/
{
  size_t tMin_parameters = 7;
  if(tokens.size() != tMin_parameters)
  {
    std::cout << "ERROR:XMLGenerator:parseLoads: Wrong number of parameters specified for \"traction\" load.\n";
    return false;
  }
  new_load.app_type = "body";
  new_load.values.push_back(tokens[1]);
  new_load.values.push_back(tokens[2]);
  new_load.values.push_back(tokens[3]);
  if(tokens[4] != "load" || tokens[5] != "id")
  {
      std::cout << "ERROR:XMLGenerator:parseLoads: \"load id\" keywords not specified after acceleration components.\n";
      return false;
  }
  new_load.load_id = tokens[6];
  return true;
}

/******************************************************************************/
bool XMLGenerator::parseHeatFluxLoad(std::vector<std::string>& tokens, XMLGen::Load& new_load)
/******************************************************************************/
{
  size_t tMin_parameters = 9;
  if(tokens.size() < tMin_parameters)
  {
    std::cout << "ERROR:XMLGenerator:parseLoads: Wrong number of parameters specified for \"traction\" load.\n";
    return false;
  }
  size_t tTokenIndex = 0;
  if(!tokens[++tTokenIndex].compare("flux"))
  {
      new_load.app_type = tokens[++tTokenIndex];
      if(new_load.app_type != "sideset")
      {
          std::cout << "ERROR:XMLGenerator:parseLoads: Heat flux can only be specified on sidesets currently.\n";
          return false;
      }

      if(parseMeshSetNameOrID(tTokenIndex,tokens,new_load))
      {
        if(!parseMeshSetNameOrID(tTokenIndex,tokens,new_load))
          --tTokenIndex;
      }
      else
      {
        new_load.app_name = "";
        new_load.app_id = tokens[tTokenIndex];
      }

      if(tokens[++tTokenIndex] != "value")
      {
          std::cout << "ERROR:XMLGenerator:parseLoads: 'value' keyword not specified after sideset id\n";
          return false;
      }
      new_load.values.push_back(tokens[++tTokenIndex]);
      if(tokens[++tTokenIndex] != "load" || tokens[++tTokenIndex] != "id")
      {
          std::cout << "ERROR:XMLGenerator:parseLoads: \"load id\" keywords not specified after value components.\n";
          return false;
      }
      new_load.load_id = tokens[++tTokenIndex];
  }
  else
  {
      std::cout << "ERROR:XMLGenerator:parseLoads: \"flux\" keyword must follow \"heat\" keyword.\n";
      return false;
  }

  return true;
}

/******************************************************************************/
bool XMLGenerator::parseForceLoad(std::vector<std::string>& tokens, XMLGen::Load& new_load)
/******************************************************************************/
{
  size_t tMin_parameters = 10;
  if(tokens.size() < tMin_parameters)
  {
    std::cout << "ERROR:XMLGenerator:parseLoads: Wrong number of parameters specified for \"traction\" load.\n";
    return false;
  }
  size_t tTokenIndex = 0;
  new_load.app_type = tokens[++tTokenIndex];
  if(new_load.app_type != "sideset" && new_load.app_type != "nodeset")
  {
      std::cout << "ERROR:XMLGenerator:parseLoads: Forces can only be applied to nodesets or sidesets currently.\n";
      return false;
  }

  if(parseMeshSetNameOrID(tTokenIndex,tokens,new_load))
  {
    if(!parseMeshSetNameOrID(tTokenIndex,tokens,new_load))
      --tTokenIndex;
  }
  else
  {
    new_load.app_name = "";
    new_load.app_id = tokens[tTokenIndex];
  }
  
  if(tokens[++tTokenIndex] != "value")
  {
      std::cout << "ERROR:XMLGenerator:parseLoads: \"value\" keyword not specified after nodeset or sideset id.\n";
      return false;
  }
  new_load.values.push_back(tokens[++tTokenIndex]);
  new_load.values.push_back(tokens[++tTokenIndex]);
  new_load.values.push_back(tokens[++tTokenIndex]);
  if(tokens[++tTokenIndex] != "load" || tokens[++tTokenIndex] != "id")
  {
      std::cout << "ERROR:XMLGenerator:parseLoads: \"load id\" keywords not specified after value components.\n";
      return false;
  }
  new_load.load_id = tokens[++tTokenIndex];

  return true;
}

/******************************************************************************/
void XMLGenerator::putLoadInLoadCase(XMLGen::Load& new_load)
/******************************************************************************/
{
  bool found_load_case = putLoadInLoadCaseWithMatchingID(new_load);

  if(!found_load_case)
    createNewLoadCase(new_load);
}

/******************************************************************************/
bool XMLGenerator::putLoadInLoadCaseWithMatchingID(XMLGen::Load& new_load)
/******************************************************************************/
{
  for(size_t h=0; h<m_InputData.load_cases.size(); ++h)
      if(m_InputData.load_cases[h].id == new_load.load_id)
      {
        m_InputData.load_cases[h].loads.push_back(new_load);
        return true;
      }
  return false;
}

/******************************************************************************/
void XMLGenerator::createNewLoadCase(XMLGen::Load& new_load)
/******************************************************************************/
{
  XMLGen::LoadCase new_load_case;
  new_load_case.id = new_load.load_id;
  new_load_case.loads.push_back(new_load);
  m_InputData.load_cases.push_back(new_load_case);
}


/******************************************************************************/
bool XMLGenerator::parseUncertainties(std::istream &aInputFile)
/******************************************************************************/
{
    XMLGen::ParseUncertainty tParseUncertainty;
    tParseUncertainty.parse(aInputFile);
    m_InputData.uncertainties = tParseUncertainty.data();
    return true;
}

/******************************************************************************/
bool XMLGenerator::parseBCs(std::istream &fin)
/******************************************************************************/
{
    std::vector<std::string> tInputStringList;
    std::vector<std::string> tokens;
    std::string tStringValue;
    bool bc_block_found = false;

    // read each line of the file
    while (!fin.eof())
    {
        getTokensFromLine(fin,tokens);

        // process the tokens
        if(tokens.size() > 0)
        {
            if(parseSingleValue(tokens, tInputStringList = {"begin","boundary","conditions"}, tStringValue))
            {
              if(!parseBCsBlock(fin))
                return false;
              bc_block_found = true;
            }
        }
    }

    if(!bc_block_found)
    {
      std::cout << "ERROR:XMLGenerator:parseLoads: No boundary condition block found \n";
      return false;
    }

    return true;
}

/******************************************************************************/
bool XMLGenerator::parseBCsBlock(std::istream &fin)
/******************************************************************************/
{
  std::vector<std::string> tInputStringList;
  std::vector<std::string> tokens;
  std::string tStringValue;
  
  while (!fin.eof())
  {
    getTokensFromLine(fin,tokens);

    if(tokens.size() > 0)
    {
      for(size_t j=0; j<tokens.size(); ++j)
          tokens[j] = toLower(tokens[j]);
      if(parseSingleValue(tokens, tInputStringList = {"end","boundary","conditions"}, tStringValue))
        break;
      else
        if(!parseBCLine(tokens))
          return false;
    }
  }
  return true;
}

/******************************************************************************/
bool XMLGenerator::parseBCLine(std::vector<std::string>& tokens)
/******************************************************************************/
{
    XMLGen::BC new_bc;
    bool return_status = true;

    if(tokens.size() < 7)
    {
        std::cout << "ERROR:XMLGenerator:parseBCs: Not enough parameters were specified for BC in \"boundary conditions\" block.\n";
        return false;
    }
    if(tokens[0] != "fixed")
    {
        std::cout << "ERROR:XMLGenerator:parseBCs: First boundary condition token must be \"fixed\".\n";
        return false;
    }
    new_bc.type = tokens[1];

    if(!new_bc.type.compare("displacement"))
      return_status = parseDisplacementBC(tokens,new_bc);
    else if(!new_bc.type.compare("temperature"))
      return_status = parseTemperatureBC(tokens,new_bc);
    else
    {
        PrintUnrecognizedTokens(tokens);
        std::cout << "ERROR:XMLGenerator:parseLoads: Unrecognized boundary condition type.\n";
        return false;
    }

    m_InputData.bcs.push_back(new_bc);

    return return_status;
}

/******************************************************************************/
bool XMLGenerator::parseDisplacementBC(std::vector<std::string>& tokens, XMLGen::BC& new_bc)
/******************************************************************************/
{
  // Potential syntax:
  // fixed displacement nodeset/sideset 1 bc id 1                 // all dofs have fixed disp of 0.0
  // fixed displacement nodeset/sideset 1 <x,y,z> bc id 1         // x, y, or z dof has fixed disp of 0.0
  // fixed displacement nodeset/sideset 1 <x,y,z> 3.0 bc id 1     // x, y, or z dof has fixed disp of 3.0

  size_t tTokenIndex = 1;
  bool tNameOrIDSpecified = false;

  if(tokens[++tTokenIndex] != "nodeset" && tokens[tTokenIndex] != "sideset")
  {
    std::cout << "ERROR:XMLGenerator:parseBCs: Boundary conditions can only be applied to \"nodeset\" or \"sideset\" types.\n";
    return false;
  }
  new_bc.app_type = tokens[tTokenIndex];

  if(tokens[++tTokenIndex] == "id")
  {
    new_bc.app_id = tokens[++tTokenIndex];
    tNameOrIDSpecified = true;
  }
  else if(tokens[tTokenIndex] == "name")
  {
    new_bc.app_name = tokens[++tTokenIndex];
    tNameOrIDSpecified = true;
  }
  else
  {
    new_bc.app_id = tokens[tTokenIndex];
    new_bc.app_name = "";
  }
  
  if(tNameOrIDSpecified)
  {
    if(tokens[++tTokenIndex] == "id")
      new_bc.app_id = tokens[++tTokenIndex];
    else if(tokens[tTokenIndex] == "name")
      new_bc.app_name = tokens[++tTokenIndex];
    else
      --tTokenIndex;
  }

  new_bc.dof = "";
  new_bc.value = "";
  if(tokens[++tTokenIndex] != "bc")
  {
    if(tokens[tTokenIndex] != "x" && tokens[tTokenIndex] != "y" && tokens[tTokenIndex] != "z")
    {
        std::cout << "ERROR:XMLGenerator:parseBCs: Boundary condition degree of freedom must be either \"x\", \"y\", or \"z\".\n";
        return false;
    }
    new_bc.dof = tokens[tTokenIndex];
    if(tokens[++tTokenIndex] != "bc")
    {
        new_bc.value = tokens[tTokenIndex];
        tTokenIndex += 3;
        new_bc.bc_id = tokens[tTokenIndex];
    }
    else
    {
        tTokenIndex += 2;
        new_bc.bc_id = tokens[tTokenIndex];
    }
  }
  else
  {
    tTokenIndex += 2;
    new_bc.bc_id = tokens[tTokenIndex];
  }

  return true;
}

/******************************************************************************/
bool XMLGenerator::parseTemperatureBC(std::vector<std::string>& tokens, XMLGen::BC& new_bc)
/******************************************************************************/
{
  // Potential syntax:
  // fixed temperature nodeset 1 bc id 1
  // fixed temperature nodeset 1 value 25.0 bc id 1
  if(tokens[2] != "nodeset" && tokens[2] != "sideset")
  {
      std::cout << "ERROR:XMLGenerator:parseBCs: Boundary conditions can only be applied to \"nodeset\" or \"sideset\" types.\n";
      return false;
  }
  new_bc.app_type = tokens[2];

  size_t tOffset = 0;
  bool name_or_id_specified = false;
  if(tokens[3] == "id")
  {
    ++tOffset;
    new_bc.app_id = tokens[3+tOffset];
    name_or_id_specified = true;
  }
  else if(tokens[3] == "name")
  {
    ++tOffset;
    new_bc.app_name = tokens[3+tOffset];
    name_or_id_specified = true;
  }
  else
  {
    new_bc.app_id = tokens[3];
    new_bc.app_name = "";
  }

  if(name_or_id_specified)
  {
    if(tokens[4+tOffset] == "id")
    {
      ++tOffset;
      new_bc.app_id = tokens[4+tOffset];
    }
    else if(tokens[4+tOffset] == "name")
    {
      ++tOffset;
      new_bc.app_name = tokens[4+tOffset];
    }
  }

  new_bc.value = "";
  if(tokens[4+tOffset] != "bc")
  {
      if(tokens[4+tOffset] != "value")
      {
          std::cout << "ERROR:XMLGenerator:parseBCs: Invalid BC syntax.\n";
          return false;
      }
      new_bc.value = tokens[5+tOffset];
      if(tokens[6+tOffset] != "bc")
      {
          std::cout << "ERROR:XMLGenerator:parseBCs: Invalid BC syntax.\n";
          return false;
      }
      new_bc.bc_id = tokens[8+tOffset];
  }
  else
      new_bc.bc_id = tokens[6+tOffset];

  return true;
}

/******************************************************************************/
bool XMLGenerator::parseSingleValue(const std::vector<std::string> &aTokens,
                                    const std::vector<std::string> &aInputStrings,
                                    std::string &aReturnStringValue)
/******************************************************************************/
{
    size_t i;

    aReturnStringValue = "";

    if(aInputStrings.size() < 1 || aTokens.size() < 1 || aTokens.size() < aInputStrings.size())
        return false;

    for(i=0; i<aInputStrings.size(); ++i)
    {
        if(aTokens[i].compare(aInputStrings[i]))
        {
            return false;
        }
    }

    if(aTokens.size() == (aInputStrings.size() + 1))
        aReturnStringValue = aTokens[i];

    return true;
}

/******************************************************************************/
bool XMLGenerator::parseSingleUnLoweredValue(const std::vector<std::string> &aTokens,
                                             const std::vector<std::string> &aUnLoweredTokens,
                                    const std::vector<std::string> &aInputStrings,
                                    std::string &aReturnStringValue)
/******************************************************************************/
{
    size_t i;

    aReturnStringValue = "";

    assert(aTokens.size() == aUnLoweredTokens.size());

    if(aInputStrings.size() < 1 || aTokens.size() < 1 || aTokens.size() < aInputStrings.size())
        return false;

    for(i=0; i<aInputStrings.size(); ++i)
    {
        if(aTokens[i].compare(aInputStrings[i]))
        {
            return false;
        }
    }

    if(aTokens.size() == (aInputStrings.size() + 1))
        aReturnStringValue = aUnLoweredTokens[i];

    return true;
}

/******************************************************************************/
void XMLGenerator::initializePlatoProblemOptions()
/******************************************************************************/
{
    m_InputData.output_frequency="5";
    m_InputData.discretization="density";
    m_InputData.initial_density_value="0.5";
    m_InputData.optimization_algorithm="oc";
    m_InputData.output_method="epu";
    m_InputData.check_gradient = "false";
    m_InputData.check_hessian = "false";
    m_InputData.filter_type = "kernel";
    m_InputData.filter_power = "1";

    m_InputData.mInnerKKTtoleranceGCMMA = "";
    m_InputData.mOuterKKTtoleranceGCMMA = "";
    m_InputData.mInnerControlStagnationToleranceGCMMA = "";
    m_InputData.mOuterControlStagnationToleranceGCMMA = "";
    m_InputData.mOuterObjectiveStagnationToleranceGCMMA = "";
    m_InputData.mMaxInnerIterationsGCMMA = "";
    m_InputData.mOuterStationarityToleranceGCMMA = "";
    m_InputData.mInitialMovingAsymptotesScaleFactorGCMMA = "";

    m_InputData.mMaxRadiusScale = "";
    m_InputData.mInitialRadiusScale = "";
    m_InputData.mMaxTrustRegionRadius = "";
    m_InputData.mMinTrustRegionRadius = "";
    m_InputData.mMaxTrustRegionIterations = "5";
    m_InputData.mTrustRegionExpansionFactor = "";
    m_InputData.mTrustRegionContractionFactor = "";
    m_InputData.mTrustRegionRatioLowKS = "";
    m_InputData.mTrustRegionRatioMidKS = "";
    m_InputData.mTrustRegionRatioUpperKS = "";

    m_InputData.mUseMeanNorm = "";
    m_InputData.mAugLagPenaltyParam = "";
    m_InputData.mFeasibilityTolerance = "";
    m_InputData.mAugLagPenaltyParamScale = "";
    m_InputData.mMaxNumAugLagSubProbIter = "";

    m_InputData.mHessianType = "disabled";
    m_InputData.mLimitedMemoryStorage = "8";
    m_InputData.mDisablePostSmoothingKS = "true";
    m_InputData.mProblemUpdateFrequency = "5";
    m_InputData.mOuterGradientToleranceKS = "";
    m_InputData.mOuterStationarityToleranceKS = "";
    m_InputData.mOuterStagnationToleranceKS = "";
    m_InputData.mOuterControlStagnationToleranceKS = "";
    m_InputData.mOuterActualReductionToleranceKS = "";

    m_InputData.filter_heaviside_min = "";
    m_InputData.filter_heaviside_update = "";
    m_InputData.filter_heaviside_max = "";

    m_InputData.filter_projection_start_iteration = "";
    m_InputData.filter_projection_update_interval = "";
    m_InputData.filter_use_additive_continuation = "";

    m_InputData.write_restart_file = "False";
    m_InputData.optimization_type = "topology";
}

/******************************************************************************/
bool XMLGenerator::parseOptimizationParameters(std::istream &fin)
/******************************************************************************/
{
  // Initialize variables
  this->initializePlatoProblemOptions();

  std::string tStringValue;
  std::vector<std::string> tInputStringList;
  // read each line of the file
  while (!fin.eof())
  {
    // read an entire line into memory
    char buf[MAX_CHARS_PER_LINE];
    fin.getline(buf, MAX_CHARS_PER_LINE);
    std::vector<std::string> tokens;
    parseTokens(buf, tokens);

    // process the tokens
    if(tokens.size() > 0)
    {
      for(size_t j=0; j<tokens.size(); ++j)
        tokens[j] = toLower(tokens[j]);

      if(parseSingleValue(tokens, tInputStringList = {"begin","optimization","parameters"}, tStringValue))
      {
        // found optimization parameters
        while (!fin.eof())
        {
          fin.getline(buf, MAX_CHARS_PER_LINE);
          tokens.clear();
          parseTokens(buf, tokens);
          // process the tokens
          if(tokens.size() > 0)
          {
            std::vector<std::string> unlowered_tokens = tokens;

            for(size_t j=0; j<tokens.size(); ++j)
              tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"end","optimization","parameters"}, tStringValue))
            {
              break;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"begin","material_box"}, tStringValue))
            {
              while (!fin.eof())
              {
                fin.getline(buf, MAX_CHARS_PER_LINE);
                tokens.clear();
                parseTokens(buf, tokens);
                // process the tokens
                if(tokens.size() > 0)
                {
                  for(size_t j=0; j<tokens.size(); ++j)
                    tokens[j] = toLower(tokens[j]);

                  if(parseSingleValue(tokens, tInputStringList = {"end","material_box"}, tStringValue))
                  {
                    if(m_InputData.levelset_material_box_min == "" ||
                        m_InputData.levelset_material_box_max == "")
                    {
                      std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Not enough info in \"material_box\" block.\n";
                      return false;
                    }
                    break;
                  }
                  else if(parseSingleValue(tokens, tInputStringList = {"min", "coords"}, tStringValue))
                  {
                    if(tokens.size() != 5)
                    {
                      std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Wrong number of params in \"min coords\" line.\n";
                      return false;
                    }
                    for(size_t j=2; j<5; ++j)
                    {
                      m_InputData.levelset_material_box_min += tokens[j];
                      if(j<4)
                        m_InputData.levelset_material_box_min += " ";
                    }
                  }
                  else if(parseSingleValue(tokens, tInputStringList = {"max", "coords"}, tStringValue))
                  {
                    if(tokens.size() != 5)
                    {
                      std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Wrong number of params in \"max coords\" line.\n";
                      return false;
                    }
                    for(size_t j=2; j<5; ++j)
                    {
                      m_InputData.levelset_material_box_max += tokens[j];
                      if(j<4)
                        m_InputData.levelset_material_box_max += " ";
                    }
                  }
                }
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"objective","number","standard","deviations"}, tStringValue))
            {
              if(tokens.size() < 5)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"objective number standard deviations\" keywords.\n";
                return false;
              }
              m_InputData.objective_number_standard_deviations = tokens[4];
            }
            else if(parseSingleValue(tokens, tInputStringList = {"output", "standard","deviations"}, tStringValue))
            {
              if(tokens.size() < 4)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No outputs specified after \"output standard deviations\" keywords.\n";
                return false;
              }
              for(size_t j=3; j<tokens.size(); ++j)
              {
                m_InputData.mStandardDeviations.push_back(tokens[j]);
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"input","generator","version"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"input generator version\" keyword(s).\n";
                return false;
              }
              m_InputData.input_generator_version = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"initial","density","value"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial density value\" keyword(s).\n";
                return false;
              }
              m_InputData.initial_density_value = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"optimization","type"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"optimization type\" keyword(s).\n";
                return false;
              }
              m_InputData.optimization_type = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"csm","file"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"csm file\" keyword(s).\n";
                return false;
              }
              m_InputData.csm_filename = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"use","normalization","in","aggregator"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"use normalization in aggregator\" keyword(s).\n";
                return false;
              }
              m_InputData.mUseNormalizationInAggregator = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"create","levelset","spheres"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"create levelset spheres\" keyword(s).\n";
                return false;
              }
              m_InputData.create_levelset_spheres = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"write","restart","file"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"write restart file\" keyword(s).\n";
                return false;
              }
              if(tStringValue == "true")
                m_InputData.write_restart_file = "True";
            }
            else if(parseSingleValue(tokens, tInputStringList = {"levelset","initialization","method"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset initialization method\" keyword(s).\n";
                return false;
              }
              m_InputData.levelset_initialization_method = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"max","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"max iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.max_iterations = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"restart","iteration"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"restart iteration\" keyword(s).\n";
                return false;
              }
              m_InputData.restart_iteration = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"initial","guess","filename"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial guess filename\" keyword(s).\n";
                return false;
              }
              m_InputData.initial_guess_filename = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"initial","guess","field","name"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial guess field name\" keyword(s).\n";
                return false;
              }
              m_InputData.initial_guess_field_name = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"prune","mesh"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"prune mesh\" keyword(s).\n";
                return false;
              }
              m_InputData.prune_mesh = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","buffer","layers"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number buffer layers\" keyword(s).\n";
                return false;
              }
              m_InputData.number_buffer_layers = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","prune","and","refine","processors"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number prune and refine processors\" keyword(s).\n";
                return false;
              }
              m_InputData.number_prune_and_refine_processors = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","refines"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number refines\" keyword(s).\n";
                return false;
              }
              m_InputData.number_refines = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","move","limit"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma move limit\" keyword(s).\n";
                return false;
              }
              m_InputData.mMMAMoveLimit = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","control","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma control stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mMMAControlStagnationTolerance = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","objective","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma objective stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mMMAObjectiveStagnationTolerance = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","asymptote","expansion"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma asymptote expansion\" keyword(s).\n";
                return false;
              }
              m_InputData.mMMAAsymptoteExpansion = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","asymptote","contraction"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma asymptote contraction\" keyword(s).\n";
                return false;
              }
              m_InputData.mMMAAsymptoteContraction = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","max","sub","problem","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma max sub problem iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.mMMAMaxNumSubProblemIterations = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","max","trust","region","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma max trust region iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.mMMAMaxTrustRegionIterations = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","max","trust","region","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks max trust region iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.mMaxTrustRegionIterations = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","expansion","factor"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region expansion factor\" keyword(s).\n";
                return false;
              }
              m_InputData.mTrustRegionExpansionFactor = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","contraction","factor"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region contraction factor\" keyword(s).\n";
                return false;
              }
              m_InputData.mTrustRegionContractionFactor = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","gradient","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer gradient tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mOuterGradientToleranceKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","stationarity","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer stationarity tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mOuterStationarityToleranceKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mOuterStagnationToleranceKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","control","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer control stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mOuterControlStagnationToleranceKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","disable","post","smoothing"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks disable post smoothing\" keyword(s).\n";
                return false;
              }
              m_InputData.mDisablePostSmoothingKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","actual","reduction","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer actual reduction tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mOuterActualReductionToleranceKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","ratio","low"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region ratio low\" keyword(s).\n";
                return false;
              }
              m_InputData.mTrustRegionRatioLowKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","ratio","mid"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region ratio mid\" keyword(s).\n";
                return false;
              }
              m_InputData.mTrustRegionRatioMidKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","ratio","high"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region ratio high\" keyword(s).\n";
                return false;
              }
              m_InputData.mTrustRegionRatioUpperKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","initial","radius","scale"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks initial radius scale\" keyword(s).\n";
                return false;
              }
              m_InputData.mInitialRadiusScale = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","max","radius","scale"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks max radius scale\" keyword(s).\n";
                return false;
              }
              m_InputData.mMaxRadiusScale = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"problem","update","frequency"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"problem update frequency\" keyword(s).\n";
                return false;
              }
              m_InputData.mProblemUpdateFrequency = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","max","inner","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma max inner iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.mMaxInnerIterationsGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","inner","kkt","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma inner kkt tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mInnerKKTtoleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","inner","control","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma inner control stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mInnerControlStagnationToleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","kkt","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer kkt tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mOuterKKTtoleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","control","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma control stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mOuterControlStagnationToleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","objective","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer objective stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mOuterObjectiveStagnationToleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","stationarity","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer stationarity tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mOuterStationarityToleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","initial","moving","asymptotes","scale","factor"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma initial moving asymptotes scale factor\" keyword(s).\n";
                return false;
              }
              m_InputData.mInitialMovingAsymptotesScaleFactorGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"levelset","sphere","packing", "factor"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset sphere packing factor\" keyword(s).\n";
                return false;
              }
              m_InputData.levelset_sphere_packing_factor = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"levelset","sphere","radius"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset sphere radius\" keyword(s).\n";
                return false;
              }
              m_InputData.levelset_sphere_radius = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"levelset","nodesets"}, tStringValue))
            {
              if(tokens.size() < 3)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset nodesets\" keyword(s).\n";
                return false;
              }
              for(size_t j=2; j<tokens.size(); ++j)
              {
                m_InputData.levelset_nodesets.push_back(tokens[j]);
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"output","frequency"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"output frequency\" keyword(s).\n";
                return false;
              }
              m_InputData.output_frequency = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"output","method"}, tStringValue))
            {
              if(tokens.size() < 3)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"output method\" keyword(s).\n";
                return false;
              }
              m_InputData.output_method = tokens[2];
              for(size_t j=3; j<tokens.size(); ++j)
              {
                m_InputData.output_method += " ";
                m_InputData.output_method += tokens[j];
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"fixed","blocks"}, tStringValue))
            {
              if(tokens.size() < 3)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"fixed blocks\" keyword(s).\n";
                return false;
              }
              for(size_t j=2; j<tokens.size(); ++j)
              {
                m_InputData.fixed_block_ids.push_back(tokens[j]);
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"fixed","sidesets"}, tStringValue))
            {
              if(tokens.size() < 3)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"fixed sidesets\" keyword(s).\n";
                return false;
              }
              for(size_t j=2; j<tokens.size(); ++j)
              {
                m_InputData.fixed_sideset_ids.push_back(tokens[j]);
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"fixed","nodesets"}, tStringValue))
            {
              if(tokens.size() < 3)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"fixed nodesets\" keyword(s).\n";
                return false;
              }
              for(size_t j=2; j<tokens.size(); ++j)
              {
                m_InputData.fixed_nodeset_ids.push_back(tokens[j]);
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"number","processors"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number processors\" keyword(s).\n";
                return false;
              }
              m_InputData.num_opt_processors = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","type"}, tStringValue))
            {
              // retrieve input
              m_InputData.filter_type = "";
              for(size_t j=2; j<tokens.size(); ++j)
              {
                if(j!=2)
                {
                  m_InputData.filter_type += " ";
                }
                m_InputData.filter_type += tokens[j];
              }

              // check input is valid
              if(m_InputData.filter_type != m_InputData.m_filterType_identity_generatorName &&
                  m_InputData.filter_type != m_InputData.m_filterType_kernel_generatorName &&
                  m_InputData.filter_type != m_InputData.m_filterType_kernelThenHeaviside_generatorName &&
                  m_InputData.filter_type != m_InputData.m_filterType_kernelThenTANH_generatorName)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter type\" did not match allowed types which include:\n\t"
                  <<"\""<<m_InputData.m_filterType_identity_generatorName<<"\","
                  <<"\""<<m_InputData.m_filterType_kernel_generatorName<<"\","
                  <<"\""<<m_InputData.m_filterType_kernelThenHeaviside_generatorName<<"\""
                  <<"\""<<m_InputData.m_filterType_kernelThenTANH_generatorName<<"\""
                  <<".\n";
                return false;
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","heaviside","min"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter heaviside min\" keyword(s).\n";
                return false;
              }
              if(m_InputData.filter_heaviside_min!="")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside min\" both specified.\n"
                  <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
                return false;
              }
              m_InputData.filter_heaviside_min = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","heaviside","update"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter heaviside update\" keyword(s).\n";
                return false;
              }
              m_InputData.filter_heaviside_update = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","heaviside","max"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter heaviside max\" keyword(s).\n";
                return false;
              }
              if(m_InputData.filter_heaviside_max!="")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside max\" both specified."
                  <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
                return false;
              }
              m_InputData.filter_heaviside_max = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","heaviside","scale"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter heaviside scale\" keyword(s).\n";
                return false;
              }
              if(m_InputData.filter_heaviside_min!="")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside min\" both specified."
                  <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
                return false;
              }
              if(m_InputData.filter_heaviside_max!="")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside max\" both specified."
                  <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
                return false;
              }
              m_InputData.filter_heaviside_min = tStringValue;
              m_InputData.filter_heaviside_max = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","radial","power"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter power\" keyword(s).\n";
                return false;
              }
              m_InputData.filter_power = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","radius","scale"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter radius scale\" keyword(s).\n";
                return false;
              }
              m_InputData.filter_radius_scale = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","radius","absolute"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter radius absolute\" keyword(s).\n";
                return false;
              }
              m_InputData.filter_radius_absolute = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","projection","start","iteration"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter projection start iteration\" keyword(s).\n";
                return false;
              }
              m_InputData.filter_projection_start_iteration = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","projection","update","interval"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter projection update interval\" keyword(s).\n";
                return false;
              }
              m_InputData.filter_projection_update_interval = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","use","additive","continuation"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter use additive continuation\" keyword(s).\n";
                return false;
              }
              m_InputData.filter_use_additive_continuation = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"algorithm"}, tStringValue))
            {
              if(tokens.size() < 2)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"algorithm\" keyword(s).\n";
                return false;
              }
              m_InputData.optimization_algorithm = tokens[1];
              for(size_t j=2; j<tokens.size(); ++j)
              {
                m_InputData.optimization_algorithm += " ";
                m_InputData.optimization_algorithm += tokens[j];
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"discretization"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"discretization\" keyword(s).\n";
                return false;
              }
              m_InputData.discretization = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"check","gradient"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"check gradient\" keyword(s).\n";
                return false;
              }
              m_InputData.check_gradient = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"check","hessian"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"check hessian\" keyword(s).\n";
                return false;
              }
              m_InputData.check_hessian = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"hessian","type"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"hessian type\" keyword(s).\n";
                return false;
              }
              m_InputData.mHessianType = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"limited","memory","storage"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"limited memory storage\" keyword(s).\n";
                return false;
              }
              m_InputData.mLimitedMemoryStorage = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"use","mean","norm"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"use mean norm\" keyword(s).\n";
                return false;
              }
              m_InputData.mUseMeanNorm = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"al","penalty","parameter"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"al penalty parameter\" keyword(s).\n";
                return false;
              }
              m_InputData.mAugLagPenaltyParam = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"feasibility","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"feasibility tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.mFeasibilityTolerance = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"al","penalty","scale","factor"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"al penalty scale factor\" keyword(s).\n";
                return false;
              }
              m_InputData.mAugLagPenaltyParamScale = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"al","max","subproblem","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"al max subproblem iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.mMaxNumAugLagSubProbIter = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"max","trust","region","radius"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"max trust region radius\" keyword(s).\n";
                return false;
              }
              m_InputData.mMaxTrustRegionRadius = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks", "min","trust","region","radius"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks min trust region radius\" keyword(s).\n";
                return false;
              }
              m_InputData.mMinTrustRegionRadius = tStringValue;
            }
            else
            {
              PrintUnrecognizedTokens(tokens);
              std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Unrecognized keyword.\n";
              return false;
            }
          }
        }
      }
    }
  }

  // If there is a restart iteration but no filename specified then we will
  // assume we are coming from the gui and we will use the default "restart_XXX.exo"
  // filename, iteration 1 from the file, and field name "optimizationdofs".
  if(m_InputData.restart_iteration != "" &&
      m_InputData.restart_iteration != "0")
  {
    if(m_InputData.initial_guess_filename == "" &&
        m_InputData.initial_guess_field_name == "")
    {
      // This block indicates that we are coming from the gui so only the
      // restart iteration was specified.  We will fill in the other values
      // based on what we know the gui will be providing for the run.
      m_InputData.initial_guess_filename = "restart_" + m_InputData.restart_iteration + ".exo";
      m_InputData.restart_iteration = "1";
      m_InputData.initial_guess_field_name = "optimizationdofs";
    }
    else
    {
      // This block indicates that the user is manually setting up the
      // restart file and so we depend on him having specified a filename
      // and field name.  If either of these is empty we need to error out.
      if(m_InputData.initial_guess_field_name == "" ||
          m_InputData.initial_guess_filename == "")
      {
        std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: You must specify a valid initial guess mesh filename and a valid field name on that mesh from which initial values will be obtained.\n";
        return false;
      }
    }
  }

  // If "use normalization in aggregator" was not specified choose a default value based on
  // the optimization algorithm.
  if(m_InputData.mUseNormalizationInAggregator.length() == 0)
  {
    if(m_InputData.optimization_algorithm == "ksal" ||
        m_InputData.optimization_algorithm == "rol ksal")
    {
      m_InputData.mUseNormalizationInAggregator = "true";
    }
    else
    {
      m_InputData.mUseNormalizationInAggregator = "false";
    }
  }

  return true;
}

/******************************************************************************/
bool XMLGenerator::parseMesh(std::istream &fin)
/******************************************************************************/
{
  std::string tStringValue;
  std::vector<std::string> tInputStringList;

  // read each line of the file
  while (!fin.eof())
  {
    // read an entire line into memory
    char buf[MAX_CHARS_PER_LINE];
    fin.getline(buf, MAX_CHARS_PER_LINE);
    std::vector<std::string> tokens;
    parseTokens(buf, tokens);

    // process the tokens
    if(tokens.size() > 0)
    {
      for(size_t j=0; j<tokens.size(); ++j)
        tokens[j] = toLower(tokens[j]);

      if(parseSingleValue(tokens, tInputStringList = {"begin","mesh"}, tStringValue))
      {
        // found mesh block
        while (!fin.eof())
        {
          fin.getline(buf, MAX_CHARS_PER_LINE);
          tokens.clear();
          parseTokens(buf, tokens);
          // process the tokens
          if(tokens.size() > 0)
          {
            std::vector<std::string> unlowered_tokens = tokens;

            for(size_t j=0; j<tokens.size(); ++j)
              tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"end","mesh"}, tStringValue))
            {
              break;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"name"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseMesh: No value specified after \"name\" keyword.\n";
                return false;
              }
              m_InputData.mesh_name = tStringValue;

              // find last dot in filename, get mesh filename base from this
              size_t loc = tStringValue.find_last_of('.');
              if(loc == std::string::npos)
              {
                // mesh name: mesh_file
                // without extension: mesh_file
                m_InputData.mesh_name_without_extension = m_InputData.mesh_name;
              }
              else if(tStringValue[loc] == '.')
              {
                // mesh name: some_file.gen
                // without extension: some_file
                m_InputData.mesh_name_without_extension = tStringValue.substr(0,loc);
                m_InputData.mesh_extension = tStringValue.substr(loc);
              }
              else
              {
                // I don't know when this case will ever occur
                m_InputData.mesh_name_without_extension = m_InputData.mesh_name;
              }
            }
            else
            {
              PrintUnrecognizedTokens(tokens);
              std::cout << "ERROR:XMLGenerator:parseMesh: Unrecognized keyword.\n";
              return false;
            }
          }
        }
      }
    }
  }
  return true;
}
/******************************************************************************/
bool XMLGenerator::parseCodePaths(std::istream &fin)
/******************************************************************************/
{
  std::string tStringValue;
  std::vector<std::string> tInputStringList;

  // read each line of the file
  while (!fin.eof())
  {
    // read an entire line into memory
    char buf[MAX_CHARS_PER_LINE];
    fin.getline(buf, MAX_CHARS_PER_LINE);
    std::vector<std::string> tokens;
    parseTokens(buf, tokens);

    // process the tokens
    if(tokens.size() > 0)
    {
      for(size_t j=0; j<tokens.size(); ++j)
        tokens[j] = toLower(tokens[j]);

      if(parseSingleValue(tokens, tInputStringList = {"begin","paths"}, tStringValue))
      {
        // found mesh block
        while (!fin.eof())
        {
          fin.getline(buf, MAX_CHARS_PER_LINE);
          tokens.clear();
          parseTokens(buf, tokens);
          // process the tokens
          if(tokens.size() > 0)
          {
            std::vector<std::string> unlowered_tokens = tokens;

            for(size_t j=0; j<tokens.size(); ++j)
              tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"end","paths"}, tStringValue))
            {
              break;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","sierra_sd"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code sierra_sd\" keywords.\n";
                return false;
              }
              m_InputData.sierra_sd_path = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","platomain"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code platomain\" keywords.\n";
                return false;
              }
              m_InputData.plato_main_path = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","lightmp"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code lightmp\" keywords.\n";
                return false;
              }
              m_InputData.lightmp_path = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","prune_and_refine"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code prune_and_refine\" keywords.\n";
                return false;
              }
              m_InputData.prune_and_refine_path = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","albany"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code albany\" keywords.\n";
                return false;
              }
              m_InputData.albany_path = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","plato_analyze"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code plato_analyze\" keywords.\n";
                return false;
              }
              m_InputData.plato_analyze_path = tStringValue;
            }
            else
            {
              PrintUnrecognizedTokens(tokens);
              std::cout << "ERROR:XMLGenerator:parseCodePaths: Unrecognized keyword.\n";
              return false;
            }
          }
        }
      }
    }
  }
  return true;
}
/******************************************************************************/
bool XMLGenerator::parseBlocks(std::istream &fin)
/******************************************************************************/
{
  std::string tStringValue;
  std::vector<std::string> tInputStringList;

  // read each line of the file
  while (!fin.eof())
  {
    // read an entire line into memory
    char buf[MAX_CHARS_PER_LINE];
    fin.getline(buf, MAX_CHARS_PER_LINE);
    std::vector<std::string> tokens;
    parseTokens(buf, tokens);

    // process the tokens
    if(tokens.size() > 0)
    {
      for(size_t j=0; j<tokens.size(); ++j)
        tokens[j] = toLower(tokens[j]);

      if(parseSingleValue(tokens, tInputStringList = {"begin","block"}, tStringValue))
      {
        XMLGen::Block new_block;
        if(tStringValue == "")
        {
          std::cout << "ERROR:XMLGenerator:parseBlocks: No block id specified.\n";
          return false;
        }
        new_block.block_id = tStringValue;
        // found mesh block
        while (!fin.eof())
        {
          fin.getline(buf, MAX_CHARS_PER_LINE);
          tokens.clear();
          parseTokens(buf, tokens);
          // process the tokens
          if(tokens.size() > 0)
          {
            for(size_t j=0; j<tokens.size(); ++j)
              tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"end","block"}, tStringValue))
            {
              if(new_block.block_id == "")
              {
                std::cout << "ERROR:XMLGenerator:parseBlocks: Block id was not specified for block.\n";
                return false;
              }
              break;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"material"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseBlocks: No value specified after \"material\" keyword.\n";
                return false;
              }
              new_block.material_id = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"element","type"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseBlocks: No value specified after \"element type\" keywords.\n";
                return false;
              }
              new_block.element_type = tStringValue;
            }
            else
            {
              PrintUnrecognizedTokens(tokens);
              std::cout << "ERROR:XMLGenerator:parseBlocks: Unrecognized keyword.\n";
              return false;
            }
          }
        }
        m_InputData.blocks.push_back(new_block);
      }
    }
  }
  return true;
}
/******************************************************************************/
bool XMLGenerator::parseMaterials(std::istream &fin)
/******************************************************************************/
{
  std::string tStringValue;
  std::vector<std::string> tInputStringList;

  // read each line of the file
  while (!fin.eof())
  {
    // read an entire line into memory
    char buf[MAX_CHARS_PER_LINE];
    fin.getline(buf, MAX_CHARS_PER_LINE);
    std::vector<std::string> tokens;
    parseTokens(buf, tokens);

    // process the tokens
    if(tokens.size() > 0)
    {
      for(size_t j=0; j<tokens.size(); ++j)
        tokens[j] = toLower(tokens[j]);

      if(parseSingleValue(tokens, tInputStringList = {"begin","material"}, tStringValue))
      {
        XMLGen::Material new_material;
        new_material.property("penalty exponent", "3.0");
        if(tStringValue == "")
        {
          std::cout << "ERROR:XMLGenerator:parseMaterials: No material id specified.\n";
          return false;
        }
        new_material.id(tStringValue);
        // found mesh block
        while (!fin.eof())
        {
          fin.getline(buf, MAX_CHARS_PER_LINE);
          tokens.clear();
          parseTokens(buf, tokens);
          // process the tokens
          if(tokens.size() > 0)
          {
            for(size_t j=0; j<tokens.size(); ++j)
              tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"end","material"}, tStringValue))
            {
              if(new_material.id().empty())
              {
                std::cout << "ERROR:XMLGenerator:parseMaterials: Material ids was not specified for material.\n";
                return false;
              }
              break;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"penalty","exponent"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"penalty exponent\" keywords.\n";
                return false;
              }
              new_material.property("penalty exponent", tStringValue);
            }
            else if(parseSingleValue(tokens, tInputStringList = {"youngs","modulus"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"youngs modulus\" keywords.\n";
                return false;
              }
              new_material.property("youngs modulus", tStringValue);
            }
            else if(parseSingleValue(tokens, tInputStringList = {"specific","heat"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"specific heat\" keywords.\n";
                return false;
              }
              new_material.property("specific heat", tStringValue);
            }
            else if(parseSingleValue(tokens, tInputStringList = {"poissons","ratio"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"poissons ratio\" keywords.\n";
                return false;
              }
              new_material.property("poissons ratio", tStringValue);
            }
            else if(parseSingleValue(tokens, tInputStringList = {"thermal","conductivity"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"thermal conductivity coefficient\" keywords.\n";
                return false;
              }
              new_material.property("thermal conductivity", tStringValue);
            }
            else if(parseSingleValue(tokens, tInputStringList = {"thermal","expansion","coefficient"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"thermal expansion coefficient\" keywords.\n";
                return false;
              }
              new_material.property("thermal expansion", tStringValue);
            }
            else if(parseSingleValue(tokens, tInputStringList = {"reference","temperature"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"reference temperature\" keywords.\n";
                return false;
              }
              new_material.property("reference temperature", tStringValue);
            }
            else if(parseSingleValue(tokens, tInputStringList = {"density"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"density\" keywords.\n";
                return false;
              }
              new_material.property("density", tStringValue);
            }
            else
            {
              PrintUnrecognizedTokens(tokens);
              std::cout << "ERROR:XMLGenerator:parseMaterials: Unrecognized keyword.\n";
              return false;
            }
          }
        }
        m_InputData.materials.push_back(new_material);
      }
    }
  }
  return true;
}
/******************************************************************************/
bool XMLGenerator::parseConstraints(std::istream &fin)
/******************************************************************************/
{
  std::string tStringValue;
  std::vector<std::string> tInputStringList;

  // read each line of the file
  while (!fin.eof())
  {
    // read an entire line into memory
    char buf[MAX_CHARS_PER_LINE];
    fin.getline(buf, MAX_CHARS_PER_LINE);
    std::vector<std::string> tokens;
    parseTokens(buf, tokens);

    // process the tokens
    if(tokens.size() > 0)
    {
      for(size_t j=0; j<tokens.size(); ++j)
        tokens[j] = toLower(tokens[j]);

      if(parseSingleValue(tokens, tInputStringList = {"begin","constraint"}, tStringValue))
      {
        XMLGen::Constraint new_constraint;
        // found constraint
        while (!fin.eof())
        {
          fin.getline(buf, MAX_CHARS_PER_LINE);
          tokens.clear();
          parseTokens(buf, tokens);
          // process the tokens
          if(tokens.size() > 0)
          {
            for(size_t j=0; j<tokens.size(); ++j)
              tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"end","constraint"}, tStringValue))
            {
              break;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"type"}, tStringValue))
            {
              if(tokens.size() < 2)
              {
                std::cout << "ERROR:XMLGenerator:parseConstraints: Not enough params after \"type\" keyword.\n";
                return false;
              }
              new_constraint.type = tokens[1];
              for(size_t j=2; j<tokens.size(); ++j)
              {
                new_constraint.type += " ";
                new_constraint.type += tokens[j];
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"name"}, tStringValue))
            {
              if(tokens.size() < 2)
              {
                std::cout << "ERROR:XMLGenerator:parseConstraints: Not enough params after \"name\" keyword.\n";
                return false;
              }
              new_constraint.name = tokens[1];
              for(size_t j=2; j<tokens.size(); ++j)
              {
                new_constraint.name += " ";
                new_constraint.name += tokens[j];
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"volume","fraction"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseConstraints: Not value specified for \"volume fraction\".\n";
                return false;
              }
              new_constraint.volume_fraction = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"volume","absolute"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseConstraints: Not value specified for \"volume absolute\".\n";
                return false;
              }
              new_constraint.volume_absolute = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"surface","area","sideset","id"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseConstraints: Not value specified for \"surface area sideset id\".\n";
                return false;
              }
              new_constraint.surface_area_ssid = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"surface","area"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseConstraints: Not value specified for \"surface area\".\n";
                return false;
              }
              new_constraint.surface_area = tStringValue;
            }
            else
            {
              std::cout << "ERROR:XMLGenerator:parseConstraints: Invalid keyword.\n";
              return false;
            }
          }
        }
        m_InputData.constraints.push_back(new_constraint);
      }
    }
  }

  return true;
}

/******************************************************************************/
bool XMLGenerator::find_tokens(std::vector<std::string> &tokens,
    const int &start_index,
    const char *str1,
    const char *str2)
/******************************************************************************/
{
  return true;
}

/******************************************************************************/
bool XMLGenerator::parseFile()
/******************************************************************************/
{
  std::ifstream tInputFile;
  tInputFile.open(m_InputFilename.c_str()); // open a file
  if (!tInputFile.good())
  {
    std::cout << "Failed to open " << m_InputFilename << "." << std::endl;
    return false; // exit if file not found
  }

  parseBCs(tInputFile);
  tInputFile.close();
  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseLoads(tInputFile);
  tInputFile.close();
  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseObjectives(tInputFile);
  tInputFile.close();
  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseOptimizationParameters(tInputFile);
  tInputFile.close();
  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseConstraints(tInputFile);
  tInputFile.close();
  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseMesh(tInputFile);
  tInputFile.close();
  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseMaterials(tInputFile);
  tInputFile.close();
  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseBlocks(tInputFile);
  tInputFile.close();
  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseCodePaths(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseUncertainties(tInputFile);
  tInputFile.close();

  // If we will need to run the prune_and_refine executable for any
  // reason we need to have our "run" mesh name not be the same
  // as the input mesh name.
  int tNumRefines = 0;
  if(m_InputData.number_refines != "")
    tNumRefines = std::atoi(m_InputData.number_refines.c_str());
  if(tNumRefines > 0 ||
      (m_InputData.initial_guess_filename != "" && m_InputData.initial_guess_field_name != ""))
  {
    m_InputData.run_mesh_name_without_extension = m_InputData.mesh_name_without_extension + "_mod";
    m_InputData.run_mesh_name = m_InputData.run_mesh_name_without_extension;
    if(m_InputData.mesh_extension != "")
      m_InputData.run_mesh_name += m_InputData.mesh_extension;
  }
  else
  {
    m_InputData.run_mesh_name = m_InputData.mesh_name;
    m_InputData.run_mesh_name_without_extension = m_InputData.mesh_name_without_extension;
  }

  return true;
}

/******************************************************************************/
std::string XMLGenerator::toLower(const std::string &s)
/******************************************************************************/
{
  char buffer[500];
  std::string ret;
  if(s.size() > 500)
    ret = "";
  else
  {
    size_t i;
    for(i=0; i<s.size(); ++i)
    {
      buffer[i] = tolower(s[i]);
    }
    buffer[i] = '\0';
    ret = buffer;
  }
  return ret;
}

/******************************************************************************/
std::string XMLGenerator::toUpper(const std::string &s)
/******************************************************************************/
{
  char buffer[500];
  std::string ret;
  if(s.size() > 500)
    ret = "";
  else
  {
    size_t i;
    for(i=0; i<s.size(); ++i)
    {
      buffer[i] = toupper(s[i]);
    }
    buffer[i] = '\0';
    ret = buffer;
  }
  return ret;
}

/******************************************************************************/
void XMLGenerator::getUncertaintyFlags()
/******************************************************************************/
{
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        const XMLGen::Objective cur_obj = m_InputData.objectives[i];
        for(size_t k=0; k<cur_obj.load_case_ids.size(); k++)
        {
            std::string cur_load_string = cur_obj.load_case_ids[k];
            for(size_t j=0; m_InputData.m_RequestedVonMisesOutput == false && j<cur_obj.output_for_plotting.size(); j++)
            {
                if(cur_obj.output_for_plotting[j] == "vonmises")
                {
                    m_InputData.m_RequestedVonMisesOutput = true;
                }
            }
            for(size_t j=0; m_InputData.m_HasUncertainties == false && j<m_InputData.uncertainties.size(); ++j)
            {
                if(cur_load_string == m_InputData.uncertainties[j].id)
                {
                    m_InputData.m_HasUncertainties = true;
                }
            }
        }
    }
}




}


