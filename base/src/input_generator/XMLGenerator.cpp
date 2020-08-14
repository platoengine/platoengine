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

#include "XMLGenerator.hpp"

// #include "Plato_SromXML.hpp"
// #include "Plato_SromXMLGenTools.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorParserUtilities.hpp"
// #include "Plato_SolveUncertaintyProblem.hpp"
// #include "Plato_UniqueCounter.hpp"
// #include "Plato_Vector3DVariations.hpp"
// #include "Plato_FreeFunctions.hpp"
#include "XMLG_Macros.hpp"

// #include "XMLGeneratorPlatoAnalyzeProblem.hpp"
#include "XMLGeneratorValidInputKeys.hpp"

#include "XMLGeneratorParseOutput.hpp"
// #include "XMLGeneratorParseService.hpp"
#include "XMLGeneratorParseMaterial.hpp"
#include "XMLGeneratorParseCriteria.hpp"
// #include "XMLGeneratorParseObjective.hpp"
// #include "XMLGeneratorParseConstraint.hpp"
#include "XMLGeneratorParseUncertainty.hpp"

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
  // m_InputData.m_UseLaunch = use_launch;
  // m_InputData.m_Arch = arch;
  // m_InputData.m_filterType_identity_generatorName = "identity";
  // m_InputData.m_filterType_identity_XMLName = "Identity";
  // m_InputData.m_filterType_kernel_generatorName = "kernel";
  // m_InputData.m_filterType_kernel_XMLName = "Kernel";
  // m_InputData.m_filterType_kernelThenHeaviside_generatorName = "kernel then heaviside";
  // m_InputData.m_filterType_kernelThenHeaviside_XMLName = "KernelThenHeaviside";
  // m_InputData.m_filterType_kernelThenTANH_generatorName = "kernel then tanh";
  // m_InputData.m_filterType_kernelThenTANH_XMLName = "KernelThenTANH";
  // m_InputData.m_HasUncertainties = false;
  // m_InputData.m_RequestedVonMisesOutput = false;
}

/******************************************************************************/
XMLGenerator::~XMLGenerator() 
/******************************************************************************/
{
}

// /******************************************************************************//**
//  * \fn writeInputFiles
//  * \brief Write input files, i.e. write all the XML files needed by Plato.
// **********************************************************************************/
// void XMLGenerator::writeInputFiles()
// {
//     XMLGen::Analyze::write_optimization_problem(m_InputData);
// }

/******************************************************************************/
void XMLGenerator::generate()
/******************************************************************************/
{
    this->parseInputFile();

    // this->getUncertaintyFlags();

    // if(!runSROMForUncertainVariables())
    // {
    //     PRINTERR("Failed to expand uncertainties in file generation.")
    //     return false;
    // }

    // this->writeInputFiles();
}

// /******************************************************************************/
// bool XMLGenerator::runSROMForUncertainVariables()
// /******************************************************************************/
// {
//     if(m_InputData.uncertainties.size() > 0)
//     {
//         Plato::srom::solve(m_InputData);
//         this->setNumPerformers();
//     }

//     return true;
// }

// /******************************************************************************/
// void XMLGenerator::setNumPerformers()
// /******************************************************************************/
// {
//     m_InputData.m_UncertaintyMetaData.numPerformers = std::stoi(m_InputData.services()[0].numberRanks());

//     if (m_InputData.mRandomMetaData.numSamples() % m_InputData.m_UncertaintyMetaData.numPerformers != 0)
//     {
//         THROWERR("Set Number for Performers: Number of samples must divide evenly into number of ranks.");
//     }
// }

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
void XMLGenerator::parseOutput(std::istream &aInputFile)
/******************************************************************************/
{
    XMLGen::ParseOutput tParseOutput;
    tParseOutput.parse(aInputFile);
    m_InputData.mOutputMetaData = tParseOutput.data();
}

// /******************************************************************************/
// bool XMLGenerator::parseService(std::istream &aInputFile)
// /******************************************************************************/
// {
//     XMLGen::ParseService tParseService;
//     tParseService.parse(aInputFile);
//     auto tServices = tParseService.data();
//     m_InputData.set(tServices);
//     return true;
// }

// /******************************************************************************/
// bool XMLGenerator::parseObjective(std::istream &aInputFile)
// /******************************************************************************/
// {
//     XMLGen::ParseObjective tParseObjective;
//     tParseObjective.parse(aInputFile);
//     m_InputData.objective = tParseObjective.data();
//     return true;
// }

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
      m_InputData.loads.push_back(new_load);

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
void XMLGenerator::parseUncertainties(std::istream &aInputFile)
/******************************************************************************/
{
    XMLGen::ParseUncertainty tParseUncertainty;
    tParseUncertainty.parse(aInputFile);
    m_InputData.uncertainties = tParseUncertainty.data();
}

/******************************************************************************/
void XMLGenerator::parseBCs(std::istream &fin)
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
              this->parseBCsBlock(fin);
              bc_block_found = true;
            }
        }
    }

    if(!bc_block_found)
        THROWERR("ERROR:XMLGenerator:parseBCs: No boundary condition block found \n")
}

/******************************************************************************/
void XMLGenerator::parseBCsBlock(std::istream &fin)
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
        parseBCLine(tokens);
    }
  }
}

/******************************************************************************/
void XMLGenerator::parseBCLine(std::vector<std::string>& tokens)
/******************************************************************************/
{
    XMLGen::BC new_bc;

    if(tokens.size() < 7)
    {
        THROWERR("ERROR:XMLGenerator:parseBCs: Not enough parameters were specified for BC in \"boundary conditions\" block.\n")
    }

    XMLGen::ValidEssentialBoundaryConditionsKeys tValidKeys;
    auto tLowerKey = XMLGen::to_lower(tokens[0]);
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerKey);
    if (tItr == tValidKeys.mKeys.end())
    {
        THROWERR("ERROR:XMLGenerator:parseBCs: Essential boundary condition with tag '" + tLowerKey + "' is not supported.")
    }
    new_bc.type = tokens[1];

    if(!new_bc.type.compare("displacement"))
      parseDisplacementBC(tokens,new_bc);
    else if(!new_bc.type.compare("temperature"))
      parseTemperatureBC(tokens,new_bc);
    else
    {
        PrintUnrecognizedTokens(tokens);
        THROWERR("ERROR:XMLGenerator:parseLoads: Unrecognized boundary condition type.\n")
    }

    m_InputData.bcs.push_back(new_bc);
}

/******************************************************************************/
void XMLGenerator::parseDisplacementBC(std::vector<std::string>& tokens, XMLGen::BC& new_bc)
/******************************************************************************/
{
  // Potential syntax:
  // fixed displacement nodeset/sideset 1 bc id 1                 // all dofs have fixed disp of 0.0
  // fixed displacement nodeset/sideset 1 <x,y,z> bc id 1         // x, y, or z dof has fixed disp of 0.0
  // fixed displacement nodeset/sideset 1 <x,y,z> 3.0 bc id 1     // x, y, or z dof has fixed disp of 3.0

  size_t tTokenIndex = 1;
  bool tNameOrIDSpecified = false;

  new_bc.mCategory = tokens[0];
  if(tokens[++tTokenIndex] != "nodeset" && tokens[tTokenIndex] != "sideset")
  {
      THROWERR("ERROR:XMLGenerator:parseBCs: Boundary conditions can only be applied to \"nodeset\" or \"sideset\" types.\n")
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
        THROWERR("ERROR:XMLGenerator:parseBCs: Boundary condition degree of freedom must be either \"x\", \"y\", or \"z\".\n")
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
}

/******************************************************************************/
void XMLGenerator::parseTemperatureBC(std::vector<std::string>& tokens, XMLGen::BC& new_bc)
/******************************************************************************/
{
  // Potential syntax:
  // fixed temperature nodeset 1 bc id 1
  // fixed temperature nodeset 1 value 25.0 bc id 1
  if(tokens[2] != "nodeset" && tokens[2] != "sideset")
  {
      THROWERR("ERROR:XMLGenerator:parseBCs: Boundary conditions can only be applied to \"nodeset\" or \"sideset\" types.\n")
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
          THROWERR("ERROR:XMLGenerator:parseBCs: Invalid BC syntax.\n")
      }
      new_bc.value = tokens[5+tOffset];
      if(tokens[6+tOffset] != "bc")
      {
          THROWERR("ERROR:XMLGenerator:parseBCs: Invalid BC syntax.\n")
      }
      new_bc.bc_id = tokens[8+tOffset];
  }
  else
      new_bc.bc_id = tokens[6+tOffset];
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

// /******************************************************************************/
// void XMLGenerator::initializePlatoProblemOptions()
// /******************************************************************************/
// {
//     m_InputData.output_frequency="5";
//     m_InputData.discretization="density";
//     m_InputData.initial_density_value="0.5";
//     m_InputData.optimization_algorithm="oc";
//     m_InputData.output_method="epu";
//     m_InputData.check_gradient = "false";
//     m_InputData.check_hessian = "false";
//     m_InputData.filter_type = "kernel";
//     m_InputData.filter_power = "1";

//     m_InputData.mInnerKKTtoleranceGCMMA = "";
//     m_InputData.mOuterKKTtoleranceGCMMA = "";
//     m_InputData.mInnerControlStagnationToleranceGCMMA = "";
//     m_InputData.mOuterControlStagnationToleranceGCMMA = "";
//     m_InputData.mOuterObjectiveStagnationToleranceGCMMA = "";
//     m_InputData.mMaxInnerIterationsGCMMA = "";
//     m_InputData.mOuterStationarityToleranceGCMMA = "";
//     m_InputData.mInitialMovingAsymptotesScaleFactorGCMMA = "";

//     m_InputData.mMaxRadiusScale = "";
//     m_InputData.mInitialRadiusScale = "";
//     m_InputData.mMaxTrustRegionRadius = "";
//     m_InputData.mMinTrustRegionRadius = "";
//     m_InputData.mMaxTrustRegionIterations = "5";
//     m_InputData.mTrustRegionExpansionFactor = "";
//     m_InputData.mTrustRegionContractionFactor = "";
//     m_InputData.mTrustRegionRatioLowKS = "";
//     m_InputData.mTrustRegionRatioMidKS = "";
//     m_InputData.mTrustRegionRatioUpperKS = "";

//     m_InputData.mUseMeanNorm = "";
//     m_InputData.mAugLagPenaltyParam = "";
//     m_InputData.mFeasibilityTolerance = "";
//     m_InputData.mAugLagPenaltyParamScale = "";
//     m_InputData.mMaxNumAugLagSubProbIter = "";

//     m_InputData.mHessianType = "disabled";
//     if(m_InputData.mHessianType.compare("lbfgs") == 0)
//     {
//         m_InputData.mLimitedMemoryStorage = "8";
//     }
//     m_InputData.mDisablePostSmoothingKS = "true";
//     m_InputData.mProblemUpdateFrequency = "5";
//     m_InputData.mOuterGradientToleranceKS = "";
//     m_InputData.mOuterStationarityToleranceKS = "";
//     m_InputData.mOuterStagnationToleranceKS = "";
//     m_InputData.mOuterControlStagnationToleranceKS = "";
//     m_InputData.mOuterActualReductionToleranceKS = "";

//     m_InputData.filter_heaviside_min = "";
//     m_InputData.filter_heaviside_update = "";
//     m_InputData.filter_heaviside_max = "";

//     m_InputData.filter_projection_start_iteration = "";
//     m_InputData.filter_projection_update_interval = "";
//     m_InputData.filter_use_additive_continuation = "";

//     m_InputData.write_restart_file = "False";
//     m_InputData.optimization_type = "topology";
// }

// /******************************************************************************/
// bool XMLGenerator::parseOptimizationParameters(std::istream &fin)
// /******************************************************************************/
// {
//   // Initialize variables
//   this->initializePlatoProblemOptions();

//   std::string tStringValue;
//   std::vector<std::string> tInputStringList;
//   // read each line of the file
//   while (!fin.eof())
//   {
//     // read an entire line into memory
//     char buf[MAX_CHARS_PER_LINE];
//     fin.getline(buf, MAX_CHARS_PER_LINE);
//     std::vector<std::string> tokens;
//     parseTokens(buf, tokens);

//     // process the tokens
//     if(tokens.size() > 0)
//     {
//       for(size_t j=0; j<tokens.size(); ++j)
//         tokens[j] = toLower(tokens[j]);

//       if(parseSingleValue(tokens, tInputStringList = {"begin","optimization","parameters"}, tStringValue))
//       {
//         // found optimization parameters
//         while (!fin.eof())
//         {
//           fin.getline(buf, MAX_CHARS_PER_LINE);
//           tokens.clear();
//           parseTokens(buf, tokens);
//           // process the tokens
//           if(tokens.size() > 0)
//           {
//             std::vector<std::string> unlowered_tokens = tokens;

//             for(size_t j=0; j<tokens.size(); ++j)
//               tokens[j] = toLower(tokens[j]);

//             if(parseSingleValue(tokens, tInputStringList = {"end","optimization","parameters"}, tStringValue))
//             {
//               break;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"begin","material_box"}, tStringValue))
//             {
//               while (!fin.eof())
//               {
//                 fin.getline(buf, MAX_CHARS_PER_LINE);
//                 tokens.clear();
//                 parseTokens(buf, tokens);
//                 // process the tokens
//                 if(tokens.size() > 0)
//                 {
//                   for(size_t j=0; j<tokens.size(); ++j)
//                     tokens[j] = toLower(tokens[j]);

//                   if(parseSingleValue(tokens, tInputStringList = {"end","material_box"}, tStringValue))
//                   {
//                     if(m_InputData.levelset_material_box_min == "" ||
//                         m_InputData.levelset_material_box_max == "")
//                     {
//                       std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Not enough info in \"material_box\" block.\n";
//                       return false;
//                     }
//                     break;
//                   }
//                   else if(parseSingleValue(tokens, tInputStringList = {"min", "coords"}, tStringValue))
//                   {
//                     if(tokens.size() != 5)
//                     {
//                       std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Wrong number of params in \"min coords\" line.\n";
//                       return false;
//                     }
//                     for(size_t j=2; j<5; ++j)
//                     {
//                       m_InputData.levelset_material_box_min += tokens[j];
//                       if(j<4)
//                         m_InputData.levelset_material_box_min += " ";
//                     }
//                   }
//                   else if(parseSingleValue(tokens, tInputStringList = {"max", "coords"}, tStringValue))
//                   {
//                     if(tokens.size() != 5)
//                     {
//                       std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Wrong number of params in \"max coords\" line.\n";
//                       return false;
//                     }
//                     for(size_t j=2; j<5; ++j)
//                     {
//                       m_InputData.levelset_material_box_max += tokens[j];
//                       if(j<4)
//                         m_InputData.levelset_material_box_max += " ";
//                     }
//                   }
//                 }
//               }
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"objective","number","standard","deviations"}, tStringValue))
//             {
//               if(tokens.size() < 5)
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"objective number standard deviations\" keywords.\n";
//                 return false;
//               }
//               m_InputData.objective_number_standard_deviations = tokens[4];
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"output", "standard","deviations"}, tStringValue))
//             {
//               if(tokens.size() < 4)
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No outputs specified after \"output standard deviations\" keywords.\n";
//                 return false;
//               }
//               for(size_t j=3; j<tokens.size(); ++j)
//               {
//                 m_InputData.mStandardDeviations.push_back(tokens[j]);
//               }
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"input","generator","version"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"input generator version\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.input_generator_version = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"initial","density","value"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial density value\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.initial_density_value = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"optimization","type"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"optimization type\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.optimization_type = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"csm","file"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"csm file\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.csm_filename = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"use","normalization","in","aggregator"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"use normalization in aggregator\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mUseNormalizationInAggregator = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"create","levelset","spheres"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"create levelset spheres\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.create_levelset_spheres = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"write","restart","file"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"write restart file\" keyword(s).\n";
//                 return false;
//               }
//               if(tStringValue == "true")
//                 m_InputData.write_restart_file = "True";
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"levelset","initialization","method"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset initialization method\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.levelset_initialization_method = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"max","iterations"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"max iterations\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.max_iterations = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"restart","iteration"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"restart iteration\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.restart_iteration = tStringValue;
//             }
//             else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"initial","guess","filename"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial guess filename\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.initial_guess_filename = tStringValue;
//             }
//             else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"initial","guess","field","name"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial guess field name\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.initial_guess_field_name = tStringValue;
//             }
//             else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"prune","mesh"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"prune mesh\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.prune_mesh = tStringValue;
//             }
//             else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","buffer","layers"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number buffer layers\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.number_buffer_layers = tStringValue;
//             }
//             else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","prune","and","refine","processors"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number prune and refine processors\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.number_prune_and_refine_processors = tStringValue;
//             }
//             else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","refines"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number refines\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.number_refines = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"verbose"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"verbose\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mVerbose = XMLGen::to_lower(tStringValue);
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"mma","move","limit"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma move limit\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMMAMoveLimit = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"mma","control","stagnation","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma control stagnation tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMMAControlStagnationTolerance = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"mma","objective","stagnation","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma objective stagnation tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMMAObjectiveStagnationTolerance = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"mma","asymptote","expansion"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma asymptote expansion\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMMAAsymptoteExpansion = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"mma","asymptote","contraction"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma asymptote contraction\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMMAAsymptoteContraction = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"mma","max","sub","problem","iterations"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma max sub problem iterations\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMMAMaxNumSubProblemIterations = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"mma","max","trust","region","iterations"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma max trust region iterations\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMMAMaxTrustRegionIterations = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","max","trust","region","iterations"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks max trust region iterations\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMaxTrustRegionIterations = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","expansion","factor"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region expansion factor\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mTrustRegionExpansionFactor = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","contraction","factor"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region contraction factor\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mTrustRegionContractionFactor = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","gradient","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer gradient tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mOuterGradientToleranceKS = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","stationarity","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer stationarity tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mOuterStationarityToleranceKS = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","stagnation","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer stagnation tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mOuterStagnationToleranceKS = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","control","stagnation","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer control stagnation tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mOuterControlStagnationToleranceKS = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","disable","post","smoothing"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks disable post smoothing\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mDisablePostSmoothingKS = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","actual","reduction","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer actual reduction tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mOuterActualReductionToleranceKS = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","ratio","low"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region ratio low\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mTrustRegionRatioLowKS = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","ratio","mid"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region ratio mid\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mTrustRegionRatioMidKS = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","ratio","high"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region ratio high\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mTrustRegionRatioUpperKS = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","initial","radius","scale"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks initial radius scale\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mInitialRadiusScale = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks","max","radius","scale"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks max radius scale\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMaxRadiusScale = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"problem","update","frequency"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"problem update frequency\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mProblemUpdateFrequency = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"gcmma","max","inner","iterations"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma max inner iterations\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMaxInnerIterationsGCMMA = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"gcmma","inner","kkt","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma inner kkt tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mInnerKKTtoleranceGCMMA = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"gcmma","inner","control","stagnation","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma inner control stagnation tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mInnerControlStagnationToleranceGCMMA = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","kkt","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer kkt tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mOuterKKTtoleranceGCMMA = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","control","stagnation","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma control stagnation tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mOuterControlStagnationToleranceGCMMA = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","objective","stagnation","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer objective stagnation tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mOuterObjectiveStagnationToleranceGCMMA = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","stationarity","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer stationarity tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mOuterStationarityToleranceGCMMA = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"gcmma","initial","moving","asymptotes","scale","factor"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma initial moving asymptotes scale factor\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mInitialMovingAsymptotesScaleFactorGCMMA = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"levelset","sphere","packing", "factor"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset sphere packing factor\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.levelset_sphere_packing_factor = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"levelset","sphere","radius"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset sphere radius\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.levelset_sphere_radius = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"levelset","nodesets"}, tStringValue))
//             {
//               if(tokens.size() < 3)
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset nodesets\" keyword(s).\n";
//                 return false;
//               }
//               for(size_t j=2; j<tokens.size(); ++j)
//               {
//                 m_InputData.levelset_nodesets.push_back(tokens[j]);
//               }
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"output","frequency"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"output frequency\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.output_frequency = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"output","method"}, tStringValue))
//             {
//               if(tokens.size() < 3)
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"output method\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.output_method = tokens[2];
//               for(size_t j=3; j<tokens.size(); ++j)
//               {
//                 m_InputData.output_method += " ";
//                 m_InputData.output_method += tokens[j];
//               }
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"fixed","blocks"}, tStringValue))
//             {
//               if(tokens.size() < 3)
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"fixed blocks\" keyword(s).\n";
//                 return false;
//               }
//               for(size_t j=2; j<tokens.size(); ++j)
//               {
//                 m_InputData.fixed_block_ids.push_back(tokens[j]);
//               }
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"fixed","sidesets"}, tStringValue))
//             {
//               if(tokens.size() < 3)
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"fixed sidesets\" keyword(s).\n";
//                 return false;
//               }
//               for(size_t j=2; j<tokens.size(); ++j)
//               {
//                 m_InputData.fixed_sideset_ids.push_back(tokens[j]);
//               }
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"fixed","nodesets"}, tStringValue))
//             {
//               if(tokens.size() < 3)
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"fixed nodesets\" keyword(s).\n";
//                 return false;
//               }
//               for(size_t j=2; j<tokens.size(); ++j)
//               {
//                 m_InputData.fixed_nodeset_ids.push_back(tokens[j]);
//               }
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"number","processors"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number processors\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.num_opt_processors = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"filter","type"}, tStringValue))
//             {
//               // retrieve input
//               m_InputData.filter_type = "";
//               for(size_t j=2; j<tokens.size(); ++j)
//               {
//                 if(j!=2)
//                 {
//                   m_InputData.filter_type += " ";
//                 }
//                 m_InputData.filter_type += tokens[j];
//               }

//               // check input is valid
//               if(m_InputData.filter_type != m_InputData.m_filterType_identity_generatorName &&
//                   m_InputData.filter_type != m_InputData.m_filterType_kernel_generatorName &&
//                   m_InputData.filter_type != m_InputData.m_filterType_kernelThenHeaviside_generatorName &&
//                   m_InputData.filter_type != m_InputData.m_filterType_kernelThenTANH_generatorName)
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter type\" did not match allowed types which include:\n\t"
//                   <<"\""<<m_InputData.m_filterType_identity_generatorName<<"\","
//                   <<"\""<<m_InputData.m_filterType_kernel_generatorName<<"\","
//                   <<"\""<<m_InputData.m_filterType_kernelThenHeaviside_generatorName<<"\""
//                   <<"\""<<m_InputData.m_filterType_kernelThenTANH_generatorName<<"\""
//                   <<".\n";
//                 return false;
//               }
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"filter","heaviside","min"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter heaviside min\" keyword(s).\n";
//                 return false;
//               }
//               if(m_InputData.filter_heaviside_min!="")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside min\" both specified.\n"
//                   <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
//                 return false;
//               }
//               m_InputData.filter_heaviside_min = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"filter","heaviside","update"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter heaviside update\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.filter_heaviside_update = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"filter","heaviside","max"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter heaviside max\" keyword(s).\n";
//                 return false;
//               }
//               if(m_InputData.filter_heaviside_max!="")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside max\" both specified."
//                   <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
//                 return false;
//               }
//               m_InputData.filter_heaviside_max = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"filter","heaviside","scale"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter heaviside scale\" keyword(s).\n";
//                 return false;
//               }
//               if(m_InputData.filter_heaviside_min!="")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside min\" both specified."
//                   <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
//                 return false;
//               }
//               if(m_InputData.filter_heaviside_max!="")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside max\" both specified."
//                   <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
//                 return false;
//               }
//               m_InputData.filter_heaviside_min = tStringValue;
//               m_InputData.filter_heaviside_max = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"filter","radial","power"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter power\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.filter_power = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"filter","radius","scale"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter radius scale\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.filter_radius_scale = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"filter","radius","absolute"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter radius absolute\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.filter_radius_absolute = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"filter","projection","start","iteration"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter projection start iteration\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.filter_projection_start_iteration = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"filter","projection","update","interval"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter projection update interval\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.filter_projection_update_interval = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"filter","use","additive","continuation"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter use additive continuation\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.filter_use_additive_continuation = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"algorithm"}, tStringValue))
//             {
//               if(tokens.size() < 2)
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"algorithm\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.optimization_algorithm = tokens[1];
//               for(size_t j=2; j<tokens.size(); ++j)
//               {
//                 m_InputData.optimization_algorithm += " ";
//                 m_InputData.optimization_algorithm += tokens[j];
//               }
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"discretization"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"discretization\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.discretization = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"check","gradient"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"check gradient\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.check_gradient = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"check","hessian"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"check hessian\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.check_hessian = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"hessian","type"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"hessian type\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mHessianType = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"limited","memory","storage"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"limited memory storage\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mLimitedMemoryStorage = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"use","mean","norm"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"use mean norm\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mUseMeanNorm = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"al","penalty","parameter"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"al penalty parameter\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mAugLagPenaltyParam = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"feasibility","tolerance"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"feasibility tolerance\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mFeasibilityTolerance = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"al","penalty","scale","factor"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"al penalty scale factor\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mAugLagPenaltyParamScale = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"al","max","subproblem","iterations"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"al max subproblem iterations\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMaxNumAugLagSubProbIter = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"max","trust","region","radius"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"max trust region radius\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMaxTrustRegionRadius = tStringValue;
//             }
//             else if(parseSingleValue(tokens, tInputStringList = {"ks", "min","trust","region","radius"}, tStringValue))
//             {
//               if(tStringValue == "")
//               {
//                 std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks min trust region radius\" keyword(s).\n";
//                 return false;
//               }
//               m_InputData.mMinTrustRegionRadius = tStringValue;
//             }
//             else
//             {
//               PrintUnrecognizedTokens(tokens);
//               std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Unrecognized keyword.\n";
//               return false;
//             }
//           }
//         }
//       }
//     }
//   }

//   // If there is a restart iteration but no filename specified then we will
//   // assume we are coming from the gui and we will use the default "restart_XXX.exo"
//   // filename, iteration 1 from the file, and field name "optimizationdofs".
//   if(m_InputData.restart_iteration != "" &&
//       m_InputData.restart_iteration != "0")
//   {
//     if(m_InputData.initial_guess_filename == "" &&
//         m_InputData.initial_guess_field_name == "")
//     {
//       // This block indicates that we are coming from the gui so only the
//       // restart iteration was specified.  We will fill in the other values
//       // based on what we know the gui will be providing for the run.
//       m_InputData.initial_guess_filename = "restart_" + m_InputData.restart_iteration + ".exo";
//       m_InputData.restart_iteration = "1";
//       m_InputData.initial_guess_field_name = "optimizationdofs";
//     }
//     else
//     {
//       // This block indicates that the user is manually setting up the
//       // restart file and so we depend on him having specified a filename
//       // and field name.  If either of these is empty we need to error out.
//       if(m_InputData.initial_guess_field_name == "" ||
//           m_InputData.initial_guess_filename == "")
//       {
//         std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: You must specify a valid initial guess mesh filename and a valid field name on that mesh from which initial values will be obtained.\n";
//         return false;
//       }
//     }
//   }

//   // If "use normalization in aggregator" was not specified choose a default value based on
//   // the optimization algorithm.
//   if(m_InputData.mUseNormalizationInAggregator.length() == 0)
//   {
//     if(m_InputData.optimization_algorithm == "ksal" ||
//         m_InputData.optimization_algorithm == "rol ksal")
//     {
//       m_InputData.mUseNormalizationInAggregator = "true";
//     }
//     else
//     {
//       m_InputData.mUseNormalizationInAggregator = "false";
//     }
//   }

//   return true;
// }

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
              m_InputData.mesh.name = tStringValue;

              // find last dot in filename, get mesh filename base from this
              size_t loc = tStringValue.find_last_of('.');
              if(loc == std::string::npos)
              {
                // mesh name: mesh_file
                // without extension: mesh_file
                m_InputData.mesh.name_without_extension = m_InputData.mesh.name;
              }
              else if(tStringValue[loc] == '.')
              {
                // mesh name: some_file.gen
                // without extension: some_file
                m_InputData.mesh.name_without_extension = tStringValue.substr(0,loc);
                m_InputData.mesh.file_extension = tStringValue.substr(loc);
              }
              else
              {
                // I don't know when this case will ever occur
                m_InputData.mesh.name_without_extension = m_InputData.mesh.name;
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
              m_InputData.codepaths.sierra_sd_path = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","platomain"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code platomain\" keywords.\n";
                return false;
              }
              m_InputData.codepaths.plato_main_path = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","lightmp"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code lightmp\" keywords.\n";
                return false;
              }
              m_InputData.codepaths.lightmp_path = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","prune_and_refine"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code prune_and_refine\" keywords.\n";
                return false;
              }
              m_InputData.codepaths.prune_and_refine_path = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","albany"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code albany\" keywords.\n";
                return false;
              }
              m_InputData.codepaths.albany_path = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","plato_analyze"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code plato_analyze\" keywords.\n";
                return false;
              }
              m_InputData.codepaths.plato_analyze_path = tStringValue;
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
void XMLGenerator::parseMaterials(std::istream &aInput)
/******************************************************************************/
{
    XMLGen::ParseMaterial tParseMaterial;
    tParseMaterial.parse(aInput);
    m_InputData.materials = tParseMaterial.data();
}

/******************************************************************************/
void XMLGenerator::parseCriteria(std::istream &aInput)
/******************************************************************************/
{
    XMLGen::ParseCriteria tParseCriteria;
    tParseCriteria.parse(aInput);
    m_InputData.criteria = tParseCriteria.data();
}

// /******************************************************************************/
// bool XMLGenerator::parseConstraints(std::istream &aInput)
// /******************************************************************************/
// {
//     XMLGen::ParseConstraint tParseConstraint;
//     tParseConstraint.parse(aInput);
//     m_InputData.constraints = tParseConstraint.data();
//     return true;
// }

// /******************************************************************************/
// bool XMLGenerator::find_tokens(std::vector<std::string> &tokens,
//     const int &start_index,
//     const char *str1,
//     const char *str2)
// /******************************************************************************/
// {
//   return true;
// }

/******************************************************************************/
void XMLGenerator::parseInputFile()
/******************************************************************************/
{
  std::ifstream tInputFile;
  tInputFile.open(m_InputFilename.c_str()); // open a file
  if (!tInputFile.good())
  {
      THROWERR("Failed to open " + m_InputFilename + ".")
  }

  parseBCs(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseLoads(tInputFile);
  tInputFile.close();

  // tInputFile.open(m_InputFilename.c_str()); // open a file
  // parseOptimizationParameters(tInputFile);
  // tInputFile.close();
  
  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseMesh(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseBlocks(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseCodePaths(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseMaterials(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseCriteria(tInputFile);
  tInputFile.close();

  // tInputFile.open(m_InputFilename.c_str()); // open a file
  // this->parseObjective(tInputFile);
  // tInputFile.close();

  // tInputFile.open(m_InputFilename.c_str()); // open a file
  // this->parseConstraints(tInputFile);
  // tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  this->parseUncertainties(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  this->parseOutput(tInputFile);
  tInputFile.close();

  // tInputFile.open(m_InputFilename.c_str()); // open a file
  // this->parseService(tInputFile);
  // tInputFile.close();

  // // If we will need to run the prune_and_refine executable for any
  // // reason we need to have our "run" mesh name not be the same
  // // as the input mesh name.
  // int tNumRefines = 0;
  // if(m_InputData.number_refines != "")
  //   tNumRefines = std::atoi(m_InputData.number_refines.c_str());
  // if(tNumRefines > 0 ||
  //     (m_InputData.initial_guess_filename != "" && m_InputData.initial_guess_field_name != ""))
  // {
  //   m_InputData.run_mesh_name_without_extension = m_InputData.mesh_name_without_extension + "_mod";
  //   m_InputData.run_mesh_name = m_InputData.run_mesh_name_without_extension;
  //   if(m_InputData.mesh_extension != "")
  //     m_InputData.run_mesh_name += m_InputData.mesh_extension;
  // }
  // else
  // {
  //   m_InputData.run_mesh_name = m_InputData.mesh_name;
  //   m_InputData.run_mesh_name_without_extension = m_InputData.mesh_name_without_extension;
  // }
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

// /******************************************************************************/
// void XMLGenerator::getUncertaintyFlags()
// /******************************************************************************/
// {
//     if(!m_InputData.uncertainties.empty())
//     {
//         m_InputData.m_HasUncertainties = true;
//     }
// }

}
