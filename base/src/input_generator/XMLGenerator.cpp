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

#include "Plato_SromXML.hpp"
#include "Plato_SromXMLGenTools.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLG_Macros.hpp"

#include "XMLGeneratorPlatoAnalyzeProblem.hpp"
#include "XMLGeneratorProblem.hpp"
#include "XMLGeneratorValidInputKeys.hpp"

#include "XMLGeneratorParseOutput.hpp"
#include "XMLGeneratorParseScenario.hpp"
#include "XMLGeneratorParseServices.hpp"
#include "XMLGeneratorParseMaterial.hpp"
#include "XMLGeneratorParseCriteria.hpp"
#include "XMLGeneratorParseObjective.hpp"
#include "XMLGeneratorParseConstraint.hpp"
#include "XMLGeneratorParseUncertainty.hpp"
#include "XMLGeneratorParseOptimizationParameters.hpp"
#include "XMLGeneratorParseEssentialBoundaryCondition.hpp"
#include "XMLGeneratorParseNaturalBoundaryCondition.hpp"

namespace XMLGen
{


const int MAX_CHARS_PER_LINE = 10000;

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
void XMLGenerator::writeInputFiles()
{
    //XMLGen::Analyze::write_optimization_problem(m_InputData);
    XMLGen::Problem::write_optimization_problem(m_InputData, m_PreProcessedInputData);
}

/******************************************************************************/
void XMLGenerator::generate()
/******************************************************************************/
{
    parseInputFile();

    if(!runSROMForUncertainVariables())
    {
        PRINTERR("Failed to expand uncertainties in file generation.")
    }

    preProcessInputMetaData();

    writeInputFiles();
}

/******************************************************************************/
void XMLGenerator::expandEssentialBoundaryConditions()
/******************************************************************************/
{
    m_InputDataWithExpandedEBCs = m_InputData;
    std::vector<XMLGen::EssentialBoundaryCondition> tNewEBCs;
    std::map<int, std::vector<int> > tOldIDToNewIDMap;
    int tNewID = 1;
    for(auto &tCurEBC : m_InputDataWithExpandedEBCs.ebcs)
    {
        int tOldID = std::stoi(tCurEBC.id());
        std::string tDofString = tCurEBC.value("degree_of_freedom");
        std::string tValueString = tCurEBC.value("value");
        std::vector<std::string> tDofTokens;
        std::vector<std::string> tValueTokens;
        XMLGen::parseTokens((char*)(tDofString.c_str()), tDofTokens); 
        XMLGen::parseTokens((char*)(tValueString.c_str()), tValueTokens); 
        if(tDofTokens.size() != tValueTokens.size())
        {
            THROWERR(std::string("Parse EssentialBoundaryCondition:expandDofs:  Number of Dofs does not equal the number of values. "))
        } 
        for(size_t i=0; i<tDofTokens.size(); ++i)
        {
            XMLGen::EssentialBoundaryCondition tNewEBC = tCurEBC;
            tNewEBC.property("degree_of_freedom", tDofTokens[i]);
            tNewEBC.property("value", tValueTokens[i]);
            tNewEBC.id(std::to_string(tNewID));
            tNewEBCs.push_back(tNewEBC);
            tOldIDToNewIDMap[tOldID].push_back(tNewID);
            tNewID++;
        }
    }
    m_InputDataWithExpandedEBCs.ebcs = tNewEBCs;
    updateScenariosWithExpandedBoundaryConditions(tOldIDToNewIDMap);
}

/******************************************************************************/
void XMLGenerator::determineIfPlatoEngineFilteringIsNeeded()
/******************************************************************************/
{
    if(m_InputDataWithExpandedEBCs.optimization_parameters().needsMeshMap())
    {
        XMLGen::OptimizationParameters tOptimizationParameters = m_InputDataWithExpandedEBCs.optimization_parameters();
        tOptimizationParameters.filter_in_engine("false");
        m_InputDataWithExpandedEBCs.set(tOptimizationParameters);
    }
}

/******************************************************************************/
void XMLGenerator::updateScenariosWithExpandedBoundaryConditions(std::map<int, std::vector<int> > aOldIDToNewIDMap)
/******************************************************************************/
{
    std::vector<XMLGen::Scenario> tNewScenarios;
    for(auto &tScenario : m_InputDataWithExpandedEBCs.scenarios())
    {
       XMLGen::Scenario tNewScenario = tScenario;
       std::vector<std::string> tNewBCIDs;
       for(auto &tBCID : tNewScenario.bcIDs())
       {
           int tCurID = std::stoi(tBCID);
           if(aOldIDToNewIDMap.count(tCurID) > 0)
           {
               for(auto &tCurNewBCID : aOldIDToNewIDMap[tCurID])
               {
                   tNewBCIDs.push_back(std::to_string(tCurNewBCID));
               }
           }
           else
           {
               tNewBCIDs.push_back(tBCID);
           }
       }
       tNewScenario.setBCIDs(tNewBCIDs);
       tNewScenarios.push_back(tNewScenario);
   }
   m_InputDataWithExpandedEBCs.set(tNewScenarios);
}

/******************************************************************************/
void XMLGenerator::preProcessInputMetaData()
/******************************************************************************/
{
    expandEssentialBoundaryConditions();
    determineIfPlatoEngineFilteringIsNeeded();
    createCopiesForPerformerCreation();
}

/******************************************************************************/
void XMLGenerator::clearInputDataLists(XMLGen::InputData &aInputData)
/******************************************************************************/
{
    aInputData.objective.scenarioIDs.clear();
    aInputData.objective.serviceIDs.clear();
    aInputData.objective.shapeServiceIDs.clear();
    aInputData.objective.criteriaIDs.clear();
    aInputData.objective.weights.clear();
    aInputData.constraints.clear();
    std::vector<XMLGen::Scenario> tEmptyScenarioList;
    aInputData.set(tEmptyScenarioList);
    std::vector<XMLGen::Service> tEmptyServiceList;
    aInputData.set(tEmptyServiceList);
    aInputData.mOutputMetaData.clear();
    aInputData.materials.clear();
}

/******************************************************************************/
void XMLGenerator::loadMaterialData(XMLGen::InputData &aNewInputData, 
                                     const std::string &aScenarioID)
/******************************************************************************/
{
    for(auto &tMaterial : m_InputData.materials)
    {
        bool tFound = false;
        for(auto &tExistingMaterial : aNewInputData.materials)
        {
            if(tExistingMaterial.id() == tMaterial.id())
            {
                tFound = true;
                break;
            }
        }
        if(!tFound)
        {
            aNewInputData.materials.push_back(tMaterial);
        }
    }
}

/******************************************************************************/
void XMLGenerator::loadObjectiveData(XMLGen::InputData &aNewInputData, 
                                     const std::string &aScenarioID,
                                     const std::string &aServiceID,
                                     const std::string &aShapeServiceID)
/******************************************************************************/
{
    for(size_t j=0; j<m_InputData.objective.scenarioIDs.size(); j++)
    {
        std::string tTempScenarioID = m_InputData.objective.scenarioIDs[j];
        std::string tTempServiceID = m_InputData.objective.serviceIDs[j];
        std::string tTempShapeServiceID = "";
        if(m_InputData.objective.scenarioIDs.size() == m_InputData.objective.shapeServiceIDs.size())
        {
            tTempShapeServiceID = m_InputData.objective.shapeServiceIDs[j];
        }
        if(tTempScenarioID == aScenarioID && tTempServiceID == aServiceID &&
           tTempShapeServiceID == aShapeServiceID)
        {
            std::string tTempCriterionID = m_InputData.objective.criteriaIDs[j];
            std::string tWeight = m_InputData.objective.weights[j];
            aNewInputData.objective.serviceIDs.push_back(tTempServiceID);
            if(aShapeServiceID != "")
            {
                aNewInputData.objective.shapeServiceIDs.push_back(tTempShapeServiceID);
            }
            aNewInputData.objective.scenarioIDs.push_back(tTempScenarioID);
            aNewInputData.objective.criteriaIDs.push_back(tTempCriterionID);
            aNewInputData.objective.weights.push_back(tWeight);
        } 
    }
}

/******************************************************************************/
void XMLGenerator::loadConstraintData(XMLGen::InputData &aNewInputData, 
                                     const std::string &aScenarioID,
                                     const std::string &aServiceID)
/******************************************************************************/
{
    for(auto &tConstraint : m_InputData.constraints)
    {
        std::string tTempScenarioID = tConstraint.scenario();
        std::string tTempServiceID = tConstraint.service();
        std::string tTempCriterionID = tConstraint.criterion();
        if(tTempScenarioID == aScenarioID && tTempServiceID == aServiceID)
        {
            aNewInputData.constraints.push_back(tConstraint);
        } 
    }
}

/******************************************************************************/
void XMLGenerator::loadOutputData(XMLGen::InputData &aNewInputData, 
                                  const std::string &aServiceID)
/******************************************************************************/
{
    for(auto &tOutput : m_InputData.mOutputMetaData)
    {
        if(tOutput.serviceID() == aServiceID)
        {
            aNewInputData.mOutputMetaData.push_back(tOutput);
            break;
        } 
    }
}

/******************************************************************************/
void XMLGenerator::createCopiesForMultiPerformerCase
(std::set<std::tuple<std::string,std::string,std::string>> &aObjectiveScenarioServiceTuples)
/******************************************************************************/
{
    for(auto &tTuple : aObjectiveScenarioServiceTuples)
    {
        auto tScenarioID = std::get<0>(tTuple);
        auto tServiceID = std::get<1>(tTuple);
        auto tShapeServiceID = std::get<2>(tTuple);
       
        XMLGen::Scenario tCurScenario = m_InputDataWithExpandedEBCs.scenario(tScenarioID);
        XMLGen::Service tCurService = m_InputDataWithExpandedEBCs.service(tServiceID);
        XMLGen::Service tCurShapeService;
        if(tShapeServiceID != "")
        {
            tCurShapeService = m_InputDataWithExpandedEBCs.service(tShapeServiceID);
        }

        // Clear out the data in the new metadata--we will only keep the 
        // necessary parts.
        XMLGen::InputData tNewInputData = m_InputDataWithExpandedEBCs;
        clearInputDataLists(tNewInputData);

        // Add back in relevant materials
        loadMaterialData(tNewInputData, tScenarioID);

        // Add back in relevant scenario and serivce data.
        tNewInputData.append(tCurScenario);
        tNewInputData.append(tCurService);
        if(tShapeServiceID != "")
        {
            tNewInputData.append(tCurShapeService);
        }
        
        // Add back in the relevant objective data.
        loadObjectiveData(tNewInputData, tScenarioID, tServiceID, tShapeServiceID);

        // Add back in the relevant constraint data.
        loadConstraintData(tNewInputData, tScenarioID, tServiceID);

        // Add output block data
        loadOutputData(tNewInputData, tServiceID);

        m_PreProcessedInputData.push_back(tNewInputData);

        m_InputData.mPerformerServices.push_back(m_InputData.service(tServiceID));
        if(tShapeServiceID != "")
        {
            m_InputData.mPerformerServices.push_back(m_InputData.service(tShapeServiceID));
        }
    }
}

/******************************************************************************/
void XMLGenerator::verifyAllServicesAreTheSame()
/******************************************************************************/
{
    int tCntr = 0;
    std::string tFirstServiceID = "";
    for(auto tServiceID : m_InputData.objective.serviceIDs)
    {
        if(tCntr == 0)
        {
            tFirstServiceID = tServiceID;
        }
        else
        {
            if(tServiceID != tFirstServiceID)
            {
                THROWERR("Not all service ids are the same for multi load case.");
            }
        }
        ++tCntr;
    }
}

/******************************************************************************/
bool XMLGenerator::serviceExists(std::vector<XMLGen::Service> &aServiceList, XMLGen::Service &aService)
/******************************************************************************/
{
    for(auto &tService : aServiceList)
    {
        if(tService.id() == aService.id())
        {
            return true;
        }
    }
    return false;
}

/******************************************************************************/
void XMLGenerator::createObjectiveCopiesForMultiLoadCase
(std::set<std::tuple<std::string,std::string,std::string>>& aObjectiveScenarioServiceTuples)
/******************************************************************************/
{
    verifyAllServicesAreTheSame();
    XMLGen::InputData tNewInputData = m_InputDataWithExpandedEBCs;
    clearInputDataLists(tNewInputData);
    for(auto &tTuple : aObjectiveScenarioServiceTuples)
    {
        auto tScenarioID = std::get<0>(tTuple);
        auto tServiceID = std::get<1>(tTuple);
        auto tShapeServiceID = std::get<2>(tTuple);
       
        XMLGen::Scenario tCurScenario = m_InputDataWithExpandedEBCs.scenario(tScenarioID);
        XMLGen::Service tCurService = m_InputDataWithExpandedEBCs.service(tServiceID);
        XMLGen::Service tCurShapeService;
        if(tShapeServiceID != "")
        {
            tCurShapeService = m_InputDataWithExpandedEBCs.service(tShapeServiceID);
        }

        // Add back in relevant materials
        loadMaterialData(tNewInputData, tScenarioID);

        // Add back in relevant scenario and serivce data.
        tNewInputData.append_unique(tCurScenario);
        tNewInputData.append_unique(tCurService);
        if(tShapeServiceID != "")
        {
            tNewInputData.append_unique(tCurShapeService);
        }
        
        // Add back in the relevant objective data.
        loadObjectiveData(tNewInputData, tScenarioID, tServiceID, tShapeServiceID);

        // Add back in the relevant constraint data.
        loadConstraintData(tNewInputData, tScenarioID, tServiceID);

        // Add output block data
        loadOutputData(tNewInputData, tServiceID);

        if(!serviceExists(m_InputData.mPerformerServices, tCurService))
        {
            m_InputData.mPerformerServices.push_back(tCurService);
        }
        if(tShapeServiceID != "")
        {
            if(!serviceExists(m_InputData.mPerformerServices, tCurShapeService))
            {
                m_InputData.mPerformerServices.push_back(tCurService);
            }
        }
    }
    m_PreProcessedInputData.push_back(tNewInputData);
}

/******************************************************************************/
void XMLGenerator::findObjectiveScenarioServiceTuples
(std::set<std::tuple<std::string,std::string,std::string>>& aObjectiveScenarioServiceTuples)
/******************************************************************************/
{
    for(size_t i=0; i<m_InputData.objective.scenarioIDs.size(); i++)
    {
        std::string tScenarioID = m_InputData.objective.scenarioIDs[i];
        std::string tServiceID = m_InputData.objective.serviceIDs[i];
        std::string tCriterionID = m_InputData.objective.criteriaIDs[i];
        std::string tShapeServiceID = "";
        if(m_InputData.objective.shapeServiceIDs.size() == m_InputData.objective.scenarioIDs.size())
        {
            tShapeServiceID = m_InputData.objective.shapeServiceIDs[i];
        }

        std::tuple<std::string,std::string,std::string> tCurTuple = std::make_tuple(tScenarioID,tServiceID,tShapeServiceID);
        if(aObjectiveScenarioServiceTuples.find(tCurTuple) == aObjectiveScenarioServiceTuples.end())
        {
            aObjectiveScenarioServiceTuples.insert(tCurTuple);
        }
    }
}

/******************************************************************************/
void XMLGenerator::findConstraintScenarioServiceTuples
(std::set<std::tuple<std::string,std::string,std::string>>& aConstraintScenarioServiceTuples)
/******************************************************************************/
{
    for(auto &tConstraint : m_InputData.constraints)
    {
        std::string tServiceID = tConstraint.service();
        auto &tService = m_InputData.service(tServiceID);
        if(tService.code() != "platomain") // platomain will be added elsewhere
        {
            std::string tScenarioID = tConstraint.scenario();
            std::string tCriterionID = tConstraint.criterion();
            std::string tShapeServiceID = "";

            std::tuple<std::string,std::string,std::string> tCurTuple = std::make_tuple(tScenarioID,tServiceID,tShapeServiceID);
            if(aConstraintScenarioServiceTuples.find(tCurTuple) == aConstraintScenarioServiceTuples.end())
            {
                aConstraintScenarioServiceTuples.insert(tCurTuple);
            }
        }
    }
}

/******************************************************************************/
void XMLGenerator::createInputDataCopiesForObjectivePerformers
(std::set<std::tuple<std::string,std::string,std::string>>& aObjectiveScenarioServiceTuples)
/******************************************************************************/
{
    if(m_InputData.objective.multi_load_case == "true")
    {
        createObjectiveCopiesForMultiLoadCase(aObjectiveScenarioServiceTuples);
    }
    else
    {
        createCopiesForMultiPerformerCase(aObjectiveScenarioServiceTuples);
    }
}

/******************************************************************************/
void XMLGenerator::createInputDataCopiesForConstraintPerformers
(std::set<std::tuple<std::string,std::string,std::string>>& aConstraintScenarioServiceTuples)
/******************************************************************************/
{
    createCopiesForMultiPerformerCase(aConstraintScenarioServiceTuples);
}

/******************************************************************************/
void XMLGenerator::removeDuplicateTuplesFromConstraintList
(std::set<std::tuple<std::string,std::string,std::string>>& aObjectiveScenarioServiceTuples,
 std::set<std::tuple<std::string,std::string,std::string>>& aConstraintScenarioServiceTuples)
/******************************************************************************/
{
    std::vector<std::tuple<std::string,std::string,std::string>> tTuplesToRemove;
    for(auto tConstraintTuple : aConstraintScenarioServiceTuples)
    {
        bool tFound = false;
        for(auto tObjectiveTuple : aObjectiveScenarioServiceTuples)
        {
            // Just check the first two entries since the constraint 
            // might not have shape services defined.
            if((std::get<0>(tConstraintTuple) == std::get<0>(tObjectiveTuple)) &&
               (std::get<1>(tConstraintTuple) == std::get<1>(tObjectiveTuple)))
            {
                tFound = true;
            }
        }
        
        if(tFound)
        {
            tTuplesToRemove.push_back(tConstraintTuple);
        }
    }
    for(auto tTuple : tTuplesToRemove)
    {
        aConstraintScenarioServiceTuples.erase(tTuple);
    }
}

/******************************************************************************/
void XMLGenerator::createCopiesForPerformerCreation()
/******************************************************************************/
{
    std::set<std::tuple<std::string,std::string,std::string>> tObjectiveScenarioServiceTuples;
    std::set<std::tuple<std::string,std::string,std::string>> tConstraintScenarioServiceTuples;
    findObjectiveScenarioServiceTuples(tObjectiveScenarioServiceTuples);
    findConstraintScenarioServiceTuples(tConstraintScenarioServiceTuples);
    removeDuplicateTuplesFromConstraintList(tObjectiveScenarioServiceTuples, tConstraintScenarioServiceTuples);
    createInputDataCopiesForObjectivePerformers(tObjectiveScenarioServiceTuples);
    createInputDataCopiesForConstraintPerformers(tConstraintScenarioServiceTuples);
}

/******************************************************************************/
bool XMLGenerator::runSROMForUncertainVariables()
/******************************************************************************/
{
    if(m_InputData.uncertainties.size() > 0)
    {
        Plato::srom::solve(m_InputData);
        this->setNumPerformers();
    }

    return true;
}

/******************************************************************************/
void XMLGenerator::setNumPerformers()
/******************************************************************************/
{
    auto &tService = m_InputData.service(m_InputData.objective.serviceIDs[0]); 
    m_InputData.m_UncertaintyMetaData.numPerformers = std::stoi(tService.numberProcessors());

    if (m_InputData.mRandomMetaData.numSamples() % m_InputData.m_UncertaintyMetaData.numPerformers != 0)
    {
        THROWERR("Set Number for Performers: Number of samples must divide evenly into number of ranks.");
    }
}

/******************************************************************************/
void XMLGenerator::parseOutput(std::istream &aInputFile)
/******************************************************************************/
{
    XMLGen::ParseOutput tParseOutput;
    tParseOutput.parse(aInputFile);
    m_InputData.mOutputMetaData = tParseOutput.data();
}

/******************************************************************************/
void XMLGenerator::parseScenarios(std::istream &aInputFile)
/******************************************************************************/
{
    XMLGen::ParseScenario tParseScenario;
    tParseScenario.parse(aInputFile);
    auto tScenarios = tParseScenario.data();
    m_InputData.set(tScenarios);
}

/******************************************************************************/
void XMLGenerator::parseServices(std::istream &aInputFile)
/******************************************************************************/
{
    XMLGen::ParseService tParseService;
    tParseService.parse(aInputFile);
    auto tServices = tParseService.data();
    m_InputData.set(tServices);
}

/******************************************************************************/
void XMLGenerator::parseObjective(std::istream &aInputFile)
/******************************************************************************/
{
    XMLGen::ParseObjective tParseObjective;
    tParseObjective.parse(aInputFile);
    m_InputData.objective = tParseObjective.data();
}

/******************************************************************************/
bool XMLGenerator::parseLoads(std::istream &fin)
/******************************************************************************/
{
    XMLGen::ParseNaturalBoundaryCondition tParseNaturalBoundaryCondition;
    tParseNaturalBoundaryCondition.parse(fin);
    m_InputData.loads = tParseNaturalBoundaryCondition.data();
    return true;
}

/******************************************************************************/
void XMLGenerator::getTokensFromLine(std::istream &fin, std::vector<std::string>& tokens)
/******************************************************************************/
{
    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    fin.getline(buf, MAX_CHARS_PER_LINE);
    XMLGen::parseTokens(buf, tokens);
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
bool XMLGenerator::parseOptimizationParameters(std::istream &fin)
/******************************************************************************/
{
    XMLGen::ParseOptimizationParameters tParseOptimizationParameters;
    tParseOptimizationParameters.parse(fin);
    if(tParseOptimizationParameters.data().size() > 0)
    {
        m_InputData.set(tParseOptimizationParameters.data()[0]);
    }
    else
    {
      THROWERR("Failed to parse an optimization_parameters block.")
    }
#if 0
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
    XMLGen::parseTokens(buf, tokens);

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
          XMLGen::parseTokens(buf, tokens);
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
                XMLGen::parseTokens(buf, tokens);
                // process the tokens
                if(tokens.size() > 0)
                {
                  for(size_t j=0; j<tokens.size(); ++j)
                    tokens[j] = toLower(tokens[j]);

                  if(parseSingleValue(tokens, tInputStringList = {"end","material_box"}, tStringValue))
                  {
                    if(m_InputData.optimizer.levelset_material_box_min == "" ||
                        m_InputData.optimizer.levelset_material_box_max == "")
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
                      m_InputData.optimizer.levelset_material_box_min += tokens[j];
                      if(j<4)
                        m_InputData.optimizer.levelset_material_box_min += " ";
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
                      m_InputData.optimizer.levelset_material_box_max += tokens[j];
                      if(j<4)
                        m_InputData.optimizer.levelset_material_box_max += " ";
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
              m_InputData.optimizer.objective_number_standard_deviations = tokens[4];
            }
            // else if(parseSingleValue(tokens, tInputStringList = {"output", "standard","deviations"}, tStringValue))
            // {
            //   if(tokens.size() < 4)
            //   {
            //     std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No outputs specified after \"output standard deviations\" keywords.\n";
            //     return false;
            //   }
            //   for(size_t j=3; j<tokens.size(); ++j)
            //   {
            //     m_InputData.mStandardDeviations.push_back(tokens[j]);
            //   }
            // }
            else if(parseSingleValue(tokens, tInputStringList = {"initial","density","value"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial density value\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.initial_density_value = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"optimization","type"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"optimization type\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.optimization_type = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"csm","file"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"csm file\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.csm_filename = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"normalize_in_aggregator"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"normalize_in_aggregator\" keyword.\n";
                return false;
              }
              m_InputData.optimizer.mNormalizeInAggregator = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"create","levelset","spheres"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"create levelset spheres\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.create_levelset_spheres = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"write","restart","file"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"write restart file\" keyword(s).\n";
                return false;
              }
              if(tStringValue == "true")
                m_InputData.optimizer.write_restart_file = "True";
            }
            else if(parseSingleValue(tokens, tInputStringList = {"levelset","initialization","method"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset initialization method\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.levelset_initialization_method = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"max","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"max iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.max_iterations = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"restart","iteration"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"restart iteration\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.restart_iteration = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"initial","guess","filename"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial guess filename\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.initial_guess_filename = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"initial","guess","field","name"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial guess field name\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.initial_guess_field_name = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"prune","mesh"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"prune mesh\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.prune_mesh = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","buffer","layers"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number buffer layers\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.number_buffer_layers = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","prune","and","refine","processors"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number prune and refine processors\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.number_prune_and_refine_processors = tStringValue;
            }
            else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","refines"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number refines\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.number_refines = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"verbose"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"verbose\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mVerbose = XMLGen::to_lower(tStringValue);
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","move","limit"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma move limit\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMMAMoveLimit = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","control","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma control stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMMAControlStagnationTolerance = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","objective","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma objective stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMMAObjectiveStagnationTolerance = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","asymptote","expansion"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma asymptote expansion\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMMAAsymptoteExpansion = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","asymptote","contraction"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma asymptote contraction\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMMAAsymptoteContraction = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","max","sub","problem","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma max sub problem iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMMAMaxNumSubProblemIterations = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"mma","max","trust","region","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"mma max trust region iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMMAMaxTrustRegionIterations = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","max","trust","region","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks max trust region iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMaxTrustRegionIterations = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","expansion","factor"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region expansion factor\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mTrustRegionExpansionFactor = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","contraction","factor"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region contraction factor\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mTrustRegionContractionFactor = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","gradient","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer gradient tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mOuterGradientToleranceKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","stationarity","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer stationarity tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mOuterStationarityToleranceKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mOuterStagnationToleranceKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","control","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer control stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mOuterControlStagnationToleranceKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","disable","post","smoothing"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks disable post smoothing\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mDisablePostSmoothingKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","actual","reduction","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer actual reduction tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mOuterActualReductionToleranceKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","ratio","low"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region ratio low\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mTrustRegionRatioLowKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","ratio","mid"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region ratio mid\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mTrustRegionRatioMidKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","ratio","high"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region ratio high\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mTrustRegionRatioUpperKS = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","initial","radius","scale"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks initial radius scale\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mInitialRadiusScale = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks","max","radius","scale"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks max radius scale\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMaxRadiusScale = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"problem","update","frequency"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"problem update frequency\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mProblemUpdateFrequency = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","max","inner","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma max inner iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMaxInnerIterationsGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","inner","kkt","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma inner kkt tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mInnerKKTtoleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","inner","control","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma inner control stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mInnerControlStagnationToleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","kkt","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer kkt tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mOuterKKTtoleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","control","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma control stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mOuterControlStagnationToleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","objective","stagnation","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer objective stagnation tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mOuterObjectiveStagnationToleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","stationarity","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer stationarity tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mOuterStationarityToleranceGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"gcmma","initial","moving","asymptotes","scale","factor"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma initial moving asymptotes scale factor\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mInitialMovingAsymptotesScaleFactorGCMMA = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"levelset","sphere","packing", "factor"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset sphere packing factor\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.levelset_sphere_packing_factor = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"levelset","sphere","radius"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset sphere radius\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.levelset_sphere_radius = tStringValue;
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
                m_InputData.optimizer.levelset_nodesets.push_back(tokens[j]);
              }
            }
            // else if(parseSingleValue(tokens, tInputStringList = {"output","frequency"}, tStringValue))
            // {
            //   if(tStringValue == "")
            //   {
            //     std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"output frequency\" keyword(s).\n";
            //     return false;
            //   }
            //   m_InputData.output_frequency = tStringValue;
            // }
            // else if(parseSingleValue(tokens, tInputStringList = {"output","method"}, tStringValue))
            // {
            //   if(tokens.size() < 3)
            //   {
            //     std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"output method\" keyword(s).\n";
            //     return false;
            //   }
            //   m_InputData.output_method = tokens[2];
            //   for(size_t j=3; j<tokens.size(); ++j)
            //   {
            //     m_InputData.output_method += " ";
            //     m_InputData.output_method += tokens[j];
            //   }
            // }
            else if(parseSingleValue(tokens, tInputStringList = {"fixed","blocks"}, tStringValue))
            {
              if(tokens.size() < 3)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"fixed blocks\" keyword(s).\n";
                return false;
              }
              for(size_t j=2; j<tokens.size(); ++j)
              {
                m_InputData.optimizer.fixed_block_ids.push_back(tokens[j]);
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
                m_InputData.optimizer.fixed_sideset_ids.push_back(tokens[j]);
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
                m_InputData.optimizer.fixed_nodeset_ids.push_back(tokens[j]);
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","type"}, tStringValue))
            {
              // retrieve input
              m_InputData.optimizer.filter_type = "";
              for(size_t j=2; j<tokens.size(); ++j)
              {
                if(j!=2)
                {
                  m_InputData.optimizer.filter_type += " ";
                }
                m_InputData.optimizer.filter_type += tokens[j];
              }

              // check input is valid
              if(m_InputData.optimizer.filter_type != m_InputData.optimizer.m_filterType_identity_generatorName &&
                  m_InputData.optimizer.filter_type != m_InputData.optimizer.m_filterType_kernel_generatorName &&
                  m_InputData.optimizer.filter_type != m_InputData.optimizer.m_filterType_kernelThenHeaviside_generatorName &&
                  m_InputData.optimizer.filter_type != m_InputData.optimizer.m_filterType_kernelThenTANH_generatorName)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter type\" did not match allowed types which include:\n\t"
                  <<"\""<<m_InputData.optimizer.m_filterType_identity_generatorName<<"\","
                  <<"\""<<m_InputData.optimizer.m_filterType_kernel_generatorName<<"\","
                  <<"\""<<m_InputData.optimizer.m_filterType_kernelThenHeaviside_generatorName<<"\""
                  <<"\""<<m_InputData.optimizer.m_filterType_kernelThenTANH_generatorName<<"\""
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
              if(m_InputData.optimizer.filter_heaviside_min!="")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside min\" both specified.\n"
                  <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
                return false;
              }
              m_InputData.optimizer.filter_heaviside_min = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","heaviside","update"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter heaviside update\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.filter_heaviside_update = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","heaviside","max"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter heaviside max\" keyword(s).\n";
                return false;
              }
              if(m_InputData.optimizer.filter_heaviside_max!="")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside max\" both specified."
                  <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
                return false;
              }
              m_InputData.optimizer.filter_heaviside_max = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","heaviside","scale"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter heaviside scale\" keyword(s).\n";
                return false;
              }
              if(m_InputData.optimizer.filter_heaviside_min!="")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside min\" both specified."
                  <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
                return false;
              }
              if(m_InputData.optimizer.filter_heaviside_max!="")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: \"filter heaviside scale\" and \"filter heaviside max\" both specified."
                  <<"\tOnly specify one of them. \"max/min/update\" are for updating continuation problems. \"scale\" is used otherwise.\n";
                return false;
              }
              m_InputData.optimizer.filter_heaviside_min = tStringValue;
              m_InputData.optimizer.filter_heaviside_max = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","radial","power"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter power\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.filter_power = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","radius","scale"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter radius scale\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.filter_radius_scale = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","radius","absolute"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter radius absolute\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.filter_radius_absolute = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","projection","start","iteration"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter projection start iteration\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.filter_projection_start_iteration = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","projection","update","interval"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter projection update interval\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.filter_projection_update_interval = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"filter","use","additive","continuation"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter use additive continuation\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.filter_use_additive_continuation = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"algorithm"}, tStringValue))
            {
              if(tokens.size() < 2)
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"algorithm\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.optimization_algorithm = tokens[1];
              for(size_t j=2; j<tokens.size(); ++j)
              {
                m_InputData.optimizer.optimization_algorithm += " ";
                m_InputData.optimizer.optimization_algorithm += tokens[j];
              }
            }
            else if(parseSingleValue(tokens, tInputStringList = {"discretization"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"discretization\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.discretization = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"check","gradient"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"check gradient\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.check_gradient = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"check","hessian"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"check hessian\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.check_hessian = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"hessian","type"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"hessian type\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mHessianType = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"limited","memory","storage"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"limited memory storage\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mLimitedMemoryStorage = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"use","mean","norm"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"use mean norm\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mUseMeanNorm = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"al","penalty","parameter"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"al penalty parameter\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mAugLagPenaltyParam = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"feasibility","tolerance"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"feasibility tolerance\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mFeasibilityTolerance = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"al","penalty","scale","factor"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"al penalty scale factor\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mAugLagPenaltyParamScale = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"al","max","subproblem","iterations"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"al max subproblem iterations\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMaxNumAugLagSubProbIter = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"max","trust","region","radius"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"max trust region radius\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMaxTrustRegionRadius = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"ks", "min","trust","region","radius"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks min trust region radius\" keyword(s).\n";
                return false;
              }
              m_InputData.optimizer.mMinTrustRegionRadius = tStringValue;
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
  if(m_InputData.optimizer.restart_iteration != "" &&
      m_InputData.optimizer.restart_iteration != "0")
  {
    if(m_InputData.optimizer.initial_guess_filename == "" &&
        m_InputData.optimizer.initial_guess_field_name == "")
    {
      // This block indicates that we are coming from the gui so only the
      // restart iteration was specified.  We will fill in the other values
      // based on what we know the gui will be providing for the run.
      m_InputData.optimizer.initial_guess_filename = "restart_" + m_InputData.optimizer.restart_iteration + ".exo";
      m_InputData.optimizer.restart_iteration = "1";
      m_InputData.optimizer.initial_guess_field_name = "optimizationdofs";
    }
    else if(m_InputData.optimizer.initial_guess_filename == "" &&
            m_InputData.optimizer.initial_guess_field_name != "")
    {
      m_InputData.optimizer.initial_guess_filename = "restart_" + m_InputData.optimizer.restart_iteration + ".exo";
      m_InputData.optimizer.restart_iteration = "1";
    }
    else
    {
      // This block indicates that the user is manually setting up the
      // restart file and so we depend on him having specified a filename
      // and field name.  If either of these is empty we need to error out.
      if(m_InputData.optimizer.initial_guess_field_name == "" ||
          m_InputData.optimizer.initial_guess_filename == "")
      {
        std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: You must specify a valid initial guess mesh filename and a valid field name on that mesh from which initial values will be obtained.\n";
        return false;
      }
    }
  }

  // // If "use normalization in aggregator" was not specified choose a default value based on
  // // the optimization algorithm.
  // if(m_InputData.mUseNormalizationInAggregator.length() == 0)
  // {
  //   if(m_InputData.optimization_algorithm == "ksal" ||
  //       m_InputData.optimization_algorithm == "rol ksal")
  //   {
  //     m_InputData.mUseNormalizationInAggregator = "true";
  //   }
  //   else
  //   {
  //     m_InputData.mUseNormalizationInAggregator = "false";
  //   }
  // }
#endif

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
    XMLGen::parseTokens(buf, tokens);

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
          XMLGen::parseTokens(buf, tokens);
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
    XMLGen::parseTokens(buf, tokens);

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
          XMLGen::parseTokens(buf, tokens);
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
    XMLGen::parseTokens(buf, tokens);

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
          XMLGen::parseTokens(buf, tokens);
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
            else if(parseSingleValue(tokens, tInputStringList = {"element_type"}, tStringValue))
            {
              if(tStringValue == "")
              {
                std::cout << "ERROR:XMLGenerator:parseBlocks: No value specified after \"element type\" keywords.\n";
                return false;
              }
              new_block.element_type = tStringValue;
            }
            else if(parseSingleValue(tokens, tInputStringList = {"name"}, tStringValue))
            {
                new_block.name = tStringValue;
            }
            else
            {
              PrintUnrecognizedTokens(tokens);
              std::cout << "ERROR:XMLGenerator:parseBlocks: Unrecognized keyword.\n";
              return false;
            }
          }
        }
        if(new_block.name.empty())
            new_block.name = "block_" + new_block.block_id;
        m_InputData.blocks.push_back(new_block);
      }
    }
  }
  return true;
}

/******************************************************************************/
void XMLGenerator::parseBoundaryConditions(std::istream &aInput)
/******************************************************************************/
{
    XMLGen::ParseEssentialBoundaryCondition tParseEssentialBoundaryCondition;
    tParseEssentialBoundaryCondition.parse(aInput);
//    tParseEssentialBoundaryCondition.expandDofs();
    m_InputData.ebcs = tParseEssentialBoundaryCondition.data();
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
    m_InputData.set(tParseCriteria.data());
}

/******************************************************************************/
bool XMLGenerator::parseConstraints(std::istream &aInput)
/******************************************************************************/
{
    XMLGen::ParseConstraint tParseConstraint;
    tParseConstraint.parse(aInput);
    m_InputData.constraints = tParseConstraint.data();
    return true;
}

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

  parseBoundaryConditions(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseLoads(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  parseOptimizationParameters(tInputFile);
  tInputFile.close();
  
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

  tInputFile.open(m_InputFilename.c_str()); // open a file
  this->parseObjective(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  this->parseConstraints(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  this->parseUncertainties(tInputFile);
  tInputFile.close();

  tInputFile.open(m_InputFilename.c_str()); // open a file
  this->parseOutput(tInputFile);
  tInputFile.close();

   tInputFile.open(m_InputFilename.c_str()); // open a file
   this->parseServices(tInputFile);
   tInputFile.close();
  
  tInputFile.open(m_InputFilename.c_str()); // open a file
  this->parseScenarios(tInputFile);
  tInputFile.close();

    // If we will need to run the prune_and_refine executable for any
    // reason we need to have our "run" mesh name not be the same
    // as the input mesh name.
    int tNumRefines = 0;
    if(m_InputData.optimization_parameters().number_refines() != "")
        tNumRefines = std::atoi(m_InputData.optimization_parameters().number_refines().c_str());
    if(tNumRefines > 0 ||
        (m_InputData.optimization_parameters().initial_guess_file_name() != "" && m_InputData.optimization_parameters().initial_guess_field_name() != ""))
    {
        m_InputData.mesh.run_name_without_extension = m_InputData.mesh.name_without_extension + "_mod";
        m_InputData.mesh.run_name = m_InputData.mesh.run_name_without_extension;
        if(m_InputData.mesh.file_extension != "")
            m_InputData.mesh.run_name += m_InputData.mesh.file_extension;
    }
    else
    {
        m_InputData.mesh.run_name = m_InputData.mesh.name;
        m_InputData.mesh.run_name_without_extension = m_InputData.mesh.name_without_extension;
    }
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

}
