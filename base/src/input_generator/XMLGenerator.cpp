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
#include <tuple>

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
void XMLGenerator::writeInputFiles
(XMLGen::InputData& aInputData)
{
    XMLGen::Problem::write_optimization_problem(aInputData, m_PreProcessedInputData);
}

// /******************************************************************************//**
//  * \fn checkForProblemSetupErrors
//  * \brief Check input data for errors
// **********************************************************************************/
void XMLGenerator::checkForProblemSetupErrors
(XMLGen::InputData& aInputData)
{
    this->checkForShapeOptimizationSetupErrors(aInputData);
}

// /******************************************************************************//**
//  * \fn checkForShapeOptimizationSetupErrors
//  * \brief Look for errors specific to shape optimization problems
// **********************************************************************************/
void XMLGenerator::checkForShapeOptimizationSetupErrors
(XMLGen::InputData& aInputData)
{
    if(aInputData.optimization_parameters().optimizationType() == OT_SHAPE)
    {
        std::string tNumDesignVariablesString = aInputData.optimization_parameters().num_shape_design_variables();
        int tNumDesignVariables = std::atoi(tNumDesignVariablesString.c_str());
        for(auto tShapeServiceID : aInputData.objective.shapeServiceIDs)
        {
            XMLGen::Service tCurService = aInputData.service(tShapeServiceID);
            std::string tNumProcsString = tCurService.numberProcessors();
            int tNumProcs = std::atoi(tNumProcsString.c_str());
            if(tNumProcs > 1 && tNumProcs != tNumDesignVariables)
            {
                THROWERR("If using more than one processor in the service that is calculating the shape sensitivities you must use the same number of processors as num_shape_design_variables specified in the optimization_parameters block.");
            }
        }
    }
}

/******************************************************************************/
void XMLGenerator::generate()
/******************************************************************************/
{
    this->parseInputFile();
    this->finalize(m_InputData);

    if(!this->runSROMForUncertainVariables(m_InputData))
    {
        PRINTERR("Failed to expand uncertainties in input file generation.")
    }

    this->preProcessInputMetaData(m_InputData);
    this->checkForProblemSetupErrors(m_InputData);
    this->writeInputFiles(m_InputData);
}

/******************************************************************************/
void XMLGenerator::generate
(XMLGen::InputData& aInputData)
/******************************************************************************/
{
    this->finalize(aInputData);
    if(!this->runSROMForUncertainVariables(aInputData))
    {
        PRINTERR("Failed to expand uncertainties in input file generation.")
    }
    this->preProcessInputMetaData(aInputData);
    this->checkForProblemSetupErrors(m_InputData);
    this->writeInputFiles(aInputData);
}

/******************************************************************************/
void XMLGenerator::expandEssentialBoundaryConditions
(XMLGen::InputData& aInputData)
/******************************************************************************/
{
    m_InputDataWithExpandedEBCs = aInputData;
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
        tOptimizationParameters.append("filter_in_engine", "false", false);
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
void XMLGenerator::preProcessInputMetaData
(XMLGen::InputData& aInputData)
/******************************************************************************/
{
    this->expandEssentialBoundaryConditions(aInputData);
    determineIfPlatoEngineFilteringIsNeeded();
    this->createCopiesForPerformerCreation(aInputData);
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
void XMLGenerator::loadMaterialData
(XMLGen::InputData &aNewInputData,
 const XMLGen::InputData &aOriginalInputData,
 const std::string &aScenarioID)
/******************************************************************************/
{
    for(auto &tMaterial : aOriginalInputData.materials)
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
void XMLGenerator::loadObjectiveData
(XMLGen::InputData &aNewInputData,
 const XMLGen::InputData &aOriginalInputData,
 const std::string &aScenarioID,
 const std::string &aServiceID,
 const std::string &aShapeServiceID)
/******************************************************************************/
{
    for(size_t j=0; j<aOriginalInputData.objective.scenarioIDs.size(); j++)
    {
        std::string tTempScenarioID = aOriginalInputData.objective.scenarioIDs[j];
        std::string tTempServiceID = aOriginalInputData.objective.serviceIDs[j];
        std::string tTempShapeServiceID = "";
        if(aOriginalInputData.objective.scenarioIDs.size() == aOriginalInputData.objective.shapeServiceIDs.size())
        {
            tTempShapeServiceID = aOriginalInputData.objective.shapeServiceIDs[j];
        }
        if(tTempScenarioID == aScenarioID && tTempServiceID == aServiceID &&
           tTempShapeServiceID == aShapeServiceID)
        {
            std::string tTempCriterionID = aOriginalInputData.objective.criteriaIDs[j];
            std::string tWeight = aOriginalInputData.objective.weights[j];
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
void XMLGenerator::loadConstraintData
(XMLGen::InputData &aNewInputData,
 const XMLGen::InputData &aOriginalInputData,
 const std::string &aScenarioID,
 const std::string &aServiceID)
/******************************************************************************/
{
    for(auto &tConstraint : aOriginalInputData.constraints)
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
void XMLGenerator::loadOutputData
(XMLGen::InputData &aNewInputData,
 const XMLGen::InputData &aOriginalInputData,
 const std::string &aServiceID)
/******************************************************************************/
{
    for(auto &tOutput : aOriginalInputData.mOutputMetaData)
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
(XMLGen::InputData& aInputData,
 std::set<std::tuple<std::string,std::string,std::string>> &aObjectiveScenarioServiceTuples)
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
        this->loadMaterialData(tNewInputData, aInputData, tScenarioID);

        // Add back in relevant scenario and serivce data.
        tNewInputData.append(tCurScenario);
        tNewInputData.append(tCurService);
        if(tShapeServiceID != "")
        {
            tNewInputData.append(tCurShapeService);
        }
        
        // Add back in the relevant objective data.
        this->loadObjectiveData(tNewInputData, aInputData, tScenarioID, tServiceID, tShapeServiceID);

        // Add back in the relevant constraint data.
        this->loadConstraintData(tNewInputData, aInputData, tScenarioID, tServiceID);

        // Add output block data
        this->loadOutputData(tNewInputData, aInputData, tServiceID);

        m_PreProcessedInputData.push_back(tNewInputData);

        aInputData.mPerformerServices.push_back(aInputData.service(tServiceID));
        if(tShapeServiceID != "")
        {
            aInputData.mPerformerServices.push_back(aInputData.service(tShapeServiceID));
        }
    }
}

/******************************************************************************/
void XMLGenerator::verifyAllServicesAreTheSame
(XMLGen::InputData& aInputData)
/******************************************************************************/
{
    int tCntr = 0;
    std::string tFirstServiceID = "";
    for(auto tServiceID : aInputData.objective.serviceIDs)
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
(XMLGen::InputData& aInputData,
 std::set<std::tuple<std::string,std::string,std::string>>& aObjectiveScenarioServiceTuples)
/******************************************************************************/
{
    this->verifyAllServicesAreTheSame(aInputData);

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
        this->loadMaterialData(tNewInputData, aInputData, tScenarioID);

        // Add back in relevant scenario and serivce data.
        tNewInputData.append_unique(tCurScenario);
        tNewInputData.append_unique(tCurService);
        if(tShapeServiceID != "")
        {
            tNewInputData.append_unique(tCurShapeService);
        }
        
        // Add back in the relevant objective data.
        this->loadObjectiveData(tNewInputData, aInputData, tScenarioID, tServiceID, tShapeServiceID);

        // Add back in the relevant constraint data.
        this->loadConstraintData(tNewInputData, aInputData, tScenarioID, tServiceID);

        // Add output block data
        this->loadOutputData(tNewInputData, aInputData, tServiceID);

        if(!serviceExists(aInputData.mPerformerServices, tCurService))
        {
            aInputData.mPerformerServices.push_back(tCurService);
        }
        if(tShapeServiceID != "")
        {
            if(!serviceExists(aInputData.mPerformerServices, tCurShapeService))
            {
                aInputData.mPerformerServices.push_back(tCurService);
            }
        }
    }
    m_PreProcessedInputData.push_back(tNewInputData);
}

/******************************************************************************/
void XMLGenerator::findObjectiveScenarioServiceTuples
(XMLGen::InputData& aInputData,
 std::set<std::tuple<std::string,std::string,std::string>>& aObjectiveScenarioServiceTuples)
/******************************************************************************/
{
    for(size_t i=0; i<aInputData.objective.scenarioIDs.size(); i++)
    {
        std::string tScenarioID = aInputData.objective.scenarioIDs[i];
        std::string tServiceID = aInputData.objective.serviceIDs[i];
        std::string tCriterionID = aInputData.objective.criteriaIDs[i];
        std::string tShapeServiceID = "";
        if(aInputData.objective.shapeServiceIDs.size() == aInputData.objective.scenarioIDs.size())
        {
            tShapeServiceID = aInputData.objective.shapeServiceIDs[i];
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
(XMLGen::InputData& aInputData,
 std::set<std::tuple<std::string,std::string,std::string>>& aConstraintScenarioServiceTuples)
/******************************************************************************/
{
    for(auto &tConstraint : aInputData.constraints)
    {
        std::string tServiceID = tConstraint.service();
        auto &tService = aInputData.service(tServiceID);
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
(XMLGen::InputData& aInputData,
 std::set<std::tuple<std::string,std::string,std::string>>& aObjectiveScenarioServiceTuples)
/******************************************************************************/
{
    if(aInputData.objective.multi_load_case == "true")
    {
        createObjectiveCopiesForMultiLoadCase(aInputData, aObjectiveScenarioServiceTuples);
    }
    else
    {
        this->createCopiesForMultiPerformerCase(aInputData, aObjectiveScenarioServiceTuples);
    }
}

/******************************************************************************/
void XMLGenerator::createInputDataCopiesForConstraintPerformers
(XMLGen::InputData& aInputData,
 std::set<std::tuple<std::string,std::string,std::string>>& aConstraintScenarioServiceTuples)
/******************************************************************************/
{
    this->createCopiesForMultiPerformerCase(aInputData, aConstraintScenarioServiceTuples);
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
void XMLGenerator::createCopiesForPerformerCreation
(XMLGen::InputData& aInputData)
/******************************************************************************/
{
    std::set<std::tuple<std::string,std::string,std::string>> tObjectiveScenarioServiceTuples;
    std::set<std::tuple<std::string,std::string,std::string>> tConstraintScenarioServiceTuples;
    this->findObjectiveScenarioServiceTuples(aInputData, tObjectiveScenarioServiceTuples);
    this->findConstraintScenarioServiceTuples(aInputData, tConstraintScenarioServiceTuples);
    this->removeDuplicateTuplesFromConstraintList(tObjectiveScenarioServiceTuples, tConstraintScenarioServiceTuples);
    this->createInputDataCopiesForObjectivePerformers(aInputData, tObjectiveScenarioServiceTuples);
    this->createInputDataCopiesForConstraintPerformers(aInputData, tConstraintScenarioServiceTuples);
}

/******************************************************************************/
bool XMLGenerator::runSROMForUncertainVariables
(XMLGen::InputData &aInputData)
/******************************************************************************/
{
    if(aInputData.uncertainties.size() > 0)
    {
        Plato::srom::solve(aInputData);
        this->setNumPerformers(aInputData);
    }

    return true;
}

/******************************************************************************/
void XMLGenerator::setNumPerformers
(XMLGen::InputData& aInputData)
/******************************************************************************/
{
    auto &tService = aInputData.service(aInputData.objective.serviceIDs[0]);
    aInputData.m_UncertaintyMetaData.numPerformers = std::stoi(tService.numberProcessors());

    if (aInputData.mRandomMetaData.numSamples() % aInputData.m_UncertaintyMetaData.numPerformers != 0)
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
}

void XMLGenerator::finalize
(XMLGen::InputData& aInputData)
{
  this->finalizeScenarios(aInputData);
  this->finalizeMeshMetaData(aInputData);
}

void XMLGenerator::finalizeScenarios
(XMLGen::InputData& aInputData)
{
  auto tDiscretization = aInputData.optimization_parameters().discretization();

  for(auto& tScenario : aInputData.mScenarios)
  {
    tScenario.append("discretization", tDiscretization);
  }
}

void XMLGenerator::finalizeMeshMetaData
(XMLGen::InputData& aInputData)
{
    // If we will need to run the prune_and_refine executable for any
    // reason (restart flag should tell us this since prune_and_refine
    // is used for the initial guess field transfer) our "run" mesh name 
    // can not be the same as the input mesh name.
    if(aInputData.optimization_parameters().isARestartRun())
    {
        aInputData.mesh.run_name_without_extension = aInputData.mesh.name_without_extension + "_mod";
        aInputData.mesh.run_name = aInputData.mesh.run_name_without_extension;
        if(aInputData.mesh.file_extension != "")
            aInputData.mesh.run_name += aInputData.mesh.file_extension;
    }
    else
    {
        aInputData.mesh.run_name = aInputData.mesh.name;
        aInputData.mesh.run_name_without_extension = aInputData.mesh.name_without_extension;
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
