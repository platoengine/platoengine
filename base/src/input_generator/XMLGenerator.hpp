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
 * XMLGenerator.hpp
 *
 *  Created on: Sep 28, 2017
 *
 */

#ifndef SRC_XMLGENERATOR_HPP_
#define SRC_XMLGENERATOR_HPP_

#include <string>
#include <map>
#include <vector>
#include <fstream>

#include "Plato_Parser.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{

class XMLGenerator
{

public:
    XMLGenerator(const std::string &input_filename = "", bool use_launch = false, const XMLGen::Arch& arch = XMLGen::Arch::CEE);
    ~XMLGenerator();
    void generate();
    const InputData& getInputData(){ return m_InputData; }

protected:

    bool parseLoads(std::istream &fin);
    void getTokensFromLine(std::istream &fin, std::vector<std::string>& tokens);
    void parseBoundaryConditions(std::istream &fin);
    bool runSROMForUncertainVariables();
    void setNumPerformers();
    void parseInputFile();
    bool parseMesh(std::istream &fin);
    void parseOutput(std::istream &fin);
    bool parseCodePaths(std::istream &fin);
    void parseMaterials(std::istream &fin);
    void parseCriteria(std::istream &fin);
    bool parseBlocks(std::istream &fin);
    void parseObjective(std::istream &fin);
    void parseScenarios(std::istream &fin);
    void parseServices(std::istream &fin);
    bool parseConstraints(std::istream &fin);
    bool parseOptimizationParameters(std::istream &fin);
    void parseUncertainties(std::istream &fin);
    std::string toLower(const std::string &s);
    std::string toUpper(const std::string &s);
    bool parseSingleValue(const std::vector<std::string> &aTokens,
                          const std::vector<std::string> &aInputStrings,
                          std::string &aReturnStringValue);

    bool parseSingleUnLoweredValue(const std::vector<std::string> &aTokens,
                                   const std::vector<std::string> &aUnLoweredTokens,
                                   const std::vector<std::string> &aInputStrings,
                                   std::string &aReturnStringValue);

    std::string m_InputFilename;
    XMLGen::InputData m_InputData;
    XMLGen::InputData m_InputDataWithExpandedEBCs;
    std::vector<XMLGen::InputData> m_PreProcessedInputData;
private:

    void preProcessInputMetaData();
    void determineIfPlatoEngineFilteringIsNeeded();
    void expandEssentialBoundaryConditions();
    void updateScenariosWithExpandedBoundaryConditions(std::map<int, std::vector<int> > aOldIDToNewIDMap);
    void loadOutputData(XMLGen::InputData &aNewInputData, 
                                  const std::string &aServiceID);
    void loadMaterialData(XMLGen::InputData &aNewInputData,
                                     const std::string &aScenarioID);
    void loadObjectiveData(XMLGen::InputData &aNewInputData,
                                     const std::string &aScenarioID,
                                     const std::string &aServiceID,
                                     const std::string &aShapeServiceID);
    void loadConstraintData(XMLGen::InputData &aNewInputData,
                                     const std::string &aScenarioID,
                                     const std::string &aServiceID);
    void clearInputDataLists(XMLGen::InputData &aInputData);
    void createCopiesForMultiLoadCase();
    void createCopiesForMultiPerformerCase(std::set<std::tuple<std::string,std::string,std::string>> &aScenarioServiceTuples);
    void createCopiesForPerformerCreation();
    void findObjectiveScenarioServiceTuples(std::set<std::tuple<std::string,std::string,std::string>>& aObjectiveScenarioServiceTuples);
    void findConstraintScenarioServiceTuples(std::set<std::tuple<std::string,std::string,std::string>>& aConstraintScenarioServiceTuples);
    void removeDuplicateTuplesFromConstraintList(
         std::set<std::tuple<std::string,std::string,std::string>>& aObjectiveScenarioServiceTuples, 
         std::set<std::tuple<std::string,std::string,std::string>>& aConstraintScenarioServiceTuples);
    void createInputDataCopiesForObjectivePerformers(
         std::set<std::tuple<std::string,std::string,std::string>>& aObjectiveScenarioServiceTuples);
    void createInputDataCopiesForConstraintPerformers(
         std::set<std::tuple<std::string,std::string,std::string>>& aConstraintScenarioServiceTuples);
    void createObjectiveCopiesForMultiLoadCase(
         std::set<std::tuple<std::string,std::string,std::string>>& aObjectiveScenarioServiceTuples);
    void verifyAllServicesAreTheSame();
    bool serviceExists(std::vector<XMLGen::Service> &aServiceList, XMLGen::Service &aService);








    void writeInputFiles();
};

}

#endif /* SRC_XMLGENERATOR_HPP_ */
