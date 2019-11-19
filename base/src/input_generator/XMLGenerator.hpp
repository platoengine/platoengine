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
#include "XMLGeneratorDataStruct.hpp"

class XMLGenerator
{

public:
    XMLGenerator(const std::string &input_filename = "", bool use_launch = false);
    ~XMLGenerator();
    bool generate();

protected:

    pugi::xml_node createSingleUserNodalSharedData(pugi::xml_document &aDoc,
                                                   const std::string &aName,
                                                   const std::string &aType,
                                                   const std::string &aOwner,
                                                   const std::string &aUser);
    pugi::xml_node createSingleUserElementSharedData(pugi::xml_document &aDoc,
                                                     const std::string &aName,
                                                     const std::string &aType,
                                                     const std::string &aOwner,
                                                     const std::string &aUser);
    pugi::xml_node createSingleUserGlobalSharedData(pugi::xml_document &aDoc,
                                                    const std::string &aName,
                                                    const std::string &aType,
                                                    const std::string &aSize,
                                                    const std::string &aOwner,
                                                    const std::string &aUser);
    pugi::xml_node createMultiUserGlobalSharedData(pugi::xml_document &aDoc,
                                                   const std::string &aName,
                                                   const std::string &aType,
                                                   const std::string &aSize,
                                                   const std::string &aOwner,
                                                   const std::vector<std::string> &aUsers);
    bool parseCSMFileFromStream(std::istream &aStream);
    bool parseCSMFile();
    void generateROLInput();
    void generateAMGXInput();
    bool parseLoads(std::istream &fin);
    bool parseBCs(std::istream &fin);
    bool generateInterfaceXML();
    bool generateLaunchScript();
    bool generatePlatoOperationsXML();
    bool generatePlatoMainInputDeckXML();
    bool generatePerformerOperationsXML();
    bool generateSalinasOperationsXML();
    bool generateAlbanyOperationsXML();
    bool generatePlatoAnalyzeOperationsXML();
    bool generateLightMPOperationsXML();
    bool generatePhysicsInputDecks();
    bool generateSalinasInputDecks();
    bool generatePlatoAnalyzeInputDecks(std::ostringstream *aStringStream = NULL);
    bool generateLightMPInputDecks();
    bool generateAlbanyInputDecks();
    void getUncertaintyFlags(bool &aHasUncertainties,
                             bool &aRequestedVonMisesOutput);
    bool runSROMForUncertainVariables();
    bool expandUncertaintiesForGenerate();
    bool distributeObjectivesForGenerate();
    bool parseFile();
    bool parseMesh(std::istream &fin);
    bool parseCodePaths(std::istream &fin);
    bool parseMaterials(std::istream &fin);
    bool parseBlocks(std::istream &fin);
    bool parseObjectives(std::istream &fin);
    bool fillObjectiveAndPerfomerNames();
    bool parseConstraints(std::istream &fin);
    bool parseOptimizationParameters(std::istream &fin);
    bool parseUncertainties(std::istream &fin);
    bool parseTokens(char *buffer, std::vector<std::string> &tokens);
    void addVolumeConstraintForPlatoAnalyze(pugi::xml_node aNode);
    bool addNTVParameter(pugi::xml_node parent_node,
                         const std::string &name,
                         const std::string &type,
                         const std::string &value);
    bool addChild(pugi::xml_node parent_node, const std::string &name, const std::string &value);
    bool outputVolumeStage(pugi::xml_document &doc);
    void outputUpdateProblemStage(pugi::xml_document &doc);
    void outputOutputToFileStage(pugi::xml_document &doc,
                                 bool &aHasUncertainties,
                                 bool &aRequestedVonMises);
    void addStochasticObjectiveValueOperation(pugi::xml_document &aDoc);
    void addStochasticObjectiveGradientOperation(pugi::xml_document &aDoc);
    void addVonMisesStatisticsOperation(pugi::xml_document &aDoc);
    void addFilterControlOperation(pugi::xml_document &aDoc);
    void addFilterGradientOperation(pugi::xml_document &aDoc);
    void addFilterHessianOperation(pugi::xml_document &aDoc);
    void addPlatoMainOutputOperation(pugi::xml_document &aDoc,
                                     bool &aHasUncertainties,
                                     bool &aRequestedVonMises);
    void buildMinimizeThermoelasticEnergyParamsForPlatoAnalyze(const XMLGen::Objective& cur_obj, pugi::xml_node aNode);
    void buildMaximizeStiffnessParamsForPlatoAnalyze(const XMLGen::Objective& cur_obj, pugi::xml_node aNode);
    void buildMaximizeHeatConductionParamsForPlatoAnalyze(const XMLGen::Objective& cur_obj, pugi::xml_node aNode);
    void buildThermalNBCsForPlatoAnalyze(const XMLGen::Objective& aObjective, pugi::xml_node aNode, const std::string &aTitle, int &aBCCounter);
    void buildMaximizeHeatConductionEBCsForPlatoAnalyze(const XMLGen::Objective& aObjective, pugi::xml_node aNode, int &aBCCounter);
    void buildMechanicsNBCsForPlatoAnalyze(const XMLGen::Objective& cur_obj, pugi::xml_node aNode, const std::string &aTitle, int &aBCCounter);
    void buildMaximizeStiffnessEBCsForPlatoAnalyze(const XMLGen::Objective& cur_obj, pugi::xml_node aNode, int &aBCCounter);
    void addCSMMeshOutputOperation(pugi::xml_document &aDoc);
    void addEnforceBoundsOperationToFile(pugi::xml_document &aDoc);
    void addEnforceBoundsOperationToStage(pugi::xml_node &aStageNode);
    void addSetUpperBoundsOperation(pugi::xml_document &aDoc);
    void addSetLowerBoundsOperation(pugi::xml_document &aDoc);
    void addAggregateHessianOperation(pugi::xml_document &aDoc);
    void addAggregateEnergyOperation(pugi::xml_document &aDoc);
    void addAggregateValuesOperation(pugi::xml_document &aDoc);
    void addUpdateGeometryOnChangeOperation(pugi::xml_document &aDoc);
    void addAggregateGradientOperation(pugi::xml_document &aDoc);
    void addComputeVolumeOperation(pugi::xml_document &aDoc);
    void addDesignVolumeOperation(pugi::xml_document &aDoc);
    void addUpdateProblemOperation(pugi::xml_document &aDoc);
    void addFilterInfo(pugi::xml_document &aDoc);
    void addInitializeFieldOperation(pugi::xml_document &aDoc);
    void addInitializeValuesOperation(pugi::xml_document &aDoc);
    void outputInitializeOptimizationStage(pugi::xml_document &doc);
    void outputInitializeOptimizationStageForSO(pugi::xml_document &doc);
    void outputInitializeOptimizationStageForTO(pugi::xml_document &doc);
    bool outputVolumeGradientStage(pugi::xml_document &doc);
    bool outputSurfaceAreaStage(pugi::xml_document &doc);
    void outputSetLowerBoundsStage(pugi::xml_document &doc);
    void outputSetUpperBoundsStage(pugi::xml_document &doc);
    void outputCacheStateStage(pugi::xml_document &doc, bool &aHasUncertainties);
    bool outputConstraintStage(pugi::xml_document &doc);
    bool outputConstraintGradientStage(pugi::xml_document &doc);
    bool outputSurfaceAreaGradientStage(pugi::xml_document &doc);
    void outputDesignVolumeStage(pugi::xml_document &doc);
    bool outputComputeStateStage(pugi::xml_document &doc);
    bool outputInternalEnergyStage(pugi::xml_document &doc, bool &aHasUncertainties);
    bool outputInternalEnergyGradientStage(pugi::xml_document &doc,
                                           bool &aHasUncertainties);
    bool outputObjectiveStage(pugi::xml_document &doc);
    bool outputObjectiveHessianStage(pugi::xml_document &doc);
    bool outputObjectiveGradientStage(pugi::xml_document &doc);
    bool outputInternalEnergyHessianStage(pugi::xml_document &doc);
    std::string toLower(const std::string &s);
    std::string toUpper(const std::string &s);
    bool find_tokens(std::vector<std::string> &tokens, const int &start_index, const char *str1, const char *str2);
    bool parseSingleValue(const std::vector<std::string> &aTokens,
                          const std::vector<std::string> &aInputStrings,
                          std::string &aReturnStringValue);

    bool parseSingleUnLoweredValue(const std::vector<std::string> &aTokens,
                                   const std::vector<std::string> &aUnLoweredTokens,
                                   const std::vector<std::string> &aInputStrings,
                                   std::string &aReturnStringValue);
    void lookForPlatoAnalyzePerformers();

    std::string m_InputFilename;
    bool m_UseLaunch;
    XMLGen::InputData m_InputData;
    std::string m_filterType_identity_generatorName;
    std::string m_filterType_identity_XMLName;
    std::string m_filterType_kernel_generatorName;
    std::string m_filterType_kernel_XMLName;
    std::string m_filterType_kernelThenHeaviside_generatorName;
    std::string m_filterType_kernelThenHeaviside_XMLName;
    std::string m_filterType_kernelThenTANH_generatorName;
    std::string m_filterType_kernelThenTANH_XMLName;
private:
    /******************************************************************************//**
     * @brief Initialize Plato problem options
     **********************************************************************************/
    void initializePlatoProblemOptions();

    /******************************************************************************//**
     * @brief Set algorithm used to solve optimization problem
     * @param [in] aXMLnode data structure with information parsed from XML input file.
     **********************************************************************************/
    bool setOptimizerMethod(pugi::xml_node & aXMLnode);

    /******************************************************************************//**
     * @brief Set parameters associated with the Optimality Criteria algorithm
     * @param [in] aXMLnode data structure with information parsed from XML input file.
     **********************************************************************************/
    bool setOptimalityCriteriaOptions(pugi::xml_node & aXMLnode);

    /******************************************************************************//**
     * @brief Set parameters associated with the augmented Lagrangian algorithm
     * @param [in] aXMLnode data structure with information parsed from XML input file.
     **********************************************************************************/
    bool setAugmentedLagrangianOptions(const pugi::xml_node & aXMLnode);

    /******************************************************************************//**
     * @brief Set parameters associated with the trust region algorithm
     * @param [in] aXMLnode data structure with information parsed from XML input file.
     **********************************************************************************/
    bool setTrustRegionAlgorithmOptions(const pugi::xml_node & aXMLnode);

    /******************************************************************************//**
     * @brief Set parameters associated with the trust region Kelley-Sachs algorithm
     * @param [in] aXMLnode data structure with information parsed from XML input file.
     **********************************************************************************/
    bool setKelleySachsAlgorithmOptions(const pugi::xml_node & aXMLnode);

    /******************************************************************************//**
     * @brief Set parameters associated with the Globally Convergent Method of Moving Asymptotes algorithm
     * @param [in] aXMLnode data structure with information parsed from XML input file.
     **********************************************************************************/
    bool setGCMMAoptions(const pugi::xml_node & aXMLnode);

    /******************************************************************************//**
     * @brief Set parameters associated with the Globally Convergent Method of Moving Asymptotes algorithm
     * @param [in] aXMLnode data structure with information parsed from XML input file.
     **********************************************************************************/
    bool setMMAoptions(const pugi::xml_node & aXMLnode);
};

#endif /* SRC_XMLGENERATOR_HPP_ */
