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
 * DefaultInputGenerator.hpp
 *
 *  Created on: Apr 16, 2020
 *
 */

#ifndef SRC_DEFAULTINPUTGENERATOR_HPP_
#define SRC_DEFAULTINPUTGENERATOR_HPP_

#include <string>
#include <map>
#include <vector>
#include <fstream>

#include "XMLGeneratorDataStruct.hpp"
#include "Plato_Parser.hpp"

namespace XMLGen
{

class DefaultInputGenerator
{

public:
    DefaultInputGenerator(const InputData& aInputData);
    ~DefaultInputGenerator();
    virtual bool generateInputFiles();


protected:

    void generateROLInput();
    void generateAMGXInput();
    bool generateDefinesXML(std::ostringstream *aStringStream = NULL);
    bool generatePlatoAnalyzeShapeDefinesXML();
    virtual bool generateInterfaceXML(std::ostringstream *aStringStream = NULL);
    bool generatePlatoAnalyzeShapeInterfaceXML();
    virtual bool generateLaunchScript();
    bool generatePlatoAnalyzeShapeLaunchScript();
    bool generateSummitLaunchScripts();
    virtual bool generatePlatoMainOperationsXML();
    bool generatePlatoMainOperationsXMLForShape();
    bool generatePlatoMainInputDeckXML();
    bool generatePlatoESPInputDeckXML();
    virtual bool generatePerformerOperationsXML();
    bool generateSalinasOperationsXML();
    bool generateAlbanyOperationsXML();
    virtual bool generatePlatoAnalyzeOperationsXML();
    bool generatePlatoAnalyzeOperationsXMLForShape();
    bool generatePlatoESPOperationsXMLForShape();
    bool generateLightMPOperationsXML();
    virtual bool generatePhysicsInputDecks();
    bool generateSalinasInputDecks(std::ostringstream *aStringStream = NULL);
    virtual bool generatePlatoAnalyzeInputDecks(std::ostringstream *aStringStream = NULL);
    bool generatePlatoAnalyzeInputDeckForNewUncertaintyWorkflow();
    bool generateLightMPInputDecks();
    bool generateAlbanyInputDecks();
    bool outputVolumeStage(pugi::xml_document &doc);
    virtual void outputUpdateProblemStage(pugi::xml_document &doc);
    virtual void outputOutputToFileStage(pugi::xml_document &doc,
                                 const bool &aHasUncertainties,
                                 const bool &aRequestedVonMises);
    void outputOutputToFileStageForNewUncertaintyWorkflow(pugi::xml_document &doc,
                                 const bool &aHasUncertainties,
                                 const bool &aRequestedVonMises);
    void addStochasticObjectiveValueOperation(pugi::xml_document &aDoc);
    void addStochasticObjectiveGradientOperation(pugi::xml_document &aDoc);
    void addVonMisesStatisticsOperation(pugi::xml_document &aDoc);
    void addFilterControlOperation(pugi::xml_document &aDoc);
    void addFilterGradientOperation(pugi::xml_document &aDoc);
    void addFilterHessianOperation(pugi::xml_document &aDoc);
    virtual void addPlatoMainOutputOperation(pugi::xml_document &aDoc,
                                     const bool &aHasUncertainties,
                                     const bool &aRequestedVonMises);
    void addCSMMeshOutputOperation(pugi::xml_document &aDoc);
    void addEnforceBoundsOperationToFile(pugi::xml_document &aDoc);
    void addEnforceBoundsOperationToStage(pugi::xml_node &aStageNode);
    void addSetUpperBoundsOperation(pugi::xml_document &aDoc);
    void addSetLowerBoundsOperation(pugi::xml_document &aDoc);
    bool addAggregateHessianOperation(pugi::xml_document &aDoc);
    virtual bool addAggregateEnergyOperation(pugi::xml_document &aDoc);
    virtual bool addAggregateValuesOperation(pugi::xml_document &aDoc);
    virtual bool addAggregateGradientOperation(pugi::xml_document &aDoc);
    void addUpdateGeometryOnChangeOperation(pugi::xml_document &aDoc);
    void addComputeVolumeOperation(pugi::xml_document &aDoc);
    void addDesignVolumeOperation(pugi::xml_document &aDoc);
    virtual void addUpdateProblemOperation(pugi::xml_document &aDoc);
    virtual void addFilterInfo(pugi::xml_document &aDoc);
    virtual void addInitializeFieldOperation(pugi::xml_document &aDoc);
    void addInitializeValuesOperation(pugi::xml_document &aDoc);
    void outputInitializeOptimizationStage(pugi::xml_document &doc);
    void outputInitializeOptimizationStageForSO(pugi::xml_document &doc);
    virtual void outputInitializeOptimizationStageForTO(pugi::xml_document &doc);
    bool outputVolumeGradientStage(pugi::xml_document &doc);
    bool outputSurfaceAreaStage(pugi::xml_document &doc);
    void outputSetLowerBoundsStage(pugi::xml_document &doc);
    void outputSetUpperBoundsStage(pugi::xml_document &doc);
    virtual void outputCacheStateStage(pugi::xml_document &doc, const bool &aHasUncertainties);
    bool outputConstraintStage(pugi::xml_document &doc);
    bool outputConstraintGradientStage(pugi::xml_document &doc);
    bool outputSurfaceAreaGradientStage(pugi::xml_document &doc);
    void outputDesignVolumeStage(pugi::xml_document &doc);
    bool outputComputeStateStage(pugi::xml_document &doc);
    bool outputObjectiveHessianStage(pugi::xml_document &doc);
    virtual bool outputObjectiveStage(pugi::xml_document &doc, const bool &aHasUncertainties);
    virtual bool outputObjectiveGradientStage(pugi::xml_document &doc,
                                           const bool &aHasUncertainties);
    bool outputObjectiveStageForShape(pugi::xml_document &doc);
    bool outputObjectiveHessianStageForShape(pugi::xml_document &doc);
    bool outputObjectiveGradientStageForShape(pugi::xml_document &doc);
    bool checkForNodesetSidesetNameConflicts();
    void addComputeObjectiveValueOperationForNewUncertaintyWorkflow(pugi::xml_node &aNode);
    void addComputeObjectiveGradientOperationForNewUncertaintyWorkflow(pugi::xml_node &aNode);
    bool addDefinesToDoc(pugi::xml_document& doc);

    const InputData &m_InputData;


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

private:
    void addVersionEntryToDoc(pugi::xml_document& doc);
    size_t stringToSizeT(const std::string& aString);
    size_t getGreatestDivisor(const size_t& aDividend, size_t aDivisor);
    virtual size_t computeNumberOfNodesNeeded();
    virtual void generateBatchScript();
    virtual void generateJSRunScript();
    void generatePerformerBashScripts();
    void generateEngineBashScript();
    virtual void generateAnalyzeBashScripts();
    std::string makeValuesString(const std::vector<std::string>& aValues);

};

}

#endif /* SRC_DEFAULTINPUTGENERATOR_HPP_ */
