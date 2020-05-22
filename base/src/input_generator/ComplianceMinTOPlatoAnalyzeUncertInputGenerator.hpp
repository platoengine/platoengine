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
 * ComplianceMinTOPlatoAnalyzeUncertInputGenerator.hpp
 *
 *  Created on: Apr 27, 2020
 *
 */

#ifndef SRC_COMPMIN_TO_PA_UNCERT_INPUTGENERATOR_HPP_
#define SRC_COMPMIN_TO_PA_UNCERT_INPUTGENERATOR_HPP_

#include <string>
#include <map>
#include <vector>
#include <fstream>

#include "XMLGeneratorDataStruct.hpp"
#include "DefaultInputGenerator.hpp"
#include "Plato_Parser.hpp"

namespace XMLGen
{

class ComplianceMinTOPlatoAnalyzeUncertInputGenerator : public DefaultInputGenerator
{

public:
    ComplianceMinTOPlatoAnalyzeUncertInputGenerator(const InputData& aInputData);
    virtual ~ComplianceMinTOPlatoAnalyzeUncertInputGenerator();
    virtual bool generateInputFiles();


protected:

    virtual bool generateInterfaceXML(std::ostringstream *aStringStream = NULL);
    virtual bool generateLaunchScript();
    virtual bool generatePlatoMainOperationsXML();
    virtual bool generatePlatoMainInputDeckXML();
    virtual bool generatePerformerOperationsXML();
    virtual bool generatePlatoAnalyzeOperationsXML();
    virtual bool generatePhysicsInputDecks();
    virtual bool generatePlatoAnalyzeInputDecks(std::ostringstream *aStringStream = NULL);
    virtual void outputUpdateProblemStage(pugi::xml_document &doc);
    virtual void outputOutputToFileStage(pugi::xml_document &doc,
                                 const bool &aHasUncertainties,
                                 const bool &aRequestedVonMises);
    virtual void addStochasticObjectiveValueOperation(pugi::xml_document &aDoc);
    virtual void addStochasticObjectiveGradientOperation(pugi::xml_document &aDoc);
    virtual void addVonMisesStatisticsOperation(pugi::xml_document &aDoc);
    virtual void addPlatoMainOutputOperation(pugi::xml_document &aDoc,
                                     const bool &aHasUncertainties,
                                     const bool &aRequestedVonMises);
    virtual bool addAggregateHessianOperation(pugi::xml_document &aDoc);
    virtual bool addAggregateEnergyOperation(pugi::xml_document &aDoc);
    virtual bool addAggregateGradientOperation(pugi::xml_document &aDoc);
    virtual void outputInitializeOptimizationStage(pugi::xml_document &doc);
    virtual void outputInitializeOptimizationStageForTO(pugi::xml_document &doc);
    virtual void outputCacheStateStage(pugi::xml_document &doc, const bool &aHasUncertainties);
    virtual bool outputObjectiveHessianStage(pugi::xml_document &doc);
    virtual bool outputObjectiveStage(pugi::xml_document &doc, const bool &aHasUncertainties);
    virtual bool outputObjectiveGradientStage(pugi::xml_document &doc,
                                           const bool &aHasUncertainties);

private:
    virtual size_t computeNumberOfNodesNeeded();
    virtual void generateBatchScript();
    virtual void generateJSRunScript();
    virtual void generateAnalyzeBashScripts();

};

}

#endif /* SRC_COMPMIN_TO_PA_UNCERT_INPUTGENERATOR_HPP_ */
