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
 * PlatoAnalyzeInputDeckWriter.hpp
 *
 *  Created on: Nov 19, 2019
 *
 */

#ifndef SRC_PLATOANALYZEINPUTDECKWRITER_HPP_
#define SRC_PLATOANALYZEINPUTDECKWRITER_HPP_

#include "Plato_Parser.hpp"
#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{

class PlatoAnalyzeInputDeckWriter
{

public:
    PlatoAnalyzeInputDeckWriter(const InputData &aInputData);
    ~PlatoAnalyzeInputDeckWriter();
    bool generate(std::ostringstream *aStringStream = NULL);

protected:

private:
    void buildMinimizeThermoelasticEnergyParamsForPlatoAnalyze(const XMLGen::Objective& cur_obj, pugi::xml_node aNode);
    void buildMaximizeStiffnessParamsForPlatoAnalyze(const XMLGen::Objective& cur_obj, pugi::xml_node aNode);
    void buildMaximizeHeatConductionParamsForPlatoAnalyze(const XMLGen::Objective& cur_obj, pugi::xml_node aNode);
    void addVolumeConstraintForPlatoAnalyze(pugi::xml_node aNode);
    void addPAObjectiveBlock(pugi::xml_node aNode, const char* aObjectiveName);
    void addPAPDEConstraintBlock(pugi::xml_node aNode, const char* aPDEConstraintName);
    void addPAMaterialModelBlock(pugi::xml_node aNode, const char* aPDEConstraintName);
    void buildThermalNBCsForPlatoAnalyze(const XMLGen::Objective& aObjective, pugi::xml_node aNode, const std::string &aTitle, int &aBCCounter);
    void buildThermalEBCsForPlatoAnalyze(const XMLGen::Objective& aObjective, pugi::xml_node aNode, int &aBCCounter,
                                         const char* aVariableIndex);
    void buildMechanicalNBCsForPlatoAnalyze(const XMLGen::Objective& cur_obj, pugi::xml_node aNode, const std::string &aTitle, int &aBCCounter);
    void buildMechanicalEBCsForPlatoAnalyze(const XMLGen::Objective& cur_obj, pugi::xml_node aNode, int &aBCCounter);
    bool checkForNodesetSidesetNameConflicts();

    const InputData &mInputData;
};

}

#endif /* SRC_PLATOANALYZEINPUTDECKWRITER_HPP_ */
