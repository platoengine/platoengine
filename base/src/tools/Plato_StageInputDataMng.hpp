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
 * Plato_StageInputDataMng.hpp
 *
 *  Created on: Oct 10, 2017
 */

#ifndef PLATO_STAGEINPUTDATAMNG_HPP_
#define PLATO_STAGEINPUTDATAMNG_HPP_

#include <map>
#include <vector>
#include <string>
#include <utility>

#include "Plato_InputData.hpp"

namespace Plato
{

class OperationInputDataMng;

class StageInputDataMng : public Plato::InputData
{
public:
    StageInputDataMng();
    ~StageInputDataMng();

    int getNumStages() const;
    const std::string & getStageName() const;
    const std::string & getStageName(const int & aStageIndex) const;

    int getNumInputs() const;
    int getNumInputs(const int & aStageIndex) const;
    int getNumInputs(const std::string & aStageName) const;
    const std::vector<std::string> & getInputs(const int & aStageIndex) const;
    const std::string & getInput(const int & aStageIndex, const int & aInputIndex) const;
    const std::vector<std::string> & getInputs(const std::string & aStageName) const;
    const std::string & getInput(const std::string & aStageName, const int & aInputIndex) const;

    int getNumOutputs() const;
    int getNumOutputs(const int & aStageIndex) const;
    int getNumOutputs(const std::string & aStageName) const;
    const std::vector<std::string> & getOutputs(const int & aStageIndex) const;
    const std::string & getOutput(const int & aStageIndex, const int & aOutputIndex) const;
    const std::vector<std::string> & getOutputs(const std::string & aStageName) const;
    const std::string & getOutput(const std::string & aStageName, const int & aOutputIndex) const;

    void add(const std::string & aStageName);
    void add(const std::string & aStageName, const std::vector<std::string> & aInputs, const std::vector<std::string> & aOutputs);

    void addInputs(const std::string & aStageName, const std::vector<std::string> & aInputs);
    void addOutputs(const std::string & aStageName, const std::vector<std::string> & aOutputs);

    int getNumOperations() const;
    int getNumOperations(const int & aStageIndex) const;
    int getNumOperations(const std::string & aStageName) const;
    const Plato::OperationInputDataMng & getOperationInputData(const int & aStageIndex, const int & aOperationIndex) const;
    const Plato::OperationInputDataMng & getOperationInputData(const std::string & aStageName, const int & aOperationIndex) const;
    void addOperationInputData(const std::string & aStageName, const Plato::OperationInputDataMng & aOperationInputData);
    void addOperationInputData(const std::string & aStageName, const std::vector<Plato::OperationInputDataMng> & aOperationInputData);

private:
    std::vector<std::string> mStageNames;
    std::map<std::string, std::vector<Plato::OperationInputDataMng>> mOperationInputs;
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>> mSharedDataMap;

    // USING DEFAULT COPY AND ASSIGNMENT CONSTRUCTORS
};

}

#endif /* PLATO_STAGEINPUTDATAMNG_HPP_ */
