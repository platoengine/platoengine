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
 * Plato_OperationInputDataMng.hpp
 *
 *  Created on: Oct 10, 2017
 */

#ifndef PLATO_OPERATIONINPUTDATAMNG_HPP_
#define PLATO_OPERATIONINPUTDATAMNG_HPP_

#include <map>
#include <vector>
#include <string>
#include <utility>

#include "Plato_InputData.hpp"

namespace Plato
{

class OperationInputDataMng : public Plato::InputData
{
public:
    OperationInputDataMng();
    ~OperationInputDataMng();

    int getNumPerformers() const;
    int getNumOperations() const;
    bool hasSubOperations() const;

    const std::string & getPerformerName() const;
    const std::string & getPerformerName(const int & aOperationIndex) const;
    const std::string & getOperationName(const int & aOperationIndex) const;
    const std::string & getOperationName(const std::string & aPerformerName) const;

    int getNumInputs(const int & aOperationIndex) const;
    int getNumInputs(const std::string & aPerformerName) const;
    const std::vector<std::pair<std::string, std::string>> & getInputs(const int & aOperationIndex) const;
    const std::vector<std::pair<std::string, std::string>> & getInputs(const std::string & aPerformerName) const;
    const std::string & getInputSharedData(const int & aOperationIndex, const int & aInputDataIndex) const;
    const std::string & getInputSharedData(const std::string & aPerformerName, const int & aInputDataIndex) const;
    const std::string & getInputArgument(const int & aOperationIndex, const int & aInputDataIndex) const;
    const std::string & getInputArgument(const std::string & aPerformerName, const int & aInputDataIndex) const;

    int getNumOutputs(const int & aOperationIndex) const;
    int getNumOutputs(const std::string & aPerformerName) const;
    const std::vector<std::pair<std::string, std::string>> & getOutputs(const int & aOperationIndex) const;
    const std::vector<std::pair<std::string, std::string>> & getOutputs(const std::string & aPerformerName) const;
    const std::string & getOutputSharedData(const int & aOperationIndex, const int & aInputDataIndex) const;
    const std::string & getOutputSharedData(const std::string & aPerformerName, const int & aInputDataIndex) const;
    const std::string & getOutputArgument(const int & aOperationIndex, const int & aInputDataIndex) const;
    const std::string & getOutputArgument(const std::string & aPerformerName, const int & aInputDataIndex) const;

    void addInput(const std::string & aPerformerName,
                  const std::string & aOperationName,
                  const std::string & aSharedDataName,
                  const std::string & aArgumentName);
    void addInputs(const std::string & aPerformerName,
                   const std::string & aOperationName,
                   const std::vector<std::string> & aSharedDataNames,
                   const std::vector<std::string> & aArgumentNames);
    void addOutput(const std::string & aPerformerName,
                   const std::string & aOperationName,
                   const std::string & aSharedDataName,
                   const std::string & aArgumentName);
    void addOutputs(const std::string & aPerformerName,
                    const std::string & aOperationName,
                    const std::vector<std::string> & aSharedDataNames,
                    const std::vector<std::string> & aArgumentNames);


private:
    std::vector<std::string> mPerformerName;
    std::map<std::string, std::string> mOperationMap;
    std::map<std::string, std::vector< std::pair<std::string, std::string> > > mInputDataMap;
    std::map<std::string, std::vector< std::pair<std::string, std::string> > > mOutputDataMap;

    // USING DEFAULT COPY AND ASSIGNMENT CONSTRUCTORS
};

}

#endif /* PLATO_OPERATIONINPUTDATAMNG_HPP_ */
