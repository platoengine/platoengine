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
 * Plato_SingleOperation.cpp
 *
 *  Created on: March 30, 2018
 *
 */

#include <iostream>
#include <algorithm>
#include <sstream>

#include "Plato_Exceptions.hpp"
#include "Plato_SingleOperation.hpp"
#include "Plato_Performer.hpp"
#include "Plato_SharedData.hpp"
#include "Plato_Utils.hpp"
#include "Plato_OperationInputDataMng.hpp"

namespace Plato {

/******************************************************************************/
SingleOperation::
SingleOperation(const Plato::OperationInputDataMng & aOperationDataMng,
                const std::shared_ptr<Plato::Performer> aPerformer,
                const std::vector<Plato::SharedData*>& aSharedData) :
  Operation(aOperationDataMng, aPerformer, aSharedData)
/******************************************************************************/
{
    const std::string & tPerformerName = aOperationDataMng.getPerformerName();
    m_operationName = aOperationDataMng.getOperationName(tPerformerName);

    auto tAllParamsData = aOperationDataMng.get<Plato::InputData>("Parameters");
    if( tAllParamsData.size<Plato::InputData>(tPerformerName) )
    {
        auto tParamsData = tAllParamsData.get<Plato::InputData>(tPerformerName);
        for( auto tParamData : tParamsData.getByName<Plato::InputData>("Parameter") )
        {
            auto tArgName  = Plato::Get::String(tParamData,"ArgumentName");
            auto tArgValue = Plato::Get::Double(tParamData,"ArgumentValue");
            m_parameters.insert(
              std::pair<std::string, Parameter*>(tArgName, new Parameter(tArgName, m_operationName, tArgValue)));
        }
    }

    const int tNumInputs = aOperationDataMng.getNumInputs(tPerformerName);
    for(int tInputIndex = 0; tInputIndex < tNumInputs; tInputIndex++)
    {
        const std::string & tArgumentName = aOperationDataMng.getInputArgument(tPerformerName, tInputIndex);
        const std::string & tSharedDataName = aOperationDataMng.getInputSharedData(tPerformerName, tInputIndex);
        this->addArgument(tArgumentName, tSharedDataName, aSharedData, m_inputData);
    }
    const int tNumOutputs = aOperationDataMng.getNumOutputs(tPerformerName);
    for(int tOutputIndex = 0; tOutputIndex < tNumOutputs; tOutputIndex++)
    {
        const std::string & tArgumentName = aOperationDataMng.getOutputArgument(tPerformerName, tOutputIndex);
        const std::string & tSharedDataName = aOperationDataMng.getOutputSharedData(tPerformerName, tOutputIndex);
        this->addArgument(tArgumentName, tSharedDataName, aSharedData, m_outputData);
    }

    if(aPerformer->myName() == tPerformerName)
    {
        m_performer = aPerformer;
    }
}
} // End namespace Plato
