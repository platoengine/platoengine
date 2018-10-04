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
 * Plato_MultiOperation.cpp
 *
 *  Created on: March 30, 2018
 *
 */

#include <iostream>
#include <algorithm>
#include <sstream>

#include "Plato_Exceptions.hpp"
#include "Plato_MultiOperation.hpp"
#include "Plato_Performer.hpp"
#include "Plato_SharedData.hpp"
#include "Plato_Utils.hpp"
#include "Plato_OperationInputDataMng.hpp"

namespace Plato {

/******************************************************************************/
MultiOperation::
MultiOperation(const Plato::OperationInputDataMng & aOperationDataMng,
               const std::shared_ptr<Plato::Performer> aPerformer,
               const std::vector<Plato::SharedData*>& aSharedData) :
  Operation(aOperationDataMng, aPerformer, aSharedData)
/******************************************************************************/
{
    const int tNumSubOperations = aOperationDataMng.getNumOperations();
    for(int tSubOperationIndex = 0; tSubOperationIndex < tNumSubOperations; tSubOperationIndex++)
    {
        const std::string & tPerformerName = aOperationDataMng.getPerformerName(tSubOperationIndex);
        if(aPerformer->myName() != tPerformerName)
        {
             continue;
        }
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
    }

    // collect arrays of all input and output SharedData
    //
    for(int tSubOperationIndex = 0; tSubOperationIndex < tNumSubOperations; tSubOperationIndex++)
    {
        const std::string & tPerformerName = aOperationDataMng.getPerformerName(tSubOperationIndex);
        const int tNumInputs = aOperationDataMng.getNumInputs(tPerformerName);
        for(int tInputIndex = 0; tInputIndex < tNumInputs; tInputIndex++)
        {
            const std::string & tSharedDataName = aOperationDataMng.getInputSharedData(tPerformerName, tInputIndex);
            Plato::SharedData* tSharedData = Utils::byName(aSharedData, tSharedDataName);
            if(tSharedData != nullptr)
            {
                if(std::count(m_inputData.begin(), m_inputData.end(), tSharedData) == 0)
                {
                    m_inputData.push_back(tSharedData);
                }
            }
            else
            {
                std::stringstream tErrorMessage;
                tErrorMessage << "Plato::Operation: requested field ('" << tSharedDataName << "') that doesn't exist.";
                throw Plato::ParsingException(tErrorMessage.str());
            }
        }
        const int tNumOutputs = aOperationDataMng.getNumOutputs(tPerformerName);
        for(int tOutputIndex = 0; tOutputIndex < tNumOutputs; tOutputIndex++)
        {
            const std::string & tSharedDataName = aOperationDataMng.getOutputSharedData(tPerformerName, tOutputIndex);
            Plato::SharedData *tSharedData = Utils::byName(aSharedData, tSharedDataName);
            if(tSharedData != nullptr)
            {
                if(std::count(m_outputData.begin(), m_outputData.end(), tSharedData) == 0)
                {
                    m_outputData.push_back(tSharedData);
                }
            }
            else
            {
                std::stringstream tErrorMessage;
                tErrorMessage << "Plato::Operation: requested field ('" << tSharedDataName << "') that doesn't exist.";
                throw Plato::ParsingException(tErrorMessage.str());
            }
        }
    }

    // Loop on the sub-operation specs to See if any of the locally available
    // performers are involved in this operation.
    //
    for(int tSubOperationIndex = 0; tSubOperationIndex < tNumSubOperations; tSubOperationIndex++)
    {
        const std::string & tPerformerName = aOperationDataMng.getPerformerName(tSubOperationIndex);
        if(aPerformer->myName() == tPerformerName)
        {
            m_performer = aPerformer;
        }
        if(m_performer != nullptr)
        {

            // A local performer is participating in this operation, so parse the input
            // and output argument multimaps
            //
            m_operationName = aOperationDataMng.getOperationName(tPerformerName);

            const int tNumInputs = aOperationDataMng.getNumInputs(tPerformerName);
            for(int tInputIndex = 0; tInputIndex < tNumInputs; tInputIndex++)
            {
                const std::string & tArgumentName = aOperationDataMng.getInputArgument(tPerformerName, tInputIndex);
                const std::string & tSharedDataName = aOperationDataMng.getInputSharedData(tPerformerName, tInputIndex);
                for(Plato::SharedData* tSharedData : m_inputData)
                {
                    if(tSharedData->myName() == tSharedDataName)
                    {
                        m_argumentNames.insert(std::pair<std::string, std::string>(tSharedDataName, tArgumentName));
                        break;
                    }
                }
            }
            const int tNumOutputs = aOperationDataMng.getNumOutputs(tPerformerName);
            for(int tOutputIndex = 0; tOutputIndex < tNumOutputs; tOutputIndex++)
            {
                const std::string & tArgumentName = aOperationDataMng.getOutputArgument(tPerformerName, tOutputIndex);
                const std::string & tSharedDataName = aOperationDataMng.getOutputSharedData(tPerformerName, tOutputIndex);
                for(Plato::SharedData* tSharedData : m_outputData)
                {
                    if(tSharedData->myName() == tSharedDataName)
                    {
                        m_argumentNames.insert(std::pair<std::string, std::string>(tSharedDataName, tArgumentName));
                        break;
                    }
                }
            }
            break;
        }
    }
}
} // End namespace Plato
