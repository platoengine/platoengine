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
               const std::vector<Plato::Performer*>& aPerformers,
               const std::vector<Plato::SharedData*>& aSharedData) :
  Operation(aOperationDataMng, aPerformers, aSharedData)
/******************************************************************************/
{
    // collect arrays of all input and output SharedData
    //
    const int tNumSubOperations = aOperationDataMng.getNumOperations();
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
        for(Plato::Performer* tPerformer : aPerformers)
        {
            if(tPerformer->myName() == tPerformerName)
            {
                m_performer = tPerformer;
            }
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
