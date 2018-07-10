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
                const std::vector<Plato::Performer*>& aPerformers,
                const std::vector<Plato::SharedData*>& aSharedData) :
  Operation(aOperationDataMng, aPerformers, aSharedData)
/******************************************************************************/
{
    const std::string & tPerformerName = aOperationDataMng.getPerformerName();
    m_operationName = aOperationDataMng.getOperationName(tPerformerName);

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

    for(Plato::Performer* tPerformer : aPerformers)
    {
        if(tPerformer->myName() == tPerformerName)
        {
            m_performer = tPerformer;
        }
    }
}
} // End namespace Plato
