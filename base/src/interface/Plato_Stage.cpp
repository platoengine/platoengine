/*
 * Plato_Stage.cpp
 *
 *  Created on: April 24, 2017
 *
 */

#include "Plato_Stage.hpp"
#include "Plato_Operation.hpp"
#include "Plato_OperationFactory.hpp"
#include "Plato_SharedData.hpp"
#include "Plato_Performer.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_Utils.hpp"
#include "Plato_StageInputDataMng.hpp"
#include "Plato_OperationInputDataMng.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <iostream>

namespace Plato
{

/******************************************************************************/
Stage::Stage(const Plato::StageInputDataMng & aStageInputData,
             const std::vector<Plato::Performer*>& aPerformers,
             const std::vector<Plato::SharedData*>& aSharedData) :
        m_name(aStageInputData.getStageName()),
        m_operations(),
        m_inputData(),
        m_outputData(),
        currentOperationIndex()
/******************************************************************************/
{
    int tNumInputs = aStageInputData.getNumInputs();
    for(int tInputIndex = 0; tInputIndex < tNumInputs; tInputIndex++)
    {
        std::string tSharedDataName = aStageInputData.getInput(m_name, tInputIndex);
        Plato::SharedData* tSharedData = Utils::byName(aSharedData, tSharedDataName);
        if(tSharedData)
        {
            m_inputData.push_back(tSharedData);
        }
        else
        {
            std::stringstream tErrorMessage;
            tErrorMessage << "While parsing Stage '" << m_name << "', requested SharedData ('" << tSharedDataName << "') which doesn't exist.";
            throw Plato::ParsingException(tErrorMessage.str());
        }
    }

    // TODO: find input SharedValues
    //

    int tNumOutputs = aStageInputData.getNumOutputs();
    for(int tOutputIndex = 0; tOutputIndex < tNumOutputs; tOutputIndex++)
    {
        std::string tSharedDataName = aStageInputData.getOutput(m_name, tOutputIndex);
        Plato::SharedData* tSharedData = Utils::byName(aSharedData, tSharedDataName);
        if(tSharedData)
        {
            m_outputData.push_back(tSharedData);
        }
        else
        {
            std::stringstream tErrorMessage;
            tErrorMessage << "While parsing Stage '" << m_name << "', requested SharedData ('" << tSharedDataName << "') which doesn't exist.";
            throw Plato::ParsingException(tErrorMessage.str());
        }
    }

    // TODO: find output SharedValues
    //

    // parse/create Operations
    //
    Plato::OperationFactory opFactory;

    const int tNumOperations = aStageInputData.getNumOperations(m_name);
    for(int tOperationIndex = 0; tOperationIndex < tNumOperations; tOperationIndex++)
    {
        const Plato::OperationInputDataMng & tOperationDataMng = aStageInputData.getOperationInputData(m_name, tOperationIndex);
        m_operations.push_back(opFactory.create(tOperationDataMng, aPerformers, aSharedData));
    }

}

/******************************************************************************/
Stage::~Stage()
/******************************************************************************/
{
    const size_t num_operations = m_operations.size();
    for(size_t operation_index = 0u; operation_index < num_operations; operation_index++)
    {
        delete m_operations[operation_index];
    }
    m_operations.clear();
}

/******************************************************************************/
std::vector<std::string> Stage::getInputDataNames() const
/******************************************************************************/
{
    std::vector<std::string> tNames;
    for(Plato::SharedData* tSharedData : m_inputData)
    {
        tNames.push_back(tSharedData->myName());
    }
    return tNames;
}

/******************************************************************************/
std::vector<std::string> Stage::getOutputDataNames() const
/******************************************************************************/
{
    std::vector<std::string> tNames;
    for(Plato::SharedData* tSharedData : m_outputData)
    {
        tNames.push_back(tSharedData->myName());
    }
    return tNames;
}
/******************************************************************************/
void Stage::begin()
/******************************************************************************/
{
    for(Plato::SharedData* tSharedData : m_inputData)
    {
        tSharedData->transmitData();
    }
    // reset to first operation
    currentOperationIndex = 0;
}

/******************************************************************************/
Plato::Operation*
Stage::getNextOperation()
/******************************************************************************/
{
    Plato::Operation* tOperation = nullptr;
    if(static_cast<int>(m_operations.size()) > currentOperationIndex)
    {
        tOperation = m_operations[currentOperationIndex];
    }
    currentOperationIndex++;
    return tOperation;
}

/******************************************************************************/
void Stage::end()
/******************************************************************************/
{
    for(Plato::SharedData* tSharedData : m_outputData)
    {
        tSharedData->transmitData();
    }
}

} // End namespace Plato
