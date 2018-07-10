/*
 * Plato_Operation.cpp
 *
 *  Created on: March 30, 2018
 *
 */

#include <iostream>
#include <algorithm>
#include <sstream>

#include "Plato_Exceptions.hpp"
#include "Plato_Operation.hpp"
#include "Plato_Performer.hpp"
#include "Plato_SharedData.hpp"
#include "Plato_Utils.hpp"
#include "Plato_OperationInputDataMng.hpp"

namespace Plato {

/******************************************************************************/
Operation::
Operation(const ::Plato::OperationInputDataMng & aOperationDataMng,
              const std::vector<::Plato::Performer*>& aPerformers,
              const std::vector<::Plato::SharedData*>& aSharedData) :
        m_performer(nullptr),
        m_operationName(),
        m_inputData(),
        m_outputData()
/******************************************************************************/
{

    // parse/create m_parameters
    auto tInputData = aOperationDataMng.get<Plato::InputData>("Input Data");
    
    m_operationName = Plato::Get::String(tInputData, "Name");
    for( auto tParameterData : tInputData.getByName<Plato::InputData>("Parameter") )
    {
        auto tArgName  = Plato::Get::String(tParameterData,"ArgumentName");
        auto tArgValue = Plato::Get::Double(tParameterData,"ArgumentValue");
        m_parameters.insert(std::pair<std::string, Parameter*>(tArgName, new Parameter(tArgName, m_operationName, tArgValue)));
    }
}

/******************************************************************************/
void 
Operation::
addArgument(const std::string & tArgumentName,
            const std::string & tSharedDataName,
            const std::vector<Plato::SharedData*>& aSharedData,
            std::vector<SharedData*>& aLocalData)
/******************************************************************************/
{
    bool tFoundData = false;
    for(Plato::SharedData* tSharedData : aSharedData)
    {
        if(tSharedData->myName() == tSharedDataName)
        {
            m_argumentNames.insert(std::pair<std::string, std::string>(tSharedDataName, tArgumentName));
            if(std::count(aLocalData.begin(), aLocalData.end(), tSharedData) == 0)
            {
                aLocalData.push_back(tSharedData);
                tFoundData = true;
            }
            break;
        }
    }
    if(!tFoundData)
    {
        std::stringstream tErrorMessage;
        tErrorMessage << "Plato::Operation: requested field ('" << tSharedDataName << "') that doesn't exist.";
        throw Plato::ParsingException(tErrorMessage.str());
    }
}

/******************************************************************************/
Operation::~Operation()
/******************************************************************************/
{

}

/******************************************************************************/
void
Operation::
sendInput()
/******************************************************************************/
{
  for( SharedData* sd : m_inputData )
    sd->transmitData();
}

/******************************************************************************/
void
Operation::
sendOutput()
/******************************************************************************/
{
  for( SharedData* sd : m_outputData )
    sd->transmitData();
}

/******************************************************************************/
std::vector<std::string>
Operation::
getInputDataNames() const
/******************************************************************************/
{
  std::vector<std::string> names;
  for(SharedData* sf : m_inputData)
    names.push_back(sf->myName());

  return names;
}

/******************************************************************************/
std::vector<std::string>
Operation::
getOutputDataNames() const
/******************************************************************************/
{
  std::vector<std::string> names;
  for(SharedData* sf : m_outputData)
    names.push_back(sf->myName());

  return names;
}

/******************************************************************************/
void 
Operation::
compute()    
/******************************************************************************/
{
  if(m_performer)
  {
     for( auto p : m_parameters )
     {
       m_performer->importData(p.first, *(p.second));
     }
     m_performer->compute(m_operationName);
  }
}

/******************************************************************************/
void 
Operation::
importData(std::string aSharedDataName, SharedData* aImportData)
/******************************************************************************/
{
  if(m_performer){
    auto range = m_argumentNames.equal_range(aSharedDataName);
    for( auto it = range.first; it != range.second; ++it ){
      m_performer->importData(it->second, *aImportData);
    }
  }
}

/******************************************************************************/
void 
Operation::
exportData(std::string aSharedDataName, SharedData* aExportData)
/******************************************************************************/
{
  if(m_performer){
    auto range = m_argumentNames.equal_range(aSharedDataName);
    for( auto it = range.first; it != range.second; ++it ){
      m_performer->exportData(it->second, *aExportData);
    }
  }
}

/******************************************************************************/
std::string 
Operation::
getPerformerName() const
/******************************************************************************/
{
    return m_performer->myName();
}

/******************************************************************************/
std::string 
Operation::
getOperationName() const
/******************************************************************************/
{
    return m_operationName;
}

} // End namespace Plato
