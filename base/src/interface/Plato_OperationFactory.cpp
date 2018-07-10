/*
 * Plato_OperationFactory.cpp
 *
 *  Created on: March 30, 2018
 *
 */

#include <iostream>
#include <algorithm>
#include <sstream>

#include "Plato_Exceptions.hpp"

#include "Plato_OperationFactory.hpp"
#include "Plato_Operation.hpp"
#include "Plato_MultiOperation.hpp"
#include "Plato_SingleOperation.hpp"

#include "Plato_Parser.hpp"
#include "Plato_Performer.hpp"
#include "Plato_SharedData.hpp"
#include "Plato_Utils.hpp"
#include "Plato_OperationInputDataMng.hpp"

namespace Plato {

/******************************************************************************/
Operation*
OperationFactory::create(
  const Plato::OperationInputDataMng & aOperationDataMng,
  const std::vector<Plato::Performer*>& aPerformers,
  const std::vector<Plato::SharedData*>& aSharedData)
/******************************************************************************/
{
    Plato::InputData inputNode = aOperationDataMng.get<Plato::InputData>("Input Data");

    std::string opType = inputNode.name();
    if( opType == "Operation" ){
      bool tHasSubOperations = aOperationDataMng.hasSubOperations();
      if(tHasSubOperations == true)
      {
        return new MultiOperation(aOperationDataMng, aPerformers, aSharedData);
      }
      else
      {
        return new SingleOperation(aOperationDataMng, aPerformers, aSharedData);
      }
    }
    return nullptr;
}
} // End namespace Plato
