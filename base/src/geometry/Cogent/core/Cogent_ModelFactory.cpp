/*
 * Cogent_ModelFactory.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: jrobbin
 */

#include <string>

#include "Cogent_ModelFactory.hpp"

#include "Cogent_ParameterizedModel.hpp"
#include "Cogent_NonParameterizedModel.hpp"

namespace Cogent {

ModelFactory::ModelFactory() { }

ModelFactory::~ModelFactory() { }

Teuchos::RCP<Model> ModelFactory::create(const Teuchos::ParameterList& params) {

  //parse
  
  std::string modelType = params.get<std::string>("Model Type");

  if(modelType == "Parameterized"){
    return Teuchos::rcp(new Cogent::ParameterizedModel(params));
  } else 
  if(modelType == "Non-parameterized"){
    return Teuchos::rcp(new Cogent::NonParameterizedModel(params));
  } else {
    TEUCHOS_TEST_FOR_EXCEPTION(true, std::logic_error,
      std::endl << "Cogent_ModelFactory: 'Model Type' " << modelType << 
      " not recognized.  Known types are 'Parameterized' or 'Non-parameterized'. "
      << std::endl);
  }
  


}

} /* namespace Cogent */
