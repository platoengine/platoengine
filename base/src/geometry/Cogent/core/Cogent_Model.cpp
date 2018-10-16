/*
 * Cogent_Model.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: jrobbin
 */

#include "Cogent_Model.hpp"

namespace Cogent {

Model::Model(const Teuchos::ParameterList& modelSpec) {
       // Parse generic inputs here
}

Model::~Model() {
	// TODO Auto-generated destructor stub
}

int Model::getSurfaceIndex()
{
  TEUCHOS_TEST_FOR_EXCEPTION(!m_surfaceIsPresent, std::runtime_error, 
    std::endl << "Cogent_Model: Requested levelset index that doesn't exist." << std::endl);
  return m_surfaceIndex;
}


} /* namespace Cogent */
