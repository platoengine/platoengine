/*
 * Cogent_ModelFactory.hpp
 *
 *  Created on: Dec 6, 2016
 *      Author: jrobbin
 */

#ifndef COGENT_MODELFACTORY_HPP_
#define COGENT_MODELFACTORY_HPP_

#include <Teuchos_RCP.hpp>

#include "Cogent_Model.hpp"

namespace Cogent {

class ModelFactory {
public:
	ModelFactory();
	virtual ~ModelFactory();

	Teuchos::RCP<Cogent::Model> create(const Teuchos::ParameterList& params);
};

} /* namespace Cogent */

#endif /* COGENT_MODELFACTORY_HPP_ */
