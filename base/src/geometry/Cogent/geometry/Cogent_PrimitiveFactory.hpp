/*
 * Cogent_PrimitiveFactory.hpp
 *
 *  Created on: Dec 12, 2016
 *      Author: jrobbin
 */

#ifndef COGENT_PRIMITIVEFACTORY_HPP_
#define COGENT_PRIMITIVEFACTORY_HPP_

#include <Teuchos_RCP.hpp>

#include "Cogent_Geometry_Primitive.hpp"
#include "core/Cogent_CoordinateSystem.hpp"


namespace Cogent {
namespace Geometry {

class PrimitiveFactory {
public:
  PrimitiveFactory();
  virtual ~PrimitiveFactory();

  Teuchos::RCP<Cogent::Geometry::Primitive> 
        create(const Teuchos::ParameterList& params, 
               const std::vector<std::string>& shapeParameterNames,
               Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem);

};

} /* namespace Geometry */
} /* namespace Cogent */

#endif /* COGENT_PRIMITIVEFACTORY_HPP_ */
