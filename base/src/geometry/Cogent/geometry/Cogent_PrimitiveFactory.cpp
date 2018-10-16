/*
 * Cogent_PrimitiveFactory.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: jrobbin
 */

#include <string>

#include "Cogent_PrimitiveFactory.hpp"

#include "Cogent_Geometry_Brick.hpp"
#include "Cogent_Geometry_Cylinder.hpp"
#include "Cogent_Geometry_Frustum.hpp"
#include "Cogent_Geometry_Sphere.hpp"
#include "Cogent_Geometry_Torus.hpp"

namespace Cogent {
namespace Geometry {

PrimitiveFactory::PrimitiveFactory() { }

PrimitiveFactory::~PrimitiveFactory() { }

Teuchos::RCP<Cogent::Geometry::Primitive> 
PrimitiveFactory::create(const Teuchos::ParameterList& params, 
                         const std::vector<std::string>& shapeParameterNames,
                         Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem){

  //parse
  
  std::string primitiveType = params.get<std::string>("Type");
  
  if(primitiveType == "Brick"){
    return Teuchos::rcp(new Cogent::Geometry::Brick(params, shapeParameterNames, 
                                                    coordinateSystem));
  } else 
  if(primitiveType == "Cylinder"){
    return Teuchos::rcp(new Cogent::Geometry::Cylinder(params, shapeParameterNames, 
                                                       coordinateSystem));
  } else 
  if(primitiveType == "Frustum"){
    return Teuchos::rcp(new Cogent::Geometry::Frustum(params, shapeParameterNames, 
                                                       coordinateSystem));
  } else 
  if(primitiveType == "Sphere"){
    return Teuchos::rcp(new Cogent::Geometry::Sphere(params, shapeParameterNames, 
                                                       coordinateSystem));
  } else 
  if(primitiveType == "Torus"){
    return Teuchos::rcp(new Cogent::Geometry::Torus(params, shapeParameterNames, 
                                                       coordinateSystem));
  } else {
    TEUCHOS_TEST_FOR_EXCEPTION(true, std::logic_error,
      std::endl << "Cogent_PrimitiveFactory: 'Type' " << primitiveType << 
      " not recognized." << std::endl);
  }

}

} /* namespace Geometry */
} /* namespace Cogent */
