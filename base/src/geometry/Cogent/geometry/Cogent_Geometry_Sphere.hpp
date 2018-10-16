/*
 * Cogent_Geometry_Sphere.hpp
 *
 *  Created on: July 31, 2017
 *      Author: jrobbin
 */

#ifndef GEOMETRY_COGENT_GEOMETRY_SPHERE_HPP_
#define GEOMETRY_COGENT_GEOMETRY_SPHERE_HPP_

#include "core/Cogent_ParameterFunction.hpp"
#include "core/Cogent_CoordinateSystem.hpp"
#include "Cogent_Geometry_Primitive.hpp"
#include "Cogent_Geometry_Types.hpp"

namespace Cogent {
namespace Geometry {

class Sphere: public Primitive {
public:
  Sphere(const Teuchos::ParameterList& params,
           const std::vector<std::string>& shapeParameterNames,
           Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem);

  virtual ~Sphere();

private:

  enum Parameter : int { Radius };

};

} /* namespace Geometry */
} /* namespace Cogent */

#endif /* GEOMETRY_COGENT_GEOMETRY_SPHERE_HPP_ */
