/*
 * Cogent_Geometry_Cylinder.hpp
 *
 *  Created on: Nov 23, 2016
 *      Author: jrobbin
 */

#ifndef GEOMETRY_COGENT_GEOMETRY_CYLINDER_HPP_
#define GEOMETRY_COGENT_GEOMETRY_CYLINDER_HPP_

#include "core/Cogent_ParameterFunction.hpp"
#include "core/Cogent_CoordinateSystem.hpp"
#include "Cogent_Geometry_Primitive.hpp"
#include "Cogent_Geometry_Types.hpp"

namespace Cogent {
namespace Geometry {

class Cylinder: public Primitive {
public:
  Cylinder(const Teuchos::ParameterList& params,
           const std::vector<std::string>& shapeParameterNames,
           Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem);

  virtual ~Cylinder();

private:

  enum Parameter : int { Radius, Height };

  Axis m_axis;
};

} /* namespace Geometry */
} /* namespace Cogent */

#endif /* GEOMETRY_COGENT_GEOMETRY_CYLINDER_HPP_ */
