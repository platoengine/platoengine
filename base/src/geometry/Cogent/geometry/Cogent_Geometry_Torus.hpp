/*
 * Cogent_Geometry_Torus.hpp
 *
 *  Created on: July 31, 2017
 *      Author: jrobbin
 */

#ifndef GEOMETRY_COGENT_GEOMETRY_TORUS_HPP_
#define GEOMETRY_COGENT_GEOMETRY_TORUS_HPP_

#include "core/Cogent_ParameterFunction.hpp"
#include "core/Cogent_CoordinateSystem.hpp"
#include "Cogent_Geometry_Primitive.hpp"
#include "Cogent_Geometry_Types.hpp"

namespace Cogent {
namespace Geometry {

class Torus: public Primitive {
public:
  Torus(const Teuchos::ParameterList& params,
           const std::vector<std::string>& shapeParameterNames,
           Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem);

  virtual ~Torus();

private:

  enum Parameter : int { MajorRadius, MinorRadius };

  Axis m_axis;

};

} /* namespace Geometry */
} /* namespace Cogent */

#endif /* GEOMETRY_COGENT_GEOMETRY_TORUS_HPP_ */
