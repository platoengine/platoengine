/*
 * Cogent_Geometry_Frustum.hpp
 *
 *  Created on: Nov 23, 2016
 *      Author: jrobbin
 */

#ifndef GEOMETRY_COGENT_GEOMETRY_FRUSTUM_HPP_
#define GEOMETRY_COGENT_GEOMETRY_FRUSTUM_HPP_

#include "core/Cogent_ParameterFunction.hpp"
#include "core/Cogent_CoordinateSystem.hpp"
#include "Cogent_Geometry_Primitive.hpp"
#include "Cogent_Geometry_Types.hpp"

namespace Cogent {
namespace Geometry {

class Frustum: public Primitive {
public:
  Frustum(const Teuchos::ParameterList& params,
          const std::vector<std::string>& shapeParameterNames,
          Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem);

  virtual ~Frustum();

private:

  enum Parameter : int { Radius0, Radius1, Height };

  Axis m_axis;
};

} /* namespace Geometry */
} /* namespace Cogent */

#endif /* GEOMETRY_COGENT_GEOMETRY_FRUSTUM_HPP_ */
