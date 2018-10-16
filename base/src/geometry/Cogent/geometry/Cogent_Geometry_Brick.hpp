/*
 * Cogent_Geometry_Brick.hpp
 *
 *  Created on: Dec 13, 2016
 *      Author: jrobbin
 */

#ifndef GEOMETRY_COGENT_GEOMETRY_BRICK_HPP_
#define GEOMETRY_COGENT_GEOMETRY_BRICK_HPP_

#include "core/Cogent_ParameterFunction.hpp"
#include "core/Cogent_CoordinateSystem.hpp"
#include "Cogent_Geometry_Types.hpp"
#include "Cogent_Geometry_Primitive.hpp"

namespace Cogent {
namespace Geometry {

class Brick: public Primitive {
public:
  Brick(const Teuchos::ParameterList& params, 
        const std::vector<std::string>& shapeParameterNames,
        Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem);
  virtual ~Brick();

private:
  enum Parameter : int { Xdim, Ydim, Zdim };

};

} /* namespace Geometry */
} /* namespace Cogent */

#endif /* GEOMETRY_COGENT_GEOMETRY_BRICK_HPP_ */
