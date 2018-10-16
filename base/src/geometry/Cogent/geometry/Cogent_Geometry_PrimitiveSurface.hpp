/*
 * Cogent_Surface_Primitive.hpp
 *
 *  Created on: Dec 21, 2016
 *      Author: jrobbin
 */

#ifndef COGENT_GEOMETRY_PRIMITIVESURFACE_HPP_
#define COGENT_GEOMETRY_PRIMITIVESURFACE_HPP_

#include <Teuchos_ParameterList.hpp>

#include "core/Cogent_Types.hpp"
#include "Cogent_Geometry_Types.hpp"
#include "core/Cogent_ParameterFunction.hpp"

namespace Cogent {
namespace Geometry {

class PrimitiveSurface {
public:
  PrimitiveSurface(SurfaceType st) : c_surfaceType(st){}
  virtual ~PrimitiveSurface(){}
//  virtual void update(){}
  virtual void compute(const RealType* X, RealType& val)=0;
  virtual void computeDeriv(const DFadType* X, RealType* vals)=0;

  virtual bool isEqual(const Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>&);

  SurfaceType getSurfaceType() const { return c_surfaceType; }
protected:

  const SurfaceType c_surfaceType;
  
};

} /* namespace Geometry */
} /* namespace Cogent */

#endif /* COGENT_GEOMETRY_PRIMITIVESURFACE_HPP_ */
