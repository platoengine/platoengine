/*
 * Cogent_Geometry_Plane.hpp
 *
 *  Created on: Dec 21, 2016
 *      Author: jrobbin
 */

#ifndef COGENT_GEOMETRY_PLANE_HPP_
#define COGENT_GEOMETRY_PLANE_HPP_

#include "core/Cogent_ParameterFunction.hpp"
#include "Cogent_Geometry_PrimitiveSurface.hpp"
#include "Cogent_Geometry_Types.hpp"

namespace Cogent {
namespace Geometry {

class Plane: public PrimitiveSurface {
public:
  Plane(std::vector<Teuchos::RCP<Cogent::ParameterFunction>> localParameters,
        int dimIndex, RealType dimScale,
        Cogent::Geometry::Axis axis, Cogent::Sense sense);
  virtual ~Plane(){}

  bool isEqual(const Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>&);
  bool operator==(const Plane& rhs);

//  void update();
  void compute(const RealType* X, RealType& val);
  void computeDeriv(const DFadType* X, RealType* vals);

private:
  Teuchos::RCP<Cogent::ParameterFunction> m_dimFunc;

  Axis     m_axis;
  Sense    m_sense;
  RealType m_scale;
  RealType m_realSense;

  int m_vecIndex;
  int m_dimIndex;

  template <typename T> T compute(const T* X);

};

template <typename T> 
T Plane::compute(const T* X){
  T val = m_realSense*(X[m_vecIndex] - m_scale*m_dimFunc->getValue<T>());
  return val;
}




} /* namespace Geometry */
} /* namespace Cogent */

#endif /* GEOMETRY_COGENT_GEOMETRY_BRICK_HPP_ */
