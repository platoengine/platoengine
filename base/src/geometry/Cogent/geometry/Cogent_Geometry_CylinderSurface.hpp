/*
 * Cogent_Geometry_CylinderSurface.hpp
 *
 *  Created on: Dec 22, 2016
 *      Author: jrobbin
 */

#ifndef COGENT_GEOMETRY_CYLINDERSURFACE_HPP_
#define COGENT_GEOMETRY_CYLINDERSURFACE_HPP_

#include "core/Cogent_ParameterFunction.hpp"
#include "Cogent_Geometry_PrimitiveSurface.hpp"
#include "Cogent_Geometry_Types.hpp"

namespace Cogent {
namespace Geometry {

class CylinderSurface: public PrimitiveSurface {
public:
  CylinderSurface(std::vector<Teuchos::RCP<Cogent::ParameterFunction>> localParameters,
        int radiusIndex, Cogent::Geometry::Axis axis);
  virtual ~CylinderSurface(){}

  virtual bool isEqual(const Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>&);

  void update();
  void compute(const RealType* X, RealType& val);
  void computeDeriv(const DFadType* X, RealType* vals);

  bool operator==(const CylinderSurface& rhs);

private:
  Teuchos::RCP<Cogent::ParameterFunction> m_radiusFunc;

  Axis     m_axis;

  int m_index1, m_index2;
  int m_radiusIndex;

  template <typename T> T compute(const T* X);

};

template <typename T>
T CylinderSurface::compute(const T* X){

  T d1 = X[m_index1];
  T d2 = X[m_index2];
  T val = m_radiusFunc->getValue<T>() - sqrt(d1*d1+d2*d2);
 
  return val;

}


} /* namespace Geometry */
} /* namespace Cogent */

#endif /* GEOMETRY_COGENT_GEOMETRY_BRICK_HPP_ */
