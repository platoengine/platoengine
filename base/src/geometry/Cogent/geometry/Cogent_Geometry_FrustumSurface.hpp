/*
 * Cogent_Geometry_FrustumSurface.hpp
 *
 *  Created on: July 31, 2017
 *      Author: jrobbin
 */

#ifndef COGENT_GEOMETRY_FRUSTUMSURFACE_HPP_
#define COGENT_GEOMETRY_FRUSTUMSURFACE_HPP_

#include "core/Cogent_ParameterFunction.hpp"
#include "Cogent_Geometry_PrimitiveSurface.hpp"
#include "Cogent_Geometry_Types.hpp"

namespace Cogent {
namespace Geometry {

class FrustumSurface: public PrimitiveSurface {
public:
  FrustumSurface(std::vector<Teuchos::RCP<Cogent::ParameterFunction>> localParameters,
        int radius0Index, int radius1Index, int heightIndex, Cogent::Geometry::Axis axis);
  virtual ~FrustumSurface(){}

  virtual bool isEqual(const Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>&);

  void update();
  void compute(const RealType* X, RealType& val);
  void computeDeriv(const DFadType* X, RealType* vals);

  bool operator==(const FrustumSurface& rhs);

private:
  Teuchos::RCP<Cogent::ParameterFunction> m_radius0Func;
  Teuchos::RCP<Cogent::ParameterFunction> m_radius1Func;
  Teuchos::RCP<Cogent::ParameterFunction> m_heightFunc;

  Axis     m_axis;

  int m_axisIndex, m_index1, m_index2;
//  int m_radius0Index, m_radius1Index, m_heightIndex;

  template <typename T> T compute(const T* X);

};

template <typename T>
T FrustumSurface::compute(const T* X){

  T eta = X[m_axisIndex];
  T d1 = X[m_index1];
  T d2 = X[m_index2];
  eta /= m_heightFunc->getValue<T>();
  T R0 = m_radius0Func->getValue<T>();
  T R1 = m_radius1Func->getValue<T>();
  T radius = R0*(1.0/2.0-eta) + R1*(1.0/2.0+eta);
  T val = radius - sqrt(d1*d1+d2*d2);
 
  return val;

}


} /* namespace Geometry */
} /* namespace Cogent */

#endif /* GEOMETRY_COGENT_GEOMETRY_BRICK_HPP_ */
