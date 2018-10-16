/*
 * Cogent_Geometry_TorusSurface.hpp
 *
 *  Created on: July 31, 2017
 *      Author: jrobbin
 */

#ifndef COGENT_GEOMETRY_TORUSSURFACE_HPP_
#define COGENT_GEOMETRY_TORUSSURFACE_HPP_

#include "core/Cogent_ParameterFunction.hpp"
#include "Cogent_Geometry_PrimitiveSurface.hpp"
#include "Cogent_Geometry_Types.hpp"

namespace Cogent {
namespace Geometry {

class TorusSurface: public PrimitiveSurface {
public:
  TorusSurface(std::vector<Teuchos::RCP<Cogent::ParameterFunction>> localParameters, 
               int majorRadiusIndex, int minorRadiusIndex, Cogent::Geometry::Axis axis);
  virtual ~TorusSurface(){}

  virtual bool isEqual(const Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>&);

  void update();
  void compute(const RealType* X, RealType& val);
  void computeDeriv(const DFadType* X, RealType* vals);

  bool operator==(const TorusSurface& rhs);

private:
  Teuchos::RCP<Cogent::ParameterFunction> m_majorRadiusFunc;
  Teuchos::RCP<Cogent::ParameterFunction> m_minorRadiusFunc;

  Axis m_axis;
  int m_index1, m_index2;
  int m_axisIndex;

  template <typename T> T compute(const T* X);

};

template <typename T>
T TorusSurface::compute(const T* X){

  T d0 = X[m_axisIndex];
  T d1 = X[m_index1];
  T d2 = X[m_index2];
  T magInPlane_squared = d1*d1+d2*d2;
  T magInPlane = sqrt(magInPlane_squared);
  T scaleToMinor = 1.0 - 1.0/magInPlane*m_majorRadiusFunc->getValue<T>();
  T magMinor_squared = d0*d0 + scaleToMinor*scaleToMinor*magInPlane_squared;
  T val = m_minorRadiusFunc->getValue<T>() - sqrt(magMinor_squared);
 
  return val;

}

} /* namespace Geometry */
} /* namespace Cogent */

#endif /* COGENT_GEOMETRY_TORUSSURFACE_HPP_ */
