/*
 * Cogent_Geometry_SphereSurface.hpp
 *
 *  Created on: July 31, 2017
 *      Author: jrobbin
 */

#ifndef COGENT_GEOMETRY_SPHERESURFACE_HPP_
#define COGENT_GEOMETRY_SPHERESURFACE_HPP_

#include "core/Cogent_ParameterFunction.hpp"
#include "Cogent_Geometry_PrimitiveSurface.hpp"
#include "Cogent_Geometry_Types.hpp"

namespace Cogent {
namespace Geometry {

class SphereSurface: public PrimitiveSurface {
public:
  SphereSurface(std::vector<Teuchos::RCP<Cogent::ParameterFunction>> localParameters, int radiusIndex);
  virtual ~SphereSurface(){}

  virtual bool isEqual(const Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>&);

  void update();
  void compute(const RealType* X, RealType& val);
  void computeDeriv(const DFadType* X, RealType* vals);

  bool operator==(const SphereSurface& rhs);

private:
  Teuchos::RCP<Cogent::ParameterFunction> m_radiusFunc;

  int m_radiusIndex;

  template <typename T> T compute(const T* X);

};

template <typename T>
T SphereSurface::compute(const T* X){

  T d0 = X[0];
  T d1 = X[1];
  T d2 = X[2];
  T val = m_radiusFunc->getValue<T>() - sqrt(d0*d0+d1*d1+d2*d2);
 
  return val;

}

} /* namespace Geometry */
} /* namespace Cogent */

#endif /* COGENT_GEOMETRY_SPHERESURFACE_HPP_ */
