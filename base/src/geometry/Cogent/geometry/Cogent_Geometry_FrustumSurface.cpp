/*
 * Cogent_Geometry_FrustumSurface.cpp
 *
 *  Created on: Dec 22, 2016
 *      Author: jrobbin
 */

#include "Cogent_Geometry_FrustumSurface.hpp"
#include "core/Cogent_ParameterFunction.hpp"

namespace Cogent {
namespace Geometry {

FrustumSurface::
FrustumSurface(
      std::vector<Teuchos::RCP<Cogent::ParameterFunction>> localParameters,
      int radius0Index, int radius1Index, int heightIndex, Cogent::Geometry::Axis axis):
 PrimitiveSurface(SurfaceType::FrustumSurface),
 m_radius0Func(localParameters[radius0Index]),
 m_radius1Func(localParameters[radius1Index]),
 m_heightFunc(localParameters[heightIndex]),
 m_axis(axis)
{

  if( m_axis == Axis::X ){ m_axisIndex = 0; m_index1 = 1; m_index2 = 2; }
  else
  if( m_axis == Axis::Y ){ m_axisIndex = 1; m_index1 = 0; m_index2 = 2; }
  else
  if( m_axis == Axis::Z ){ m_axisIndex = 2; m_index1 = 0; m_index2 = 1; }

}

void 
FrustumSurface::compute(const RealType* X, RealType& val)
{
  val = compute(X);
}

void 
FrustumSurface::computeDeriv(const DFadType* X, RealType* vals)
{
  DFadType val;
  val = compute(X);
  int nDerivs = val.size();
  for(int iDeriv=0; iDeriv<nDerivs; iDeriv++)
    vals[iDeriv] = val.dx(iDeriv);
}

bool 
FrustumSurface::isEqual(const Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>& compareToSurf)
{
  const FrustumSurface* compareToCyl = dynamic_cast<const FrustumSurface*>(compareToSurf.get());

  if( compareToCyl ){
    return *this == *compareToCyl;
  } else return false;
}

bool
FrustumSurface::operator==(const FrustumSurface& rhs)
{
  return 
     ( std::tie(m_radius0Func, m_radius1Func, m_heightFunc, m_axis) ==
       std::tie(rhs.m_radius0Func, m_radius1Func, m_heightFunc, rhs.m_axis) );
}

} /* namespace Geometry */
} /* namespace Cogent */
