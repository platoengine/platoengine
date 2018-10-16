/*
 * Cogent_Geometry_TorusSurface.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: jrobbin
 */

#include "Cogent_Geometry_TorusSurface.hpp"
#include "core/Cogent_ParameterFunction.hpp"

namespace Cogent {
namespace Geometry {

TorusSurface::
TorusSurface(
      std::vector<Teuchos::RCP<Cogent::ParameterFunction>> localParameters, 
      int majorRadiusIndex, int minorRadiusIndex, Cogent::Geometry::Axis axis) :
 PrimitiveSurface(SurfaceType::TorusSurface),
 m_majorRadiusFunc(localParameters[majorRadiusIndex]),
 m_minorRadiusFunc(localParameters[minorRadiusIndex]),
 m_axis(axis) 
{ 
  if( m_axis == Axis::X ){ m_axisIndex = 0; m_index1 = 1; m_index2 = 2; }
  else
  if( m_axis == Axis::Y ){ m_axisIndex = 1; m_index1 = 0; m_index2 = 2; }
  else
  if( m_axis == Axis::Z ){ m_axisIndex = 2; m_index1 = 0; m_index2 = 1; }
}

void 
TorusSurface::compute(const RealType* X, RealType& val)
{
  val = compute(X);
}

void 
TorusSurface::computeDeriv(const DFadType* X, RealType* vals)
{
  DFadType val;
  val = compute(X);
  int nDerivs = val.size();
  for(int iDeriv=0; iDeriv<nDerivs; iDeriv++)
    vals[iDeriv] = val.dx(iDeriv);
}

bool 
TorusSurface::isEqual(const Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>& compareToSurf)
{
  const TorusSurface* compareToTorus = dynamic_cast<const TorusSurface*>(compareToSurf.get());

  if( compareToTorus ){
    return *this == *compareToTorus;
  } else return false;
}

bool
TorusSurface::operator==(const TorusSurface& rhs)
{
  return 
     ( std::tie(m_majorRadiusFunc,m_minorRadiusFunc,m_axis) ==
       std::tie(rhs.m_majorRadiusFunc,rhs.m_minorRadiusFunc,rhs.m_axis) );
}

} /* namespace Geometry */
} /* namespace Cogent */
