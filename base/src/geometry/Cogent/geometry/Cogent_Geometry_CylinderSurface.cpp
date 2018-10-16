/*
 * Cogent_Geometry_CylinderSurface.cpp
 *
 *  Created on: Dec 22, 2016
 *      Author: jrobbin
 */

#include "Cogent_Geometry_CylinderSurface.hpp"
#include "core/Cogent_ParameterFunction.hpp"

namespace Cogent {
namespace Geometry {

CylinderSurface::
CylinderSurface(
      std::vector<Teuchos::RCP<Cogent::ParameterFunction>> localParameters,
      int radiusIndex, Cogent::Geometry::Axis axis) :
 PrimitiveSurface(SurfaceType::CylinderSurface),
 m_radiusFunc(localParameters[radiusIndex]),
 m_axis(axis), m_radiusIndex(radiusIndex)
{

  if( m_axis == Axis::X ){ m_index1 = 1; m_index2 = 2; }
  else
  if( m_axis == Axis::Y ){ m_index1 = 0; m_index2 = 2; }
  else
  if( m_axis == Axis::Z ){ m_index1 = 0; m_index2 = 1; }

}

void 
CylinderSurface::compute(const RealType* X, RealType& val)
{
  val = compute(X);
}

void 
CylinderSurface::computeDeriv(const DFadType* X, RealType* vals)
{
  DFadType val;
  val = compute(X);
  int nDerivs = val.size();
  for(int iDeriv=0; iDeriv<nDerivs; iDeriv++)
    vals[iDeriv] = val.dx(iDeriv);
}

bool 
CylinderSurface::isEqual(const Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>& compareToSurf)
{
  const CylinderSurface* compareToCyl = dynamic_cast<const CylinderSurface*>(compareToSurf.get());

  if( compareToCyl ){
    return *this == *compareToCyl;
  } else return false;
}

bool
CylinderSurface::operator==(const CylinderSurface& rhs)
{
  return 
     ( std::tie(m_radiusFunc, m_axis) ==
       std::tie(rhs.m_radiusFunc, rhs.m_axis) );
}

} /* namespace Geometry */
} /* namespace Cogent */
