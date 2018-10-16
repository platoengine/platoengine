/*
 * Cogent_Geometry_SphereSurface.cpp
 *
 *  Created on: July 31, 2017
 *      Author: jrobbin
 */

#include "Cogent_Geometry_SphereSurface.hpp"
#include "core/Cogent_ParameterFunction.hpp"

namespace Cogent {
namespace Geometry {

SphereSurface::
SphereSurface(
      std::vector<Teuchos::RCP<Cogent::ParameterFunction>> localParameters, int radiusIndex) :
 PrimitiveSurface(SurfaceType::SphereSurface),
 m_radiusFunc(localParameters[radiusIndex]) { }

void 
SphereSurface::compute(const RealType* X, RealType& val)
{
  val = compute(X);
}

void 
SphereSurface::computeDeriv(const DFadType* X, RealType* vals)
{
  DFadType val;
  val = compute(X);
  int nDerivs = val.size();
  for(int iDeriv=0; iDeriv<nDerivs; iDeriv++)
    vals[iDeriv] = val.dx(iDeriv);
}

bool 
SphereSurface::isEqual(const Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>& compareToSurf)
{
  const SphereSurface* compareToCyl = dynamic_cast<const SphereSurface*>(compareToSurf.get());

  if( compareToCyl ){
    return *this == *compareToCyl;
  } else return false;
}

bool
SphereSurface::operator==(const SphereSurface& rhs)
{
  return 
     ( std::tie(m_radiusFunc) ==
       std::tie(rhs.m_radiusFunc) );
}

} /* namespace Geometry */
} /* namespace Cogent */
