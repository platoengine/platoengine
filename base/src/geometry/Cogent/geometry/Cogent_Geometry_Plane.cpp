/*
 * Cogent_Geometry_Plane.cpp
 *
 *  Created on: Dec 21, 2016
 *      Author: jrobbin
 */

#include "Cogent_Geometry_Plane.hpp"
#include "core/Cogent_ParameterFunction.hpp"

namespace Cogent {
namespace Geometry {

Plane::
Plane(std::vector<Teuchos::RCP<Cogent::ParameterFunction>> localParameters,
      int dimIndex, RealType dimScale,
      Cogent::Geometry::Axis axis, Cogent::Sense sense) :
 PrimitiveSurface(SurfaceType::Plane),
 m_dimFunc(localParameters[dimIndex]), 
 m_axis(axis), m_sense(sense), m_scale(dimScale), 
 m_dimIndex(dimIndex)
{

  if( m_axis == Axis::X ){ m_vecIndex = 0; }
  else
  if( m_axis == Axis::Y ){ m_vecIndex = 1; }
  else
  if( m_axis == Axis::Z ){ m_vecIndex = 2; }

  if( m_sense == Sense::Positive ){ m_realSense =  1.0; }
  else
  if( m_sense == Sense::Negative ){ m_realSense = -1.0; }


}

void 
Plane::compute(const RealType* X, RealType& val)
{
  val = compute(X);
}

void 
Plane::computeDeriv(const DFadType* X, RealType* vals)
{
  DFadType val;
  val = compute(X);
  int nDerivs = val.size();
  for(int iDeriv=0; iDeriv<nDerivs; iDeriv++)
    vals[iDeriv] = val.dx(iDeriv);
}

bool 
Plane::isEqual(const Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>& compareToSurf)
{
  const Plane* compareToCyl = dynamic_cast<const Plane*>(compareToSurf.get());

  if( compareToCyl ){
    return *this == *compareToCyl;
  } else return false;
}

bool
Plane::operator==(const Plane& rhs)
{
  return std::tie(m_dimFunc,
                  m_axis,
                  m_sense,
                  m_scale
                 ) ==
         std::tie(
              rhs.m_dimFunc,
              rhs.m_axis,
              rhs.m_sense,
              rhs.m_scale
                 );
}



} /* namespace Geometry */
} /* namespace Cogent */
