#include "Cogent_Dicer.hpp"

#include <Intrepid2_HGRAD_TET_C1_FEM.hpp>
#include <Intrepid2_HGRAD_TRI_C1_FEM.hpp>
#include <Intrepid2_HGRAD_HEX_C1_FEM.hpp>
#include <Intrepid2_HGRAD_QUAD_C1_FEM.hpp>
#include <Intrepid2_HGRAD_QUAD_C2_FEM.hpp>

namespace Cogent {
//******************************************************************************//
template <>
void Dicer::SortMap<RealType>(
       const std::vector<typename Vector3D<RealType>::Type >& points, 
       std::vector<int>& map)
//******************************************************************************//
{
  uint nPoints = points.size();

  map.resize(0);
  
  if( nPoints <= 2 ) return;

  // find centerpoint
  typename Vector3D<RealType>::Type center(points[0]);
  for(uint i=1; i<nPoints; i++) center += points[i];
  center /= nPoints;
     
  // sort by counterclockwise angle about surface normal
  RealType pi = acos(-1.0);
  typename Vector3D<RealType>::Type X(points[0]-center);
  RealType xnorm = minitensor::norm(X);
  if(xnorm==0) return;
  X /= xnorm;
  bool foundNormal = false;
  typename Vector3D<RealType>::Type Y, Z;
  for(uint i=1; i<nPoints; i++){
    typename Vector3D<RealType>::Type X1(points[i]-center);
    Z = minitensor::cross(X, X1);
    RealType znorm = minitensor::norm(Z);
    if( znorm == 0 ) continue;
    foundNormal = true;
    Z /= znorm;
    Y = minitensor::cross(Z, X);
    break;
  }

  if( !foundNormal )
    return;

  std::map<RealType, uint> angles;
  angles.insert( std::pair<RealType, uint>(0.0,0) );
  for(uint i=1; i<nPoints; i++){
    typename Vector3D<RealType>::Type comp = points[i] - center;
    RealType compnorm = minitensor::norm(comp);
    if(compnorm==0) return;
    comp /= compnorm;
    RealType prod = X*comp;
    RealType fabs_prod = fabs(prod);
    if( fabs_prod > 1.0 ) prod = prod/fabs_prod;
    RealType angle = acos(prod);
//    RealType angle = acos((float)prod);
    if( Y * comp < 0.0 ) angle = 2.0*pi - angle;
    angles.insert( std::pair<RealType, uint>(angle,i) );
  }

  map.resize(nPoints);
  typename std::map<RealType, uint>::iterator ait;
  std::vector<int>::iterator mit;
  for(mit=map.begin(),ait=angles.begin(); ait!=angles.end(); mit++, ait++){
    *mit = ait->second;
  }
}



//******************************************************************************//
template <>
void Dicer::SortMap<DFadType>(const std::vector<typename Vector3D<DFadType>::Type >& points, 
                           std::vector<int>& map)
//******************************************************************************//
{
  uint nPoints = points.size();
  
  std::vector<typename Vector3D<RealType>::Type > rpoints(nPoints);
  for(uint i=0; i<nPoints; i++){
    rpoints[i](0) = points[i](0).val();
    rpoints[i](1) = points[i](1).val();
    rpoints[i](2) = points[i](2).val();
  }

  SortMap<RealType>(rpoints, map);

}
//******************************************************************************//
template <>
bool Dicer::areColinear<RealType>(
 const std::vector<typename Vector3D<RealType>::Type >& points)
//******************************************************************************//
{
// Description:
// 

  uint nPoints = points.size();
  
  if( nPoints <= 2 ) return true;

  // find centerpoint
  typename Vector3D<RealType>::Type center(points[0]);
  for(uint i=1; i<nPoints; i++) center += points[i];
  center /= nPoints;
     
  typename Vector3D<RealType>::Type X(points[0]-center);
  RealType xnorm = minitensor::norm(X);
  if(xnorm == 0) return true;
  X /= xnorm;
  for(uint i=1; i<nPoints; i++){
    typename Vector3D<RealType>::Type X1(points[i]-center);
    typename Vector3D<RealType>::Type Z = minitensor::cross(X, X1);
    RealType znorm = minitensor::norm(Z);
    if(znorm != 0) return false;
  }

  return true;
}

//******************************************************************************//
template <>
bool Dicer::areColinear<DFadType>(
       const std::vector<typename Vector3D<DFadType>::Type >& points)
//******************************************************************************//
{
// Description:
// 

  uint nPoints = points.size();
  
  std::vector<typename Vector3D<RealType>::Type > rpoints(nPoints);
  for(uint i=0; i<nPoints; i++){
    rpoints[i](0) = points[i](0).val();
    rpoints[i](1) = points[i](1).val();
    rpoints[i](2) = points[i](2).val();
  }

  return areColinear<RealType>(rpoints);
}

//******************************************************************************//
template<>
void Dicer::getValues<>( Cogent::FContainer<RealType>& Nvals,
                         const Cogent::FContainer<RealType>& evalPoints)
//******************************************************************************//
{ m_basis->getValues(Nvals, evalPoints, Intrepid2::OPERATOR_VALUE); }


//******************************************************************************//
template<>
void Dicer::getValues<>( Cogent::FContainer<DFadType>& Nvals,
                         const Cogent::FContainer<DFadType>& evalPoints)
//******************************************************************************//
{ m_DFadBasis->getValues(Nvals, evalPoints, Intrepid2::OPERATOR_VALUE); }


std::string strint(std::string s, int i, char delim) {
    std::ostringstream ss;
    ss << s << delim << i;
    return ss.str();
}

//******************************************************************************//
Cogent::Dicer::Dicer(
   Teuchos::RCP<const shards::CellTopology> _cellTopology,
   Teuchos::RCP<const Intrepid2::Basis<Kokkos::Serial, RealType, RealType> > _basis,
   RealType _interfaceValue):
   m_basis(_basis),
   m_interfaceValue(_interfaceValue)
//******************************************************************************//
{
  m_numDims = _cellTopology->getDimension();

  // *** tetrahedral element ***/
  if( _cellTopology->getBaseName() == shards::getCellTopologyData< shards::Tetrahedron<4> >()->name ){

    m_DFadBasis = Teuchos::rcp(
      new Intrepid2::Basis_HGRAD_TET_C1_FEM<Kokkos::Serial, DFadType, DFadType>() );

  } else

  
  // *** hexahedral element ***/
  if( _cellTopology->getBaseName() == shards::getCellTopologyData< shards::Hexahedron<8> >()->name ){

    m_DFadBasis = Teuchos::rcp(
     new Intrepid2::Basis_HGRAD_HEX_C1_FEM<Kokkos::Serial, DFadType, DFadType>() );

  } else

  // *** quadrilateral element **/
  if( _cellTopology->getBaseName() == shards::getCellTopologyData< shards::Quadrilateral<4> >()->name ){

    if( _cellTopology->getName() == shards::getCellTopologyData< shards::Quadrilateral<4> >()->name ){
      m_DFadBasis = Teuchos::rcp(
       new Intrepid2::Basis_HGRAD_QUAD_C1_FEM<Kokkos::Serial, DFadType, DFadType>() );
    } else 
    if( _cellTopology->getName() == shards::getCellTopologyData< shards::Quadrilateral<8> >()->name 
     || _cellTopology->getName() == shards::getCellTopologyData< shards::Quadrilateral<9> >()->name ){
      m_DFadBasis = Teuchos::rcp(
       new Intrepid2::Basis_HGRAD_QUAD_C2_FEM<Kokkos::Serial, DFadType, DFadType>() );
    }

  } else

  // *** triangle element **/
  if( _cellTopology->getBaseName() == shards::getCellTopologyData< shards::Triangle<3> >()->name ){

    m_DFadBasis = Teuchos::rcp(
     new Intrepid2::Basis_HGRAD_TRI_C1_FEM<Kokkos::Serial, DFadType, DFadType>() );

  }

}

} /** end namespace Cogent */
