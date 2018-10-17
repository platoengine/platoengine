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

  m_cutStencils = makeStencils();

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

Cogent::SimplexStencil::
SimplexStencil(std::vector<int> cutPts,
               std::vector<std::vector<int>> newPts,
               std::vector<std::vector<int>> subTets,
               std::vector<Cogent::Sense> sense) : m_cutPts(cutPts), m_newPts(newPts), m_subTets(subTets), m_sense(sense)
{
  m_topoMap.resize(m_subTets.size());
 
  // create vertex index lists of the faces of the background tet
  //
  std::vector<std::vector<int>> faceSets({{0,1,2},{1,2,3},{2,3,0},{3,0,1}});

  // add intersection points
  //
  int newPointIndex = Cogent::nTetPts;
  for( const auto& pt : m_newPts ){
    for( auto& faceSet : faceSets ){
      if( std::count(faceSet.begin(), faceSet.end(), pt[0]) &&
          std::count(faceSet.begin(), faceSet.end(), pt[1]) ) faceSet.push_back(newPointIndex);
    }
    newPointIndex++;
  }

  // add intersection plane to list of face index sets
  //
  faceSets.push_back(m_cutPts);
  int interiorFace = faceSets.size();

  // create topoMap for each sub-tet.  
  //
  int nsubs = m_subTets.size();
  int nfaces = faceSets.size();
  for(int iSubTet=0; iSubTet<nsubs; iSubTet++){
    m_topoMap[iSubTet].resize(nfaces);
    const auto& subTet = subTets[iSubTet];
    for(int iFace=0; iFace<Cogent::nTetPts; iFace++){
      int i0 = subTet[ iFace ];
      int i1 = subTet[(iFace+1)%Cogent::nTetPts];
      int i2 = subTet[(iFace+2)%Cogent::nTetPts];
      bool foundFace = false;
      for(int jFace=0; jFace<nfaces; jFace++){
        auto begin = faceSets[jFace].begin();
        auto end = faceSets[jFace].end();
        if( std::count( begin, end, i0) &&
            std::count( begin, end, i1) &&
            std::count( begin, end, i2) ) {
          m_topoMap[iSubTet][iFace] = jFace;
          foundFace = true; break;
        }
      }
      if( !foundFace ) m_topoMap[iSubTet][iFace] = interiorFace;
    }
  }

}


std::vector<Cogent::SimplexStencil> 
Cogent::makeStencils()
{
  using Cogent::Sense;
  using Cogent::SimplexStencil;

  std::vector<SimplexStencil> cutStencils = {
  /* - - - - */ /* 0 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* - - - 0 */ /* 1 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* - - - + */ /* 2 */
  SimplexStencil({4,5,6},{{3,0},{3,1},{3,2}},
                  {{3,4,6,5},{6,4,2,5},{5,4,2,1},{2,4,0,1}},
                  {Sense::Positive,Sense::Negative,Sense::Negative,Sense::Negative}),

  /* - - 0 - */ /* 3 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* - - 0 0 */ /* 4 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* - - 0 + */ /* 5 */
  SimplexStencil({2,4,5},{{0,3},{1,3}},{{4,5,2,3},{4,1,2,5},{2,0,1,4}},{Sense::Positive,Sense::Negative,Sense::Negative}),


  /* - - + - */ /* 6 */
  SimplexStencil({4,5,6},{{2,0},{2,1},{2,3}},
                  {{2,4,5,6},{6,0,4,5},{3,6,5,0},{3,5,1,0}},
                  {Sense::Positive,Sense::Negative,Sense::Negative,Sense::Negative}),

  /* - - + 0 */ /* 7 */
  SimplexStencil({3,4,5},{{0,2},{1,2}},{{3,5,4,2},{3,4,5,1},{3,4,1,0}},{Sense::Positive,Sense::Negative,Sense::Negative}),

  /* - - + + */ /* 8 */
  SimplexStencil({4,5,6,7},{{0,2},{0,3},{1,2},{1,3}},
                  {{6,0,1,7},{4,6,5,0},{5,6,7,0},{3,5,2,7},{6,5,7,2},{4,5,6,2}},
                  {Sense::Negative,Sense::Negative,Sense::Negative,Sense::Positive,Sense::Positive,Sense::Positive}),



  /* - 0 - - */ /* 9 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* - 0 - 0 */ /* 10 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* - 0 - + */ /* 11 */
  SimplexStencil({1,4,5},{{0,3},{2,3}},{{3,4,5,1},{5,4,2,1},{4,0,2,1}},{Sense::Positive,Sense::Negative,Sense::Negative}),



  /* - 0 0 - */ /* 12 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* - 0 0 0 */ /* 13 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* - 0 0 + */ /* 14 */
  SimplexStencil({1,2,4},{{0,3}},{{0,1,2,4},{2,4,1,3}},{Sense::Negative,Sense::Positive}),


  /* - 0 + - */ /* 15 */
  SimplexStencil({1,4,5},{{0,2},{2,3}},{{5,1,4,2},{5,4,1,0},{5,0,1,3}},{Sense::Positive,Sense::Negative,Sense::Negative}),

  /* - 0 + 0 */ /* 16 */
  SimplexStencil({1,3,4},{{0,2}},{{3,4,1,0},{3,1,4,2}},{Sense::Negative,Sense::Positive}),

  /* - 0 + + */ /* 17 */
  SimplexStencil({1,4,5},{{0,2},{0,3}},{{5,4,1,0},{5,2,1,4},{3,2,1,5}},{Sense::Negative,Sense::Positive,Sense::Positive}),


  /* - + - - */ /* 18 */
  SimplexStencil({4,5,6},{{1,0},{1,2},{1,3}},
                  {{5,4,1,6},{2,4,5,6},{2,4,6,3},{0,4,2,3}},
                  {Sense::Positive,Sense::Negative,Sense::Negative,Sense::Negative}),

  /* - + - 0 */ /* 19 */
  SimplexStencil({3,4,5},{{0,1},{1,2}},{{3,4,5,1},{3,4,2,5},{3,0,2,4}},{Sense::Positive,Sense::Negative,Sense::Negative}),

  /* - + - + */ /* 20 */
  SimplexStencil({4,5,6,7},{{0,1},{0,3},{1,2},{2,3}},
                  {{2,0,4,5},{2,5,4,6},{2,7,5,6},{6,4,1,3},{6,5,4,3},{6,7,5,3}},
                  {Sense::Negative,Sense::Negative,Sense::Negative,Sense::Positive,Sense::Positive,Sense::Positive}),


  /* - + 0 - */ /* 21 */
  SimplexStencil({2,4,5},{{0,1},{1,3}},{{2,4,1,5},{2,0,4,5},{3,0,2,5}},{Sense::Positive,Sense::Negative,Sense::Negative}),

  /* - + 0 0 */ /* 22 */
  SimplexStencil({2,3,4},{{0,1}},{{3,0,2,4},{3,4,2,1}},{Sense::Negative,Sense::Positive}),

  /* - + 0 + */ /* 23 */
  SimplexStencil({2,4,5},{{0,1},{0,2}},{{5,0,2,4},{5,4,2,1},{3,5,2,1}},{Sense::Negative,Sense::Positive,Sense::Negative}),



  /* - + + - */ /* 24 */
  SimplexStencil({4,5,6,7},{{0,1},{0,2},{1,3},{2,3}},
                  {{1,2,4,6},{7,5,2,4},{7,2,6,4},{3,0,7,4},{7,5,4,0},{7,4,6,3}},
                  {Sense::Positive,Sense::Positive,Sense::Positive,Sense::Negative,Sense::Negative,Sense::Negative}),

  /* - + + 0 */ /* 25 */
  SimplexStencil({3,4,5},{{0,1},{0,2}},{{5,0,4,3},{5,4,1,3},{2,5,1,3}},{Sense::Negative,Sense::Positive,Sense::Positive}),

  /* - + + + */ /* 26 */
  SimplexStencil({4,5,6},{{0,1},{0,2},{0,3}},
                  {{0,4,5,6},{5,4,2,6},{6,4,2,3},{2,4,1,3}},
                  {Sense::Negative,Sense::Positive,Sense::Positive,Sense::Positive}),


  /* 0 - - - */ /* 27 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* 0 - - 0 */ /* 28 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* 0 - - + */ /* 29 */
  SimplexStencil({0,4,5},{{1,3},{2,3}},{{3,5,4,0},{2,4,5,0},{4,2,1,0}},{Sense::Positive,Sense::Negative,Sense::Negative}),


  /* 0 - 0 - */ /* 30 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* 0 - 0 0 */ /* 31 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* 0 - 0 + */ /* 32 */
  SimplexStencil({0,2,4},{{1,3}},{{3,2,4,0},{4,2,1,0}},{Sense::Positive,Sense::Negative}),


  /* 0 - + - */ /* 33 */
  SimplexStencil({0,4,5},{{1,2},{2,3}},{{5,2,4,0},{3,5,4,0},{3,4,1,0}},{Sense::Positive,Sense::Negative,Sense::Negative}),

  /* 0 - + 0 */ /* 34 */
  SimplexStencil({0,3,4},{{1,2}},{{3,0,2,4},{3,0,4,1}},{Sense::Positive,Sense::Negative}),

  /* 0 - + + */ /* 35 */
  SimplexStencil({0,4,5},{{1,2},{1,3}},{{3,2,5,0},{5,2,4,0},{5,4,1,0}},{Sense::Positive,Sense::Positive,Sense::Negative}),


  /* 0 0 - - */ /* 36 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* 0 0 - 0 */ /* 37 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* 0 0 - + */ /* 38 */
  SimplexStencil({0,1,4},{{2,3}},{{4,2,1,0},{3,4,1,0}},{Sense::Negative,Sense::Positive}),


  /* 0 0 0 - */ /* 39 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Negative}),

  /* 0 0 0 0 */ /* 40 */
  SimplexStencil({},{},{},{}),

  /* 0 0 0 + */ /* 41 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),


  /* 0 0 + - */ /* 42 */
  SimplexStencil({0,1,4},{{2,3}},{{4,2,1,0},{3,4,1,0}},{Sense::Positive,Sense::Negative}),

  /* 0 0 + 0 */ /* 43 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),

  /* 0 0 + + */ /* 44 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),




  /* 0 + - - */ /* 45 */
  SimplexStencil({0,4,5},{{1,2},{1,3}},{{5,4,1,0},{3,4,5,0},{3,2,4,0}},{Sense::Positive,Sense::Negative,Sense::Negative}),

  /* 0 + - 0 */ /* 46 */
  SimplexStencil({0,3,4},{{1,2}},{{3,2,4,0},{3,4,1,0}},{Sense::Negative,Sense::Positive}),

  /* 0 + - + */ /* 47 */
  SimplexStencil({0,4,5},{{1,2},{2,3}},{{5,2,4,0},{5,4,1,0},{3,5,1,0}},{Sense::Negative,Sense::Positive,Sense::Positive}),

  /* 0 + 0 - */ /* 48 */
  SimplexStencil({0,2,4},{{1,3}},{{3,2,4,0},{4,2,1,0}},{Sense::Negative,Sense::Positive}),

  /* 0 + 0 0 */ /* 49 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),

  /* 0 + 0 + */ /* 50 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),

  /* 0 + + - */ /* 51 */
  SimplexStencil({0,4,5},{{1,3},{2,3}},{{3,5,4,0},{5,2,4,0},{4,2,1,0}},{Sense::Negative,Sense::Positive,Sense::Positive}),

  /* 0 + + 0 */ /* 52 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),

  /* 0 + + + */ /* 53 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),








  /* + - - - */ /* 54 */
  SimplexStencil({4,5,6},{{0,1},{0,2},{0,3}},
                  {{6,0,5,4},{6,5,2,4},{2,4,1,6},{2,1,3,6}},
                  {Sense::Positive,Sense::Negative,Sense::Negative,Sense::Negative}),

  /* + - - 0 */ /* 55 */
  SimplexStencil({3,4,5},{{0,1},{0,2}},{{5,0,4,3},{5,4,1,3},{5,1,2,3}},{Sense::Positive,Sense::Negative,Sense::Negative}),

  /* + - - + */ /* 56 */
  SimplexStencil({4,5,6,7},{{0,1},{0,2},{1,3},{2,3}},
                 {{2,4,1,6},{6,4,5,2},{7,6,5,2},{5,0,4,3},{6,5,4,3},{7,5,6,3}},
                 {Sense::Negative,Sense::Negative,Sense::Negative,Sense::Positive,Sense::Positive,Sense::Positive}),


  /* + - 0 - */ /* 57 */
  SimplexStencil({2,4,5},{{0,1},{0,3}},{{5,0,2,4},{1,4,5,2},{1,5,3,2}},{Sense::Positive,Sense::Negative,Sense::Negative}),

  /* + - 0 0 */ /* 58 */
  SimplexStencil({2,3,4},{{0,1}},{{2,0,4,3},{2,4,1,3}},{Sense::Positive,Sense::Negative}),

  /* + - 0 + */ /* 59 */
  SimplexStencil({2,4,5},{{0,1},{1,3}},{{2,0,4,3},{2,4,5,3},{2,4,1,5}},{Sense::Positive,Sense::Positive,Sense::Negative}),


  /* + - + - */ /* 60 */
  SimplexStencil({4,5,6,7},{{0,1},{0,3},{1,2},{2,3}},
                 {{2,0,4,5},{2,4,6,7},{2,5,4,7},{3,4,6,1},{7,5,4,3},{7,4,6,3}},
                 {Sense::Positive,Sense::Positive,Sense::Positive,Sense::Negative,Sense::Negative,Sense::Negative}),

  /* + - + 0 */ /* 61 */
  SimplexStencil({3,4,5},{{0,1},{1,2}},{{5,4,1,3},{5,0,4,3},{2,0,5,3}},{Sense::Negative,Sense::Positive,Sense::Positive}),

  /* + - + + */ /* 62 */
  SimplexStencil({4,5,6},{{0,1},{1,2},{1,3}},
                  {{5,4,1,6},{2,4,5,6},{2,0,4,6},{3,0,2,6}},
                  {Sense::Negative,Sense::Positive,Sense::Positive,Sense::Positive}),


  /* + 0 - - */ /* 63 */
  SimplexStencil({1,4,5},{{0,2},{0,3}},{{5,0,4,1},{5,4,2,1},{3,5,2,1}},{Sense::Positive,Sense::Negative,Sense::Negative}),

  /* + 0 - 0 */ /* 64 */
  SimplexStencil({1,3,4},{{0,2}},{{4,0,1,3},{2,4,1,3}},{Sense::Positive,Sense::Negative}),

  /* + 0 - + */ /* 65 */
  SimplexStencil({1,4,5},{{0,2},{2,3}},{{5,4,2,1},{5,0,4,1},{3,0,5,1}},{Sense::Negative,Sense::Positive,Sense::Positive}),


  /* + 0 0 - */ /* 66 */
  SimplexStencil({1,2,4},{{0,3}},{{4,0,2,1},{3,4,2,1}},{Sense::Positive,Sense::Negative}),

  /* + 0 0 0 */ /* 67 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),

  /* + 0 0 + */ /* 68 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),


  /* + 0 + - */ /* 69 */
  SimplexStencil({1,4,5},{{0,3},{2,3}},{{3,4,5,1},{5,4,2,1},{4,0,2,1}},{Sense::Negative,Sense::Positive,Sense::Positive}),

  /* + 0 + 0 */ /* 70 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),

  /* + 0 + + */ /* 71 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),

  /* + + - - */ /* 72 */
  SimplexStencil({4,5,6,7},{{0,2},{0,3},{1,2},{1,3}},
                 {{3,5,2,7},{7,2,6,5},{2,4,6,5},{5,0,4,1},{5,4,6,1},{7,5,6,1}},
                 {Sense::Negative,Sense::Negative,Sense::Negative,Sense::Positive,Sense::Positive,Sense::Positive}),

  /* + + - 0 */ /* 73 */
  SimplexStencil({3,4,5},{{0,2},{1,2}},{{3,4,2,5},{3,0,4,5},{3,0,5,1}},{Sense::Negative,Sense::Positive,Sense::Positive}),

  /* + + - + */ /* 74 */
  SimplexStencil({4,5,6},{{0,2},{1,2},{2,3}},
                  {{6,4,2,5},{6,0,4,5},{3,0,6,5},{5,0,1,3}},
                  {Sense::Negative,Sense::Positive,Sense::Positive,Sense::Positive}),

  /* + + 0 - */ /* 75 */
  SimplexStencil({2,4,5},{{0,3},{1,3}},{{3,4,2,5},{4,0,2,5},{5,0,2,1}},{Sense::Negative,Sense::Positive,Sense::Positive}),

  /* + + 0 0 */ /* 76 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),

  /* + + 0 + */ /* 77 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),

  /* + + + - */ /* 78 */
  SimplexStencil({4,5,6},{{0,3},{1,3},{2,3}},
                  {{3,4,6,5},{6,4,2,5},{4,0,2,1},{5,4,2,1}},
                  {Sense::Negative,Sense::Positive,Sense::Positive,Sense::Positive}),

  /* + + + 0 */ /* 79 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),

  /* + + + + */ /* 80 */
  SimplexStencil({},{},{{0,1,2,3}},{Sense::Positive}),

  };
  return cutStencils;
}
