#include <Shards_CellTopology.hpp>
#include <MiniTensor.h>
#include <Shards_CellTopologyData.h>
#include <Intrepid2_FunctionSpaceTools.hpp>
#include <Intrepid2_DefaultCubatureFactory.hpp>
#include <map>
#include <Intrepid2_HGRAD_TET_C1_FEM.hpp>
#include <Intrepid2_HGRAD_HEX_C1_FEM.hpp>


//******************************************************************************//
template<typename V, typename P>
void Cogent::Integrator::Refine( 
  std::vector<Simplex<V,P> >& insimplexes,
  std::vector<Simplex<V,P> >& outsimplexes)
//******************************************************************************//
{

  if( insimplexes.size() == 0 ) return;

   
  const int nVerts = insimplexes[0].points.size();

  if( m_numDims == 3 ){
    int nsimplexes = insimplexes.size();
    for(int isimplex=0; isimplex<nsimplexes; isimplex++){
      std::vector<typename Vector3D<P>::Type>& pnts = insimplexes[isimplex].points;
 
      typename Vector3D<P>::Type bodyCenter(pnts[0]);
      for(int i=1; i<nVerts; i++) bodyCenter += pnts[i];
      bodyCenter /= nVerts;
 
      Simplex<V,P> tet(nVerts);
      tet.points[0] = pnts[0];
      tet.points[1] = (pnts[0]+pnts[1])/2.0;
      tet.points[2] = (pnts[0]+pnts[2])/2.0;
      tet.points[3] = (pnts[0]+pnts[3])/2.0;
      outsimplexes.push_back(tet);
 
      tet.points[0] = pnts[1];
      tet.points[1] = (pnts[1]+pnts[2])/2.0;
      tet.points[2] = (pnts[1]+pnts[0])/2.0;
      tet.points[3] = (pnts[1]+pnts[3])/2.0;
      outsimplexes.push_back(tet);
 
      tet.points[0] = pnts[2];
      tet.points[1] = (pnts[2]+pnts[0])/2.0;
      tet.points[2] = (pnts[2]+pnts[1])/2.0;
      tet.points[3] = (pnts[2]+pnts[3])/2.0;
      outsimplexes.push_back(tet);
 
      tet.points[0] = pnts[3];
      tet.points[1] = (pnts[3]+pnts[2])/2.0;
      tet.points[2] = (pnts[3]+pnts[1])/2.0;
      tet.points[3] = (pnts[3]+pnts[0])/2.0;
      outsimplexes.push_back(tet);
 
      tet.points[0] = (pnts[0]+pnts[1])/2.0;
      tet.points[1] = (pnts[1]+pnts[2])/2.0;
      tet.points[2] = (pnts[2]+pnts[0])/2.0;
      tet.points[3] = bodyCenter;
      outsimplexes.push_back(tet);
 
      tet.points[0] = (pnts[1]+pnts[2])/2.0;
      tet.points[1] = (pnts[1]+pnts[3])/2.0;
      tet.points[2] = (pnts[3]+pnts[2])/2.0;
      tet.points[3] = bodyCenter;
      outsimplexes.push_back(tet);
 
      tet.points[0] = (pnts[2]+pnts[3])/2.0;
      tet.points[1] = (pnts[3]+pnts[0])/2.0;
      tet.points[2] = (pnts[0]+pnts[2])/2.0;
      tet.points[3] = bodyCenter;
      outsimplexes.push_back(tet);
 
      tet.points[0] = (pnts[0]+pnts[1])/2.0;
      tet.points[1] = (pnts[3]+pnts[0])/2.0;
      tet.points[2] = (pnts[1]+pnts[3])/2.0;
      tet.points[3] = bodyCenter;
      outsimplexes.push_back(tet);
 
      tet.points[0] = (pnts[0]+pnts[1])/2.0;
      tet.points[1] = (pnts[0]+pnts[2])/2.0;
      tet.points[2] = (pnts[0]+pnts[3])/2.0;
      tet.points[3] = bodyCenter;
      outsimplexes.push_back(tet);
 
      tet.points[0] = (pnts[1]+pnts[2])/2.0;
      tet.points[1] = (pnts[1]+pnts[0])/2.0;
      tet.points[2] = (pnts[1]+pnts[3])/2.0;
      tet.points[3] = bodyCenter;
      outsimplexes.push_back(tet);
 
      tet.points[0] = (pnts[2]+pnts[0])/2.0;
      tet.points[1] = (pnts[2]+pnts[1])/2.0;
      tet.points[2] = (pnts[2]+pnts[3])/2.0;
      tet.points[3] = bodyCenter;
      outsimplexes.push_back(tet);
 
      tet.points[0] = (pnts[3]+pnts[2])/2.0;
      tet.points[1] = (pnts[3]+pnts[1])/2.0;
      tet.points[2] = (pnts[3]+pnts[0])/2.0;
      tet.points[3] = bodyCenter;
      outsimplexes.push_back(tet);
 
    }
  } else 
  if( m_numDims == 2 ){
    int nsimplexes = insimplexes.size();
    for(int isimplex=0; isimplex<nsimplexes; isimplex++){
      std::vector<typename Vector3D<P>::Type>& pnts = insimplexes[isimplex].points;
 
      Simplex<V,P> tri(nVerts);
      tri.points[0] = pnts[0];
      tri.points[1] = (pnts[0]+pnts[1])/2.0;
      tri.points[2] = (pnts[0]+pnts[2])/2.0;
      outsimplexes.push_back(tri);
 
      tri.points[0] = pnts[1];
      tri.points[1] = (pnts[1]+pnts[2])/2.0;
      tri.points[2] = (pnts[1]+pnts[0])/2.0;
      outsimplexes.push_back(tri);
 
      tri.points[0] = pnts[2];
      tri.points[1] = (pnts[2]+pnts[0])/2.0;
      tri.points[2] = (pnts[2]+pnts[1])/2.0;
      outsimplexes.push_back(tri);
 
      tri.points[0] = (pnts[0]+pnts[1])/2.0;
      tri.points[1] = (pnts[1]+pnts[2])/2.0;
      tri.points[2] = (pnts[2]+pnts[0])/2.0;
      outsimplexes.push_back(tri);
 
    }
  }
}
 
//******************************************************************************//
template<typename V, typename P>
V Cogent::Integrator::BoundaryMeasure(Simplex<V,P>& simplex)
//******************************************************************************//
{
#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif

  P zero(0.0);

  std::vector<typename Vector3D<P>::Type>& pnts = simplex.points;

  if(pnts.size() == 3){
    P x2 = pnts[2](0),    y2 = pnts[2](1),    z2 = pnts[2](2);
    P x0 = pnts[0](0)-x2, y0 = pnts[0](1)-y2, z0 = pnts[0](2)-z2;
    P x1 = pnts[1](0)-x2, y1 = pnts[1](1)-y2, z1 = pnts[1](2)-z2;
    P cx = y0*z1-y1*z0, cy = x1*z0-x0*z1, cz = x0*y1-x1*y0;
    P n2 = cx*cx+cy*cy+cz*cz;
    if( n2 > zero ) return sqrt(n2)/2.0;
    else return zero;
  } else
  if(pnts.size() == 2){
    P x0 = pnts[0](0), y0 = pnts[0](1);
    P x1 = pnts[1](0), y1 = pnts[1](1);
    P dx = x1-x0, dy = y1-y0;
    P n2 = dx*dx+dy*dy;
    if( n2 > zero ) return sqrt(n2)/2.0;
    else return zero;
  }
  return 0.0;
}

//******************************************************************************//
template<typename V, typename P>
V Cogent::Integrator::Measure(Simplex<V,P>& simplex)
//******************************************************************************//
{

#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif

// JR:  this is needlessly expensive
  int nPoints = simplex.points.size();
  Cogent::FContainer<P> vertPoints("vertPoints",1,nPoints, m_numDims);

  for(int i=0; i<nPoints; i++)
    for(uint j=0; j<m_numDims; j++)
      vertPoints(0, i, j) = simplex.points[i](j);

  int numQPs = m_bodySimplexWeights.dimension(0);
  Cogent::FContainer<P> jacobian("jacobian",/*numCells=*/1,numQPs,m_numDims,m_numDims);
  Cogent::FContainer<P> jacobian_det("jacobian_det",/*numCells=*/1,numQPs);
  Cogent::FContainer<P> weighted_measure("weighted_measure",/*numCells=*/1,numQPs);

  Intrepid2::CellTools<Kokkos::Serial>::setJacobian(jacobian, m_bodySimplexPoints, vertPoints, m_simplexBodyBasis);
  Intrepid2::CellTools<Kokkos::Serial>::setJacobianDet(jacobian_det, jacobian);

  Intrepid2::FunctionSpaceTools<Kokkos::Serial>::computeCellMeasure(weighted_measure, jacobian_det, m_bodySimplexWeights);

  P measure = 0.0;
  for(int i=0; i<numQPs; i++){
    measure += weighted_measure(0,i);
  }
  return measure;
}


//******************************************************************************//
template <typename V, typename P>
void Cogent::Integrator::getBoundarySimplexes(
   std::vector<Simplex<V,P> >& explicitSimplexes,
   std::vector<Simplex<V,P> >& boundarySimplexes, int iTopo)
//******************************************************************************//
{
  if(explicitSimplexes.size() == 0) return;

  int nVerts = explicitSimplexes[0].points.size();
  int nBV = nVerts-1;

  typename std::vector<Simplex<V,P> >::iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
    if(it->boundaryLS[0] == iTopo){
      Simplex<V,P> b(nBV);
      for(int ib=0; ib<nBV; ib++){
        b.points[ib] = it->points[ib];
        b.fieldvals[ib] = it->fieldvals[ib];
      }
      b.bodyLS = it->boundaryLS[0];
      boundarySimplexes.push_back(b);
    }   
  }
}
