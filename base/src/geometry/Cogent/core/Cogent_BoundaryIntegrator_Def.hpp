//******************************************************************************//
template<typename V, typename P>
void Cogent::BoundaryIntegrator::getQPoints(
  Simplex<V,P>& simplex, int offset, 
  Cogent::FContainer<P>& weights,
  Cogent::FContainer<P>& points,
  const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{

  int numNodes = m_simplexBoundaryBasis->getCardinality();
  int nPoints = simplex.points.size();

  // get simplex vertices, P_Ij, in the reference coords of the parent element
  //
  Cogent::FContainer<P> vertPoints =
    Kokkos::createDynRankView(points,"vertPoints",nPoints, m_numDims);
  for(int i=0; i<nPoints; i++)
    for(uint j=0; j<m_numDims; j++)
      vertPoints(i, j) = simplex.points[i](j);

  // get simplex vertices, P_Ij, in the physical coords of the parent element
  //
  Cogent::FContainer<P> physVertPoints =
    Kokkos::createDynRankView(points,"physVertPoints",nPoints, m_numDims);
  int numBaseNodes = m_baseElementBasis->getCardinality();
  Cogent::FContainer<P> Nphys = 
    Kokkos::createDynRankView(points,"Nphys",numBaseNodes, nPoints);
  m_dicer->getValues(Nphys, vertPoints);
  for(int i=0; i<nPoints; i++)
    for(uint j=0; j<m_numDims; j++){
      physVertPoints(i,j) = 0.0;
      for(int I=0; I<numBaseNodes; I++)
        physVertPoints(i,j) += coordCon(I,j)*Nphys(I,i);
   }

  // get basis values, N_I(\xi_i), at m_boundarySimplexPoints, \xi_i.
  //
  int nRefPoints = m_boundarySimplexPoints.dimension(0);
  Cogent::FContainer<RealType> Nvals("Nvals",numNodes, nRefPoints);
  m_simplexBoundaryBasis->getValues(Nvals, m_boundarySimplexPoints, Intrepid2::OPERATOR_VALUE);

  // compute tangent vectors
  //
  int bDims = m_numDims - 1;
  Cogent::FContainer<RealType> Bvals("Bvals",numNodes, nRefPoints, bDims);
  m_simplexBoundaryBasis->getValues(Bvals, m_boundarySimplexPoints, Intrepid2::OPERATOR_GRAD);
  
  // compute weighting
  //
  if( m_numDims == 3){
    for(int i=0; i<nRefPoints; i++){
      typename Vector3D<P>::Type s(0.0,0.0,0.0), t(0.0,0.0,0.0);
      for(uint j=0; j<m_numDims; j++){
        for(int I=0; I<numNodes; I++){
          s(j) += Bvals(I,i,0)*physVertPoints(I,j);
          t(j) += Bvals(I,i,1)*physVertPoints(I,j);
        }
      }
      typename Vector3D<P>::Type n = minitensor::cross(s,t);
      P norm = minitensor::norm(n);
      weights(offset+i) = norm * m_boundarySimplexWeights(i);
    }
  } else {
  }
 
  // compute quadrature point locations, N_I(\xi_i) P_Ij, in parent reference coords
  //
  for(int I=0; I<numNodes; I++)
    for(int i=0; i<nRefPoints; i++)
      for(uint j=0; j<m_numDims; j++)
        points(offset+i,j) += Nvals(I,i)*vertPoints(I,j);

  return;
}
