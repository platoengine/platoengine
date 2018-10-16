//******************************************************************************//
template<typename V, typename P>
void Cogent::BodyIntegrator::getQPoints(
  Simplex<V,P>& simplex, int offset, 
  Cogent::FContainer<P>& weights,
  Cogent::FContainer<P>& points)
//******************************************************************************//
{

  int numNodes = m_simplexBodyBasis->getCardinality();
  int nPoints = simplex.points.size();

  // get simplex vertices, P_Ij, in the reference coords of the parent element
  //
  Cogent::FContainer<P> vertPoints =
  Kokkos::createDynRankView(points,"vertPoints",1,nPoints,m_numDims);
//  Cogent::FContainer<P> vertPoints("vertPoints",1, nPoints, m_numDims);
  for(int i=0; i<nPoints; i++)
    for(uint j=0; j<m_numDims; j++)
      vertPoints(0, i, j) = simplex.points[i](j);

  // get basis values, N_I(\xi_i), at m_bodySimplexPoints, \xi_i.
  //
  int nRefPoints = m_bodySimplexPoints.dimension(0);
  Cogent::FContainer<RealType> Nvals("Nvals",numNodes, nRefPoints);
  m_simplexBodyBasis->getValues(Nvals, m_bodySimplexPoints, Intrepid2::OPERATOR_VALUE);

  // get Jacobian determinant at each refPoint
  //
  int numQPs = m_bodySimplexWeights.dimension(0);
  Cogent::FContainer<P> jacobian = 
    Kokkos::createDynRankView(points,"jacobian",/*numCells=*/1,numQPs,m_numDims,m_numDims);
  Cogent::FContainer<P> jacobian_det = 
    Kokkos::createDynRankView(points,"jacobian_det",/*numCells=*/1,numQPs);
  Cogent::FContainer<P> weighted_measure = 
    Kokkos::createDynRankView(points,"weighted_measure",/*numCells=*/1,numQPs);

  Intrepid2::CellTools<Kokkos::Serial>::setJacobian(jacobian, m_bodySimplexPoints, vertPoints, m_simplexBodyBasis);
  Intrepid2::CellTools<Kokkos::Serial>::setJacobianDet(jacobian_det, jacobian);
  
  // compute weighted measure
  //
  Intrepid2::FunctionSpaceTools<Kokkos::Serial>::computeCellMeasure(weighted_measure, jacobian_det, m_bodySimplexWeights);

  for(int i=0; i<nRefPoints; i++)
    weights(offset+i) = weighted_measure(0,i);
 
  // compute quadrature point locations, N_I(\xi_i) P_Ij, in parent reference coords
  //
  int nDim = m_bodySimplexPoints.dimension(1);
  for(int I=0; I<numNodes; I++)
    for(int i=0; i<nRefPoints; i++)
      for(int j=0; j<nDim; j++)
        points(offset+i,j) += Nvals(I,i)*vertPoints(0,I,j);


  return;
}
