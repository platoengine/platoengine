#include "Cogent_BodyIntegrator_Direct.hpp"
#include "Cogent_WriteUtils.hpp"

//******************************************************************************//
void Cogent::BodyIntegrator_Direct::getBodySimplexes(
  const FContainer<RealType>& geomData, 
  const FContainer<RealType>& coordCon,
  std::vector<Simplex<RealType,RealType>>& tets)
//******************************************************************************//
{
  m_model->setModelData(geomData);
  m_model->setElementData(coordCon);

  if(m_model->isEmptyElement()){
    return;
  } else
  if(!m_model->isPartialElement()){
    return;

  } else {
    std::vector<Simplex<RealType,RealType> > explicitSimplexes;
    BodyIntegrator::getBodySimplexes<RealType,RealType>(explicitSimplexes);
 
    if(explicitSimplexes.size() == 0) return;

    int nNodes = m_baseElementBasis->getCardinality();
    int nPoints = explicitSimplexes[0].points.size();
    Cogent::FContainer<RealType> Nvals("Nvals", nNodes, nPoints);
    Cogent::FContainer<RealType> Points("Points", nPoints, m_numDims);

    for( auto& tet : explicitSimplexes ){
 
      for(int iPoint=0; iPoint<nPoints; iPoint++)
        for(uint iDim=0; iDim<m_numDims; iDim++)
          Points(iPoint,iDim) = tet.points[iPoint](iDim);

      m_baseElementBasis->getValues(Nvals, Points, Intrepid2::OPERATOR_VALUE);
 
      for(int iPoint=0; iPoint<nPoints; iPoint++){
        for(uint iDim=0; iDim<m_numDims; iDim++)
          tet.points[iPoint](iDim) = 0.0;
        for(int iNode=0; iNode<nNodes; iNode++)
          for(uint iDim=0; iDim<m_numDims; iDim++)
            tet.points[iPoint](iDim) += Nvals(iNode,iPoint)*coordCon(iNode,iDim);
      }

      tets.push_back(tet);
    }
  }
  return;
}
  
//******************************************************************************//
void Cogent::BodyIntegrator_Direct::getBoundarySimplexes(
  const FContainer<RealType>& geomData, 
  const FContainer<RealType>& coordCon,
  std::vector<Simplex<RealType,RealType>>& tets)
//******************************************************************************//
{
}
  

//******************************************************************************//
void Cogent::BodyIntegrator_Direct::getQPoints(
  std::vector<Simplex<RealType,RealType> >& explicitSimplexes,
  Cogent::FContainer<RealType>& weights,
  Cogent::FContainer<RealType>& points)
//******************************************************************************//
{
  uint offset = 0;
  uint nRefPoints = m_bodySimplexPoints.dimension(0);
  uint nSimplexes = explicitSimplexes.size();
  uint nQPs = nSimplexes*nRefPoints;
  weights = Cogent::FContainer<RealType>("weights",nQPs);
  points = Cogent::FContainer<RealType>("points",nQPs,m_numDims);
  typename std::vector<Simplex<RealType,RealType> >::iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
     Cogent::BodyIntegrator::getQPoints(*it, offset, weights, points);
     offset+=nRefPoints;
  }
}
//******************************************************************************//
void Cogent::BodyIntegrator_Direct::getQPoints(
  std::vector<Simplex<DFadType,DFadType> >& explicitSimplexes,
  Cogent::FContainer<RealType>& weights,
  Cogent::FContainer<RealType>& dwdtopo,
  Cogent::FContainer<RealType>& points)
//******************************************************************************//
{

  uint nSimplexes = explicitSimplexes.size();

  if( nSimplexes == 0 ) return;

  uint nRefPoints = m_bodySimplexPoints.dimension(0);
  uint nTopos = explicitSimplexes[0].fieldvals[0].size();
  uint nQPs = nSimplexes*nRefPoints;
  uint nNodes = m_baseElementBasis->getCardinality();

  // conformal tets are computed.  create weights and points.
  uint offset = 0;
  weights = Cogent::FContainer<RealType>("weights",nQPs);
  dwdtopo = Cogent::FContainer<RealType>("dwdtopo",nQPs,nNodes*nTopos);
  points = Cogent::FContainer<RealType>("points",nQPs,m_numDims);
  typename std::vector<Simplex<DFadType,DFadType> >::iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
     getQPoints(*it, offset, weights, dwdtopo, points);
     offset+=nRefPoints;
  }
}

//******************************************************************************//
void Cogent::BodyIntegrator_Direct::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{

  m_model->setModelData(geomData);
  getCubature(weights, points, coordCon);
}

//******************************************************************************//
void Cogent::BodyIntegrator_Direct::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  m_model->setElementData(coordCon);

  if(m_model->isEmptyElement()){
	weights = Cogent::FContainer<RealType>("weights",0);
    return;
  } else
  if(!m_model->isPartialElement()){
    m_baseElementCubature->getCubature(points, weights);
    return;

  } else {
    std::vector<Simplex<RealType,RealType> > explicitSimplexes;
    BodyIntegrator::getBodySimplexes<RealType,RealType>(explicitSimplexes);

    if(m_Verbosity > 2) writeTets(explicitSimplexes);

    getQPoints(explicitSimplexes, weights, points);

    RealType scale = getUniformScaling(coordCon);
    int nQPs = weights.dimension(0);
    for(int j=0; j<nQPs; j++)
      weights(j) *= scale;
  }
  return;
}

//******************************************************************************//
void Cogent::BodyIntegrator_Direct::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& dwdtopo,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  m_model->setModelData(geomData);
  getCubature(weights,dwdtopo,points,coordCon);
}

//******************************************************************************//
void Cogent::BodyIntegrator_Direct::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& dwdtopo,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  m_model->setElementData(coordCon);

  if(m_model->isEmptyElement()){
	dwdtopo = Cogent::FContainer<RealType>("dwdtopo",0);
    return;
  } else
  if(!m_model->isPartialElement()){
    return;

  } else {
    std::vector<Simplex<DFadType,DFadType> > explicitSimplexes;
    BodyIntegrator::getBodySimplexes<DFadType,DFadType>(explicitSimplexes);

    getQPoints(explicitSimplexes, weights, dwdtopo, points);
  }
}


//******************************************************************************//
void Cogent::BodyIntegrator_Direct::getQPoints(
  Simplex<DFadType,DFadType>& simplex, int offset, 
  FContainer<RealType>& weights,
  FContainer<RealType>& dwdtopo,
  FContainer<RealType>& points)
//******************************************************************************//
{
  int numNodes = m_simplexBodyBasis->getCardinality();
  int nPoints = simplex.points.size();

  // get simplex vertices, P_Ij, in the reference coords of the parent element
  //
  int nDerivs = dwdtopo.dimension(1)+1;
  Cogent::FContainer<DFadType> vertPoints("vertPoints",1, nPoints, m_numDims, nDerivs);
  for(int i=0; i<nPoints; i++)
    for(uint j=0; j<m_numDims; j++)
      vertPoints(0, i, j) = simplex.points[i](j);

  // get Jacobian determinant at each refPoint
  //
  int numQPs = m_bodySimplexWeights.dimension(0);
  Cogent::FContainer<DFadType> jacobian("jacobian",/*numCells=*/1,numQPs,m_numDims,m_numDims, nDerivs);
  Cogent::FContainer<DFadType> jacobian_det("jacobian_det",/*numCells=*/1,numQPs, nDerivs);
  Cogent::FContainer<DFadType> weighted_measure("weighted_measure",/*numCells=*/1,numQPs, nDerivs);

  Intrepid2::CellTools<Kokkos::Serial>::setJacobian(jacobian, m_bodySimplexPoints, vertPoints, m_simplexBodyBasis);
  Intrepid2::CellTools<Kokkos::Serial>::setJacobianDet(jacobian_det, jacobian);
  
  // compute weighted measure
  //
  Intrepid2::FunctionSpaceTools<Kokkos::Serial>::computeCellMeasure(weighted_measure, jacobian_det, m_bodySimplexWeights);

  int nRefPoints = m_bodySimplexPoints.dimension(0);
  for(int i=0; i<nRefPoints; i++){
    weights(offset+i) = weighted_measure(0,i).val();
    int nTopo = weighted_measure(0,i).size();
    for(int j=0; j<nTopo; j++)
      dwdtopo(offset+i,j) = weighted_measure(0,i).dx(j);
  }

  // get basis values, N_I(\xi_i), at m_bodySimplexPoints, \xi_i.
  //
  Cogent::FContainer<RealType> Nvals("Nvals",numNodes, nRefPoints);
  m_simplexBodyBasis->getValues(Nvals, m_bodySimplexPoints, Intrepid2::OPERATOR_VALUE);
 
  // compute quadrature point locations, N_I(\xi_i) P_Ij, in parent reference coords
  //
  int nDim = m_bodySimplexPoints.dimension(1);
  for(int I=0; I<numNodes; I++)
    for(int i=0; i<nRefPoints; i++)
      for(int j=0; j<nDim; j++)
        points(offset+i,j) += Nvals(I,i)*vertPoints(0,I,j).val();

  return;
}
