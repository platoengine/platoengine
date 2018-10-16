#include "Cogent_BoundaryIntegrator_Projected.hpp"
#include "Cogent_WriteUtils.hpp"

//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getStandardPoints(
     Cogent::FContainer<RealType>& points)
//******************************************************************************//
{
  int numQPs = m_standardPoints.dimension(0);
  int numDims = m_standardPoints.dimension(1);
  points = Cogent::FContainer<RealType>("points",numQPs,numDims);
  for(int i=0; i<numQPs; ++i)
    for(int j=0; j<numDims; ++j)
      points(i,j) = m_standardPoints(i,j);
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  getStandardPoints(points);
  getCubatureWeights(weights,coordCon);
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  getStandardPoints(points);
  getCubatureWeights(weights,geomData,coordCon);
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& dwdtopo,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  getStandardPoints(points);
  getCubatureWeights(weights,dwdtopo,coordCon);
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& dwdtopo,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  getStandardPoints(points);
  getCubatureWeights(weights,dwdtopo,geomData,coordCon);
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getCubatureWeights(
     Cogent::FContainer<RealType>& weights,
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  m_model->setModelData(geomData);
  getCubatureWeights(weights,coordCon);
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getCubatureWeights(
     Cogent::FContainer<RealType>& weights,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
// JR TODO: This function is performing the decomposition, then checking
// to see if the relevant boundary intersects the element.  Check first, 
// so the decomposition isn't done without need.
  m_model->setElementData(coordCon);

  if(!m_model->isSurfaceElement()){
    for(int I=0; I<m_systemSize; I++)
      weights(I) = 0.0;
  } else {
    std::vector<Simplex<RealType,RealType> > explicitSimplexes;
    getBodySimplexes<RealType,RealType>(explicitSimplexes);

    std::vector<Simplex<RealType,RealType> > boundarySimplexes;
    getBoundarySimplexes(explicitSimplexes, boundarySimplexes, m_model->getSurfaceIndex());

    if(m_Verbosity > 2){
      writeTets(explicitSimplexes);
      writeTris(boundarySimplexes);
    }

    getQPointWeights(coordCon, boundarySimplexes, weights);
  }

  if(m_ErrorChecking > 2){
    int nqps = weights.dimension(0);
    RealType totalWeight = 0.0;
    for(int i=0; i<nqps; i++)
      totalWeight += weights(i);
    std::cout << "Total weight (boundary): " << totalWeight << std::endl;
  }
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getCubatureWeights(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& dwdtopo,
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  m_model->setModelData(geomData);
  getCubatureWeights(weights,dwdtopo,coordCon);
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getCubatureWeights(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& dwdtopo,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  m_model->setElementData(coordCon);

  if(!m_model->isSurfaceElement()){
    for(int I=0; I<m_systemSize; I++)
      weights(I) = 0.0;
    dwdtopo = Cogent::FContainer<RealType>("dwdtopo",0);
    return;
  } else {
    std::vector<Simplex<DFadType,DFadType> > explicitSimplexes;
    getBodySimplexes<DFadType,DFadType>(explicitSimplexes);

    std::vector<Simplex<DFadType,DFadType> > boundarySimplexes;
    getBoundarySimplexes(explicitSimplexes, boundarySimplexes, m_model->getSurfaceIndex());

    if(m_Verbosity > 2){ 
      writeTris(boundarySimplexes);
      writeTets(explicitSimplexes);
    }

    getQPointWeights(coordCon, boundarySimplexes, weights, dwdtopo);

  }
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getQPointWeights(
  const Cogent::FContainer<RealType>& coordCon,
  std::vector<Simplex<DFadType,DFadType> >& explicitSimplexes,
  Cogent::FContainer<RealType>& weights,
  Cogent::FContainer<RealType>& dwdtopo)
//******************************************************************************//
{
  uint nNodes = m_baseElementBasis->getCardinality();
  uint nTopos = explicitSimplexes[0].fieldvals[0].size();
  uint offset = 0;
  uint nRefPoints = m_boundarySimplexPoints.dimension(0);
  uint nSimplexes = explicitSimplexes.size();
  uint nQPs = nSimplexes*nRefPoints;
  uint nDerivs = nTopos*nNodes;
  Cogent::FContainer<DFadType> cweights("cweights",nQPs,nDerivs+1);
  Cogent::FContainer<DFadType> cpoints("cpoints",nQPs,m_numDims,nDerivs+1);
  typename std::vector<Simplex<DFadType,DFadType> >::iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
     getQPoints(*it, offset, cweights, cpoints, coordCon);
     offset+=nRefPoints;
  }

  Teuchos::SerialDenseMatrix<int,DFadType> B(m_systemSize,1);
  for(int I=0; I<m_systemSize; I++){
    B(I, 0) = 0.0;
    for(uint K=0; K<nQPs; K++)
      B(I, 0) += cweights(K)*mono<DFadType>(cpoints(K,0),cpoints(K,1),cpoints(K,2),
                                  m_monomial(I,0),m_monomial(I,1),m_monomial(I,2));
  }

  int surfaceIndex = m_model->getSurfaceIndex();
  weights = Cogent::FContainer<RealType>("weights",m_systemSize);
  dwdtopo = Cogent::FContainer<RealType>("dwdtopo",m_systemSize,nNodes);
  Kokkos::deep_copy(weights,0.0);
  Kokkos::deep_copy(dwdtopo,0.0);
  for(int I=0; I<m_systemSize; I++){
    for(int J=0; J<m_systemSize; J++){
      weights(I) += m_A(I,J)*B(J,0).val();
      for(uint K=0; K<nNodes; K++)
        dwdtopo(I,K) += m_A(I,J)*B(J,0).dx(K*nTopos+surfaceIndex);
    }
  }
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getQPointWeights(
  const Cogent::FContainer<RealType>& coordCon,
  std::vector<Simplex<RealType,RealType> >& explicitSimplexes,
  Cogent::FContainer<RealType>& weights)
//******************************************************************************//
{

  uint offset = 0;
  uint nRefPoints = m_boundarySimplexPoints.dimension(0);
  uint nSimplexes = explicitSimplexes.size();
  uint nQPs = nSimplexes*nRefPoints;
  Cogent::FContainer<RealType> cweights("cweights",nQPs);
  Cogent::FContainer<RealType> cpoints("cpoints",nQPs,m_numDims);
  typename std::vector<Simplex<RealType,RealType> >::iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
     getQPoints(*it, offset, cweights, cpoints, coordCon);
     offset+=nRefPoints;
  }
  Teuchos::SerialDenseMatrix<int,RealType> B(m_systemSize,1);
  for(int I=0; I<m_systemSize; I++){
    B(I, 0) = 0.0;
    for(uint K=0; K<nQPs; K++)
      B(I, 0) += cweights(K)*mono<RealType>(cpoints(K,0),cpoints(K,1),cpoints(K,2),
                                  m_monomial(I,0),m_monomial(I,1),m_monomial(I,2));
  }
  weights = Cogent::FContainer<RealType>("weights",m_systemSize);
  Kokkos::deep_copy(weights,0.0);
  for(int I=0; I<m_systemSize; I++){
    for(int J=0; J<m_systemSize; J++)
      weights(I) += m_A(I,J)*B(J,0);
  }
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Projected::getQPointWeights(
  const Cogent::FContainer<RealType>& coordCon,
  std::vector<Simplex<DFadType,DFadType> >& explicitSimplexes,
  Cogent::FContainer<RealType>& weights)
//******************************************************************************//
{

  uint offset = 0;
  uint nRefPoints = m_boundarySimplexPoints.dimension(0);
  uint nSimplexes = explicitSimplexes.size();
  uint nQPs = nSimplexes*nRefPoints;
  uint nTopos = explicitSimplexes[0].fieldvals[0].size();
  uint nNodes = m_baseElementBasis->getCardinality();
  uint nDerivs = nTopos*nNodes+1;
  Cogent::FContainer<DFadType> cweights("cweights",nQPs,nDerivs);
  Cogent::FContainer<DFadType> cpoints("cpoints",nQPs,m_numDims,nDerivs);
  typename std::vector<Simplex<DFadType,DFadType> >::iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
     getQPoints(*it, offset, cweights, cpoints, coordCon);
     offset+=nRefPoints;
  }

  Teuchos::SerialDenseMatrix<int,DFadType> B(m_systemSize,1);
  for(int I=0; I<m_systemSize; I++){
    B(I, 0) = 0.0;
    for(uint K=0; K<nQPs; K++)
      B(I, 0) += cweights(K)*mono<DFadType>(cpoints(K,0),cpoints(K,1),cpoints(K,2),
                                  m_monomial(I,0),m_monomial(I,1),m_monomial(I,2));
  }

  int surfaceIndex = m_model->getSurfaceIndex();
  weights = Cogent::FContainer<RealType>("weights",m_systemSize);
  Kokkos::deep_copy(weights,0.0);
  for(int I=0; I<m_systemSize; I++){
    for(int J=0; J<m_systemSize; J++)
      for(uint K=0; K<nNodes; K++)
        weights(I) += m_A(I,J)*B(J,0).dx(K*nTopos+surfaceIndex);
  }
}
