#include "Cogent_BodyIntegrator_Projected.hpp"

//******************************************************************************//
void Cogent::BodyIntegrator_Projected::getStandardPoints(
     Cogent::FContainer<RealType>& points)
//******************************************************************************//
{
#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif

  int numQPs = m_standardPoints.dimension(0);
  int numDims = m_standardPoints.dimension(1);
  points = Cogent::FContainer<RealType>("points",numQPs,numDims);
  for(int i=0; i<numQPs; ++i)
    for(int j=0; j<numDims; ++j)
      points(i,j) = m_standardPoints(i,j);
}
//******************************************************************************//
void Cogent::BodyIntegrator_Projected::getCubature(
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
void Cogent::BodyIntegrator_Projected::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  getStandardPoints(points);
  getCubatureWeights(weights,coordCon);
}
//******************************************************************************//
void Cogent::BodyIntegrator_Projected::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& dwdgeom,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  getStandardPoints(points);
  getCubatureWeights(weights,dwdgeom,geomData,coordCon);
}

//******************************************************************************//
void Cogent::BodyIntegrator_Projected::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& dwdgeom,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  getStandardPoints(points);
  getCubatureWeights(weights,dwdgeom,coordCon);
}

//******************************************************************************//
void Cogent::BodyIntegrator_Projected::getCubatureWeights(
     Cogent::FContainer<RealType>& weights,
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  m_model->setModelData(geomData);
  getCubatureWeights(weights,coordCon);

}

//******************************************************************************//
void Cogent::BodyIntegrator_Projected::getCubatureWeights(
     Cogent::FContainer<RealType>& weights,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif

  m_model->setElementData(coordCon);

  if(m_model->isEmptyElement()){
    for(int I=0; I<m_systemSize; I++)
      weights(I) = 0.0;
  } else
  if(!m_model->isPartialElement()){
    for(int I=0; I<m_systemSize; I++){
      weights(I) = m_standardWeights(I);
    }
  } else {
    std::vector<Simplex<RealType,RealType> > explicitSimplexes;
    getBodySimplexes<RealType,RealType>(explicitSimplexes);
    getQPointWeights(explicitSimplexes, weights);
    if(m_Verbosity > 2) writeTets(explicitSimplexes,"case");
  }


  RealType scale = getUniformScaling(coordCon);
  for(int j=0; j<m_systemSize; j++)
    weights(j) *= scale;

  if(m_ErrorChecking > 2){
    int nqps = weights.dimension(0);
    RealType totalWeight = 0.0;
    for(int i=0; i<nqps; i++)
      totalWeight += weights(i);
    std::cout << "Total weight (body): " << totalWeight << std::endl;
  }
}

//******************************************************************************//
void Cogent::BodyIntegrator_Projected::getCubatureWeights(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& dwdgeom,
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  m_model->setModelData(geomData);
  getCubatureWeights(weights,dwdgeom,coordCon);
}

//******************************************************************************//
void Cogent::BodyIntegrator_Projected::getCubatureWeights(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& dwdgeom,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif

  m_model->setElementData(coordCon);

  if(m_model->isEmptyElement()){
    for(int I=0; I<m_systemSize; I++)
      weights(I) = 0.0;
    dwdgeom = Cogent::FContainer<RealType>("dwdgeom",0);
    return;
  } else
  if(!m_model->isPartialElement()){
    for(int I=0; I<m_systemSize; I++){
      weights(I) = m_standardWeights(I);
    }
    return;

  } else {
    std::vector<Simplex<DFadType,DFadType> > explicitSimplexes;
    getBodySimplexes<DFadType,DFadType>(explicitSimplexes);
    getQPointWeights(explicitSimplexes, coordCon, weights, dwdgeom);
  }

  RealType scale = getUniformScaling(coordCon);
  int shapeDim = dwdgeom.dimension(1);
  for(int i=0; i<m_systemSize; i++){
    weights(i) *= scale;
    for(int j=0; j<shapeDim; j++){
      dwdgeom(i,j) *= scale;
    }
  }

}

//******************************************************************************//
void Cogent::BodyIntegrator_Projected::getQPointWeights(
  std::vector<Simplex<DFadType,DFadType> >& explicitSimplexes,
  const Cogent::FContainer<RealType>& coordCon,
  Cogent::FContainer<RealType>& weights,
  Cogent::FContainer<RealType>& dwdG)
//******************************************************************************//
{

#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif

  uint nSimplexes = explicitSimplexes.size();

  if( nSimplexes == 0 ) return;

  uint nRefPoints = m_bodySimplexPoints.dimension(0);
  uint nTopos = explicitSimplexes[0].fieldvals[0].size();
  uint nQPs = nSimplexes*nRefPoints;
  uint nNodes = m_baseElementBasis->getCardinality();

  uint offset = 0;
  uint nDerivs = nTopos*nNodes;
  Cogent::FContainer<DFadType> cweights("cweights",nQPs,nDerivs+1);
  Cogent::FContainer<DFadType> cpoints("cpoints",nQPs,m_numDims,nDerivs+1);
  typename std::vector<Simplex<DFadType,DFadType> >::iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
     Cogent::BodyIntegrator::getQPoints(*it, offset, cweights, cpoints);
     offset+=nRefPoints;
  }

  Teuchos::SerialDenseMatrix<int,DFadType> B(m_systemSize,1);
  for(int I=0; I<m_systemSize; I++){
    B(I, 0) = 0.0;
    for(uint K=0; K<nQPs; K++)
      B(I, 0) += cweights(K)*mono<DFadType>(cpoints(K,0),cpoints(K,1),cpoints(K,2),
                                  m_monomial(I,0),m_monomial(I,1),m_monomial(I,2));
  }

  weights = Cogent::FContainer<RealType>("weights",m_systemSize);
  Cogent::FContainer<RealType>
    dwdT = Cogent::FContainer<RealType>("dwdT",m_systemSize,nNodes,nTopos);
  Kokkos::deep_copy(dwdT,0.0);
  for(int I=0; I<m_systemSize; I++){
    weights(I)=0.0;
    for(int J=0; J<m_systemSize; J++){
      weights(I) += m_A(I,J)*B(J,0).val();
      for(uint K=0; K<nNodes; K++)
        for(uint L=0; L<nTopos; L++)
          dwdT(I,K,L) += m_A(I,J)*B(J,0).dx(K*nTopos+L);
    }
  }

  m_model->mapToGeometryData(dwdT,dwdG); 
  
}

//******************************************************************************//
void Cogent::BodyIntegrator_Projected::getQPointWeights(
  std::vector<Simplex<RealType,RealType> >& explicitSimplexes,
  Cogent::FContainer<RealType>& weights)
//******************************************************************************//
{
#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif

  uint offset = 0;
  uint nRefPoints = m_bodySimplexPoints.dimension(0);
  uint nSimplexes = explicitSimplexes.size();
  uint nQPs = nSimplexes*nRefPoints;
  Cogent::FContainer<RealType> cweights("cweights",nQPs);
  Cogent::FContainer<RealType> cpoints("cpoints",nQPs,m_numDims);
  typename std::vector<Simplex<RealType,RealType> >::iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
     Cogent::BodyIntegrator::getQPoints(*it, offset, cweights, cpoints);
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

