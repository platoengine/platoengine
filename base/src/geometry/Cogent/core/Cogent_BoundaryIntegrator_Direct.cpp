#include "Cogent_BoundaryIntegrator_Direct.hpp"
#include "Cogent_WriteUtils.hpp"

//******************************************************************************//
void Cogent::BoundaryIntegrator_Direct::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  m_model->setModelData(geomData);
  getCubature(weights,points,coordCon);
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Direct::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  m_model->setElementData(coordCon);

  if(m_model->isEmptyElement()){
	weights = Cogent::FContainer<RealType>("weights",0);
	points = Cogent::FContainer<RealType>("points",0);
    return;
  } else
  if(!m_model->isPartialElement()){
	weights = Cogent::FContainer<RealType>("weights",0);
	points = Cogent::FContainer<RealType>("points",0);
    return;

  } else {
    std::vector<Simplex<RealType,RealType> > explicitSimplexes;
    getBodySimplexes<RealType,RealType>(explicitSimplexes);

    std::vector<Simplex<RealType,RealType> > boundarySimplexes;
    getBoundarySimplexes(explicitSimplexes, boundarySimplexes, m_surfaceIndex);

    if(m_Verbosity > 2) {
      writeTets(explicitSimplexes);
      writeTris(boundarySimplexes, std::cout);
    }

    getQPoints(boundarySimplexes, weights, points, coordCon);

//    RealType scale = getUniformScaling(coordCon);
//    int nQPs = weights.dimension(0);
//    for(int j=0; j<nQPs; j++)
//      weights(j) *= scale;
  }
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Direct::getCubature(
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
void Cogent::BoundaryIntegrator_Direct::getCubature(
     Cogent::FContainer<RealType>& weights,
     Cogent::FContainer<RealType>& dwdtopo,
     Cogent::FContainer<RealType>& points,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  m_model->setElementData(coordCon);

  if(m_model->isEmptyElement()){
	weights = Cogent::FContainer<RealType>("weights",0);
	dwdtopo = Cogent::FContainer<RealType>("dwdtopo",0);
	points = Cogent::FContainer<RealType>("points",0);
    return;
  } else
  if(!m_model->isPartialElement()){
	weights = Cogent::FContainer<RealType>("weights",0);
	dwdtopo = Cogent::FContainer<RealType>("dwdtopo",0);
	points = Cogent::FContainer<RealType>("points",0);
    return;

  } else {
    std::vector<Simplex<DFadType,DFadType> > explicitSimplexes;
    getBodySimplexes<DFadType,DFadType>(explicitSimplexes);

    std::vector<Simplex<DFadType,DFadType> > boundarySimplexes;
    getBoundarySimplexes(explicitSimplexes, boundarySimplexes, m_surfaceIndex);

    if(m_Verbosity > 2){ 
      writeTris(boundarySimplexes, std::cout);
      writeTets(explicitSimplexes);
    }

    getQPoints(boundarySimplexes, weights, dwdtopo, points, coordCon);

  }
}
//******************************************************************************//
void Cogent::BoundaryIntegrator_Direct::getQPoints(
  std::vector<Simplex<DFadType,DFadType> >& explicitSimplexes,
  Cogent::FContainer<RealType>& weights,
  Cogent::FContainer<RealType>& dwdtopo,
  Cogent::FContainer<RealType>& points,
  const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  uint nSimplexes = explicitSimplexes.size();

  if( nSimplexes == 0 ) return;

  uint offset = 0;
  uint nRefPoints = m_boundarySimplexPoints.dimension(0);
  uint nTopos = explicitSimplexes[0].fieldvals[0].size();
  uint nQPs = nSimplexes*nRefPoints;
  uint nNodes = m_baseElementBasis->getCardinality();
  weights = Cogent::FContainer<RealType>("weights",nQPs);
  points = Cogent::FContainer<RealType>("points",nQPs,m_numDims);
  dwdtopo = Cogent::FContainer<RealType>("dwdtopo",nQPs,nNodes,nTopos);
  typename std::vector<Simplex<DFadType,DFadType> >::iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
     Cogent::BoundaryIntegrator::getQPoints(*it, offset, weights, dwdtopo, points, coordCon);
     offset+=nRefPoints;
  }
}

//******************************************************************************//
void Cogent::BoundaryIntegrator_Direct::getQPoints(
  std::vector<Simplex<RealType,RealType> >& explicitSimplexes,
  Cogent::FContainer<RealType>& weights,
  Cogent::FContainer<RealType>& points,
  const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  uint offset = 0;
  uint nRefPoints = m_boundarySimplexPoints.dimension(0);
  uint nSimplexes = explicitSimplexes.size();
  uint nQPs = nSimplexes*nRefPoints;
  weights = Cogent::FContainer<RealType>("weights",nQPs);
  points = Cogent::FContainer<RealType>("points",nQPs,m_numDims);
  typename std::vector<Simplex<RealType,RealType> >::iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
     Cogent::BoundaryIntegrator::getQPoints(*it, offset, weights, points, coordCon);
     offset+=nRefPoints;
  }
}

