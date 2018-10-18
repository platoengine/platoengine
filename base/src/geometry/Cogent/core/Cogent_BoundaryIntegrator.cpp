#include "Cogent_BoundaryIntegrator.hpp"
#include "Cogent_WriteUtils.hpp"

//******************************************************************************//
void 
Cogent::BoundaryIntegrator::getMeasure(
   RealType& measure, 
   Cogent::FContainer<RealType>& dMdtopo,
   const Cogent::FContainer<RealType>& geomData,
   const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  measure = 0.0;
 
  m_model->setModelData(geomData);

  m_model->setElementData(coordCon);

  if(m_model->isEmptyElement()){
    measure = 0.0;
    return;
  } else
  if(!m_model->isPartialElement()){
    measure = 0.0;
    return;

  } else {
    std::vector<Simplex<DFadType,DFadType> > explicitSimplexes;
    getBodySimplexes<DFadType,DFadType>(explicitSimplexes);

    std::vector<Simplex<DFadType,DFadType> > boundarySimplexes;
    getBoundarySimplexes(explicitSimplexes, boundarySimplexes, m_model->getSurfaceIndex());

    if(m_Verbosity > 2){ 
      writeTris(boundarySimplexes, std::cout);
      writeTets(explicitSimplexes);
    }

    DFadType newMeasure(0.0);
    typename std::vector<Simplex<DFadType,DFadType> >::iterator it;
    for(it=boundarySimplexes.begin(); it!=boundarySimplexes.end(); it++){
       newMeasure += BoundaryMeasure(*it);
    }

/// TODO: (FIX)  specific to non-parameterize model
    if(newMeasure.size()) {
      uint nNodes = geomData.dimension(0);
      uint nTopos = geomData.dimension(1);
      dMdtopo = Cogent::FContainer<RealType>("dMdtopo",nNodes,nTopos);
      for (uint i=0;i<nNodes;i++)
        for (uint j=0;j<nTopos;j++)
          dMdtopo(i,j)=newMeasure.dx(i*nTopos+j);
    }
  }
}

//******************************************************************************//
void 
Cogent::BoundaryIntegrator::getMeasure(
   RealType& measure, 
   const Cogent::FContainer<RealType>& geomData,
   const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  measure = 0.0;

  m_model->setModelData(geomData);

  if(m_model->isEmptyElement()){
    measure = 0.0;
    return;
  } else
  if(!m_model->isPartialElement()){
    measure = 0.0;
    return;
  } else
  {
    std::vector<Simplex<RealType,RealType> > explicitSimplexes;
    getBodySimplexes<RealType,RealType>(explicitSimplexes);

    std::vector<Simplex<RealType,RealType> > boundarySimplexes;
    getBoundarySimplexes(explicitSimplexes, boundarySimplexes, m_model->getSurfaceIndex());

    if(m_Verbosity > 2){ 
      writeTris(boundarySimplexes, std::cout);
      writeTets(explicitSimplexes);
    }

    typename std::vector<Simplex<RealType,RealType> >::iterator it;
    for(it=boundarySimplexes.begin(); it!=boundarySimplexes.end(); it++){
       measure += BoundaryMeasure(*it);
    }
    
  }
}

//******************************************************************************//
Cogent::BoundaryIntegrator::BoundaryIntegrator(
   Teuchos::RCP<shards::CellTopology> _celltype,
   Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > _basis,
   const Teuchos::ParameterList& geomSpec):
   Integrator(_celltype, _basis, geomSpec)
//******************************************************************************//
{
//  m_surfaceIndex = geomSpec.get<int>("Surface Level Set Index");
}

//******************************************************************************//
void Cogent::BoundaryIntegrator::getQPoints(
  Simplex<DFadType,DFadType>& simplex, int offset, 
  Cogent::FContainer<RealType>& weights,
  Cogent::FContainer<RealType>& dwdtopo,
  Cogent::FContainer<RealType>& points,
  const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{

  int numNodes = m_simplexBoundaryBasis->getCardinality();
  int nPoints = simplex.points.size();
  int nTopos = simplex.fieldvals[0].size();

  // get simplex vertices, P_Ij, in the reference coords of the parent element
  //
  uint nDerivs = dwdtopo.dimension(1)+1;
  Cogent::FContainer<DFadType> vertPoints("vertPoints", nPoints, m_numDims, nDerivs);
  for(int i=0; i<nPoints; i++)
    for(uint j=0; j<m_numDims; j++)
      vertPoints(i, j) = simplex.points[i](j);

  // get simplex vertices, P_Ij, in the physical coords of the parent element
  //
  Cogent::FContainer<DFadType> physVertPoints("physVertPoints", nPoints, m_numDims, nDerivs);
  int numBaseNodes = m_baseElementBasis->getCardinality();
  Cogent::FContainer<DFadType> Nphys("Nphys", numBaseNodes, nPoints, nDerivs);
  m_dicer->getValues(Nphys, vertPoints);
  for(int i=0; i<nPoints; i++)
    for(uint j=0; j<m_numDims; j++){
      physVertPoints(i,j) = 0.0;
      for(int I=0; I<numBaseNodes; I++)
        physVertPoints(i,j) += Nphys(I,i)*coordCon(I,j);
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
    for(int iQP=0; iQP<nRefPoints; iQP++){
      typename Vector3D<DFadType>::Type s(0.0,0.0,0.0), t(0.0,0.0,0.0);
      for(uint iDim=0; iDim<m_numDims; iDim++){
        for(int I=0; I<numNodes; I++){
          s(iDim) += physVertPoints(I,iDim)*Bvals(I,iQP,0);
          t(iDim) += physVertPoints(I,iDim)*Bvals(I,iQP,1);
        }
      }
      typename Vector3D<DFadType>::Type n = minitensor::cross(s,t);
      DFadType norm = minitensor::norm(n);
      weights(offset+iQP) = norm.val() * m_boundarySimplexWeights(iQP);

      for (int iNode=0;iNode<numNodes;iNode++)
        for (int iTopo=0;iTopo<nTopos;iTopo++)
          dwdtopo(offset+iQP,iNode,iTopo)=norm.dx(iNode*nTopos+iTopo) * m_boundarySimplexWeights(iQP);
    }
  } else {
  }
 
  // compute quadrature point locations, N_I(\xi_i) P_Ij, in parent reference coords
  //
  for(int I=0; I<numNodes; I++)
    for(int iQP=0; iQP<nRefPoints; iQP++)
      for(uint iDim=0; iDim<m_numDims; iDim++)
        points(offset+iQP,iDim) += Nvals(I,iQP)*vertPoints(I,iDim).val();

  return;
}
