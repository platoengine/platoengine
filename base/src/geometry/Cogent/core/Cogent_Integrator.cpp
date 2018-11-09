#include "Cogent_Integrator.hpp"
#include "Cogent_ModelFactory.hpp"
#include "Cogent_WriteUtils.hpp"

#ifdef ENABLE_BOOST_TIMER
#include <boost/timer/timer.hpp>
#endif



// JR To do:
// 3.  In Simplex<V,P>, is V always the same as P?  If so, ...
// 4.  getBodyCubature must map to physical coords
// 5.  Create test for projected body cubature
// 6.  Create test for projected boundary cubature
// 7.  2D.
// 8.  2D and 3D should be separate Dicer classes.

namespace Cogent {

#ifdef ENABLE_BOOST_TIMER
int Timer::indent = 0;
#endif

//******************************************************************************//
void toDFadType(const FContainer<RealType>& from, 
                                  FContainer<DFadType>& to)
//******************************************************************************//
{
// TODO:  make general.  This only works for 2D array. need 1D as well.
  uint nNodes = from.dimension(0);
  uint nTopos = from.dimension(1);
  uint nDerivs = nNodes*nTopos;
  Cogent::FContainer<RealType> Tval("Tval",nNodes,nTopos);
  to = Cogent::FContainer<DFadType>("Tfad",nNodes,nTopos,nDerivs+1);
  for(uint i=0; i<nNodes; i++)
    for(uint j=0; j<nTopos; j++){
      Tval(i,j) = Sacado::ScalarValue<RealType>::eval(from(i,j));
      to(i,j)   = DFadType(nTopos*nNodes, i*nTopos+j, Tval(i,j));
    }
}

//******************************************************************************//
template <>
DFadType Integrator::getBodySimplexes(
   std::vector<Simplex<DFadType,DFadType> >& explicitSimplexes)
//******************************************************************************//
{
 
#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif

  DFadType measure = 0.0;
  DFadType measureChange = 1e6;
  DFadType tolerance(m_maxError);


  Cogent::FContainer<RealType> topoData;
  m_model->getSignedDistance(topoData);
  
  Cogent::FContainer<DFadType> Tfad;
  toDFadType(topoData, Tfad);

  uint level = 0;
  while (1){
    std::vector<Simplex<DFadType,DFadType> >& implicitSimplexes = m_DFadRefinement[level];

    std::vector<Simplex<DFadType,DFadType>> allSimplexes;
    m_dicer->Cut( Tfad, implicitSimplexes, allSimplexes );

    // condense explicitSimplexes to only include subdomains with material.
    //
    explicitSimplexes.clear();
    typename std::vector<Simplex<DFadType,DFadType> >::iterator it;
    for(it=allSimplexes.begin(); it!=allSimplexes.end(); it++){
      if(m_model->isMaterial(it->sense))
        explicitSimplexes.push_back(*it);
    }

    DFadType newMeasure = 0.0;
    for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
       newMeasure += m_dicer->Volume(*it);
    }

    measureChange = fabs(measure - newMeasure);
    measure = newMeasure.val();
  
    level++;

    if(measureChange < tolerance || level >= m_maxLevels) break;

    if( level >= m_DFadRefinement.size() ){
      m_DFadRefinement.resize(level+1);
      Refine(m_DFadRefinement[level-1], m_DFadRefinement[level]);
    }
  }
  return measure;
}

//******************************************************************************//
template <>
RealType Integrator::getBodySimplexes(
   std::vector<Simplex<RealType,RealType> >& explicitSimplexes)
//******************************************************************************//
{

#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif

  RealType measure = 0.0;
  RealType measureChange = 1e6;

  Cogent::FContainer<RealType> topoData;
  m_model->getSignedDistance(topoData);

  uint level = 0;
  while (1){
    std::vector<Simplex<RealType,RealType> >& implicitSimplexes = m_refinement[level];

    std::vector<Simplex<RealType,RealType>> allSimplexes;
    m_dicer->Cut( topoData, implicitSimplexes, allSimplexes);
    
    // condense explicitSimplexes to only include subdomains with material.
    //
    explicitSimplexes.clear();
    typename std::vector<Simplex<RealType,RealType> >::iterator it;
    for(it=allSimplexes.begin(); it!=allSimplexes.end(); it++){
      if(m_model->isMaterial(it->sense))
        explicitSimplexes.push_back(*it);
    }

    RealType newMeasure = 0.0;
    for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
       newMeasure += m_dicer->Volume(*it);
    }

    measureChange = fabs(measure - newMeasure);
    measure = newMeasure;
  
    level++;

    if(measureChange < m_maxError || level >= m_maxLevels) break;

    if( level >= m_refinement.size() ){
      m_refinement.resize(level+1);
      Refine(m_refinement[level-1], m_refinement[level]);
    }
  }
  return measure;
}


//******************************************************************************//
RealType Cogent::Integrator::getUniformScaling(const FContainer<RealType>& coordCon)
//******************************************************************************//
{
  int numNodes = m_baseElementBasis->getCardinality();

  FContainer<RealType> Bvals("Bvals",numNodes, m_centerPoint.dimension(0), m_numDims);
  m_baseElementBasis->getValues(Bvals, m_centerPoint, Intrepid2::OPERATOR_GRAD);

  LocalMatrix<RealType> b(m_numDims,m_numDims);

  // compute scaling
  for(uint i=0; i<m_numDims; i++){
    for(uint j=0; j<m_numDims; j++){
      b(i,j) = 0.0;
      for(int I=0; I<numNodes; I++)
        b(i,j) += Bvals(I,0,j)*coordCon(I,i);
    }
  }
  RealType scale = 0.0;
  if(m_numDims == 3){
    scale += b(0,0)*b(1,1)*b(2,2);
    scale += b(0,1)*b(1,2)*b(2,0);
    scale += b(0,2)*b(1,0)*b(2,1);
    scale -= b(0,0)*b(1,2)*b(2,1);
    scale -= b(0,1)*b(1,0)*b(2,2);
    scale -= b(0,2)*b(1,1)*b(2,0);
  } else 
  if(m_numDims == 2){
    scale += b(0,0)*b(1,1);
    scale -= b(0,1)*b(1,0);
  }

  return scale;
}


//******************************************************************************//
void Cogent::Integrator::getMeasure(
     RealType& measure, 
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{

#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif

  measure = 0.0;

  m_model->setModelData(geomData);

  m_model->setElementData(coordCon);

  if(m_model->isEmptyElement()){
    measure = 0.0;
    return;
  } else
  if(!m_model->isPartialElement()){
    std::vector<Simplex<RealType,RealType> >& implicitSimplexes = m_refinement[0];
    typename std::vector<Simplex<RealType,RealType> >::iterator it;
    for(it=implicitSimplexes.begin(); it!=implicitSimplexes.end(); it++){
//       measure += Measure(*it);
       measure += m_dicer->Volume(*it);
    }
    return;
  } else // element is partially full, ...
  {
    std::vector<Simplex<RealType,RealType> > explicitSimplexes;
    measure = getBodySimplexes(explicitSimplexes);

    if(m_Verbosity > 2){ 
      writeTets(explicitSimplexes, "case");
    }
  }
}

//******************************************************************************//
void Cogent::Integrator::getMeasure(
     RealType& measure, 
     Cogent::FContainer<RealType>& dMdgeom,
     const Cogent::FContainer<RealType>& geomData,
     const Cogent::FContainer<RealType>& coordCon)
//******************************************************************************//
{
  
  measure = 0.0;
  DFadType measureChange = 1e6;
  DFadType tolerance(m_maxError);

  m_model->setModelData(geomData);

  m_model->setElementData(coordCon);

  if(m_model->isEmptyElement()){
    measure = 0.0;
    dMdgeom = Cogent::FContainer<RealType>("dMdgeom",0);
    return;
  } else
  if(!m_model->isPartialElement()){
    // element is full, use standard integration ...
    return;

  } else {
    // element is partially full, ...

    Cogent::FContainer<RealType> topoData;
    m_model->getSignedDistance(topoData);

    Cogent::FContainer<DFadType> Tfad;
    toDFadType(topoData, Tfad);
    
    uint level = 0;
    std::vector<Simplex<DFadType,DFadType> > explicitSimplexes;
    while (1){
      std::vector<Simplex<DFadType,DFadType> >& implicitSimplexes = m_DFadRefinement[level];

      std::vector<Simplex<DFadType,DFadType>> allSimplexes;
      m_dicer->Cut( Tfad, implicitSimplexes, allSimplexes );

      // condense explicitSimplexes to only include subdomains with material.
      //
      explicitSimplexes.clear();
      typename std::vector<Simplex<DFadType,DFadType> >::iterator it;
      for(it=allSimplexes.begin(); it!=allSimplexes.end(); it++){
        if(m_model->isMaterial(it->sense))
          explicitSimplexes.push_back(*it);
      }

      DFadType newMeasure = 0.0;
      for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); it++){
         newMeasure += m_dicer->Volume(*it);
      }

   
      measureChange = fabs(measure - newMeasure);
      measure = newMeasure.val();

      if(newMeasure.size()) {
        uint nNodes = topoData.dimension(0);
        uint nTopos = topoData.dimension(1);
        Cogent::FContainer<RealType> 
          dMdtopo = Cogent::FContainer<RealType>("dMdtopo",nNodes,nTopos);
        for (uint i=0;i<nNodes;i++)
          for (uint j=0;j<nTopos;j++)
            dMdtopo(i,j)=newMeasure.dx(i*nTopos+j);
        m_model->mapToGeometryData(dMdtopo,dMdgeom);
      }

  
      level++;

      if(measureChange < tolerance || level >= m_maxLevels) break;

      if( level >= m_DFadRefinement.size() ){
        m_DFadRefinement.resize(level+1);
        Refine(m_DFadRefinement[level-1], m_DFadRefinement[level]);
      }
    }

    if(m_Verbosity > 2){ 
      writeTets(explicitSimplexes);
    }
  }
}


//******************************************************************************//
Cogent::Integrator::Integrator(
   Teuchos::RCP<shards::CellTopology> _celltype,
   Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType, RealType> > _basis,
   const Teuchos::ParameterList& geomSpec):
   cellTopology(_celltype),
   m_baseElementBasis(_basis)
//******************************************************************************//
{
   
  // Parse()
  if(geomSpec.isType<int>("Verbosity"))
    m_Verbosity = geomSpec.get<int>("Verbosity");
  else
    m_Verbosity = 0;
   
  if(geomSpec.isType<int>("Error Checking"))
    m_ErrorChecking = geomSpec.get<int>("Error Checking");
  else
    m_ErrorChecking = 0;
   
  m_maxLevels = 1;
  if(geomSpec.isType<int>("Maximum Refinements"))
    m_maxLevels += geomSpec.get<int>("Maximum Refinements");
   
  if(geomSpec.isType<double>("Maximum Error"))
    m_maxError = geomSpec.get<double>("Maximum Error");
  else
    m_maxError = 1e-3;

  //
  // end Parse()
  //
  
  Cogent::ModelFactory modelFactory;
  m_model = modelFactory.create(geomSpec);

 
  m_refinement.resize(1);

  m_numDims = cellTopology->getDimension();
  FContainer<RealType> parentCoords("parentCoords",m_baseElementBasis->getCardinality(),m_numDims);

  Intrepid2::DefaultCubatureFactory cubFactory;

  int cubatureDegree = 2;
  if(geomSpec.isType<int>("Cubature Degree"))
    cubatureDegree = geomSpec.get<int>("Cubature Degree");
  Teuchos::RCP<Intrepid2::Cubature<Kokkos::Serial, RealType, RealType> > 
    m_baseElementCubature = cubFactory.create<Kokkos::Serial, RealType, RealType>(*cellTopology, cubatureDegree);

  Teuchos::RCP<Intrepid2::Cubature<Kokkos::Serial, RealType, RealType> > 
    centerCubature = cubFactory.create<Kokkos::Serial, RealType, RealType>(*cellTopology, 0);
  m_centerPoint = Cogent::FContainer<RealType>("m_centerPoint",1,m_numDims);
  m_centerWeight = Cogent::FContainer<RealType>("m_centerWeight",1);
  centerCubature->getCubature(m_centerPoint, m_centerWeight);

  const CellTopologyData *bodycelldata, *boundarycelldata;
  if(m_numDims == 3){
    bodycelldata =  shards::getCellTopologyData< shards::Tetrahedron<4> >();
    boundarycelldata =  shards::getCellTopologyData< shards::Triangle<3> >();
    m_simplexBodyBasis = Teuchos::rcp(
      new Intrepid2::Basis_HGRAD_TET_C1_FEM<Kokkos::Serial, RealType, RealType>() );
    m_simplexBoundaryBasis = Teuchos::rcp(
      new Intrepid2::Basis_HGRAD_TRI_C1_FEM<Kokkos::Serial, RealType, RealType>() );

  } else
  if(m_numDims == 2){
    bodycelldata =  shards::getCellTopologyData< shards::Triangle<3> >();
    m_simplexBodyBasis = Teuchos::rcp(
     new Intrepid2::Basis_HGRAD_TRI_C1_FEM<Kokkos::Serial, RealType, RealType>() );
    boundarycelldata = nullptr;

  } else {
    // JR:  error out
    bodycelldata = nullptr;
    boundarycelldata = nullptr;
  }

  shards::CellTopology simplexCellTopology = shards::CellTopology( bodycelldata );
  Teuchos::RCP<Intrepid2::Cubature<Kokkos::Serial, RealType, RealType> > 
    simplexBodyCubature = cubFactory.create<Kokkos::Serial, RealType, RealType>(simplexCellTopology, /*degree=*/ 6);
  int numQPs = simplexBodyCubature->getNumPoints();
  m_bodySimplexPoints  = Cogent::FContainer<RealType>("bodySimplexPoints", numQPs,m_numDims);
  m_bodySimplexWeights = Cogent::FContainer<RealType>("bodySimplexWeights",numQPs);
  simplexBodyCubature->getCubature(m_bodySimplexPoints, m_bodySimplexWeights);

  simplexCellTopology = shards::CellTopology( boundarycelldata );
  Teuchos::RCP<Intrepid2::Cubature<Kokkos::Serial, RealType, RealType> > 
    simplexBoundaryCubature = cubFactory.create<Kokkos::Serial, RealType, RealType>(simplexCellTopology, /*degree=*/ 6);
  numQPs = simplexBoundaryCubature->getNumPoints();
  m_boundarySimplexPoints = Cogent::FContainer<RealType>("m_boundarySimplexPoints",numQPs,m_numDims-1);
  m_boundarySimplexWeights = Cogent::FContainer<RealType>("m_boundarySimplexWeights",numQPs);
  simplexBoundaryCubature->getCubature(m_boundarySimplexPoints, m_boundarySimplexWeights);
  
  m_baseElementBasis->getDofCoords(parentCoords);

  const CellTopologyData& topo = *(cellTopology->getBaseCellTopologyData());

  // *** tetrahedral element ***/
  if( cellTopology->getBaseName() == shards::getCellTopologyData< shards::Tetrahedron<4> >()->name ){

    int nVerts = topo.vertex_count;

    Simplex<RealType,RealType> tet(nVerts);

    for(int ivert=0; ivert<nVerts; ivert++){
      uint nodeIndex = topo.subcell[0][ivert].node[0];
      for(uint idim=0; idim<m_numDims; idim++)
        tet.points[ivert](idim) =  parentCoords(nodeIndex, idim);
    }

    m_refinement[0].push_back(tet);
    
  } else

  
  // *** hexahedral element ***/
  if( cellTopology->getBaseName() == shards::getCellTopologyData< shards::Hexahedron<8> >()->name ){

    Vector3D<RealType>::Type bodyCenter(0.0, 0.0, 0.0);

    const int nFaceVerts = 4;
    int nFaces = topo.side_count;
    for(int iside=0; iside<nFaces; iside++){

      std::vector<Vector3D<RealType>::Type> V(nFaceVerts);
      for(int inode=0; inode<nFaceVerts; inode++){
        V[inode].clear();
        for(uint idim=0; idim<m_numDims; idim++)
          V[inode](idim) = parentCoords(topo.side[iside].node[inode],idim);
      }
      
      Vector3D<RealType>::Type sideCenter(V[0]);
      for(int inode=1; inode<nFaceVerts; inode++) sideCenter += V[inode];
      sideCenter /= nFaceVerts;

      for(int inode=0; inode<nFaceVerts; inode++){
        Simplex<RealType,RealType> tet(4);
        int jnode = (inode+1)%nFaceVerts;
        tet.points[0] = V[inode];
        tet.points[1] = V[jnode];
        tet.points[2] = sideCenter;
        tet.points[3] = bodyCenter;
        m_refinement[0].push_back(tet);
      }
    }

  } else

  // *** quadrilateral element **/
  if( cellTopology->getBaseName() == shards::getCellTopologyData< shards::Quadrilateral<4> >()->name ){

    const int nVerts = topo.vertex_count;
    std::vector<Vector3D<RealType>::Type> V(nVerts);
    for(int inode=0; inode<nVerts; inode++){
      V[inode].clear();
      for(uint idim=0; idim<m_numDims; idim++)
        V[inode](idim) = parentCoords(topo.subcell[0][inode].node[0],idim);
    }
      
    Vector3D<RealType>::Type sideCenter(V[0]);
    for(int inode=1; inode<nVerts; inode++) sideCenter += V[inode];
    sideCenter /= nVerts;

    for(int inode=0; inode<nVerts; inode++){
      Simplex<RealType,RealType> tri(3);
      int jnode = (inode+1)%nVerts;
      tri.points[0] = V[inode];
      tri.points[1] = V[jnode];
      tri.points[2] = sideCenter;
      m_refinement[0].push_back(tri);
    }
  } else

  // *** triangle element **/
  if( cellTopology->getBaseName() == shards::getCellTopologyData< shards::Triangle<3> >()->name ){

    const int nVerts = topo.vertex_count;
    Simplex<RealType,RealType> tri(nVerts);
    for(int inode=0; inode<nVerts; inode++){
      tri.points[inode].clear();
      for(uint idim=0; idim<m_numDims; idim++)
        tri.points[inode](idim) = parentCoords(topo.subcell[0][inode].node[0],idim);
    }
    m_refinement[0].push_back(tri);
  } else {
    // error out
  }

  m_DFadRefinement.resize(1);
  uint nPoly = m_refinement[0].size();
  m_DFadRefinement[0].resize(nPoly);
  for(uint ip=0; ip<nPoly; ip++){
    int nPts = m_refinement[0][ip].points.size();
    m_DFadRefinement[0][ip] = Simplex<DFadType,DFadType>(nPts);
    for(int ipt=0; ipt<nPts; ipt++)
      for(int idim=0; idim<3; idim++)
        m_DFadRefinement[0][ip].points[ipt](idim) = m_refinement[0][ip].points[ipt](idim);
  }

  RealType interfaceValue {0.0};
  if(geomSpec.isType<double>("Interface Value"))
    interfaceValue = geomSpec.get<double>("Interface Value");

  m_dicer = Teuchos::rcp(new Dicer(cellTopology, m_baseElementBasis, interfaceValue));

}


}
