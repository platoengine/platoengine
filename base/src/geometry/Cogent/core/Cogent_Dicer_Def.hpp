//******************************************************************************//
//  TODO: N and P are never different.......
template<typename N, typename V, typename P>
void Cogent::Dicer::Project(
     const Cogent::FContainer<N>& topoData, 
     std::vector<Simplex<V,P> >& implicitSimplexes)
//******************************************************************************//
{
#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif

  int numNodes = m_basis->getCardinality();
  int nPoints = implicitSimplexes[0].points.size();
  int nTopos = topoData.dimension(1);

  Cogent::FContainer<N> 
   Nvals = Kokkos::createDynRankView(topoData,"Nvals",numNodes,nPoints);
  Cogent::FContainer<N>
   evalPoints = Kokkos::createDynRankView(topoData,"evalPoints",nPoints,m_numDims);

  typename std::vector<Simplex<V,P> >::iterator it;
  for(it=implicitSimplexes.begin(); it!=implicitSimplexes.end(); it++){
   
    std::vector<typename Vector3D<P>::Type>& pnts = it->points;

    // buffer the vertex coordinates
    for(int i=0; i<nPoints; i++)
      for(uint j=0; j<m_numDims; j++){
        evalPoints(i, j) = pnts[i](j);
      }

    // evaluate the parent element basis at the vertices
    getValues<V,P>(Nvals, evalPoints);

    // compute the topology values at the vertices: 
    // \phi_i(\xi) = N_I(\xi) \phi_{Ii}
    std::vector<std::vector<V> >& vals = it->fieldvals;
    for(int i=0; i<nPoints; i++){
      vals[i].resize(nTopos);
      for(int j=0; j<nTopos; j++){
        vals[i][j] = 0.0;
        for(int J=0; J<numNodes; J++)
          vals[i][j] += Nvals(J,i)*topoData(J,j);
//        if(vals[i][j] == 0.0) vals[i][j] += 1e-8;
      }
    }

  }
}

//******************************************************************************//
template<typename V, typename P>
void Cogent::Dicer::Mesh(
  const std::vector<typename Vector3D<P>::Type >& points,
  std::vector<Simplex<V,P> >& explicitSimplexes, int iTopo)
//******************************************************************************//
{
// Description:
//

  std::vector<int> map;
  SortMap<P>(points, map);
  if(map.size() == 0) return;
  
  // find centerpoint
  uint nNewPoints = points.size();
  for(uint i=2; i<nNewPoints; i++){
    Simplex<V,P> tri(Cogent::nTriPts);
    tri.points[0] = points[map[0]];
    tri.points[1] = points[map[i-1]];
    tri.points[2] = points[map[i]];
    tri.bodyLS = iTopo;
    explicitSimplexes.push_back(tri);
  }

}


//******************************************************************************//
template<typename V, typename P>
void Cogent::Dicer::Cut(
  const Cogent::FContainer<P>& topoData, 
        std::vector<Simplex<V,P> >& implicitSimplexes, 
        std::vector<Simplex<V,P> >& explicitSimplexes)
//******************************************************************************//
{
 /*
 *  This function creates a conformal decomposition.  Subdomains that contain 
 *  material are returned in explicitSimplexes.
 */

#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif


  Project(/*in*/ topoData, /*in/out*/ implicitSimplexes);
   
  std::vector<Simplex<V,P> > simplexLists[2];
  std::vector<Simplex<V,P> > *simplexListIn, *simplexListOut, *listPtr;

  simplexListIn = &(simplexLists[0]);
  simplexListOut = &(simplexLists[1]);

  // initialize simplex senses to 'Mixed'
  typename std::vector<Simplex<V,P> >::iterator it;
  int nTopos = implicitSimplexes[0].fieldvals[0].size();
  for(it=implicitSimplexes.begin(); it!=implicitSimplexes.end(); it++){
    std::vector<Sense>& senses = it->sense;
    senses.resize(nTopos);
    for(int j=0; j<nTopos; j++)
      senses[j] = Sense::Mixed;
  }

  if(implicitSimplexes.size() > 0 && implicitSimplexes[0].points.size()==Cogent::nTetPts) {
    // volume
#ifdef USE_OLD_DICER
    Positive<P> positive;
    Negative<P> negative;
    CutTet(implicitSimplexes, *simplexListOut, positive, /*iTopo=*/0);
    CutTet(implicitSimplexes, *simplexListOut, negative, /*iTopo=*/0);
#else
    CutTet(implicitSimplexes, *simplexListOut, /*iTopo=*/0);
#endif
    Project(/*in*/ topoData, /*in/out*/ *simplexListOut);
    for(int iTopo=1; iTopo<nTopos; iTopo++){
      listPtr = simplexListIn;
      simplexListIn = simplexListOut;
      simplexListOut = listPtr;
      simplexListOut->clear();
#ifdef USE_OLD_DICER
      CutTet(*simplexListIn, *simplexListOut, positive, iTopo);
      CutTet(*simplexListIn, *simplexListOut, negative, iTopo);
#else
      CutTet(*simplexListIn, *simplexListOut, iTopo);
#endif
      Project(/*in*/ topoData, /*in/out*/ *simplexListOut);
    }
  } else 
  if(implicitSimplexes.size() > 0 && implicitSimplexes[0].points.size()==Cogent::nTriPts) {
    // surface
    int nTopos = implicitSimplexes[0].fieldvals[0].size();
    for(int iTopo=0; iTopo<nTopos; iTopo++){
      Positive<P> positive;
      CutTri(implicitSimplexes, explicitSimplexes, positive, iTopo);
    }
  } else {
    // throw exception.
  }

  explicitSimplexes = std::move(*simplexListOut);

}


//******************************************************************************//
template<typename C, typename V, typename P>
void Cogent::Dicer::CutTri(
  const std::vector<Simplex<V,P> >& implicitSimplexes, 
        std::vector<Simplex<V,P> >& explicitSimplexes, const C compare, int iTopo)
//******************************************************************************//
{
// Description:
//

#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif


    typename std::vector<Simplex<V,P> >::const_iterator it;
    for(it=implicitSimplexes.begin(); it!=implicitSimplexes.end(); it++){
      
      const std::vector<typename Vector3D<P>::Type >& triPoints = it->points;
      const std::vector<std::vector<V> >& triVals = it->fieldvals;
      int nTotalPoints = triPoints.size();

      std::vector<typename Vector3D<P>::Type > points;
      for(int i=0; i<nTotalPoints; i++){
        if(compare.compare(triVals[i][iTopo], m_interfaceValue)){
          points.push_back(triPoints[i]);
        } 
      }

      std::vector<int> map;
      SortMap<P>(triPoints, map);
      if(map.size() == 0) return;

      // find itersections
      for(int i=0; i<nTotalPoints; i++){
        int j = (i+1)%nTotalPoints;
        int im = map[i], jm = map[j];
        if((triVals[im][iTopo]-m_interfaceValue)*(triVals[jm][iTopo]-m_interfaceValue) < 0.0){
          typename Vector3D<P>::Type newpoint = triPoints[im];
          V factor = (m_interfaceValue - triVals[im][iTopo])/(triVals[jm][iTopo]-triVals[im][iTopo]);
          newpoint += factor*(triPoints[jm]-triPoints[im]);
          points.push_back(newpoint);
        }
      }
      
      if( (points.size() > 2) && (!areColinear<P>(points)) ){
        SortMap<P>(points, map);
        if(map.size() == 0) return;
        uint nNewPoints = points.size();
        for(uint i=2; i<nNewPoints; i++){
          Simplex<V,P> tri(Cogent::nTriPts);
          tri.points[0] = points[map[0]];
          tri.points[1] = points[map[i-1]];
          tri.points[2] = points[map[i]];
          tri.bodyLS = it->bodyLS;
          explicitSimplexes.push_back(tri);
        }
      }
    }
}

//******************************************************************************//
template<typename C, typename V, typename P>
Cogent::Sense Cogent::Dicer::getSense(
  const Simplex<V,P>& simplex,
  const C& compare, int iTopo)
//******************************************************************************//
{ // This function checks topology iTopo on the simplex to see if it
// meets the comparison (either > or < ) against m_interfaceValue.  

  // initially, sense is assumed to meet the comparison
  Cogent::Sense sense = compare.sense;

  // if any of the values of topology iTopo don't meet the comparison,
  // set sense to NotSet and break, otherwise leave sense alone.
  const std::vector<std::vector<V> >& vals = simplex.fieldvals;
  uint nPts = simplex.points.size();
  for(uint i=0; i<nPts; i++){
    if(!compare.compare(vals[i][iTopo], m_interfaceValue)){
      sense = Cogent::Sense::NotSet;
      break;
    }
  }
  // if sense met the comparison, return the sense.
  if( sense == compare.sense ) return sense;

  // Check for sign changes in the nodal values of topology iTopo. If
  // there are mixed signs, return Mixed.
  for(uint i=0; i<nPts; i++)
    for(uint j=i+1; j<nPts; j++) // segment (i,j)
      if((vals[i][iTopo]-m_interfaceValue)*(vals[j][iTopo]-m_interfaceValue) < 0.0)
        return Cogent::Sense::Mixed;
  
  // To get here, sense is NotSet
  return sense;
}


//******************************************************************************//
template<typename V, typename P>
void Cogent::Dicer::CutTet(
  const std::vector<Simplex<V,P> >& implicitSimplexes,
        std::vector<Simplex<V,P> >& explicitSimplexes,
        int iTopo)
//******************************************************************************//
{
  for( auto& implicitSimplex : implicitSimplexes ){
    const std::vector<std::vector<V>>& topoVals = implicitSimplex.fieldvals;
    short int d0 = (topoVals[0][iTopo] == 0) ? 1 : ((topoVals[0][iTopo] > 0.0) ? 2 : 0);
    short int d1 = (topoVals[1][iTopo] == 0) ? 1 : ((topoVals[1][iTopo] > 0.0) ? 2 : 0);
    short int d2 = (topoVals[2][iTopo] == 0) ? 1 : ((topoVals[2][iTopo] > 0.0) ? 2 : 0);
    short int d3 = (topoVals[3][iTopo] == 0) ? 1 : ((topoVals[3][iTopo] > 0.0) ? 2 : 0);
    unsigned short int index = d3*1 + d2*3 + d1*9 + d0*27;

    const SimplexStencil& cutStencil = m_cutStencils[index];

    int nTets = cutStencil.m_subTets.size();
    int nIntx = cutStencil.m_newPts.size();

    int nPts = 4;

    std::vector<typename Vector3D<P>::Type> p(nPts+nIntx);
    p[0] = implicitSimplex.points[0];
    p[1] = implicitSimplex.points[1];
    p[2] = implicitSimplex.points[2];
    p[3] = implicitSimplex.points[3];
    for(int i=0; i<nIntx; i++){
      int p0 = cutStencil.m_newPts[i][0];
      int p1 = cutStencil.m_newPts[i][1];
      auto& v0 = topoVals[p0][iTopo];
      auto& v1 = topoVals[p1][iTopo];
      p[nPts+i] = p[p0]*(v1/(v1-v0)) - p[p1]*(v0/(v1-v0));
    }

    std::vector<int> topoIDs = implicitSimplex.boundaryLS;
    topoIDs.push_back(iTopo);
    topoIDs.push_back(-1);

    for(int i=0; i<nTets; i++){
      Cogent::Simplex<V,P> tet(nPts);
      auto& topoMap = cutStencil.m_topoMap[i];
      for(int j=0; j<nPts; j++){
        tet.points[j] = p[cutStencil.m_subTets[i][j]];
        tet.boundaryLS[j] = topoIDs[topoMap[j]];
      }
      tet.sense = implicitSimplex.sense;
      tet.sense[iTopo] = cutStencil.m_sense[i];
      explicitSimplexes.push_back(tet);
    }
  }
}


//******************************************************************************//
template<typename C, typename V, typename P>
void Cogent::Dicer::CutTet(
  const std::vector<Simplex<V,P> >& implicitSimplexes, 
        std::vector<Simplex<V,P> >& explicitSimplexes,
  const C& compare, int iTopo)
//******************************************************************************//
{
// Cut input tetrahedra (implicitSimplexes) by 'compare' comparison against
// topology iTopo and return tetrahedra that meet the comparison.

#ifdef ENABLE_BOOST_TIMER
  Timer timer(__func__);
#endif


  int iSimp=0;

  typename std::vector<Simplex<V,P> >::const_iterator it;
  for(it=implicitSimplexes.begin(); it!=implicitSimplexes.end(); it++){

    Cogent::Sense mySense = getSense(*it,compare,iTopo);

    if(mySense == Cogent::Sense::Mixed){
  
      // Cut tet surfaces
      const std::vector<typename Vector3D<P>::Type >& pnts = it->points;
      const std::vector<std::vector<V> >& vals = it->fieldvals;
      std::vector<Simplex<V,P> > implicitTris(Cogent::nTetPts);

      for(uint i=0; i<Cogent::nTetPts; i++){
        implicitTris[i] = Simplex<V,P>(Cogent::nTriPts);
        Simplex<V,P>& face = implicitTris[i];
        for(uint j=0; j<Cogent::nTriPts; j++){
          face.points[j] = pnts[(i+j)%Cogent::nTetPts];
          // only bring along topology iTopo (so resize to one)
          face.fieldvals[j].resize(1);
          face.fieldvals[j][0] = vals[(i+j)%Cogent::nTetPts][iTopo];
        }
        face.bodyLS = it->boundaryLS[i];
      }
      std::vector<Simplex<V,P> > explicitTris;
      // there's only one active topology, so the iTopo argument is 0
      CutTri(implicitTris, explicitTris, compare, /*iTopo=*/0);

      // construct points on intersecting surface
      std::vector<typename Vector3D<P>::Type> cutpoints;
      for(uint i=0; i<Cogent::nTetPts; i++)
        for(uint j=i+1; j<Cogent::nTetPts; j++) // segment (i,j)
          if((vals[i][iTopo]-m_interfaceValue)*(vals[j][iTopo]-m_interfaceValue) < 0.0){
            typename Vector3D<P>::Type newpoint = pnts[i];
            P factor = (m_interfaceValue - vals[i][iTopo])/(vals[j][iTopo]-vals[i][iTopo]);
            newpoint += factor*(pnts[j]-pnts[i]);
            cutpoints.push_back(newpoint);
          }

      // check for points that are exactly m_interfaceValue
      for(uint i=0; i<Cogent::nTetPts; i++)
        if(vals[i][iTopo] == m_interfaceValue) cutpoints.push_back(pnts[i]);

      // mesh intersecting plane
      Mesh(cutpoints, explicitTris, iTopo);

      // if no surface simplexes where found ...
      if(explicitTris.size() == 0) continue;

      // create tets
      typename Vector3D<P>::Type center(0.0, 0.0, 0.0);
      typename std::vector<Simplex<V,P> >::iterator itg;
      for(itg=explicitTris.begin(); itg!=explicitTris.end(); itg++){
        for(uint i=0; i<Cogent::nTriPts; i++) center += itg->points[i];
      }
      center /= Cogent::nTriPts*explicitTris.size();

      int iSurf=0;
      for(itg=explicitTris.begin(); itg!=explicitTris.end(); itg++){
        Simplex<V,P> tet(Cogent::nTetPts);
        for(uint i=0; i<Cogent::nTriPts; i++) tet.points[i] = itg->points[i];
        tet.points[3] = center;
        V vol = Volume(tet);
        if( vol < 0.0 ){
          typename Vector3D<P>::Type p = tet.points[0];
          tet.points[0] = tet.points[1];
          tet.points[1] = p;
        }
        tet.sense = it->sense;
        tet.sense[iTopo] = compare.sense;

        tet.boundaryLS[0] = itg->bodyLS;
// JR hack
        if(fabs(vol) > 1.0e-9)
          explicitSimplexes.push_back(tet);
        iSurf++;
      }
    } else 
    if(mySense == compare.sense){
      Simplex<V,P> tet(Cogent::nTetPts);
      tet = *it;
      tet.sense[iTopo] = compare.sense;
      explicitSimplexes.push_back(tet);
    }
    iSimp++;
  }
}


//******************************************************************************//
template<typename V, typename P>
V Cogent::Dicer::Volume(Simplex<V,P>& simplex)
//******************************************************************************//
{
  std::vector<typename Vector3D<P>::Type>& points = simplex.points;

  if(points.size() == 4){
    P x0 = points[0](0), y0 = points[0](1), z0 = points[0](2);
    P x1 = points[1](0), y1 = points[1](1), z1 = points[1](2);
    P x2 = points[2](0), y2 = points[2](1), z2 = points[2](2);
    P x3 = points[3](0), y3 = points[3](1), z3 = points[3](2);
    P j11 = -x0+x1, j12 = -x0+x2, j13 = -x0+x3;
    P j21 = -y0+y1, j22 = -y0+y2, j23 = -y0+y3;
    P j31 = -z0+z1, j32 = -z0+z2, j33 = -z0+z3;
    P detj = -j13*j22*j31+j12*j23*j31+j13*j21*j32-j11*j23*j32-j12*j21*j33+j11*j22*j33;

    return detj/6.0;

  } else 
  if(points.size() == 3){
    P x0 = points[0](0), y0 = points[0](1);
    P x1 = points[1](0), y1 = points[1](1);
    P x2 = points[2](0), y2 = points[2](1);
    P detj = -x1*y0+x2*y0+x0*y1-x2*y1-x0*y2+x1*y2;

    return detj/2.0;

  }
  return 0.0;
}
