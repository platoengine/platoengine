//******************************************************************************//
template<typename V, typename P>
void Cogent::writeTets(
  const std::vector<Simplex<V,P> >& explicitSimplexes, 
  std::string format)
//******************************************************************************//
{
  int nTets = explicitSimplexes.size();
  if(nTets == 0) return;

  if(format == "vtk")
    writeTetsVTK(explicitSimplexes);
  else
  if(format == "case")
    writeTetsCase(explicitSimplexes);
}

//******************************************************************************//
template<typename V, typename P>
void Cogent::writeTetsCase(const std::vector<Simplex<V,P> >& explicitSimplexes)
//******************************************************************************//
{

  int nTets = explicitSimplexes.size();

  std::cout << "Tet output from Cogent" << std::endl;
  std::cout << "Ensight Gold format" << std::endl;
  std::cout << "node id assign" << std::endl;
  std::cout << "element id assign" << std::endl;
  std::cout << "part" << std::endl;
  std::cout << "         1" << std::endl;
  std::cout << "3D uns-elements" << std::endl;
  std::cout << "coordinates" << std::endl;
  std::cout << std::setw(10) << nTets*Cogent::nTetPts << std::endl;

  std::cout << std::scientific;
  std::cout << std::setprecision(5);

  typename std::vector<Simplex<V,P>>::const_iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); ++it){
    const std::vector<typename Vector3D<P>::Type >& points = it->points;
    std::cout << std::setw(12) << points[0](0) << std::endl;
    std::cout << std::setw(12) << points[1](0) << std::endl;
    std::cout << std::setw(12) << points[2](0) << std::endl;
    std::cout << std::setw(12) << points[3](0) << std::endl;
  }
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); ++it){
    const std::vector<typename Vector3D<P>::Type >& points = it->points;
    std::cout << std::setw(12) << points[0](1) << std::endl;
    std::cout << std::setw(12) << points[1](1) << std::endl;
    std::cout << std::setw(12) << points[2](1) << std::endl;
    std::cout << std::setw(12) << points[3](1) << std::endl;
  }
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); ++it){
    const std::vector<typename Vector3D<P>::Type >& points = it->points;
    std::cout << std::setw(12) << points[0](2) << std::endl;
    std::cout << std::setw(12) << points[1](2) << std::endl;
    std::cout << std::setw(12) << points[2](2) << std::endl;
    std::cout << std::setw(12) << points[3](2) << std::endl;
  }

  std::cout << "tetra4" << std::endl;
  std::cout << std::setw(10);
  std::cout << nTets << std::endl;
  int iNode=1;
  for(int i=0; i<nTets; i++){
    std::cout << std::setw(10) << iNode; iNode++;
    std::cout << std::setw(10) << iNode; iNode++;
    std::cout << std::setw(10) << iNode; iNode++;
    std::cout << std::setw(10) << iNode << std::endl; iNode++;
  }
  
}

//******************************************************************************//
template<typename V, typename P>
void Cogent::writeTetsVTK(const std::vector<Simplex<V,P> >& explicitSimplexes)
//******************************************************************************//
{

  int nTets = explicitSimplexes.size();

  std::cout << "# vtk DataFile Version 2.0" << std::endl;
  std::cout << "3d tets" << std::endl;
  std::cout << "ASCII" << std::endl;
  std::cout << std::endl;
  std::cout << "DATASET UNSTRUCTURED_GRID" << std::endl;
  std::cout << "POINTS " << nTets*Cogent::nTetPts << " float" << std::endl;

  typename std::vector<Simplex<V,P>>::const_iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); ++it){
    const std::vector<typename Vector3D<P>::Type >& points = it->points;
    std::cout << points[0](0) << " " << points[0](1) << " " << points[0](2) << std::endl;
    std::cout << points[1](0) << " " << points[1](1) << " " << points[1](2) << std::endl;
    std::cout << points[2](0) << " " << points[2](1) << " " << points[2](2) << std::endl;
    std::cout << points[3](0) << " " << points[3](1) << " " << points[3](2) << std::endl;
  }
  std::cout << std::endl;
  std::cout << "CELLS " << nTets << " " << nTets*(Cogent::nTetPts+1) << std::endl;
  int ptIndex=0;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); ++it){
    std::cout << Cogent::nTetPts << " " << ptIndex << " " << ptIndex+1 << " " << ptIndex+2 << " " << ptIndex+3 << std::endl;
    ptIndex+=Cogent::nTetPts;
  }
  std::cout << std::endl;
  std::cout << "CELL_TYPES " << nTets << std::endl;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); ++it){
    std::cout << "10" << std::endl;
  }

  int nTopos = explicitSimplexes[0].fieldvals[0].size();
  std::cout << std::endl;
  std::cout << "POINT_DATA " << nTets*Cogent::nTetPts << std::endl;
  for(int iTopo=0; iTopo<nTopos; iTopo++){
    std::cout << "SCALARS topo_" << iTopo << " float " << iTopo+1 << std::endl;
    std::cout << "LOOKUP_TABLE default" << std::endl;
    for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); ++it){
      const std::vector<std::vector<V> >& fieldvals = it->fieldvals;
      std::cout << fieldvals[0][iTopo] << std::endl;
      std::cout << fieldvals[1][iTopo] << std::endl;
      std::cout << fieldvals[2][iTopo] << std::endl;
      std::cout << fieldvals[3][iTopo] << std::endl;
    }
    std::cout << std::endl;
  }
}

//******************************************************************************//
template<typename V, typename P>
void Cogent::writeTris(const std::vector<Simplex<V,P> >& explicitSimplexes)
//******************************************************************************//
{
  int nTris = explicitSimplexes.size();
  std::cout << "# vtk DataFile Version 2.0" << std::endl;
  std::cout << "3d tris" << std::endl;
  std::cout << "ASCII" << std::endl;
  std::cout << std::endl;
  std::cout << "DATASET UNSTRUCTURED_GRID" << std::endl;
  std::cout << "POINTS " << nTris*Cogent::nTriPts << " float" << std::endl;

  typename std::vector<Simplex<V,P>>::const_iterator it;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); ++it){
    const std::vector<typename Vector3D<P>::Type >& points = it->points;
    std::cout << points[0](0) << " " << points[0](1) << " " << points[0](2) << std::endl;
    std::cout << points[1](0) << " " << points[1](1) << " " << points[1](2) << std::endl;
    std::cout << points[2](0) << " " << points[2](1) << " " << points[2](2) << std::endl;
  }
  std::cout << std::endl;
  std::cout << "CELLS " << nTris << " " << nTris*(Cogent::nTriPts+1) << std::endl;
  int ptIndex=0;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); ++it){
    std::cout << Cogent::nTriPts << " " << ptIndex << " " << ptIndex+1 << " " << ptIndex+2 << std::endl;
    ptIndex+=Cogent::nTriPts;
  }
  std::cout << std::endl;
  std::cout << "CELL_TYPES " << nTris << std::endl;
  for(it=explicitSimplexes.begin(); it!=explicitSimplexes.end(); ++it){
    std::cout << "5" << std::endl;
  }
}

