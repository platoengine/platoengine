#include <Intrepid2_HGRAD_HEX_Cn_FEM.hpp>
#include <core/Cogent_IntegratorFactory.hpp>
#include <core/Cogent_WriteUtils.hpp>

#include <algorithm>
#include <iostream>
#include <stdlib.h>

#include <Teuchos_ParameterList.hpp>
#include <Teuchos_XMLParameterListHelpers.hpp>
#include <Teuchos_CommandLineProcessor.hpp>

int main(int argc, char **argv) {

  using Cogent::RealType;

  Kokkos::initialize(argc, argv);


  // read geometry file and setup
  //
  Teuchos::CommandLineProcessor tCLP(
      /*throwExceptions=*/       false, 
      /*recogniseAllOptions=*/   false, 
      /*addOutputSetupOptions=*/ false);

  tCLP.setDocString(
      "Cogent computes a discrete geometric representation \n"
      "from a parameterized constructive solid geometry (CSG) model.\n");

  std::string       xmlFileName = "parameterized.xml";
  tCLP.setOption("input", &xmlFileName, xmlFileName.c_str());

  std::string       outFileName = "model.vtk";
  tCLP.setOption("output", &outFileName, outFileName.c_str());

  Teuchos::CommandLineProcessor::EParseCommandLineReturn parseReturn =
      Teuchos::CommandLineProcessor::PARSE_ERROR;
  parseReturn = tCLP.parse(argc, argv);
  if (parseReturn != Teuchos::CommandLineProcessor::PARSE_SUCCESSFUL) {
    std::cerr << "Command line processing failed !" << std::endl;
    exit(-1);
  }

  Teuchos::ParameterList geomSpec("Geometry");
  Teuchos::updateParametersFromXmlFile(xmlFileName, Teuchos::ptrFromRef(geomSpec));

  Teuchos::Array<double> paramVals = geomSpec.get<Teuchos::Array<double>>("Shape Parameter Values");
  int numVals = paramVals.size();
  Cogent::FContainer<RealType> geomVals("geomVals",numVals);
  for(int i=0; i<numVals; i++) geomVals(i) = paramVals[i];

  const Cogent::FContainer<RealType>& constGeomVals = geomVals;
  std::vector<Cogent::Simplex<RealType,RealType>> tets;
  std::vector<Cogent::Simplex<RealType,RealType>> tris;

  Teuchos::ParameterList& gridSpec = geomSpec.sublist("Background Grid");
  double gridSize = gridSpec.get<double>("Grid Size");
  Teuchos::Array<double> Xlimits = gridSpec.sublist("Limits").get<Teuchos::Array<double>>("X");
  Teuchos::Array<double> Ylimits = gridSpec.sublist("Limits").get<Teuchos::Array<double>>("Y");
  Teuchos::Array<double> Zlimits = gridSpec.sublist("Limits").get<Teuchos::Array<double>>("Z");

  std::string gridType = gridSpec.get<std::string>("Grid Type","Hex8");
  std::transform(gridType.begin(), gridType.end(), gridType.begin(), ::tolower);

  Teuchos::RCP<shards::CellTopology> celltype;
  Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType> > intrepidBasis;
  if (gridType == "hex8" || gridType == "hex") {
    // define HEX8 element topology
    const CellTopologyData& celldata = *shards::getCellTopologyData< shards::Hexahedron<8> >();
    celltype = Teuchos::rcp(new shards::CellTopology( &celldata ) );

    // define basis
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM<Kokkos::Serial, RealType, RealType>() );
  } else 
  if (gridType == "tet4" || gridType == "tet") {
    // define TET4 element topology
    const CellTopologyData& celldata = *shards::getCellTopologyData< shards::Tetrahedron<4> >();
    celltype = Teuchos::rcp(new shards::CellTopology( &celldata ) );

    // define basis
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_TET_C1_FEM<Kokkos::Serial, RealType, RealType>() );
  }
  
  // create integrator
  Cogent::IntegratorFactory integratorFactory;
  Teuchos::RCP<Cogent::Integrator> integrator = integratorFactory.create(celltype, intrepidBasis, geomSpec);

  // create element coordinates
  int numNodes = celltype->getVertexCount();
  int numDims = celltype->getDimension();
  Cogent::FContainer<RealType> coordCon("coordCon", numNodes, numDims);

  int nIntsX = (Xlimits[1]-Xlimits[0])/gridSize;
  int nIntsY = (Ylimits[1]-Ylimits[0])/gridSize;
  int nIntsZ = (Zlimits[1]-Zlimits[0])/gridSize;
  if (gridType == "hex8" || gridType == "hex") {
    RealType dx = gridSize;
    for(int i=0; i<nIntsX; i++)
      for(int j=0; j<nIntsY; j++)
        for(int k=0; k<nIntsZ; k++){
          coordCon(0,0) = (i  )*dx ; coordCon(0,1) = (j  )*dx; coordCon(0,2) = (k  )*dx;
          coordCon(1,0) = (i+1)*dx ; coordCon(1,1) = (j  )*dx; coordCon(1,2) = (k  )*dx;
          coordCon(2,0) = (i+1)*dx ; coordCon(2,1) = (j+1)*dx; coordCon(2,2) = (k  )*dx;
          coordCon(3,0) = (i  )*dx ; coordCon(3,1) = (j+1)*dx; coordCon(3,2) = (k  )*dx;
          coordCon(4,0) = (i  )*dx ; coordCon(4,1) = (j  )*dx; coordCon(4,2) = (k+1)*dx;
          coordCon(5,0) = (i+1)*dx ; coordCon(5,1) = (j  )*dx; coordCon(5,2) = (k+1)*dx;
          coordCon(6,0) = (i+1)*dx ; coordCon(6,1) = (j+1)*dx; coordCon(6,2) = (k+1)*dx;
          coordCon(7,0) = (i  )*dx ; coordCon(7,1) = (j+1)*dx; coordCon(7,2) = (k+1)*dx;
  
          tets.clear();
          integrator->getBodySimplexes(constGeomVals, coordCon, tets);
          Cogent::getSurfaceTris(tets,tris);
        }
  } else 
  if (gridType == "tet4" || gridType == "tet") {
    // create a base hex that we'll dice into 24 tets:
    const CellTopologyData& hextopo = *shards::getCellTopologyData< shards::Hexahedron<8> >();
    const int hexNumNodes = hextopo.vertex_count;
    Cogent::FContainer<RealType> hexCoordCon("hexCoordCon", hexNumNodes, numDims);
    RealType dx = gridSize;
    for(int i=0; i<nIntsX; i++)
      for(int j=0; j<nIntsY; j++)
        for(int k=0; k<nIntsZ; k++){
          hexCoordCon(0,0) = (i  )*dx ; hexCoordCon(0,1) = (j  )*dx; hexCoordCon(0,2) = (k  )*dx;
          hexCoordCon(1,0) = (i+1)*dx ; hexCoordCon(1,1) = (j  )*dx; hexCoordCon(1,2) = (k  )*dx;
          hexCoordCon(2,0) = (i+1)*dx ; hexCoordCon(2,1) = (j+1)*dx; hexCoordCon(2,2) = (k  )*dx;
          hexCoordCon(3,0) = (i  )*dx ; hexCoordCon(3,1) = (j+1)*dx; hexCoordCon(3,2) = (k  )*dx;
          hexCoordCon(4,0) = (i  )*dx ; hexCoordCon(4,1) = (j  )*dx; hexCoordCon(4,2) = (k+1)*dx;
          hexCoordCon(5,0) = (i+1)*dx ; hexCoordCon(5,1) = (j  )*dx; hexCoordCon(5,2) = (k+1)*dx;
          hexCoordCon(6,0) = (i+1)*dx ; hexCoordCon(6,1) = (j+1)*dx; hexCoordCon(6,2) = (k+1)*dx;
          hexCoordCon(7,0) = (i  )*dx ; hexCoordCon(7,1) = (j+1)*dx; hexCoordCon(7,2) = (k+1)*dx;

          Cogent::Vector3D<RealType>::Type bodyCenter(0.0, 0.0, 0.0);
          for(int inode=0; inode<hexNumNodes; inode++){
            for(int idim=0; idim<numDims; idim++){
              bodyCenter(idim) += hexCoordCon(inode,idim);
            }
          }
          bodyCenter /= hexNumNodes;

          const int nFaceVerts = 4;
          int nFaces = hextopo.side_count;
          for(int iside=0; iside<nFaces; iside++){

            std::vector<Cogent::Vector3D<RealType>::Type> V(nFaceVerts);
            for(int inode=0; inode<nFaceVerts; inode++){
              V[inode].clear();
              for(uint idim=0; idim<numDims; idim++)
                V[inode](idim) = hexCoordCon(hextopo.side[iside].node[inode],idim);
            }

            Cogent::Vector3D<RealType>::Type sideCenter(V[0]);
            for(int inode=1; inode<nFaceVerts; inode++) sideCenter += V[inode];
            sideCenter /= nFaceVerts;

            for(int inode=0; inode<nFaceVerts; inode++){
              Cogent::Simplex<RealType,RealType> tet(4);
              int jnode = (inode+1)%nFaceVerts;
              coordCon(0,0) = V[inode](0);   coordCon(0,1) = V[inode](1);   coordCon(0,2) = V[inode](2);
              coordCon(1,0) = V[jnode](0);   coordCon(1,1) = V[jnode](1);   coordCon(1,2) = V[jnode](2);
              coordCon(2,0) = sideCenter(0); coordCon(2,1) = sideCenter(1); coordCon(2,2) = sideCenter(2);
              coordCon(3,0) = bodyCenter(0); coordCon(3,1) = bodyCenter(1); coordCon(3,2) = bodyCenter(2);

              tets.clear();
              integrator->getBodySimplexes(constGeomVals, coordCon, tets);
              Cogent::getSurfaceTris(tets,tris);
            }
          }
        }
  }

  std::ofstream tModelFile;
  tModelFile.open(outFileName);

  Cogent::writeTris(tris, tModelFile);

  tModelFile.close();
}
