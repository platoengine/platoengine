#include <Intrepid2_HGRAD_HEX_Cn_FEM.hpp>
#include <core/Cogent_IntegratorFactory.hpp>
#include <core/Cogent_WriteUtils.hpp>

#include <iostream>
#include <stdlib.h>

#include <Teuchos_ParameterList.hpp>
#include <Teuchos_XMLParameterListHelpers.hpp>

int main(int argc, char **argv) {

  Kokkos::initialize(argc, argv);

  // define HEX8 element topology
  const CellTopologyData& celldata = *shards::getCellTopologyData< shards::Hexahedron<8> >();
  Teuchos::RCP<shards::CellTopology> celltype = Teuchos::rcp(new shards::CellTopology( &celldata ) );

  // define basis
  Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType> >
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM<Kokkos::Serial, RealType, RealType>() );
  
  // read geometry file and setup
  std::string xmlFileName = "parameterized.xml";
  Teuchos::ParameterList geomSpec;
  Teuchos::updateParametersFromXmlFile(xmlFileName, Teuchos::ptrFromRef(geomSpec));

  // create integrator
  Cogent::IntegratorFactory integratorFactory;
  Teuchos::RCP<Cogent::Integrator> integrator = integratorFactory.create(celltype, intrepidBasis, geomSpec);

  // create element coordinates
  int numNodes = celltype->getVertexCount();
  int numDims = celltype->getDimension();
  Cogent::FContainer<RealType> coordCon("coordCon", numNodes, numDims);

  Teuchos::Array<double> paramVals = geomSpec.get<Teuchos::Array<double>>("Shape Parameters Initial Values");
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

  int nIntsX = (Xlimits[1]-Xlimits[0])/gridSize;
  int nIntsY = (Ylimits[1]-Ylimits[0])/gridSize;
  int nIntsZ = (Zlimits[1]-Zlimits[0])/gridSize;
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

  Cogent::writeTris(tris);
}
