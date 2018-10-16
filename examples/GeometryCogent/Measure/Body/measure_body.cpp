#include "Intrepid2_HGRAD_HEX_Cn_FEM.hpp"
#include <Cogent_IntegratorFactory.hpp>

#include <iostream>
#include <stdlib.h>

#include <Teuchos_ParameterList.hpp>
#include <Teuchos_XMLParameterListHelpers.hpp>

int main() {

  // define HEX8 element topology
  const CellTopologyData& celldata = *shards::getCellTopologyData< shards::Hexahedron<8> >();
  Teuchos::RCP<shards::CellTopology> celltype = Teuchos::rcp(new shards::CellTopology( &celldata ) );

  // define basis
  Teuchos::RCP<Intrepid2::Basis<PHX::Device, RealType> >
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM<PHX::Device, RealType, RealType >() );
  
  // read geometry file and setup
  std::string xmlFileName = "geom_file.xml";
  Teuchos::ParameterList geomSpec;
  Teuchos::updateParametersFromXmlFile(xmlFileName, Teuchos::ptrFromRef(geomSpec));

  // create integrator
  Cogent::IntegratorFactory integratorFactory;
  Teuchos::RCP<Cogent::Integrator> integrator = integratorFactory.create(celltype, intrepidBasis, geomSpec);

  // create element coordinates
  int numNodes = celltype->getVertexCount();
  int numDims = celltype->getDimension();
  Cogent::FContainer<RealType> coordCon("coordCon", numNodes, numDims);
  coordCon(0,0) = -1; coordCon(0,1) = -1; coordCon(0,2) = -1;
  coordCon(1,0) =  1; coordCon(1,1) = -1; coordCon(1,2) = -1;
  coordCon(2,0) =  1; coordCon(2,1) =  1; coordCon(2,2) = -1;
  coordCon(3,0) = -1; coordCon(3,1) =  1; coordCon(3,2) = -1;
  coordCon(4,0) = -1; coordCon(4,1) = -1; coordCon(4,2) =  1;
  coordCon(5,0) =  1; coordCon(5,1) = -1; coordCon(5,2) =  1;
  coordCon(6,0) =  1; coordCon(6,1) =  1; coordCon(6,2) =  1;
  coordCon(7,0) = -1; coordCon(7,1) =  1; coordCon(7,2) =  1;

  // create levelsets
  int numLevelsets = 1;
  Cogent::FContainer<RealType> topoVals("TopoVals",numNodes,numLevelsets);
  topoVals(0,0) = -1.0001;   topoVals(1,0) =  1.0;   topoVals(2,0) =  1.0;   topoVals(3,0) = -1.0001;
  topoVals(4,0) = -1.0001;   topoVals(5,0) =  1.0;   topoVals(6,0) =  1.0;   topoVals(7,0) = -1.0001;

  // compute the volume and derivatives.
  //
  double volume;
  Cogent::FContainer<RealType> dVdtopo;
  integrator->getMeasure(volume, dVdtopo, topoVals, coordCon);
  std::cout << "Volume: " << std::setprecision(10) <<  volume << std::endl;
  std::cout << "Derivatives (using AD): " << std::endl;
  for(int j=0; j<numLevelsets; j++){
    for(int k=0; k<numNodes; k++) 
      std::cout << dVdtopo(k,j) << " ";
    std::cout << std::endl;
  }

  // use finite differencing to compute derivatives.
  //
  integrator->getMeasure(volume, topoVals, coordCon);
  std::cout << "Volume: " << volume << std::endl;

  Cogent::FContainer<RealType> dMdtopoFD("dMdtopoFD",numNodes,numLevelsets);
  Cogent::FContainer<RealType> dtopoVals("dtopoVals",numNodes,numLevelsets);

  for(int j=0; j<numLevelsets; j++)
    for(int k=0; k<numNodes; k++)
     dtopoVals(k,j) = topoVals(k,j);
 
  for(int j=0; j<numLevelsets; j++){
    for(int k=0; k<numNodes; k++){
      double dvolume = 0.0;
      double epsilon = 1e-9;
      dtopoVals(k,j) += epsilon;
      integrator->getMeasure(dvolume, dtopoVals, coordCon);
      dtopoVals(k,j) -= epsilon;
      dMdtopoFD(k,j) = (dvolume-volume)/epsilon;
    }
  }
  std::cout << "Derivatives (using finite differencing): " << std::endl;
  for(int j=0; j<numLevelsets; j++){
    for(int k=0; k<numNodes; k++) 
      std::cout << dMdtopoFD(k,j) << " ";
    std::cout << std::endl;
  }
  std::cout << std::endl;

}
