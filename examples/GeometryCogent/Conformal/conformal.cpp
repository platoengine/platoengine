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
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM<PHX::Device, RealType, RealType>() );
  
  // read geometry file and setup
  std::string xmlFileName = "geom_file.xml";
  Teuchos::ParameterList geomSpec;
  Teuchos::updateParametersFromXmlFile(xmlFileName, Teuchos::ptrFromRef(geomSpec));
  geomSpec.set("Maximum Refinements", 0);
  geomSpec.set("Maximum Error", 1.0e-3);
  geomSpec.set("Interface Value", 0.0);

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
  int numLevelsets = 3;
  Cogent::FContainer<RealType> topoVals("topoVals",numNodes,numLevelsets);
  topoVals(0,0) = -0.5;   topoVals(1,0) =  1.5;   topoVals(2,0) =  1.5;   topoVals(3,0) = -0.5;
  topoVals(4,0) = -0.5;   topoVals(5,0) =  1.5;   topoVals(6,0) =  1.5;   topoVals(7,0) = -0.5;
  topoVals(0,1) = -0.515; topoVals(1,1) = -0.515; topoVals(2,1) =  1.5;   topoVals(3,1) =  1.5;
  topoVals(4,1) = -0.515; topoVals(5,1) = -0.515; topoVals(6,1) =  1.5;   topoVals(7,1) =  1.5;
  topoVals(0,2) = -0.515; topoVals(1,2) = -0.515; topoVals(2,2) = -0.515; topoVals(3,2) = -0.415;
  topoVals(4,2) =  1.5;   topoVals(5,2) =  1.5;   topoVals(6,2) =  1.5;   topoVals(7,2) =  1.5;

  // compute the weights and points
  //
  Cogent::FContainer<RealType> cweights, cpoints;
  const Cogent::FContainer<RealType>& constTopoVals = topoVals;
  integrator->getCubature(cweights, cpoints, constTopoVals, coordCon);

  // write to screen
  //
  int nQPs = cweights.dimension(0);
  std::cout << "Number of points: " << nQPs << std::endl;
  double totalWeight = 0.0;
  for(int j=0; j<nQPs; j++) {
    totalWeight += cweights(j);
    std::cout << "qp " << j << ": " << cweights(j);
    for(int k=0; k<numDims; k++)
      std::cout << " " << cpoints(j,k);
    std::cout << std::endl;
  }
  std::cout << "Total weight: " << totalWeight << std::endl;
}
