#include "Intrepid2_HGRAD_HEX_Cn_FEM.hpp"
#include <Cogent_IntegratorFactory.hpp>

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <Teuchos_ParameterList.hpp>
#include <Teuchos_XMLParameterListHelpers.hpp>

int main() {

  typedef Cogent::FContainer<RealType> Container;

  // define HEX8 element topology
  const CellTopologyData& celldata = *shards::getCellTopologyData< shards::Hexahedron<8> >();
  Teuchos::RCP<shards::CellTopology> celltype = Teuchos::rcp(new shards::CellTopology( &celldata ) );

  // define basis
  Teuchos::RCP<Intrepid2::Basis<PHX::Device, RealType > >
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM<PHX::Device, RealType, RealType >() );
  
  // read geometry file and setup
  std::string xmlFileName = "harness_file.xml";
  Teuchos::ParameterList geomSpec;
  Teuchos::updateParametersFromXmlFile(xmlFileName, Teuchos::ptrFromRef(geomSpec));

  geomSpec.set("Projection Order", 2);
  geomSpec.set("Uniform Quadrature", true);

  // create projector
  Cogent::IntegratorFactory integratorFactory;
  Teuchos::RCP<Cogent::Integrator> projector = integratorFactory.create(celltype, intrepidBasis, geomSpec);

  // create element coordinates
  int numNodes = celltype->getVertexCount();
  int numDims = celltype->getDimension();
  Container coordCon("coordCon", numNodes, numDims);

  coordCon(0,0) = 126.976744; coordCon(0,1) = -144.000000; coordCon(0,2) = -83.571428;
  coordCon(1,0) = 136.046511; coordCon(1,1) = -144.000000; coordCon(1,2) = -83.571428;
  coordCon(2,0) = 136.571428; coordCon(2,1) = -135.000000; coordCon(2,2) = -83.571428;
  coordCon(3,0) = 126.976744; coordCon(3,1) = -135.000000; coordCon(3,2) = -83.571428;
  coordCon(4,0) = 126.976744; coordCon(4,1) = -144.000000; coordCon(4,2) = -74.285714;
  coordCon(5,0) = 136.046511; coordCon(5,1) = -144.000000; coordCon(5,2) = -74.285714;
  coordCon(6,0) = 136.571428; coordCon(6,1) = -135.000000; coordCon(6,2) = -74.285714;
  coordCon(7,0) = 126.976744; coordCon(7,1) = -135.000000; coordCon(7,2) = -74.285714;

  // create levelsets
  int numLevelsets = 2;
  Container topoVals("topoVals",numNodes,numLevelsets), cweights, cpoints;

  topoVals(0,0) = -12.037259; topoVals(1,0) = -5.623966; topoVals(2,0) =  0.739996; topoVals(3,0) = -5.673299; 
  topoVals(4,0) = -12.037259; topoVals(5,0) = -5.623966; topoVals(6,0) =  0.739996; topoVals(7,0) = -5.673299;

  topoVals(0,1) =  -1.987222; topoVals(1,1) = -8.102633; topoVals(2,1) = -1.660255; topoVals(3,1) =  4.667613; 
  topoVals(4,1) =  -1.987222; topoVals(5,1) = -8.102633; topoVals(6,1) = -1.660255; topoVals(7,1) =  4.667613;

  // compute reduced GPs
  //
  Container rweights, rpoints;
  const Container& constTopoVals = topoVals;
  projector->getCubature(rweights, rpoints, constTopoVals, coordCon);
  
  // write to screen
  //
  int nrQPs = rweights.dimension(0);
  double totalWeight = 0.0;
  for(int j=0; j<nrQPs; j++) totalWeight += rweights(j);
  std::cout << "Total weight: " << totalWeight << std::endl;

}
