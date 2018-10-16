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
  std::string xmlFileName = "geom_file_2ls.xml";
  Teuchos::ParameterList geomSpec;
  Teuchos::updateParametersFromXmlFile(xmlFileName, Teuchos::ptrFromRef(geomSpec));

  geomSpec.set("Projection Order", 2);
  geomSpec.set("Uniform Quadrature", true);

  Cogent::IntegratorFactory integratorFactory;
  Teuchos::RCP<Cogent::Integrator> projector = integratorFactory.create(celltype, intrepidBasis, geomSpec);

  // create element coordinates
  int numNodes = celltype->getVertexCount();
  int numDims = celltype->getDimension();
  Container coordCon("coordCon", numNodes, numDims);
  coordCon(0,0) = -3; coordCon(0,1) = -2; coordCon(0,2) = -2;
  coordCon(1,0) = -1; coordCon(1,1) = -2; coordCon(1,2) = -2;
  coordCon(2,0) = -1; coordCon(2,1) =  0; coordCon(2,2) = -2;
  coordCon(3,0) = -3; coordCon(3,1) =  0; coordCon(3,2) = -2;
  coordCon(4,0) = -3; coordCon(4,1) = -2; coordCon(4,2) =  0;
  coordCon(5,0) = -1; coordCon(5,1) = -2; coordCon(5,2) =  0;
  coordCon(6,0) = -1; coordCon(6,1) =  0; coordCon(6,2) =  0;
  coordCon(7,0) = -3; coordCon(7,1) =  0; coordCon(7,2) =  0;

  int nSteps = 10;

  // projection order of 2 produces 27 gauss weights
  int nrQPs = 27;
  Container weights("weights",nSteps+1,nSteps+1,nrQPs);
  Container volumes("volumes",nSteps+1,nSteps+1);

  // create levelsets
  int numLevelsets = 2;
  Container topoVals("topoVals",numNodes,numLevelsets);
  Container rweights, rpoints;

  for(int i=0; i<=nSteps; ++i){
    for(int j=0; j<=nSteps; ++j){
      double a = 2.0/nSteps*i;
      double b = 2.0/nSteps*j;
      std::cout << "a, b = " << a << ", " << b << std::endl;
      topoVals(0,0) = -2.0+a; topoVals(1,0) = 0.0+a; topoVals(2,0) = 0.0+a; topoVals(3,0) = -2.0+a;
      topoVals(4,0) = -2.0+a; topoVals(5,0) = 0.0+a; topoVals(6,0) = 0.0+a; topoVals(7,0) = -2.0+a;

      topoVals(0,1) = -2.0+b; topoVals(1,1) = -2.0+b; topoVals(2,1) = 0.0+b; topoVals(3,1) = 0.0+b;
      topoVals(4,1) = -2.0+b; topoVals(5,1) = -2.0+b; topoVals(6,1) = 0.0+b; topoVals(7,1) = 0.0+b;
    
      // compute reduced GPs
      //
      const Container& constTopoVals = topoVals;
      projector->getCubature(rweights, rpoints, constTopoVals, coordCon);

      // write to container
      //
      volumes(i,j)=0.0;
      for(int q=0; q<nrQPs; q++){
        weights(i,j,q) = rweights(q);
        volumes(i,j) += rweights(q);
      }
    }
  }

  std::ofstream outfile;
  outfile.open("volume.data");
  for(int i=0; i<=nSteps; ++i){
    for(int j=0; j<=nSteps; ++j)
      outfile << volumes(i,j) << " ";
    outfile << std::endl;
  }
  outfile.close();

  outfile.open("volume.gnu");
  outfile << "splot 'volume.data' matrix with lines" << std::endl;
  outfile.close();
  
  
  outfile.open("weights.data");
  for(int i=0; i<=nSteps; ++i){
    for(int j=0; j<=nSteps; ++j){
      outfile << i << " " << j << " ";
      for(int q=0; q<nrQPs; q++)
        outfile << weights(i,j,q) << " ";
      outfile << std::endl;
    }
    outfile << std::endl;
  }
  outfile.close();
  
  outfile.open("weights.gnu");
  outfile << "set key off" << std::endl;
  outfile << "splot \\" << std::endl;
  outfile << "     'weights.data' u 1:2:3 w l";
  for(int i=1; i<nrQPs; ++i)
    outfile << ", \\" << std::endl << "     'weights.data' u 1:2:" << i+3 << " w l";
  outfile.close();
  
}
