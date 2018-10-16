#include "Intrepid2_HGRAD_HEX_Cn_FEM.hpp"
#include <Cogent_Integrator.hpp>

#include <iostream>
#include <stdlib.h>

#include <Teuchos_ParameterList.hpp>
#include <Teuchos_XMLParameterListHelpers.hpp>

int main() {

  // define HEX8 element topology
  const CellTopologyData& celldata = *shards::getCellTopologyData< shards::Hexahedron<8> >();
  Teuchos::RCP<shards::CellTopology> celltype = Teuchos::rcp(new shards::CellTopology( &celldata ) );

  // define basis
  typedef Intrepid2::FieldContainer_Kokkos< RealType, PHX::Layout, PHX::Device > FContainer;
  Teuchos::RCP<Intrepid2::Basis<RealType, FContainer > >
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM< RealType, FContainer >() );
  
  // read geometry file and setup
  std::string xmlFileName = "geom_file.xml";
  Teuchos::ParameterList geomSpec;
  Teuchos::updateParametersFromXmlFile(xmlFileName, Teuchos::ptrFromRef(geomSpec));
  geomSpec.set("Maximum Refinements", 1);
  geomSpec.set("Maximum Error", 1.0e-3);
  geomSpec.set("Interface Value", 0.0);

  // create integrator
  Cogent::Integrator integrator(celltype, intrepidBasis, geomSpec);

  geomSpec.set("Projection Order", 2);
  geomSpec.set("Uniform Quadrature", true);
  Cogent::Integrator projector(celltype, intrepidBasis, geomSpec);

  // create element coordinates
  int numNodes = celltype->getVertexCount();
  int numDims = celltype->getDimension();
  FContainer coordCon(numNodes, numDims);
  coordCon(0,0) = -3; coordCon(0,1) = -2; coordCon(0,2) = -2;
  coordCon(1,0) = -1; coordCon(1,1) = -2; coordCon(1,2) = -2;
  coordCon(2,0) = -1; coordCon(2,1) =  0; coordCon(2,2) = -2;
  coordCon(3,0) = -3; coordCon(3,1) =  0; coordCon(3,2) = -2;
  coordCon(4,0) = -3; coordCon(4,1) = -2; coordCon(4,2) =  0;
  coordCon(5,0) = -1; coordCon(5,1) = -2; coordCon(5,2) =  0;
  coordCon(6,0) = -1; coordCon(6,1) =  0; coordCon(6,2) =  0;
  coordCon(7,0) = -3; coordCon(7,1) =  0; coordCon(7,2) =  0;
/*
  coordCon(0,0) = -1; coordCon(0,1) = -1; coordCon(0,2) = -1;
  coordCon(1,0) =  1; coordCon(1,1) = -1; coordCon(1,2) = -1;
  coordCon(2,0) =  1; coordCon(2,1) =  1; coordCon(2,2) = -1;
  coordCon(3,0) = -1; coordCon(3,1) =  1; coordCon(3,2) = -1;
  coordCon(4,0) = -1; coordCon(4,1) = -1; coordCon(4,2) =  1;
  coordCon(5,0) =  1; coordCon(5,1) = -1; coordCon(5,2) =  1;
  coordCon(6,0) =  1; coordCon(6,1) =  1; coordCon(6,2) =  1;
  coordCon(7,0) = -1; coordCon(7,1) =  1; coordCon(7,2) =  1;
*/

  // create levelsets
  int numLevelsets = 1;
  FContainer topoVals(numNodes,numLevelsets);
  topoVals(0,0) = -1.0;   topoVals(1,0) =  1.0;   topoVals(2,0) =  1.0;   topoVals(3,0) = -1.0;
  topoVals(4,0) = -1.0;   topoVals(5,0) =  1.0;   topoVals(6,0) =  1.0;   topoVals(7,0) = -1.0;

/*
  topoVals(0,1) = -1.01;   topoVals(1,1) = -1.01;   topoVals(2,1) =  1.0;   topoVals(3,1) =  1.0;
  topoVals(4,1) = -1.01;   topoVals(5,1) = -1.01;   topoVals(6,1) =  1.0;   topoVals(7,1) =  1.0;

  topoVals(0,2) = -1.01;   topoVals(1,2) = -1.01;   topoVals(2,2) = -1.01;   topoVals(3,2) = -1.01;
  topoVals(4,2) =  1.0;   topoVals(5,2) =  1.0;   topoVals(6,2) =  1.0;   topoVals(7,2) =  1.0;

  topoVals(0,0) = -1.0;   topoVals(1,0) =  1.0;   topoVals(2,0) =  1.0;   topoVals(3,0) = -1.0;
  topoVals(4,0) = -1.0;   topoVals(5,0) =  1.0;   topoVals(6,0) =  1.0;   topoVals(7,0) = -1.0;

  topoVals(0,1) = -1.0;   topoVals(1,1) = -1.0;   topoVals(2,1) =  1.0;   topoVals(3,1) =  1.0;
  topoVals(4,1) = -1.0;   topoVals(5,1) = -1.0;   topoVals(6,1) =  1.0;   topoVals(7,1) =  1.0;

  topoVals(0,2) = -1.0;   topoVals(1,2) = -1.0;   topoVals(2,2) = -1.0;   topoVals(3,2) = -1.0;
  topoVals(4,2) =  1.0;   topoVals(5,2) =  1.0;   topoVals(6,2) =  1.0;   topoVals(7,2) =  1.0;
*/
/*
  topoVals(0,0) = -0.5;   topoVals(1,0) =  1.5;   topoVals(2,0) =  1.5;   topoVals(3,0) = -0.5;
  topoVals(4,0) = -0.5;   topoVals(5,0) =  1.5;   topoVals(6,0) =  1.5;   topoVals(7,0) = -0.5;

  topoVals(0,1) = -0.515; topoVals(1,1) = -0.515; topoVals(2,1) =  1.5;   topoVals(3,1) =  1.5;
  topoVals(4,1) = -0.515; topoVals(5,1) = -0.515; topoVals(6,1) =  1.5;   topoVals(7,1) =  1.5;

  topoVals(0,2) = -0.515; topoVals(1,2) = -0.515; topoVals(2,2) = -0.515; topoVals(3,2) = -0.415;
  topoVals(4,2) =  1.5;   topoVals(5,2) =  1.5;   topoVals(6,2) =  1.5;   topoVals(7,2) =  1.5;
*/

  // compute conformal GPs
  //
  FContainer cweights, cpoints;
  integrator.getCubature(cweights, cpoints, topoVals, coordCon);

  // compute stiffness using conformal GPs
  //
  int ncQPs = cweights.dimension(0);
  FContainer gradAtConformalGPs;
  gradAtConformalGPs.resize(numNodes, ncQPs, numDims);
  intrepidBasis->getValues(gradAtConformalGPs, cpoints, Intrepid2::OPERATOR_GRAD);
  
  FContainer conformalStiffness;
  conformalStiffness.resize(numNodes, numNodes);

  for(int I=0; I<numNodes; I++)
    for(int J=0; J<numNodes; J++){
      conformalStiffness(I,J) = 0.0;
      for(int i=0; i<ncQPs; i++)
        for(int j=0; j<numDims; j++)
          conformalStiffness(I,J) += cweights(i)*gradAtConformalGPs(I,i,j)*gradAtConformalGPs(J,i,j);
    }

  // write to screen
  //
  std::cout << "Conformal Stiffness" << std::endl;
  for(int I=0; I<numNodes; I++){
    for(int J=0; J<numNodes; J++){
      std::cout << conformalStiffness(I,J) << " " ;
    }
    std::cout << std::endl;
  }

  // compute reduced GPs
  //
  FContainer rweights, rpoints, rdwdtopo;
  projector.getCubature(rweights, rdwdtopo, rpoints, topoVals, coordCon);

  // write to screen
  //
  for(int j=0; j<numNodes; j++) std::cout << topoVals(j) << " ";
  std::cout << std::endl;
  int nrQPs = rweights.dimension(0);
  std::cout << "Number of points: " << nrQPs << std::endl;
  double totalWeight = 0.0;
  for(int j=0; j<nrQPs; j++) {
    totalWeight += rweights(j);
    std::cout << "qp " << j << ": " << rweights(j);
    for(int k=0; k<numDims; k++)
      std::cout << " " << rpoints(j,k);
    std::cout << std::endl;
  }
  std::cout << "Total weight: " << totalWeight << std::endl;
  for(int j=0; j<nrQPs; j++) {
    std::cout << "qp " << j << ": " << rweights(j);
    for(int m=0; m<numLevelsets; m++){
      for(int k=0; k<numNodes; k++)
        std::cout << " " << rdwdtopo(j,k,m);
      std::cout << std::endl;
    }
  }
    
  // compute stiffness using standard GP locations w/ computed weights
  //
  FContainer gradAtStandardGPs;
  gradAtStandardGPs.resize(numNodes, nrQPs, numDims);
  intrepidBasis->getValues(gradAtStandardGPs, rpoints, Intrepid2::OPERATOR_GRAD);
  
  FContainer projectedStiffness;
  projectedStiffness.resize(numNodes, numNodes);
  for(int I=0; I<numNodes; I++)
    for(int J=0; J<numNodes; J++){
      projectedStiffness(I,J) = 0.0;
      for(int i=0; i<nrQPs; i++)
        for(int j=0; j<numDims; j++)
          projectedStiffness(I,J) += rweights(i)*gradAtStandardGPs(I,i,j)*gradAtStandardGPs(J,i,j);
    }

  std::cout << "Projected Stiffness" << std::endl;
  for(int I=0; I<numNodes; I++){
    for(int J=0; J<numNodes; J++){
      std::cout << projectedStiffness(I,J) << " " ;
    }
    std::cout << std::endl;
  }
}
