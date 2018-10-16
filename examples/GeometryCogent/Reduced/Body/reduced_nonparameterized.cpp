#include <Intrepid2_HGRAD_HEX_Cn_FEM.hpp>
#include <Cogent_IntegratorFactory.hpp>

#include <iostream>
#include <fstream>
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
  std::string xmlFileName = "nonparameterized.xml";
  Teuchos::ParameterList geomSpec;
  Teuchos::updateParametersFromXmlFile(xmlFileName, Teuchos::ptrFromRef(geomSpec));

  // create integrator
  Cogent::IntegratorFactory integratorFactory;
  Teuchos::RCP<Cogent::Integrator> integrator = integratorFactory.create(celltype, intrepidBasis, geomSpec);

  // create projector
  geomSpec.set("Projection Order", 2);
  geomSpec.set("Uniform Quadrature", true);
  Teuchos::RCP<Cogent::Integrator> projector = integratorFactory.create(celltype, intrepidBasis, geomSpec);

  bool verbose = true;
  if(geomSpec.isType<bool>("Verbose"))
    verbose = geomSpec.get<bool>("Verbose");

  // create element coordinates
  int numNodes = celltype->getVertexCount();
  int numDims = celltype->getDimension();
  Cogent::FContainer<RealType> coordCon("coordCon", numNodes, numDims);
  coordCon(0,0) = -1.1; coordCon(0,1) = -0.1; coordCon(0,2) = -0.1;
  coordCon(1,0) = -0.9; coordCon(1,1) = -0.1; coordCon(1,2) = -0.1;
  coordCon(2,0) = -0.9; coordCon(2,1) =  0.1; coordCon(2,2) = -0.1;
  coordCon(3,0) = -1.1; coordCon(3,1) =  0.1; coordCon(3,2) = -0.1;
  coordCon(4,0) = -1.1; coordCon(4,1) = -0.1; coordCon(4,2) =  0.1;
  coordCon(5,0) = -0.9; coordCon(5,1) = -0.1; coordCon(5,2) =  0.1;
  coordCon(6,0) = -0.9; coordCon(6,1) =  0.1; coordCon(6,2) =  0.1;
  coordCon(7,0) = -1.1; coordCon(7,1) =  0.1; coordCon(7,2) =  0.1;

  // create levelsets
  const int numLevelsets = 1;
  Cogent::FContainer<RealType> topoVals("topoVals",numNodes,numLevelsets), cweights, cpoints;
  topoVals(0,0) =-0.1;   topoVals(1,0) = 0.1;   topoVals(2,0) = 0.1;   topoVals(3,0) =-0.1;
  topoVals(4,0) =-0.1;   topoVals(5,0) = 0.1;   topoVals(6,0) = 0.1;   topoVals(7,0) =-0.1;

  // compute conformal GPs
  //
  const Cogent::FContainer<RealType>& constTopoVals = topoVals;
  integrator->getCubature(cweights, cpoints, constTopoVals, coordCon);

  // compute stiffness using conformal GPs
  //
  int ncQPs = cweights.dimension(0);
  double conformalWeight = 0.0;
  for(int q=0; q<ncQPs; ++q){
    conformalWeight += cweights(q);
    if(verbose){
      std::cout << "cQP " << q << ": " << cweights(q) << ", " 
                << cpoints(q,0) << " " << cpoints(q,1) << " " << cpoints(q,2) << std::endl;
    }
  }
  std::cout << "Conformal Weight: " << conformalWeight << std::endl;


  // compute reduced GPs
  //
  Cogent::FContainer<RealType> rweights, rpoints, rdwdtopo;
  projector->getCubature(rweights, rdwdtopo, rpoints, constTopoVals, coordCon);
  
  // write to screen
  //
  int nrQPs = rweights.dimension(0);
  int numTopoVals = numNodes*numLevelsets;
  std::vector<RealType> dWeightdTopo(numTopoVals,0.0);
  double totalWeight = 0.0;
  for(int j=0; j<nrQPs; j++){
    totalWeight += rweights(j);
    for(int m=0; m<numNodes; m++)
      for(int n=0; n<numLevelsets; n++)
        dWeightdTopo[n*numNodes+m] += rdwdtopo(j,m,n);
  }
  std::cout << "Total weight: " << totalWeight;
  for(int m=0; m<numTopoVals; m++)
    std::cout << ", " << dWeightdTopo[m];
  std::cout << std::endl;

  if(verbose){
    for(int j=0; j<nrQPs; j++) {
      std::cout << "qp " << j << ": " << rweights(j);
    for(int m=0; m<numNodes; m++)
      for(int n=0; n<numLevelsets; n++)
        std::cout << " " << rdwdtopo(j,m,n);
      std::cout << std::endl;
    }
  }

  // verify gp derivatives
  //
  std::cout << "Computing derivatives (using finite differencing): " << std::endl;
  Cogent::FContainer<RealType> dQPdtopoFD("dQPdtopoFD",nrQPs,numTopoVals), dtopoVals(topoVals), drweights;
  const Cogent::FContainer<RealType>& constDtopoVals = dtopoVals;
  projector->getCubature(rweights, rpoints, constDtopoVals, coordCon);
   
  for(int m=0; m<numNodes; m++)
    for(int n=0; n<numLevelsets; n++){
      double epsilon = 1e-9;
      dtopoVals(m,n) += epsilon;
      projector->getCubature(drweights, rpoints, constDtopoVals, coordCon);
      dtopoVals(m,n) -= epsilon;
      for(int q=0; q<nrQPs; q++){
        dQPdtopoFD(q,n*numNodes+m) = (drweights(q)-rweights(q))/epsilon;
      }
  }
  std::vector<RealType> dWeightdtopoFD(numTopoVals,0.0);
  totalWeight = 0.0;
  for(int j=0; j<nrQPs; j++){
    totalWeight += drweights(j);
    for(int m=0; m<numTopoVals; m++)
      dWeightdtopoFD[m] += dQPdtopoFD(j,m);
  }
  std::cout << "Total weight: " << totalWeight;
  for(int m=0; m<numTopoVals; m++)
    std::cout << ", " << dWeightdtopoFD[m];
  std::cout << std::endl;

  if(verbose){
    for(int q=0; q<nrQPs; q++){
      std::cout << "qp " << q << ": " << rweights(q) << " ";
      for(int j=0; j<numTopoVals; j++)
        std::cout << dQPdtopoFD(q,j) << " ";
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }

  RealType qpAbsError = 0.0;
  RealType qpRelError = 0.0;
  for(int q=0; q<nrQPs; q++)
    for(int j=0; j<numNodes; j++)
      for(int j=0; j<numLevelsets; j++){
        double err = (dQPdtopoFD(q,j) - rdwdtopo(q,j,0));
        double den = (dQPdtopoFD(q,j) + rdwdtopo(q,j,0))/2.0;
        qpAbsError += fabs(err);
        if( den != 0.0 )
          qpRelError += fabs(err/den);
  }
  int nvals = nrQPs*numTopoVals;
  std::cout << "-- QP Derivatives --" << std::endl;
  std::cout << " Average absolute error: " << qpAbsError/nvals << std::endl;
  std::cout << " Average relative error: " << qpRelError/nvals << std::endl;

}
