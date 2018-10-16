#include "Intrepid2_HGRAD_HEX_Cn_FEM.hpp"
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
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM<PHX::Device, RealType, RealType>() );
  
  // read geometry file and setup
  std::string xmlFileName = "geom_file.xml";
  Teuchos::ParameterList geomSpec;
  Teuchos::updateParametersFromXmlFile(xmlFileName, Teuchos::ptrFromRef(geomSpec));

  // create integrator
  Cogent::IntegratorFactory integratorFactory;
  Teuchos::RCP<Cogent::Integrator> integrator = integratorFactory.create(celltype, intrepidBasis, geomSpec);

  // create projector
  geomSpec.set("Projection Order", 2);
  geomSpec.set("Uniform Quadrature", true);
  Teuchos::RCP<Cogent::Integrator> projector = integratorFactory.create(celltype, intrepidBasis, geomSpec);

  bool verbose = false;
  if(geomSpec.isType<bool>("Verbose"))
    verbose = geomSpec.get<bool>("Verbose");

  // create element coordinates
  int numNodes = celltype->getVertexCount();
  int numDims = celltype->getDimension();
  Cogent::FContainer<RealType> coordCon("coordCon", numNodes, numDims);
  coordCon(0,0) = -.3; coordCon(0,1) = -.2; coordCon(0,2) = -.2;
  coordCon(1,0) = -.1; coordCon(1,1) = -.2; coordCon(1,2) = -.2;
  coordCon(2,0) = -.1; coordCon(2,1) =  .0; coordCon(2,2) = -.2;
  coordCon(3,0) = -.3; coordCon(3,1) =  .0; coordCon(3,2) = -.2;
  coordCon(4,0) = -.3; coordCon(4,1) = -.2; coordCon(4,2) =  .0;
  coordCon(5,0) = -.1; coordCon(5,1) = -.2; coordCon(5,2) =  .0;
  coordCon(6,0) = -.1; coordCon(6,1) =  .0; coordCon(6,2) =  .0;
  coordCon(7,0) = -.3; coordCon(7,1) =  .0; coordCon(7,2) =  .0;


  int nSteps = 10;
  std::vector<double> qpAbsError(nSteps), qpRelError(nSteps);

  // create levelsets
  int numLevelsets = 1;
  Cogent::FContainer<RealType> topoVals("topoVals",numNodes,numLevelsets);
  Cogent::FContainer<RealType> cweights, cpoints;

  for(int i=1; i<nSteps; ++i){
    double a = 2.0/nSteps*i;
    std::cout << "a = " << a << std::endl;
    topoVals(0,0) = -2.0+a;   topoVals(1,0) =  0.0+a;   topoVals(2,0) =  0.0+a;   topoVals(3,0) = -2.0+a;
    topoVals(4,0) = -2.0+a;   topoVals(5,0) =  0.0+a;   topoVals(6,0) =  0.0+a;   topoVals(7,0) = -2.0+a;

    // compute conformal GPs
    //
    const Cogent::FContainer<RealType>& constTopoVals = topoVals;
    integrator->getCubature(cweights, cpoints, constTopoVals, coordCon);

    // compute stiffness using conformal GPs
    //
    int ncQPs = cweights.dimension(0);
    double conformalWeight = 0.0;
    for(int q=0; q<ncQPs; ++q) conformalWeight += cweights(q);
    std::cout << "Conformal Weight: " << conformalWeight << std::endl;

    // compute reduced GPs
    //
    Cogent::FContainer<RealType> rweights, rpoints, rdwdtopo;
    projector->getCubature(rweights, rdwdtopo, rpoints, constTopoVals, coordCon);
  
    // write to screen
    //
    int nrQPs = rweights.dimension(0);
    double totalWeight = 0.0;
    for(int j=0; j<nrQPs; j++) totalWeight += rweights(j);
    std::cout << "Projected Weight: " << totalWeight << std::endl;

    std::cout << "Computing derivatives (using AD): " << std::endl;
    if(verbose){
      for(int j=0; j<nrQPs; j++) {
        std::cout << "qp " << j << ": " << rweights(j);
        for(int k=0; k<numNodes; k++)
          std::cout << " " << rdwdtopo(j,k);
        std::cout << std::endl;
      }
    }

    // verify gp derivatives
    //
    std::cout << "Computing derivatives (using finite differencing): " << std::endl;

    Cogent::FContainer<RealType> dtopoVals("dtopoVals",numNodes,numLevelsets);
    const Cogent::FContainer<RealType>& constDtopoVals = dtopoVals;
    for(int j=0; j<numLevelsets; j++)
      for(int k=0; k<numNodes; k++)
        dtopoVals(k,j) = topoVals(k,j);
    projector->getCubature(rweights, rpoints, constDtopoVals, coordCon);
   
    Cogent::FContainer<RealType> drweights;
    Cogent::FContainer<RealType> dQPdtopoFD("dQPdtopoFD",nrQPs,numNodes,numLevelsets);
    for(int j=0; j<numLevelsets; j++){
      for(int k=0; k<numNodes; k++){
        double epsilon = 1e-9;
        dtopoVals(k,j) += epsilon;
        projector->getCubature(drweights, rpoints, constDtopoVals, coordCon);
        dtopoVals(k,j) -= epsilon;
        for(int q=0; q<nrQPs; q++){
          dQPdtopoFD(q,k,j) = (drweights(q)-rweights(q))/epsilon;
        }
      }
    }
    if(verbose){
      for(int q=0; q<nrQPs; q++){
        std::cout << "qp " << q << ": " << rweights(q) << " ";
        for(int j=0; j<numLevelsets; j++){
          for(int k=0; k<numNodes; k++) 
            std::cout << dQPdtopoFD(q,k,j) << " ";
          std::cout << std::endl;
        }
      }
      std::cout << std::endl;
    }

    qpAbsError[i] = 0.0;
    qpRelError[i] = 0.0;
    for(int q=0; q<nrQPs; q++)
      for(int j=0; j<numLevelsets; j++)
        for(int k=0; k<numNodes; k++){
          double err = (dQPdtopoFD(q,k,j) - rdwdtopo(q,k));
          double den = (dQPdtopoFD(q,k,j) + rdwdtopo(q,k))/2.0;
          qpAbsError[i] += fabs(err);
          if( den != 0.0 )
            qpRelError[i] += fabs(err/den);
      }
    int nvals = nrQPs*numLevelsets*numNodes;
    std::cout << "-- QP Derivatives --" << std::endl;
    std::cout << " Average absolute error: " << qpAbsError[i]/nvals << std::endl;
    std::cout << " Average relative error: " << qpRelError[i]/nvals << std::endl;

  }
}
