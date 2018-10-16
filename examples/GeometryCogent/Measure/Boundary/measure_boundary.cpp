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

  int nSteps=10;
  double delta=1.0;
  double d = delta/nSteps;

  double pi = acos(-1.0);
 
  for(int iStep=0; iStep<nSteps; ++iStep){
    double dx = -sin(iStep*d*pi);
    double dy = -iStep*d;
    double dz = -iStep*d;

    // create levelsets
    int numLevelsets = 3;
    Cogent::FContainer<RealType> topoVals("topoVals",numNodes,numLevelsets);
    topoVals(0,0) = -0.5+dx;   topoVals(1,0) =  1.5+dx;   topoVals(2,0) =  1.5+dx;   topoVals(3,0) = -0.5+dx;
    topoVals(4,0) = -0.5+dx;   topoVals(5,0) =  1.5+dx;   topoVals(6,0) =  1.5+dx;   topoVals(7,0) = -0.5+dx;
  
    topoVals(0,1) = -0.5+dy;   topoVals(1,1) = -0.5+dy;   topoVals(2,1) =  1.5+dy;   topoVals(3,1) =  1.5+dy;
    topoVals(4,1) = -0.5+dy;   topoVals(5,1) = -0.5+dy;   topoVals(6,1) =  1.5+dy;   topoVals(7,1) =  1.5+dy;
  
    topoVals(0,2) = -0.5+dz;   topoVals(1,2) = -0.5+dz;   topoVals(2,2) = -0.5+dz;   topoVals(3,2) = -0.5+dz;
    topoVals(4,2) =  1.5+dz;   topoVals(5,2) =  1.5+dz;   topoVals(6,2) =  1.5+dz;   topoVals(7,2) =  1.5+dz;

    std::cout << "iStep: " << iStep << std::endl;

    // compute the area
    //
    double area;
    integrator->getMeasure(area, topoVals, coordCon);
    std::cout << "X intersection: " << -0.5-dx << std::endl;
    std::cout << "Y intersection: " << -0.5-dy << std::endl;
    std::cout << "Z intersection: " << -0.5-dz << std::endl;
    std::cout << "Actual area: " << (1.0-(-0.5-dy))*(1.0-(-0.5-dz)) << std::endl;
    std::cout << "Computed area: " << area << std::endl;

    // compute the derivatives.
    //
    Cogent::FContainer<RealType> dVdtopo;
    integrator->getMeasure(area, dVdtopo, topoVals, coordCon);
    std::cout << "Derivatives (using AD): " << std::endl;
    for(int j=0; j<numLevelsets; j++){
      for(int k=0; k<numNodes; k++) 
        std::cout << dVdtopo(k,j) << " ";
      std::cout << std::endl;
    }

    // use finite differencing to compute derivatives.
    //
    integrator->getMeasure(area, topoVals, coordCon);

    Cogent::FContainer<RealType> dMdtopoFD("dMdtopoFD",numNodes,numLevelsets);
    Cogent::FContainer<RealType> dtopoVals("dtopoVals",numNodes,numLevelsets);

    for(int j=0; j<numLevelsets; j++)
      for(int k=0; k<numNodes; k++)
        dtopoVals(k,j) = topoVals(k,j);
 
    for(int j=0; j<numLevelsets; j++){
      for(int k=0; k<numNodes; k++){
        double dArea = 0.0;
        double epsilon = 0.000001;
        dtopoVals(k,j) += epsilon;
        integrator->getMeasure(dArea, dtopoVals, coordCon);
        dtopoVals(k,j) -= epsilon;
        dMdtopoFD(k,j) = (dArea-area)/epsilon;
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
}
