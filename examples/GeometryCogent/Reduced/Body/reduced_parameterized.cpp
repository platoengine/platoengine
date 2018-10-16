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
  std::string xmlFileName = "parameterized.xml";
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
  int numGeomVals = 4;
  Cogent::FContainer<RealType> geomVals("geomVals",numGeomVals), cweights, cpoints;
  geomVals(0) = 2.0;   geomVals(1) =  2.0;   geomVals(2) =  2.0;   geomVals(3) = 0.5;

  // compute conformal GPs
  //
  const Cogent::FContainer<RealType>& constGeomVals = geomVals;
  integrator->getCubature(cweights, cpoints, constGeomVals, coordCon);

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
  Cogent::FContainer<RealType> rweights, rpoints, rdwdgeom;
  projector->getCubature(rweights, rdwdgeom, rpoints, constGeomVals, coordCon);
  
  // write to screen
  //
  int nrQPs = rweights.dimension(0);
  std::vector<RealType> dWeightdGeom(numGeomVals,0.0);
  double totalWeight = 0.0;
  for(int j=0; j<nrQPs; j++){
    totalWeight += rweights(j);
    for(int m=0; m<numGeomVals; m++)
      dWeightdGeom[m] += rdwdgeom(j,m);
  }
  std::cout << "Total weight: " << totalWeight;
  for(int m=0; m<numGeomVals; m++)
    std::cout << ", " << dWeightdGeom[m];
  std::cout << std::endl;

  if(verbose){
    for(int j=0; j<nrQPs; j++) {
      std::cout << "qp " << j << ": " << rweights(j);
    for(int m=0; m<numGeomVals; m++)
        std::cout << " " << rdwdgeom(j,m);
      std::cout << std::endl;
    }
  }

  // verify gp derivatives
  //
  std::cout << "Computing derivatives (using finite differencing): " << std::endl;
  Cogent::FContainer<RealType> dQPdgeomFD("dQPdgeomFD",nrQPs,numGeomVals), dgeomVals(geomVals), drweights;
  const Cogent::FContainer<RealType>& constDgeomVals = dgeomVals;
  projector->getCubature(rweights, rpoints, constDgeomVals, coordCon);
   
  for(int j=0; j<numGeomVals; j++){
    double epsilon = 1e-9;
    dgeomVals(j) += epsilon;
    projector->getCubature(drweights, rpoints, constDgeomVals, coordCon);
    dgeomVals(j) -= epsilon;
    for(int q=0; q<nrQPs; q++){
      dQPdgeomFD(q,j) = (drweights(q)-rweights(q))/epsilon;
    }
  }
  std::vector<RealType> dWeightdGeomFD(numGeomVals,0.0);
  totalWeight = 0.0;
  for(int j=0; j<nrQPs; j++){
    totalWeight += drweights(j);
    for(int m=0; m<numGeomVals; m++)
      dWeightdGeomFD[m] += dQPdgeomFD(j,m);
  }
  std::cout << "Total weight: " << totalWeight;
  for(int m=0; m<numGeomVals; m++)
    std::cout << ", " << dWeightdGeomFD[m];
  std::cout << std::endl;

  if(verbose){
    for(int q=0; q<nrQPs; q++){
      std::cout << "qp " << q << ": " << rweights(q) << " ";
      for(int j=0; j<numGeomVals; j++)
        std::cout << dQPdgeomFD(q,j) << " ";
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }

  RealType qpAbsError = 0.0;
  RealType qpRelError = 0.0;
  for(int q=0; q<nrQPs; q++)
    for(int j=0; j<numGeomVals; j++){
      double err = (dQPdgeomFD(q,j) - rdwdgeom(q,j));
      double den = (dQPdgeomFD(q,j) + rdwdgeom(q,j))/2.0;
      qpAbsError += fabs(err);
      if( den != 0.0 )
        qpRelError += fabs(err/den);
  }
  int nvals = nrQPs*numGeomVals;
  std::cout << "-- QP Derivatives --" << std::endl;
  std::cout << " Average absolute error: " << qpAbsError/nvals << std::endl;
  std::cout << " Average relative error: " << qpRelError/nvals << std::endl;

}
