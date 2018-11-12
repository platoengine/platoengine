#include <gtest/gtest.h>

#include <Intrepid2_HGRAD_HEX_Cn_FEM.hpp>
#include "core/Cogent_IntegratorFactory.hpp"

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <Teuchos_ParameterList.hpp>
#include <Teuchos_XMLParameterListHelpers.hpp>

using Cogent::RealType;

static RealType tolerance = 1e-9;

TEST(BodyReducedTest, shapeParameter)
{

  // define HEX8 element topology and basis

  const CellTopologyData& celldata = *shards::getCellTopologyData< shards::Hexahedron<8> >();
  Teuchos::RCP<shards::CellTopology> celltype = Teuchos::rcp(new shards::CellTopology( &celldata ) );

  Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType> >
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM<Kokkos::Serial, RealType, RealType >() );
  

  //-- specify geometry

  Teuchos::RCP<Teuchos::ParameterList> geomSpec = 
    Teuchos::getParametersFromXmlString(
    "<ParameterList name='Geometry Construction'>                                        \n"
    "  <Parameter name='Geometry Type' type='string' value='Body'/>                      \n"
    "  <Parameter name='Model Type' type='string' value='Parameterized'/>                \n"
    "  <Parameter name='Error Checking' type='int' value='5'/>                           \n"
    "  <Parameter name='Number of Subdomains' type='int' value='2'/>                     \n"
    "  <Parameter name='Level Set Names' type='Array(string)' value='{LS00,LS01}'/>      \n"
    "  <Parameter name='Shape Parameters' type='Array(string)' value='{P0,P1,P2,P3}'/>   \n"
    "  <ParameterList name='Subdomain 0'>                                                \n"
    "    <Parameter name='Type' type='string' value='Primitive'/>                        \n"
    "    <ParameterList name='Primitive'>                                                \n"
    "      <Parameter name='Type' type='string' value='Brick'/>                          \n"
    "      <Parameter name='X Dimension' type='string' value='P0'/>                      \n"
    "      <Parameter name='Y Dimension' type='string' value='P1'/>                      \n"
    "      <Parameter name='Z Dimension' type='string' value='P2'/>                      \n"
    "    </ParameterList>                                                                \n"
    "    <Parameter name='Operation' type='string' value='Add'/>                         \n"
    "  </ParameterList>                                                                  \n"
    "  <ParameterList name='Subdomain 1'>                                                \n"
    "    <Parameter name='Type' type='string' value='Primitive'/>                        \n"
    "    <ParameterList name='Primitive'>                                                \n"
    "      <Parameter name='Type' type='string' value='Cylinder'/>                       \n"
    "      <Parameter name='Radius' type='string' value='P3'/>                           \n"
    "      <Parameter name='Height' type='string' value='P2'/>                           \n"
    "      <Parameter name='Axis' type='string' value='Z'/>                              \n"
    "    </ParameterList>                                                                \n"
    "    <Parameter name='Operation' type='string' value='Subtract'/>                    \n"
    "  </ParameterList>                                                                  \n"
    "</ParameterList>                                                                    \n"
    );


  // create integrator

  Cogent::IntegratorFactory integratorFactory;
  Teuchos::RCP<Cogent::Integrator> integrator = integratorFactory.create(celltype, intrepidBasis, *geomSpec);


  // create projector

  geomSpec->set("Projection Order", 2);
  geomSpec->set("Uniform Quadrature", true);
  Teuchos::RCP<Cogent::Integrator> projector = integratorFactory.create(celltype, intrepidBasis, *geomSpec);


  // create element coordinates

  int numNodes = celltype->getVertexCount();
  int numDims = celltype->getDimension();
  Cogent::FContainer<RealType> coordCon("coordCon", numNodes, numDims);
  coordCon(0,0) = -2.0; coordCon(0,1) = -1.0; coordCon(0,2) = -1.0;
  coordCon(1,0) =  0.0; coordCon(1,1) = -1.0; coordCon(1,2) = -1.0;
  coordCon(2,0) =  0.0; coordCon(2,1) =  1.0; coordCon(2,2) = -1.0;
  coordCon(3,0) = -2.0; coordCon(3,1) =  1.0; coordCon(3,2) = -1.0;
  coordCon(4,0) = -2.0; coordCon(4,1) = -1.0; coordCon(4,2) =  1.0;
  coordCon(5,0) =  0.0; coordCon(5,1) = -1.0; coordCon(5,2) =  1.0;
  coordCon(6,0) =  0.0; coordCon(6,1) =  1.0; coordCon(6,2) =  1.0;
  coordCon(7,0) = -2.0; coordCon(7,1) =  1.0; coordCon(7,2) =  1.0;


  // specify shape parameter values

  int numGeomVals = 4;
  Cogent::FContainer<RealType> geomVals("geomVals",numGeomVals), cweights, cpoints, cdwdgeom;
  geomVals(0) = 2.0;    // Brick dim in X
  geomVals(1) = 2.0;    // Brick dim in Y
  geomVals(2) = 2.0;    // Brick dim in Z
  geomVals(3) = 0.5;    // Cylinder radius


  // compute conformal GPs

  const Cogent::FContainer<RealType>& constGeomVals = geomVals;
  integrator->getCubature(cweights, cdwdgeom, cpoints, constGeomVals, coordCon);


  // compute total weight of conformal GPs

  double conformalWeight = 0.0;
  std::vector<RealType> dWeightdGeomConf(numGeomVals,0.0);
  int ncQPs = cweights.dimension(0);
  for(int q=0; q<ncQPs; ++q){
    conformalWeight += cweights(q);
    for(int m=0; m<numGeomVals; m++)
      dWeightdGeomConf[m] += cdwdgeom(q,m);
  }
  std::cout << "Conformal: " << conformalWeight;
  for(int m=0; m<numGeomVals; m++)
    std::cout << ", " << dWeightdGeomConf[m];
  std::cout << std::endl;


  // compute reduced GPs

  Cogent::FContainer<RealType> rweights, rpoints, rdwdgeom;
  projector->getCubature(rweights, rdwdgeom, rpoints, constGeomVals, coordCon);
  

  // compute total weight of projected GPs

  double projectedWeight = 0.0;
  int nrQPs = rweights.dimension(0);
  for(int j=0; j<nrQPs; j++){
    projectedWeight += rweights(j);
  }
  

  // compare conformal vs projected

  EXPECT_NEAR(conformalWeight, projectedWeight, tolerance);


  std::vector<RealType> dWeightdGeom(numGeomVals,0.0);
  for(int j=0; j<nrQPs; j++){
    for(int m=0; m<numGeomVals; m++)
      dWeightdGeom[m] += rdwdgeom(j,m);
  }
  std::cout << "Projected: " << projectedWeight;
  for(int m=0; m<numGeomVals; m++)
    std::cout << ", " << dWeightdGeom[m];
  std::cout << std::endl;


  // verify gp derivatives
  //
  std::cout << "Computing derivatives (using finite differencing): " << std::endl;
  Cogent::FContainer<RealType> dQPdgeomFD("dQPdgeomFD",nrQPs,numGeomVals), dgeomVals(geomVals), drweights;
  const Cogent::FContainer<RealType>& constDgeomVals = dgeomVals;
  integrator->getCubature(rweights, rpoints, constDgeomVals, coordCon);
   
  for(int j=0; j<numGeomVals; j++){
    double epsilon = 1e-6;
    dgeomVals(j) += epsilon;
    integrator->getCubature(drweights, rpoints, constDgeomVals, coordCon);
    dgeomVals(j) -= epsilon;
    for(int q=0; q<nrQPs; q++){
      dQPdgeomFD(q,j) = (drweights(q)-rweights(q))/epsilon;
    }
  }
  std::vector<RealType> dWeightdGeomFD(numGeomVals,0.0);
  RealType totalWeight = 0.0;
  for(int j=0; j<nrQPs; j++){
    totalWeight += drweights(j);
    for(int m=0; m<numGeomVals; m++)
      dWeightdGeomFD[m] += dQPdgeomFD(j,m);
  }
  std::cout << "Total weight: " << totalWeight;
  for(int m=0; m<numGeomVals; m++)
    std::cout << ", " << dWeightdGeomFD[m];
  std::cout << std::endl;


  RealType qpAbsError = 0.0;
  RealType qpRelError = 0.0;
  for(int q=0; q<nrQPs; q++)
    for(int j=0; j<numGeomVals; j++){
      std::cout << "FD: " << dQPdgeomFD(q,j) << ", AD: " << rdwdgeom(q,j) << std::endl;
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
