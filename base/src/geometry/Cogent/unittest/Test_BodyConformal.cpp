#include <gtest/gtest.h>

#include <Intrepid2_HGRAD_HEX_Cn_FEM.hpp>
#include "core/Cogent_IntegratorFactory.hpp"

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <Teuchos_ParameterList.hpp>
#include <Teuchos_XMLParameterListHelpers.hpp>

using Cogent::RealType;

static RealType tolerance = 1e-6;

TEST(BodyConformalTest, Parameterized)
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


  // compare conformal vs actual

  EXPECT_NEAR(conformalWeight, 4.0, tolerance);
}

TEST(BodyConformalTest, NonParameterized_Value)
{

  // define HEX8 element topology
  const CellTopologyData& celldata = *shards::getCellTopologyData< shards::Hexahedron<8> >();
  Teuchos::RCP<shards::CellTopology> celltype = Teuchos::rcp(new shards::CellTopology( &celldata ) );

  // define basis
  Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType> >
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM<Kokkos::Serial, RealType, RealType>() );
  
  //-- specify geometry

  Teuchos::RCP<Teuchos::ParameterList> geomSpec = 
    Teuchos::getParametersFromXmlString(
    "<ParameterList name='Geometry Construction'>                                        \n"
    "  <Parameter name='Model Type' type='string' value='Non-parameterized'/>            \n"
    "  <Parameter name='Geometry Type' type='string' value='Body'/>                      \n"
    "  <Parameter name='Number of Subdomains' type='int' value='1'/>                     \n"
    "  <Parameter name='Level Set Names' type='Array(string)' value='{LS0}'/>            \n"
    "  <Parameter name='Interface Value' type='double' value='0.0'/>                     \n"
    "  <Parameter name='Maximum Error' type='double' value='1.0e-3'/>                    \n"
    "  <Parameter name='Maximum Refinements' type='int' value='0'/>                      \n"
    "  <ParameterList name='Subdomain 0'>                                                \n"
    "    <Parameter name='Operation' type='string' value='Add'/>                         \n"
    "    <Parameter name='Level Set Indices' type='Array(int)' value='{0}'/>             \n"
    "  </ParameterList>                                                                  \n"
    "</ParameterList>                                                                    \n"
    );

  // create integrator
  Cogent::IntegratorFactory integratorFactory;
  Teuchos::RCP<Cogent::Integrator> integrator = integratorFactory.create(celltype, intrepidBasis, *geomSpec);

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
  Cogent::FContainer<RealType> topoVals("topoVals",numNodes,numLevelsets);
  topoVals(0,0) = -0.5;   topoVals(1,0) =  1.5;   topoVals(2,0) =  1.5;   topoVals(3,0) = -0.5;
  topoVals(4,0) = -0.5;   topoVals(5,0) =  1.5;   topoVals(6,0) =  1.5;   topoVals(7,0) = -0.5;

  // compute the weights and points
  //
  Cogent::FContainer<RealType> cweights, cpoints;
  const Cogent::FContainer<RealType>& constTopoVals = topoVals;
  integrator->getCubature(cweights, cpoints, constTopoVals, coordCon);

  // write to screen
  //
  int nQPs = cweights.dimension(0);
  double totalWeight = 0.0;
  for(int j=0; j<nQPs; j++) {
    totalWeight += cweights(j);
  }
  std::cout << "Total weight: " << totalWeight << std::endl;

  EXPECT_NEAR(totalWeight, 6.0, tolerance);

}

TEST(BodyConformalTest, NonParameterized_Deriv_SingleLS)
{

  // define HEX8 element topology
  const CellTopologyData& celldata = *shards::getCellTopologyData< shards::Hexahedron<8> >();
  Teuchos::RCP<shards::CellTopology> celltype = Teuchos::rcp(new shards::CellTopology( &celldata ) );

  // define basis
  Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType> >
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM<Kokkos::Serial, RealType, RealType>() );
  
  //-- specify geometry

  Teuchos::RCP<Teuchos::ParameterList> geomSpec = 
    Teuchos::getParametersFromXmlString(
    "<ParameterList name='Geometry Construction'>                                        \n"
    "  <Parameter name='Model Type' type='string' value='Non-parameterized'/>            \n"
    "  <Parameter name='Geometry Type' type='string' value='Body'/>                      \n"
    "  <Parameter name='Number of Subdomains' type='int' value='1'/>                     \n"
    "  <Parameter name='Level Set Names' type='Array(string)' value='{LS0}'/>            \n"
    "  <Parameter name='Interface Value' type='double' value='0.0'/>                     \n"
    "  <Parameter name='Maximum Error' type='double' value='1.0e-3'/>                    \n"
    "  <Parameter name='Maximum Refinements' type='int' value='0'/>                      \n"
    "  <ParameterList name='Subdomain 0'>                                                \n"
    "    <Parameter name='Operation' type='string' value='Add'/>                         \n"
    "    <Parameter name='Level Set Indices' type='Array(int)' value='{0}'/>             \n"
    "  </ParameterList>                                                                  \n"
    "</ParameterList>                                                                    \n"
    );

  // create integrator
  Cogent::IntegratorFactory integratorFactory;
  Teuchos::RCP<Cogent::Integrator> integrator = integratorFactory.create(celltype, intrepidBasis, *geomSpec);

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
  Cogent::FContainer<RealType> topoVals("topoVals",numNodes,numLevelsets);
  topoVals(0,0) = -0.5;   topoVals(1,0) =  1.5;   topoVals(2,0) =  1.5;   topoVals(3,0) = -0.5;
  topoVals(4,0) = -0.5;   topoVals(5,0) =  1.5;   topoVals(6,0) =  1.5;   topoVals(7,0) = -0.5;

  // compute the weights and points
  //
  Cogent::FContainer<RealType> cweights, cdwdgeom, cpoints;
  const Cogent::FContainer<RealType>& constTopoVals = topoVals;
  integrator->getCubature(cweights, cdwdgeom, cpoints, constTopoVals, coordCon);

  // write to screen
  //
  int nQPs = cweights.dimension(0);
  double totalWeight = 0.0;
  for(int j=0; j<nQPs; j++) {
    totalWeight += cweights(j);
  }
  std::cout << "Total weight: " << totalWeight << std::endl;

  int nDerivs = cdwdgeom.dimension(1);
  std::vector<double> dwdgeom(nDerivs,0.0);
  for(int j=0; j<nQPs; j++) {
    for(int k=0; k<nDerivs; k++)
      dwdgeom[k] += cdwdgeom(j,k);
  }
  std::cout << "   AD Derivative: ";
  for(int k=0; k<nDerivs; k++)
    std::cout << std::setw(8) << dwdgeom[k] << " ";
  std::cout << std::endl;

  // verify gp derivatives
  //
  std::vector<double> fd_dwdgeom(nDerivs,0.0);
  Cogent::FContainer<RealType> dtopoVals(topoVals), dcweights;
  const Cogent::FContainer<RealType> constDtopoVals = dtopoVals;

  for(int j=0; j<numLevelsets; j++){
    for(int k=0; k<numNodes; k++){
      double epsilon = 1e-6;
      dtopoVals(k,j) += epsilon;
      integrator->getCubature(dcweights, cpoints, constDtopoVals, coordCon);
      dtopoVals(k,j) -= epsilon;
      int nQPs = dcweights.dimension(0);
      double dtotalWeight = 0.0;
      for(int q=0; q<nQPs; q++) {
        dtotalWeight += dcweights(q);
      }
      fd_dwdgeom[j*numNodes+k] = (dtotalWeight - totalWeight)/epsilon;
    }
  }
  std::cout << "   FD Derivative: ";
  for(int k=0; k<nDerivs; k++)
    std::cout << std::setw(8) << fd_dwdgeom[k] << " ";
  std::cout << std::endl;

  /*
    The values below, 'correctValue' and 'correctDerivs', were computed in Mathematica using:
      n[x_, y_,z_] := 
       {{1/8 (1 - x) (1 - y) (1 - z), 1/8 (1 + x) (1 - y) (1 - z), 
         1/8 (1 + x) (1 + y) (1 - z), 1/8 (1 - x) (1 + y) (1 - z), 
         1/8 (1 - x) (1 - y) (1 + z), 1/8 (1 + x) (1 - y) (1 + z), 
         1/8 (1 + x) (1 + y) (1 + z), 1/8 (1 - x) (1 + y) (1 + z)}};

      p[p1_, p2_, p3_, p4_, p5_, p6_, p7_, p8_, x_, y_, z_] := 
       (n[x, y,z].Transpose[{{p1, p2, p3, p4, p5, p6, p7, p8}}])[[1, 1]];

      correctValue = 
       Integrate[ HeavisideTheta[ p[-1/2, 3/2, 3/2, -1/2, -1/2, 3/2, 3/2, -1/2, x, y, z]], 
                  {x, -1, 1}, {y, -1, 1}, {z, -1, 1}]

      correctDerivs = {
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p1], {y, -1, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p2], {y, -1, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p3], {y, -1, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p4], {y, -1, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p5], {y, -1, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p6], {y, -1, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p7], {y, -1, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p8], {y, -1, 1}, {z, -1, 1}]
      }
 
  */
  double correctValue = 6;

  EXPECT_NEAR(totalWeight, correctValue, tolerance);

  std::vector<double> correctDerivs = {0.75,0.25,0.25,0.75,0.75,0.25,0.25,0.75};
  
  std::cout << "Exact Derivative: ";
  for(int k=0; k<nDerivs; k++)
    std::cout << std::setw(8) << correctDerivs[k] << " ";
  std::cout << std::endl;

  for(int i=0; i<nDerivs; i++)
    EXPECT_NEAR(dwdgeom[i], correctDerivs[i], tolerance);

}

TEST(BodyConformalTest, NonParameterized_Deriv_TwoLS)
{

  // define HEX8 element topology
  const CellTopologyData& celldata = *shards::getCellTopologyData< shards::Hexahedron<8> >();
  Teuchos::RCP<shards::CellTopology> celltype = Teuchos::rcp(new shards::CellTopology( &celldata ) );

  // define basis
  Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, RealType> >
    intrepidBasis = Teuchos::rcp(new Intrepid2::Basis_HGRAD_HEX_C1_FEM<Kokkos::Serial, RealType, RealType>() );
  
  //-- specify geometry

  Teuchos::RCP<Teuchos::ParameterList> geomSpec = 
    Teuchos::getParametersFromXmlString(
    "<ParameterList name='Geometry Construction'>                                        \n"
    "  <Parameter name='Model Type' type='string' value='Non-parameterized'/>            \n"
    "  <Parameter name='Geometry Type' type='string' value='Body'/>                      \n"
    "  <Parameter name='Number of Subdomains' type='int' value='1'/>                     \n"
    "  <Parameter name='Level Set Names' type='Array(string)' value='{LS0,LS1}'/>        \n"
    "  <Parameter name='Interface Value' type='double' value='0.0'/>                     \n"
    "  <Parameter name='Maximum Error' type='double' value='1.0e-3'/>                    \n"
    "  <Parameter name='Maximum Refinements' type='int' value='0'/>                      \n"
    "  <ParameterList name='Subdomain 0'>                                                \n"
    "    <Parameter name='Operation' type='string' value='Add'/>                         \n"
    "    <Parameter name='Level Set Indices' type='Array(int)' value='{0,1}'/>           \n"
    "  </ParameterList>                                                                  \n"
    "</ParameterList>                                                                    \n"
    );

//    "  <Parameter name='Verbosity' type='int' value='5'/>                                \n"

  // create integrator
  Cogent::IntegratorFactory integratorFactory;
  Teuchos::RCP<Cogent::Integrator> integrator = integratorFactory.create(celltype, intrepidBasis, *geomSpec);

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
  int numLevelsets = 2;
  Cogent::FContainer<RealType> topoVals("topoVals",numNodes,numLevelsets);
  topoVals(0,0) = -0.5;   topoVals(1,0) =  1.5;   topoVals(2,0) =  1.5;   topoVals(3,0) = -0.5;
  topoVals(4,0) = -0.5;   topoVals(5,0) =  1.5;   topoVals(6,0) =  1.5;   topoVals(7,0) = -0.5;
  topoVals(0,1) =  1.5;   topoVals(1,1) =  1.5;   topoVals(2,1) = -0.5;   topoVals(3,1) = -0.5;
  topoVals(4,1) =  1.5;   topoVals(5,1) =  1.5;   topoVals(6,1) = -0.5;   topoVals(7,1) = -0.5;

  // compute the weights and points
  //
  Cogent::FContainer<RealType> cweights, cdwdgeom, cpoints;
  const Cogent::FContainer<RealType>& constTopoVals = topoVals;
  integrator->getCubature(cweights, cdwdgeom, cpoints, constTopoVals, coordCon);

  // write to screen
  //
  int nQPs = cweights.dimension(0);
  double totalWeight = 0.0;
  for(int j=0; j<nQPs; j++) {
    totalWeight += cweights(j);
  }
  std::cout << "Total weight: " << totalWeight << std::endl;

  int nDerivs = cdwdgeom.dimension(1);
  std::vector<double> dwdgeom(nDerivs,0.0);
  for(int j=0; j<nQPs; j++) {
    for(int k=0; k<nDerivs; k++)
      dwdgeom[k] += cdwdgeom(j,k);
  }
  std::cout << "   AD Derivative: ";
  for(int k=0; k<nDerivs; k++)
    std::cout << std::setw(10) << dwdgeom[k] << " ";
  std::cout << std::endl;

  // verify gp derivatives
  //
  std::vector<double> fd_dwdgeom(nDerivs,0.0);
  Cogent::FContainer<RealType> dtopoVals(topoVals), dcweights;
  const Cogent::FContainer<RealType> constDtopoVals = dtopoVals;

  for(int j=0; j<numLevelsets; j++){
    for(int k=0; k<numNodes; k++){
      double epsilon = 1e-4;
      dtopoVals(k,j) += epsilon;
      integrator->getCubature(dcweights, cpoints, constDtopoVals, coordCon);
      dtopoVals(k,j) -= epsilon;
      int nQPs = dcweights.dimension(0);
      double dtotalWeight = 0.0;
      for(int q=0; q<nQPs; q++) {
        dtotalWeight += dcweights(q);
      }
      fd_dwdgeom[j*numNodes+k] = (dtotalWeight - totalWeight)/epsilon;
    }
  }
  std::cout << "   FD Derivative: ";
  for(int k=0; k<nDerivs; k++)
    std::cout << std::setw(10) << fd_dwdgeom[k] << " ";
  std::cout << std::endl;

  /*
    The values below, 'correctValue' and 'correctDerivs', were computed in Mathematica using:
      n[x_, y_,z_] := 
       {{1/8 (1 - x) (1 - y) (1 - z), 1/8 (1 + x) (1 - y) (1 - z), 
         1/8 (1 + x) (1 + y) (1 - z), 1/8 (1 - x) (1 + y) (1 - z), 
         1/8 (1 - x) (1 - y) (1 + z), 1/8 (1 + x) (1 - y) (1 + z), 
         1/8 (1 + x) (1 + y) (1 + z), 1/8 (1 - x) (1 + y) (1 + z)}};

      p[p1_, p2_, p3_, p4_, p5_, p6_, p7_, p8_, x_, y_, z_] := 
       (n[x, y,z].Transpose[{{p1, p2, p3, p4, p5, p6, p7, p8}}])[[1, 1]];

      correctValue = 
       Integrate[HeavisideTheta[ p[-1/2, 3/2, 3/2, -1/2, -1/2, 3/2, 3/2, -1/2, x, y, z]]*
                 HeavisideTheta[ p[3/2, 3/2, -1/2, -1/2, 3/2, 3/2, -1/2, -1/2, x, y, z]], 
                 {x, -1, 1}, {y, -1, 1}, {z, -1, 1}]

      correctDerivs = {
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p1], {y, -1, 1/2}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p2], {y, -1, 1/2}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p3], {y, -1, 1/2}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p4], {y, -1, 1/2}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p5], {y, -1, 1/2}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p6], {y, -1, 1/2}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p7], {y, -1, 1/2}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, -1/2, y, z], p8], {y, -1, 1/2}, {z, -1, 1}], 
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, x, 1/2, z], p1], {x, -1/2, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, x, 1/2, z], p2], {x, -1/2, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, x, 1/2, z], p3], {x, -1/2, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, x, 1/2, z], p4], {x, -1/2, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, x, 1/2, z], p5], {x, -1/2, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, x, 1/2, z], p6], {x, -1/2, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, x, 1/2, z], p7], {x, -1/2, 1}, {z, -1, 1}],
       Integrate[ D[p[p1, p2, p3, p4, p5, p6, p7, p8, x, 1/2, z], p8], {x, -1/2, 1}, {z, -1, 1}]
      }
 
  */
  double correctValue = 9.0/2.0;


  std::vector<double> correctDerivs = {
   45.0/64.0, 15.0/64.0, 9.0/64.0, 27.0/64.0, 45.0/64.0, 15.0/64.0, 9.0/64.0, 27.0/64.0,
   9.0/64.0, 15.0/64.0, 45.0/64.0, 27.0/64.0, 9.0/64.0, 15.0/64.0, 45.0/64.0, 27.0/64.0};
  
  std::cout << "Exact Derivative: ";
  for(int k=0; k<nDerivs; k++)
    std::cout << std::setw(10) << correctDerivs[k] << " ";
  std::cout << std::endl;

  EXPECT_NEAR(totalWeight, correctValue, tolerance);

  for(int i=0; i<nDerivs; i++)
    EXPECT_NEAR(dwdgeom[i], correctDerivs[i], tolerance);

}
