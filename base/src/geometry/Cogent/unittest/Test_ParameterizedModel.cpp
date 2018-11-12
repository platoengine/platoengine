#include <gtest/gtest.h>

#include <Teuchos_ParameterList.hpp>
#include <Teuchos_XMLParameterListHelpers.hpp>

#include "core/Cogent_Types.hpp"
#include "core/Cogent_ParameterizedModel.hpp"

using Cogent::RealType;

static RealType tolerance = 1e-9;

/******************************************************************************
*  Test: TBD
*  Purpose:  TBD
*******************************************************************************/
TEST(ParameterizedModel, mapToGeometryData)
{


  // define a model 
  // add Brick { x: P0, y: P1, z: P2 }
  // subtract cylinder { radius: P3, height: P2 }

  Teuchos::RCP<Teuchos::ParameterList> params =
    Teuchos::getParametersFromXmlString(
    "<ParameterList name='Geometry Construction'>                                        \n"
    "  <Parameter name='Geometry Type' type='string' value='Body'/>                      \n"
    "  <Parameter name='Number of Subdomains' type='int' value='2'/>                     \n"
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

  Teuchos::RCP<Cogent::Model> model = Teuchos::rcp(new Cogent::ParameterizedModel(*params));


  // set parameter values 

  const int numGeomVals = 4;
  Cogent::FContainer<RealType> geomVals("geomVals",numGeomVals);
  geomVals(0) = 2.0;    // Brick dim in X
  geomVals(1) = 2.0;    // Brick dim in Y
  geomVals(2) = 2.0;    // Brick dim in Z
  geomVals(3) = 0.5;    // Cylinder radius
  model->setModelData(geomVals);


  // define an element and set in model

  const int numNodes = 8;
  const int numDims = 3;
  Cogent::FContainer<RealType> coordCon("coordCon", numNodes, numDims);
  coordCon(0,0) = -1.1; coordCon(0,1) = -0.1; coordCon(0,2) = -0.1;
  coordCon(1,0) = -0.9; coordCon(1,1) = -0.1; coordCon(1,2) = -0.1;
  coordCon(2,0) = -0.9; coordCon(2,1) =  0.1; coordCon(2,2) = -0.1;
  coordCon(3,0) = -1.1; coordCon(3,1) =  0.1; coordCon(3,2) = -0.1;
  coordCon(4,0) = -1.1; coordCon(4,1) = -0.1; coordCon(4,2) =  0.1;
  coordCon(5,0) = -0.9; coordCon(5,1) = -0.1; coordCon(5,2) =  0.1;
  coordCon(6,0) = -0.9; coordCon(6,1) =  0.1; coordCon(6,2) =  0.1;
  coordCon(7,0) = -1.1; coordCon(7,1) =  0.1; coordCon(7,2) =  0.1;
  model->setElementData(coordCon);

  
  // get topo data and check

  Cogent::FContainer<RealType> topoData;
  model->getSignedDistance(topoData);

  EXPECT_EQ(topoData.dimension(1), (unsigned int)1);
  EXPECT_NEAR(topoData(0,0), -0.1, tolerance);
  EXPECT_NEAR(topoData(0,1),  0.1, tolerance);
  EXPECT_NEAR(topoData(0,2),  0.1, tolerance);
  EXPECT_NEAR(topoData(0,3), -0.1, tolerance);
  EXPECT_NEAR(topoData(0,4), -0.1, tolerance);
  EXPECT_NEAR(topoData(0,5),  0.1, tolerance);
  EXPECT_NEAR(topoData(0,6),  0.1, tolerance);
  EXPECT_NEAR(topoData(0,7), -0.1, tolerance);

  
  //
  
  const int numMeas = 1;
  Cogent::FContainer<RealType> dMdT("dMdT",numMeas,numNodes,numGeomVals), dMdG;
  for(int i=0; i<numMeas; i++)
    for(int j=0; j<numNodes; j++)
      for(int k=0; k<numGeomVals; k++)
        dMdT(i,j,k) = 1.0;
  model->mapToGeometryData(dMdT,dMdG);

  EXPECT_NEAR(dMdG(0,0,0), 4.0, tolerance);

  std::cout << " wait " << std::endl;

}

