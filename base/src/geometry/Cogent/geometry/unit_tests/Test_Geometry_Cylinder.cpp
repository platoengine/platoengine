/*
 * Test_Geometry_Cylinder.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: jrobbin
 */

#include <gtest/gtest.h>
#include <iostream>
#include <cmath>

#include <Teuchos_XMLParameterListCoreHelpers.hpp>

#include "../Cogent_PrimitiveFactory.hpp"

static RealType tolerance = 1e-7;


/******************************************************************************
*  Test: cylinderTest
*  Purpose:  Verify level set values for Cylinder on X axis with no local 
*           coordinate system.
*******************************************************************************/
TEST(CylinderTest, cylinderOnX)
{

  Cogent::Geometry::PrimitiveFactory pFactory;

  Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem = Teuchos::null;

  Teuchos::ParameterList primSpec("Primitive");
  primSpec.set<std::string>("Type","Cylinder");
  primSpec.set<std::string>("Height","P0");
  primSpec.set<std::string>("Radius","P1");
  primSpec.set<std::string>("Axis","X");

  std::vector<std::string> paramNames{"P0","P1"};

  Teuchos::RCP<Cogent::Geometry::Primitive> prim = pFactory.create(primSpec, paramNames, coordinateSystem);
  
  std::vector<RealType> P{1.0,0.5};
  prim->update(P);
  
  const int nNodes = 8;
  const int nDims = 3;

  Cogent::FContainer<RealType> coordCon("coordConn",nNodes,nDims);
  Cogent::FContainer<RealType> topoData("topoData",nNodes,prim->getNumLevelsets());

  coordCon(0,0) = -1.0; coordCon(0,1) = -1.0; coordCon(0,2) = -1.0;
  coordCon(1,0) =  1.0; coordCon(1,1) = -1.0; coordCon(1,2) = -1.0;
  coordCon(2,0) =  1.0; coordCon(2,1) =  1.0; coordCon(2,2) = -1.0;
  coordCon(3,0) = -1.0; coordCon(3,1) =  1.0; coordCon(3,2) = -1.0;
  coordCon(4,0) = -1.0; coordCon(4,1) = -1.0; coordCon(4,2) =  1.0;
  coordCon(5,0) =  1.0; coordCon(5,1) = -1.0; coordCon(5,2) =  1.0;
  coordCon(6,0) =  1.0; coordCon(6,1) =  1.0; coordCon(6,2) =  1.0;
  coordCon(7,0) = -1.0; coordCon(7,1) =  1.0; coordCon(7,2) =  1.0;

  int offset=0;
  for(int i=0; i<prim->getNumLevelsets(); i++){
    prim->compute(coordCon, topoData, i, offset);
    offset++;
  }
  
  EXPECT_EQ( topoData(0,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(1,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(2,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(3,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(4,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(5,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(6,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(7,0), 0.5-sqrt(2.0) );

  EXPECT_EQ( topoData(0,1),-0.5 );
  EXPECT_EQ( topoData(1,1), 1.5 );
  EXPECT_EQ( topoData(2,1), 1.5 );
  EXPECT_EQ( topoData(3,1),-0.5 );
  EXPECT_EQ( topoData(4,1),-0.5 );
  EXPECT_EQ( topoData(5,1), 1.5 );
  EXPECT_EQ( topoData(6,1), 1.5 );
  EXPECT_EQ( topoData(7,1),-0.5 );

  EXPECT_EQ( topoData(0,2), 1.5 );
  EXPECT_EQ( topoData(1,2),-0.5 );
  EXPECT_EQ( topoData(2,2),-0.5 );
  EXPECT_EQ( topoData(3,2), 1.5 );
  EXPECT_EQ( topoData(4,2), 1.5 );
  EXPECT_EQ( topoData(5,2),-0.5 );
  EXPECT_EQ( topoData(6,2),-0.5 );
  EXPECT_EQ( topoData(7,2), 1.5 );
}

/******************************************************************************
*  Test: cylinderTest
*  Purpose:  Verify level set values for Cylinder on Y axis with no local 
*           coordinate system.
*******************************************************************************/
TEST(CylinderTest, cylinderOnY)
{

  Cogent::Geometry::PrimitiveFactory pFactory;

  Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem = Teuchos::null;

  Teuchos::ParameterList primSpec("Primitive");
  primSpec.set<std::string>("Type","Cylinder");
  primSpec.set<std::string>("Height","P0");
  primSpec.set<std::string>("Radius","P1");
  primSpec.set<std::string>("Axis","Y");

  std::vector<std::string> paramNames{"P0","P1"};

  Teuchos::RCP<Cogent::Geometry::Primitive> prim = pFactory.create(primSpec, paramNames, coordinateSystem);
  
  std::vector<RealType> P{1.0,0.5};
  prim->update(P);
  
  const int nNodes = 8;
  const int nDims = 3;
  const int nLS = prim->getNumLevelsets();

  Cogent::FContainer<RealType> coordCon("coordConn",nNodes,nDims);
  Cogent::FContainer<RealType> topoData("topoData",nNodes,nLS);

  coordCon(0,0) = -1.0; coordCon(0,1) = -1.0; coordCon(0,2) = -1.0;
  coordCon(1,0) =  1.0; coordCon(1,1) = -1.0; coordCon(1,2) = -1.0;
  coordCon(2,0) =  1.0; coordCon(2,1) =  1.0; coordCon(2,2) = -1.0;
  coordCon(3,0) = -1.0; coordCon(3,1) =  1.0; coordCon(3,2) = -1.0;
  coordCon(4,0) = -1.0; coordCon(4,1) = -1.0; coordCon(4,2) =  1.0;
  coordCon(5,0) =  1.0; coordCon(5,1) = -1.0; coordCon(5,2) =  1.0;
  coordCon(6,0) =  1.0; coordCon(6,1) =  1.0; coordCon(6,2) =  1.0;
  coordCon(7,0) = -1.0; coordCon(7,1) =  1.0; coordCon(7,2) =  1.0;

  int offset=0;
  for(int i=0; i<nLS; i++){
    prim->compute(coordCon, topoData, i, offset);
    offset++;
  }
  
  EXPECT_EQ( topoData(0,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(1,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(2,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(3,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(4,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(5,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(6,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(7,0), 0.5-sqrt(2.0) );

  EXPECT_EQ( topoData(0,1),-0.5 );
  EXPECT_EQ( topoData(1,1),-0.5 );
  EXPECT_EQ( topoData(2,1), 1.5 );
  EXPECT_EQ( topoData(3,1), 1.5 );
  EXPECT_EQ( topoData(4,1),-0.5 );
  EXPECT_EQ( topoData(5,1),-0.5 );
  EXPECT_EQ( topoData(6,1), 1.5 );
  EXPECT_EQ( topoData(7,1), 1.5 );

  EXPECT_EQ( topoData(0,2), 1.5 );
  EXPECT_EQ( topoData(1,2), 1.5 );
  EXPECT_EQ( topoData(2,2),-0.5 );
  EXPECT_EQ( topoData(3,2),-0.5 );
  EXPECT_EQ( topoData(4,2), 1.5 );
  EXPECT_EQ( topoData(5,2), 1.5 );
  EXPECT_EQ( topoData(6,2),-0.5 );
  EXPECT_EQ( topoData(7,2),-0.5 );

  const int nParams = paramNames.size();
  Cogent::LocalMatrix<RealType> dTdG(nNodes, nParams);

  prim->computeDeriv(coordCon, dTdG, 0);
  EXPECT_NEAR( dTdG(0,0), 0.0, tolerance ); EXPECT_NEAR( dTdG(0,1), 1.0, tolerance );
  EXPECT_NEAR( dTdG(1,0), 0.0, tolerance ); EXPECT_NEAR( dTdG(1,1), 1.0, tolerance );
  EXPECT_NEAR( dTdG(2,0), 0.0, tolerance ); EXPECT_NEAR( dTdG(2,1), 1.0, tolerance );
  EXPECT_NEAR( dTdG(3,0), 0.0, tolerance ); EXPECT_NEAR( dTdG(3,1), 1.0, tolerance );

  prim->computeDeriv(coordCon, dTdG, 1);
  EXPECT_NEAR( dTdG(0,0), 0.5, tolerance ); EXPECT_NEAR( dTdG(0,1), 0.0, tolerance );
  EXPECT_NEAR( dTdG(1,0), 0.5, tolerance ); EXPECT_NEAR( dTdG(1,1), 0.0, tolerance );
  EXPECT_NEAR( dTdG(2,0), 0.5, tolerance ); EXPECT_NEAR( dTdG(2,1), 0.0, tolerance );
  EXPECT_NEAR( dTdG(3,0), 0.5, tolerance ); EXPECT_NEAR( dTdG(3,1), 0.0, tolerance );

  prim->computeDeriv(coordCon, dTdG, 2);
  EXPECT_NEAR( dTdG(0,0), 0.5, tolerance ); EXPECT_NEAR( dTdG(0,1), 0.0, tolerance );
  EXPECT_NEAR( dTdG(1,0), 0.5, tolerance ); EXPECT_NEAR( dTdG(1,1), 0.0, tolerance );
  EXPECT_NEAR( dTdG(2,0), 0.5, tolerance ); EXPECT_NEAR( dTdG(2,1), 0.0, tolerance );
  EXPECT_NEAR( dTdG(3,0), 0.5, tolerance ); EXPECT_NEAR( dTdG(3,1), 0.0, tolerance );

}

/******************************************************************************
*  Test: cylinderTest
*  Purpose:  Verify level set values for Cylinder on Z axis with no local 
*           coordinate system.
*******************************************************************************/
TEST(CylinderTest, cylinderOnZ)
{

  Cogent::Geometry::PrimitiveFactory pFactory;

  Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem = Teuchos::null;

  Teuchos::ParameterList primSpec("Primitive");
  primSpec.set<std::string>("Type","Cylinder");
  primSpec.set<std::string>("Height","P0");
  primSpec.set<std::string>("Radius","P1");
  primSpec.set<std::string>("Axis","Z");

  std::vector<std::string> paramNames{"P0","P1"};

  Teuchos::RCP<Cogent::Geometry::Primitive> prim = pFactory.create(primSpec, paramNames, coordinateSystem);
  
  std::vector<RealType> P{1.0,0.5};
  prim->update(P);
  
  const int nNodes = 8;
  const int nDims = 3;

  Cogent::FContainer<RealType> coordCon("coordConn",nNodes,nDims);
  Cogent::FContainer<RealType> topoData("topoData",nNodes,prim->getNumLevelsets());

  coordCon(0,0) = -1.0; coordCon(0,1) = -1.0; coordCon(0,2) = -1.0;
  coordCon(1,0) =  1.0; coordCon(1,1) = -1.0; coordCon(1,2) = -1.0;
  coordCon(2,0) =  1.0; coordCon(2,1) =  1.0; coordCon(2,2) = -1.0;
  coordCon(3,0) = -1.0; coordCon(3,1) =  1.0; coordCon(3,2) = -1.0;
  coordCon(4,0) = -1.0; coordCon(4,1) = -1.0; coordCon(4,2) =  1.0;
  coordCon(5,0) =  1.0; coordCon(5,1) = -1.0; coordCon(5,2) =  1.0;
  coordCon(6,0) =  1.0; coordCon(6,1) =  1.0; coordCon(6,2) =  1.0;
  coordCon(7,0) = -1.0; coordCon(7,1) =  1.0; coordCon(7,2) =  1.0;

  int offset=0;
  for(int i=0; i<prim->getNumLevelsets(); i++){
    prim->compute(coordCon, topoData, i, offset);
    offset++;
  }
  
  EXPECT_EQ( topoData(0,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(1,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(2,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(3,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(4,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(5,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(6,0), 0.5-sqrt(2.0) );
  EXPECT_EQ( topoData(7,0), 0.5-sqrt(2.0) );

  EXPECT_EQ( topoData(0,1),-0.5 );
  EXPECT_EQ( topoData(1,1),-0.5 );
  EXPECT_EQ( topoData(2,1),-0.5 );
  EXPECT_EQ( topoData(3,1),-0.5 );
  EXPECT_EQ( topoData(4,1), 1.5 );
  EXPECT_EQ( topoData(5,1), 1.5 );
  EXPECT_EQ( topoData(6,1), 1.5 );
  EXPECT_EQ( topoData(7,1), 1.5 );

  EXPECT_EQ( topoData(0,2), 1.5 );
  EXPECT_EQ( topoData(1,2), 1.5 );
  EXPECT_EQ( topoData(2,2), 1.5 );
  EXPECT_EQ( topoData(3,2), 1.5 );
  EXPECT_EQ( topoData(4,2),-0.5 );
  EXPECT_EQ( topoData(5,2),-0.5 );
  EXPECT_EQ( topoData(6,2),-0.5 );
  EXPECT_EQ( topoData(7,2),-0.5 );
}



/******************************************************************************
*  Test: cylinderTest
*  Purpose:  Verify level set values for Cylinder on X axis with a given local
*           coordinate system.  The local coordinate system is rotated pi/4 
*           around the X axis, so the derivative of the cylinder level set 
*           values wrt the rotation should be zero.
*******************************************************************************/
TEST(CylinderTest, cylinderOnXwithRotationOnX)
{

  std::vector<std::string> paramNames{"P0","P1","P2"};

  Cogent::Geometry::PrimitiveFactory pFactory;

  Teuchos::ParameterList coordSpec("Local Coordinate System");
  coordSpec.sublist("Origin").set<std::string>("X","-0.5");
  coordSpec.sublist("Origin").set<std::string>("Y"," 0.0");
  coordSpec.sublist("Origin").set<std::string>("Z"," 0.0");
  coordSpec.sublist("Basis").set<Teuchos::Array<std::string>>("X Axis",
  Teuchos::Array<std::string>(std::vector<std::string>{"1.0", "0.0", "0.0"}));
  coordSpec.sublist("Basis").set<Teuchos::Array<std::string>>("Y Axis",
  Teuchos::Array<std::string>(std::vector<std::string>{"0.0", "cos(P2)", "sin(P2)"}));
  coordSpec.sublist("Basis").set<Teuchos::Array<std::string>>("Z Axis",
  Teuchos::Array<std::string>(std::vector<std::string>{"0.0", "-sin(P2)", "cos(P2)"}));

  Teuchos::RCP<Cogent::CoordinateSystem> 
    coordinateSystem = Teuchos::rcp(new Cogent::CoordinateSystem(coordSpec, paramNames, Teuchos::null));

  Teuchos::ParameterList primSpec("Primitive");
  primSpec.set<std::string>("Type","Cylinder");
  primSpec.set<std::string>("Height","P0");
  primSpec.set<std::string>("Radius","P1");
  primSpec.set<std::string>("Axis","X");

  Teuchos::RCP<Cogent::Geometry::Primitive> prim = pFactory.create(primSpec, paramNames, coordinateSystem);
  
  std::vector<RealType> P{1.0,0.5,acos(-1.0)/4.0};
  prim->update(P);
  
  const int nNodes = 8;
  const int nDims = 3;

  Cogent::FContainer<RealType> coordCon("coordConn",nNodes,nDims);
  Cogent::FContainer<RealType> topoData("topoData",nNodes,prim->getNumLevelsets());

  coordCon(0,0) = -1.0; coordCon(0,1) = -1.0; coordCon(0,2) = -1.0;
  coordCon(1,0) =  1.0; coordCon(1,1) = -1.0; coordCon(1,2) = -1.0;
  coordCon(2,0) =  1.0; coordCon(2,1) =  1.0; coordCon(2,2) = -1.0;
  coordCon(3,0) = -1.0; coordCon(3,1) =  1.0; coordCon(3,2) = -1.0;
  coordCon(4,0) = -1.0; coordCon(4,1) = -1.0; coordCon(4,2) =  1.0;
  coordCon(5,0) =  1.0; coordCon(5,1) = -1.0; coordCon(5,2) =  1.0;
  coordCon(6,0) =  1.0; coordCon(6,1) =  1.0; coordCon(6,2) =  1.0;
  coordCon(7,0) = -1.0; coordCon(7,1) =  1.0; coordCon(7,2) =  1.0;

  int offset=0;
  for(int i=0; i<prim->getNumLevelsets(); i++){
    prim->compute(coordCon, topoData, i, offset);
    offset++;
  }
  
  EXPECT_NEAR( topoData(0,0), 0.5-sqrt(2.0), tolerance);
  EXPECT_NEAR( topoData(1,0), 0.5-sqrt(2.0), tolerance);
  EXPECT_NEAR( topoData(2,0), 0.5-sqrt(2.0), tolerance);
  EXPECT_NEAR( topoData(3,0), 0.5-sqrt(2.0), tolerance);
  EXPECT_NEAR( topoData(4,0), 0.5-sqrt(2.0), tolerance);
  EXPECT_NEAR( topoData(5,0), 0.5-sqrt(2.0), tolerance);
  EXPECT_NEAR( topoData(6,0), 0.5-sqrt(2.0), tolerance);
  EXPECT_NEAR( topoData(7,0), 0.5-sqrt(2.0), tolerance);

  EXPECT_EQ( topoData(0,1), 0.0 );
  EXPECT_EQ( topoData(1,1), 2.0 );
  EXPECT_EQ( topoData(2,1), 2.0 );
  EXPECT_EQ( topoData(3,1), 0.0 );
  EXPECT_EQ( topoData(4,1), 0.0 );
  EXPECT_EQ( topoData(5,1), 2.0 );
  EXPECT_EQ( topoData(6,1), 2.0 );
  EXPECT_EQ( topoData(7,1), 0.0 );

  EXPECT_EQ( topoData(0,2), 1.0 );
  EXPECT_EQ( topoData(1,2),-1.0 );
  EXPECT_EQ( topoData(2,2),-1.0 );
  EXPECT_EQ( topoData(3,2), 1.0 );
  EXPECT_EQ( topoData(4,2), 1.0 );
  EXPECT_EQ( topoData(5,2),-1.0 );
  EXPECT_EQ( topoData(6,2),-1.0 );
  EXPECT_EQ( topoData(7,2), 1.0 );

  const int nParams = paramNames.size();
  Cogent::LocalMatrix<RealType> dTdG(nNodes, nParams);

  prim->computeDeriv(coordCon, dTdG, 2);
  EXPECT_NEAR( dTdG(0,2), 0.0, tolerance ); 
  EXPECT_NEAR( dTdG(1,2), 0.0, tolerance );
  EXPECT_NEAR( dTdG(2,2), 0.0, tolerance );
  EXPECT_NEAR( dTdG(3,2), 0.0, tolerance );
  EXPECT_NEAR( dTdG(4,2), 0.0, tolerance );
  EXPECT_NEAR( dTdG(5,2), 0.0, tolerance );
  EXPECT_NEAR( dTdG(6,2), 0.0, tolerance );
  EXPECT_NEAR( dTdG(7,2), 0.0, tolerance );
}


/******************************************************************************
*  Test: cylinderTest
*  Purpose:  Verify level set values for Cylinder on Z axis with a given local
*           coordinate system.  The local coordinate system is rotated pi/4 
*           around the Z axis.
*
*         Paste the following into mathematica to generate the expressions for 
*         'p' and 'dpda' in the lambdas below.
*         R  = {{1, 0, 0}, {0, Cos[a], Sin[a]}, {0, -Sin[a], Cos[a]}};
*         s  = {-1, 0, 0};
*         X  = {x, y, z);
*         P  = Transpose[R.Transpose[{X-s}]][[1]];
*         p  = r - (P[[1]]*P[[1]]+P[[2]]*P[[2]])^(1/2);
*         dpda = D[d, a];
*
*******************************************************************************/
TEST(CylinderTest, cylinderOnZwithRotationOnX)
{

  // define shape parameters

  const RealType a = acos(-1.0)/2.0;   // rotation about X (90 degrees)
  const RealType r = 0.5;              // Cylinder radius
  const RealType h = 1.0;              // Cylinder height
  std::vector<std::string> paramNames{"h","r","a"};
  std::vector<RealType> P{h,r,a};

  
  // define solution lambdas

  // distance function:
  auto p = [r](RealType a, RealType x, RealType y, RealType z){
    return r - sqrt(pow((1.0+x),2.0)+pow((y*cos(a)+z*sin(a)),2.0));
  };

  // derivative of distance function wrt angle of rotation:
  auto dpda = [](RealType a, RealType x, RealType y, RealType z){
    return -((z*cos(a) - y*sin(a))*(y*cos(a)+z*sin(a)))/
            sqrt(pow((1.0+x),2.0)+pow((y*cos(a)+z*sin(a)),2.0));
  };


  // Define element node coordinates

  const int nNodes = 8;
  const int nDims = 3;
  Cogent::FContainer<RealType> coordCon("coordConn",nNodes,nDims);
  coordCon(0,0) = -1.0; coordCon(0,1) = -1.0; coordCon(0,2) = -1.0;
  coordCon(1,0) =  1.0; coordCon(1,1) = -1.0; coordCon(1,2) = -1.0;
  coordCon(2,0) =  1.0; coordCon(2,1) =  1.0; coordCon(2,2) = -1.0;
  coordCon(3,0) = -1.0; coordCon(3,1) =  1.0; coordCon(3,2) = -1.0;
  coordCon(4,0) = -1.0; coordCon(4,1) = -1.0; coordCon(4,2) =  1.0;
  coordCon(5,0) =  1.0; coordCon(5,1) = -1.0; coordCon(5,2) =  1.0;
  coordCon(6,0) =  1.0; coordCon(6,1) =  1.0; coordCon(6,2) =  1.0;
  coordCon(7,0) = -1.0; coordCon(7,1) =  1.0; coordCon(7,2) =  1.0;


  // Create local coordinate system with rotation about global X

  Teuchos::RCP<Teuchos::ParameterList> coordSpec =
  Teuchos::getParametersFromXmlString(
    "<ParameterList name='Local Coordinate System'>                                      \n"
    "  <ParameterList name='Origin'>                                                     \n"
    "    <Parameter name='X' type='string' value='-1.0'/>                                \n"
    "    <Parameter name='Y' type='string' value=' 0.0'/>                                \n"
    "    <Parameter name='Z' type='string' value=' 0.0'/>                                \n"
    "  </ParameterList>                                                                  \n"
    "  <ParameterList name='Basis'>                                                      \n"
    "     <Parameter name='X Axis' type='Array(string)' value='{1.0, 0.0, 0.0}'/>        \n"
    "     <Parameter name='Y Axis' type='Array(string)' value='{0.0, cos(a), sin(a)}'/>  \n"
    "     <Parameter name='Z Axis' type='Array(string)' value='{0.0,-sin(a), cos(a)}'/>  \n"
    "  </ParameterList>                                                                  \n"
    "</ParameterList>                                                                    \n"
  );

  Teuchos::RCP<Cogent::CoordinateSystem> 
    coordinateSystem = Teuchos::rcp(new Cogent::CoordinateSystem(*coordSpec, paramNames, Teuchos::null));


  // Create Cylinder on Z axis in local coordinate system

  Teuchos::RCP<Teuchos::ParameterList> primSpec =
  Teuchos::getParametersFromXmlString(
    "<ParameterList name='Primitive'>                             \n"
    "   <Parameter name='Type' type='string' value='Cylinder'/>   \n"
    "     <Parameter name='Height' type='string' value='h'/>      \n"
    "     <Parameter name='Radius' type='string' value='r'/>      \n"
    "   <Parameter name='Axis' type='string' value='Z'/>          \n"
    "</ParameterList>                                             \n"
  );

  Cogent::Geometry::PrimitiveFactory pFactory;
  Teuchos::RCP<Cogent::Geometry::Primitive> prim = pFactory.create(*primSpec, paramNames, coordinateSystem);
  
  
  // Set shape parameter values

  prim->update(P);
  

  // compute level set values at nodes and compare against p()

  int offset=0;
  Cogent::FContainer<RealType> topoData("topoData",nNodes,prim->getNumLevelsets());
  for(int i=0; i<prim->getNumLevelsets(); i++){
    prim->compute(coordCon, topoData, i, offset);
    offset++;
  }
  for(int i=0; i<nNodes; i++){
    EXPECT_NEAR( topoData(i,0), p(P[2], coordCon(i,0), coordCon(i,1), coordCon(i,2)), tolerance);
  }


  // compute derivatives of level set values wrt rotation parameter, a, and compare against dpda()

  const int nParams = paramNames.size();
  Cogent::LocalMatrix<RealType> dTdG(nNodes, nParams);
  prim->computeDeriv(coordCon, dTdG, 0);
  for(int i=0; i<nNodes; i++){
    EXPECT_NEAR( dTdG(i,/*param_index=*/2), dpda(P[2], coordCon(i,0), coordCon(i,1), coordCon(i,2)), tolerance);
  }


  // Repeat for 45 degree rotation

  // Set shape parameter values

  P[2] = acos(-1.0)/4.0;
  prim->update(P);
  

  // compute level set values at nodes and compare against p()

  offset=0;
  for(int i=0; i<prim->getNumLevelsets(); i++){
    prim->compute(coordCon, topoData, i, offset);
    offset++;
  }
  for(int i=0; i<nNodes; i++){
    EXPECT_NEAR( topoData(i,0), p(P[2], coordCon(i,0), coordCon(i,1), coordCon(i,2)), tolerance);
  }


  // compute derivatives of level set values wrt rotation parameter, a, and compare against dpda()

  prim->computeDeriv(coordCon, dTdG, 0);
  for(int i=0; i<nNodes; i++){
    EXPECT_NEAR( dTdG(i,/*param_index=*/2), dpda(P[2], coordCon(i,0), coordCon(i,1), coordCon(i,2)), tolerance);
  }
}
