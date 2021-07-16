/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

/*
 * Plato_Test_Geometry_MLS.cpp
 *
 *  Created on: Feb 27, 2018
 */

#include <gtest/gtest.h>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include "Plato_MLS.hpp"

#include "pugixml.hpp"

namespace PlatoDevel
{
}

namespace PlatoTestGeometryMLS
{
  TEST(PlatoTestGeometryMLS, NeighborMap3D)
  {
    int n1=6, n2=8, n3=2, n4=4;
    double s1=1.0, s2=4.0/3.0, s3=1.0/3.0, s4=2.0/3.0, radius=s1/2.0;

    std::stringstream input;
    input << "<PointArray>";
    input << "  <N1>" << n1 << "</N1>";
    input << "  <N2>" << n2 << "</N2>";
    input << "  <N3>" << n3 << "</N3>";
    input << "  <N4>" << n4 << "</N4>";
    input << "  <Size1>" << std::setprecision(18) << s1 << "</Size1>";
    input << "  <Size2>" << std::setprecision(18) << s2 << "</Size2>";
    input << "  <Size3>" << std::setprecision(18) << s3 << "</Size3>";
    input << "  <Size4>" << std::setprecision(18) << s4 << "</Size4>";
    input << "  <Radius>" << std::setprecision(18) << radius << "</Radius>";
    input << "</PointArray>";
    
    Plato::Parser* parser = new Plato::PugiParser();
    auto tInputData = parser->parseString(input.str());
    delete parser;

    auto tMLSParams = tInputData.get<Plato::InputData>("PointArray");
    Plato::Geometry::MovingLeastSquares<3, double> mls_double(tMLSParams);

    auto coords_double = mls_double.getPointCoords();

    Plato::Geometry::NeighborMapper<3,double> tNeighborMapper(coords_double);
    auto t_map = tNeighborMapper.map( 27 );

    auto t_mapHost = Kokkos::create_mirror_view(t_map);
    Kokkos::deep_copy(t_mapHost, t_map);
    EXPECT_EQ(t_mapHost(0,0), 0);
    EXPECT_EQ(t_mapHost(0,1), 2);
    EXPECT_EQ(t_mapHost(0,2), 16);
  }

  TEST(PlatoTestGeometryMLS, MovingLeastSquares_Coords)
  {

    int n1=6, n2=8, n3=2, n4=4;
    double s1=1.0, s2=4.0/3.0, s3=1.0/3.0, s4=2.0/3.0, radius=s1/2.0;

    std::stringstream input;
    input << "<PointArray>";
    input << "  <N1>" << n1 << "</N1>";
    input << "  <N2>" << n2 << "</N2>";
    input << "  <N3>" << n3 << "</N3>";
    input << "  <N4>" << n4 << "</N4>";
    input << "  <Size1>" << std::setprecision(18) << s1 << "</Size1>";
    input << "  <Size2>" << std::setprecision(18) << s2 << "</Size2>";
    input << "  <Size3>" << std::setprecision(18) << s3 << "</Size3>";
    input << "  <Size4>" << std::setprecision(18) << s4 << "</Size4>";
    input << "  <Radius>" << std::setprecision(18) << radius << "</Radius>";
    input << "</PointArray>";
    
    Plato::Parser* parser = new Plato::PugiParser();
    auto tInputData = parser->parseString(input.str());
    delete parser;

    auto tMLSParams = tInputData.get<Plato::InputData>("PointArray");
    Plato::Geometry::MovingLeastSquares<3, double> mls_double(tMLSParams);

    double tol_double = 1e-16;

    auto coords_double = mls_double.getPointCoords();

    EXPECT_NEAR(/*Gold=*/   s3/(n3-1), /*Result=*/ coords_double(1,2), tol_double);



    Plato::Geometry::MovingLeastSquares<3, float> mls_float(tMLSParams);

    double tol_float = 1e-8;

    auto coords_float = mls_float.getPointCoords();
 
    EXPECT_NEAR(/*Gold=*/ s3/(n3-1), /*Result=*/ coords_float(1,2), tol_float);



    Plato::Geometry::MovingLeastSquares<2, double> mls_double_2D(tMLSParams);

    auto coords_double_2D = mls_double_2D.getPointCoords();

    EXPECT_NEAR(/*Gold=*/ s2/(n2-1), /*Result=*/ coords_double_2D(1,1), tol_double);



    Plato::Geometry::MovingLeastSquares<1, double> mls_double_1D(tMLSParams);

    auto coords_double_1D = mls_double_1D.getPointCoords();

    EXPECT_NEAR(/*Gold=*/ s1/(n1-1), /*Result=*/ coords_double_1D(1,0), tol_double);



    Plato::Geometry::MovingLeastSquares<4, double> mls_double_4D(tMLSParams);

    auto coords_double_4D = mls_double_4D.getPointCoords();
 
    EXPECT_NEAR(/*Gold=*/ s4/(n4-1), /*Result=*/ coords_double_4D(1,3), tol_double);


  }

  TEST(PlatoTestGeometryMLS, MovingLeastSquaresFunction3D)
  {

    int n1=6, n2=8, n3=2, n4=4;
    double s1=1.0, s2=4.0/3.0, s3=1.0/3.0, s4=2.0/3.0, radius=s1/2.0;

    std::stringstream input;
    input << "<PointArray>";
    input << "  <N1>" << n1 << "</N1>";
    input << "  <N2>" << n2 << "</N2>";
    input << "  <N3>" << n3 << "</N3>";
    input << "  <N4>" << n4 << "</N4>";
    input << "  <Size1>" << std::setprecision(18) << s1 << "</Size1>";
    input << "  <Size2>" << std::setprecision(18) << s2 << "</Size2>";
    input << "  <Size3>" << std::setprecision(18) << s3 << "</Size3>";
    input << "  <Size4>" << std::setprecision(18) << s4 << "</Size4>";
    input << "  <Radius>" << std::setprecision(18) << radius << "</Radius>";
    input << "</PointArray>";

    Plato::Parser* parser = new Plato::PugiParser();
    auto tInputData = parser->parseString(input.str());
    delete parser;

    constexpr int numDims=3;
    auto tMLSParams = tInputData.get<Plato::InputData>("PointArray");
    Plato::Geometry::MovingLeastSquares<numDims, double> mls(tMLSParams);

    int m1=13, m2=17, m3=5;
    int numNodes = m1*m2*m3;

    Kokkos::View<double**, Kokkos::LayoutRight, Kokkos::DefaultExecutionSpace::memory_space>
      nodeCoords("coords", numNodes, numDims);

    auto nodeCoordsHost = Kokkos::create_mirror_view(nodeCoords);

    double d1 = s1/(m1-1);
    double d2 = s2/(m2-1);
    double d3 = s3/(m3-1);
    int nodeIndex = 0;
    for(int i=0; i<m1; i++)
      for(int j=0; j<m2; j++)
        for(int k=0; k<m3; k++){
          nodeCoordsHost(nodeIndex,0) = i*d1;
          nodeCoordsHost(nodeIndex,1) = j*d2;
          nodeCoordsHost(nodeIndex,2) = k*d3;
          nodeIndex++;
        }
    Kokkos::deep_copy(nodeCoords, nodeCoordsHost);

    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      nodeValues("values", numNodes);
 
    auto pointCoords = mls.getPointCoords();
    auto numPoints = pointCoords.extent(0);
    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      pointValues("values", numPoints);

    auto pointValuesHost = Kokkos::create_mirror_view(pointValues);

    for(decltype(numPoints) iPoint=0; iPoint<numPoints; iPoint++){
      auto x = pointCoords(iPoint,0);
      auto y = pointCoords(iPoint,1);
      auto z = pointCoords(iPoint,2);
      pointValuesHost(iPoint) = cos(x)*cos(y)*cos(z);
    }
    Kokkos::deep_copy(pointValues, pointValuesHost);
 
    mls.f(pointValues, nodeCoords, nodeValues);

    auto nodeValuesHost = Kokkos::create_mirror_view(nodeValues);
    Kokkos::deep_copy(nodeValuesHost, nodeValues);

    EXPECT_NEAR(/*Gold=*/ 0.884527284424596938, /*Result=*/ nodeValuesHost(0), /*tTolerance=*/1e-14);
    EXPECT_NEAR(/*Gold=*/ 0.291296592937485666, /*Result=*/ nodeValuesHost(numNodes-1), /*tTolerance=*/1e-14);
  }

  TEST(PlatoTestGeometryMLS, MovingLeastSquaresFunction2D)
  {

    int n1=12, n2=16, n3=2, n4=4;
    double s1=1.0, s2=4.0/3.0, s3=1.0/3.0, s4=2.0/3.0, radius=0.05;

    std::stringstream input;
    input << "<PointArray>";
    input << "  <N1>" << n1 << "</N1>";
    input << "  <N2>" << n2 << "</N2>";
    input << "  <N3>" << n3 << "</N3>";
    input << "  <N4>" << n4 << "</N4>";
    input << "  <Size1>" << std::setprecision(18) << s1 << "</Size1>";
    input << "  <Size2>" << std::setprecision(18) << s2 << "</Size2>";
    input << "  <Size3>" << std::setprecision(18) << s3 << "</Size3>";
    input << "  <Size4>" << std::setprecision(18) << s4 << "</Size4>";
    input << "  <Radius>" << std::setprecision(18) << radius << "</Radius>";
    input << "</PointArray>";

    Plato::Parser* parser = new Plato::PugiParser();
    auto tInputData = parser->parseString(input.str());
    delete parser;

    
    constexpr int numDims=3;
    auto tMLSParams = tInputData.get<Plato::InputData>("PointArray");
    Plato::Geometry::MovingLeastSquares<numDims, double> mls(tMLSParams);

    int m1=13, m2=17;
    int numNodes = m1*m2;

    Kokkos::View<double**, Kokkos::LayoutRight, Kokkos::DefaultExecutionSpace::memory_space>
      nodeCoords("coords", numNodes, numDims);

    auto nodeCoordsHost = Kokkos::create_mirror_view(nodeCoords);

    double d1 = s1/(m1-1);
    double d2 = s2/(m2-1);
    int nodeIndex = 0;
    for(int i=0; i<m1; i++)
      for(int j=0; j<m2; j++){
        nodeCoordsHost(nodeIndex,0) = i*d1;
        nodeCoordsHost(nodeIndex,1) = j*d2;
        nodeIndex++;
      }
    Kokkos::deep_copy(nodeCoords, nodeCoordsHost);

    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      nodeValues("values", numNodes);
 
    auto pointCoords = mls.getPointCoords();
    auto numPoints = pointCoords.extent(0);
    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      pointValues("values", numPoints);

    auto pointValuesHost = Kokkos::create_mirror_view(pointValues);
 
    double pi = acos(-1.0);
    for(decltype(numPoints) iPoint=0; iPoint<numPoints; iPoint++){
      auto x = pointCoords(iPoint,0);
      auto y = pointCoords(iPoint,1);
      pointValuesHost(iPoint) = cos(2.0*pi*x)*cos(2.0*pi*y);
    }
    Kokkos::deep_copy(pointValues, pointValuesHost);
 
    mls.f(pointValues, nodeCoords, nodeValues);

    auto nodeValuesHost = Kokkos::create_mirror_view(nodeValues);
    Kokkos::deep_copy(nodeValuesHost, nodeValues);

    // write output
#ifdef NOPE
    std::ofstream pointsStream;
    pointsStream.open("points.dat");
    for(decltype(numPoints) iPoint=0; iPoint<numPoints; iPoint++){
      pointsStream << std::setprecision(18) << pointCoords(iPoint,0) << " " << pointCoords(iPoint,1) << " " << pointValues(iPoint) << std::endl;
    }
    pointsStream.close();
    
    std::ofstream nodeStream;
    nodeStream.open("nodes.dat");
    for(int iNode=0; iNode<numNodes; iNode++){
      if( iNode%m2 == 0 ) nodeStream << std::endl;
      nodeStream << std::setprecision(18) << nodeCoords(iNode,0) << " " << nodeCoords(iNode,1) << " " << nodeValues(iNode) << std::endl;
    }
    nodeStream.close();
#endif

    EXPECT_NEAR(/*Gold=*/0.988235244463390061, /*Result=*/ nodeValuesHost(0), /*tTolerance=*/1e-14);
    EXPECT_NEAR(/*Gold=*/-0.475551223063872264, /*Result=*/ nodeValuesHost(numNodes-1), /*tTolerance=*/1e-14);
  }

  TEST(PlatoTestGeometryMLS, MovingLeastSquaresMap2D)
  {

    int n1=12, n2=16, n3=2, n4=4;
    double s1=1.0, s2=4.0/3.0, s3=1.0/3.0, s4=2.0/3.0, radius=0.05;

    std::stringstream input;
    input << "<PointArray>";
    input << "  <N1>" << n1 << "</N1>";
    input << "  <N2>" << n2 << "</N2>";
    input << "  <N3>" << n3 << "</N3>";
    input << "  <N4>" << n4 << "</N4>";
    input << "  <Size1>" << std::setprecision(18) << s1 << "</Size1>";
    input << "  <Size2>" << std::setprecision(18) << s2 << "</Size2>";
    input << "  <Size3>" << std::setprecision(18) << s3 << "</Size3>";
    input << "  <Size4>" << std::setprecision(18) << s4 << "</Size4>";
    input << "  <Radius>" << std::setprecision(18) << radius << "</Radius>";
    input << "</PointArray>";

    Plato::Parser* parser = new Plato::PugiParser();
    auto tInputData = parser->parseString(input.str());
    delete parser;

    
    constexpr int numDims=2;
    auto tMLSParams = tInputData.get<Plato::InputData>("PointArray");
    Plato::Geometry::MovingLeastSquares<numDims, double> mls(tMLSParams);

    int m1=13, m2=17;
    int numNodes = m1*m2;

    Kokkos::View<double**, Kokkos::LayoutRight, Kokkos::DefaultExecutionSpace::memory_space>
      nodeCoords("coords", numNodes, numDims);

    auto nodeCoordsHost = Kokkos::create_mirror_view(nodeCoords);

    double d1 = s1/(m1-1);
    double d2 = s2/(m2-1);
    int nodeIndex = 0;
    for(int i=0; i<m1; i++)
      for(int j=0; j<m2; j++){
        nodeCoordsHost(nodeIndex,0) = i*d1;
        nodeCoordsHost(nodeIndex,1) = j*d2;
        nodeIndex++;
      }
    Kokkos::deep_copy(nodeCoords, nodeCoordsHost);

    auto pointCoords = mls.getPointCoords();
    auto numPoints = pointCoords.extent(0);
    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      pointValues("values", numPoints);
 
    auto pointValuesHost = Kokkos::create_mirror_view(pointValues);
 
    double pi = acos(-1.0);
    for(decltype(numPoints) iPoint=0; iPoint<numPoints; iPoint++){
      auto x = pointCoords(iPoint,0);
      auto y = pointCoords(iPoint,1);
      pointValuesHost(iPoint) = cos(2.0*pi*x)*cos(2.0*pi*y);
    }
    Kokkos::deep_copy(pointValues, pointValuesHost);
 
    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      nodeValues("values", numNodes);

    mls.f(pointValues, nodeCoords, nodeValues);

    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      mappedValues("values", numPoints);
 
    mls.mapToPoints(nodeCoords, nodeValues, mappedValues);

    auto nodeValuesHost = Kokkos::create_mirror_view(nodeValues);
    Kokkos::deep_copy(nodeValuesHost, nodeValues);

    auto mappedValuesHost = Kokkos::create_mirror_view(mappedValues);
    Kokkos::deep_copy(mappedValuesHost, mappedValues);

#ifdef NOPE
    std::ofstream pointsStream;
    pointsStream.open("points.dat");
    for(decltype(numPoints) iPoint=0; iPoint<numPoints; iPoint++){
      if( iPoint%n2 == 0 ) pointsStream << std::endl;
      pointsStream << std::setprecision(18) << pointCoords(iPoint,0) << " " << pointCoords(iPoint,1) << " " << pointValues(iPoint) << std::endl;
    }
    pointsStream.close();

    std::ofstream mappedStream;
    mappedStream.open("mapped.dat");
    for(decltype(numPoints) iPoint=0; iPoint<numPoints; iPoint++){
      if( iPoint%n2 == 0 ) mappedStream << std::endl;
      mappedStream << std::setprecision(18) << pointCoords(iPoint,0) << " " << pointCoords(iPoint,1) << " " << mappedValues(iPoint) << std::endl;
    }
    mappedStream.close();

    std::ofstream nodeStream;
    nodeStream.open("nodes.dat");
    for(int iNode=0; iNode<numNodes; iNode++){
      if( iNode%m2 == 0 ) nodeStream << std::endl;
      nodeStream << std::setprecision(18) << nodeCoords(iNode,0) << " " << nodeCoords(iNode,1) << " " << nodeValues(iNode) << std::endl;
    }
    nodeStream.close();
#endif
    
    EXPECT_NEAR(/*Gold=*/ 0.98823524446339006, /*Result=*/ nodeValuesHost(0), /*tTolerance=*/1e-14);
    EXPECT_NEAR(/*Gold=*/ 1.0108182583697762, /*Result=*/ mappedValuesHost(0), /*tTolerance=*/1e-14);
  }

  TEST(PlatoTestGeometryMLS, MovingLeastSquaresMap3D)
  {

    int n1=12, n2=16, n3=2, n4=4;
    double s1=1.0, s2=4.0/3.0, s3=1.0/3.0, s4=2.0/3.0, radius=0.05;

    std::stringstream input;
    input << "<PointArray>";
    input << "  <N1>" << n1 << "</N1>";
    input << "  <N2>" << n2 << "</N2>";
    input << "  <N3>" << n3 << "</N3>";
    input << "  <N4>" << n4 << "</N4>";
    input << "  <Size1>" << std::setprecision(18) << s1 << "</Size1>";
    input << "  <Size2>" << std::setprecision(18) << s2 << "</Size2>";
    input << "  <Size3>" << std::setprecision(18) << s3 << "</Size3>";
    input << "  <Size4>" << std::setprecision(18) << s4 << "</Size4>";
    input << "  <Radius>" << std::setprecision(18) << radius << "</Radius>";
    input << "</PointArray>";

    Plato::Parser* parser = new Plato::PugiParser();
    auto tInputData = parser->parseString(input.str());
    delete parser;

    
    constexpr int numDims=3;
    auto tMLSParams = tInputData.get<Plato::InputData>("PointArray");
    Plato::Geometry::MovingLeastSquares<numDims, double> mls(tMLSParams);

    int m1=13, m2=17, m3=3;
    int numNodes = m1*m2*m3;

    Kokkos::View<double**, Kokkos::LayoutRight, Kokkos::DefaultExecutionSpace::memory_space>
      nodeCoords("coords", numNodes, numDims);

    auto nodeCoordsHost = Kokkos::create_mirror_view(nodeCoords);

    double d1 = s1/(m1-1);
    double d2 = s2/(m2-1);
    double d3 = s3/(m3-1);
    int nodeIndex = 0;
    for(int i=0; i<m1; i++)
      for(int j=0; j<m2; j++)
        for(int k=0; k<m3; k++){
          nodeCoordsHost(nodeIndex,0) = i*d1;
          nodeCoordsHost(nodeIndex,1) = j*d2;
          nodeCoordsHost(nodeIndex,2) = k*d3;
          nodeIndex++;
        }
    Kokkos::deep_copy(nodeCoords, nodeCoordsHost);

    auto pointCoords = mls.getPointCoords();
    auto numPoints = pointCoords.extent(0);
    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      pointValues("values", numPoints);
 
    auto pointValuesHost = Kokkos::create_mirror_view(pointValues);
 
    double pi = acos(-1.0);
    for(decltype(numPoints) iPoint=0; iPoint<numPoints; iPoint++){
      auto x = pointCoords(iPoint,0);
      auto y = pointCoords(iPoint,1);
      auto z = pointCoords(iPoint,2);
      pointValuesHost(iPoint) = cos(2.0*pi*x)*cos(2.0*pi*y)*cos(2.0*pi*z);
    }
    Kokkos::deep_copy(pointValues, pointValuesHost);
 
    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      nodeValues("values", numNodes);

    mls.f(pointValues, nodeCoords, nodeValues);

    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      mappedValues("values", numPoints);
 
    mls.mapToPoints(nodeCoords, nodeValues, mappedValues);

    auto nodeValuesHost = Kokkos::create_mirror_view(nodeValues);
    Kokkos::deep_copy(nodeValuesHost, nodeValues);

    auto mappedValuesHost = Kokkos::create_mirror_view(mappedValues);
    Kokkos::deep_copy(mappedValuesHost, mappedValues);

#ifdef NOPE
    std::ofstream pointsStream;
    pointsStream.open("points.dat");
    for(decltype(numPoints) iPoint=0; iPoint<numPoints; iPoint++){
      if( iPoint%n2 == 0 ) pointsStream << std::endl;
      pointsStream << std::setprecision(18) << pointCoords(iPoint,0) << " " << pointCoords(iPoint,1) << " " << pointValues(iPoint) << std::endl;
    }
    pointsStream.close();

    std::ofstream mappedStream;
    mappedStream.open("mapped.dat");
    for(decltype(numPoints) iPoint=0; iPoint<numPoints; iPoint++){
      if( iPoint%n2 == 0 ) mappedStream << std::endl;
      mappedStream << std::setprecision(18) << pointCoords(iPoint,0) << " " << pointCoords(iPoint,1) << " " << mappedValues(iPoint) << std::endl;
    }
    mappedStream.close();

    std::ofstream nodeStream;
    nodeStream.open("nodes.dat");
    for(int iNode=0; iNode<numNodes; iNode++){
      if( iNode%m2 == 0 ) nodeStream << std::endl;
      nodeStream << std::setprecision(18) << nodeCoords(iNode,0) << " " << nodeCoords(iNode,1) << " " << nodeValues(iNode) << std::endl;
    }
    nodeStream.close();
#endif
    
    EXPECT_NEAR(/*Gold=*/ 0.98823524446339006, /*Result=*/ nodeValuesHost(0), /*tTolerance=*/1e-14);
    EXPECT_NEAR(/*Gold=*/ 1.1371705406659982, /*Result=*/ mappedValuesHost(0), /*tTolerance=*/1e-14);
  }

  TEST(PlatoTestGeometryMLS, MovingLeastSquaresDefineField2D)
  {

    int n1=12, n2=16;
    double s1=1.0, s2=4.0/3.0, radius=0.05;
    double uniformValue = 0.5;
    double insideValue = 0.0, outsideValue = 1.0;

    std::stringstream input;
    input << "<PointArray>";
    input << "  <N1>" << n1 << "</N1>";
    input << "  <N2>" << n2 << "</N2>";
    input << "  <Size1>" << std::setprecision(18) << s1 << "</Size1>";
    input << "  <Size2>" << std::setprecision(18) << s2 << "</Size2>";
    input << "  <Radius>" << std::setprecision(18) << radius << "</Radius>";
    input << "  <Field>";
    input << "    <Name>Initial Porous Design</Name>";
    input << "    <Initializer>SphereArray</Initializer>";
    input << "    <SphereArray>";
    input << "      <Radius>0.15</Radius>";
    input << "      <Spacing>0.4</Spacing>";
    input << "      <InsideValue>" << insideValue << "</InsideValue>";
    input << "      <OutsideValue>" << outsideValue << "</OutsideValue>";
    input << "    </SphereArray>";
    input << "  </Field>";
    input << "  <Field>";
    input << "    <Name>Initial Uniform Design</Name>";
    input << "    <Initializer>Uniform</Initializer>";
    input << "    <Uniform>";
    input << "      <Value>" << uniformValue << "</Value>";
    input << "    </Uniform>";
    input << "  </Field>";
    input << "</PointArray>";

    Plato::Parser* parser = new Plato::PugiParser();
    auto tInputData = parser->parseString(input.str());
    delete parser;

    
    constexpr int numDims=2;
    auto tMLSParams = tInputData.get<Plato::InputData>("PointArray");
    Plato::Geometry::MovingLeastSquares<numDims, double> mls(tMLSParams);

    auto fields = mls.getPointFields();
    auto uniformField = fields["Initial Uniform Design"];
    auto uniformFieldHost = Kokkos::create_mirror_view(uniformField);
    Kokkos::deep_copy(uniformFieldHost, uniformField);
    EXPECT_NEAR(/*Gold=*/ uniformValue, /*Result=*/ uniformFieldHost[0], /*tTolerance=*/1e-14);

    auto porousField = fields["Initial Porous Design"];
    auto porousFieldHost = Kokkos::create_mirror_view(porousField);
    Kokkos::deep_copy(porousFieldHost, porousField);
    EXPECT_NEAR(/*Gold=*/ insideValue, /*Result=*/ porousFieldHost[0], /*tTolerance=*/1e-14);

    int m1=13, m2=17;
    int numNodes = m1*m2;

    Kokkos::View<double**, Kokkos::LayoutRight, Kokkos::DefaultExecutionSpace::memory_space>
      nodeCoords("coords", numNodes, numDims);

    auto nodeCoordsHost = Kokkos::create_mirror_view(nodeCoords);

    double d1 = s1/(m1-1);
    double d2 = s2/(m2-1);
    int nodeIndex = 0;
    for(int i=0; i<m1; i++)
      for(int j=0; j<m2; j++){
        nodeCoordsHost(nodeIndex,0) = i*d1;
        nodeCoordsHost(nodeIndex,1) = j*d2;
        nodeIndex++;
      }
    Kokkos::deep_copy(nodeCoordsHost, nodeCoords);

    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      nodeValues("values", numNodes);

    mls.f(porousField, nodeCoords, nodeValues);

    auto pointCoords = mls.getPointCoords();
    auto numPoints = pointCoords.extent(0);
    Kokkos::View<double*, Kokkos::DefaultExecutionSpace::memory_space>
      mappedValues("values", numPoints);
 
    mls.mapToPoints(nodeCoords, nodeValues, mappedValues);

    auto mappedValuesHost = Kokkos::create_mirror_view(mappedValues);
    Kokkos::deep_copy(mappedValuesHost, mappedValues);

    EXPECT_NEAR(/*Gold=*/ 0.0027358850269484326, /*Result=*/ mappedValuesHost(0), /*tTolerance=*/1e-14);

#ifdef NOPE
    std::ofstream pointsStream;
    pointsStream.open("points.dat");
    for(decltype(numPoints) iPoint=0; iPoint<numPoints; iPoint++){
      if( iPoint%n2 == 0 ) pointsStream << std::endl;
      pointsStream << std::setprecision(18) << pointCoords(iPoint,0) << " " << pointCoords(iPoint,1) << " " << porousField(iPoint) << std::endl;
    }
    pointsStream.close();

    std::ofstream mappedStream;
    mappedStream.open("mapped.dat");
    for(decltype(numPoints) iPoint=0; iPoint<numPoints; iPoint++){
      if( iPoint%n2 == 0 ) mappedStream << std::endl;
      mappedStream << std::setprecision(18) << pointCoords(iPoint,0) << " " << pointCoords(iPoint,1) << " " << mappedValues(iPoint) << std::endl;
    }
    mappedStream.close();

    std::ofstream nodeStream;
    nodeStream.open("nodes.dat");
    for(int iNode=0; iNode<numNodes; iNode++){
      if( iNode%m2 == 0 ) nodeStream << std::endl;
      nodeStream << std::setprecision(18) << nodeCoords(iNode,0) << " " << nodeCoords(iNode,1) << " " << nodeValues(iNode) << std::endl;
    }
    nodeStream.close();
#endif
    
  }
}

