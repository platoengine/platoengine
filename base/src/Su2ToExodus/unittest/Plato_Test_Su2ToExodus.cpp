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

#include <gtest/gtest.h>
#include "Su2ToExodus_UnitTester.hpp"

namespace PlatoTestSu2ToExodus
{

TEST(PlatoTestSu2ToExodus, getNamedIntegerField)
{
    Su2ToExodus::Su2ToExodus_UnitTester tTester;
    std::istringstream tStringStream;
    std::string tStringInput;

    tStringInput = "test=22";
    tStringStream.str(tStringInput);
    int tVal = tTester.publicGetNamedIntegerField(tStringStream, "test");
    EXPECT_EQ(tVal, 22);

    tStringInput = "test= 23";
    tStringStream.str(tStringInput);
    tStringStream.clear();
    tStringStream.seekg (0);
    tVal = tTester.publicGetNamedIntegerField(tStringStream, "test");
    EXPECT_EQ(tVal, 23);

    tStringInput = "test =24";
    tStringStream.str(tStringInput);
    tStringStream.clear();
    tStringStream.seekg (0);
    tVal = tTester.publicGetNamedIntegerField(tStringStream, "test");
    EXPECT_EQ(tVal, 24);

}

TEST(PlatoTestSu2ToExodus, readElementConnectivity)
{
    Su2ToExodus::Su2ToExodus_UnitTester tTester;
    std::istringstream tStringStream;
    std::string tStringInput;

    tTester.setNumElements(2);
    tStringInput = "10 1715 1680 3482 1679 8\n"
                   "10 11769 4079 4078 44 9";

    tStringStream.str(tStringInput);
    bool tVal = tTester.publicReadElementConnectivity(tStringStream);
    EXPECT_EQ(tVal, true);
    EXPECT_EQ(tTester.getElementConnectivity(0,0), 1715);
    EXPECT_EQ(tTester.getElementConnectivity(0,1), 1680);
    EXPECT_EQ(tTester.getElementConnectivity(0,2), 3482);
    EXPECT_EQ(tTester.getElementConnectivity(0,3), 1679);
    EXPECT_EQ(tTester.getElementConnectivity(1,0), 11769);
    EXPECT_EQ(tTester.getElementConnectivity(1,1), 4079);
    EXPECT_EQ(tTester.getElementConnectivity(1,2), 4078);
    EXPECT_EQ(tTester.getElementConnectivity(1,3), 44);
}

TEST(PlatoTestSu2ToExodus, readNodeCoordinates)
{
    Su2ToExodus::Su2ToExodus_UnitTester tTester;
    std::istringstream tStringStream;
    std::string tStringInput;

    tTester.setNumNodes(2);
    tTester.setNumDimensions(3);
    tStringInput = "6.6 7.7 8.8 9\n"
                   "2.3 2.2 3.4 10";

    tStringStream.str(tStringInput);
    bool tVal = tTester.publicReadNodeCoordinates(tStringStream);
    EXPECT_EQ(tVal, true);
    EXPECT_EQ(tTester.getNodeX(0), 6.6);
    EXPECT_EQ(tTester.getNodeY(0), 7.7);
    EXPECT_EQ(tTester.getNodeZ(0), 8.8);
    EXPECT_EQ(tTester.getNodeX(1), 2.3);
    EXPECT_EQ(tTester.getNodeY(1), 2.2);
    EXPECT_EQ(tTester.getNodeZ(1), 3.4);
}

TEST(PlatoTestSu2ToExodus, readMarks)
{
    Su2ToExodus::Su2ToExodus_UnitTester tTester;
    std::istringstream tStringStream;
    std::string tStringInput;

    tTester.setNumMarks(2);
    tStringInput = "MARKER_TAG= 1\n"
            "MARKER_ELEMS= 2\n"
            "5 223 177 178\n"
            "5 255 415 151\n"
            "MARKER_TAG= 5\n"
            "MARKER_ELEMS= 3\n"
            "5 223 177 178\n"
            "5 255 415 151\n"
            "5 3 5 7\n";
    tStringStream.str(tStringInput);
    bool tVal = tTester.publicReadMarks(tStringStream);
    EXPECT_EQ(tVal, true);
    EXPECT_EQ(tTester.getMarkNodeIndex(0,0,1), 177);
    EXPECT_EQ(tTester.getMarkNodeIndex(0,0,0), 223);
    EXPECT_EQ(tTester.getMarkNodeIndex(0,0,2), 178);
    EXPECT_EQ(tTester.getMarkNodeIndex(1,1,0), 255);
    EXPECT_EQ(tTester.getMarkNodeIndex(1,1,1), 415);
    EXPECT_EQ(tTester.getMarkNodeIndex(1,1,2), 151);
}

TEST(PlatoTestSu2ToExodus, createNodeSetFromMark)
{
    Su2ToExodus::Su2ToExodus_UnitTester tTester;
    std::istringstream tStringStream;
    std::string tStringInput;

    tTester.setNumMarks(2);
    tStringInput = "MARKER_TAG= 1\n"
            "MARKER_ELEMS= 2\n"
            "5 223 177 178\n"
            "5 255 415 151\n"
            "MARKER_TAG= 5\n"
            "MARKER_ELEMS= 3\n"
            "5 223 177 178\n"
            "5 223 415 178\n"
            "5 178 5 7\n";
    tStringStream.str(tStringInput);
    bool tVal = tTester.publicReadMarks(tStringStream);
    EXPECT_EQ(tVal, true);
    int tIndex = 0;
    std::string tName = "dummy";
    tVal = tTester.publicCreateNodeSetFromMark(tIndex, tName);
    EXPECT_EQ(tVal, true);
    tIndex = 1;
    tVal = tTester.publicCreateNodeSetFromMark(tIndex, tName);
    EXPECT_EQ(tVal, true);

    EXPECT_EQ(tTester.getNodeSetNode(0,0), 152);
    EXPECT_EQ(tTester.getNodeSetNode(0,1), 178);
    EXPECT_EQ(tTester.getNodeSetNode(0,2), 179);
    EXPECT_EQ(tTester.getNodeSetNode(0,3), 224);
    EXPECT_EQ(tTester.getNodeSetNode(0,4), 256);
    EXPECT_EQ(tTester.getNodeSetNode(0,5), 416);

    EXPECT_EQ(tTester.getNodeSetNode(1,0), 6);
    EXPECT_EQ(tTester.getNodeSetNode(1,1), 8);
    EXPECT_EQ(tTester.getNodeSetNode(1,2), 178);
    EXPECT_EQ(tTester.getNodeSetNode(1,3), 179);
    EXPECT_EQ(tTester.getNodeSetNode(1,4), 224);
    EXPECT_EQ(tTester.getNodeSetNode(1,5), 416);
}

TEST(PlatoTestSu2ToExodus, createSideSetFromMark)
{
    Su2ToExodus::Su2ToExodus_UnitTester tTester;
    std::istringstream tStringStream;
    std::string tStringInput;

    // Read nodal coords
    tTester.setNumNodes(4);
    tTester.setNumDimensions(3);
    tStringInput = "6.6 7.7 8.8 0\n"
                   "2.3 2.2 3.4 1\n"
                   "3.3 3.2 4.4 2\n"
                   "6.3 10.2 -3.4 3";
    tStringStream.str(tStringInput);
    bool tVal = tTester.publicReadNodeCoordinates(tStringStream);
    EXPECT_EQ(tVal, true);

    // Read element connectivity
    tTester.setNumElements(1);
    tStringInput = "10 1 3 2 0 0";
    tStringStream.str(tStringInput);
    tStringStream.clear();
    tStringStream.seekg (0);
    tVal = tTester.publicReadElementConnectivity(tStringStream);
    EXPECT_EQ(tVal, true);

    // Read marks
    tTester.setNumMarks(1);
    tStringInput = "MARKER_TAG= 1\n"
            "MARKER_ELEMS= 1\n"
            "5 1 2 3\n"
            "5 0 3 2";
    tStringStream.str(tStringInput);
    tStringStream.clear();
    tStringStream.seekg (0);
    tVal = tTester.publicReadMarks(tStringStream);
    EXPECT_EQ(tVal, true);

    tTester.publicCreateFaceToTetMap();

    std::string tName = "dummy";
    int tIndex = 0;
    tVal = tTester.publicCreateSideSetFromMark(tIndex, tName);
    EXPECT_EQ(tVal, true);

    EXPECT_EQ(tTester.getSideSetElem(0,0), 1);
    EXPECT_EQ(tTester.getSideSetSide(0,0), 4);
}

TEST(PlatoTestSu2ToExodus, getFaceIndex)
{
    Su2ToExodus::Su2ToExodus_UnitTester tTester;
    std::istringstream tStringStream;
    std::string tStringInput;

    // Read element connectivity
    tTester.setNumElements(3);
    tStringInput = "10 1 3 2 0 0\n"
                   "10 4 7 88 100 1\n"
                   "10 20 24 344 9 2";
    tStringStream.str(tStringInput);
    bool tVal = tTester.publicReadElementConnectivity(tStringStream);
    EXPECT_EQ(tVal, true);

    std::vector<int> tFaceConn(3);
    tFaceConn[0] = 0;
    tFaceConn[1] = 1;
    tFaceConn[2] = 3;
    int tElemIndex = 0;
    int tFaceIndex = tTester.publicGetFaceIndex(tElemIndex, tFaceConn);
    EXPECT_EQ(tFaceIndex, 0);
    tFaceConn[0] = 4;
    tFaceConn[1] = 88;
    tFaceConn[2] = 100;
    tElemIndex = 1;
    tFaceIndex = tTester.publicGetFaceIndex(tElemIndex, tFaceConn);
    EXPECT_EQ(tFaceIndex, 2);
    tFaceConn[0] = 20;
    tFaceConn[1] = 24;
    tFaceConn[2] = 344;
    tElemIndex = 2;
    tFaceIndex = tTester.publicGetFaceIndex(tElemIndex, tFaceConn);
    EXPECT_EQ(tFaceIndex, 3);
}

TEST(PlatoTestSu2ToExodus, readSu2FileFromStream)
{
    Su2ToExodus::Su2ToExodus_UnitTester tTester;
    std::istringstream tStringStream;
    std::string tStringInput;

    tStringInput = "NDIME=3\n"
                   "NELEM=500";
    tStringStream.str(tStringInput);
    bool tVal = tTester.publicReadSu2FileFromStream(tStringStream);
    EXPECT_EQ(tVal, false);

    tStringInput = "NDIMEx=3\n"
                   "NELEM=500";
    tStringStream.str(tStringInput);
    tStringStream.clear();
    tStringStream.seekg (0);
    tVal = tTester.publicReadSu2FileFromStream(tStringStream);
    EXPECT_EQ(tVal, false);

    tStringInput = "NDIME=3\n"
                   "NELEM=2\n"
                   "10 1715 1680 3482 1679 8\n"
                   "10 11769 4079 4078 44 9\n"
                   "NPOIN=3\n"
                   "0.0 1.0 2.0 5\n"
                   "3.3 3.4 3.5 6\n"
                   "2.2 2.3 2.4 7\n"
                   "NMARK=2\n"
                   "MARKER_TAG= 1\n"
                   "MARKER_ELEMS= 2\n"
                   "5 223 177 178\n"
                   "5 255 415 151\n"
                   "MARKER_TAG= 5\n"
                   "MARKER_ELEMS= 3\n"
                   "5 223 177 178\n"
                   "5 255 415 151\n"
                   "5 3 5 7\n";
    tStringStream.str(tStringInput);
    tStringStream.clear();
    tStringStream.seekg (0);
    tVal = tTester.publicReadSu2FileFromStream(tStringStream);
    EXPECT_EQ(tVal, true);
}

} // end PlatoTestSu2ToExodus namespace
