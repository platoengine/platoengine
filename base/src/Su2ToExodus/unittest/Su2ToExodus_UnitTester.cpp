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
 * Su2ToExodus_UnitTester.cpp
 *
 *  Created on: Oct 31, 2019
 *
 */

#include "Su2ToExodus_UnitTester.hpp"
#include <fstream>
#include <cstdlib>

namespace Su2ToExodus
{

/******************************************************************************/
int Su2ToExodus_UnitTester::publicGetNamedIntegerField(std::istream &aStream, const char *aName)
/******************************************************************************/
{
    return getNamedIntegerField(aStream, aName);
}
/******************************************************************************/
bool Su2ToExodus_UnitTester::publicReadSu2FileFromStream(std::istream &aStream)
/******************************************************************************/
{
    return readSu2FileFromStream(aStream);
}
/******************************************************************************/
bool Su2ToExodus_UnitTester::publicReadElementConnectivity(std::istream &aStream)
/******************************************************************************/
{
    return readElementConnectivity(aStream);
}
/******************************************************************************/
bool Su2ToExodus_UnitTester::publicReadNodeCoordinates(std::istream &aStream)
/******************************************************************************/
{
    return readNodeCoordinates(aStream);
}
/******************************************************************************/
bool Su2ToExodus_UnitTester::publicReadMarks(std::istream &aStream)
/******************************************************************************/
{
    return readMarks(aStream);
}
/******************************************************************************/
bool Su2ToExodus_UnitTester::publicCreateNodeSetFromMark(int &aMarkIndex, std::string &aName)
/******************************************************************************/
{
    return createNodeSetFromMark(aMarkIndex, aName);
}
/******************************************************************************/
bool Su2ToExodus_UnitTester::publicCreateSideSetFromMark(int &aMarkIndex, std::string &aName)
/******************************************************************************/
{
    return createSideSetFromMark(aMarkIndex, aName);
}
/******************************************************************************/
bool Su2ToExodus_UnitTester::publicCreateFaceToTetMap()
/******************************************************************************/
{
    return createFaceToTetMap();
}
/******************************************************************************/
int Su2ToExodus_UnitTester::publicGetFaceIndex(int &aConnectedElem, std::vector<int> &aFaceConn)
/******************************************************************************/
{
    return getFaceIndex(aConnectedElem, aFaceConn);
}

} // namespace Su2ToExodus


