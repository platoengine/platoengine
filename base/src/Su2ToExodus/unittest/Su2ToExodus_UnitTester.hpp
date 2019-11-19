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
 * Su2ToExodus_UnitTester.hpp
 *
 *  Created on: Oct 31, 2019
 *
 */

#ifndef SU2TOEXODUS_UNITTESTER_HPP_
#define SU2TOEXODUS_UNITTESTER_HPP_

#include <string>
#include "Su2ToExodus.hpp"

namespace Su2ToExodus
{

class Su2ToExodus_UnitTester : public Su2ToExodus
{

public:
    Su2ToExodus_UnitTester(){}
    ~Su2ToExodus_UnitTester(){}

    int publicGetNamedIntegerField(std::istream &aStream, const char *aName);
    bool publicReadSu2FileFromStream(std::istream &aStream);
    bool publicReadElementConnectivity(std::istream &aStream);
    bool publicReadNodeCoordinates(std::istream &aStream);
    bool publicReadMarks(std::istream &aStream);
    bool publicCreateNodeSetFromMark(int &aMarkIndex, std::string &aName);
    bool publicCreateSideSetFromMark(int &aMarkIndex, std::string &aName);
    bool publicCreateFaceToTetMap();
    int publicGetFaceIndex(int &aConnectedElem, std::vector<int> &aFaceConn);


protected:

private:


};

} // namespace Su2ToExodus_UnitTester

#endif /* SU2TOEXODUS_UNITTESTER_HPP_ */
