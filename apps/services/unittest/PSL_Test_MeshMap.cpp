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

// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_MeshMap.hpp"

#include <vector>
#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

using MeshMapType = MeshMap<double,int>;

PSL_TEST(MeshMap,construction)
{
    std::vector<std::vector<double>> tCoordinates;

    std::vector<std::vector<int>> tConnectivity;

    // tCoordinates empty
    EXPECT_THROW(MeshMapType tMeshMap(tCoordinates,tConnectivity),std::domain_error);

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    // tConnectivity empty
    EXPECT_THROW(MeshMapType tMeshMap(tCoordinates,tConnectivity),std::domain_error);

    tConnectivity.push_back({0,1,2,3});

    // valid construction
    EXPECT_NO_THROW(MeshMapType tMeshMap(tCoordinates,tConnectivity));

    // wrong size coordinate vector
    tCoordinates.push_back({0.0,1.0,1.0,0.0});
    EXPECT_THROW(MeshMapType tMeshMap(tCoordinates,tConnectivity),std::domain_error);

    // wrong size connectivity vector
    tCoordinates.pop_back();
    tConnectivity.push_back({0,1,2});
    EXPECT_THROW(MeshMapType tMeshMap(tCoordinates,tConnectivity),std::domain_error);

    // index in connectivity out of range of coordinate vector
    tConnectivity.pop_back();
    tConnectivity.push_back({1,2,3,4});
    EXPECT_THROW(MeshMapType tMeshMap(tCoordinates,tConnectivity),std::out_of_range);
}


}
}
