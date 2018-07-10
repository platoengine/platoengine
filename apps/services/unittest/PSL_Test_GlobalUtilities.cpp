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

#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Random.hpp"

#include <iostream>
#include <vector>

namespace PlatoSubproblemLibrary
{
namespace GlobalUtilitiesTesting
{

PSL_TEST(GlobalUtilities, basic)
{
    set_rand_seed();
    example::Interface_BasicGlobalUtilities interface;

    // test print
    interface.print("a pleasant message\n");

    // test that an abort has not yet occurred
    EXPECT_EQ(interface.get_ignored_abort(), false);

    // disable abort
    interface.set_never_abort(true);
    interface.fatal_error("a fatal message\n");

    // test that an abort should have occurred
    EXPECT_EQ(interface.get_ignored_abort(), true);
    interface.set_never_abort(false);
}

PSL_TEST(GlobalUtilities, vectors)
{
    set_rand_seed();
    example::Interface_BasicGlobalUtilities interface;
    AbstractInterface::GlobalUtilities* abs_interface = &interface;

    // print some vectors
    const std::vector<double> v_double = {0.1, -3.0, 4.2};
    abs_interface->print(v_double, false);

    // print some vectors
    const std::vector<bool> v_bool = {true, false, true, false};
    abs_interface->print(v_bool, true);

    // print some vectors
    const std::vector<double> v_int = {-3, 1, 4, 2, -1, 0, 7};
    abs_interface->print(v_int, true);
}

}
}
