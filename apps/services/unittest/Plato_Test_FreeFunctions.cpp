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
 * Plato_Test_FreeFunctions.cpp
 *
 */

#include "gtest/gtest.h"

#include "Plato_FreeFunctions.hpp"

#include <cstddef>

namespace PlatoTest
{

TEST(FreeFunctions, DivideUpProcessors)
{
    // first scenario
    EXPECT_EQ(4u, Plato::divide_up_atmost_processors(4u,3u,12u));
    EXPECT_EQ(3u, Plato::divide_up_atmost_processors(4u,3u,10u));
    EXPECT_EQ(3u, Plato::divide_up_atmost_processors(4u,3u,9u));
    EXPECT_EQ(2u, Plato::divide_up_atmost_processors(4u,3u,8u));

    // second scenario
    EXPECT_EQ(7u, Plato::divide_up_atmost_processors(7u,5u,43u));
    EXPECT_EQ(7u, Plato::divide_up_atmost_processors(7u,5u,42u));
    EXPECT_EQ(7u, Plato::divide_up_atmost_processors(7u,5u,41u));
    EXPECT_EQ(7u, Plato::divide_up_atmost_processors(7u,5u,36u));
    EXPECT_EQ(7u, Plato::divide_up_atmost_processors(7u,5u,35u));
    EXPECT_EQ(6u, Plato::divide_up_atmost_processors(7u,5u,34u));
    EXPECT_EQ(5u, Plato::divide_up_atmost_processors(7u,5u,26u));
    EXPECT_EQ(5u, Plato::divide_up_atmost_processors(7u,5u,25u));
    EXPECT_EQ(4u, Plato::divide_up_atmost_processors(7u,5u,24u));
}

} // namespace PlatoTest
