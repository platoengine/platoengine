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
#include <Plato_InitializeValues.hpp>
#include <Plato_InputData.hpp>

namespace PlatoTestOperations
{

TEST(InitializeValues, getValuesFromCSMFile)
{
    Plato::InputData tNode;
    Plato::InitializeValues tIntializeValuesOperation(NULL, tNode);

    std::istringstream tStringStream;
    std::string tStringInput;

    tStringInput =
            "# Constant, Design, and Output Parameters:\n"
            "despmtr   Px       2.00000 lbound 1.5  ubound 2.5\n"
            "despmtr   Py       2.00000 lbound 1.5  ubound 2.5\n"
            "despmtr   Pz       2.00000 lbound 1.5  ubound 2.5\n"
            "despmtr   Lx       5.00000 lbound 3.0  ubound 10.0\n"
            "despmtr   Rx       8.00000 lbound 3.0  ubound 10.0\n"
            "despmtr   Rcap     1.00000 lbound 1.5  ubound 2.5\n"
            "despmtr   Boffset  0.25000 lbound 0.05 ubound 0.49\n"
            "despmtr   Rp       0.70000 lbound 0.5  ubound 0.9\n"
            "#despmtr   Rp       0.70000 lbound 0.5  ubound 0.9*Px\n"
            "despmtr   Rpo      0.50000 lbound 0.1  ubound 0.9\n"
            "#despmtr   Rpo      0.50000 lbound 0.1  ubound 0.9*Rp\n"
            "despmtr   Rlo      0.30000 lbound 0.1  ubound 0.9\n"
            "#despmtr   Rlo      0.30000 lbound 0.1  ubound 0.9*Px/4.0\n"
            "despmtr   Po       0.20000 lbound 0.1  ubound 0.5\n";
    tStringStream.str(tStringInput);
    tIntializeValuesOperation.getValuesFromStream(tStringStream);
    EXPECT_EQ(tIntializeValuesOperation.getValue(0), 2.0);
    EXPECT_EQ(tIntializeValuesOperation.getValue(1), 2.0);
    EXPECT_EQ(tIntializeValuesOperation.getValue(2), 2.0);
    EXPECT_EQ(tIntializeValuesOperation.getValue(3), 5.0);
    EXPECT_EQ(tIntializeValuesOperation.getValue(4), 8.0);
    EXPECT_EQ(tIntializeValuesOperation.getValue(5), 1.0);
    EXPECT_EQ(tIntializeValuesOperation.getValue(6), .25);
    EXPECT_EQ(tIntializeValuesOperation.getValue(7), .7);
    EXPECT_EQ(tIntializeValuesOperation.getValue(8), .5);
    EXPECT_EQ(tIntializeValuesOperation.getValue(9), .3);
    EXPECT_EQ(tIntializeValuesOperation.getValue(10), .2);

    EXPECT_EQ(tIntializeValuesOperation.getValueUpperBound(0), 2.5);
    EXPECT_EQ(tIntializeValuesOperation.getValueUpperBound(1), 2.5);
    EXPECT_EQ(tIntializeValuesOperation.getValueUpperBound(2), 2.5);
    EXPECT_EQ(tIntializeValuesOperation.getValueUpperBound(3), 10.0);
    EXPECT_EQ(tIntializeValuesOperation.getValueUpperBound(4), 10.0);
    EXPECT_EQ(tIntializeValuesOperation.getValueUpperBound(5), 2.5);
    EXPECT_EQ(tIntializeValuesOperation.getValueUpperBound(6), .49);
    EXPECT_EQ(tIntializeValuesOperation.getValueUpperBound(7), .9);
    EXPECT_EQ(tIntializeValuesOperation.getValueUpperBound(8), .9);
    EXPECT_EQ(tIntializeValuesOperation.getValueUpperBound(9), .9);
    EXPECT_EQ(tIntializeValuesOperation.getValueUpperBound(10), .5);

    EXPECT_EQ(tIntializeValuesOperation.getValueLowerBound(0), 1.5);
    EXPECT_EQ(tIntializeValuesOperation.getValueLowerBound(1), 1.5);
    EXPECT_EQ(tIntializeValuesOperation.getValueLowerBound(2), 1.5);
    EXPECT_EQ(tIntializeValuesOperation.getValueLowerBound(3), 3.);
    EXPECT_EQ(tIntializeValuesOperation.getValueLowerBound(4), 3.0);
    EXPECT_EQ(tIntializeValuesOperation.getValueLowerBound(5), 1.5);
    EXPECT_EQ(tIntializeValuesOperation.getValueLowerBound(6), .05);
    EXPECT_EQ(tIntializeValuesOperation.getValueLowerBound(7), .5);
    EXPECT_EQ(tIntializeValuesOperation.getValueLowerBound(8), .1);
    EXPECT_EQ(tIntializeValuesOperation.getValueLowerBound(9), .1);
    EXPECT_EQ(tIntializeValuesOperation.getValueLowerBound(10), .1);
}


} // end PlatoTestOperations namespace
