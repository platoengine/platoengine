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
 * Plato_Test_AugmentedLagrangian.cpp
 *
 *  Created on: Oct 21, 2017
 */

#include "gtest/gtest.h"

#include "Plato_UnitTestUtils.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_Himmelblau.hpp"
#include "Plato_GoldsteinPrice.hpp"
#include "Plato_ShiftedEllipse.hpp"

#include <cstddef>

namespace PlatoTest
{

TEST(PlatoTest, EvaluateHimmelblau)
{
    // establish criterion
    Plato::Himmelblau<double> criterion;

    // working vector
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::StandardMultiVector<double> tControlVector(tNumVectors, tNumControls);
    Plato::StandardMultiVector<double> tGradientVector(tNumVectors, tNumControls);
    Plato::StandardMultiVector<double> tHessianVector(tNumVectors, tNumControls);
    const double tTolerance = 1e-4;

    // evaluate at minimums
    tControlVector(0, 0) = 3.;
    tControlVector(0, 1) = 2.;
    EXPECT_NEAR(0., criterion.value(tControlVector), tTolerance);
    criterion.gradient(tControlVector, tGradientVector);
    EXPECT_NEAR(0., tGradientVector[0].dot(tGradientVector[0]), tTolerance);

    tControlVector(0, 0) = -2.805118;
    tControlVector(0, 1) = 3.131312;
    EXPECT_NEAR(0., criterion.value(tControlVector), tTolerance);
    criterion.gradient(tControlVector, tGradientVector);
    EXPECT_NEAR(0., tGradientVector[0].dot(tGradientVector[0]), tTolerance);

    tControlVector(0, 0) = -3.779310;
    tControlVector(0, 1) = -3.283186;
    EXPECT_NEAR(0., criterion.value(tControlVector), tTolerance);
    criterion.gradient(tControlVector, tGradientVector);
    EXPECT_NEAR(0., tGradientVector[0].dot(tGradientVector[0]), tTolerance);

    tControlVector(0, 0) = 3.584428;
    tControlVector(0, 1) = -1.848126;
    EXPECT_NEAR(0., criterion.value(tControlVector), tTolerance);
    criterion.gradient(tControlVector, tGradientVector);
    EXPECT_NEAR(0., tGradientVector[0].dot(tGradientVector[0]), tTolerance);

    // evaluate at non-minimums
    tControlVector(0, 0) = 0.;
    tControlVector(0, 1) = 0.;
    EXPECT_NEAR(170., criterion.value(tControlVector), tTolerance);
    criterion.gradient(tControlVector, tGradientVector);
    EXPECT_NEAR(680., tGradientVector[0].dot(tGradientVector[0]), tTolerance);

    tControlVector(0, 0) = 1.;
    tControlVector(0, 1) = 2.;
    EXPECT_NEAR(68., criterion.value(tControlVector), tTolerance);
    criterion.gradient(tControlVector, tGradientVector);
    EXPECT_NEAR(2320., tGradientVector[0].dot(tGradientVector[0]), tTolerance);

    // evaluate Hessian
    tControlVector(0, 0) = .2;
    tControlVector(0, 1) = -.6;
    tGradientVector(0, 0) = .5;
    tGradientVector(0, 1) = -.4;
    criterion.hessian(tControlVector, tGradientVector, tHessianVector);
    EXPECT_NEAR(511.575104, tHessianVector[0].dot(tHessianVector[0]), tTolerance);
}

TEST(PlatoTest, EvaluateGoldsteinPrice)
{
    // establish criterion
    Plato::GoldsteinPrice<double> criterion;

    // working vector
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::StandardMultiVector<double> tControlVector(tNumVectors, tNumControls);
    Plato::StandardMultiVector<double> tGradientVector(tNumVectors, tNumControls);
    Plato::StandardMultiVector<double> tHessianVector(tNumVectors, tNumControls);
    const double tTolerance = 5e-3;

    // evaluate at minimum
    tControlVector(0, 0) = 0;
    tControlVector(0, 1) = -1;
    EXPECT_NEAR(3., criterion.value(tControlVector), tTolerance);
    criterion.gradient(tControlVector, tGradientVector);
    EXPECT_NEAR(0., tGradientVector[0].dot(tGradientVector[0]), tTolerance);

    // evaluate at non-minimums
    tControlVector(0, 0) = 1.5;
    tControlVector(0, 1) = .5;
    EXPECT_NEAR(887.25, criterion.value(tControlVector), tTolerance);
    criterion.gradient(tControlVector, tGradientVector);
    EXPECT_NEAR(1857492., tGradientVector[0].dot(tGradientVector[0]), tTolerance);

    // evaluate Hessian
    tControlVector(0, 0) = .2;
    tControlVector(0, 1) = -.6;
    tGradientVector(0, 0) = .5;
    tGradientVector(0, 1) = -.4;
    criterion.hessian(tControlVector, tGradientVector, tHessianVector);
    EXPECT_NEAR(3390355.065, tHessianVector[0].dot(tHessianVector[0]), tTolerance);
}

TEST(PlatoTest, EvaluateShiftedEllipse)
{
    // establish criterion
    Plato::ShiftedEllipse<double> criterion;
    criterion.specify(.1, 1.2, -.5, .7);

    // working vector
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::StandardMultiVector<double> tControlVector(tNumVectors, tNumControls);
    Plato::StandardMultiVector<double> tGradientVector(tNumVectors, tNumControls);
    Plato::StandardMultiVector<double> tHessianVector(tNumVectors, tNumControls);
    const double tTolerance = 1e-4;

    // value
    tControlVector(0, 0) = 1.3;
    tControlVector(0, 1) = -2.1;
    EXPECT_NEAR(5.224489796, criterion.value(tControlVector), tTolerance);

    // gradient
    criterion.gradient(tControlVector, tGradientVector);
    EXPECT_NEAR(45.42667409, tGradientVector[0].dot(tGradientVector[0]), tTolerance);

    // Hessian
    tGradientVector(0, 0) = .5;
    tGradientVector(0, 1) = -.4;
    criterion.hessian(tControlVector, tGradientVector, tHessianVector);
    EXPECT_NEAR(3.147809106, tHessianVector[0].dot(tHessianVector[0]), tTolerance);
}

} // namespace PlatoTest
