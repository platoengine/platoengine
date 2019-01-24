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
 //S
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
 * Plato_Test_GradFreeCriteria.cpp
 *
 *  Created on: Jan 23, 2019
 */

#include "gtest/gtest.h"

#include "Plato_StandardVector.hpp"
#include "Plato_StandardMultiVector.hpp"

#include "Plato_GradFreeCircle.hpp"
#include "Plato_GradFreeRadius.hpp"
#include "Plato_GradFreeHimmelblau.hpp"
#include "Plato_GradFreeRosenbrock.hpp"
#include "Plato_GradFreeRocketObjFunc.hpp"
#include "Plato_GradFreeShiftedEllipse.hpp"
#include "Plato_GradFreeGoldsteinPrice.hpp"

namespace GradFreeCriteriaTest
{

TEST(PlatoTest, GradFreeRadius)
{
    // ********* Allocate Core Optimization Data Templates *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 3;
    Plato::StandardMultiVector<double> tControls(tNumParticles, tNumControls);
    tControls[0].fill(0.705);
    tControls[1].fill(0.695);
    tControls[2].fill(0.725);

    // TEST OBJECTIVE
    Plato::StandardVector<double> tObjVals(tNumParticles);
    Plato::GradFreeRadius<double> tCriterion;
    tCriterion.value(tControls, tObjVals);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(-0.00595, tObjVals[0], tTolerance);
    EXPECT_NEAR(-0.03395, tObjVals[1], tTolerance);
    EXPECT_NEAR(0.05125, tObjVals[2], tTolerance);
}

TEST(PlatoTest, GradFreeRosenbrock)
{
    // ********* Allocate Core Optimization Data Templates *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 3;
    Plato::StandardMultiVector<double> tControls(tNumParticles, tNumControls);
    tControls[0].fill(2);
    tControls[1].fill(0.95);
    tControls[2].fill(1);

    // TEST OBJECTIVE
    Plato::StandardVector<double> tObjVals(tNumParticles);
    Plato::GradFreeRosenbrock<double> tCriterion;
    tCriterion.value(tControls, tObjVals);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(401, tObjVals[0], tTolerance);
    EXPECT_NEAR(0.228125, tObjVals[1], tTolerance);
    EXPECT_NEAR(0, tObjVals[2], tTolerance);
}

TEST(PlatoTest, GradFreeGoldsteinPrice)
{
    // establish criterion
    Plato::GradFreeGoldsteinPrice<double> tCriterion;

    // working vector
    const size_t tNumControls = 2;
    const size_t tNumParticles = 2;
    Plato::StandardVector<double> tObjVals(tNumParticles);
    Plato::StandardMultiVector<double> tControls(tNumParticles, tNumControls);
    tControls(0, 0) = 0.; tControls(1, 0) = 1.5;
    tControls(0, 1) = -1; tControls(1, 1) = 0.5;

    // evaluate
    tCriterion.value(tControls, tObjVals);

    // evaluate at minimum
    const double tTolerance = 5e-3;
    EXPECT_NEAR(3., tObjVals[0], tTolerance);

    // evaluate at non-minimums
    EXPECT_NEAR(887.25, tObjVals[1], tTolerance);
}


TEST(PlatoTest, GradFreeHimmelblau)
{
    // working vector
    const size_t tNumControls = 2;
    const size_t tNumParticles = 6;
    Plato::StandardVector<double> tCriterionValues(tNumParticles);
    Plato::StandardMultiVector<double> tParticles(tNumParticles, tNumControls);

    // define particles
    tParticles(0 /*particle index*/, 0 /*control index*/) = 3;
    tParticles(0 /*particle index*/, 1 /*control index*/) = 2;
    tParticles(1 /*particle index*/, 0 /*control index*/) = -2.805118;
    tParticles(1 /*particle index*/, 1 /*control index*/) = 3.131312;
    tParticles(2 /*particle index*/, 0 /*control index*/) = -3.779310;
    tParticles(2 /*particle index*/, 1 /*control index*/) = -3.283186;
    tParticles(3 /*particle index*/, 0 /*control index*/) = 3.584428;
    tParticles(3 /*particle index*/, 1 /*control index*/) = -1.848126;
    tParticles(4 /*particle index*/, 0 /*control index*/) = 0;
    tParticles(4 /*particle index*/, 1 /*control index*/) = 0;
    tParticles(5 /*particle index*/, 0 /*control index*/) = 1;
    tParticles(5 /*particle index*/, 1 /*control index*/) = 2;

    Plato::GradFreeHimmelblau<double> tHimmelblau;
    tHimmelblau.value(tParticles, tCriterionValues);

    // evaluate at minimums
    const double tTolerance = 1e-4;
    EXPECT_NEAR(0., tCriterionValues[0 /*particle index*/], tTolerance);
    EXPECT_NEAR(0., tCriterionValues[1 /*particle index*/], tTolerance);
    EXPECT_NEAR(0., tCriterionValues[2 /*particle index*/], tTolerance);
    EXPECT_NEAR(0., tCriterionValues[3 /*particle index*/], tTolerance);
    // evaluate at non-minimums
    EXPECT_NEAR(170., tCriterionValues[4 /*particle index*/], tTolerance);
    EXPECT_NEAR(68., tCriterionValues[5 /*particle index*/], tTolerance);
}

TEST(PlatoTest, GradFreeShiftedEllipse)
{
    // working vector
    const size_t tNumParticles = 2;
    const size_t tNumControls = 2;
    Plato::StandardVector<double> tCriterionValues(tNumParticles);
    Plato::StandardMultiVector<double> tParticles(tNumParticles, tNumControls);

    // define particles
    tParticles(0 /*particle index*/, 0 /*control index*/) = 1.3;
    tParticles(0 /*particle index*/, 1 /*control index*/) = -2.1;
    tParticles(1 /*particle index*/, 0 /*control index*/) = -1.3;
    tParticles(1 /*particle index*/, 1 /*control index*/) = -2.1;

    // define criterion
    Plato::GradFreeShiftedEllipse<double> tShiftedEllipse;
    tShiftedEllipse.define(0.1 /*center x-coord*/, 1.2 /*center y-coord*/, -.5 /*x-radius*/, 0.7 /*y-radius*/);

    // evaluate criterion
    tShiftedEllipse.value(tParticles, tCriterionValues);
    const double tTolerance = 1e-4;
    EXPECT_NEAR(5.224489796, tCriterionValues[0 /*particle index*/], tTolerance);
    EXPECT_NEAR(5.585600907, tCriterionValues[1 /*particle index*/], tTolerance);
}

TEST(PlatoTest, GradFreeCircle)
{
    const size_t tNumControls = 2;
    const size_t tNumParticles = 2;
    Plato::StandardVector<double> tCriterionValues(tNumParticles);
    Plato::StandardMultiVector<double> tParticles(tNumParticles, tNumControls);

    // define particles
    tParticles(0 /*particle index*/, 0 /*control index*/) = 1;
    tParticles(0 /*particle index*/, 1 /*control index*/) = 1;
    tParticles(1 /*particle index*/, 0 /*control index*/) = 0.5;
    tParticles(1 /*particle index*/, 1 /*control index*/) = 1.2;

    // evaluate criterion
    Plato::GradFreeCircle<double> tCircle;

    // TEST OBJECTIVE FUNCTION EVALUATION
    tCircle.value(tParticles, tCriterionValues);
    const double tTolerance = 1e-4;
    EXPECT_NEAR(0.3825, tCriterionValues[0 /*particle index*/], tTolerance);
    EXPECT_NEAR(0.7825, tCriterionValues[1 /*particle index*/], tTolerance);
}

} // namespace GradFreeCriteriaTest
