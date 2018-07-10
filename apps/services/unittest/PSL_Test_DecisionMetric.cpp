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

#include "PSL_UnitTestingHelper.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_DecisionMetric.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_GiniImpurityMetric.hpp"
#include "PSL_EntropyMetric.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_DecisionMetricFactory.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <iomanip>
#include <iostream>

namespace PlatoSubproblemLibrary
{
namespace TestingDecisionMetric
{

PSL_TEST(DecisionMetric,decisionMetricFactory)
{
    set_rand_seed();
    // allocate
    example::Interface_BasicGlobalUtilities utilities;

    // list types
    std::vector<decision_metric_t::decision_metric_t> types = {decision_metric_t::decision_metric_t::gini_impurity_metric,
                                                               decision_metric_t::decision_metric_t::entropy_metric};

    // for each type to try
    DecisionMetric* dm = NULL;
    const int num_types = types.size();
    for(int t = 0; t < num_types; t++)
    {
        dm = build_decision_metric(types[t], &utilities);
        EXPECT_EQ(dm->get_type(), types[t]);
        safe_free(dm);
    }
}

PSL_TEST(DecisionMetric,someExampleClassMembers)
{
    set_rand_seed();
    example::Interface_BasicGlobalUtilities utilities;

    // some examples
    const std::vector<int> bad_input0 = {};
    const std::vector<int> bad_input1 = {0};
    const std::vector<int> bad_input2 = {0, 0, 0, 0};
    const std::vector<int> input0 = {0, 42, 0};
    const std::vector<int> input1 = {5, 0, 8, 7};
    const std::vector<int> input2 = {2, 0, 15, 3};
    const std::vector<int> input3 = {5, 20, 0, 2, 3};
    const std::vector<int> input4 = {2, 26, 0, 1, 1};
    const std::vector<int> input5 = {10, 130, 0, 5, 5};
    const std::vector<int> input6 = {20, 2, 1, 0, 0};
    const std::vector<int> input7 = {100, 5, 2, 1, 0};
    const std::vector<int> input8 = {500, 10, 5, 2, 1};

    // test gini
    DecisionMetric* gini = build_decision_metric(decision_metric_t::decision_metric_t::gini_impurity_metric, &utilities);
    EXPECT_FLOAT_EQ(gini->measure(bad_input0), 0.);
    EXPECT_FLOAT_EQ(gini->measure(bad_input1), 0.);
    EXPECT_FLOAT_EQ(gini->measure(bad_input2), 0.);
    EXPECT_FLOAT_EQ(gini->measure(input0), 0.);
    EXPECT_FLOAT_EQ(gini->measure(input1), .655);
    EXPECT_FLOAT_EQ(gini->measure(input2), .405);
    EXPECT_FLOAT_EQ(gini->measure(input3), .5133333333);
    EXPECT_FLOAT_EQ(gini->measure(input4), .2422222222);
    EXPECT_FLOAT_EQ(gini->measure(input5), .2422222222);
    EXPECT_FLOAT_EQ(gini->measure(input6), .2344045369);
    EXPECT_FLOAT_EQ(gini->measure(input7), .1400891632);
    EXPECT_FLOAT_EQ(gini->measure(input8), .0678060852);
    safe_free(gini);

    // test entropy
    DecisionMetric* entropy = build_decision_metric(decision_metric_t::decision_metric_t::entropy_metric, &utilities);
    EXPECT_FLOAT_EQ(entropy->measure(bad_input0), 0.);
    EXPECT_FLOAT_EQ(entropy->measure(bad_input1), 0.);
    EXPECT_FLOAT_EQ(entropy->measure(bad_input2), 0.);
    EXPECT_FLOAT_EQ(entropy->measure(input0), 0.);
    EXPECT_FLOAT_EQ(entropy->measure(input1), 1.558871848);
    EXPECT_FLOAT_EQ(entropy->measure(input2), 1.054015773);
    EXPECT_FLOAT_EQ(entropy->measure(input3), 1.413454266);
    EXPECT_FLOAT_EQ(entropy->measure(input4), .7665095066);
    EXPECT_FLOAT_EQ(entropy->measure(input5), .7665095066);
    EXPECT_FLOAT_EQ(entropy->measure(input6), .6784070909);
    EXPECT_FLOAT_EQ(entropy->measure(input7), .4771536183);
    EXPECT_FLOAT_EQ(entropy->measure(input8), .2721730295);
    safe_free(entropy);
}

}
}
