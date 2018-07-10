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

#include "PSL_FreeHelpers.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_DistanceVoterFactory.hpp"
#include "PSL_DistanceVoter.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingDistanceVoter
{

PSL_TEST(DistanceVoter,buildVoterFromFactory)
{
    set_rand_seed();
    // allocate
    AbstractAuthority authority;

    // for each type to try
    DistanceVoter* voter = NULL;
    const int num_types = distance_voter_t::distance_voter_t::TOTAL_NUM_DISTANCE_VOTERS;
    for(int t = 0; t < num_types; t++)
    {
        const distance_voter_t::distance_voter_t this_type = distance_voter_t::distance_voter_t(t);
        voter = build_distance_voter(this_type, &authority);
        EXPECT_EQ(voter->get_distance_voter_type(), this_type);
        safe_free(voter);
    }
}
void test_DistanceVoter_expectedVotes(const int& num_classes,
                                      const std::vector<double>& distances_per_instance,
                                      const std::vector<int>& classes_per_instance,
                                      const std::vector<int>& expected_votes)
{
    // allocate
    AbstractAuthority authority;

    // for each type to try
    DistanceVoter* voter = NULL;
    const int num_types = distance_voter_t::distance_voter_t::TOTAL_NUM_DISTANCE_VOTERS;
    for(int t = 0; t < num_types; t++)
    {
        const distance_voter_t::distance_voter_t this_type = distance_voter_t::distance_voter_t(t);
        voter = build_distance_voter(this_type, &authority);

        // compare vote
        const int actual_vote = voter->choose_output(num_classes, distances_per_instance, classes_per_instance);
        EXPECT_EQ(actual_vote, expected_votes[t]);

        safe_free(voter);
    }
}
PSL_TEST(DistanceVoter,countSeparatingExample)
{
    set_rand_seed();
    // pose problem
    const int num_classes = 5;
    const std::vector<double> distances_per_instance = {.5, .8, 1.2, .7, .2, .1, .05, .2};
    const std::vector<int> classes_per_instance = {0, 0, 2, 2, 2, 3, 3, 4};

    // expected votes
    const std::vector<int> expected_votes = {2, 3, 3};
    test_DistanceVoter_expectedVotes(num_classes, distances_per_instance, classes_per_instance, expected_votes);
}
PSL_TEST(DistanceVoter,linearSeparatingExample)
{
    set_rand_seed();
    // pose problem
    const int num_classes = 3;
    const std::vector<double> distances_per_instance = {.6, .06, .2, .3, .5, .15, .4};
    const std::vector<int> classes_per_instance = {2, 0, 1, 2, 0, 1, 0};

    // expected votes
    const std::vector<int> expected_votes = {0, 1, 0};
    test_DistanceVoter_expectedVotes(num_classes, distances_per_instance, classes_per_instance, expected_votes);
}
PSL_TEST(DistanceVoter,inverseSeparatingExample)
{
    set_rand_seed();
    // pose problem
    const int num_classes = 4;
    const std::vector<double> distances_per_instance = {.14, .02, .008, .28, .38, .17, .06};
    const std::vector<int> classes_per_instance = {1, 2, 0, 1, 2, 0, 2};

    // expected votes
    const std::vector<int> expected_votes = {2, 2, 0};
    test_DistanceVoter_expectedVotes(num_classes, distances_per_instance, classes_per_instance, expected_votes);
}
PSL_TEST(DistanceVoter,zeroDistance)
{
    set_rand_seed();
    // pose problem
    const int num_classes = 2;
    const std::vector<double> distances_per_instance = {.5, 0., .1};
    const std::vector<int> classes_per_instance = {0, 1, 0};

    // expected votes
    const std::vector<int> expected_votes = {0, 1, 1};
    test_DistanceVoter_expectedVotes(num_classes, distances_per_instance, classes_per_instance, expected_votes);
}
PSL_TEST(DistanceVoter,allZeroDistance)
{
    set_rand_seed();
    // pose problem
    const int num_classes = 3;
    const std::vector<double> distances_per_instance = {0., 0., 0., 0., 0.};
    const std::vector<int> classes_per_instance = {1, 0, 0, 1, 0};

    // expected votes
    const std::vector<int> expected_votes = {0, 0, 0};
    test_DistanceVoter_expectedVotes(num_classes, distances_per_instance, classes_per_instance, expected_votes);
}

}
}
