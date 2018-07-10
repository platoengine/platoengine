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
