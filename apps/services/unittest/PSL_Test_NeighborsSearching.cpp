#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_KNearestNeighborsFactory.hpp"
#include "PSL_KNearestNeighborsSearcher.hpp"
#include "PSL_RadixHashByPrime.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_Random.hpp"

#include <mpi.h>
#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace TestingNeighborsSearching
{

typedef void (*knn_searcher_function)(AbstractAuthority*,const k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t&,KNearestNeighborsSearcher*);
void for_each_searcher_type(knn_searcher_function func)
{
    // allocate
    AbstractAuthority authority;
    DataSequence sequence(&authority, data_flow_t::data_flow_t::inert_data_flow);

    // for each type to try
    KNearestNeighborsSearcher* knn_searcher = NULL;
    const int num_types = k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t::TOTAL_NUM_SEARCHERS;
    for(int t = 0; t < num_types; t++)
    {
        const k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t this_type =
                k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t(t);
        knn_searcher = build_knn_searcher(this_type, &authority, &sequence);

        // do function
        func(&authority, this_type, knn_searcher);

        safe_free(knn_searcher);
    }
}

// confirm types from factory
void confirm_searcher_type(AbstractAuthority* authority,
                           const k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t& expected_type,
                           KNearestNeighborsSearcher* knn_searcher)
{
    EXPECT_EQ(expected_type, knn_searcher->get_searcher_type());
}
PSL_TEST(NeighborsSearching,buildNeighborsSearchingFromFactory)
{
    set_rand_seed();
    for_each_searcher_type(confirm_searcher_type);
}

// confirm against expected search
void confirm_basic_expected_search(AbstractAuthority* authority,
                                   const k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t& expected_type,
                                   KNearestNeighborsSearcher* knn_searcher)
{
    // fill answers
    const int dimension = 2;
    const int num_answers = 5;
    const std::vector<double> rowMajor_answerPoints = {0., 0., 0., 1., 1., 1., 1., 0., .5, .5};
    AbstractInterface::DenseMatrix* answer_points = authority->dense_builder->build_by_row_major(num_answers, dimension, rowMajor_answerPoints);
    const std::vector<int> answer_class_outputs = {0, 1, 2, 0, 1};

    // some parameter data
    ParameterData parameters;
    parameters.set_random_projection_forest_num_trees(30);
    knn_searcher->extract_parameters(&parameters);

    // build data
    knn_searcher->before_allocate(num_answers);
    knn_searcher->allocate_data();
    knn_searcher->unpack_data();
    knn_searcher->after_unpack(answer_points, answer_class_outputs);

    // build searcher
    knn_searcher->initialize();
    std::vector<double> worst_to_best_distances;
    std::vector<int> worst_to_best_class_outputs;

    // confirm against expectations
    const std::vector<double> trial0 = {1.15, 1.2};
    knn_searcher->get_neighbors(1, trial0, worst_to_best_distances, worst_to_best_class_outputs);
    ASSERT_EQ(worst_to_best_distances.size(), 1u);
    EXPECT_FLOAT_EQ(worst_to_best_distances[0], .25);
    ASSERT_EQ(worst_to_best_class_outputs.size(), 1u);
    EXPECT_EQ(worst_to_best_class_outputs[0], 2);

    // confirm against expectations
    const std::vector<double> trial1 = {.512, .484};
    knn_searcher->get_neighbors(1, trial1, worst_to_best_distances, worst_to_best_class_outputs);
    ASSERT_EQ(worst_to_best_distances.size(), 1u);
    EXPECT_FLOAT_EQ(worst_to_best_distances[0], .02);
    ASSERT_EQ(worst_to_best_class_outputs.size(), 1u);
    EXPECT_EQ(worst_to_best_class_outputs[0], 1);

    // confirm against expectations
    const std::vector<double> trial2 = {2.2, .5};
    knn_searcher->get_neighbors(3, trial2, worst_to_best_distances, worst_to_best_class_outputs);
    const std::vector<double> t2_wtbd_expected = {1.7, 1.3, 1.3};
    expect_equal_float_vectors(t2_wtbd_expected, worst_to_best_distances);
    ASSERT_EQ(worst_to_best_class_outputs.size(), 3u);
    EXPECT_EQ(worst_to_best_class_outputs[0], 1);
    if(worst_to_best_class_outputs[1] == 2)
    {
        EXPECT_EQ(worst_to_best_class_outputs[2], 0);
    }
    else
    {
        EXPECT_EQ(worst_to_best_class_outputs[1], 0);
        EXPECT_EQ(worst_to_best_class_outputs[2], 2);
    }

    // confirm against expectations
    const std::vector<double> trial3 = {.3, .125};
    knn_searcher->get_neighbors(2, trial3, worst_to_best_distances, worst_to_best_class_outputs);
    const std::vector<double> t3_wtbd_expected = {.425, .325};
    expect_equal_float_vectors(t3_wtbd_expected, worst_to_best_distances);
    const std::vector<int> t3_wtbco_expected = {1, 0};
    expect_equal_vectors(t3_wtbco_expected, worst_to_best_class_outputs);
}
PSL_TEST(NeighborsSearching,confirmBasicExpectedSearch)
{
    set_rand_seed();
    for_each_searcher_type(confirm_basic_expected_search);
}

typedef void (*knn_searcher_comparison_function)(AbstractAuthority*,KNearestNeighborsSearcher*,KNearestNeighborsSearcher*);
void compare_each_searcher_to_brute_force(knn_searcher_comparison_function func)
{
    // allocate
    AbstractAuthority authority;

    KNearestNeighborsSearcher* brute_force_searcher = NULL;
    DataSequence bruteForce_sequence(&authority, data_flow_t::data_flow_t::inert_data_flow);
    KNearestNeighborsSearcher* other_searcher = NULL;
    DataSequence other_sequence(&authority, data_flow_t::data_flow_t::inert_data_flow);

    // for each type to try
    const int num_types = k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t::TOTAL_NUM_SEARCHERS;
    const k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t brute_force_type =
            k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t::brute_force_searcher;
    for(int t = 0; t < num_types; t++)
    {
        // make brute force
        brute_force_searcher = build_knn_searcher(brute_force_type, &authority, &bruteForce_sequence);

        // make other searcher
        const k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t this_type =
                k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t(t);
        other_searcher = build_knn_searcher(this_type, &authority, &other_sequence);

        // do function
        func(&authority, brute_force_searcher, other_searcher);

        // clean up
        safe_free(brute_force_searcher);
        safe_free(other_searcher);
    }
}

// confirm against brute force search
void confirm_basic_against_brute_force_search(AbstractAuthority* authority,
                                              KNearestNeighborsSearcher* brute_force,
                                              KNearestNeighborsSearcher* other_searcher)
{
    ASSERT_EQ(true, brute_force->is_exact());

    // construct answers
    const int dimension = rand_int(10, 15);
    const int num_answers = rand_int(2000, 5000);
    std::vector<double> rowMajor_answerPoints(num_answers * dimension);
    uniform_rand_double(-uniform_rand_double() - .5, .2 + uniform_rand_double(), rowMajor_answerPoints);
    AbstractInterface::DenseMatrix* other_answer_points = authority->dense_builder->build_by_row_major(num_answers,
                                                                                                       dimension,
                                                                                                       rowMajor_answerPoints);
    std::vector<int> answer_class_outputs(num_answers);
    const int num_classes = rand_int(2, 9);
    rand_int(0, num_classes, answer_class_outputs);

    // answers for brute_force
    AbstractInterface::DenseMatrix* brute_force_answer_points =
            authority->dense_builder->build_by_row_major(num_answers, dimension, rowMajor_answerPoints);

    // some parameter data
    ParameterData parameters;
    parameters.set_random_projection_forest_approx_leaf_size(60);
    brute_force->extract_parameters(&parameters);
    other_searcher->extract_parameters(&parameters);

    // build data
    brute_force->before_allocate(num_answers);
    brute_force->allocate_data();
    brute_force->unpack_data();
    brute_force->after_unpack(brute_force_answer_points, answer_class_outputs);
    other_searcher->before_allocate(num_answers);
    other_searcher->allocate_data();
    other_searcher->unpack_data();
    other_searcher->after_unpack(other_answer_points, answer_class_outputs);

    // build searchers
    brute_force->initialize();
    other_searcher->initialize();
    std::vector<double> bruteForce_worstToBest_distances;
    std::vector<int> bruteForce_worstToBest_classOutputs;
    std::vector<double> other_worstToBest_distances;
    std::vector<int> other_worstToBest_classOutputs;

    double brute_time = 0.;
    double other_time = 0.;
    std::cout << "dimen:" << dimension << ",num_answers:" << num_answers << std::endl;

    // for each trial
    const int num_trials = 25;
    std::vector<double> distance_ratios(num_trials);
    for(int trial = 0; trial < num_trials; trial++)
    {
        // construct trial
        std::vector<double> this_trial(dimension);
        uniform_rand_double(-2., 2., this_trial);

        // do searches
        const int num_neighbors = rand_int(5, 10);

        brute_time -= authority->mpi_wrapper->get_time();
        brute_force->get_neighbors(num_neighbors,
                                   this_trial,
                                   bruteForce_worstToBest_distances,
                                   bruteForce_worstToBest_classOutputs);
        brute_time += authority->mpi_wrapper->get_time();

        other_time -= authority->mpi_wrapper->get_time();
        other_searcher->get_neighbors(num_neighbors, this_trial, other_worstToBest_distances, other_worstToBest_classOutputs);
        other_time += authority->mpi_wrapper->get_time();

        if(other_searcher->is_exact())
        {
            // expect same results
            expect_equal_float_vectors(other_worstToBest_distances, bruteForce_worstToBest_distances);
            expect_equal_vectors(bruteForce_worstToBest_classOutputs, other_worstToBest_classOutputs);
        }

        // compute ratio
        double other_mean_distance = mean(other_worstToBest_distances);
        double brute_mean_distance = mean(bruteForce_worstToBest_distances);
        distance_ratios[trial] = other_mean_distance / brute_mean_distance;
        EXPECT_GE(distance_ratios[trial], 1.);
    }

    // ratio means should be at most just above unity
    double means_of_ratios_of_means = mean(distance_ratios);
    EXPECT_NEAR(means_of_ratios_of_means, 1.01, .0101);

    // announce times
    std::cout << "brute:" << brute_time << ",other:" << other_time << std::endl;
}
PSL_TEST(NeighborsSearching,confirmBasicBruteForceSearch)
{
    set_rand_seed();
    compare_each_searcher_to_brute_force(confirm_basic_against_brute_force_search);
}

void hashCounts_stackedForLoop(RadixHashByPrime* hasher,
                               const std::vector<int>& radix_sizes,
                               const int& index,
                               std::vector<int>& this_radix,
                               std::vector<int>& hash_counts)
{
    // get sizes
    const int length = radix_sizes.size();
    const int this_radix_index_size = radix_sizes[index];

    if(index + 1 < length)
    {
        // recurse
        for(int i = 0; i < this_radix_index_size; i++)
        {
            this_radix[index] = i;
            hashCounts_stackedForLoop(hasher, radix_sizes, index + 1, this_radix, hash_counts);
        }
    }
    else
    {
        // invoke
        for(int i = 0; i < this_radix_index_size; i++)
        {
            this_radix[index] = i;
            const int this_hash_value = hasher->hash(this_radix);
            ASSERT_EQ(true, (0 <= this_hash_value));
            ASSERT_EQ(true, (this_hash_value < int(hash_counts.size())));

            // count this hash value
            hash_counts[this_hash_value]++;
        }
    }
}
PSL_TEST(NeighborsSearching,exampleSpatialHash)
{
    set_rand_seed();
    // allocate
    AbstractAuthority authority;
    RadixHashByPrime hasher(&authority);

    // pose problem
    const int length = rand_int(4, 7);
    std::vector<int> radix_sizes(length, -1);
    for(int i = 0; i < length; i++)
    {
        radix_sizes[i] = rand_int(2, 12);
    }

    // determine true maximum hash
    const int upper_max_hash = 259;
    hasher.initialize(upper_max_hash, radix_sizes);
    const int max_hash = hasher.get_hash_size();
    EXPECT_EQ(max_hash, 257);

    // count hashes for each radix
    std::vector<int> hash_counts(max_hash, 0);
    std::vector<int> this_radix(length, -1);
    hashCounts_stackedForLoop(&hasher, radix_sizes, 0, this_radix, hash_counts);

    // expect evenly divided hash counts
    const double max_deviation = double(max(hash_counts) - min(hash_counts)) / double(max_hash);
    EXPECT_NEAR(max_deviation, .06, .061);

    // do some trials
    for(int rep = 0; rep < 1024; rep++)
    {
        // get a random trial radix
        std::vector<int> trial_radix(length, 0);
        for(int i = 0; i < length; i++)
        {
            trial_radix[i] = rand_int(0, radix_sizes[i]);
        }

        // get the trial's hash
        const int this_trial_hash = hasher.hash(trial_radix);

        // modify the trial radix in one index
        for(int i = 0; i < length; i++)
        {
            std::vector<int> modified_trial_radix = trial_radix;
            if(modified_trial_radix[i] == 0)
            {
                modified_trial_radix[i]++;
            }
            else if(modified_trial_radix[i] + 1 == radix_sizes[i])
            {
                modified_trial_radix[i]--;
            }
            else if(uniform_rand_double() < .5)
            {
                modified_trial_radix[i]++;
            }
            else
            {
                modified_trial_radix[i]--;
            }

            // expect a different hash after modification
            const int modified_trial_hash = hasher.hash(modified_trial_radix);
            EXPECT_NE(this_trial_hash, modified_trial_hash);
        }
    }
}

}
}
