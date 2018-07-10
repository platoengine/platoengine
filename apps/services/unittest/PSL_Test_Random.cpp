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

#include "PSL_Random.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_FreeStandardization.hpp"
#include "PSL_LinearCongruentialGenerator.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <math.h>
#include <iostream>
#include <cstddef>
#include <utility>

namespace PlatoSubproblemLibrary
{
namespace TestingRandom
{

PSL_TEST(Random,linearCongruentialGenerator_determinism)
{
    set_rand_seed();
    globalLcg_deterministic_seed();

    // get some values
    const std::vector<unsigned long int> expected_values = {2027382, 1226992407, 551494037, 961371815, 1404753842, 2076553157,
                                                            1350734175, 1538354858, 90320905, 488601845, 1634248641, 1151860813,
                                                            974199846, 3864260, 1848100818};
    const int num_generated = expected_values.size();
    for(int g = 0; g < num_generated; g++)
    {
        unsigned long int computed_value = globalLcg_getRandInt();
        EXPECT_EQ(expected_values[g], computed_value);
    }

    for(int g = 0; g < num_generated; g++)
    {
        const double computed_value = globalLcg_getRandDouble();
        EXPECT_GT(computed_value, 0.);
        EXPECT_GT(1., computed_value);
    }
}

PSL_TEST(Random,randomSubsetIndexesEdgeCases)
{
    set_rand_seed();
    std::vector<int> selected_indexes;

    // select all
    random_subset_indexes(10, 10, selected_indexes);
    EXPECT_EQ(selected_indexes.size(), 10u);

    // select more than all
    random_subset_indexes(10, 15, selected_indexes);
    EXPECT_EQ(selected_indexes.size(), 10u);

    // select none
    random_subset_indexes(10, 0, selected_indexes);
    EXPECT_EQ(selected_indexes.size(), 0u);

    // select "fewer than none"
    random_subset_indexes(10, -1, selected_indexes);
    EXPECT_EQ(selected_indexes.size(), 0u);
}
PSL_TEST(Random,randomSubsetIndexesBasic)
{
    set_rand_seed();
    // repeat tests
    const int num_repeat = 10;
    for(int repeat = 0; repeat < num_repeat; repeat++)
    {
        // pose problem
        const int original_size = 128;
        const double subset_fraction = uniform_rand_double(0.1, 0.9);
        const int final_size = int(ceil(double(original_size) * subset_fraction));

        // get indexes
        std::vector<int> selected_indexes;
        random_subset_indexes(original_size, final_size, selected_indexes);

        // correct count of indexes
        ASSERT_EQ(int(selected_indexes.size()), final_size);
        std::set<int> selecteds;
        for(int i = 0; i < final_size; i++)
        {
            selecteds.insert(selected_indexes[i]);
        }
        EXPECT_EQ(selecteds.size(), selected_indexes.size());
    }
}

PSL_TEST(Random,randomSubsetBasic)
{
    set_rand_seed();
    // pose original set
    const std::set<size_t> original_set = {1u, 5u, 11u, 1u, 9u, 7u};
    std::set<size_t> mutating_set;

    const size_t original_size = original_set.size();
    for(size_t o = 1u; o < original_size; o++)
    {
        mutating_set = original_set;
        random_subset(o, mutating_set);
        EXPECT_EQ(o, mutating_set.size());
    }
}

PSL_TEST(Random,randomSubsetEdgeCases)
{
    set_rand_seed();
    // pose original set
    std::set<std::pair<int, size_t> > original_set;
    original_set.insert(std::make_pair(5, 4u));
    original_set.insert(std::make_pair(1, 4u));
    original_set.insert(std::make_pair(7, 2u));
    std::set<std::pair<int, size_t> > mutating_set;

    // expect no change
    mutating_set = original_set;
    random_subset(3, mutating_set);
    expect_equal_sets(mutating_set, original_set);

    // expect no change
    mutating_set = original_set;
    random_subset(4, mutating_set);
    expect_equal_sets(mutating_set, original_set);

    // expect empty set
    mutating_set = original_set;
    random_subset(0, mutating_set);
    EXPECT_EQ(mutating_set.size(), 0u);

    // expect empty set
    mutating_set = original_set;
    random_subset(-1, mutating_set);
    EXPECT_EQ(mutating_set.size(), 0u);
}

bool test_of_randomProperSubsets()
{
    bool passed = true;

    const int max_value = 1024;
    const int num_rand_values = 128;
    const double subset_fraction = uniform_rand_double(0.1, 0.9);

    // build random set
    std::set<int> original_rand_set;
    for(int r = 0; r < num_rand_values; r++)
    {
        original_rand_set.insert(rand_int(0, max_value));
    }

    // compute target size
    const int original_size = original_rand_set.size();
    const int target_subset_size = int(ceil(double(original_size) * subset_fraction));

    // get subsets
    std::set<int> first_subset = original_rand_set;
    random_subset(target_subset_size, first_subset);
    std::set<int> second_subset = original_rand_set;
    random_subset(target_subset_size, second_subset);

    // should exactly match target sizes
    EXPECT_EQ(int(first_subset.size()), target_subset_size);
    EXPECT_EQ(int(second_subset.size()), target_subset_size);

    // subsets should be distinct
    const bool are_distinct = (first_subset != second_subset);
    passed &= are_distinct;

    return passed;
}
PSL_TEST(Random,randomProperSubset)
{
    set_rand_seed();
    stocastic_test(test_of_randomProperSubsets, 5, 25, .79);
}

bool testing_randMode_pass()
{
    // pose problem
    const int length = 52;
    const int first_mode_index = 11;
    const int second_mode_index = 28;
    const int mode_value = 72;
    const int repeat = 10;

    // get values
    std::vector<int> values(length);
    rand_int(0, mode_value, values);
    values[first_mode_index] = mode_value;
    values[second_mode_index] = mode_value;

    // get modes
    std::set<int> mode_indexes;
    for(int r = 0; r < repeat; r++)
    {
        mode_indexes.insert(rand_max_index(values));
    }

    // expect to have found both modes
    const bool found_only_two = (mode_indexes.size() == 2u);
    const bool found_first = (mode_indexes.find(first_mode_index) != mode_indexes.end());
    const bool found_second = (mode_indexes.find(second_mode_index) != mode_indexes.end());
    return (found_only_two && found_first && found_second);
}
PSL_TEST(Random, randMode)
{
    set_rand_seed();
    stocastic_test(testing_randMode_pass, 10, 100, .9);
}

PSL_TEST(Random, deterministicMode)
{
    set_rand_seed();
    std::vector<int> occurrences1 = {1, 0, 3, 5, 7, 2};
    EXPECT_EQ(4, rand_max_index(occurrences1));

    std::vector<int> occurrences2 = {1, 11, -1, 7, 2};
    EXPECT_EQ(1, rand_max_index(occurrences2));
}

bool testing_randIntVector_pass()
{
    bool result = true;

    // pose problem
    const int length = 39;
    const int lower_bound = -12;
    const int upper_bound = 37;

    // get values
    std::vector<int> values(length);
    rand_int(lower_bound, upper_bound, values);

    // expect bounds enforced
    result &= (lower_bound <= min(values));
    result &= (max(values) < upper_bound);

    // expect distinct
    result &= (min(values) < max(values));
    result &= (values[0]*length != sum(values));

    return result;
}
PSL_TEST(Random, randIntVector)
{
    set_rand_seed();
    stocastic_test(testing_randIntVector_pass, 10, 100, .9);
}

bool testing_randNormal_pass()
{
    // pose problem
    const double expected_mean = -3.1;
    const double expected_std = 1.575;
    const int num_values = 10000;
    const double absolute_tolerance = .05;
    const double fraction_tolerance = .05;

    // get values
    std::vector<double> normal_values(num_values);
    normal_rand_double(expected_mean, expected_std, normal_values);

    // get mean and std
    double actual_mean = -1.;
    double actual_std = -1.;
    get_mean_and_std(normal_values, actual_mean, actual_std);

    // compute tolerances
    const double mean_tolerance = absolute_tolerance + fabs(expected_mean) * fraction_tolerance;
    const double std_tolerance = absolute_tolerance + fabs(expected_std) * fraction_tolerance;

    // expect within tolerance
    bool result = true;
    result &= (fabs(expected_mean - actual_mean) < mean_tolerance);
    result &= (fabs(expected_std - actual_std) < std_tolerance);
    return result;
}
PSL_TEST(Random, randNormal)
{
    set_rand_seed();
    stocastic_test(testing_randNormal_pass, 10, 100, .9);
}

bool testing_rand_pass()
{
    return (.1 < uniform_rand_double());
}
PSL_TEST(Random, randPass)
{
    set_rand_seed();
    stocastic_test(testing_rand_pass, 100, 10000, .8);
}

PSL_TEST(Random, randInt)
{
    set_rand_seed();
    // specify problem
    const int lower = -4;
    const int upper = 72;
    const int repeat = 200000;
    const int min_expect = 2;

    // allocate
    const int length = upper - lower;
    std::vector<int> occurences(length, 0);
    for(int r = 0; r < repeat; r++)
    {
        const int rint = rand_int(lower, upper);

        EXPECT_GE(rint, lower);
        EXPECT_GT(upper, rint);
        occurences[rint - lower]++;
    }

    // expect on min
    const int actual_min = min(occurences);
    EXPECT_GE(actual_min, min_expect);
}

void confirm_rand_vector(const double& lower, const double& upper, const std::vector<double>&rand_vals)
{
    const int length = rand_vals.size();

    bool sequential_distinct = true;
    double last_rand = rand_vals[0];
    double min = last_rand;
    double max = last_rand;
    for(int i = 1; i < length; i++)
    {
        const double rand = rand_vals[i];
        max = std::max(rand, max);
        min = std::min(rand, min);

        sequential_distinct &= (rand != last_rand);
        last_rand = rand;
    }

    // ensure in appropriate range
    EXPECT_GT(min, lower);
    EXPECT_GT(upper, max);

    // ensure some random values are distinct
    EXPECT_EQ(sequential_distinct, true);
}

PSL_TEST(Random, rand)
{
    set_rand_seed();
    set_rand_seed();

    // get some random values
    const int length = 42;
    std::vector<double> rand_vals(length, 2.);
    for(int i = 0; i < length; i++)
    {
        rand_vals[i] = uniform_rand_double();
    }

    // confirm them
    confirm_rand_vector(0., 1., rand_vals);
}

PSL_TEST(Random, randRange)
{
    set_rand_seed();

    // get some random values
    const int length = 37;
    const double lower = -5. * uniform_rand_double();
    const double upper = 10. * uniform_rand_double();
    std::vector<double> rand_vals(length, upper * 2.);
    for(int i = 0; i < length; i++)
    {
        rand_vals[i] = uniform_rand_double(lower, upper);
    }

    // confirm them
    confirm_rand_vector(lower,upper,rand_vals);
}

PSL_TEST(Random, randRangeVector)
{
    set_rand_seed();

    // get some random values
    const int length = 19;
    const double lower = -4. * uniform_rand_double();
    const double upper = 9. * uniform_rand_double();
    std::vector<double> rand_vals(length, upper * 2.);
    uniform_rand_double(lower, upper, rand_vals);

    // confirm them
    confirm_rand_vector(lower, upper, rand_vals);
}

PSL_TEST(Random, permutation)
{
    set_rand_seed();

    // specify problem
    const int length = 420;

    // get permutations
    std::vector<int> first_permutation;
    random_permutation(length, first_permutation);
    std::vector<int> second_permutation;
    random_permutation(length, second_permutation);

    // ensure sizes
    ASSERT_EQ(first_permutation.size(), size_t(length));
    ASSERT_EQ(second_permutation.size(), size_t(length));

    // mark values found in permutation
    std::vector<bool> found_in_first(length, false);
    std::vector<bool> found_in_second(length, false);

    // mark if found difference in permutations
    bool found_difference = false;

    // scan permutations
    for(int i = 0; i < length; i++)
    {
        found_in_first[first_permutation[i]] = true;
        found_in_second[second_permutation[i]] = true;
        found_difference |= (first_permutation[i] != second_permutation[i]);
    }

    // expect difference
    EXPECT_EQ(found_difference, true);

    // ensure all values found in both permutations
    std::vector<bool> all_true(length, true);
    expect_equal_vectors(all_true, found_in_first);
    expect_equal_vectors(all_true, found_in_second);
}

void test_permutations(const int& permutation_length, const int& output_length)
{
    set_rand_seed();

    // get permutations
    std::vector<int> first_output;
    random_permutations(permutation_length, output_length, first_output);
    std::vector<int> second_output;
    random_permutations(permutation_length, output_length, second_output);

    // ensure sizes
    ASSERT_EQ(first_output.size(), size_t(output_length));
    ASSERT_EQ(first_output.size(), size_t(output_length));

    // count occurrences of values found in permutation
    std::vector<int> count_in_first(permutation_length, 0);
    std::vector<int> count_in_second(permutation_length, 0);

    // mark if found difference in permutations
    bool found_difference = false;

    // scan permutations
    for(int i = 0; i < output_length; i++)
    {
        count_in_first[first_output[i]]++;
        count_in_second[second_output[i]]++;
        found_difference |= (first_output[i] != second_output[i]);
    }

    // expect difference
    EXPECT_EQ(found_difference, true);

    // expect balanced output
    const double ideal_percent = double(output_length) / double(permutation_length);
    for(int p = 0; p < permutation_length; p++)
    {
        EXPECT_GE(ceil(ideal_percent+1.), count_in_first[p]);
    }
}

PSL_TEST(Random, permutationsShortOuput)
{
    set_rand_seed();
    test_permutations(27, 8);
}
PSL_TEST(Random, permutationsLongOuput)
{
    set_rand_seed();
    test_permutations(19, 420);
}

void test_batching(const int& length, const int& max_batch_size)
{
    // get batches
    std::vector<std::vector<int> > batches;
    get_random_batches(length, max_batch_size, batches);
    const int num_batches = batches.size();

    // scan batches
    std::vector<bool> found(length, false);
    int actual_max_batch_size = 0;
    for(int batch = 0; batch < num_batches; batch++)
    {
        // update max
        const int this_batch_length = batches[batch].size();
        actual_max_batch_size = std::max(this_batch_length, actual_max_batch_size);

        // update found
        for(int in_batch = 0; in_batch < this_batch_length; in_batch++)
        {
            found[batches[batch][in_batch]] = true;
        }
    }

    // ensure maximum batch size
    EXPECT_GE(max_batch_size, actual_max_batch_size);

    // ensure all values found in some batch
    std::vector<bool> all_true(length, true);
    expect_equal_vectors(all_true, found);

    // ensure as few batches used as needed
    if(num_batches > 1)
    {
        // if one less batch was used, would have violated maximum
        EXPECT_GE(double(length) / double(num_batches - 1.), max_batch_size);
    }
    else
    {
        // one batch was sufficient
        EXPECT_GE(max_batch_size, length);
    }
}

PSL_TEST(Random, smallBatchDivisible)
{
    set_rand_seed();
    test_batching(6, 2);
}
PSL_TEST(Random, smallBatchNotDivisible)
{
    set_rand_seed();
    test_batching(7, 3);
}
PSL_TEST(Random, largeBatchDivisible)
{
    set_rand_seed();
    test_batching(119, 17);
}
PSL_TEST(Random, largeBatchNotDivisible)
{
    set_rand_seed();
    test_batching(140, 3);
}
PSL_TEST(Random, exactlyOneBatch)
{
    set_rand_seed();
    test_batching(12, 12);
}
PSL_TEST(Random, oneBatch)
{
    set_rand_seed();
    test_batching(42, 50);
}
PSL_TEST(Random, twoBatch)
{
    set_rand_seed();
    test_batching(34, 30);
}
PSL_TEST(Random, singletonBatches)
{
    set_rand_seed();
    test_batching(19, 1);
}

PSL_TEST(Random, distinctBatches)
{
    set_rand_seed();

    // specify problem
    const int length = 257;
    const int max_batch_size = 16;

    // get batches
    std::vector<std::vector<int> > first_batches;
    get_random_batches(length, max_batch_size, first_batches);
    std::vector<std::vector<int> > second_batches;
    get_random_batches(length, max_batch_size, second_batches);

    // ensure same number of batches
    const int num_batches = first_batches.size();
    EXPECT_EQ(num_batches, int(second_batches.size()));

    // scan batches
    bool found_distinct = false;
    for(int batch = 0; batch < num_batches; batch++)
    {
        // ensure each corresponding batch is same size
        const int this_batch_size = first_batches[batch].size();
        EXPECT_EQ(this_batch_size, int(second_batches[batch].size()));

        // update if found distinction
        for(int in_batch = 0; in_batch < this_batch_size; in_batch++)
        {
            found_distinct |= (first_batches[batch][in_batch] != second_batches[batch][in_batch]);
        }
    }

    // batches should not be same
    EXPECT_EQ(found_distinct, true);
}

void test_division(const int& length, const std::vector<std::vector<int> >& divisions)
{
    // mark found
    std::vector<bool> found(length, false);
    const size_t num_divisions = divisions.size();
    for(size_t div = 0u; div < num_divisions; div++)
    {
        const size_t this_division_length = divisions[div].size();
        for(size_t i = 0u; i < this_division_length; i++)
        {
            EXPECT_EQ(found[divisions[div][i]], false);
            found[divisions[div][i]] = true;
        }
    }

    // expect all found
    std::vector<bool> all_found(length, true);
    expect_equal_vectors(found, all_found);
}

PSL_TEST(Random, randomDivision)
{
    set_rand_seed();
    // pose problem
    const int length = 4;
    const double percent = .5;
    std::vector<std::vector<int> > divisions;
    random_division(length, percent, divisions);

    // ensure accurate split
    ASSERT_EQ(divisions.size(), 2u);
    ASSERT_EQ(divisions[0].size(), 2u);
    ASSERT_EQ(divisions[1].size(), 2u);

    test_division(length, divisions);
}

PSL_TEST(Random, randomDivisions)
{
    set_rand_seed();
    // pose problem
    const int length = 25;
    std::vector<double> percents = {.39, .05, .23, .33};
    std::vector<std::vector<int> > divisions;
    random_division(length, percents, divisions);

    // ensure accurate split
    ASSERT_EQ(divisions.size(), percents.size());
    ASSERT_EQ(divisions[0].size(), 10u);
    ASSERT_EQ(divisions[1].size(), 1u);
    ASSERT_EQ(divisions[2].size(), 6u);
    ASSERT_EQ(divisions[3].size(), 8u);

    test_division(length, divisions);
}

}
}
