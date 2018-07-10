#include "PSL_Random.hpp"

#include "PSL_FreeHelpers.hpp"
#include "PSL_LinearCongruentialGenerator.hpp"

#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <sys/time.h>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <math.h>
#include <cstddef>
#include <cassert>

namespace PlatoSubproblemLibrary
{

void set_rand_seed()
{
#ifdef PSL_IS_DETERMINISTIC
    globalLcg_deterministic_seed();
#else
    std::srand(std::time(0));
#endif
}

// random double between 0 and 1
double uniform_rand_double()
{
#ifdef PSL_IS_DETERMINISTIC
    return globalLcg_getRandDouble();
#else
    return double(rand()) / RAND_MAX;
#endif
}
double uniform_rand_double(const double& lower, const double& upper)
{
    return lower + (upper - lower) * uniform_rand_double();
}
void uniform_rand_double(const double& lower, const double& upper, std::vector<double>& to_fill)
{
    const size_t length = to_fill.size();
    for(size_t i = 0u; i < length; i++)
    {
        to_fill[i] = uniform_rand_double(lower, upper);
    }
}

double normal_rand_double()
{
    // Box-Mueller transform
    return std::sqrt(-2. * log(uniform_rand_double())) * cos(2 * M_PI * uniform_rand_double());
}
double normal_rand_double(const double& mean, const double& std)
{
    return mean + normal_rand_double() * std;
}
void normal_rand_double(const double& mean, const double& std, std::vector<double>& to_fill)
{
    const size_t length = to_fill.size();
    for(size_t i = 0u; i < length; i++)
    {
        to_fill[i] = normal_rand_double(mean, std);
    }
}

int rand_int(const int& inclusive_lower, const int& exclusive_upper)
{
    const int range = exclusive_upper - inclusive_lower;
    return inclusive_lower + floor(double(range) * uniform_rand_double());
}
void rand_int(const int& inclusive_lower, const int& exclusive_upper, std::vector<int>& to_fill)
{
    const size_t length = to_fill.size();
    for(size_t i = 0u; i < length; i++)
    {
        to_fill[i] = rand_int(inclusive_lower, exclusive_upper);
    }
}

void random_permutation(const int& length, std::vector<int>& permutation)
{
    random_permutations(length, length, permutation);
}
void random_permutations(const int& permutation_length, const int& output_length, std::vector<int>& output)
{
    // extend output length to a multiple of the permutation length
    const int extended_output_length = output_length + (output_length % permutation_length);

    // allocate the output at the extended length
    output.resize(extended_output_length);
    for(int i = 0; i < extended_output_length; i++)
    {
        output[i] = i % permutation_length;
    }

    // randomly permute
    std::random_shuffle(output.begin(), output.end());

    // remove extension
    output.resize(output_length);
}

void get_random_batches(const int& length, const int& max_batch_size, std::vector<std::vector<int> >& batches)
{
    // get random ordering
    std::vector<int> permutation;
    random_permutation(length, permutation);

    // allocate batches
    const int num_batches = ceil(double(length)/double(max_batch_size));
    batches.resize(num_batches);

    // batches will either be large or small, with a difference in size of 1 between large and small
    const int large_batch_size = ceil(double(length)/double(num_batches));
    const int num_large_batches = length - (large_batch_size - 1) * num_batches;

    // fill batches
    int assigned_from_permutation = 0;
    for(int batch = 0; batch < num_batches; batch++)
    {
        // set this batch's size
        int this_batch_size = large_batch_size;
        if(batch >= num_large_batches)
        {
            this_batch_size--;
        }
        const int num_remaining = length - assigned_from_permutation;
        this_batch_size = std::min(this_batch_size, num_remaining);

        // assign batch
        batches[batch].assign(&permutation[assigned_from_permutation], &permutation[assigned_from_permutation + this_batch_size]);

        // update assigned
        assigned_from_permutation += this_batch_size;
    }
}

void random_division(const int& length, const double& percent, std::vector<std::vector<int> >& divisions)
{
    std::vector<double> percents = {percent, 1. - percent};
    assert((0. <= percent) && (percent <= 1.));
    random_division(length, percents, divisions);
}

void random_division(const int& length, const std::vector<double>& percents, std::vector<std::vector<int> >& divisions)
{
    // get cumulative percents
    const size_t num_percents = percents.size();
    std::vector<double> cumulative_percents(num_percents, 0.);
    cumulative_sum(percents, cumulative_percents);

    // divide length
    std::vector<int> division_beginnings(num_percents + 1u, -1);
    division_beginnings[0] = 0;
    for(size_t p = 0u; p < num_percents; p++)
    {
        division_beginnings[p + 1u] = round(cumulative_percents[p] * length);
    }

    // get a random permutation
    std::vector<int> permutation;
    random_permutation(length, permutation);

    // assign divisions
    divisions.resize(num_percents);
    for(size_t p = 0u; p < num_percents; p++)
    {
        divisions[p].assign(&permutation[division_beginnings[p]], &permutation[division_beginnings[p + 1u]]);
    }
}

void random_subset_indexes(const int& original_size, const int& final_size, std::vector<int>& selected_indexes)
{
    if(final_size <= 0)
    {
        selected_indexes.clear();
        return;
    }
    random_permutation(original_size, selected_indexes);
    selected_indexes.resize(std::min(final_size, original_size));
}

}

