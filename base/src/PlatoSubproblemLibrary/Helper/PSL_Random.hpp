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

#pragma once

// enable for library to be determinism:
#define PSL_IS_DETERMINISTIC

#include <vector>
#include <set>
#include <cassert>

namespace PlatoSubproblemLibrary
{

void set_rand_seed();

double uniform_rand_double();
double uniform_rand_double(const double& lower, const double& upper);
void uniform_rand_double(const double& lower, const double& upper, std::vector<double>& to_fill);

double normal_rand_double();
double normal_rand_double(const double& mean, const double& std);
void normal_rand_double(const double& mean, const double& std, std::vector<double>& to_fill);

int rand_int(const int& inclusive_lower, const int& exclusive_upper);
void rand_int(const int& inclusive_lower, const int& exclusive_upper, std::vector<int>& to_fill);

template<typename t>
int rand_max_index(const std::vector<t>& occurrences)
{
    // determine maximums and indexes
    t max_value = occurrences[0];
    std::vector<int> indexes_of_modes(1, 0);
    const int length = occurrences.size();
    assert(length > 0);
    for(int i = 1; i < length; i++)
    {
        if(max_value < occurrences[i])
        {
            max_value = occurrences[i];
            indexes_of_modes.assign(1, i);
        }
        else if(occurrences[i] == max_value)
        {
            indexes_of_modes.push_back(i);
        }
    }

    // select index randomly
    const int num_possible_indexes = indexes_of_modes.size();
    assert(0 < num_possible_indexes);
    const int result_index = indexes_of_modes[rand_int(0, num_possible_indexes)];
    return result_index;
}

void random_permutation(const int& length, std::vector<int>& permutation);
void random_permutations(const int& permutation_length, const int& output_length, std::vector<int>& output);

void get_random_batches(const int& length, const int& max_batch_size, std::vector<std::vector<int> >& batches);

void random_division(const int& length, const double& percent, std::vector<std::vector<int> >& divisions);
void random_division(const int& length, const std::vector<double>& percents, std::vector<std::vector<int> >& divisions);

void random_subset_indexes(const int& original_size, const int& final_size, std::vector<int>& selected_indexes);
template<typename T>
void random_subset(const int& final_size, std::set<T>& set)
{
    const int original_size = set.size();

    // if not removing elements, return original set
    if(original_size <= final_size)
    {
        return;
    }

    // if empty set, clear set
    if(final_size <= 0)
    {
        set.clear();
        return;
    }

    // get subset indexes
    std::vector<int> permutation;
    random_subset_indexes(original_size, final_size, permutation);
    std::vector<bool> selected(original_size, false);
    for(int i = 0; i < final_size; i++)
    {
        selected[permutation[i]] = true;
    }

    // remove not selected indexes
    typename std::set<T>::iterator it = set.begin();
    for(int i = 0; i < original_size; i++)
    {
        typename std::set<T>::iterator previous = it++;
        if(!selected[i])
        {
            set.erase(previous);
        }
    }
}

}
