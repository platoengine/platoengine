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
#include "PSL_AbstractAuthority.hpp"
#include "PSL_RadixHashByPrime.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <cassert>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{
namespace TestingSpatialHashing
{

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

PSL_TEST(SpatialHashing,exampleSpatialHash)
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
    EXPECT_NEAR(max_deviation, 0, .05);

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
