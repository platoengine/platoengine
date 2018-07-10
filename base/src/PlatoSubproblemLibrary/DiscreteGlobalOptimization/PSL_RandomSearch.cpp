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

#include "PSL_RandomSearch.hpp"

#include "PSL_DiscreteGlobalOptimizer.hpp"
#include "PSL_DiscreteObjective.hpp"
#include "PSL_Random.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cassert>

namespace PlatoSubproblemLibrary
{

RandomSearch::RandomSearch(AbstractAuthority* authority) :
        DiscreteGlobalOptimizer(authority),
        m_num_trials()
{
    default_parameters();
}
RandomSearch::~RandomSearch()
{

}

void RandomSearch::default_parameters()
{
    m_num_trials = 100;
}
double RandomSearch::find_min(std::vector<double>& best_parameters)
{
    // store best found
    bool found_a_best = false;
    double best_objective = 0.;

    // get sizes
    std::vector<int> num_values;
    get_num_values(num_values);
    const int dimension = num_values.size();

    // for a random usage
    std::vector<int> values(dimension);
    std::vector<double> parameters(dimension);
    for(int trial = 0; trial < m_num_trials; trial++)
    {
        // generate values
        for(int d = 0; d < dimension; d++)
        {
            values[d] = rand_int(0, num_values[d]);
        }

        // compute parameters
        fill_parameters(values, parameters);

        // evaluate
        const double obj_value = m_obj->evaluate(parameters);
        count_evaluated_objective();

        // if new best, update
        if(!found_a_best || (obj_value < best_objective))
        {
            found_a_best = true;
            best_objective = obj_value;
            best_parameters = parameters;
        }
    }

    assert(found_a_best);
    return best_objective;
}

}

