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

#include "PSL_BruteForceKNNSearcher.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_KNearestNeighborsSearcher.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_ParameterData.hpp"

#include <vector>
#include <queue>
#include <functional>
#include <utility>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{

BruteForceKNNSearcher::BruteForceKNNSearcher(AbstractAuthority* authority) :
        KNearestNeighborsSearcher(authority, k_nearest_neighbors_searchers_t::brute_force_searcher)
{
}
BruteForceKNNSearcher::~BruteForceKNNSearcher()
{
}

bool BruteForceKNNSearcher::is_exact()
{
    return true;
}
void BruteForceKNNSearcher::extract_parameters(ParameterData* parameters)
{
}

// for data flow
void BruteForceKNNSearcher::unpack_data()
{
    KNearestNeighborsSearcher::unpack_data();
}
void BruteForceKNNSearcher::allocate_data()
{
    KNearestNeighborsSearcher::allocate_data();
}

// for searching
void BruteForceKNNSearcher::initialize()
{
    // nothing to do
}
void BruteForceKNNSearcher::get_neighbors(const int& num_neighbors,
                                          const std::vector<double>& input_to_search,
                                          std::vector<double>& worst_to_best_distances,
                                          std::vector<int>& worst_to_best_class_outputs)
{
    // allocate queue
    std::priority_queue<std::pair<double, int>,std::vector<std::pair<double, int> >,std::less<std::pair<double, int> > > distance_then_outputClass;

    // for each row
    const int num_rows = (*m_answer_points)->get_num_rows();
    for(int r = 0; r < num_rows; r++)
    {
        // determine distance
        std::vector<double> this_row;
        (*m_answer_points)->get_row(r, this_row);
        const double dist = std::sqrt(m_authority->dense_vector_operations->delta_squared(input_to_search, this_row));

        bool should_enqueue = false;

        if(int(distance_then_outputClass.size()) < num_neighbors)
        {
            // if not enough neighbors, add
            should_enqueue = true;
        }
        else if(dist < distance_then_outputClass.top().first)
        {
            // if better than top, add
            should_enqueue = true;
            distance_then_outputClass.pop();
        }

        // if adding, enqueue
        if(should_enqueue)
        {
            distance_then_outputClass.push(std::make_pair(dist, (*m_answer_class_outputs)[r]));
        }
    }

    worst_to_best_distances.resize(num_neighbors);
    worst_to_best_class_outputs.resize(num_neighbors);
    int counter = 0;
    while(!distance_then_outputClass.empty())
    {
        // count
        worst_to_best_distances[counter] = distance_then_outputClass.top().first;
        worst_to_best_class_outputs[counter] = distance_then_outputClass.top().second;
        // dequeue
        distance_then_outputClass.pop();
        // advance
        counter++;
    }
    worst_to_best_distances.resize(counter);
    worst_to_best_class_outputs.resize(counter);
}

}
