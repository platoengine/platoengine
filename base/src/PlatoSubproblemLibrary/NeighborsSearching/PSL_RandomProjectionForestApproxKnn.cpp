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

#include "PSL_RandomProjectionForestApproxKnn.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_KNearestNeighborsSearcher.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Random.hpp"
#include "PSL_FreeStandardization.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_ParameterData.hpp"

#include <vector>
#include <queue>
#include <functional>
#include <utility>
#include <cmath>
#include <math.h>
#include <string>

namespace PlatoSubproblemLibrary
{

RandomProjectionForestApproxKnn::RandomProjectionForestApproxKnn(AbstractAuthority* authority) :
        KNearestNeighborsSearcher(authority, k_nearest_neighbors_searchers_t::random_projection_forest_searcher),
        m_initial_num_trees(25),
        m_initial_approx_leaf_size(50),
        m_num_trees(NULL),
        m_num_levels(NULL),
        m_tree_then_leaf_then_rows(),
        m_tree_then_split_points(),
        m_random_projections()
{
}
RandomProjectionForestApproxKnn::~RandomProjectionForestApproxKnn()
{
}

bool RandomProjectionForestApproxKnn::is_exact()
{
    return false;
}
void RandomProjectionForestApproxKnn::extract_parameters(ParameterData* parameters)
{
    if(parameters->didUserInput_random_projection_forest_num_trees())
    {
        m_initial_num_trees = parameters->get_random_projection_forest_num_trees();
    }
    assert(0 < m_initial_num_trees);
    if(parameters->didUserInput_random_projection_forest_approx_leaf_size())
    {
        m_initial_approx_leaf_size = parameters->get_random_projection_forest_approx_leaf_size();
    }
    assert(7 < m_initial_approx_leaf_size);
}

// for data flow
void RandomProjectionForestApproxKnn::allocate_data()
{
    KNearestNeighborsSearcher::allocate_data();

    int num_levels = 1 + round(log2(double(m_num_answers) / double(m_initial_approx_leaf_size)));
    num_levels = std::max(num_levels, 2);

    m_data_sequence->enqueue_int(m_initial_num_trees);
    m_data_sequence->enqueue_int(num_levels);

    // compute number of leaf nodes in each tree
    const int leaf_nodes_per_tree = std::pow(2., num_levels - 1);
    // each tree needs a vector for each leaf
    m_data_sequence->enqueue_int_vector(m_initial_num_trees * leaf_nodes_per_tree);

    // each tree needs a vector of splits for each internal node
    m_data_sequence->enqueue_double_vector(m_initial_num_trees);

    // each tree needs a random projection!
    m_data_sequence->enqueue_matrix(m_initial_num_trees);
}
void RandomProjectionForestApproxKnn::unpack_data()
{
    KNearestNeighborsSearcher::unpack_data();

    // unpack sizes
    m_num_trees = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_num_levels = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());

    // for each tree
    const int leaf_nodes_per_tree = std::pow(2., (*m_num_levels) - 1);
    m_tree_then_leaf_then_rows.resize(*m_num_trees);
    m_tree_then_split_points.resize(*m_num_trees);
    m_random_projections.resize(*m_num_trees);
    for(int t = 0; t < *m_num_trees; t++)
    {
        m_tree_then_split_points[t] = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
        m_random_projections[t] = m_data_sequence->get_matrix_ptr(m_data_sequence->dequeue_matrix());

        // for each leaf
        m_tree_then_leaf_then_rows[t].resize(leaf_nodes_per_tree);
        for(int l = 0; l < leaf_nodes_per_tree; l++)
        {
            m_tree_then_leaf_then_rows[t][l] = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());
        }
    }
}

// for searching
void RandomProjectionForestApproxKnn::initialize()
{
    const int num_answers = (*m_answer_points)->get_num_rows();
    const int answer_dimension = (*m_answer_points)->get_num_columns();

    // build projections
    const int projection_num_rows = (*m_num_levels) - 1;
    for(int t = 0; t < *m_num_trees; t++)
    {
        *m_random_projections[t] = m_authority->dense_builder->build_by_fill(projection_num_rows, answer_dimension, 0.);
        for(int r = 0; r < projection_num_rows; r++)
        {
            for(int c = 0; c < answer_dimension; c++)
            {
                (*m_random_projections[t])->set_value(r, c, normal_rand_double());
            }
        }
    }

    // compute all projections
    std::vector<std::vector<std::vector<double> > > row_tree_level(num_answers);
    for(int r = 0; r < num_answers; r++)
    {
        // get row
        std::vector<double> this_row;
        (*m_answer_points)->get_row(r, this_row);

        // for each tree
        row_tree_level[r].resize(*m_num_trees);
        for(int t = 0; t < *m_num_trees; t++)
        {
            // project
            (*m_random_projections[t])->matvec(this_row, row_tree_level[r][t], false);
        }
    }

    // compute project forest
    const int leaf_nodes_per_tree = std::pow(2., (*m_num_levels) - 1);
    const int internal_nodes_per_tree = leaf_nodes_per_tree - 1;
    for(int tree = 0; tree < *m_num_trees; tree++)
    {
        m_tree_then_split_points[tree]->resize(internal_nodes_per_tree);

        const int this_level = 0;
        const int this_cell = 0;

        // all rows active
        std::vector<int> active_rows(num_answers);
        fill_with_index(active_rows);

        // invoke recursion
        recursively_build_project_forest(tree, this_level, this_cell, row_tree_level, active_rows);
    }
}

void RandomProjectionForestApproxKnn::recursively_build_project_forest(const int& tree,
                                                                 const int& this_level,
                                                                 const int& this_cell,
                                                                 const std::vector<std::vector<std::vector<double> > >& row_tree_level,
                                                                 const std::vector<int>& active_rows)
{
    if(this_level + 1 == *m_num_levels)
    {
        // end recursion
        *m_tree_then_leaf_then_rows[tree][this_cell] = active_rows;
    }
    else
    {
        // continue recursion

        // assemble projected values
        const int input_num_active_rows = active_rows.size();
        std::vector<std::pair<double, int> > projected_value_then_absolute_row(input_num_active_rows);
        for(int active_row = 0; active_row < input_num_active_rows; active_row++)
        {
            const int absolute_row = active_rows[active_row];
            const double projected_value = row_tree_level[absolute_row][tree][this_level];
            projected_value_then_absolute_row[active_row] = std::make_pair(projected_value, absolute_row);
        }

        // sort for splitting
        std::sort(projected_value_then_absolute_row.begin(), projected_value_then_absolute_row.end());

        // get indexes for split value
        int left_index = -1;
        int right_index = -1;
        get_median_indexes(input_num_active_rows, left_index, right_index);

        // get split value
        const int splits_index = std::pow(2., this_level) - 1 + this_cell;
        const double split_value = .5 * (projected_value_then_absolute_row[left_index].first
                                      + projected_value_then_absolute_row[right_index].first);
        (*m_tree_then_split_points[tree])[splits_index] = split_value;

        // if disagreement, decide who gets center randomly
        if(left_index == right_index)
        {
            if(uniform_rand_double() < .5)
            {
                left_index--;
            }
            else
            {
                right_index++;
            }
        }

        // fill active rows
        std::vector<int> left_subtree_active_rows(right_index);
        for(int si = 0; si < right_index; si++)
        {
            left_subtree_active_rows[si] = projected_value_then_absolute_row[si].second;
        }
        std::vector<int> right_subtree_active_rows(input_num_active_rows - right_index);
        for(int si = right_index; si < input_num_active_rows; si++)
        {
            right_subtree_active_rows[si - right_index] = projected_value_then_absolute_row[si].second;
        }

        // recurse
        const int left_tree_cell = this_cell * 2;
        recursively_build_project_forest(tree, this_level + 1, left_tree_cell, row_tree_level, left_subtree_active_rows);
        const int right_tree_cell = left_tree_cell + 1;
        recursively_build_project_forest(tree, this_level + 1, right_tree_cell, row_tree_level, right_subtree_active_rows);
    }
}

void RandomProjectionForestApproxKnn::get_neighbors(const int& num_neighbors,
                                                    const std::vector<double>& query_point,
                                                    std::vector<double>& worst_to_best_distances,
                                                    std::vector<int>& worst_to_best_class_outputs)
{
    const int num_answers = (*m_answer_points)->get_num_rows();

    // allocate queue
    std::priority_queue<std::pair<double, int> > distance_then_outputClass;
    std::vector<bool> have_searched(num_answers, false);

    // for each tree
    for(int tree = 0; tree < *m_num_trees; tree++)
    {
        // project
        std::vector<double> projected_values;
        (*m_random_projections[tree])->matvec(query_point, projected_values, false);

        const int this_level = 0;
        const int this_cell = 0;
        recursively_follow_project_forest(tree,
                                          this_level,
                                          this_cell,
                                          num_neighbors,
                                          query_point,
                                          distance_then_outputClass,
                                          have_searched,
                                          projected_values);
    }

    // transfer results
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

void RandomProjectionForestApproxKnn::recursively_follow_project_forest(
                                       const int& tree,
                                       const int& this_level,
                                       const int& this_cell,
                                       const int& num_neighbors,
                                       const std::vector<double>& query_point,
                                       std::priority_queue<std::pair<double, int> >& distance_then_outputClass,
                                       std::vector<bool>& have_searched,
                                       const std::vector<double>& projected_values)
{
    if(this_level + 1 == *m_num_levels)
    {
        // end recursion, do brute force search
        const int num_rows_in_leaf = m_tree_then_leaf_then_rows[tree][this_cell]->size();
        for(int leaf_index = 0; leaf_index < num_rows_in_leaf; leaf_index++)
        {
            const int actual_row = (*m_tree_then_leaf_then_rows[tree][this_cell])[leaf_index];
            if(!have_searched[actual_row])
            {
                have_searched[actual_row] = true;
                // compare this point

                // determine distance
                std::vector<double> this_answer_row;
                (*m_answer_points)->get_row(actual_row, this_answer_row);
                const double dist = std::sqrt(m_authority->dense_vector_operations->delta_squared(query_point, this_answer_row));

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
                    distance_then_outputClass.push(std::make_pair(dist, (*m_answer_class_outputs)[actual_row]));
                }
            }
        }
    }
    else
    {
        // continue recursion

        int next_cell = -1;

        // get split value
        const int splits_index = std::pow(2., this_level) - 1 + this_cell;
        if(projected_values[this_level] < (*m_tree_then_split_points[tree])[splits_index])
        {
            // left subtree
            next_cell = this_cell * 2;
        }
        else
        {
            // right subtree
            next_cell = (this_cell * 2) + 1;
        }

        recursively_follow_project_forest(tree,
                                          this_level + 1,
                                          next_cell,
                                          num_neighbors,
                                          query_point,
                                          distance_then_outputClass,
                                          have_searched,
                                          projected_values);
    }
}

}

