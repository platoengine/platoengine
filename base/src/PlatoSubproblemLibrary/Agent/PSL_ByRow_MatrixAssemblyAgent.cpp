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

// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_ByRow_MatrixAssemblyAgent.hpp"

#include "PSL_Abstract_MatrixAssemblyAgent.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Abstract_SparseMatrix.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_Abstract_SparseMatrixBuilder.hpp"
#include "PSL_Point.hpp"
#include "PSL_Abstract_FixedRadiusNearestNeighborsSearcher.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AxisAlignedBoundingBox.hpp"
#include "PSL_SpatialSearcherFactory.hpp"
#include "PSL_Abstract_BoundedSupportFunction.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <cassert>
#include <vector>
#include <cstddef>
#include <cmath> // for pow, sqrt, fabs
#include <algorithm> // for sort

namespace PlatoSubproblemLibrary
{

ByRow_MatrixAssemblyAgent::ByRow_MatrixAssemblyAgent(AbstractAuthority* authority,
                                                     ParameterData* input_data) :
        Abstract_MatrixAssemblyAgent(matrix_assembly_agent_t::by_row, authority),
        m_input_data(input_data),
        m_support_distance(-1.)
{
}

ByRow_MatrixAssemblyAgent::~ByRow_MatrixAssemblyAgent()
{
}

void ByRow_MatrixAssemblyAgent::build(Abstract_BoundedSupportFunction* bounded_support_function,
                                      PointCloud* local_kernel_points,
                                      std::vector<PointCloud*>& nonlocal_kernel_points,
                                      const std::vector<size_t>& processor_neighbors_below,
                                      const std::vector<size_t>& processor_neighbors_above,
                                      AbstractInterface::SparseMatrix** local_kernel_matrix,
                                      std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                      std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices)
{
    assert(m_input_data);
    assert(m_authority);
    assert(local_kernel_points);

    // handle input
    m_support_distance = bounded_support_function->get_support();
    m_searcher = build_fixed_radius_nearest_neighbors_searcher(m_input_data->get_spatial_searcher(), m_authority);
    assert(m_searcher);

    // allocate
    const size_t mpi_size = m_authority->mpi_wrapper->get_size();
    parallel_block_row_kernel_matrices.resize(mpi_size);
    parallel_block_column_kernel_matrices.resize(mpi_size);

    // get points per processor
    std::vector<int> num_points_per_processor(mpi_size);
    m_authority->mpi_wrapper->all_gather(local_kernel_points->get_num_points(), num_points_per_processor);

    build_local(bounded_support_function, local_kernel_points, local_kernel_matrix);

    compute_nonlocal_matrix_for_above_processor(bounded_support_function,
                                                processor_neighbors_above,
                                                nonlocal_kernel_points,
                                                local_kernel_points,
                                                parallel_block_row_kernel_matrices,
                                                num_points_per_processor);

    send_block_row_to_above_processor(processor_neighbors_above, parallel_block_row_kernel_matrices);

    recv_block_row_from_below_processor(processor_neighbors_below, parallel_block_row_kernel_matrices);

    delete m_searcher;
    m_searcher = NULL;
}

void ByRow_MatrixAssemblyAgent::build_local(Abstract_BoundedSupportFunction* bounded_support_function,
                                            PointCloud* kernel_points,
                                            AbstractInterface::SparseMatrix** local_kernel_matrix)
{
    // build searcher
    m_searcher->build(kernel_points, m_support_distance);

    // prepare for building local kernel matrix
    const size_t num_repeats = m_authority->sparse_builder->get_number_of_passes_over_all_nonzero_entries();
    const size_t num_points = kernel_points->get_num_points();
    std::vector<size_t> neighbors_buffer(num_points);
    size_t num_neighbors = 0u;

    // build local sparse matrix
    m_authority->sparse_builder->begin_build(num_points, num_points);
    for(size_t repeat = 0u; repeat < num_repeats; repeat++)
    {
        for(size_t point1_index = 0; point1_index < num_points; point1_index++)
        {
            Point* point1 = kernel_points->get_point(point1_index);

            // determine which points are within the radius of point1
            num_neighbors = 0;
            m_searcher->get_neighbors(point1, neighbors_buffer, num_neighbors);

            // this sort is not necessary but promotes more sequential access
            std::sort(&neighbors_buffer[0], &neighbors_buffer[num_neighbors]);

            // for each neighbor found, calculate the distance weight
            for(size_t neighbor_index = 0; neighbor_index < num_neighbors; neighbor_index++)
            {
                const size_t point2_index = neighbors_buffer[neighbor_index];
                Point* point2 = kernel_points->get_point(point2_index);

                // if weight positive, store
                const double weight = bounded_support_function->evaluate(point1, point2);
                if(weight > 0)
                {
                    m_authority->sparse_builder->specify_nonzero(point1_index, point2_index, weight);
                }
            }
        }
        m_authority->sparse_builder->advance_pass();
    }

    // finalize construction
    *local_kernel_matrix = m_authority->sparse_builder->end_build();
}

void ByRow_MatrixAssemblyAgent::compute_nonlocal_matrix_for_above_processor(Abstract_BoundedSupportFunction* bounded_support_function,
                                                                            const std::vector<size_t>& processor_neighbors_above,
                                                                            std::vector<PointCloud*>& nonlocal_kernel_points,
                                                                            PointCloud* local_kernel_points,
                                                                            std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                                                            const std::vector<int>& num_points_per_processor)
{
    // compute matrix from higher processors that neighbor
    const size_t num_neighbors_above = processor_neighbors_above.size();
    for(size_t neighbor_proc_index = 0u; neighbor_proc_index < num_neighbors_above; neighbor_proc_index++)
    {
        const size_t upper_proc_id = processor_neighbors_above[neighbor_proc_index];
        const size_t num_nonLocal_within_radius = nonlocal_kernel_points[upper_proc_id]->get_num_points();

        // for row block matrix: rows are local and columns are nonlocal
        const size_t num_local_points = local_kernel_points->get_num_points();
        const size_t num_rows = num_local_points;
        const size_t num_columns = num_points_per_processor[upper_proc_id];

        // prepare for building sparse matrix
        const size_t num_repeats = m_authority->sparse_builder->get_number_of_passes_over_all_nonzero_entries();
        std::vector<size_t> neighbors_buffer(num_local_points);
        size_t num_neighbors = 0u;

        // build sparse matrix
        m_authority->sparse_builder->begin_build(num_rows, num_columns);
        for(size_t repeat = 0u; repeat < num_repeats; repeat++)
        {
            for(size_t nonlocal_index = 0; nonlocal_index < num_nonLocal_within_radius; nonlocal_index++)
            {
                Point* nonlocal_point = nonlocal_kernel_points[upper_proc_id]->get_point(nonlocal_index);

                // determine which points are within the radius of point1
                num_neighbors = 0;
                m_searcher->get_neighbors(nonlocal_point, neighbors_buffer, num_neighbors);

                // this sort is not necessary but promotes more sequential access
                std::sort(&neighbors_buffer[0], &neighbors_buffer[num_neighbors]);

                // for each neighbor found, calculate the distance weight
                for(size_t neighbor_index = 0; neighbor_index < num_neighbors; neighbor_index++)
                {
                    const size_t local_neighbor = neighbors_buffer[neighbor_index];
                    Point* local_point = local_kernel_points->get_point(local_neighbor);

                    // if weight positive, store
                    const double weight = bounded_support_function->evaluate(local_point, nonlocal_point);
                    if(weight > 0)
                    {
                        const size_t row = local_point->get_index();
                        const size_t column = nonlocal_point->get_index();

                        m_authority->sparse_builder->specify_nonzero(row, column, weight);
                    }
                }
            }
            m_authority->sparse_builder->advance_pass();
        }

        // finalize construction
        parallel_block_row_kernel_matrices[upper_proc_id] = m_authority->sparse_builder->end_build();

        // if empty, delete
        if(parallel_block_row_kernel_matrices[upper_proc_id]->getNumNonZeroSortedRows() == 0u)
        {
            delete parallel_block_row_kernel_matrices[upper_proc_id];
            parallel_block_row_kernel_matrices[upper_proc_id] = NULL;
        }
    }
}

void ByRow_MatrixAssemblyAgent::send_block_row_to_above_processor(const std::vector<size_t>& processor_neighbors_above,
                                                                  std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices)
{
    // send block row to above neighbors starting from last
    const int num_neighbors_above = processor_neighbors_above.size();
    for(int neighbor_proc_index = num_neighbors_above - 1; 0 <= neighbor_proc_index; neighbor_proc_index--)
    {
        const size_t upper_proc_id = processor_neighbors_above[neighbor_proc_index];

        // send matrix
        m_authority->sparse_builder->send_matrix(upper_proc_id, parallel_block_row_kernel_matrices[upper_proc_id]);
    }
}

void ByRow_MatrixAssemblyAgent::recv_block_row_from_below_processor(const std::vector<size_t>& processor_neighbors_below,
                                                                    std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices)
{
    // receive block matrix from lower processors that overlap
    const int num_neighbors_below = processor_neighbors_below.size();
    for(int neighbor_proc_index = num_neighbors_below - 1; 0 <= neighbor_proc_index; neighbor_proc_index--)
    {
        const size_t lower_proc_id = processor_neighbors_below[neighbor_proc_index];

        // receive sparse matrix
        AbstractInterface::SparseMatrix* matrix = m_authority->sparse_builder->receive_matrix(lower_proc_id);
        parallel_block_row_kernel_matrices[lower_proc_id] = m_authority->sparse_builder->transpose(matrix);
        safe_free(matrix);
    }
}

}
