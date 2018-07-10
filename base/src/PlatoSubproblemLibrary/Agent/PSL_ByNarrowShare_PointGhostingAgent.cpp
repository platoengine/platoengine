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
#include "PSL_ByNarrowShare_PointGhostingAgent.hpp"

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
#include "PSL_Abstract_OverlapSearcher.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <cassert>
#include <vector>
#include <cstddef>
#include <cmath> // for pow, sqrt, fabs
#include <algorithm> // for sort

namespace PlatoSubproblemLibrary
{

ByNarrowShare_PointGhostingAgent::ByNarrowShare_PointGhostingAgent(AbstractAuthority* authority) :
        Abstract_PointGhostingAgent(point_ghosting_agent_t::by_narrow_share, authority),
        m_overlap_searcher(NULL),
        m_support_distance(-1.)
{
}
ByNarrowShare_PointGhostingAgent::~ByNarrowShare_PointGhostingAgent()
{
}

void ByNarrowShare_PointGhostingAgent::share(double support_distance,
                                             PointCloud* local_kernel_points,
                                             std::vector<PointCloud*>& nonlocal_kernel_points,
                                             std::vector<size_t>& processor_neighbors_below,
                                             std::vector<size_t>& processor_neighbors_above)
{
    assert(local_kernel_points);

    // handle input
    m_support_distance = support_distance;
    AbstractInterface::FixedRadiusNearestNeighborsSearcher* generic_searcher =
            build_fixed_radius_nearest_neighbors_searcher(spatial_searcher_t::recommended_overlap_searcher, m_authority);
    m_overlap_searcher = dynamic_cast<AbstractInterface::OverlapSearcher*>(generic_searcher);
    if(!m_overlap_searcher)
    {
        m_authority->utilities->fatal_error("ByNarrowShare_PointGhostingAgent: failed to dynamic cast pointer. Aborting.\n\n");
    }
    m_overlap_searcher->build(local_kernel_points, m_support_distance);

    // allocate
    const size_t mpi_size = m_authority->mpi_wrapper->get_size();
    std::vector<AxisAlignedBoundingBox> processor_bounds(mpi_size);
    nonlocal_kernel_points.clear();
    nonlocal_kernel_points.resize(mpi_size);

    determine_processor_bounds_and_neighbors(local_kernel_points,
                                             processor_bounds,
                                             processor_neighbors_below,
                                             processor_neighbors_above);

    share_points_with_below_processor(processor_neighbors_below, processor_bounds, local_kernel_points, nonlocal_kernel_points);
    share_points_with_above_processor(processor_neighbors_above, processor_bounds, local_kernel_points, nonlocal_kernel_points);

    delete m_overlap_searcher;
    m_overlap_searcher = NULL;
}

void ByNarrowShare_PointGhostingAgent::determine_processor_bounds_and_neighbors(PointCloud* kernel_points,
                                                                                std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                                                                std::vector<size_t>& processor_neighbors_below,
                                                                                std::vector<size_t>& processor_neighbors_above)
{
    const size_t mpi_rank = m_authority->mpi_wrapper->get_rank();
    const size_t mpi_size = m_authority->mpi_wrapper->get_size();

    // get local bound
    AxisAlignedBoundingBox local_bound = kernel_points->get_bound();
    local_bound.set_id(mpi_rank);

    // get all local bounds
    processor_bounds.clear();
    processor_bounds.resize(mpi_size);
    m_authority->mpi_wrapper->all_gather(local_bound, processor_bounds);

    // determine processor neighbors
    processor_neighbors_below.clear();
    processor_neighbors_above.clear();
    for(size_t proc_below = 0u; proc_below < mpi_rank; proc_below++)
    {
        if(local_bound.overlap_within_tolerance(processor_bounds[proc_below], m_support_distance))
        {
            processor_neighbors_below.push_back(proc_below);
        }
    }
    for(size_t proc_above = mpi_rank + 1u; proc_above < mpi_size; proc_above++)
    {
        if(local_bound.overlap_within_tolerance(processor_bounds[proc_above], m_support_distance))
        {
            processor_neighbors_above.push_back(proc_above);
        }
    }
}

void ByNarrowShare_PointGhostingAgent::share_points_with_below_processor(const std::vector<size_t>& processor_neighbors_below,
                                                                         std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                                                         PointCloud* local_kernel_points,
                                                                         std::vector<PointCloud*>& nonlocal_kernel_points)
{
    // recv nonlocal point from, and send local points to lower processors neighbors
    const size_t num_neighbors_below = processor_neighbors_below.size();
    for(size_t neighbor_proc_index = 0u; neighbor_proc_index < num_neighbors_below; neighbor_proc_index++)
    {
        const size_t lower_proc_id = processor_neighbors_below[neighbor_proc_index];

        share_points_with_processor_receive(lower_proc_id, nonlocal_kernel_points);
        share_points_with_processor_send(lower_proc_id, processor_bounds, local_kernel_points);
    }
}

void ByNarrowShare_PointGhostingAgent::share_points_with_above_processor(const std::vector<size_t>& processor_neighbors_above,
                                                                         std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                                                         PointCloud* local_kernel_points,
                                                                         std::vector<PointCloud*>& nonlocal_kernel_points)
{
    // upper processors neighbors
    const size_t num_neighbors_above = processor_neighbors_above.size();
    for(size_t neighbor_proc_index = 0u; neighbor_proc_index < num_neighbors_above; neighbor_proc_index++)
    {
        const size_t upper_proc_id = processor_neighbors_above[neighbor_proc_index];

        share_points_with_processor_send(upper_proc_id, processor_bounds, local_kernel_points);
        share_points_with_processor_receive(upper_proc_id, nonlocal_kernel_points);
    }
}

void ByNarrowShare_PointGhostingAgent::share_points_with_processor_send(size_t other_proc_id,
                                                                        std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                                                        PointCloud* local_kernel_points)
{
    // grow lower processor by filter radius
    AxisAlignedBoundingBox grown_lower_proc_bound = processor_bounds[other_proc_id];
    grown_lower_proc_bound.grow_in_each_axial_direction(m_support_distance);

    // get local overlaps
    const size_t num_local_points = local_kernel_points->get_num_points();
    std::vector<size_t> local_point_results(num_local_points);
    size_t num_results = 0;
    m_overlap_searcher->get_overlaps(&grown_lower_proc_bound, local_point_results, num_results);
    std::sort(&local_point_results[0], &local_point_results[num_results]);

    // build local kernel points which are neighbors
    PointCloud neighbored_local_kernel_points;
    for(size_t results_index = 0u; results_index < num_results; results_index++)
    {
        const size_t point_index = local_point_results[results_index];
        Point* local_point = local_kernel_points->get_point(point_index);
        neighbored_local_kernel_points.push_back(*local_point);
    }

    // send
    m_authority->mpi_wrapper->send_point_cloud(other_proc_id, &neighbored_local_kernel_points);
}

void ByNarrowShare_PointGhostingAgent::share_points_with_processor_receive(size_t other_proc_id,
                                                                           std::vector<PointCloud*>& nonlocal_kernel_points)
{
    // receive
    nonlocal_kernel_points[other_proc_id] = m_authority->mpi_wrapper->receive_point_cloud(other_proc_id);
}

}
