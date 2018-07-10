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
#include "PSL_RegionOfInterestGhostingAgent.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_PointCloud.hpp"
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
#include <algorithm> // for sort

namespace PlatoSubproblemLibrary
{

RegionOfInterestGhostingAgent::RegionOfInterestGhostingAgent(AbstractAuthority* authority) :
        m_authority(authority)
{
}
RegionOfInterestGhostingAgent::~RegionOfInterestGhostingAgent()
{
}

PointCloud* RegionOfInterestGhostingAgent::share(double spatial_tolerance,
                                                 const std::vector<AxisAlignedBoundingBox>& local_regions_of_interest,
                                                 PointCloud* locally_indexed_local_nodes,
                                                 PointCloud* globally_indexed_local_nodes,
                                                 const std::vector<bool>& is_local_point_of_interest)
{
    assert(locally_indexed_local_nodes);
    assert(globally_indexed_local_nodes);

    PointCloud* global_points_of_interest = new PointCloud;

    const size_t mpi_rank = m_authority->mpi_wrapper->get_rank();
    const size_t mpi_size = m_authority->mpi_wrapper->get_size();
    const size_t num_regions = local_regions_of_interest.size();

    // get local bound
    AxisAlignedBoundingBox local_bound = locally_indexed_local_nodes->get_bound();
    local_bound.set_id(mpi_rank);

    // get all local bounds
    std::vector<AxisAlignedBoundingBox> processor_bounds(mpi_size);
    m_authority->mpi_wrapper->all_gather(local_bound, processor_bounds);

    // for each other processor
    std::vector<std::vector<int> > other_processor_then_region_index(mpi_size);
    for(size_t rank = 0u; rank < mpi_size; rank++)
    {
        if(rank == mpi_rank)
        {
            continue;
        }

        // for each region
        for(size_t region = 0u; region < num_regions; region++)
        {
            // if within tolerance, save
            if(processor_bounds[rank].overlap_within_tolerance(local_regions_of_interest[region], spatial_tolerance))
            {
                other_processor_then_region_index[rank].push_back(region);
            }
        }
    }

    // send/recv regions of interest requests for other processors
    std::vector<std::vector<AxisAlignedBoundingBox> > other_processors_regions_of_interest(mpi_size);
    for(size_t rank = 0u; rank < mpi_size; rank++)
    {
        int send_repeat = -1;
        int recv_repeat = -1;
        if(rank < mpi_rank)
        {
            send_repeat = 1;
            recv_repeat = 0;
        }
        else if(mpi_rank < rank)
        {
            send_repeat = 0;
            recv_repeat = 1;
        }

        for(int repeat = 0; repeat < 2; repeat++)
        {
            if(repeat == send_repeat)
            {
                const int num_regions_to_send = other_processor_then_region_index[rank].size();
                m_authority->mpi_wrapper->send(rank, num_regions_to_send);
                for(int region_send = 0; region_send < num_regions_to_send; region_send++)
                {
                    const int region_index = other_processor_then_region_index[rank][region_send];
                    m_authority->mpi_wrapper->send(rank, local_regions_of_interest[region_index]);
                }
            }
            if(repeat == recv_repeat)
            {
                int num_regions_to_recv = -1;
                m_authority->mpi_wrapper->receive(rank, num_regions_to_recv);
                other_processors_regions_of_interest[rank].resize(num_regions_to_recv);
                for(int region_recv = 0; region_recv < num_regions_to_recv; region_recv++)
                {
                    m_authority->mpi_wrapper->receive(rank, other_processors_regions_of_interest[rank][region_recv]);
                }
            }
        }
    }

    // build local searcher
    AbstractInterface::FixedRadiusNearestNeighborsSearcher* generic_searcher =
            build_fixed_radius_nearest_neighbors_searcher(spatial_searcher_t::recommended_overlap_searcher, m_authority);
    AbstractInterface::OverlapSearcher* overlap_searcher = dynamic_cast<AbstractInterface::OverlapSearcher*>(generic_searcher);
    if(!overlap_searcher)
    {
        m_authority->utilities->fatal_error("RegionOfInterestGhostingAgent: failed to dynamic cast pointer. Aborting.\n\n");
    }
    overlap_searcher->build(locally_indexed_local_nodes, spatial_tolerance);

    // send/recv fulfilled regions of interest for other processors
    const size_t num_local_points = locally_indexed_local_nodes->get_num_points();
    std::vector<size_t> local_point_results(num_local_points);
    size_t num_results = 0;
    for(size_t rank = 0u; rank < mpi_size; rank++)
    {
        int send_repeat = -1;
        int recv_repeat = -1;
        if(rank < mpi_rank)
        {
            send_repeat = 1;
            recv_repeat = 0;
        }
        else if(mpi_rank < rank)
        {
            send_repeat = 0;
            recv_repeat = 1;
        }

        for(int repeat = 0; repeat < 2; repeat++)
        {
            if(repeat == send_repeat)
            {
                // mark to send
                std::vector<bool> to_send(num_local_points, false);
                const size_t num_other_regions = other_processors_regions_of_interest[rank].size();
                for(size_t other_region_index = 0u; other_region_index < num_other_regions; other_region_index++)
                {
                    // grow other region by spatial tolerance
                    other_processors_regions_of_interest[rank][other_region_index].grow_in_each_axial_direction(spatial_tolerance);

                    // get local overlaps
                    num_results = 0;
                    overlap_searcher->get_overlaps(&other_processors_regions_of_interest[rank][other_region_index],
                                                   local_point_results,
                                                   num_results);

                    // mark
                    for(size_t results_index = 0u; results_index < num_results; results_index++)
                    {
                        to_send[local_point_results[results_index]] = true;
                    }
                }

                // fill sending points
                PointCloud points_to_send;
                for(size_t local_index = 0u; local_index < num_local_points; local_index++)
                {
                    if(to_send[local_index] && is_local_point_of_interest[local_index])
                    {
                        Point* local_point = globally_indexed_local_nodes->get_point(local_index);
                        points_to_send.push_back(*local_point);
                    }
                }

                // send
                m_authority->mpi_wrapper->send_point_cloud(rank, &points_to_send);
            }
            if(repeat == recv_repeat)
            {
                m_authority->mpi_wrapper->receive_to_point_cloud(rank, global_points_of_interest);
            }
        }
    }

    // for each region
    std::vector<bool> transfer_local(num_local_points, false);
    for(size_t region = 0u; region < num_regions; region++)
    {
        AxisAlignedBoundingBox local_region = local_regions_of_interest[region];
        local_region.grow_in_each_axial_direction(spatial_tolerance);

        num_results = 0;
        overlap_searcher->get_overlaps(&local_region,
                                       local_point_results,
                                       num_results);

        // mark
        for(size_t results_index = 0u; results_index < num_results; results_index++)
        {
            transfer_local[local_point_results[results_index]] = true;
        }
    }
    for(size_t local_index = 0u; local_index < num_local_points; local_index++)
    {
        if(transfer_local[local_index] && is_local_point_of_interest[local_index])
        {
            Point* local_point = globally_indexed_local_nodes->get_point(local_index);
            global_points_of_interest->push_back(*local_point);
        }
    }

    delete generic_searcher;

    return global_points_of_interest;
}

}
