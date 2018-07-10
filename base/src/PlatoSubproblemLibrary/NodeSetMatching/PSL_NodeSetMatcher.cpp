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
#include "PSL_NodeSetMatcher.hpp"

#include "PSL_NodeSet.hpp"
#include "PSL_Point.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_PointCloud.hpp"
#include "PSL_RegionOfInterestGhostingAgent.hpp"
#include "PSL_ByOptimizedElementSide_MeshScaleAgent.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_AxisAlignedBoundingBox.hpp"
#include "PSL_Abstract_FixedRadiusNearestNeighborsSearcher.hpp"
#include "PSL_SpatialSearcherFactory.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <map>
#include <vector>
#include <utility>
#include <iostream>

namespace PlatoSubproblemLibrary
{

NodeSetMatcher::NodeSetMatcher(AbstractAuthority* authority) :
        m_authority(authority),
        m_spatial_tolerance(1e-10),
        m_sources(),
        m_destinations(),
        m_vectors(),
        m_nodeset_pairs_to_index(),
        m_source_and_destination_pairs()
{
}

NodeSetMatcher::~NodeSetMatcher()
{
    m_authority = NULL;
    m_spatial_tolerance = 0.0;
    m_sources.clear();
    m_destinations.clear();
    m_vectors.clear();
    m_nodeset_pairs_to_index.clear();
    m_source_and_destination_pairs.clear();
}

void NodeSetMatcher::set_spatial_tolerance(double spatial_tolerance)
{
    m_spatial_tolerance = spatial_tolerance;
}
void NodeSetMatcher::infer_spatial_tolerance(AbstractInterface::PointCloud* nodes)
{
    ByOptimizedElementSide_MeshScaleAgent agent(m_authority, nodes);
    m_spatial_tolerance = agent.get_mesh_minimum_scale() / 10.0;
}
double NodeSetMatcher::get_spatial_tolerance()
{
    return m_spatial_tolerance;
}

void NodeSetMatcher::register_pair(NodeSet* source, NodeSet* destination, Point* vector)
{
    int source_index = source->get_index();
    int destination_index = destination->get_index();
    std::pair<int, int> nodeset_pair = std::make_pair(source_index, destination_index);
    std::map<std::pair<int, int>,int>::iterator it = m_nodeset_pairs_to_index.find(nodeset_pair);

    if(it == m_nodeset_pairs_to_index.end())
    {
        int pair_index = m_vectors.size();

        m_sources.push_back(std::vector<int>());
        m_sources.back().assign(source->m_nodes.begin(), source->m_nodes.end());
        m_destinations.push_back(std::vector<int>());
        m_destinations.back().assign(destination->m_nodes.begin(), destination->m_nodes.end());
        m_vectors.push_back(vector);

        m_nodeset_pairs_to_index[nodeset_pair] = pair_index;
    }
    else
    {
        m_authority->utilities->fatal_error("NodeSetMatcher: registered nodeset pair multiple times. Aborting.\n");
    }
}

void NodeSetMatcher::match(AbstractInterface::PointCloud* nodes, const std::vector<int>& local_to_global)
{
    // build global_to_local and globally indexed local nodes
    std::map<int, int> global_to_local;
    const size_t num_local = local_to_global.size();
    std::vector<bool> selected_nodes(num_local, true);
    PointCloud* locally_indexed_local_nodes = nodes->build_point_cloud(selected_nodes);
    PointCloud* globally_indexed_local_nodes = nodes->build_point_cloud(selected_nodes);
    for(size_t local_index = 0u; local_index < num_local; local_index++)
    {
        const int this_global = local_to_global[local_index];
        global_to_local[this_global] = int(local_index);
        globally_indexed_local_nodes->get_point(local_index)->set_index(this_global);
    }

    // build local regions of interest
    std::vector<AxisAlignedBoundingBox> local_regions_of_interest;
    std::vector<bool> local_points_of_interest(num_local, false);
    build_local_interest(nodes, local_regions_of_interest, local_points_of_interest);

    // share global query regions
    RegionOfInterestGhostingAgent agent(m_authority);
    PointCloud* global_points_of_interest = agent.share(m_spatial_tolerance,
                                                        local_regions_of_interest,
                                                        locally_indexed_local_nodes,
                                                        globally_indexed_local_nodes,
                                                        local_points_of_interest);

    // populate final pairs, checking for uniqueness
    build_source_and_destination_pairs(nodes, local_to_global, global_points_of_interest, global_to_local);

    delete global_points_of_interest;
    delete locally_indexed_local_nodes;
    delete globally_indexed_local_nodes;
}

void NodeSetMatcher::get_final_pairs(int source_index,
                                     int destination_index,
                                     std::vector<std::pair<int, int> >& source_and_destination_pairs)
{
    std::map<std::pair<int, int>,int>::iterator it = m_nodeset_pairs_to_index.find(std::make_pair(source_index,
                                                                                                  destination_index));

    // if exists, transfer
    if(it != m_nodeset_pairs_to_index.end())
    {
        int pair_index = it->second;
        source_and_destination_pairs.assign(m_source_and_destination_pairs[pair_index].begin(),
                                            m_source_and_destination_pairs[pair_index].end());
    }
    else
    {
        m_authority->utilities->fatal_error("NodeSetMatcher::get_final_pairs found no matching pair. Aborting.\n\n");
    }
}

void NodeSetMatcher::build_local_interest(AbstractInterface::PointCloud* nodes,
                                          std::vector<AxisAlignedBoundingBox>& local_regions_of_interest,
                                          std::vector<bool>& local_points_of_interest)
{
    // for each source/destination pair
    const size_t num_pairs = m_vectors.size();
    for(size_t pair_index = 0u; pair_index < num_pairs; pair_index++)
    {
        // follow sources forward, add to regions of interest
        const size_t num_sources = m_sources[pair_index].size();
        if(num_sources > 0u)
        {
            local_regions_of_interest.push_back(AxisAlignedBoundingBox());
            const int first_local_source = m_sources[pair_index][0];
            local_points_of_interest[first_local_source] = true;
            Point first_source_follow_point = nodes->get_point(first_local_source) + (*m_vectors[pair_index]);
            local_regions_of_interest.back().set(&first_source_follow_point);
        }
        for(size_t source_index = 1u; source_index < num_sources; source_index++)
        {
            const int local_source = m_sources[pair_index][source_index];
            local_points_of_interest[local_source] = true;
            Point source_follow_point = nodes->get_point(local_source) + (*m_vectors[pair_index]);
            local_regions_of_interest.back().grow_to_include(&source_follow_point);
        }

        // follow destinations backward, add to regions of interest
        const size_t num_destinations = m_destinations[pair_index].size();
        if(num_destinations > 0u)
        {
            local_regions_of_interest.push_back(AxisAlignedBoundingBox());
            const int first_local_destination = m_destinations[pair_index][0];
            local_points_of_interest[first_local_destination] = true;
            Point first_destination_follow_point = nodes->get_point(first_local_destination) - (*m_vectors[pair_index]);
            local_regions_of_interest.back().set(&first_destination_follow_point);
        }
        for(size_t destination_index = 1u; destination_index < num_destinations; destination_index++)
        {
            const int local_destination = m_destinations[pair_index][destination_index];
            local_points_of_interest[local_destination] = true;
            Point destination_follow_point = nodes->get_point(local_destination) - (*m_vectors[pair_index]);
            local_regions_of_interest.back().grow_to_include(&destination_follow_point);
        }
    }
}

void NodeSetMatcher::build_source_and_destination_pairs(AbstractInterface::PointCloud* nodes,
                                                        const std::vector<int>& local_to_global,
                                                        PointCloud* global_points_of_interest,
                                                        const std::map<int, int>& global_to_local)
{
    const size_t num_global_points_of_interest = global_points_of_interest->get_num_points();
    std::vector<size_t> search_results(num_global_points_of_interest);
    size_t num_results = 0;

    // build searcher
    AbstractInterface::FixedRadiusNearestNeighborsSearcher* searcher =
            build_fixed_radius_nearest_neighbors_searcher(spatial_searcher_t::recommended, m_authority);
    searcher->build(global_points_of_interest, m_spatial_tolerance);

    const size_t num_pairs = m_vectors.size();
    m_source_and_destination_pairs.clear();
    m_source_and_destination_pairs.resize(num_pairs);
    for(size_t pair_index = 0u; pair_index < num_pairs; pair_index++)
    {
        // follow sources forward, add to regions of interest
        const size_t num_sources = m_sources[pair_index].size();
        for(size_t source_index = 0u; source_index < num_sources; source_index++)
        {
            // compute destination point
            const int local_source = m_sources[pair_index][source_index];
            const int global_source = local_to_global[local_source];
            Point source_follow_point = nodes->get_point(local_source) + (*m_vectors[pair_index]);

            // search for destination
            num_results = 0;
            searcher->get_neighbors(&source_follow_point, search_results, num_results);

            // combine destination global indexes
            std::set<int> result_global_indexes(&search_results[0], &search_results[num_results]);

            // ensure exactly one global index
            if(result_global_indexes.size() > 1u)
            {
                m_authority->utilities->fatal_error("NodeSetMatcher::match found multiple distinct matching destinations. Aborting.\n\n");
                delete searcher;
                return;
            }
            if(result_global_indexes.size() == 0u)
            {
                m_authority->utilities->fatal_error("NodeSetMatcher::match found no matching destinations. Aborting.\n\n");
                delete searcher;
                return;
            }

            // register pair
            const int global_destination = *result_global_indexes.begin();
            m_source_and_destination_pairs[pair_index].push_back(std::make_pair(global_source, global_destination));
        }

        // follow destinations backward, add to regions of interest
        const size_t num_destinations = m_destinations[pair_index].size();
        for(size_t destination_index = 0u; destination_index < num_destinations; destination_index++)
        {
            // compute source point
            const int local_destination = m_destinations[pair_index][destination_index];
            const int global_destination = local_to_global[local_destination];
            Point destination_follow_point = nodes->get_point(local_destination) - (*m_vectors[pair_index]);

            // search for source
            num_results = 0;
            searcher->get_neighbors(&destination_follow_point, search_results, num_results);

            // combine source global indexes
            std::set<int> result_global_indexes(&search_results[0], &search_results[num_results]);

            // ensure exactly one global index
            if(result_global_indexes.size() > 1u)
            {
                m_authority->utilities->fatal_error("NodeSetMatcher::match found multiple distinct matching sources. Aborting.\n\n");
                delete searcher;
                return;
            }
            if(result_global_indexes.size() == 0u)
            {
                m_authority->utilities->fatal_error("NodeSetMatcher::match found no matching sources. Aborting.\n\n");
                delete searcher;
                return;
            }
            const int global_source = *result_global_indexes.begin();

            // if both local, skip
            const bool is_local_source = (global_to_local.find(global_source) != global_to_local.end());
            const bool is_local_destination = true;
            if(is_local_source && is_local_destination)
            {
                continue;
            }

            // register pair
            m_source_and_destination_pairs[pair_index].push_back(std::make_pair(global_source, global_destination));
        }
    }

    delete searcher;
}

}
