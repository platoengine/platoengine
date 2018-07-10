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
#include "PSL_SpatialSearcherFactory.hpp"

#include "PSL_BoundingBoxBruteForce.hpp"
#include "PSL_BoundingBoxMortonHierarchy.hpp"
#include "PSL_Abstract_FixedRadiusNearestNeighborsSearcher.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_BruteForceFixedRadiusNearestNeighbors.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_RadixGridFixedRadiusNearestNeighbors.hpp"
#include "PSL_Abstract_NearestNeighborSearcher.hpp"
#include "PSL_BruteForceNearestNeighbor.hpp"
#include "PSL_AbstractAuthority.hpp"

namespace PlatoSubproblemLibrary
{

AbstractInterface::FixedRadiusNearestNeighborsSearcher* build_fixed_radius_nearest_neighbors_searcher(spatial_searcher_t::spatial_searcher_t searcher_type,
                                                                                                      AbstractAuthority* authority)
{
    AbstractInterface::FixedRadiusNearestNeighborsSearcher* result = NULL;

    switch(searcher_type)
    {
        case spatial_searcher_t::bounding_box_brute_force:
        {
            result = new BoundingBoxBruteForce;
            break;
        }
        case spatial_searcher_t::recommended_overlap_searcher:
        case spatial_searcher_t::bounding_box_morton_hierarchy:
        {
            result = new BoundingBoxMortonHierarchy;
            break;
        }
        case spatial_searcher_t::brute_force_fixed_radius_nearest_neighbors:
        {
            result = new BruteForceFixedRadiusNearestNeighbors;
            break;
        }
        case spatial_searcher_t::recommended:
        case spatial_searcher_t::radix_grid_fixed_radius_nearest_neighbors:
        {
            result = new RadixGridFixedRadiusNearestNeighbors;
            break;
        }
        case spatial_searcher_t::brute_force_nearest_neighbor:
        case spatial_searcher_t::unset_spatial_searcher:
        default:
        {
            authority->utilities->fatal_error("PlatoSubproblemLibrary could match enum to build spatial searcher. Aborting.\n\n");
            break;
        }
    }

    return result;
}

AbstractInterface::NearestNeighborSearcher* build_nearest_neighbor_searcher(spatial_searcher_t::spatial_searcher_t searcher_type,
                                                                            AbstractAuthority* authority)
{
    AbstractInterface::NearestNeighborSearcher* result = NULL;

    switch(searcher_type)
    {
        case spatial_searcher_t::recommended:
        case spatial_searcher_t::brute_force_nearest_neighbor:

        {
            result = new BruteForceNearestNeighbor;
            break;
        }
        case spatial_searcher_t::recommended_overlap_searcher:
        case spatial_searcher_t::bounding_box_morton_hierarchy:
        case spatial_searcher_t::brute_force_fixed_radius_nearest_neighbors:
        case spatial_searcher_t::radix_grid_fixed_radius_nearest_neighbors:
        case spatial_searcher_t::bounding_box_brute_force:
        case spatial_searcher_t::unset_spatial_searcher:
        default:
        {
            authority->utilities->fatal_error("PlatoSubproblemLibrary could match enum to build spatial searcher. Aborting.\n\n");
            break;
        }
    }

    return result;
}

}
