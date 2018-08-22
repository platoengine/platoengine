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
