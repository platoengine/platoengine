// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class FixedRadiusNearestNeighborsSearcher;
class NearestNeighborSearcher;
}
class AbstractAuthority;

AbstractInterface::FixedRadiusNearestNeighborsSearcher* build_fixed_radius_nearest_neighbors_searcher(spatial_searcher_t::spatial_searcher_t searcher_type,
                                                                                                      AbstractAuthority* authority);

AbstractInterface::NearestNeighborSearcher* build_nearest_neighbor_searcher(spatial_searcher_t::spatial_searcher_t searcher_type,
                                                                            AbstractAuthority* authority);

}
