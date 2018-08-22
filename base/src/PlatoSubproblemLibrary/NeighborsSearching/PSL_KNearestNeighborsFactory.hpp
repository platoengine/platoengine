#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class KNearestNeighborsSearcher;
class DataSequence;

KNearestNeighborsSearcher* build_knn_searcher(const k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t& type,
                                              AbstractAuthority* authority,
                                              DataSequence* sequence);

}
