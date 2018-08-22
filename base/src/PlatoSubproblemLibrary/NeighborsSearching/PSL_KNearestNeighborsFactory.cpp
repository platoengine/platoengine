#include "PSL_KNearestNeighborsFactory.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_KNearestNeighborsSearcher.hpp"
#include "PSL_BruteForceKNNSearcher.hpp"
#include "PSL_RandomProjectionForestApproxKnn.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{

KNearestNeighborsSearcher* build_knn_searcher(const k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t& type,
                                              AbstractAuthority* authority,
                                              DataSequence* sequence)
{
    KNearestNeighborsSearcher* result = NULL;

    switch(type)
    {
        case k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t::brute_force_searcher:
        {
            result = new BruteForceKNNSearcher(authority);
            break;
        }
        case k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t::random_projection_forest_searcher:
        {
            result = new RandomProjectionForestApproxKnn(authority);
            break;
        }
        case k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t::TOTAL_NUM_SEARCHERS:
        default:
        {
            authority->utilities->fatal_error("PlatoSubproblemLibrary could not match enum to knn searcher. Aborting.\n\n");
            break;
        }
    }

    // if sequence == NULL, will be built with its own data sequence
    if(sequence != NULL)
    {
        result->set_data_sequence(sequence, false);
    }

    return result;
}

}
