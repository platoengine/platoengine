// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_BruteForceNearestNeighbor.hpp"

#include "PSL_Point.hpp"
#include "PSL_PointCloud.hpp"

#include <cstddef>
#include <vector>
#include <cassert>

namespace PlatoSubproblemLibrary
{

BruteForceNearestNeighbor::BruteForceNearestNeighbor() :
        AbstractInterface::NearestNeighborSearcher()
{
}

BruteForceNearestNeighbor::~BruteForceNearestNeighbor()
{
}

// save answer points
void BruteForceNearestNeighbor::build(PlatoSubproblemLibrary::PointCloud* answer_points)
{
    m_answer_points = answer_points;
}

// find nearest neighbor
size_t BruteForceNearestNeighbor::get_neighbor(PlatoSubproblemLibrary::Point* query_point)
{
    Point* first_point = m_answer_points->get_point(0u);
    double min_dist = query_point->distance(first_point);
    size_t nearest_neighbor_index = first_point->get_index();

    // for each answer point
    const size_t num_answer_points = m_answer_points->get_num_points();
    assert(num_answer_points > 0u);
    for(size_t answer_index = 1u; answer_index < num_answer_points; answer_index++)
    {
        PlatoSubproblemLibrary::Point* answer_point = m_answer_points->get_point(answer_index);

        // if closest, update
        double this_dist = query_point->distance(answer_point);
        if(this_dist <= min_dist)
        {
            min_dist = this_dist;
            nearest_neighbor_index = answer_point->get_index();
        }
    }

    return nearest_neighbor_index;
}

}
