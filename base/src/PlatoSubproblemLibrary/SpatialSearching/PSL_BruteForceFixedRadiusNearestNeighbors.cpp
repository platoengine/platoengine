// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_BruteForceFixedRadiusNearestNeighbors.hpp"

#include "PSL_Point.hpp"
#include "PSL_PointCloud.hpp"

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{

BruteForceFixedRadiusNearestNeighbors::BruteForceFixedRadiusNearestNeighbors() :
        AbstractInterface::FixedRadiusNearestNeighborsSearcher(),
        m_answer_points(),
        m_radius(-1.)
{
}

BruteForceFixedRadiusNearestNeighbors::~BruteForceFixedRadiusNearestNeighbors()
{
}

// save answer points
void BruteForceFixedRadiusNearestNeighbors::build(PlatoSubproblemLibrary::PointCloud* answer_points, double radius)
{
    m_answer_points = answer_points;
    m_radius = radius;
}

// find neighbors of query point within radius
void BruteForceFixedRadiusNearestNeighbors::get_neighbors(PlatoSubproblemLibrary::Point* query_point,
                                                          std::vector<size_t>& neighbors_buffer,
                                                          size_t& num_neighbors)
{
    // for each answer point
    const size_t num_answer_points = m_answer_points->get_num_points();
    for(size_t answer_index = 0u; answer_index < num_answer_points; answer_index++)
    {
        PlatoSubproblemLibrary::Point* answer_point = m_answer_points->get_point(answer_index);

        // if within radius, add to results
        if(query_point->distance(answer_point) <= m_radius)
        {
            neighbors_buffer[num_neighbors++] = answer_point->get_index();
        }
    }
}

}
