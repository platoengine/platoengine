// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_OverlapSearcher.hpp"

#include "PSL_Abstract_FixedRadiusNearestNeighborsSearcher.hpp"
#include "PSL_Point.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_AxisAlignedBoundingBox.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{

OverlapSearcher::OverlapSearcher() :
        FixedRadiusNearestNeighborsSearcher(),
        m_radius(-1)
{
}

OverlapSearcher::~OverlapSearcher()
{
}

// build searcher
void OverlapSearcher::build(PlatoSubproblemLibrary::PointCloud* answer_points, double radius)
{
    m_radius = radius;

    // build boxes
    const size_t num_points = answer_points->get_num_points();
    std::vector<AxisAlignedBoundingBox> answer_boxes(num_points);
    for(size_t index = 0u; index < num_points; index++)
    {
        answer_boxes[index].set(answer_points->get_point(index));
    }

    // build searcher from boxes
    this->build(answer_boxes);
}

// find neighbors within radius
void OverlapSearcher::get_neighbors(PlatoSubproblemLibrary::Point* query_point, std::vector<size_t>& neighbors_buffer, size_t& num_neighbors)
{
    // build query_box
    AxisAlignedBoundingBox query_box;
    query_box.set(query_point);
    query_box.grow_in_each_axial_direction(m_radius);

    // query
    this->get_overlaps(&query_box, neighbors_buffer, num_neighbors);
}

}

}
