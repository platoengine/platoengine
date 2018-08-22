// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_PointCloud.hpp"

#include "PSL_Point.hpp"
#include "PSL_AxisAlignedBoundingBox.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

PointCloud::PointCloud()
    : m_points() {

}

PointCloud::PointCloud(PointCloud* other, const std::vector<size_t>& indexes_to_transfer)
    : m_points(indexes_to_transfer.size())
{
    const size_t num_indexes_to_transfer = indexes_to_transfer.size();
    for(size_t i = 0u; i < num_indexes_to_transfer; i++)
    {
        m_points[i] = *other->get_point(indexes_to_transfer[i]);
    }
}

PointCloud::~PointCloud() {

}

void PointCloud::resize(size_t num_points)
{
    m_points.resize(num_points);
}

void PointCloud::assign(std::vector<Point>& points)
{
    m_points.assign(points.begin(), points.end());
}

void PointCloud::assign(size_t index, const Point& point)
{
    m_points[index] = point;
}

void PointCloud::push_back(const Point& point)
{
    m_points.push_back(point);
}

size_t PointCloud::get_num_points() const
{
    return m_points.size();
}

Point* PointCloud::get_point(size_t index)
{
    return &m_points[index];
}

AxisAlignedBoundingBox PointCloud::get_bound()
{
    AxisAlignedBoundingBox result(0., 0., 0., 0u);

    // if no points, return origin
    if(m_points.size() == 0u)
    {
        return result;
    }

    // build result to contain each point
    result.set(&m_points[0]);
    const size_t num_points = m_points.size();
    for(size_t index = 1u; index < num_points; index++)
    {
        result.grow_to_include(&m_points[index]);
    }

    return result;
}

}
