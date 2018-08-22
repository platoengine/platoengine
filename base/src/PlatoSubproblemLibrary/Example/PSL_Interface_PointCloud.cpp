// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_PointCloud.hpp"

#include "PSL_Abstract_PointCloud.hpp"
#include "PSL_Point.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_PointCloud::Interface_PointCloud() :
        AbstractInterface::PointCloud(),
        m_point_data()
{
}

Interface_PointCloud::~Interface_PointCloud()
{
    m_point_data.clear();
}

void Interface_PointCloud::set_num_points(size_t num_points)
{
    m_point_data.resize(num_points);
}

void Interface_PointCloud::set_point_data(size_t point_index, const std::vector<double>& data)
{
    m_point_data[point_index] = data;
}

Point Interface_PointCloud::get_point(const size_t& index)
{
    return Point(index, m_point_data[index]);
}

size_t Interface_PointCloud::get_num_points()
{
    return m_point_data.size();
}

}
}
