// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_PointCloud.hpp"
#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{
class Point;

namespace example
{

class Interface_PointCloud : public AbstractInterface::PointCloud
{
public:
    Interface_PointCloud();
    virtual ~Interface_PointCloud();

    void set_num_points(size_t num_points);
    void set_point_data(size_t point_index, const std::vector<double>& data);

    virtual Point get_point(const size_t& index);
    virtual size_t get_num_points();

protected:
    std::vector<std::vector<double> > m_point_data;
};

}
}
