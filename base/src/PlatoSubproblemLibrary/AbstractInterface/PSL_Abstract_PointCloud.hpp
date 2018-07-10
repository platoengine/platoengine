// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{
class Point;
class PointCloud;

namespace AbstractInterface
{

class PointCloud
{
public:
    PointCloud();
    virtual ~PointCloud();

    virtual Point get_point(const size_t& index) = 0;
    virtual size_t get_num_points() = 0;

    virtual PlatoSubproblemLibrary::PointCloud* build_point_cloud(std::vector<bool>& selected_nodes);
protected:
};

}

}
