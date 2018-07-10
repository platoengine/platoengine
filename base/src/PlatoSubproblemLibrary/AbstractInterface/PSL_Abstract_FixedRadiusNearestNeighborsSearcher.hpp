// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{
class PointCloud;
class Point;

namespace AbstractInterface
{

class FixedRadiusNearestNeighborsSearcher
{
public:
    FixedRadiusNearestNeighborsSearcher();
    virtual ~FixedRadiusNearestNeighborsSearcher();

    // build searcher
    virtual void build(PlatoSubproblemLibrary::PointCloud* answer_points, double radius) = 0;
    // find neighbors within radius
    virtual void get_neighbors(PlatoSubproblemLibrary::Point* query_point, std::vector<size_t>& neighbors_buffer, size_t& num_neighbors) = 0;

protected:

};

}

}
