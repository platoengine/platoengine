// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{
class PointCloud;
class Point;

namespace AbstractInterface
{

class NearestNeighborSearcher
{
public:
    NearestNeighborSearcher();
    virtual ~NearestNeighborSearcher();

    // build searcher
    virtual void build(PlatoSubproblemLibrary::PointCloud* answer_points) = 0;
    // find nearest neighbor
    virtual size_t get_neighbor(PlatoSubproblemLibrary::Point* query_point) = 0;

protected:

};

}

}
