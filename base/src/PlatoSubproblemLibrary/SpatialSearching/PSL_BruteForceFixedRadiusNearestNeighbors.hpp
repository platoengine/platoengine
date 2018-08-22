// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_FixedRadiusNearestNeighborsSearcher.hpp"

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{
class PointCloud;
class Point;

class BruteForceFixedRadiusNearestNeighbors : public AbstractInterface::FixedRadiusNearestNeighborsSearcher
{
public:
    BruteForceFixedRadiusNearestNeighbors();
    virtual ~BruteForceFixedRadiusNearestNeighbors();

    // build searcher
    virtual void build(PlatoSubproblemLibrary::PointCloud* answer_points, double radius);
    // find neighbors within radius
    virtual void get_neighbors(PlatoSubproblemLibrary::Point* query_point,
                               std::vector<size_t>& neighbors_buffer,
                               size_t& num_neighbors);

protected:
    PlatoSubproblemLibrary::PointCloud* m_answer_points;
    double m_radius;

};

}
