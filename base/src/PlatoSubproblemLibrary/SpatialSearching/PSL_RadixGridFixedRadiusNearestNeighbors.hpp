// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_FixedRadiusNearestNeighborsSearcher.hpp"

#include <cstddef>
#include <vector>
#include <map>

namespace PlatoSubproblemLibrary
{
class PointCloud;
class Point;

class RadixGridFixedRadiusNearestNeighbors : public AbstractInterface::FixedRadiusNearestNeighborsSearcher
{
public:
    RadixGridFixedRadiusNearestNeighbors();
    virtual ~RadixGridFixedRadiusNearestNeighbors();

    // build searcher
    virtual void build(PlatoSubproblemLibrary::PointCloud* answer_points, double radius);
    // find neighbors within radius
    virtual void get_neighbors(PlatoSubproblemLibrary::Point* query_point,
                               std::vector<size_t>& neighbors_buffer,
                               size_t& num_neighbors);

protected:
    PlatoSubproblemLibrary::PointCloud* m_answer_points;
    double m_radius;

    double m_min_x_domain;
    double m_max_x_domain;
    double m_min_y_domain;
    double m_max_y_domain;
    double m_min_z_domain;
    double m_max_z_domain;

    double m_radix_step_x;
    double m_radix_step_y;
    double m_radix_step_z;

    std::map<std::pair<std::pair<size_t, size_t>, size_t>, std::vector<PlatoSubproblemLibrary::Point*> > m_radix_grid;
};

}
