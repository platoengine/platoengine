// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_FixedRadiusNearestNeighborsSearcher.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
class PointCloud;
class Point;
class AxisAlignedBoundingBox;

namespace AbstractInterface
{

class OverlapSearcher : public FixedRadiusNearestNeighborsSearcher
{
public:
    OverlapSearcher();
    virtual ~OverlapSearcher();

    // public interface from FixedRadiusNearestNeighborsSearcher
    // build searcher
    virtual void build(PlatoSubproblemLibrary::PointCloud* answer_points, double radius);
    // find neighbors within radius
    virtual void get_neighbors(PlatoSubproblemLibrary::Point* query_point, std::vector<size_t>& neighbors_buffer, size_t& num_neighbors);

    // build searcher
    virtual void build(const std::vector<AxisAlignedBoundingBox>& answer_boxes) = 0;
    // find overlaps
    virtual void get_overlaps(AxisAlignedBoundingBox* query_box, std::vector<size_t>& neighbors_buffer, size_t& num_neighbors) = 0;

protected:

    // neighbor radius
    double m_radius;

};

}

}
