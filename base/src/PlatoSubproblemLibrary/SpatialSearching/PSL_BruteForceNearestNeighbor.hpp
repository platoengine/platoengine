// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_NearestNeighborSearcher.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
class PointCloud;
class Point;

class BruteForceNearestNeighbor : public AbstractInterface::NearestNeighborSearcher
{
public:
    BruteForceNearestNeighbor();
    virtual ~BruteForceNearestNeighbor();

    // build searcher
    virtual void build(PlatoSubproblemLibrary::PointCloud* answer_points);
    // find nearest neighbor
    virtual size_t get_neighbor(PlatoSubproblemLibrary::Point* query_point);

protected:

    PlatoSubproblemLibrary::PointCloud* m_answer_points;
};

}
