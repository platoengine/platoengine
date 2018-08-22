// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_OverlapSearcher.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
class AxisAlignedBoundingBox;

class BoundingBoxBruteForce : public AbstractInterface::OverlapSearcher
{
public:
    BoundingBoxBruteForce();
    virtual ~BoundingBoxBruteForce();

    // build searcher
    virtual void build(const std::vector<AxisAlignedBoundingBox>& answer_boxes);
    // find overlaps
    virtual void get_overlaps(AxisAlignedBoundingBox* query_box, std::vector<size_t>& neighbors_buffer, size_t& num_neighbors);

protected:

    std::vector<AxisAlignedBoundingBox> m_answer_boxes;
};

}
