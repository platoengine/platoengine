// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_BoundingBoxBruteForce.hpp"

#include "PSL_Abstract_OverlapSearcher.hpp"
#include "PSL_AxisAlignedBoundingBox.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

BoundingBoxBruteForce::BoundingBoxBruteForce() :
        AbstractInterface::OverlapSearcher(),
        m_answer_boxes()
{
}

BoundingBoxBruteForce::~BoundingBoxBruteForce()
{
}

// build searcher
void BoundingBoxBruteForce::build(const std::vector<AxisAlignedBoundingBox>& answer_boxes)
{
    m_answer_boxes.assign(answer_boxes.begin(), answer_boxes.end());
}

// find overlaps
void BoundingBoxBruteForce::get_overlaps(AxisAlignedBoundingBox* query_box, std::vector<size_t>& neighbors_buffer, size_t& num_neighbors)
{
    // for each answer box
    const size_t num_answer_boxes = m_answer_boxes.size();
    for(size_t answer_index = 0u; answer_index < num_answer_boxes; answer_index++)
    {
        // if overlap
        if(query_box->overlap(m_answer_boxes[answer_index]))
        {
            neighbors_buffer[num_neighbors++] = m_answer_boxes[answer_index].get_id();
        }
    }
}

}
