/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_OverlapSearcher.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
class AxisAlignedBoundingBox;

class BoundingBoxMortonHierarchy : public AbstractInterface::OverlapSearcher
{
public:
    BoundingBoxMortonHierarchy();
    virtual ~BoundingBoxMortonHierarchy();

    // build searcher
    virtual void build(const std::vector<AxisAlignedBoundingBox>& answer_boxes);
    // find overlaps
    virtual void get_overlaps(AxisAlignedBoundingBox* query_box, std::vector<size_t>& neighbors_buffer, size_t& num_neighbors);

// public utilities

    void util_get_n_and_m(const int N, int& n, int& m) const;
    void util_smallest_two_power(const int n, std::vector<int>& smallest_two_powers) const;
    void util_summed_smallest_two_power(const int n,
                                        const std::vector<int>& smallest_two_powers,
                                        std::vector<int>& summed_smallest_two_powers) const;
    void util_indexes_to_place_two_nodes(const int n,
                                         const int m,
                                         const std::vector<int>& smallest_two_powers,
                                         std::vector<int>& place_two_nodes) const;
    void util_advanced_indexes_because_of_two_nodes(const int n,
                                                    const std::vector<int>& summed_smallest_two_power,
                                                    const std::vector<bool>& is_two_nodes_here,
                                                    std::vector<int>& advanced_indexes_because_of_two_nodes) const;
    void util_is_two_nodes_here(const int n,
                                const int m,
                                const std::vector<int>& place_two_nodes,
                                std::vector<bool>& is_two_nodes_here) const;
    void util_morton_sort_boxes(const int num_boxes,
                                const std::vector<AxisAlignedBoundingBox>& input_boxes,
                                std::vector<int>& sorted_indexes) const;
    bool util_confirm_hierarchy_properties() const;

private:
    unsigned int internal_expand_bits(unsigned int v) const;
    unsigned int internal_morton3D(float x, float y, float z) const;

    void internal_solve(AxisAlignedBoundingBox* query_box, std::vector<size_t>& neighbors_buffer, size_t& num_neighbors, size_t search_start) const;

    int internal_confirm_hierarchy_properties(const int i) const;

    std::vector<AxisAlignedBoundingBox> m_hierarchy;
    int m_num_boxes;
};

}
