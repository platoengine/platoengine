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

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{
namespace example
{

class MeshMonolithic
{
public:
    MeshMonolithic();
    ~MeshMonolithic();

    void build_from_structured_grid(size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                    double x_dist, double y_dist, double z_dist);

    size_t get_num_nodes() const;
    size_t get_num_elements() const;
    std::vector<size_t> get_adjacent_nodes(size_t element_index) const;
    std::vector<size_t> get_adjacent_elements(size_t node_index) const;
    std::vector<double> get_node_location(size_t node_index) const;

private:

    size_t get_index_helper(size_t x_index, size_t y_index, size_t z_index, size_t x_dimen, size_t y_dimen, size_t z_dimen) const;
    void set_node_index_to_adjacent_elements_helper(size_t x_index, size_t y_index, size_t z_index,
                                                    size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                                    double x_spacing, double y_spacing, double z_spacing);
    void set_element_index_to_adjacent_nodes_helper(size_t x_index, size_t y_index, size_t z_index,
                                                    size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                                    double x_spacing, double y_spacing, double z_spacing);
    void set_node_locations_helper(size_t x_index, size_t y_index, size_t z_index,
                                   size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                   double x_spacing, double y_spacing, double z_spacing);

    std::vector<std::vector<size_t> > m_node_index_to_adjacent_elements;
    std::vector<std::vector<size_t> > m_element_index_to_adjacent_nodes;
    std::vector<std::vector<double> > m_node_locations;
};

}
}
