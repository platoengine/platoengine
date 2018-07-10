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

// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include <vector>
#include <cstddef>
#include <utility>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class Point;

namespace example
{

class ElementBlock;
class Element;
class Node;

class ElementBlock
{
public:
    ElementBlock();
    ~ElementBlock();

    void build_from_structured_grid(size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                    double x_dist, double y_dist, double z_dist,
                                    int rank, int num_processors);
    void build_random_subset_of_structured_grid(size_t approx_points_per_processor,
                                                double x_dist, double y_dist, double z_dist,
                                                AbstractAuthority* authority);

    void random_perturb_local_nodal_locations(double max_perturb);

    size_t get_num_nodes() const;
    Node* get_node(size_t index);
    size_t get_num_elements() const;
    Element* get_element(size_t index);
    int get_global_num_nodes() const;
    int get_global_num_elements() const;

    void get_shared_node_data(std::vector<std::vector<std::pair<size_t, size_t> > >& data);
    void get_global_ids(std::vector<size_t>& globals);

    Point* get_global_point(size_t index);

private:

    size_t get_index_helper(size_t x_index, size_t y_index, size_t z_index,
                            size_t x_dimen, size_t y_dimen, size_t z_dimen) const;
    void get_xyz_helper(size_t single_index,
                        size_t x_dimen, size_t y_dimen, size_t z_dimen,
                        size_t& x_index, size_t& y_index, size_t& z_index) const;
    void set_nodes_helper(size_t x_index, size_t y_index, size_t z_index,
                          size_t x_dimen, size_t y_dimen, size_t z_dimen,
                          double x_start, double y_start, double z_start,
                          double x_spacing, double y_spacing, double z_spacing);
    void set_elements_helper(size_t x_index, size_t y_index, size_t z_index,
                             size_t x_dimen, size_t y_dimen, size_t z_dimen,
                             double x_spacing, double y_spacing, double z_spacing);
    void add_shared_nodes_helper(size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                 int rank, size_t num_processors, size_t parallel_cube_dimension,
                                 int change_processor_x, int change_processor_y, int change_processor_z,
                                 size_t x_processor_index, size_t y_processor_index, size_t z_processor_index);
    void determined_locally_owned_elements_for_random_subset(std::vector<int>& locally_owned_elements,
                                                             double x_dist, double y_dist, double z_dist,
                                                             AbstractAuthority* authority);
    void determine_locally_aware_node(std::vector<bool>& is_locally_aware,
                                      const std::vector<int>& locally_owned_elements);

    std::vector<Element> m_elements;
    std::vector<Node> m_nodes;

    std::vector<std::vector<std::pair<size_t, size_t> > > m_processor_then_vector_of_local_and_nonlocal_pairs;
    std::vector<size_t> m_global_ids;
    int m_global_num_nodes;
    int m_global_num_elements;

    ElementBlock* m_global_block;
};

class Element
{
public:
    Element(ElementBlock* block, size_t index, const std::vector<Node*>& nodes);
    Element();
    ~Element();

    ElementBlock* get_block();
    size_t get_index();
    std::vector<Node*> get_adjacent_nodes();
    Point get_center();

private:

    ElementBlock* m_block;
    size_t m_index;
    std::vector<Node*> m_nodes;
};

class Node
{
public:
    Node(ElementBlock* block, size_t index, const std::vector<Element*>& elements, const std::vector<double>& coordinates);
    Node();
    ~Node();

    ElementBlock* get_block() const;
    size_t get_index() const;
    std::vector<Element*> get_adjacent_element() const;
    std::vector<double> get_coordinates() const;
    Point* get_point() const;

    Node& operator=(const Node& source);

private:

    ElementBlock* m_block;
    std::vector<Element*> m_elements;
    Point* m_point;
};

}
}
