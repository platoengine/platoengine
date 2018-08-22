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
