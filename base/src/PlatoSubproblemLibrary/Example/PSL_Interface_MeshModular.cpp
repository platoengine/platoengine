// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_MeshModular.hpp"

#include <cstddef>
#include <vector>
#include <set>

#include "PSL_Abstract_OptimizationMesh.hpp"
#include "PSL_Point.hpp"
#include "PSL_Implementation_MeshModular.hpp"

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_MeshModular::Interface_MeshModular() :
        AbstractInterface::OptimizationMesh()
{
}

Interface_MeshModular::~Interface_MeshModular()
{
    m_mesh = NULL;
}

void Interface_MeshModular::set_mesh(ElementBlock* mesh)
{
    m_mesh = mesh;
}

std::vector<size_t> Interface_MeshModular::get_point_neighbors(const size_t& index)
{
    // get adjacent elements
    std::vector<Element*> adjacent_elements = m_mesh->get_node(index)->get_adjacent_element();
    const size_t num_adjacent_elements = adjacent_elements.size();

    // build unique node neighbors set
    std::set<size_t> result_set;
    for(size_t element_index = 0u; element_index < num_adjacent_elements; element_index++)
    {
        std::vector<Node*> adjacent_nodes = adjacent_elements[element_index]->get_adjacent_nodes();
        const size_t num_adjacent_nodes = adjacent_nodes.size();
        for(size_t node_index = 0u; node_index < num_adjacent_nodes; node_index++) {
            const size_t other_node = adjacent_nodes[node_index]->get_index();
            if(other_node != index)
            {
                result_set.insert(other_node);
            }
        }
    }

    // convert to vector
    std::vector<size_t> result_vector(result_set.begin(), result_set.end());
    return result_vector;
}

Point Interface_MeshModular::get_point(const size_t& index)
{
    Node* node = m_mesh->get_node(index);
    return Point(node->get_index(), node->get_coordinates());
}

size_t Interface_MeshModular::get_num_points()
{
    return m_mesh->get_num_nodes();
}

size_t Interface_MeshModular::get_num_blocks()
{
    return 1u;
}

size_t Interface_MeshModular::get_num_elements(size_t block_index)
{
    return m_mesh->get_num_elements();
}

std::vector<size_t> Interface_MeshModular::get_nodes_from_element(size_t block_index, size_t element_index)
{
    // adjacent nodes
    std::vector<Node*> adjacent_nodes = m_mesh->get_element(element_index)->get_adjacent_nodes();

    // translate Node* to size_t indexes
    const size_t num_adjacent_nodes = adjacent_nodes.size();
    std::vector<size_t> adjacent_node_indexes(num_adjacent_nodes, 0u);
    for(size_t adjacent_node_index = 0u; adjacent_node_index < num_adjacent_nodes; adjacent_node_index++)
    {
        adjacent_node_indexes[adjacent_node_index] = adjacent_nodes[adjacent_node_index]->get_index();
    }

    return adjacent_node_indexes;
}

bool Interface_MeshModular::is_block_optimizable(size_t block_index)
{
    return true;
}

}
}
