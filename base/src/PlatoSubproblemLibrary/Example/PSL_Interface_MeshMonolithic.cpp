// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_MeshMonolithic.hpp"

#include <vector>
#include <cstddef>
#include <set>

#include "PSL_Abstract_OptimizationMesh.hpp"
#include "PSL_Point.hpp"
#include "PSL_Implementation_MeshMonolithic.hpp"

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_MeshMonolithic::Interface_MeshMonolithic() :
        AbstractInterface::OptimizationMesh()
{

}

Interface_MeshMonolithic::~Interface_MeshMonolithic()
{
    m_mesh = NULL;
}

void Interface_MeshMonolithic::set_mesh(MeshMonolithic* mesh)
{
    m_mesh = mesh;
}

std::vector<size_t> Interface_MeshMonolithic::get_point_neighbors(const size_t& point_index)
{
    // get adjacent elements
    std::vector<size_t> adjacent_elements = m_mesh->get_adjacent_elements(point_index);
    const size_t num_adjacent_elements = adjacent_elements.size();

    // build unique node neighbors set
    std::set<size_t> result_set;
    for(size_t element_index = 0u; element_index < num_adjacent_elements; element_index++)
    {
        std::vector<size_t> adjacent_nodes = m_mesh->get_adjacent_nodes(adjacent_elements[element_index]);
        const size_t num_adjacent_nodes = adjacent_nodes.size();
        for(size_t node_index = 0u; node_index < num_adjacent_nodes; node_index++)
        {
            const size_t other_node = adjacent_nodes[node_index];
            if(other_node != point_index)
            {
                result_set.insert(other_node);
            }
        }
    }

    // convert to vector
    std::vector<size_t> result_vector(result_set.begin(), result_set.end());
    return result_vector;
}

Point Interface_MeshMonolithic::get_point(const size_t& index)
{
    return Point(index, m_mesh->get_node_location(index));
}

size_t Interface_MeshMonolithic::get_num_points()
{
    return m_mesh->get_num_nodes();
}

size_t Interface_MeshMonolithic::get_num_blocks()
{
    return 1u;
}

size_t Interface_MeshMonolithic::get_num_elements(size_t block_index)
{
    return m_mesh->get_num_elements();
}

std::vector<size_t> Interface_MeshMonolithic::get_nodes_from_element(size_t block_index, size_t element_index)
{
    return m_mesh->get_adjacent_nodes(element_index);
}

bool Interface_MeshMonolithic::is_block_optimizable(size_t block_index)
{
    return true;
}

}
}
