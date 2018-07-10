#include "PSL_InterfaceToEngine_OptimizationMesh.hpp"

#include "PSL_Abstract_OptimizationMesh.hpp"
#include "PSL_Point.hpp"
#include "data_mesh.hpp"
#include "topological_element.hpp"

#include <cstddef>

namespace Plato
{

InterfaceToEngine_OptimizationMesh::InterfaceToEngine_OptimizationMesh(DataMesh* mesh) :
        PlatoSubproblemLibrary::AbstractInterface::OptimizationMesh(),
        m_mesh(mesh)
{
}

InterfaceToEngine_OptimizationMesh::~InterfaceToEngine_OptimizationMesh()
{
    m_mesh = NULL;
}

PlatoSubproblemLibrary::Point InterfaceToEngine_OptimizationMesh::get_point(const size_t& index)
{
    Real* X = m_mesh->getX();
    const double x = X[index];
    Real* Y = m_mesh->getY();
    const double y = Y[index];
    Real* Z = m_mesh->getZ();
    double z = 0;
    if(Z != NULL)
    {
        z = Z[index];
    }
    PlatoSubproblemLibrary::Point result(index, {x, y, z});
    return result;
}

size_t InterfaceToEngine_OptimizationMesh::get_num_points()
{
    return m_mesh->getNumNodes();
}

size_t InterfaceToEngine_OptimizationMesh::get_num_blocks()
{
    return m_mesh->getNumElemBlks();
}

size_t InterfaceToEngine_OptimizationMesh::get_num_elements(size_t block_index)
{
    return m_mesh->getNumElemInBlk(block_index);
}

std::vector<size_t> InterfaceToEngine_OptimizationMesh::get_nodes_from_element(size_t block_index, size_t element_index)
{
    std::vector<size_t> result;

    // get block
    Topological::Element& elblock = *(m_mesh->getElemBlk(block_index));

    // get element
    int* elemConnect = elblock.Connect(element_index);

    // get nodes
    int numNodesPerElem = elblock.getNnpe();
    for(int inode = 0; inode < numNodesPerElem; inode++)
    {
        result.push_back(elemConnect[inode]);
    }

    return result;
}

bool InterfaceToEngine_OptimizationMesh::is_block_optimizable(size_t block_index)
{
    // TODO: when is this true? when is this false?
    return true;
}

}

