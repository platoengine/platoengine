#pragma once

#include "PSL_Abstract_OptimizationMesh.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{
class Point;
}
class DataMesh;

namespace Plato
{

class InterfaceToEngine_OptimizationMesh : public PlatoSubproblemLibrary::AbstractInterface::OptimizationMesh
{
public:
    InterfaceToEngine_OptimizationMesh(DataMesh* mesh);
    virtual ~InterfaceToEngine_OptimizationMesh();

    virtual PlatoSubproblemLibrary::Point get_point(const size_t& index);
    virtual size_t get_num_points();

    virtual size_t get_num_blocks();
    virtual size_t get_num_elements(size_t block_index);
    virtual std::vector<size_t> get_nodes_from_element(size_t block_index, size_t element_index);

    virtual bool is_block_optimizable(size_t block_index);

private:
    DataMesh* m_mesh;
};

}
