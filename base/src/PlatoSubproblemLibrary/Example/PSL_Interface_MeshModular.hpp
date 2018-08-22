// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_OptimizationMesh.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
class Point;

namespace example
{

class ElementBlock;

class Interface_MeshModular : public AbstractInterface::OptimizationMesh
{
public:

    Interface_MeshModular();
    ~Interface_MeshModular();

    void set_mesh(ElementBlock* mesh);

    // depreciated from inheritance
    virtual std::vector<size_t> get_point_neighbors(const size_t& index);

    // PointCloud
    virtual Point get_point(const size_t& index);
    virtual size_t get_num_points();

    // Mesh
    virtual size_t get_num_blocks();
    virtual size_t get_num_elements(size_t block_index);
    virtual std::vector<size_t> get_nodes_from_element(size_t block_index, size_t element_index);

    // OptimizationMesh
    virtual bool is_block_optimizable(size_t block_index);

private:

    ElementBlock* m_mesh;
};

}
}
