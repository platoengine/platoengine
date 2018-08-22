// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_OptimizationMesh.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
class Point;
namespace AbstractInterface
{
class OptimizationMesh;
}

namespace example
{

class MeshMonolithic;

class Interface_MeshMonolithic : public AbstractInterface::OptimizationMesh
{
public:

    Interface_MeshMonolithic();
    ~Interface_MeshMonolithic();

    void set_mesh(MeshMonolithic* mesh);

    // depreciated-ish
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

    MeshMonolithic* m_mesh;
};

}
}
