// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_Mesh.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{

class OptimizationMesh : public Mesh
{
public:
    OptimizationMesh();
    virtual ~OptimizationMesh();

    virtual bool is_block_optimizable(size_t block_index) = 0;
private:
};

}

}
