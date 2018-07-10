// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_PointCloud.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{
class PointCloud;

class Mesh : public AbstractInterface::PointCloud
{
public:
    Mesh();
    virtual ~Mesh();

    virtual size_t get_num_blocks() = 0;
    virtual size_t get_num_elements(size_t block_index) = 0;
    virtual std::vector<size_t> get_nodes_from_element(size_t block_index, size_t element_index) = 0;
protected:
};

}

}
