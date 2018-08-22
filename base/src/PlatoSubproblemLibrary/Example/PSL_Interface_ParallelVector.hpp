// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_ParallelVector.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

class Interface_ParallelVector : public AbstractInterface::ParallelVector
{
public:
    Interface_ParallelVector(std::vector<double> data = std::vector<double>(0));
    virtual ~Interface_ParallelVector();

    virtual size_t get_length();
    virtual double get_value(size_t index);
    virtual void set_value(size_t index, double value);

    std::vector<double> m_data;

protected:
};

}
}
