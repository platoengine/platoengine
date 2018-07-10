// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_ParallelVector.hpp"

#include "PSL_Abstract_ParallelVector.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_ParallelVector::Interface_ParallelVector(std::vector<double> data) :
        AbstractInterface::ParallelVector(),
        m_data(data)
{

}

Interface_ParallelVector::~Interface_ParallelVector()
{
    m_data.clear();
}

size_t Interface_ParallelVector::get_length()
{
    return m_data.size();
}

double Interface_ParallelVector::get_value(size_t index)
{
    return m_data[index];
}

void Interface_ParallelVector::set_value(size_t index, double value)
{
    m_data[index] = value;
}

}
}
