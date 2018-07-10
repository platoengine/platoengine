// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_ParallelVector.hpp"

#include <cstddef>
#include <vector>
#include <cassert>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{

ParallelVector::ParallelVector()
{
}

ParallelVector::~ParallelVector()
{
}

void ParallelVector::get_values(std::vector<double>& field)
{
    const size_t length = get_length();
    field.resize(length);
    for(size_t i = 0u; i < length; i++)
    {
        field[i] = get_value(i);
    }
}
void ParallelVector::set_values(const std::vector<double>& field)
{
    const size_t length = get_length();
    assert(field.size() == length);
    for(size_t i = 0u; i < length; i++)
    {
        set_value(i, field[i]);
    }
}

}
}
