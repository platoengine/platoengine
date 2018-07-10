// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{

class ParallelVector
{
public:
    ParallelVector();
    virtual ~ParallelVector();

    virtual size_t get_length() = 0;
    virtual double get_value(size_t index) = 0;
    virtual void set_value(size_t index, double value) = 0;
    void get_values(std::vector<double>& field);
    void set_values(const std::vector<double>& field);

protected:
};

}
}
