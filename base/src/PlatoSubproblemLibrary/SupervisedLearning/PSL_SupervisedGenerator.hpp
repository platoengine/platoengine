#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{

class SupervisedGenerator
{
public:
    SupervisedGenerator();
    virtual ~SupervisedGenerator();

    virtual void get_sizes(int& output_enum_size, int& num_input_double_scalars, std::vector<int>& input_enum_sizes) = 0;
    virtual void generate(std::vector<double>& input_double_scalars, std::vector<int>& input_enums, int& actual_output_enum) = 0;

protected:

};

}
