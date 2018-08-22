#pragma once

#include "PSL_SupervisedGenerator.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;

class AdvancedSupervisedGenerator : public SupervisedGenerator
{
public:
    AdvancedSupervisedGenerator(AbstractAuthority* authority);
    virtual ~AdvancedSupervisedGenerator();

    virtual void get_sizes(int& output_enum_size, int& num_input_double_scalars, std::vector<int>& input_enum_sizes);
    virtual void generate(std::vector<double>& input_double_scalars, std::vector<int>& input_enums, int& actual_output_enum);

protected:
    AbstractAuthority* m_authority;
};

}
