#pragma once

#include "PSL_SupervisedGenerator.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class BasicSupervisedGenerator : public SupervisedGenerator
{
public:
    BasicSupervisedGenerator(const double& output_lower_transition, const double& output_upper_transition);
    virtual ~BasicSupervisedGenerator();

    virtual void get_sizes(int& output_enum_size, int& num_input_double_scalars, std::vector<int>& input_enum_sizes);
    virtual void generate(std::vector<double>& input_double_scalars, std::vector<int>& input_enums, int& actual_output_enum);

protected:
    double m_output_lower_transition;
    double m_output_upper_transition;

};

}
