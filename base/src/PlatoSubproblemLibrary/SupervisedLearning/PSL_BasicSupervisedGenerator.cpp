#include "PSL_BasicSupervisedGenerator.hpp"

#include "PSL_Random.hpp"
#include "PSL_SupervisedGenerator.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

BasicSupervisedGenerator::BasicSupervisedGenerator(const double& output_lower_transition, const double& output_upper_transition) :
        SupervisedGenerator(),
        m_output_lower_transition(output_lower_transition),
        m_output_upper_transition(output_upper_transition)
{
}

BasicSupervisedGenerator::~BasicSupervisedGenerator()
{
}

void BasicSupervisedGenerator::get_sizes(int& output_enum_size, int& num_input_double_scalars, std::vector<int>& input_enum_sizes)
{
    output_enum_size = 2;
    num_input_double_scalars = 1;
    input_enum_sizes.assign(1u, 2);
}

void BasicSupervisedGenerator::generate(std::vector<double>& input_double_scalars,
                                        std::vector<int>& input_enums,
                                        int& actual_output_enum)
{
    // define ranges
    const double sep = m_output_upper_transition - m_output_lower_transition;
    const double lower_range = m_output_lower_transition - 2. * sep;
    const double upper_range = m_output_upper_transition + 1. * sep;

    // sample
    const double scalar = uniform_rand_double(lower_range, upper_range);
    input_double_scalars.assign(1u, scalar);
    const int input_enum = rand_int(0, 2);
    input_enums.assign(1u, input_enum);

    // determine actual output
    const bool below_lower = (scalar < m_output_lower_transition);
    const bool above_upper = (m_output_upper_transition < scalar);
    if((input_enum == 1) && (below_lower || above_upper))
    {
        actual_output_enum = 1;
    }
    else
    {
        actual_output_enum = 0;
    }
}

}
