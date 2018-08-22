#include "PSL_SpiralsSupervisedGenerator.hpp"

#include "PSL_SupervisedGenerator.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{

SpiralsSupervisedGenerator::SpiralsSupervisedGenerator(const int& num_classes) :
        SupervisedGenerator(),
        m_num_classes(num_classes)
{
}

SpiralsSupervisedGenerator::~SpiralsSupervisedGenerator()
{
}

void SpiralsSupervisedGenerator::get_sizes(int& output_enum_size,
                                           int& num_input_double_scalars,
                                           std::vector<int>& input_enum_sizes)
{
    output_enum_size = m_num_classes;
    num_input_double_scalars = 2;
    input_enum_sizes.resize(0u);
}

void SpiralsSupervisedGenerator::generate(std::vector<double>& input_double_scalars,
                                          std::vector<int>& input_enums,
                                          int& actual_output_enum)
{
    // choose class
    actual_output_enum = rand_int(0, m_num_classes - 1);

    // set scalars
    input_double_scalars.resize(2u);
    const double radius = uniform_rand_double();
    const double theta = uniform_rand_double(actual_output_enum * 4, (actual_output_enum + 1) * 4) + normal_rand_double(0., .2);
    input_double_scalars[0] = radius * cos(theta);
    input_double_scalars[1] = radius * sin(theta);

    // set enums
    input_enums.assign(0u, 0);
}

}
