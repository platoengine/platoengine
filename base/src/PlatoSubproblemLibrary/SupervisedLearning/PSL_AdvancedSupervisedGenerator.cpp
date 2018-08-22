#include "PSL_AdvancedSupervisedGenerator.hpp"

#include "PSL_SupervisedGenerator.hpp"
#include "PSL_Random.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"

#include <vector>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{

AdvancedSupervisedGenerator::AdvancedSupervisedGenerator(AbstractAuthority* authority) :
                SupervisedGenerator(),
                m_authority(authority)
{
}

AdvancedSupervisedGenerator::~AdvancedSupervisedGenerator()
{
}

void AdvancedSupervisedGenerator::get_sizes(int& output_enum_size,
                                            int& num_input_double_scalars,
                                            std::vector<int>& input_enum_sizes)
{
    output_enum_size = 2;
    num_input_double_scalars = 7;
    input_enum_sizes.resize(3u);
    input_enum_sizes[0] = 3;
    input_enum_sizes[1] = 2;
    input_enum_sizes[2] = 4;
}

void AdvancedSupervisedGenerator::generate(std::vector<double>& input_double_scalars,
                                           std::vector<int>& input_enums,
                                           int& actual_output_enum)
{
    // set input scalars
    input_double_scalars.resize(7u);
    input_double_scalars[0] = normal_rand_double(uniform_rand_double(-.5,.5),1.);
    input_double_scalars[1] = normal_rand_double(2.5,.25+uniform_rand_double(0.,.5));
    input_double_scalars[2] = uniform_rand_double();
    input_double_scalars[3] = uniform_rand_double(0.5,1.+uniform_rand_double(0.,2.));
    input_double_scalars[4] = uniform_rand_double(-1., 1. + uniform_rand_double(0., 2.));
    input_double_scalars[5] = uniform_rand_double(.8, .9) * input_double_scalars[0]
                              - uniform_rand_double(.9, 1.) * input_double_scalars[2]
                              + uniform_rand_double(-.1, .1);
    input_double_scalars[6] = uniform_rand_double(.8, .9) * input_double_scalars[1]
                              - uniform_rand_double(.9, 1.) * input_double_scalars[3]
                              + uniform_rand_double(-.2, 0.);

    // set input enums
    input_enums.resize(3u);
    input_enums[0] = rand_int(0, 3);
    input_enums[1] = rand_int(0, 2);
    input_enums[2] = rand_int(0, 4);

    // define linear separator for each enum
    std::vector<double> separator_slope(7u);
    separator_slope[0]=1.;
    separator_slope[1]=-2.;
    separator_slope[2]=2.;
    separator_slope[3]=-4./3.;
    separator_slope[4]=2.;
    separator_slope[5]=-1.;
    separator_slope[6]=2./3.;
    double separator_bias = 0.6;

    // divide the problem space up
    const int switch_case = input_enums[0]*2+input_enums[1];
    switch (switch_case) {
        case 0: {
            // 0, 0
            separator_slope[1] = -2.5;
            separator_slope[6] = 3/4;
            break;
        }
        case 1: {
            // 0, 1
            separator_slope[2] = 3;
            separator_slope[4] = 1.5;
            break;
        }
        case 2: {
            // 1, 0
            separator_slope[3] = -1;
            separator_slope[5] = -1.2;
            separator_bias = 0.4;
            break;
        }
        case 3: {
            // 1, 1
            separator_slope[2] = 2.8;
            separator_slope[4] = 2.5;
            break;
        }
        case 4: {
            // 2, 0
            separator_slope[0] = 0.2;
            separator_slope[5] = -1.1;
            separator_bias = 0.8;
            break;
        }
        case 5: {
            // 2, 1
            separator_slope[0] = 0.1;
            separator_slope[2] = 3.2;
            break;
        }
        default: {
            m_authority->utilities->fatal_error("switch case not matched. Aborting.\n\n");
            break;
        }
    }

    // get output class predictor
    const double separator_value =
            m_authority->dense_vector_operations->dot(separator_slope,input_double_scalars) - separator_bias;
    // choose from separator
    if(input_double_scalars[0]<0.) {
        actual_output_enum = (separator_value < 0.);
    } else {
        actual_output_enum = (0. < separator_value);
    }

    // randomly flip sometimes to make perfect prediction impossible
    if(uniform_rand_double()<0.05) {
        actual_output_enum = 1-actual_output_enum;
    }
}

}
