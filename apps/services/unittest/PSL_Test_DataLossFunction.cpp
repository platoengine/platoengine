#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_DataLossFunction.hpp"
#include "PSL_DataLossFunctionFactory.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingDataLossFunction
{

PSL_TEST(DataLossFunction, squared_error_loss)
{
    set_rand_seed();
    example::Interface_BasicGlobalUtilities utilities;
    DataLossFunction* dlf = build_data_loss_function(data_loss_function_t::squared_error_loss, &utilities);

    // compute
    const std::vector<double> computed_input = {.1, .9, .8, -1., 1.2};
    const std::vector<double> true_input = {0., 0., 1., 0., 0.};
    std::vector<double> computed_gradient = {-3., -2.}; // intentionally superfluous
    const double computed_loss = dlf->compute_loss(computed_input, true_input, computed_gradient);

    // compare to expectations
    const double expected_loss = 1.65;
    EXPECT_FLOAT_EQ(expected_loss, computed_loss);
    const std::vector<double> expected_gradient = {.1, .9, -.2, -1., 1.2};
    expect_equal_float_vectors(computed_gradient, expected_gradient);

    delete dlf;
}

PSL_TEST(DataLossFunction, cross_entropy_loss)
{
    set_rand_seed();
    example::Interface_BasicGlobalUtilities utilities;
    DataLossFunction* dlf = build_data_loss_function(data_loss_function_t::cross_entropy_loss, &utilities);

    // compute
    const std::vector<double> computed_input = {.1, .9, .8, -1., 1.2};
    const std::vector<double> true_input = {0., 0., 1., 0., 0.};
    std::vector<double> computed_gradient = {-3., -2., -1., 0., 1., 2., 3.}; // intentionally superfluous
    const double computed_loss = dlf->compute_loss(computed_input, true_input, computed_gradient);

    // compare to expectations
    const double expected_loss = 1.449006169770361;
    EXPECT_FLOAT_EQ(expected_loss, computed_loss);
    const std::vector<double> expected_gradient = {.116599980795443, .259498029521695, -.765196472982170, .038812761966554,
                                                   .350285700698478};
    expect_equal_float_vectors(computed_gradient, expected_gradient);

    delete dlf;
}

}
}
