#include "PSL_UnitTestingHelper.hpp"

#include "PSL_ActivationFunctionFactory.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ActivationFunction.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

#define PSL_TestingActivationFunction_Allocate \
        example::Interface_BasicGlobalUtilities utilities; \
        const std::vector<double> inputs = {-1.5, -1., -.75, 0., .15, 1., 1.5};

namespace TestingActivationFunction
{

PSL_TEST(ActivationFunction, sigmoid)
{
    set_rand_seed();
    PSL_TestingActivationFunction_Allocate
    ActivationFunction* af = build_activation_function(activation_function_t::sigmoid_activation, 0., &utilities);

    // compute outputs
    std::vector<double> computed_output;
    std::vector<double> computed_output_gradients;
    af->activate(inputs, computed_output, computed_output_gradients);

    // compare to gold
    const std::vector<double> gold_outputs = {.182425523806356, .268941421369995, .320821300824607, .5, .537429845343750,
                                              .731058578630005, .817574476193644};
    expect_equal_float_vectors(gold_outputs, computed_output);
    const std::vector<double> gold_output_gradients = {.149146452070333, .196611933241482, .217894993761814, .25,
                                                       .248599006677543, .196611933241482, .149146452070333};
    expect_equal_float_vectors(gold_output_gradients, computed_output_gradients);

    delete af;
}

PSL_TEST(ActivationFunction, rectified_linear_unit)
{
    set_rand_seed();
    PSL_TestingActivationFunction_Allocate
    ActivationFunction* af = build_activation_function(activation_function_t::rectified_linear_unit, 0., &utilities);

    // compute outputs
    std::vector<double> computed_output;
    std::vector<double> computed_output_gradients;
    af->activate(inputs, computed_output, computed_output_gradients);

    // compare to gold
    const std::vector<double> gold_outputs = {0., 0., 0., 0., 0.15, 1., 1.5};
    expect_equal_float_vectors(gold_outputs, computed_output);
    const std::vector<double> gold_output_gradients = {0., 0., 0., 1., 1., 1., 1.};
    expect_equal_float_vectors(gold_output_gradients, computed_output_gradients);

    delete af;
}

PSL_TEST(ActivationFunction, leaky_rectified_linear_unit)
{
    set_rand_seed();
    PSL_TestingActivationFunction_Allocate
    ActivationFunction* af = build_activation_function(activation_function_t::leaky_rectified_linear_unit, .001, &utilities);

    // compute outputs
    std::vector<double> computed_output;
    std::vector<double> computed_output_gradients;
    af->activate(inputs, computed_output, computed_output_gradients);

    // compare to gold
    const std::vector<double> gold_outputs = {-.0015, -.001, -.00075, 0., .15, 1., 1.5};
    expect_equal_float_vectors(gold_outputs, computed_output);
    const std::vector<double> gold_output_gradients = {.001, .001, .001, 1., 1., 1., 1.};
    expect_equal_float_vectors(gold_output_gradients, computed_output_gradients);

    delete af;
}

PSL_TEST(ActivationFunction, soft_plus)
{
    set_rand_seed();
    PSL_TestingActivationFunction_Allocate
    ActivationFunction* af = build_activation_function(activation_function_t::soft_plus_activation, 0., &utilities);

    // compute outputs
    std::vector<double> computed_output;
    std::vector<double> computed_output_gradients;
    af->activate(inputs, computed_output, computed_output_gradients);

    // compare to gold
    const std::vector<double> gold_outputs = {0.201413277982752, 0.313261687518223, 0.386871006114900, 0.693147180559945,
                                              0.770957047789532, 1.313261687518223, 1.701413277982752};
    expect_equal_float_vectors(gold_outputs, computed_output);
    const std::vector<double> gold_output_gradients = {0.182425523806356, 0.268941421369995, 0.320821300824607, 0.5,
                                                       0.537429845343750, 0.731058578630005, 0.817574476193644};
    expect_equal_float_vectors(gold_output_gradients, computed_output_gradients);

    delete af;
}

}
}
