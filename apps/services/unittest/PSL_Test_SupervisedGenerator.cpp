/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_SupervisedGenerator.hpp"
#include "PSL_SupervisedGeneratorTester.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_DiscreteObjective.hpp"
#include "PSL_BasicSupervisedGenerator.hpp"
#include "PSL_SpiralsSupervisedGenerator.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_IterativeSelection.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <string>
#include <mpi.h>
#include <cmath>
#include <math.h>
#include <iostream>

namespace PlatoSubproblemLibrary
{
namespace TestingSupervisedGenerator
{

bool passed_BasicSupervisedGenerator()
{
    AbstractAuthority authority;

    // sampler
    BasicSupervisedGenerator sampler(.01, 5.1);

    // tester
    SupervisedGeneratorTester tester(&authority);

    // parameters for training
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();

    // test
    const int num_train = 385;
    const int num_test = 150;
    const double testing_accuracy = tester.get_accuracy(&sampler, num_train, &parameter_data, num_test);

    // expect accuracy
    const double minimum_accuracy = .8;
    return (testing_accuracy >= minimum_accuracy);
}

PSL_TEST(SupervisedGenerator, basicSupervisedGeneratorTestPass)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(passed_BasicSupervisedGenerator, 4, 16, .65);
}

double getAccuracy_SpiralsSupervisedGenerator(ParameterData* parameter_data)
{
    AbstractAuthority authority;

    // sampler
    const int num_class = 3;
    SpiralsSupervisedGenerator generator(num_class);

    // tester
    SupervisedGeneratorTester tester(&authority);

    // test
    const int num_train = 250 * num_class;
    const int num_test = 50 * num_class;
    const double testing_accuracy = tester.get_accuracy(&generator, num_train, parameter_data, num_test);
    return testing_accuracy;
}
bool passed_SpiralsSupervisedGenerator()
{
    // parameters for training
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();

    const double testing_accuracy = getAccuracy_SpiralsSupervisedGenerator(&parameter_data);

    // expect accuracy
    const double minimum_accuracy = .70;
    return (testing_accuracy >= minimum_accuracy);
}
PSL_TEST(SupervisedGenerator, spiralsSupervisedGeneratorTestPass)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(passed_SpiralsSupervisedGenerator, 10, 50, .74);
}

class GeneratorErrorObjective : public DiscreteObjective
{
public:
    GeneratorErrorObjective() :
        DiscreteObjective()
    {
    }

    virtual ~GeneratorErrorObjective()
    {
    }

    virtual void get_domain(std::vector<double>& inclusive_lower,
                            std::vector<double>& inclusive_upper,
                            std::vector<int>& num_values)
    {
        inclusive_lower = {.5, 1., 1., 0., 2.};
        inclusive_upper = {2., 6., 6., 5., 4.};
        num_values =      {4,  6,  6,  6,  3};
    }
    virtual double evaluate(const std::vector<double>& parameters)
    {
        assert(parameters.size() == 5u);

        ParameterData parameter_data;
        parameter_data.defaults_for_feedForwardNeuralNetwork();
        parameter_data.set_internal_activation_function(activation_function_t::activation_function_t::sigmoid_activation);
        parameter_data.set_internal_activation_function_auxiliary_parameter(-1.);
        parameter_data.set_final_activation_function(activation_function_t::activation_function_t::sigmoid_activation);
        parameter_data.set_final_activation_function_auxiliary_parameter(-1.);
        parameter_data.set_data_loss_function(data_loss_function_t::data_loss_function_t::squared_error_loss);

        // parameter 0
        parameter_data.set_initial_learning_rate(pow(2., parameters[0]));
        // parameter 1
        parameter_data.set_max_epochs(int(round(pow(2., parameters[1]))));
        // parameter 2
        parameter_data.set_batch_size(int(round(pow(2., parameters[2]))));
        // parameter 3
        // TODO
        // parameter 4
        parameter_data.set_validation_holdout(pow(2., parameters[4])/100.);

        const double error = 1. - getAccuracy_SpiralsSupervisedGenerator(&parameter_data);
        return error;
    }

protected:

};
PSL_TEST(SupervisedGenerator, spiralsErrorMinimization)
{
    set_rand_seed();
    return;
    PSL_SerialOnlyTest

    // define objective
    AbstractAuthority authority;
    GeneratorErrorObjective obj;

    // define searcher
    IterativeSelection optimizer(&authority);
    optimizer.set_verbose(true);
    optimizer.set_objective(&obj);

    // find a minimum
    std::vector<double> best_parameters;
    const double found_min = optimizer.find_min(best_parameters);
    EXPECT_GT(1. - found_min, .725);

    // report min
    best_parameters.push_back(found_min);
    authority.utilities->print(best_parameters, true);
}

}
}
