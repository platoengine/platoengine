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

#include "PSL_ClassificationArchive.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_FeedForwardNeuralNetwork.hpp"
#include "PSL_FindDataset.hpp"
#include "PSL_GradientCheck.hpp"
#include "PSL_ClassificationAccuracyEstimator.hpp"
#include "PSL_CrossValidationEstimator.hpp"
#include "PSL_SplitTrainTestEstimator.hpp"
#include "PSL_DiscreteObjective.hpp"
#include "PSL_IterativeSelection.hpp"
#include "PSL_CrossValidationErrorDiscreteObjective.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ClassificationAssessor.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_MultiLayoutVector.hpp"
#include "PSL_VectorCoding.hpp"
#include "PSL_Random.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_AdvancedSupervisedGenerator.hpp"
#include "PSL_SupervisedGeneratorTester.hpp"

#include <string>
#include <iostream>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{

namespace TestingFeedForwardNeuralNetwork
{

class FFNNErrorObjective : public CrossValidationErrorDiscreteObjective
{
public:
    FFNNErrorObjective(AbstractAuthority* authority, const std::vector<datasets_t::datasets_t>& datasets) :
            CrossValidationErrorDiscreteObjective(authority, datasets)
    {
    }
    virtual ~FFNNErrorObjective()
    {
    }

    virtual void get_domain(std::vector<double>& inclusive_lower,
                            std::vector<double>& inclusive_upper,
                            std::vector<int>& num_values)
    {
        //                 0    1    2   3
        inclusive_lower = {1.3, 11., 0., 0.};
        inclusive_upper = {2.,  15., 7., 5.};
        num_values =      {8,   5,   8,  6};
    }

protected:
    virtual void get_parameters(const std::vector<double>& parameters, ParameterData* parameter_data)
    {
        parameter_data->defaults_for_feedForwardNeuralNetwork();

        // parameter 0
        parameter_data->set_initial_learning_rate(parameters[0]);
        // parameter 1
        parameter_data->set_batch_size(parameters[1]);
        // parameter 2
        switch(int(round(parameters[2])))
        {
            case 0:
            {
                parameter_data->set_preprocessor(preprocessor_t::preprocessor_t::standardization_preprocessor);
                break;
            }
            case 1:
            {
                parameter_data->set_preprocessor(preprocessor_t::preprocessor_t::bipolar_normalization_preprocessor);
                break;
            }
            case 2:
            {
                parameter_data->set_preprocessor(preprocessor_t::preprocessor_t::skew_normalization_preprocessor);
                break;
            }
            case 3:
            {
                parameter_data->set_preprocessor(preprocessor_t::preprocessor_t::PCA_preprocessor);
                break;
            }
            case 4:
            {
                std::vector<preprocessor_t::preprocessor_t> types =
                {
                    preprocessor_t::preprocessor_t::standardization_preprocessor,
                    preprocessor_t::preprocessor_t::PCA_preprocessor
                };
                parameter_data->set_preprocessor(types);
                break;
            }
            case 5:
            {
                std::vector<preprocessor_t::preprocessor_t> types =
                {
                    preprocessor_t::preprocessor_t::bipolar_normalization_preprocessor,
                    preprocessor_t::preprocessor_t::PCA_preprocessor
                };
                parameter_data->set_preprocessor(types);
                break;
            }
            case 6:
            {
                std::vector<preprocessor_t::preprocessor_t> types =
                {
                    preprocessor_t::preprocessor_t::skew_normalization_preprocessor,
                    preprocessor_t::preprocessor_t::PCA_preprocessor
                };
                parameter_data->set_preprocessor(types);
                break;
            }
            case 7:
            {
                std::vector<preprocessor_t::preprocessor_t> types =
                {
                    preprocessor_t::preprocessor_t::skew_normalization_preprocessor,
                    preprocessor_t::preprocessor_t::bipolar_normalization_preprocessor
                };
                parameter_data->set_preprocessor(types);
                break;
            }
            default:
            {
                m_authority->utilities->fatal_error("unmatched parameter\n\n");
                break;
            }
        }
        // parameter 3
        parameter_data->set_PCA_upper_variation_fraction(1.-(std::pow(2., parameters[3])/100.));
    }

    virtual double compute_result(const std::vector<double>& results)
    {
        return sum(results);
    }

};

PSL_TEST(FeedForwardNeuralNetwork, errorMinimization)
{
    set_rand_seed();
    return;
    PSL_SerialOnlyTest

    // allocate
    AbstractAuthority authority;

    // define objective
    /* std::vector<datasets_t::datasets_t> datasets = {datasets_t::datasets_t::iris_dataset,
                                                    datasets_t::datasets_t::pop_failures_dataset,
                                                    datasets_t::datasets_t::nicotine_dataset,
                                                    datasets_t::datasets_t::lsvt_dataset,
                                                    datasets_t::datasets_t::frogs_MFCCs_dataset,
                                                    datasets_t::datasets_t::biodeg_dataset,
                                                    datasets_t::datasets_t::wilt_dataset}; */
    std::vector<datasets_t::datasets_t> datasets = {datasets_t::datasets_t::iris_dataset,
                                                    datasets_t::datasets_t::pop_failures_dataset,
                                                    datasets_t::datasets_t::nicotine_dataset,
                                                    datasets_t::datasets_t::lsvt_dataset,
                                                    datasets_t::datasets_t::biodeg_dataset,
                                                    datasets_t::datasets_t::waveform_dataset,
                                                    datasets_t::datasets_t::wine_quality_dataset,
                                                    datasets_t::datasets_t::cervical_cancer_dataset};
    FFNNErrorObjective obj(&authority, datasets);

    // define searcher
    IterativeSelection optimizer(&authority);
    optimizer.set_verbose(true);
    optimizer.set_objective(&obj);

    // find a minimum
    std::vector<double> best_parameters;
    const double found_min = optimizer.find_min(best_parameters);
    EXPECT_GT(found_min, .0);

    // report min
    best_parameters.push_back(found_min);
    authority.utilities->print(best_parameters, true);
}

bool passed_FeedForwardNeuralNetwork_Split_iris()
{
    // allocate needed interfaces
    AbstractAuthority authority;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::iris_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(5);
    parameter_data.set_validation_holdout(.25);
    const double testing_holdout = .2;
    const double expected_accuracy = .75;

    // write dataset
    FindDataset finder(&authority);
    const std::string archive_filename = finder.find(dataset);

    // assess
    SplitTrainTestEstimator estimator(&authority);
    ClassificationAssessor assessor(&authority);
    estimator.initialize(archive_filename, false, testing_holdout);
    estimator.estimate_accuracy(&parameter_data, &assessor);
    const double actual_accuracy = assessor.get_classification_accuracy();

    return (actual_accuracy >= expected_accuracy);
}
PSL_TEST(FeedForwardNeuralNetwork, passSplitIris)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(passed_FeedForwardNeuralNetwork_Split_iris, 3, 15, 0.73);
}

bool passed_FeedForwardNeuralNetwork_CrossValidation_iris()
{
    // allocate needed interfaces
    AbstractAuthority authority;

    datasets_t::datasets_t dataset = datasets_t::datasets_t::iris_dataset;
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(5);
    parameter_data.set_validation_holdout(.2);
    const int num_fold = 5;
    const double expected_accuracy = .75;

    // write dataset
    FindDataset finder(&authority);
    const std::string archive_filename = finder.find(dataset);

    // assess
    CrossValidationEstimator estimator(&authority);
    ClassificationAssessor assessor(&authority);
    estimator.initialize(archive_filename, false, num_fold);
    estimator.estimate_accuracy(&parameter_data, &assessor);
    const double actual_accuracy = assessor.get_classification_accuracy();

    return (actual_accuracy >= expected_accuracy);
}
PSL_TEST(FeedForwardNeuralNetwork, passCrossValidationIris)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(passed_FeedForwardNeuralNetwork_CrossValidation_iris, 1, 4, .49);
}

double accuracy_FeedForwardNeuralNetwork_advancedGenerator()
{
    // allocate needed interfaces
    AbstractAuthority authority;

    // sampler
    AdvancedSupervisedGenerator sampler(&authority);
    // tester
    SupervisedGeneratorTester tester(&authority);

    // parameters for training
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_batch_size(2);
    parameter_data.set_validation_holdout(.04);

    // test
    const int num_train = 512;
    const int num_test = 99;
    const double testing_accuracy = tester.get_accuracy(&sampler, num_train, &parameter_data, num_test);
    return testing_accuracy;
}
PSL_TEST(FeedForwardNeuralNetwork, advancedGeneratorAccuracy)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(accuracy_FeedForwardNeuralNetwork_advancedGenerator, 8, .88, .05);
}
bool passed_FeedForwardNeuralNetwork_advancedGenerator()
{
    // get accuracy
    const double actual_accuracy = accuracy_FeedForwardNeuralNetwork_advancedGenerator();

    const double expected_accuracy = .85;
    return (actual_accuracy >= expected_accuracy);
}
PSL_TEST(FeedForwardNeuralNetwork, advancedGeneratorPass)
{
    set_rand_seed();
    PSL_SerialOnlyTest
    stocastic_test(passed_FeedForwardNeuralNetwork_advancedGenerator, 4, 25, .5);
}

bool passed_FeedForwardNeuralNetwork_simple()
{
    bool passed = true;

    // allocate needed
    AbstractAuthority authority;

    // only classify on root
    if(!authority.mpi_wrapper->is_root())
    {
        return passed;
    }

    // write dataset
    FindDataset finder(&authority);
    const std::string archive_filename = finder.find(datasets_t::datasets_t::simple_dataset);

    // fill parameters
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_max_epochs(50);
    parameter_data.set_batch_size(4);
    parameter_data.set_archive_filename(archive_filename);
    parameter_data.set_validation_holdout(.0);

    // get from archive
    ClassificationArchive archive(&authority);
    archive.initialize(&parameter_data);

    // train network
    FeedForwardNeuralNetwork ffnn(&authority);
    ffnn.initialize(&parameter_data, &archive);
    ffnn.train(&parameter_data, &archive);

    // define testing problems
    const std::vector<std::vector<double> > testing_input_scalars = { {-0.2, 0.1}, {-0.2, -0.2}, {0.3, -0.3}, {0.3, 0.4}};
    const std::vector<std::vector<int> > testing_input_ints = { {}, {}, {}, {}};
    const std::vector<int> testing_output_ints = {0, 0, 1, 1};

    // test
    const size_t num_testing = testing_output_ints.size();
    for(size_t t = 0u; t < num_testing; t++)
    {
        const int actual = ffnn.classify(testing_input_scalars[t], testing_input_ints[t]);
        passed &= (testing_output_ints[t] == actual);
    }

    // clean-up archive file
    archive.get_io_manager()->delete_file();

    return passed;
}

PSL_TEST(FeedForwardNeuralNetwork, simple)
{
    set_rand_seed();
    stocastic_test(passed_FeedForwardNeuralNetwork_simple, 15, 40, .7);
}

class FeedForwardNeuralNetworkGradientCheck : public GradientCheck
{
public:
    FeedForwardNeuralNetworkGradientCheck(AbstractAuthority* authority,
                                          FeedForwardNeuralNetwork* ffnn,
                                          ClassificationArchive* archive) :
            GradientCheck(1e-3),
            m_authority(authority),
            m_ffnn(ffnn),
            m_archive(archive),
            m_step(NULL),
            m_network_input(),
            m_desired_network_output()
    {
    }
    virtual ~FeedForwardNeuralNetworkGradientCheck()
    {
        safe_free(m_step);
    }

    virtual void initialize()
    {
        // get current network
        MultiLayoutVector* network = m_ffnn->get_current_network();

        // set step
        m_step = new MultiLayoutVector(m_authority);
        m_step->zeros(network);
        m_step->random_double(-1e-9, 1e-9);

        // set network input/output
        AbstractInterface::DenseMatrix* training_matrix = m_archive->get_all_rows_onehot_encoded();
        const int num_train_data = training_matrix->get_num_rows();
        const int random_row = rand_int(0, num_train_data);
        std::vector<double> this_row;
        training_matrix->get_row(random_row, this_row);

        // get output enum size
        std::vector<int> input_enum_sizes;
        int output_enum_size = -1;
        m_archive->get_enum_size(input_enum_sizes, output_enum_size);

        // split encoding
        std::vector<double> this_row_input;
        std::vector<double> this_row_output;
        onehot_split(this_row, output_enum_size, m_network_input, m_desired_network_output);

        // cleanup memory
        delete training_matrix;
        delete network;
    }
    virtual double gradient_dot_step()
    {
        // get current network
        MultiLayoutVector* network = m_ffnn->get_current_network();

        // get bias and connectivities
        std::vector<std::vector<double>*> internal_layer_bias;
        std::vector<AbstractInterface::DenseMatrix**> internal_layer_connectivity;
        m_ffnn->unpack_network(network, internal_layer_bias, internal_layer_connectivity);

        // compute gradient
        MultiLayoutVector gradient(m_authority);
        gradient.zeros(network);
        m_ffnn->compute_gradient(internal_layer_bias,
                                 internal_layer_connectivity,
                                 m_network_input,
                                 m_desired_network_output,
                                 &gradient);

        // cleanup memory
        delete network;

        return gradient.dot(m_step);
    }
    virtual double objective_of_plus_step()
    {
        return objective_of_shifted_step(1.);
    }
    virtual double objective_of_minus_step()
    {
        return objective_of_shifted_step(-1.);
    }
    double objective_of_shifted_step(const double shift)
    {
        // get current network
        MultiLayoutVector* network = m_ffnn->get_current_network();

        // shift to step
        network->axpy(shift, m_step);

        // get bias and connectivities
        std::vector<std::vector<double>*> internal_layer_bias;
        std::vector<AbstractInterface::DenseMatrix**> internal_layer_connectivity;
        m_ffnn->unpack_network(network, internal_layer_bias, internal_layer_connectivity);

        // compute gradient
        MultiLayoutVector gradient(m_authority);
        gradient.zeros(network);
        const double result = m_ffnn->compute_gradient(internal_layer_bias,
                                                       internal_layer_connectivity,
                                                       m_network_input,
                                                       m_desired_network_output,
                                                       &gradient);

        // shift to back
        network->axpy(-1. * shift, m_step);

        // cleanup memory
        delete network;

        return result;
    }

protected:
    AbstractAuthority* m_authority;
    FeedForwardNeuralNetwork* m_ffnn;
    ClassificationArchive* m_archive;
    MultiLayoutVector* m_step;
    std::vector<double> m_network_input;
    std::vector<double> m_desired_network_output;

};

PSL_TEST(FeedForwardNeuralNetwork, gradientCheck)
{
    set_rand_seed();
    AbstractAuthority authority;

    // write dataset
    FindDataset finder(&authority);
    const std::string archive_filename = finder.find(datasets_t::datasets_t::simple_dataset);

    // fill parameters
    ParameterData parameter_data;
    parameter_data.defaults_for_feedForwardNeuralNetwork();
    parameter_data.set_archive_filename(archive_filename);

    // get from archive
    ClassificationArchive archive(&authority);
    archive.initialize(&parameter_data);

    // allocate and initialize network
    FeedForwardNeuralNetwork ffnn(&authority);
    ffnn.initialize(&parameter_data, &archive);

    // check gradient
    FeedForwardNeuralNetworkGradientCheck checker(&authority, &ffnn, &archive);
    EXPECT_EQ(true, checker.check_pass());
}

}
}
