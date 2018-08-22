#include "PSL_FeedForwardNeuralNetwork.hpp"

#include "PSL_Classifier.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_ActivationFunctionFactory.hpp"
#include "PSL_ActivationFunction.hpp"
#include "PSL_VectorCoding.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Random.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_DataLossFunction.hpp"
#include "PSL_DataLossFunctionFactory.hpp"
#include "PSL_MultiLayoutVector.hpp"
#include "PSL_ClassificationAssessor.hpp"
#include "PSL_Preprocessor.hpp"
#include "PSL_PreprocessorFactory.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <string>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <iostream>

namespace PlatoSubproblemLibrary
{

FeedForwardNeuralNetwork::FeedForwardNeuralNetwork(AbstractAuthority* authority) :
        Classifier(authority),
        m_num_internal_layers(NULL),
        m_internal_activation_function_enum(NULL),
        m_final_activation_function_enum(NULL),
        m_output_enum_size(NULL),
        m_internal_activation_function_auxiliary_parameter(NULL),
        m_final_activation_function_auxiliary_parameter(NULL),
        m_input_enum_sizes(NULL),
        m_internal_layer_bias(),
        m_internal_layer_connectivity(),
        m_bias_index_begin(-1),
        m_connectivity_index_begin(-1),
        m_internal_activation_function(NULL),
        m_final_activation_function(NULL),
        m_data_loss_function(NULL)
{
    m_data_sequence->set_data_flow_type(data_flow_t::data_flow_t::feed_forward_neural_network_flow);
}

FeedForwardNeuralNetwork::~FeedForwardNeuralNetwork()
{
    m_num_internal_layers = NULL;
    m_internal_activation_function_enum = NULL;
    m_final_activation_function_enum = NULL;
    m_output_enum_size = NULL;
    m_internal_activation_function_auxiliary_parameter = NULL;
    m_final_activation_function_auxiliary_parameter = NULL;
    m_input_enum_sizes = NULL;
    m_internal_layer_bias.clear();
    m_internal_layer_connectivity.clear();
    m_bias_index_begin = -1;
    m_connectivity_index_begin = -1;
    clear_built_data();
    safe_free(m_data_loss_function);
}

void FeedForwardNeuralNetwork::clear_built_data()
{
    safe_free(m_internal_activation_function);
    safe_free(m_final_activation_function);
    safe_free(m_preprocessor);
}

void FeedForwardNeuralNetwork::allocate_classifier_data(ParameterData* parameters, ClassificationArchive* archive)
{
    // allocate integers
    m_data_sequence->enqueue_int(parameters->get_num_internal_layers());
    m_data_sequence->enqueue_int(parameters->get_internal_activation_function());
    m_data_sequence->enqueue_int(parameters->get_final_activation_function());

    // allocate doubles
    m_data_sequence->enqueue_double(parameters->get_internal_activation_function_auxiliary_parameter());
    m_data_sequence->enqueue_double(parameters->get_final_activation_function_auxiliary_parameter());

    // allocate integer vectors
    std::vector<int>* preprocessors = m_data_sequence->get_int_vector(m_data_sequence->enqueue_int_vector());
    transfer_vector_types(*preprocessors, parameters->get_preprocessor());
    // get sizes
    std::vector<int>* input_enum_sizes = m_data_sequence->get_int_vector(m_data_sequence->enqueue_int_vector());
    int output_enum_size;
    archive->get_enum_size(*input_enum_sizes, output_enum_size);
    m_data_sequence->enqueue_int(output_enum_size);

    // allocate double vectors
    const int local_needed_double_vector = parameters->get_num_internal_layers();
    m_data_sequence->enqueue_double_vector(local_needed_double_vector);

    // allocate matrices
    const int local_needed_matrix = parameters->get_num_internal_layers();
    m_data_sequence->enqueue_matrix(local_needed_matrix);

    // build preprocessor
    allocate_preprocessor(parameters);
}

void FeedForwardNeuralNetwork::unpack_data()
{
    clear_built_data();

    // unpack integer scalars
    m_num_internal_layers = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_internal_activation_function_enum = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_final_activation_function_enum = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_output_enum_size = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());

    // unpack double scalars
    m_internal_activation_function_auxiliary_parameter = m_data_sequence->get_double_ptr(m_data_sequence->dequeue_double());
    m_final_activation_function_auxiliary_parameter = m_data_sequence->get_double_ptr(m_data_sequence->dequeue_double());

    // unpack integer vectors
    m_preprocessor_enum = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());
    m_input_enum_sizes = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());

    // unpack bias and connectivity
    m_bias_index_begin = m_data_sequence->dequeue_double_vector(*m_num_internal_layers);
    m_connectivity_index_begin = m_data_sequence->dequeue_matrix(*m_num_internal_layers);
    unpack_network(m_data_sequence, m_internal_layer_bias, m_internal_layer_connectivity);

    // build
    m_internal_activation_function =
            build_activation_function(activation_function_t::activation_function_t(*m_internal_activation_function_enum),
                                      *m_internal_activation_function_auxiliary_parameter,
                                      m_authority->utilities);
    m_final_activation_function =
            build_activation_function(activation_function_t::activation_function_t(*m_final_activation_function_enum),
                                      *m_final_activation_function_auxiliary_parameter,
                                      m_authority->utilities);

    // unpack preprocessor
    std::vector<preprocessor_t::preprocessor_t> enums;
    transfer_vector_types(enums, *m_preprocessor_enum);
    unpack_preprocessor(enums);
}

void FeedForwardNeuralNetwork::unpack_network(DataSequence* input,
                                              std::vector<std::vector<double>*>& internal_layer_bias,
                                              std::vector<AbstractInterface::DenseMatrix**>& internal_layer_connectivity)
{
    // unpack biases
    assert(0 <= m_bias_index_begin);
    assert(*m_num_internal_layers <= input->get_double_vectors_size());
    internal_layer_bias.resize(*m_num_internal_layers);
    for(int il = 0; il < *m_num_internal_layers; il++)
    {
        internal_layer_bias[il] = input->get_double_vector(m_bias_index_begin + il);
    }

    // unpack connectivities
    assert(0 <= m_connectivity_index_begin);
    assert(*m_num_internal_layers <= input->get_matrices_size());
    internal_layer_connectivity.resize(*m_num_internal_layers);
    for(int il = 0; il < *m_num_internal_layers; il++)
    {
        internal_layer_connectivity[il] = input->get_matrix_ptr(m_connectivity_index_begin + il);
    }
}

void FeedForwardNeuralNetwork::initialize(ParameterData* parameters, ClassificationArchive* archive)
{
    check_parameters(parameters);

    // allocate neural network
    allocate_classifier_data(parameters, archive);

    // associate pointers to underlying data
    unpack_data();

    // build preprocessor
    int processed_length = -1;
    m_preprocessor->begin_initialize(parameters, archive, processed_length);

    // initialize neural network
    AbstractInterface::DenseMatrix* training_matrix = archive->get_all_rows_onehot_encoded();
    // TODO: performance optimization possible; all rows not needed; just onehot dimension
    int input_enum_dimension = sum(*m_input_enum_sizes);
    int network_input_dimension = int(training_matrix->get_num_columns()) - *m_output_enum_size;
    int input_scalar_dimension = network_input_dimension - input_enum_dimension;
    int internal_layer_dimension = ceil(parameters->get_internal_layer_neurons_per_categorical_dimension()
                                        * double(input_enum_dimension + *m_output_enum_size)
                                        + parameters->get_internal_layer_neurons_per_numerical_dimension()
                                          * double(input_scalar_dimension));
    internal_layer_dimension = std::max(parameters->get_minimum_num_of_internal_layer_neurons(), internal_layer_dimension);
    initialize_network(parameters, processed_length, internal_layer_dimension);
    safe_free(training_matrix);

    // data loss function
    safe_free(m_data_loss_function);
    m_data_loss_function = build_data_loss_function(parameters->get_data_loss_function(), m_authority->utilities);
}

void FeedForwardNeuralNetwork::train(ParameterData* parameters, ClassificationArchive* archive)
{
    // if verbose, announce
    if(parameters->get_verbose())
    {
        m_authority->utilities->print("--- training new feed forward neural network ---\n");
    }

    // read data
    AbstractInterface::DenseMatrix* dataset = archive->get_all_rows_onehot_encoded();
    const int num_dataset_rows = dataset->get_num_rows();

    // split for validation
    std::vector<std::vector<int> > validation_then_training;
    random_division(num_dataset_rows, parameters->get_validation_holdout(), validation_then_training);
    const int num_training_row = validation_then_training[1].size();

    // assessor for validation
    ClassificationAssessor validation_assessor(m_authority);
    validation_assessor.initialize(parameters, *m_output_enum_size);

    // prepare to assess decrease in validation accuracies per epoch
    double prior_validation_accuracy = 0.;
    int epochs_of_validation_accuracy_decrease = 0;

    // set pointers to network data
    MultiLayoutVector* current_network = get_current_network();

    // allocate error gradients
    MultiLayoutVector batch_total_gradient(m_authority);
    batch_total_gradient.zeros(current_network);
    MultiLayoutVector one_row_gradient(m_authority);
    one_row_gradient.zeros(current_network);

    // for each epoch of training data
    const int max_num_epoch = parameters->get_max_epochs();
    for(int epoch = 0; epoch < max_num_epoch; epoch++)
    {
        // get batches
        std::vector<std::vector<int> > batches;
        get_random_batches(num_training_row, parameters->get_batch_size(), batches);

        // get learning rate
        const double learning_rate = parameters->get_initial_learning_rate() / double(epoch + 1);

        // for each batch
        const size_t num_batches = batches.size();
        for(size_t batch = 0u; batch < num_batches; batch++)
        {
            // zero fill gradient for this batch
            batch_total_gradient.scale(0.);

            // for each row in this batch
            const size_t num_in_batch = batches[batch].size();
            for(size_t in_batch_row = 0u; in_batch_row < num_in_batch; in_batch_row++)
            {
                // get row
                const int training_row = batches[batch][in_batch_row];
                const int absolute_row = validation_then_training[1][training_row];
                std::vector<double> this_row;
                dataset->get_row(absolute_row, this_row);

                // split encoding
                std::vector<double> this_row_input;
                std::vector<double> this_row_output;
                onehot_split(this_row, *m_output_enum_size, this_row_input, this_row_output);

                // TODO: could be done once for full dataset at start
                // pre-process
                m_preprocessor->preprocess(this_row_input);

                // assemble error gradient
                compute_gradient(m_internal_layer_bias,
                                 m_internal_layer_connectivity,
                                 this_row_input,
                                 this_row_output,
                                 &one_row_gradient);

                // add this row's contribution
                batch_total_gradient.axpy(1., &one_row_gradient);
            }

            // compute norm of this batch's gradient
            const double batch_gradient_norm = sqrt(batch_total_gradient.dot(&batch_total_gradient));

            // if gradient is not zero
            if(batch_gradient_norm != 0.)
            {
                // get update scale factor
                const double scale_factor = learning_rate / batch_gradient_norm;

                // update network
                current_network->axpy(-1. * scale_factor, &batch_total_gradient);
            }
        }

        // validate
        validation_assessor.assess(this, archive, validation_then_training[0]);
        const double validation_accuracy = validation_assessor.get_classification_accuracy();
        validation_assessor.clear();

        // if verbose, print validation accuracy
        if(parameters->get_verbose())
        {
            m_authority->utilities->print(std::string("epoch: ") + std::to_string(epoch) + std::string(", validation accuracy: ")
                                          + std::to_string(100. * validation_accuracy)
                                          + std::string("%\n"));
        }

        // check early stopping
        bool should_stop = false;

        // stop if perfect
        should_stop |= stop_for_perfection(parameters, validation_accuracy);

        // stop if validation accuracy has been decreasing
        should_stop |= stop_for_validation_accuracy_decrease_epochs(parameters,
                                                                    prior_validation_accuracy,
                                                                    validation_accuracy,
                                                                    epochs_of_validation_accuracy_decrease);

        // if found stopping criteria, stop
        if(should_stop)
        {
            break;
        }

        // advance for next epoch
        prior_validation_accuracy = validation_accuracy;
    }

    // finalize
    delete current_network;
    delete dataset;
}

void FeedForwardNeuralNetwork::initialize_network(ParameterData* parameters,
                                                  const int& input_dimension,
                                                  const int& internal_layer_dimension)
{
    assert(0 < input_dimension);
    assert(0 < internal_layer_dimension);

    const double rand_range_lower = -1e-3;
    const double rand_range_upper = 1e-3;

    for(int il = 0; il < *m_num_internal_layers; il++)
    {
        // set dimensions
        int this_layer_input_dimension = internal_layer_dimension;
        if(il == 0)
        {
            this_layer_input_dimension = input_dimension;
        }
        int this_layer_output_dimension = internal_layer_dimension;
        if(il + 1 == *m_num_internal_layers)
        {
            this_layer_output_dimension = *m_output_enum_size;
        }

        // fill connectivity randomly
        const int num_matrix_entries = this_layer_input_dimension * this_layer_output_dimension;
        std::vector<double> matrix_values(num_matrix_entries);
        uniform_rand_double(rand_range_lower, rand_range_upper, matrix_values);
        *m_internal_layer_connectivity[il] = m_authority->dense_builder->build_by_row_major(this_layer_output_dimension,
                                                                           this_layer_input_dimension,
                                                                           matrix_values);

        // fill bias randomly
        m_internal_layer_bias[il]->resize(this_layer_output_dimension);
        uniform_rand_double(rand_range_lower, rand_range_upper, *m_internal_layer_bias[il]);
    }
}

int FeedForwardNeuralNetwork::classify(const std::vector<double>& input_scalars, const std::vector<int>& input_enums)
{
    // encode
    std::vector<double> vector_a;
    onehot_encode(input_scalars, input_enums, *m_input_enum_sizes, vector_a);
    std::vector<double> vector_b;

    // pre-process
    m_preprocessor->preprocess(vector_a);

    // use pointers to allow efficient swapping
    std::vector<double>* input_vector = &vector_a;
    std::vector<double>* output_vector = &vector_b;

    // for each layer
    for(int il = 0; il < *m_num_internal_layers; il++)
    {
        const int this_layer_output_dimension = m_internal_layer_bias[il]->size();
        std::vector<double> preactivation_state(this_layer_output_dimension);
        std::vector<double> activation_gradient(this_layer_output_dimension);
        output_vector->resize(this_layer_output_dimension);

        // connectivity
        (*m_internal_layer_connectivity[il])->matvec(*input_vector, preactivation_state, false);

        // bias
        m_authority->dense_vector_operations->axpy(1., *m_internal_layer_bias[il], preactivation_state);

        // get activation function
        ActivationFunction* this_layer_activation_function = NULL;
        if(il + 1 < *m_num_internal_layers)
        {
            // internal layer
            this_layer_activation_function = m_internal_activation_function;
        }
        else
        {
            // final layer
            this_layer_activation_function = m_final_activation_function;
        }

        // activate
        this_layer_activation_function->activate(preactivation_state, *output_vector, activation_gradient);

        // output of this layer is input to next layer
        std::swap(input_vector, output_vector);
    }

    // get minimum difference from 1.
    int result = 0;
    double min_val = fabs(1. - (*input_vector)[0]);
    for(int o = 1; o < *m_output_enum_size; o++)
    {
        const double diff_val = fabs(1. - (*input_vector)[o]);
        if(diff_val < min_val)
        {
            min_val = diff_val;
            result = o;
        }
    }

    return result;
}

MultiLayoutVector* FeedForwardNeuralNetwork::get_current_network()
{
    MultiLayoutVector* current_network = new MultiLayoutVector(m_authority);

    // allocate
    current_network->resize_double_vectors(*m_num_internal_layers);
    current_network->resize_matrices(*m_num_internal_layers);

    // fill
    for(int il = 0; il < *m_num_internal_layers; il++)
    {
        current_network->set_matrix(il, *m_internal_layer_connectivity[il]);
        current_network->set_double_vector(il, m_internal_layer_bias[il]);
    }

    return current_network;
}

double FeedForwardNeuralNetwork::compute_gradient(const std::vector<std::vector<double>*>& internal_layer_bias,
                                                  const std::vector<AbstractInterface::DenseMatrix** >& internal_layer_connectivity,
                                                  const std::vector<double>& network_input,
                                                  const std::vector<double>& desired_network_output,
                                                  MultiLayoutVector* gradient_result)
{
    // count internal layers
    const int num_internal_layers = internal_layer_bias.size();
    assert(internal_layer_bias.size() == internal_layer_connectivity.size());

    // allocate
    std::vector<std::vector<double> > network_state(num_internal_layers);
    std::vector<std::vector<double> > network_partial(num_internal_layers);

    // forward propagation of state and partial
    const std::vector<double>* backward_state = &network_input;
    for(int il = 0; il < num_internal_layers; il++)
    {
        // compute pre-activation
        std::vector<double> preactivation_state;
        (*internal_layer_connectivity[il])->matvec(*backward_state, preactivation_state, false);
        m_authority->dense_vector_operations->axpy(1., *internal_layer_bias[il], preactivation_state);

        // get activation function
        ActivationFunction* this_layer_activation_function = NULL;
        if(il + 1 < num_internal_layers)
        {
            // internal layer
            this_layer_activation_function = m_internal_activation_function;
        }
        else
        {
            // final layer
            this_layer_activation_function = m_final_activation_function;
        }

        // activate
        this_layer_activation_function->activate(preactivation_state, network_state[il], network_partial[il]);

        // advance backward state
        backward_state = &network_state[il];
    }

    // apply data loss
    std::vector<double> forward_error_partial;
    const double data_loss_objective = m_data_loss_function->compute_loss(network_state.back(),
                                                                          desired_network_output,
                                                                          forward_error_partial);

    // backward propagation of gradients
    for(int il = num_internal_layers - 1; il >= 0; il--)
    {
        // set backward state
        if(il == 0)
        {
            backward_state = &network_input;
        }
        else
        {
            backward_state = &network_state[il - 1];
        }

        // compute gradient for bias
        std::vector<double>* bias_gradient = gradient_result->get_double_vector(il);
        m_authority->dense_vector_operations->multiply(network_partial[il], forward_error_partial, *bias_gradient);

        // compute gradient for connectivity
        gradient_result->transfer_matrix(il, m_authority->dense_builder->build_by_outerProduct(*bias_gradient, *backward_state));

        // update forward_error_partial
        (*internal_layer_connectivity[il])->matvec(*bias_gradient, forward_error_partial, true);
    }

    return data_loss_objective;
}

bool FeedForwardNeuralNetwork::stop_for_perfection(ParameterData* parameters, const double& validation_accuracy)
{
    return (parameters->get_perfection_accuracy() <= validation_accuracy);
}

bool FeedForwardNeuralNetwork::stop_for_validation_accuracy_decrease_epochs(ParameterData*parameters,
                                                                          const double& prior_accuracy,
                                                                          const double& current_accuracy,
                                                                          int& epochs_of_decrease)
{
    if(current_accuracy < prior_accuracy)
    {
        epochs_of_decrease++;
    }
    else
    {
        epochs_of_decrease = 0;
    }

    return (parameters->get_validation_decreasing_epochs() < epochs_of_decrease);
}

#define PSL_FeedForwardNeuralNetwork_ParameterWarning(name) \
        if(!parameters->didUserInput_##name()) \
        util->fatal_error("FeedForwardNeuralNetwork: missing parameter "#name"\n");
void FeedForwardNeuralNetwork::check_parameters(ParameterData* parameters)
{
    AbstractInterface::GlobalUtilities* util = m_authority->utilities;
    PSL_FeedForwardNeuralNetwork_ParameterWarning(classifier)
    PSL_FeedForwardNeuralNetwork_ParameterWarning(initial_learning_rate)
    if(parameters->get_initial_learning_rate() <= 0.)
    {
        util->fatal_error("FeedForwardNeuralNetwork: invalid initial_learning_rate parameter\n");
    }
    PSL_FeedForwardNeuralNetwork_ParameterWarning(max_epochs)
    if(parameters->get_max_epochs() < 1)
    {
        util->fatal_error("FeedForwardNeuralNetwork: invalid max_epochs parameter\n");
    }
    PSL_FeedForwardNeuralNetwork_ParameterWarning(num_internal_layers)
    if(parameters->get_num_internal_layers() < 1)
    {
        util->fatal_error("FeedForwardNeuralNetwork: invalid num_internal_layers parameter\n");
    }
    PSL_FeedForwardNeuralNetwork_ParameterWarning(internal_activation_function)
    PSL_FeedForwardNeuralNetwork_ParameterWarning(internal_activation_function_auxiliary_parameter)
    PSL_FeedForwardNeuralNetwork_ParameterWarning(final_activation_function)
    PSL_FeedForwardNeuralNetwork_ParameterWarning(final_activation_function_auxiliary_parameter)
    PSL_FeedForwardNeuralNetwork_ParameterWarning(data_loss_function)
    PSL_FeedForwardNeuralNetwork_ParameterWarning(validation_decreasing_epochs)
    PSL_FeedForwardNeuralNetwork_ParameterWarning(verbose)
    PSL_FeedForwardNeuralNetwork_ParameterWarning(perfection_accuracy)
    PSL_FeedForwardNeuralNetwork_ParameterWarning(preprocessor)
    PSL_FeedForwardNeuralNetwork_ParameterWarning(internal_layer_neurons_per_categorical_dimension)
    if(parameters->get_internal_layer_neurons_per_categorical_dimension() < 0.)
    {
        util->fatal_error("FeedForwardNeuralNetwork: invalid internal_layer_neurons_per_categorical_dimension parameter\n");
    }
    PSL_FeedForwardNeuralNetwork_ParameterWarning(internal_layer_neurons_per_numerical_dimension)
    if(parameters->get_internal_layer_neurons_per_numerical_dimension() < 0.)
    {
        util->fatal_error("FeedForwardNeuralNetwork: invalid internal_layer_neurons_per_numerical_dimension parameter\n");
    }
    PSL_FeedForwardNeuralNetwork_ParameterWarning(minimum_num_of_internal_layer_neurons)
}

}

