#pragma once

#include "PSL_Classifier.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class DenseMatrix;
}
class AbstractAuthority;
class ParameterData;
class ClassificationArchive;
class ActivationFunction;
class DataLossFunction;
class MultiLayoutVector;
class Preprocessor;

class FeedForwardNeuralNetwork : public Classifier
{
public:
    FeedForwardNeuralNetwork(AbstractAuthority* authority);
    virtual ~FeedForwardNeuralNetwork();

    virtual void unpack_data();

    virtual void initialize(ParameterData* parameters, ClassificationArchive* archive);
    virtual void train(ParameterData* parameters, ClassificationArchive* archive);
    virtual int classify(const std::vector<double>& input_scalars, const std::vector<int>& input_enums);

    double compute_gradient(const std::vector<std::vector<double>*>& internal_layer_bias,
                            const std::vector<AbstractInterface::DenseMatrix**>& internal_layer_connectivity,
                            const std::vector<double>& network_input,
                            const std::vector<double>& desired_network_output,
                            MultiLayoutVector* gradient_result);
    MultiLayoutVector* get_current_network();
    void unpack_network(DataSequence* input,
                        std::vector<std::vector<double>*>& internal_layer_bias,
                        std::vector<AbstractInterface::DenseMatrix**>& internal_layer_connectivity);

    void resize_biases(const int& size);
    void set_bias(const int& index, std::vector<double>* bias);
    void resize_connectivities(const int& size);
    void set_connectivities(const int& index, AbstractInterface::DenseMatrix** connectivity);

protected:

    void clear_built_data();
    void allocate_classifier_data(ParameterData* parameters, ClassificationArchive* archive);
    void initialize_network(ParameterData* parameters, const int& input_dimension, const int& internal_layer_dimension);

    // early stopping criteria
    bool stop_for_perfection(ParameterData*parameters, const double& validation_accuracy);
    bool stop_for_validation_accuracy_decrease_epochs(ParameterData*parameters,
                                                      const double& prior_accuracy,
                                                      const double& current_accuracy,
                                                      int& epochs_of_decrease);
    void check_parameters(ParameterData* parameters);

    // DataSequence data
    int* m_num_internal_layers;
    int* m_internal_activation_function_enum;
    int* m_final_activation_function_enum;
    int* m_output_enum_size;
    double* m_internal_activation_function_auxiliary_parameter;
    double* m_final_activation_function_auxiliary_parameter;
    std::vector<int>* m_input_enum_sizes;
    std::vector<std::vector<double>* > m_internal_layer_bias;
    std::vector<AbstractInterface::DenseMatrix** > m_internal_layer_connectivity;
    int m_bias_index_begin;
    int m_connectivity_index_begin;

    // built from DataSequence
    ActivationFunction* m_internal_activation_function;
    ActivationFunction* m_final_activation_function;

    // training only data
    DataLossFunction* m_data_loss_function;

};

}
