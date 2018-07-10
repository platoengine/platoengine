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
