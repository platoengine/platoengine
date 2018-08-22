// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_ParameterData.hpp"

#include <iostream>
#include <string>
#include <string.h>
#include <vector>

#include "PSL_Abstract_ParameterData.hpp"

namespace PlatoSubproblemLibrary
{

ParameterData::ParameterData() :
        Abstract_ParameterData()
{
}

ParameterData::~ParameterData()
{
}

void ParameterData::defaults_for_classification()
{
    defaults_for_feedForwardNeuralNetwork();
}
void ParameterData::defaults_for_feedForwardNeuralNetwork()
{
    set_classifier(data_flow_t::data_flow_t::feed_forward_neural_network_flow);
    set_initial_learning_rate(1.6);
    set_max_epochs(10);
    set_num_internal_layers(2);
    set_internal_activation_function(activation_function_t::activation_function_t::sigmoid_activation);
    set_internal_activation_function_auxiliary_parameter(1e-2);
    set_final_activation_function(activation_function_t::activation_function_t::sigmoid_activation);
    set_final_activation_function_auxiliary_parameter(-1.);
    set_data_loss_function(data_loss_function_t::data_loss_function_t::squared_error_loss);
    set_validation_decreasing_epochs(2);
    set_verbose(false);
    set_perfection_accuracy(.999);
    set_preprocessor(preprocessor_t::preprocessor_t::skew_normalization_preprocessor);
    set_archive_filetype_binary_preference(true);
    set_internal_layer_neurons_per_categorical_dimension(.6);
    set_internal_layer_neurons_per_numerical_dimension(.5);
    set_minimum_num_of_internal_layer_neurons(5);
    set_batch_size(12);
    set_validation_holdout(.12);
    set_PCA_upper_variation_fraction(.96);
}
void ParameterData::defaults_for_supportVectorMachine()
{
    set_classifier(data_flow_t::data_flow_t::support_vector_machine_flow);
    set_vector_kernel(vector_kernel_t::vector_kernel_t::pure_kernel);
    set_vector_kernel_parameter(2);
    set_verbose(false);
    std::vector<preprocessor_t::preprocessor_t> preprocessor_types = {
            preprocessor_t::preprocessor_t::skew_normalization_preprocessor,
            preprocessor_t::preprocessor_t::bipolar_normalization_preprocessor};
    set_preprocessor(preprocessor_types);
    set_empirical_risk(.2);
    set_archive_filetype_binary_preference(true);
    set_initial_stocastic_epochs(0);
    set_snapshot_separating_epochs(0);
    set_num_snapshots(10);
    set_snapshot_experience_rewind(.05);
    set_PCA_upper_variation_fraction(.96);
}
void ParameterData::defaults_for_randomForest()
{
    set_verbose(false);
    set_archive_filetype_binary_preference(true);
    set_classifier(data_flow_t::data_flow_t::random_forest_flow);
    set_dataset_bagging_fraction(.9);
    set_feature_subspace_power(.75);
    set_feature_subspace_scale(.65);
    set_requested_num_trees(15);
    set_decision_metric(decision_metric_t::decision_metric_t::entropy_metric);
    set_scaled_stop_splitting_metric_value(.05);
    set_minimum_leaf_size(3);
    set_minimum_features_considered(4);
    set_max_tree_depth_per_total_features(2.);
}
void ParameterData::defaults_for_nearestNeighborClassifier()
{
    set_classifier(data_flow_t::data_flow_t::nearest_neighbors_classifier_flow);
    set_verbose(false);
    set_preprocessor(preprocessor_t::preprocessor_t::standardization_preprocessor);
    set_num_neighbors(14);
    set_should_select_nearest_neighbors_by_validation(true);
    set_distance_voter(distance_voter_t::distance_voter_t::counting_voter);
}
}
