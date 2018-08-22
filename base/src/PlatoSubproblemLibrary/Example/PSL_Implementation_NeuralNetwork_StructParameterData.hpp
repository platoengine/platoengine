#pragma once

#include "PSL_ParameterDataEnums.hpp"
#include <string>

namespace PlatoSubproblemLibrary
{
namespace example
{

#define PSL_NEURALNETWORK_STRUCTPARAMETERDATA(_type, _name) \
    bool has_##_name; _type _name;

struct NeuralNetwork_StructParameterData
{
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, batch_size)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, initial_learning_rate)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, max_epochs)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(std::string, archive_filename)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, num_internal_layers)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(activation_function_t::activation_function_t, internal_activation_function)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, internal_activation_function_auxiliary_parameter)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(activation_function_t::activation_function_t, final_activation_function)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, final_activation_function_auxiliary_parameter)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(data_loss_function_t::data_loss_function_t, data_loss_function)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, validation_holdout)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, validation_decreasing_epochs)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(bool, verbose)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, perfection_accuracy)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(preprocessor_t::preprocessor_t, preprocessor)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(data_flow_t::data_flow_t, classifier)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(vector_kernel_t::vector_kernel_t, vector_kernel)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, vector_kernel_parameter)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, empirical_risk)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(bool, archive_filetype_binary_preference)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, dataset_bagging_fraction)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, feature_subspace_power)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, feature_subspace_scale)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, requested_num_trees)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(decision_metric_t::decision_metric_t, decision_metric)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, scaled_stop_splitting_metric_value)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, minimum_leaf_size)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, minimum_features_considered)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, max_tree_depth_per_total_features)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, initial_stocastic_epochs)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, snapshot_separating_epochs)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, num_snapshots)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, snapshot_experience_rewind)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, internal_layer_neurons_per_categorical_dimension)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, internal_layer_neurons_per_numerical_dimension)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, minimum_num_of_internal_layer_neurons)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(double, PCA_upper_variation_fraction)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, num_neighbors)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(bool, should_select_nearest_neighbors_by_validation)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(distance_voter_t::distance_voter_t, distance_voter)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, random_projection_forest_num_trees)
    PSL_NEURALNETWORK_STRUCTPARAMETERDATA(int, random_projection_forest_approx_leaf_size)
};

}
}
