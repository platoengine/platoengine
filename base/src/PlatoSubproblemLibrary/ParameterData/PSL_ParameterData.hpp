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

// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_ParameterData.hpp"
#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{

class ParameterData : public Abstract_ParameterData
{
public:
    ParameterData();
    virtual ~ParameterData();

    PSL_PARAMETER_DATA_POD(tokens_t, double, scale)
    PSL_PARAMETER_DATA_POD(tokens_t, double, absolute)
    PSL_PARAMETER_DATA_POD(tokens_t, int, iterations)
    PSL_PARAMETER_DATA_POD(tokens_t, double, symPlane_X)
    PSL_PARAMETER_DATA_POD(tokens_t, double, symPlane_Y)
    PSL_PARAMETER_DATA_POD(tokens_t, double, symPlane_Z)
    PSL_PARAMETER_DATA_POD(tokens_t, double, penalty)
    PSL_PARAMETER_DATA_POD(tokens_t, double, node_resolution_tolerance)
    PSL_PARAMETER_DATA_ENUM(tokens_t, spatial_searcher_t, spatial_searcher)
    PSL_PARAMETER_DATA_ENUM(tokens_t, normalization_t, normalization)
    PSL_PARAMETER_DATA_ENUM(tokens_t, reproduction_level_t, reproduction)
    PSL_PARAMETER_DATA_ENUM(tokens_t, symmetry_plane_agent_t, symmetry_plane_agent)
    PSL_PARAMETER_DATA_ENUM(tokens_t, matrix_assembly_agent_t, matrix_assembly_agent)
    PSL_PARAMETER_DATA_ENUM(tokens_t, mesh_scale_agent_t, mesh_scale_agent)
    PSL_PARAMETER_DATA_ENUM(tokens_t, matrix_normalization_agent_t, matrix_normalization_agent)
    PSL_PARAMETER_DATA_ENUM(tokens_t, point_ghosting_agent_t, point_ghosting_agent)
    PSL_PARAMETER_DATA_ENUM(tokens_t, bounded_support_function_t, bounded_support_function)
    PSL_PARAMETER_DATA_POD(tokens_t, int, batch_size)
    PSL_PARAMETER_DATA_POD(tokens_t, double, initial_learning_rate)
    PSL_PARAMETER_DATA_POD(tokens_t, int, max_epochs)
    PSL_PARAMETER_DATA_POD_LONG(tokens_t, string, std::string, archive_filename)
    PSL_PARAMETER_DATA_POD(tokens_t, int, num_internal_layers)
    PSL_PARAMETER_DATA_ENUM(tokens_t, activation_function_t, internal_activation_function)
    PSL_PARAMETER_DATA_POD(tokens_t, double, internal_activation_function_auxiliary_parameter)
    PSL_PARAMETER_DATA_ENUM(tokens_t, activation_function_t, final_activation_function)
    PSL_PARAMETER_DATA_POD(tokens_t, double, final_activation_function_auxiliary_parameter)
    PSL_PARAMETER_DATA_ENUM(tokens_t, data_loss_function_t, data_loss_function)
    PSL_PARAMETER_DATA_POD(tokens_t, double, validation_holdout)
    PSL_PARAMETER_DATA_POD(tokens_t, int, validation_decreasing_epochs)
    PSL_PARAMETER_DATA_POD(tokens_t, bool, verbose)
    PSL_PARAMETER_DATA_POD(tokens_t, double, perfection_accuracy)
    PSL_PARAMETER_DATA_VECTOR_ENUM(tokens_t, preprocessor_t, preprocessor)
    PSL_PARAMETER_DATA_ENUM(tokens_t, data_flow_t, classifier)
    PSL_PARAMETER_DATA_ENUM(tokens_t, vector_kernel_t, vector_kernel)
    PSL_PARAMETER_DATA_POD(tokens_t, int, vector_kernel_parameter)
    PSL_PARAMETER_DATA_POD(tokens_t, double, empirical_risk)
    PSL_PARAMETER_DATA_POD(tokens_t, bool, archive_filetype_binary_preference)
    PSL_PARAMETER_DATA_POD(tokens_t, double, dataset_bagging_fraction)
    PSL_PARAMETER_DATA_POD(tokens_t, double, feature_subspace_power)
    PSL_PARAMETER_DATA_POD(tokens_t, double, feature_subspace_scale)
    PSL_PARAMETER_DATA_POD(tokens_t, int, requested_num_trees)
    PSL_PARAMETER_DATA_ENUM(tokens_t, decision_metric_t, decision_metric)
    PSL_PARAMETER_DATA_POD(tokens_t, double, scaled_stop_splitting_metric_value)
    PSL_PARAMETER_DATA_POD(tokens_t, int, minimum_leaf_size)
    PSL_PARAMETER_DATA_POD(tokens_t, int, minimum_features_considered)
    PSL_PARAMETER_DATA_POD(tokens_t, double, max_tree_depth_per_total_features)
    PSL_PARAMETER_DATA_POD(tokens_t, int, initial_stocastic_epochs)
    PSL_PARAMETER_DATA_POD(tokens_t, int, snapshot_separating_epochs)
    PSL_PARAMETER_DATA_POD(tokens_t, int, num_snapshots)
    PSL_PARAMETER_DATA_POD(tokens_t, double, snapshot_experience_rewind)
    PSL_PARAMETER_DATA_POD(tokens_t, double, internal_layer_neurons_per_categorical_dimension)
    PSL_PARAMETER_DATA_POD(tokens_t, double, internal_layer_neurons_per_numerical_dimension)
    PSL_PARAMETER_DATA_POD(tokens_t, int, minimum_num_of_internal_layer_neurons)
    PSL_PARAMETER_DATA_POD(tokens_t, double, PCA_upper_variation_fraction)
    PSL_PARAMETER_DATA_POD(tokens_t, int, num_neighbors)
    PSL_PARAMETER_DATA_POD(tokens_t, bool, should_select_nearest_neighbors_by_validation)
    PSL_PARAMETER_DATA_ENUM(tokens_t, distance_voter_t, distance_voter)
    PSL_PARAMETER_DATA_POD(tokens_t, int, random_projection_forest_num_trees)
    PSL_PARAMETER_DATA_POD(tokens_t, int, random_projection_forest_approx_leaf_size)
    PSL_PARAMETER_DATA_POD(tokens_t, double, min_heaviside_parameter)
    PSL_PARAMETER_DATA_POD(tokens_t, double, heaviside_continuation_scale)
    PSL_PARAMETER_DATA_POD(tokens_t, double, max_heaviside_parameter)
    PSL_PARAMETER_DATA_POD(tokens_t, double, max_overhang_angle)
    PSL_PARAMETER_DATA_POD(tokens_t, double, overhang_projection_angle_fraction)
    PSL_PARAMETER_DATA_POD(tokens_t, double, overhang_projection_radius_fraction)
    PSL_PARAMETER_DATA_POD(tokens_t, double, build_direction_x)
    PSL_PARAMETER_DATA_POD(tokens_t, double, build_direction_y)
    PSL_PARAMETER_DATA_POD(tokens_t, double, build_direction_z)

    void defaults_for_classification();
    void defaults_for_feedForwardNeuralNetwork();
    void defaults_for_supportVectorMachine();
    void defaults_for_randomForest();
    void defaults_for_nearestNeighborClassifier();

private:

};

}
