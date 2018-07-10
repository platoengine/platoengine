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

#include "PSL_Implementation_Kernel_StructParameterData.hpp"
#include "PSL_Interface_Kernel_StructParameterDataBuilder.hpp"
#include "PSL_Implementation_NeuralNetwork_StructParameterData.hpp"
#include "PSL_Interface_NeuralNetwork_StructParameterDataBuilder.hpp"
#include "PSL_Abstract_ParameterDataBuilder.hpp"
#include "PSL_Abstract_ParameterData.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Random.hpp"

namespace PlatoSubproblemLibrary
{
namespace ParameterDataTest
{

void kernel_compare_input_datas(ParameterData* data1, ParameterData* data2);
void neural_network_compare_parameter_datas(ParameterData* data1, ParameterData* data2);

PSL_TEST(ParameterData,kernel_basic)
{
    set_rand_seed();
    // allocate ideal kernel data
    ParameterData kernel_input_data_gold;
    kernel_input_data_gold.set_scale(1.9);

    // allocate struct
    example::Kernel_StructParameterData input_data_struct;
    input_data_struct.has_scale = true;
    input_data_struct.scale = kernel_input_data_gold.get_scale();
    input_data_struct.has_absolute = false;
    input_data_struct.has_iterations = false;
    input_data_struct.has_symPlane_X = false;
    input_data_struct.has_symPlane_Y = false;
    input_data_struct.has_symPlane_Z = false;
    input_data_struct.has_penalty = false;
    input_data_struct.has_node_resolution_tolerance = false;
    input_data_struct.has_spatial_searcher = false;
    input_data_struct.has_normalization = false;
    input_data_struct.has_reproduction = false;
    input_data_struct.has_matrix_assembly_agent = false;
    input_data_struct.has_symmetry_plane_agent = false;
    input_data_struct.has_mesh_scale_agent = false;
    input_data_struct.has_point_ghosting_agent = false;
    input_data_struct.has_bounded_support_function = false;

    // build as input data
    example::Interface_Kernel_StructParameterDataBuilder struct_builder;
    struct_builder.set_data(&input_data_struct);
    ParameterData* kernel_input_data_from_struct = struct_builder.build();

    // check data from struct
    kernel_compare_input_datas(kernel_input_data_from_struct, &kernel_input_data_gold);
    delete kernel_input_data_from_struct;
}

#define PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(_class,_struct,_field) \
    _struct.has_##_field = true; \
    _struct._field = _class.get_##_field();

PSL_TEST(ParameterData,kernel_detailed)
{
    set_rand_seed();
    // allocate ideal kernel data
    ParameterData kernel_data_gold;
    kernel_data_gold.set_absolute(4.2);
    kernel_data_gold.set_iterations(42);
    kernel_data_gold.set_symPlane_Y(13.1);
    kernel_data_gold.set_penalty(2.7);
    kernel_data_gold.set_node_resolution_tolerance(1e-4);
    kernel_data_gold.set_spatial_searcher(spatial_searcher_t::bounding_box_brute_force);
    kernel_data_gold.set_normalization(normalization_t::classical_row_normalization);
    kernel_data_gold.set_reproduction(reproduction_level_t::reproduce_constant);
    kernel_data_gold.set_matrix_assembly_agent(matrix_assembly_agent_t::by_row);
    kernel_data_gold.set_symmetry_plane_agent(symmetry_plane_agent_t::by_narrow_clone);
    kernel_data_gold.set_mesh_scale_agent(mesh_scale_agent_t::by_average_optimized_element_side);
    kernel_data_gold.set_point_ghosting_agent(point_ghosting_agent_t::by_narrow_share);
    kernel_data_gold.set_bounded_support_function(bounded_support_function_t::polynomial_tent_function);

    // allocate struct
    example::Kernel_StructParameterData data_struct;
    data_struct.has_scale = false;
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,absolute)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,iterations)
    data_struct.has_symPlane_X = false;
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,symPlane_Y)
    data_struct.has_symPlane_Z = false;
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,penalty)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,node_resolution_tolerance)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,spatial_searcher)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,normalization)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,reproduction)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,matrix_assembly_agent)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,symmetry_plane_agent)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,mesh_scale_agent)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,point_ghosting_agent)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(kernel_data_gold,data_struct,bounded_support_function)

    // build as input data
    example::Interface_Kernel_StructParameterDataBuilder struct_builder;
    struct_builder.set_data(&data_struct);
    ParameterData* kernel_data_from_struct = struct_builder.build();

    // check data from struct
    kernel_compare_input_datas(kernel_data_from_struct, &kernel_data_gold);
    delete kernel_data_from_struct;
}

PSL_TEST(ParameterData,neural_network_basic)
{
    set_rand_seed();
    // allocate ideal data
    ParameterData parameter_data_gold;
    parameter_data_gold.set_batch_size(42);

    // allocate struct
    example::NeuralNetwork_StructParameterData parameter_data_struct;
    parameter_data_struct.has_batch_size = true;
    parameter_data_struct.batch_size = parameter_data_gold.get_batch_size();
    parameter_data_struct.has_initial_learning_rate = false;
    parameter_data_struct.has_max_epochs = false;
    parameter_data_struct.has_archive_filename = false;
    parameter_data_struct.has_num_internal_layers = false;
    parameter_data_struct.has_internal_activation_function = false;
    parameter_data_struct.has_internal_activation_function_auxiliary_parameter = false;
    parameter_data_struct.has_final_activation_function = false;
    parameter_data_struct.has_final_activation_function_auxiliary_parameter = false;
    parameter_data_struct.has_data_loss_function = false;
    parameter_data_struct.has_validation_holdout = false;
    parameter_data_struct.has_validation_decreasing_epochs = false;
    parameter_data_struct.has_verbose = false;
    parameter_data_struct.has_perfection_accuracy = false;
    parameter_data_struct.has_preprocessor = false;
    parameter_data_struct.has_classifier = false;
    parameter_data_struct.has_vector_kernel = false;
    parameter_data_struct.has_vector_kernel_parameter = false;
    parameter_data_struct.has_empirical_risk = false;
    parameter_data_struct.has_archive_filetype_binary_preference = false;
    parameter_data_struct.has_dataset_bagging_fraction = false;
    parameter_data_struct.has_feature_subspace_power = false;
    parameter_data_struct.has_feature_subspace_scale = false;
    parameter_data_struct.has_requested_num_trees = false;
    parameter_data_struct.has_decision_metric = false;
    parameter_data_struct.has_scaled_stop_splitting_metric_value = false;
    parameter_data_struct.has_minimum_leaf_size = false;
    parameter_data_struct.has_minimum_features_considered = false;
    parameter_data_struct.has_max_tree_depth_per_total_features = false;
    parameter_data_struct.has_initial_stocastic_epochs = false;
    parameter_data_struct.has_snapshot_separating_epochs = false;
    parameter_data_struct.has_num_snapshots = false;
    parameter_data_struct.has_snapshot_experience_rewind = false;
    parameter_data_struct.has_internal_layer_neurons_per_categorical_dimension = false;
    parameter_data_struct.has_internal_layer_neurons_per_numerical_dimension = false;
    parameter_data_struct.has_minimum_num_of_internal_layer_neurons = false;
    parameter_data_struct.has_PCA_upper_variation_fraction = false;
    parameter_data_struct.has_num_neighbors = false;
    parameter_data_struct.has_should_select_nearest_neighbors_by_validation = false;
    parameter_data_struct.has_distance_voter = false;
    parameter_data_struct.has_random_projection_forest_num_trees = false;
    parameter_data_struct.has_random_projection_forest_approx_leaf_size = false;

    // build as parameter data
    example::Interface_NeuralNetwork_StructParameterDataBuilder struct_builder;
    struct_builder.set_data(&parameter_data_struct);
    ParameterData* parameter_data_from_struct = struct_builder.build();

    // check data from struct
    neural_network_compare_parameter_datas(parameter_data_from_struct, &parameter_data_gold);
    delete parameter_data_from_struct;
}

PSL_TEST(ParameterData,neural_network_detailed)
{
    set_rand_seed();
    // allocate ideal kernel data
    ParameterData parameter_data_gold;
    parameter_data_gold.set_batch_size(44);
    parameter_data_gold.set_initial_learning_rate(1.85);
    parameter_data_gold.set_max_epochs(257);
    parameter_data_gold.set_archive_filename("_example.psl");
    parameter_data_gold.set_num_internal_layers(3);
    parameter_data_gold.set_internal_activation_function(activation_function_t::activation_function_t::soft_plus_activation);
    parameter_data_gold.set_internal_activation_function_auxiliary_parameter(-1.2);
    parameter_data_gold.set_final_activation_function(activation_function_t::activation_function_t::rectified_linear_unit);
    parameter_data_gold.set_final_activation_function_auxiliary_parameter(2.13);
    parameter_data_gold.set_data_loss_function(data_loss_function_t::data_loss_function_t::squared_error_loss);
    parameter_data_gold.set_validation_holdout(.57);
    parameter_data_gold.set_validation_decreasing_epochs(3);
    parameter_data_gold.set_verbose(true);
    parameter_data_gold.set_perfection_accuracy(0.91);
    parameter_data_gold.set_preprocessor(preprocessor_t::preprocessor_t::standardization_preprocessor);
    parameter_data_gold.set_classifier(data_flow_t::data_flow_t::support_vector_machine_flow);
    parameter_data_gold.set_vector_kernel(vector_kernel_t::vector_kernel_t::mixed_kernel);
    parameter_data_gold.set_vector_kernel_parameter(3);
    parameter_data_gold.set_empirical_risk(.0023);
    parameter_data_gold.set_archive_filetype_binary_preference(true);
    parameter_data_gold.set_dataset_bagging_fraction(.139);
    parameter_data_gold.set_feature_subspace_power(.431);
    parameter_data_gold.set_feature_subspace_scale(.125);
    parameter_data_gold.set_requested_num_trees(37);
    parameter_data_gold.set_decision_metric(decision_metric_t::decision_metric_t::entropy_metric);
    parameter_data_gold.set_scaled_stop_splitting_metric_value(1.253);
    parameter_data_gold.set_minimum_leaf_size(11);
    parameter_data_gold.set_minimum_features_considered(14);
    parameter_data_gold.set_max_tree_depth_per_total_features(1.337);
    parameter_data_gold.set_initial_stocastic_epochs(138);
    parameter_data_gold.set_snapshot_separating_epochs(37);
    parameter_data_gold.set_num_snapshots(17);
    parameter_data_gold.set_snapshot_experience_rewind(.253);
    parameter_data_gold.set_internal_layer_neurons_per_categorical_dimension(15.2);
    parameter_data_gold.set_internal_layer_neurons_per_numerical_dimension(42.25);
    parameter_data_gold.set_minimum_num_of_internal_layer_neurons(42);
    parameter_data_gold.set_PCA_upper_variation_fraction(.8);
    parameter_data_gold.set_num_neighbors(8);
    parameter_data_gold.set_should_select_nearest_neighbors_by_validation(true);
    parameter_data_gold.set_distance_voter(distance_voter_t::distance_voter_t::inverse_distance_voter);
    parameter_data_gold.set_random_projection_forest_num_trees(3);
    parameter_data_gold.set_random_projection_forest_approx_leaf_size(5);

    // allocate struct
    example::NeuralNetwork_StructParameterData parameter_data_struct;
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,batch_size)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,initial_learning_rate)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,max_epochs)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,archive_filename)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,num_internal_layers)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,internal_activation_function)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,internal_activation_function_auxiliary_parameter)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,final_activation_function)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,final_activation_function_auxiliary_parameter)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,data_loss_function)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,validation_holdout)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,validation_decreasing_epochs)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,verbose)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,perfection_accuracy)
    parameter_data_struct.has_preprocessor = true;
    parameter_data_struct.preprocessor = parameter_data_gold.get_preprocessor()[0];
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,classifier)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,vector_kernel)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,vector_kernel_parameter)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,empirical_risk)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,archive_filetype_binary_preference)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,dataset_bagging_fraction)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,feature_subspace_power)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,feature_subspace_scale)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,requested_num_trees)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,decision_metric)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,scaled_stop_splitting_metric_value)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,minimum_leaf_size)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,minimum_features_considered)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,max_tree_depth_per_total_features)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,initial_stocastic_epochs)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,snapshot_separating_epochs)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,num_snapshots)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,snapshot_experience_rewind)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,internal_layer_neurons_per_categorical_dimension)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,internal_layer_neurons_per_numerical_dimension)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,minimum_num_of_internal_layer_neurons)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,PCA_upper_variation_fraction)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,num_neighbors)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,should_select_nearest_neighbors_by_validation)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,distance_voter)
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,random_projection_forest_num_trees);
    PSL_UNITTEST_PARAMETERDATA_TOSTRUCT(parameter_data_gold,parameter_data_struct,random_projection_forest_approx_leaf_size);

    // build as parameter data
    example::Interface_NeuralNetwork_StructParameterDataBuilder struct_builder;
    struct_builder.set_data(&parameter_data_struct);
    ParameterData* parameter_data_from_struct = struct_builder.build();

    // check data from struct
    neural_network_compare_parameter_datas(parameter_data_from_struct, &parameter_data_gold);
    delete parameter_data_from_struct;
}

#define PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(_d1, _d2, _field) \
    EXPECT_EQ(_d1->didUserInput_##_field(), _d2->didUserInput_##_field()); \
    if(_d1->didUserInput_##_field() && _d2->didUserInput_##_field()) { \
        EXPECT_EQ(_d1->get_##_field(), _d2->get_##_field()); \
    }
#define PSL_UNIT_TEST_INPUT_DATA_COMPARE_VECTOR(_d1, _d2, _field) \
    EXPECT_EQ(_d1->didUserInput_##_field(), _d2->didUserInput_##_field()); \
    if(_d1->didUserInput_##_field() && _d2->didUserInput_##_field()) { \
        expect_equal_vectors(_d1->get_##_field(), _d2->get_##_field()); \
    }

void kernel_compare_input_datas(ParameterData* data1, ParameterData* data2)
{
    // both should be not NULL
    if (!data1 || !data2)
    {
        ASSERT_EQ(true,false);
    }

    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,scale)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,absolute)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,iterations)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,symPlane_X)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,symPlane_Y)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,symPlane_Z)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,penalty)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,node_resolution_tolerance)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,spatial_searcher)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,normalization)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,reproduction)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,matrix_assembly_agent)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,symmetry_plane_agent)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,mesh_scale_agent)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,point_ghosting_agent)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,bounded_support_function)
}

void neural_network_compare_parameter_datas(ParameterData* data1, ParameterData* data2)
{
    // both should be not NULL
    if (!data1 || !data2)
    {
        ASSERT_EQ(true,false);
    }

    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,batch_size)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,initial_learning_rate)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,max_epochs)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,archive_filename)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,num_internal_layers)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,internal_activation_function)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,internal_activation_function_auxiliary_parameter)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,final_activation_function)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,final_activation_function_auxiliary_parameter)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,data_loss_function)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,validation_holdout)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,validation_decreasing_epochs)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,verbose)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,perfection_accuracy)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_VECTOR(data1,data2,preprocessor)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,classifier)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,vector_kernel)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,vector_kernel_parameter)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,empirical_risk)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,archive_filetype_binary_preference)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,dataset_bagging_fraction)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,feature_subspace_power)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,feature_subspace_scale)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,requested_num_trees)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,decision_metric)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,scaled_stop_splitting_metric_value)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,minimum_leaf_size)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,minimum_features_considered)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,max_tree_depth_per_total_features)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,initial_stocastic_epochs)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,snapshot_separating_epochs)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,num_snapshots)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,snapshot_experience_rewind)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,internal_layer_neurons_per_categorical_dimension)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,internal_layer_neurons_per_numerical_dimension)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,minimum_num_of_internal_layer_neurons)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,PCA_upper_variation_fraction)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,num_neighbors)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,should_select_nearest_neighbors_by_validation)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,distance_voter)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,random_projection_forest_num_trees)
    PSL_UNIT_TEST_INPUT_DATA_COMPARE_POD(data1,data2,random_projection_forest_approx_leaf_size)
}

}

}

