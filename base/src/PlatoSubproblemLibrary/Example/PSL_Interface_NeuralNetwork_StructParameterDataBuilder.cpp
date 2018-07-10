#include "PSL_Interface_NeuralNetwork_StructParameterDataBuilder.hpp"

#include "PSL_Implementation_NeuralNetwork_StructParameterData.hpp"
#include "PSL_Abstract_ParameterDataBuilder.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ParameterDataEnums.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_NeuralNetwork_StructParameterDataBuilder::Interface_NeuralNetwork_StructParameterDataBuilder() :
        AbstractInterface::ParameterDataBuilder(),
        m_data(NULL)
{
}

Interface_NeuralNetwork_StructParameterDataBuilder::~Interface_NeuralNetwork_StructParameterDataBuilder()
{
    m_data = NULL;
}

void Interface_NeuralNetwork_StructParameterDataBuilder::set_data(example::NeuralNetwork_StructParameterData* data)
{
    m_data = data;
}

ParameterData* Interface_NeuralNetwork_StructParameterDataBuilder::build()
{
    if (m_data == NULL) {
        return NULL;
    }

    // allocate
    ParameterData* result = new ParameterData;

    // fill
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result, batch_size)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result, initial_learning_rate)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result, max_epochs)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result, archive_filename)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,num_internal_layers)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,internal_activation_function)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,internal_activation_function_auxiliary_parameter)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,final_activation_function)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,final_activation_function_auxiliary_parameter)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,data_loss_function)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,validation_holdout)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,validation_decreasing_epochs)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,verbose)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,perfection_accuracy)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,preprocessor)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,classifier)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,vector_kernel)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,vector_kernel_parameter)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,empirical_risk)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,archive_filetype_binary_preference)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,dataset_bagging_fraction)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,feature_subspace_power)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,feature_subspace_scale)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,requested_num_trees)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,decision_metric)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,scaled_stop_splitting_metric_value)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,minimum_leaf_size)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,minimum_features_considered)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,max_tree_depth_per_total_features)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,initial_stocastic_epochs)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,snapshot_separating_epochs)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,num_snapshots)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,snapshot_experience_rewind)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,internal_layer_neurons_per_categorical_dimension)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,internal_layer_neurons_per_numerical_dimension)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,minimum_num_of_internal_layer_neurons)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,PCA_upper_variation_fraction)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,num_neighbors)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,should_select_nearest_neighbors_by_validation)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,distance_voter)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,random_projection_forest_num_trees)
    PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(m_data,result,random_projection_forest_approx_leaf_size)

    return result;
}

}
}
