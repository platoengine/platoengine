// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

namespace PlatoSubproblemLibrary
{

namespace tokens_t {
enum tokens_t
{
    tokens_unset,
    scale,
    absolute,
    iterations,
    symPlane_X, symPlane_Y, symPlane_Z,
    penalty,
    node_resolution_tolerance,
    spatial_searcher,
    normalization,
    reproduction,
    matrix_assembly_agent,
    symmetry_plane_agent,
    mesh_scale_agent,
    matrix_normalization_agent,
    point_ghosting_agent,
    bounded_support_function,
    batch_size,
    initial_learning_rate,
    max_epochs,
    archive_filename,
    num_internal_layers,
    internal_activation_function,
    internal_activation_function_auxiliary_parameter,
    final_activation_function,
    final_activation_function_auxiliary_parameter,
    data_loss_function,
    validation_holdout,
    validation_decreasing_epochs,
    verbose,
    perfection_accuracy,
    preprocessor,
    classifier,
    vector_kernel,
    vector_kernel_parameter,
    empirical_risk,
    archive_filetype_binary_preference,
    dataset_bagging_fraction,
    feature_subspace_power,
    feature_subspace_scale,
    requested_num_trees,
    decision_metric,
    scaled_stop_splitting_metric_value,
    minimum_leaf_size,
    minimum_features_considered,
    max_tree_depth_per_total_features,
    voting_scheme,
    initial_stocastic_epochs,
    snapshot_separating_epochs,
    num_snapshots,
    snapshot_experience_rewind,
    internal_layer_neurons_per_categorical_dimension,
    internal_layer_neurons_per_numerical_dimension,
    minimum_num_of_internal_layer_neurons,
    PCA_upper_variation_fraction,
    num_neighbors,
    should_select_nearest_neighbors_by_validation,
    distance_voter,
    random_projection_forest_num_trees,
    random_projection_forest_approx_leaf_size,
    min_heaviside_parameter,
    heaviside_continuation_scale,
    max_heaviside_parameter,
    smooth_max_p_norm,
    critical_print_angle,
    base_layer_side_set_name,
    max_overhang_angle,
    overhang_projection_angle_fraction,
    overhang_projection_radius_fraction,
    build_direction_x,
    build_direction_y,
    build_direction_z,
};
}
namespace normalization_t {
enum normalization_t
{
    unset_normalization_method,
    classical_row_normalization,
    correction_function_reproducing_conditions,
    minimal_change_to_reproducing_conditions,
};
}
namespace reproduction_level_t {
enum reproduction_level_t
{
    unset_reproduction_level,
    reproduce_constant,
    reproduce_linear,
    reproduce_quadratic,
};
}
namespace spatial_searcher_t {
enum spatial_searcher_t
{
    unset_spatial_searcher,
    recommended,
    recommended_overlap_searcher,
    bounding_box_brute_force,
    bounding_box_morton_hierarchy,
    brute_force_fixed_radius_nearest_neighbors,
    radix_grid_fixed_radius_nearest_neighbors,
    brute_force_nearest_neighbor,
};
}
namespace bounded_support_function_t {
enum bounded_support_function_t {
    unset_bounded_support_function,
    polynomial_tent_function,
    overhang_inclusion_function,
};
}
namespace matrix_assembly_agent_t {
enum matrix_assembly_agent_t
{
    unset_matrix_assembly_agent,
    by_row,
};
}
namespace symmetry_plane_agent_t {
enum symmetry_plane_agent_t
{
    unset_symmetry_plane_agent,
    by_narrow_clone,
};
}
namespace mesh_scale_agent_t {
enum mesh_scale_agent_t
{
    unset_mesh_scale_agent,
    by_average_optimized_element_side,
};
}
namespace matrix_normalization_agent_t {
enum matrix_normalization_agent_t
{
    unset_matrix_normalization_agent,
    default_agent,
};
}
namespace point_ghosting_agent_t {
enum point_ghosting_agent_t {
    unset_point_ghosting_agent,
    by_narrow_share,
};
}
namespace activation_function_t {
enum activation_function_t {
    unset_activation_function,
    sigmoid_activation,
    rectified_linear_unit,
    leaky_rectified_linear_unit,
    soft_plus_activation,
    heaviside_function,
};
}
namespace data_loss_function_t {
enum data_loss_function_t {
    unset_data_loss_function,
    squared_error_loss,
    cross_entropy_loss,
};
}
namespace preprocessor_t {
enum preprocessor_t {
    no_preprocessor = 0,
    standardization_preprocessor = 1,
    bipolar_normalization_preprocessor = 2,
    skew_normalization_preprocessor = 3,
    PCA_preprocessor = 4,
    TOTAL_NUM_PREPROCESSORS = 5,
};
}
namespace data_flow_t {
enum data_flow_t
{
    inert_data_flow = 0,
    feed_forward_neural_network_flow = 1,
    support_vector_machine_flow = 2,
    random_forest_flow = 3,
    nearest_neighbors_classifier_flow = 4,
    TOTAL_NUM_DATA_FLOWS = 5,
};
}
namespace vector_kernel_t {
enum vector_kernel_t
{
    pure_kernel = 0,
    mixed_kernel = 1,
    cross_kernel = 2,
};
}
namespace decision_metric_t {
enum decision_metric_t
{
    gini_impurity_metric = 0,
    entropy_metric = 1,
};
}
namespace invertible_unit_range_t {
enum invertible_unit_range_t
{
    logistic_function = 0,
    algebraic_function = 1,
};
}
namespace distance_voter_t {
enum distance_voter_t
{
    counting_voter = 0,
    linear_distance_voter = 1,
    inverse_distance_voter = 2,
    TOTAL_NUM_DISTANCE_VOTERS = 3,
};
}
namespace k_nearest_neighbors_searchers_t {
enum k_nearest_neighbors_searchers_t {
    brute_force_searcher = 0,
    random_projection_forest_searcher = 1,
    TOTAL_NUM_SEARCHERS = 2,
};
}

}
