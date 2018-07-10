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
#include "PSL_RandomForest.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_DecisionMetric.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_GiniImpurityMetric.hpp"
#include "PSL_EntropyMetric.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_DecisionMetricFactory.hpp"
#include "PSL_Classifier.hpp"
#include "PSL_Random.hpp"
#include "PSL_DecisionTreeNode.hpp"
#include "PSL_DecisionMetricFactory.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <string>

namespace PlatoSubproblemLibrary
{

RandomForest::RandomForest(AbstractAuthority* authority) :
        Classifier(authority),
        m_train_parameters(),
        m_forest(),
        m_metric(),
        m_output_enum_size(NULL),
        m_input_enum_sizes()
{
    m_data_sequence->set_data_flow_type(data_flow_t::data_flow_t::random_forest_flow);
}
RandomForest::~RandomForest()
{
    m_train_parameters = NULL;

    // clear forest
    const size_t num_trees = m_forest.size();
    for(size_t t = 0u; t < num_trees; t++)
    {
        safe_free(m_forest[t]);
    }
    m_forest.clear();

    clear_built_data();
}
void RandomForest::clear_built_data()
{
    safe_free(m_metric);
}

void RandomForest::allocate_classifier_data(ParameterData* parameters, ClassificationArchive* archive)
{
    // get sizes
    std::vector<int> input_enum_sizes;
    int output_enum_size;
    archive->get_enum_size(input_enum_sizes, output_enum_size);
    const int requested_num_trees = parameters->get_requested_num_trees();

    // allocate integers
    m_data_sequence->enqueue_int(output_enum_size);
    m_data_sequence->enqueue_int(parameters->get_decision_metric());
    m_data_sequence->enqueue_int(requested_num_trees);

    // allocate doubles

    // allocate integer vectors
    const int local_needed_int_vector = 4 * requested_num_trees;
    m_data_sequence->enqueue_int_vector(local_needed_int_vector);

    // allocate double vectors
    const int local_needed_double_vector = 1 * requested_num_trees;
    m_data_sequence->enqueue_double_vector(local_needed_double_vector);

    // allocate matrices
}

void RandomForest::unpack_data()
{
    clear_built_data();

    // unpack integer scalars
    m_output_enum_size = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_decision_metric_enum = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());
    m_requested_num_trees = m_data_sequence->get_int_ptr(m_data_sequence->dequeue_int());

    // build preprocessor
    safe_free(m_metric);
    m_metric = build_decision_metric(decision_metric_t::decision_metric_t(*m_decision_metric_enum), m_authority->utilities);

    // unpack double scalars

    // unpack integer vectors
    // unpack integer forest vectors
    m_forest_right_child_index.resize(*m_requested_num_trees, NULL);
    m_forest_split_attribute_index.resize(*m_requested_num_trees, NULL);
    m_forest_categorical_split_value.resize(*m_requested_num_trees, NULL);
    m_forest_classification_if_leaf.resize(*m_requested_num_trees, NULL);
    for(int tree = 0; tree < *m_requested_num_trees; tree++)
    {
        m_forest_right_child_index[tree] = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());
        m_forest_split_attribute_index[tree] = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());
        m_forest_categorical_split_value[tree] = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());
        m_forest_classification_if_leaf[tree] = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());
    }

    // unpack double vectors
    // unpack double forest vectors
    m_forest_numerical_split_value.resize(*m_requested_num_trees, NULL);
    for(int tree = 0; tree < *m_requested_num_trees; tree++)
    {
        m_forest_numerical_split_value[tree] = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
    }

    // unpack matrices

    // conditionally unpack a trained forest
    const bool unbuilt_forest = (m_forest.size() == 0u);
    const bool have_trained_forest = (m_forest_right_child_index[0]->size() > 0u);
    if(unbuilt_forest && have_trained_forest)
    {
        // allocate trees
        assert(0 < *m_requested_num_trees);
        m_forest.resize(*m_requested_num_trees, NULL);
        for(int t = 0; t < *m_requested_num_trees; t++)
        {
            // build trees
            m_forest[t] = new DecisionTreeNode;
            m_forest[t]->build_tree_from_vectorizations_as_root(*m_forest_right_child_index[t],
                                                                *m_forest_split_attribute_index[t],
                                                                *m_forest_numerical_split_value[t],
                                                                *m_forest_categorical_split_value[t],
                                                                *m_forest_classification_if_leaf[t]);
        }
    }
}

void RandomForest::initialize(ParameterData* parameters, ClassificationArchive* archive)
{
    // allocate forest
    allocate_classifier_data(parameters, archive);

    // associate pointers to underlying data
    unpack_data();
}

void RandomForest::train(ParameterData* parameters, ClassificationArchive* archive)
{
    m_train_parameters = parameters;

    // get sizes
    archive->get_enum_size(m_input_enum_sizes, *m_output_enum_size);

    // get dataset
    std::vector<std::vector<double> > input_scalars;
    std::vector<std::vector<int> > input_enums;
    std::vector<int> output_enums;
    archive->get_all_rows(input_scalars, input_enums, output_enums);
    const int num_dataset_rows = output_enums.size();
    assert(0 < num_dataset_rows);
    const double dataset_bagging_fraction = parameters->get_dataset_bagging_fraction();
    assert(0. < dataset_bagging_fraction);
    assert(dataset_bagging_fraction <= 1.);
    const int num_bag_rows = ceil(dataset_bagging_fraction * double(num_dataset_rows));

    // allocate forest
    assert(m_forest.size() == 0u);
    const int requested_num_trees = parameters->get_requested_num_trees();
    assert(requested_num_trees > 0);
    m_forest.resize(requested_num_trees, NULL);

    // for each tree
    const size_t num_trees = m_forest.size();
    for(size_t t = 0u; t < num_trees; t++)
    {
        if(m_train_parameters->get_verbose())
        {
            m_authority->utilities->print(std::string("begin train tree ") + std::to_string(t) + std::string(" of ")
                                          + std::to_string(num_trees)
                                          + std::string("\n"));
        }

        // consider all features
        const int num_scalars = input_scalars[0].size();
        const int num_enums = m_input_enum_sizes.size();
        std::vector<bool> consider_scalar(num_scalars, true);
        std::vector<bool> consider_enum(num_enums, true);

        // bag dataset
        std::vector<int> active_rows;
        random_permutations(num_dataset_rows, num_bag_rows, active_rows);

        // build this tree
        m_forest[t] = new DecisionTreeNode;
        recursively_create_tree(0,
                                input_scalars,
                                consider_scalar,
                                input_enums,
                                consider_enum,
                                output_enums,
                                active_rows,
                                m_forest[t]);

        // TODO: this only needs to be done if saving to file; performance opt
        // fill vectorized tree representation
        m_forest[t]->build_vectorization_from_root(*m_forest_right_child_index[t],
                                                   *m_forest_split_attribute_index[t],
                                                   *m_forest_numerical_split_value[t],
                                                   *m_forest_categorical_split_value[t],
                                                   *m_forest_classification_if_leaf[t]);
    }
}
int RandomForest::classify(const std::vector<double>& input_scalars, const std::vector<int>& input_enums)
{
    // count occurrences of each classification
    std::vector<int> occurrences(*m_output_enum_size, 0);

    // for each tree
    const size_t num_trees = m_forest.size();
    for(size_t t = 0u; t < num_trees; t++)
    {
        // child walk through tree until leaf
        DecisionTreeNode* current_node = m_forest[t];
        while(!(current_node->is_leaf()))
        {
            current_node = current_node->get_child(input_scalars, input_enums);
        }

        // at leaf, get classification for tree
        const int this_tree_classification = current_node->get_classification();

        // count occurrence
        assert(0 <= this_tree_classification);
        assert(this_tree_classification < *m_output_enum_size);
        occurrences[this_tree_classification]++;
    }

    // return most occurring classification
    const int classification_result = rand_max_index(occurrences);
    return classification_result;
}

void RandomForest::recursively_create_tree(const int& current_depth,
                                           const std::vector<std::vector<double> >& input_scalars,
                                           const std::vector<bool>& consider_scalar,
                                           const std::vector<std::vector<int> >& input_enums,
                                           const std::vector<bool>& consider_enum,
                                           const std::vector<int>& output_enums,
                                           const std::vector<int>& active_rows,
                                           DecisionTreeNode* current_node)
{
    // compute current metric
    std::vector<int> current_class_counts(*m_output_enum_size, 0);
    const int num_active = active_rows.size();
    for(int a = 0; a < num_active; a++)
    {
        const int row = active_rows[a];
        const int output_enum = output_enums[row];
        current_class_counts[output_enum]++;
    }
    const double current_metric_value = m_metric->measure(current_class_counts);
    const double scaled_current_metric_value = m_metric->lower_cutoff_upscale() * current_metric_value;

    if(m_train_parameters->get_verbose())
    {
        m_authority->utilities->print(std::string("\tactive size ") + std::to_string(num_active) + std::string(", metric value ")
                                      + std::to_string(current_metric_value)
                                      + std::string("\n"));
    }

    // count features
    const int num_scalars = consider_scalar.size();
    const int num_enums = consider_enum.size();
    const int total_num_features = num_scalars + num_enums;

    // consider for childern
    std::vector<bool> consider_scalar_for_children = consider_scalar;
    std::vector<bool> consider_enum_for_children = consider_enum;

    // consider stop splitting criteria
    const double max_tree_depth_per_total_features = m_train_parameters->get_max_tree_depth_per_total_features();
    const bool stop_by_depth = (0. < max_tree_depth_per_total_features)
                               && (double(total_num_features) * max_tree_depth_per_total_features <= current_depth);

    const double scaled_stop_splitting_metric_value = m_train_parameters->get_scaled_stop_splitting_metric_value();
    const bool stop_by_metric_value = (scaled_current_metric_value <= scaled_stop_splitting_metric_value);

    const int minimum_leaf_size = m_train_parameters->get_minimum_leaf_size();
    const bool stop_by_active_size = (num_active <= minimum_leaf_size);
    if(stop_by_depth || stop_by_metric_value || stop_by_active_size)
    {
        // stop splitting; current is a leaf
        make_leaf(current_class_counts, current_node);
    }
    else
    {
        // keep splitting! current is internal node

        bool best_is_scalar = false;
        int best_index = -1;
        double best_metric = 0.;
        std::vector<int> left_child_active_rows;
        std::vector<int> right_child_active_rows;
        double scalar_split = 0.;
        double enum_split = -1;

        // consider a subset of the features
        const int full_feature_size = num_scalars + num_enums;
        const double feature_subspace_scale = m_train_parameters->get_feature_subspace_scale();
        const double feature_subspace_power = m_train_parameters->get_feature_subspace_power();
        const int minimum_subset_features = ceil(feature_subspace_scale
                                                 * std::pow(double(full_feature_size), feature_subspace_power));
        const int absolute_minimum_features_considered = m_train_parameters->get_minimum_features_considered();
        const int minimum_features_considered = std::max(absolute_minimum_features_considered, minimum_subset_features);
        int current_num_features_considered = 0;

        // get a permutation
        std::vector<int> random_permutation_of_features;
        random_permutation(full_feature_size, random_permutation_of_features);
        assert(int(random_permutation_of_features.size()) == full_feature_size);

        for(int f_index = 0; f_index < full_feature_size; f_index++)
        {
            // stop considering features if should
            const bool done_minimum_amount = (minimum_features_considered <= current_num_features_considered);
            const bool found_a_best = (0 <= best_index);
            if(done_minimum_amount && found_a_best)
            {
                break;
            }

            // get feature
            const int f = random_permutation_of_features[f_index];

            // if scalar
            if(f < num_scalars)
            {
                const int s = f;
                attempt_scalar_split(s,
                                     input_scalars,
                                     output_enums,
                                     active_rows,
                                     current_class_counts,
                                     consider_scalar_for_children,
                                     best_is_scalar,
                                     best_index,
                                     best_metric,
                                     left_child_active_rows,
                                     right_child_active_rows,
                                     scalar_split,
                                     enum_split,
                                     current_num_features_considered);
            }
            else
            {
                // else enum
                const int e = f - num_scalars;
                attempt_enum_split(e,
                                   input_enums,
                                   output_enums,
                                   active_rows,
                                   consider_enum_for_children,
                                   best_is_scalar,
                                   best_index,
                                   best_metric,
                                   left_child_active_rows,
                                   right_child_active_rows,
                                   scalar_split,
                                   enum_split,
                                   current_num_features_considered);
            }
        }

        if(best_index < 0)
        {
            // no best, current node is a leaf
            make_leaf(current_class_counts, current_node);
        }
        else
        {
            // found a best, current node is internal
            set_split(best_is_scalar, best_index, scalar_split, enum_split, current_node);

            // allocate children
            DecisionTreeNode* left_child = new DecisionTreeNode;
            DecisionTreeNode* right_child = new DecisionTreeNode;
            current_node->set_children(left_child, right_child);

            // recurse for left child
            recursively_create_tree(current_depth + 1,
                                    input_scalars,
                                    consider_scalar_for_children,
                                    input_enums,
                                    consider_enum_for_children,
                                    output_enums,
                                    left_child_active_rows,
                                    left_child);

            // recurse for right child
            recursively_create_tree(current_depth + 1,
                                    input_scalars,
                                    consider_scalar_for_children,
                                    input_enums,
                                    consider_enum_for_children,
                                    output_enums,
                                    right_child_active_rows,
                                    right_child);
        }
    }
}

void RandomForest::make_leaf(const std::vector<int>& current_class_counts, DecisionTreeNode* current_node)
{
    const int index_of_current_mode = rand_max_index(current_class_counts);
    current_node->set_leaf(index_of_current_mode);

    if(m_train_parameters->get_verbose())
    {
        m_authority->utilities->print(std::string("\t\tnode is leaf, classification ") + std::to_string(index_of_current_mode)
                                      + std::string("\n"));
    }
}

void RandomForest::set_split(const bool& best_is_scalar,
                   const int& best_index,
                   const double& scalar_split,
                   const int& enum_split,
                   DecisionTreeNode* current_node)
{
    if(best_is_scalar)
    {
        current_node->set_numerical_split(best_index, scalar_split);
        if(m_train_parameters->get_verbose())
        {
            m_authority->utilities->print(std::string("\t\tnode is internal, numeric splitting of index ")
                                          + std::to_string(best_index)
                                          + std::string(" at value ")
                                          + std::to_string(scalar_split)
                                          + std::string("\n"));
        }
    }
    else
    {
        current_node->set_categorical_split(best_index, enum_split);
        if(m_train_parameters->get_verbose())
        {
            m_authority->utilities->print(std::string("\t\tnode is internal, categorical splitting of index ")
                                          + std::to_string(best_index)
                                          + std::string(" for value ")
                                          + std::to_string(enum_split)
                                          + std::string("\n"));
        }
    }
}

void RandomForest::attempt_scalar_split(const int& s,
                                        const std::vector<std::vector<double> >& input_scalars,
                                        const std::vector<int>& output_enums,
                                        const std::vector<int>& active_rows,
                                        const std::vector<int>& current_class_counts,
                                        std::vector<bool>& consider_scalar_for_children,
                                        bool& best_is_scalar,
                                        int& best_index,
                                        double& best_metric,
                                        std::vector<int>& left_child_active_rows,
                                        std::vector<int>& right_child_active_rows,
                                        double& scalar_split,
                                        double& enum_split,
                                        int& current_num_features_considered)
{
    // if not considering, skip
    if(!consider_scalar_for_children[s])
    {
        return;
    }

    // get active scalars
    const int num_active = active_rows.size();
    std::vector<std::pair<double, int> > these_active_scalars(num_active);
    for(int a = 0; a < num_active; a++)
    {
        const int row = active_rows[a];
        const double scalar = input_scalars[row][s];
        these_active_scalars[a] = std::make_pair(scalar, row);
    }

    // sort scalars
    std::sort(these_active_scalars.begin(), these_active_scalars.end());

    std::vector<int> left_class_counts(*m_output_enum_size, 0);
    std::vector<int> right_class_counts = current_class_counts;

    bool considered_atleast_one_split = false;
    int this_scalar_new_best_num_left = -1;
    for(int a = 0; a < num_active - 1; a++)
    {
        // shift a'th
        const int ath_row_index = these_active_scalars[a].second;
        left_class_counts[output_enums[ath_row_index]]++;
        right_class_counts[output_enums[ath_row_index]]--;

        const double left = these_active_scalars[a].first;
        const double right = these_active_scalars[a + 1].first;

        if(left != right)
        {
            considered_atleast_one_split = true;

            const int num_left = a + 1;
            const int num_right = num_active - num_left;
            const double weight_left = double(num_left) / double(num_active);
            const double weight_right = double(num_right) / double(num_active);

            // measure
            const double left_measure = m_metric->measure(left_class_counts);
            const double right_measure = m_metric->measure(right_class_counts);

            // weighted sum
            const double split_measure = weight_left * left_measure + weight_right * right_measure;

            // determine if new best
            const bool no_best_found_so_far = (best_index < 0);
            const bool improving_best = (split_measure < best_metric);

            if(m_train_parameters->get_verbose())
            {
                m_authority->utilities->print(std::string("\t\t\tconsidering numerical split of category ")
                                                    + std::to_string(s)
                                                    + std::string(" at value ")
                                                    + std::to_string((left + right) * 0.5)
                                                    + std::string(", no best ")
                                                    + std::to_string(no_best_found_so_far)
                                                    + std::string(", improving best ")
                                                    + std::to_string(improving_best)
                                                    + std::string(", num left ")
                                                    + std::to_string(num_left)
                                                    + std::string(", num right ")
                                                    + std::to_string(num_right)
                                                    + std::string(", split measure ")
                                                    + std::to_string(split_measure)
                                                    + std::string("\n"));
            }

            if(no_best_found_so_far || improving_best)
            {
                // new best
                this_scalar_new_best_num_left = num_left;

                // update
                best_is_scalar = true;
                best_index = s;
                best_metric = split_measure;

                enum_split = -1;

                // set split
                const double center = (left + right) * .5;
                scalar_split = center;
            }
        }
    }

    // if this scalar is new best, update left and right active rows
    if(0 < this_scalar_new_best_num_left)
    {
        const int num_left = this_scalar_new_best_num_left;
        const int num_right = num_active - num_left;

        // build active rows
        left_child_active_rows.resize(num_left);
        for(int left_index = 0; left_index < num_left; left_index++)
        {
            left_child_active_rows[left_index] = these_active_scalars[left_index].second;
        }
        right_child_active_rows.resize(num_right);
        for(int right_index = 0; right_index < num_right; right_index++)
        {
            const int active_right = right_index + num_left;
            right_child_active_rows[right_index] = these_active_scalars[active_right].second;
        }
    }

    if(considered_atleast_one_split)
    {
        // considered; count it
        current_num_features_considered++;
    }
    else
    {
        // not considered; avoid in children
        consider_scalar_for_children[s] = false;
    }
}

void RandomForest::attempt_enum_split(const int& e,
                                      const std::vector<std::vector<int> >& input_enums,
                                      const std::vector<int>& output_enums,
                                      const std::vector<int>& active_rows,
                                      std::vector<bool>& consider_enum_for_children,
                                      bool& best_is_scalar,
                                      int& best_index,
                                      double& best_metric,
                                      std::vector<int>& left_child_active_rows,
                                      std::vector<int>& right_child_active_rows,
                                      double& scalar_split,
                                      double& enum_split,
                                      int& current_num_features_considered)
{
    // if not considering, skip
    if(!consider_enum_for_children[e])
    {
        return;
    }

    // get active rows for each input
    const int num_active = active_rows.size();
    const int this_input_enum_size = m_input_enum_sizes[e];
    std::vector<std::vector<int> > rows_per_input_enum(this_input_enum_size);
    std::vector<std::vector<int> > class_counts_per_input_enum(this_input_enum_size, std::vector<int>(*m_output_enum_size, 0));
    for(int a = 0; a < num_active; a++)
    {
        const int row = active_rows[a];
        const int input_enum = input_enums[row][e];
        assert(0 <= input_enum);
        assert(input_enum < this_input_enum_size);
        rows_per_input_enum[input_enum].push_back(row);
        class_counts_per_input_enum[input_enum][output_enums[row]]++;
    }

    // input enum
    bool considered_atleast_one_split = false;
    for(int ie = 0; ie < this_input_enum_size; ie++)
    {
        // if splitting would be useless, skip
        const int num_left = rows_per_input_enum[ie].size();
        if(num_left == 0 || num_left == num_active)
        {
            continue;
        }
        considered_atleast_one_split = true;
        const int num_right = num_active - num_left;

        // count left
        const double left_measure = m_metric->measure(class_counts_per_input_enum[ie]);

        // count right
        std::vector<int> class_counts(*m_output_enum_size, 0);
        for(int ie_right = 0; ie_right < this_input_enum_size; ie_right++)
        {
            if(ie_right != ie)
            {
                axpy(1, class_counts_per_input_enum[ie_right], class_counts);
            }
        }
        const double right_measure = m_metric->measure(class_counts);

        // weighted sum
        const double weight_left = double(num_left) / double(num_active);
        const double weight_right = double(num_right) / double(num_active);
        const double split_measure = weight_left * left_measure + weight_right * right_measure;

        // determine if new best
        const bool no_best_found_so_far = (best_index < 0);
        const bool improving_best = (split_measure < best_metric);

        if(m_train_parameters->get_verbose())
        {
            m_authority->utilities->print(std::string("\t\t\tconsidering categorical split of category ") + std::to_string(e)
                                          + std::string(" at value ")
                                          + std::to_string(ie)
                                          + std::string(", no best ")
                                          + std::to_string(no_best_found_so_far)
                                          + std::string(", improving best ")
                                          + std::to_string(improving_best)
                                          + std::string(", num left ")
                                          + std::to_string(num_left)
                                          + std::string(", num right ")
                                          + std::to_string(num_right)
                                          + std::string(", split measure ")
                                          + std::to_string(split_measure)
                                          + std::string("\n"));
        }

        if(no_best_found_so_far || improving_best)
        {
            // new best

            // update
            best_is_scalar = false;
            best_index = e;
            best_metric = split_measure;

            // set split
            scalar_split = 0.;
            enum_split = ie;
        }
    }

    // if this enum is best, build active rows
    if((best_is_scalar == false) && (best_index == e))
    {
        assert(0 <= enum_split);
        assert(enum_split < this_input_enum_size);

        left_child_active_rows = rows_per_input_enum[enum_split];
        right_child_active_rows.clear();
        for(int ie_right = 0; ie_right < this_input_enum_size; ie_right++)
        {
            if(ie_right != enum_split)
            {
                right_child_active_rows.insert(right_child_active_rows.end(),
                                               rows_per_input_enum[ie_right].begin(),
                                               rows_per_input_enum[ie_right].end());
            }
        }
    }

    // mark if actually considered at least one
    if(considered_atleast_one_split)
    {
        current_num_features_considered++;
    }
    else
    {
        // not considered; avoid in children
        consider_enum_for_children[e] = false;
    }
}

}
