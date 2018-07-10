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
class AbstractAuthority;
class ParameterData;
class ClassificationArchive;
class DecisionTreeNode;
class DecisionMetric;

class RandomForest : public Classifier
{
public:
    RandomForest(AbstractAuthority* authority);
    virtual ~RandomForest();

    virtual void unpack_data();
    virtual void initialize(ParameterData* parameters, ClassificationArchive* archive);
    virtual void train(ParameterData* parameters, ClassificationArchive* archive);
    virtual int classify(const std::vector<double>& input_scalars, const std::vector<int>& input_enums);

protected:

    void clear_built_data();
    void allocate_classifier_data(ParameterData* parameters, ClassificationArchive* archive);
    void recursively_create_tree(const int& current_depth,
                                 const std::vector<std::vector<double> >& input_scalars,
                                 const std::vector<bool>& consider_scalar,
                                 const std::vector<std::vector<int> >& input_enums,
                                 const std::vector<bool>& consider_enum,
                                 const std::vector<int>& output_enums,
                                 const std::vector<int>& active_rows,
                                 DecisionTreeNode* current_node);
    void make_leaf(const std::vector<int>& current_class_counts, DecisionTreeNode* current_node);
    void set_split(const bool& best_is_scalar,
                   const int& best_index,
                   const double& scalar_split,
                   const int& enum_split,
                   DecisionTreeNode* current_node);
    void attempt_scalar_split(const int& s,
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
                              int& current_num_features_considered);
    void attempt_enum_split(const int& e,
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
                            int& current_num_features_considered);

    ParameterData* m_train_parameters;
    std::vector<DecisionTreeNode*> m_forest;
    int* m_requested_num_trees;
    int* m_decision_metric_enum;
    DecisionMetric* m_metric;
    int* m_output_enum_size;
    std::vector<int> m_input_enum_sizes;
    std::vector<std::vector<int>*> m_forest_right_child_index;
    std::vector<std::vector<int>*> m_forest_split_attribute_index;
    std::vector<std::vector<double>*> m_forest_numerical_split_value;
    std::vector<std::vector<int>*> m_forest_categorical_split_value;
    std::vector<std::vector<int>*> m_forest_classification_if_leaf;

};

}
