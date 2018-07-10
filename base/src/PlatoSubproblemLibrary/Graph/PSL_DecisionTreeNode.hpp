#pragma once

#include "PSL_BinaryTreeNode.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class DecisionTreeNode : public BinaryTreeNode
{
public:
    DecisionTreeNode();
    virtual ~DecisionTreeNode();

    // initializers
    void set_leaf(int classification);
    void set_numerical_split(int split_attribute_index, double numerical_split_value);
    void set_categorical_split(int split_attribute_index, int categorical_split_value);
    void set_children(DecisionTreeNode* left_child, DecisionTreeNode* right_child);

    bool is_leaf();

    DecisionTreeNode* get_child(const std::vector<double>& input_scalars, const std::vector<int>& input_enums);

    int get_classification();

    void build_tree_from_vectorizations_as_root(const std::vector<int>& right_child_index,
                                                const std::vector<int>& split_attribute_index,
                                                const std::vector<double>& numerical_split_value,
                                                const std::vector<int>& categorical_split_value,
                                                const std::vector<int>& classification_if_leaf);
    void recursive_build_tree_from_vectorizations(const int& vectorized_index,
                                                  const std::vector<int>& right_child_index,
                                                  const std::vector<int>& split_attribute_index,
                                                  const std::vector<double>& numerical_split_value,
                                                  const std::vector<int>& categorical_split_value,
                                                  const std::vector<int>& classification_if_leaf);

    void build_vectorization_from_root(std::vector<int>& right_child_index,
                                       std::vector<int>& split_attribute_index,
                                       std::vector<double>& numerical_split_value,
                                       std::vector<int>& categorical_split_value,
                                       std::vector<int>& classification_if_leaf);
    void recursive_build_vectorization(std::vector<int>& right_child_index,
                                       std::vector<int>& split_attribute_index,
                                       std::vector<double>& numerical_split_value,
                                       std::vector<int>& categorical_split_value,
                                       std::vector<int>& classification_if_leaf);

protected:

    int m_split_attribute_index;
    double m_numerical_split_value;
    int m_categorical_split_value;
    int m_classification_if_leaf;
    DecisionTreeNode* m_left_decision_child;
    DecisionTreeNode* m_right_decision_child;

};

}
