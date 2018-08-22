#include "PSL_DecisionTreeNode.hpp"

#include "PSL_BinaryTreeNode.hpp"

#include <cstddef>
#include <vector>
#include <cassert>

namespace PlatoSubproblemLibrary
{

DecisionTreeNode::DecisionTreeNode() :
        BinaryTreeNode(),
        m_split_attribute_index(-1),
        m_numerical_split_value(0.),
        m_categorical_split_value(-1),
        m_classification_if_leaf(-1),
        m_left_decision_child(NULL),
        m_right_decision_child(NULL)
{
}
DecisionTreeNode::~DecisionTreeNode()
{
    // NOTE: free decision children as binary tree nodes
}

// initializers
void DecisionTreeNode::set_leaf(int classification)
{
    m_classification_if_leaf = classification;

    // must be leaf
    assert(m_left_decision_child == NULL);
}
void DecisionTreeNode::set_numerical_split(int split_attribute_index, double numerical_split_value)
{
    m_split_attribute_index = split_attribute_index;
    m_numerical_split_value = numerical_split_value;
    m_categorical_split_value = -1;

    // not classifying
    assert(m_classification_if_leaf < 0);
}
void DecisionTreeNode::set_categorical_split(int split_attribute_index, int categorical_split_value)
{
    // one versus rest categorical split
    m_split_attribute_index = split_attribute_index;
    m_numerical_split_value = 0.;
    m_categorical_split_value = categorical_split_value;

    // not classifying
    assert(m_classification_if_leaf < 0);
}
void DecisionTreeNode::set_children(DecisionTreeNode* left_child, DecisionTreeNode* right_child)
{
    // for numerical, left is less than or equal to and right is greater than
    // for categorical, left is equal to and right is everything else
    m_left_decision_child = left_child;
    m_right_decision_child = right_child;

    // set for child class
    this->BinaryTreeNode::set_children(left_child, right_child);

    // not classifying
    assert(m_classification_if_leaf < 0);
}

bool DecisionTreeNode::is_leaf()
{
    bool result = (m_left_decision_child == NULL);

    // should match
    assert( (m_right_decision_child == NULL) == result);
    return result;
}

DecisionTreeNode* DecisionTreeNode::get_child(const std::vector<double>& input_scalars, const std::vector<int>& input_enums)
{
    assert(is_leaf() == false);
    assert(m_split_attribute_index >= 0);

    if(m_categorical_split_value < 0)
    {
        // numerical split
        if(input_scalars[m_split_attribute_index] <= m_numerical_split_value)
        {
            return m_left_decision_child;
        }
        return m_right_decision_child;
    }
    // categorical split
    if(input_enums[m_split_attribute_index] == m_categorical_split_value)
    {
        return m_left_decision_child;
    }
    return m_right_decision_child;
}

int DecisionTreeNode::get_classification()
{
    assert(is_leaf() == true);
    return m_classification_if_leaf;
}

void DecisionTreeNode::build_tree_from_vectorizations_as_root(const std::vector<int>& right_child_index,
                                                              const std::vector<int>& split_attribute_index,
                                                              const std::vector<double>& numerical_split_value,
                                                              const std::vector<int>& categorical_split_value,
                                                              const std::vector<int>& classification_if_leaf)
{
    recursive_build_tree_from_vectorizations(0,
                                             right_child_index,
                                             split_attribute_index,
                                             numerical_split_value,
                                             categorical_split_value,
                                             classification_if_leaf);

}
void DecisionTreeNode::recursive_build_tree_from_vectorizations(const int& vectorized_index,
                                                                const std::vector<int>& right_child_index,
                                                                const std::vector<int>& split_attribute_index,
                                                                const std::vector<double>& numerical_split_value,
                                                                const std::vector<int>& categorical_split_value,
                                                                const std::vector<int>& classification_if_leaf)
{
    if(right_child_index[vectorized_index] < 0)
    {
        // leaf
        set_leaf(classification_if_leaf[vectorized_index]);
    }
    else
    {
        // internal node

        if(categorical_split_value[vectorized_index] < 0)
        {
            // numeric
            set_numerical_split(split_attribute_index[vectorized_index], numerical_split_value[vectorized_index]);
        }
        else
        {
            // categorical
            set_categorical_split(split_attribute_index[vectorized_index], categorical_split_value[vectorized_index]);
        }

        // set children
        DecisionTreeNode* left_child = new DecisionTreeNode;
        DecisionTreeNode* right_child = new DecisionTreeNode;
        set_children(left_child, right_child);

        // recurse
        left_child->recursive_build_tree_from_vectorizations(vectorized_index + 1,
                                                             right_child_index,
                                                             split_attribute_index,
                                                             numerical_split_value,
                                                             categorical_split_value,
                                                             classification_if_leaf);
        right_child->recursive_build_tree_from_vectorizations(right_child_index[vectorized_index],
                                                              right_child_index,
                                                              split_attribute_index,
                                                              numerical_split_value,
                                                              categorical_split_value,
                                                              classification_if_leaf);
    }
}

void DecisionTreeNode::build_vectorization_from_root(std::vector<int>& right_child_index,
                                                     std::vector<int>& split_attribute_index,
                                                     std::vector<double>& numerical_split_value,
                                                     std::vector<int>& categorical_split_value,
                                                     std::vector<int>& classification_if_leaf)
{
    // get size
    const int tree_size = vectorize_from_root();

    // allocate
    right_child_index.resize(tree_size, -1);
    split_attribute_index.resize(tree_size, -1);
    numerical_split_value.resize(tree_size, 0.);
    categorical_split_value.resize(tree_size, -1);
    classification_if_leaf.resize(tree_size, -1);

    // fill values
    recursive_build_vectorization(right_child_index,
                                  split_attribute_index,
                                  numerical_split_value,
                                  categorical_split_value,
                                  classification_if_leaf);
}
void DecisionTreeNode::recursive_build_vectorization(std::vector<int>& right_child_index,
                                                     std::vector<int>& split_attribute_index,
                                                     std::vector<double>& numerical_split_value,
                                                     std::vector<int>& categorical_split_value,
                                                     std::vector<int>& classification_if_leaf)
{
    // set values
    if(m_right_child)
    {
        right_child_index[m_vectorized_index] = m_right_child->get_vectorized_index();
    }
    else
    {
        right_child_index[m_vectorized_index] = -1;
    }
    split_attribute_index[m_vectorized_index] = m_split_attribute_index;
    numerical_split_value[m_vectorized_index] = m_numerical_split_value;
    categorical_split_value[m_vectorized_index] = m_categorical_split_value;
    classification_if_leaf[m_vectorized_index] = m_classification_if_leaf;

    // recurse
    if(m_left_decision_child)
    {
        m_left_decision_child->recursive_build_vectorization(right_child_index,
                                                             split_attribute_index,
                                                             numerical_split_value,
                                                             categorical_split_value,
                                                             classification_if_leaf);
        m_right_decision_child->recursive_build_vectorization(right_child_index,
                                                              split_attribute_index,
                                                              numerical_split_value,
                                                              categorical_split_value,
                                                              classification_if_leaf);
    }
}

}

