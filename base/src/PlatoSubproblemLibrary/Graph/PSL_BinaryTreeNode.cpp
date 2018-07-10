#include "PSL_BinaryTreeNode.hpp"

#include "PSL_FreeHelpers.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{

BinaryTreeNode::BinaryTreeNode() :
        m_left_child(NULL),
        m_right_child(NULL),
        m_vectorized_index(-1)
{
}
BinaryTreeNode::~BinaryTreeNode()
{
    safe_free(m_left_child);
    safe_free(m_right_child);
}
void BinaryTreeNode::set_children(BinaryTreeNode* left, BinaryTreeNode* right)
{
    m_left_child = left;
    m_right_child = right;
}
BinaryTreeNode* BinaryTreeNode::get_left_child()
{
    return m_left_child;
}
BinaryTreeNode* BinaryTreeNode::get_right_child()
{
    return m_right_child;
}
int BinaryTreeNode::get_vectorized_index()
{
    return m_vectorized_index;
}
int BinaryTreeNode::vectorize_from_root()
{
    return recursive_vectorize(0);
}
int BinaryTreeNode::recursive_vectorize(const int& vectorized_index)
{
    m_vectorized_index = vectorized_index;

    if(m_left_child)
    {
        // internal node

        // set children
        const int last_from_left = m_left_child->recursive_vectorize(m_vectorized_index + 1);
        const int last_from_right = m_right_child->recursive_vectorize(last_from_left);

        return last_from_right;
    }

    // leaf node
    return m_vectorized_index + 1;
}

}

