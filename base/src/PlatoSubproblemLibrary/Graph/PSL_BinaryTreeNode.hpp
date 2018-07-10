#pragma once

namespace PlatoSubproblemLibrary
{

class BinaryTreeNode
{
public:
    BinaryTreeNode();
    virtual ~BinaryTreeNode();

    void set_children(BinaryTreeNode* left, BinaryTreeNode* right);
    BinaryTreeNode* get_left_child();
    BinaryTreeNode* get_right_child();

    int get_vectorized_index();
    int vectorize_from_root();
    int recursive_vectorize(const int& vectorized_index);

protected:
    BinaryTreeNode* m_left_child;
    BinaryTreeNode* m_right_child;
    int m_vectorized_index;

};

}

