#include "PSL_UnitTestingHelper.hpp"

#include "PSL_BinaryTreeNode.hpp"
#include "PSL_Random.hpp"

namespace PlatoSubproblemLibrary
{
namespace TestingBinaryTree
{

PSL_TEST(BinaryTree, vectorizeBasic)
{
    set_rand_seed();
    // allocate nodes
    BinaryTreeNode root;
    BinaryTreeNode* l = new BinaryTreeNode;
    BinaryTreeNode* r = new BinaryTreeNode;

    // pose tree structure
    root.set_children(l, r);

    //   root
    //  |    |
    //  l    r

    // expect tree size
    const int tree_size = root.vectorize_from_root();
    EXPECT_EQ(tree_size, 3);

    // expect tree indexes
    EXPECT_EQ(root.get_vectorized_index(), 0);
    EXPECT_EQ(l->get_vectorized_index(), 1);
    EXPECT_EQ(r->get_vectorized_index(), 2);
}

PSL_TEST(BinaryTree, vectorizeSimple)
{
    set_rand_seed();
    // allocate nodes
    BinaryTreeNode root;
    BinaryTreeNode* l1 = new BinaryTreeNode;
    BinaryTreeNode* r1 = new BinaryTreeNode;
    BinaryTreeNode* l2 = new BinaryTreeNode;
    BinaryTreeNode* r2 = new BinaryTreeNode;

    // pose tree structure
    root.set_children(l1, r1);
    r1->set_children(l2, r2);

    //    root
    //   |    |
    //  l1    r1
    //       |  |
    //      l2  r2

    // expect tree size
    const int tree_size = root.vectorize_from_root();
    EXPECT_EQ(tree_size, 5);

    // expect tree indexes
    EXPECT_EQ(root.get_vectorized_index(), 0);
    EXPECT_EQ(l1->get_vectorized_index(), 1);
    EXPECT_EQ(r1->get_vectorized_index(), 2);
    EXPECT_EQ(l2->get_vectorized_index(), 3);
    EXPECT_EQ(r2->get_vectorized_index(), 4);
}

}
}
