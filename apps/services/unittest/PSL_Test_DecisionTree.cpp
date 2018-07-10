#include "PSL_UnitTestingHelper.hpp"

#include "PSL_DecisionTreeNode.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingDecisionTree
{

PSL_TEST(DecisionTree, vectorizeAdvanced)
{
    set_rand_seed();
    // allocate nodes
    DecisionTreeNode root;
    DecisionTreeNode* a = new DecisionTreeNode;
    DecisionTreeNode* b = new DecisionTreeNode;
    DecisionTreeNode* c = new DecisionTreeNode;
    DecisionTreeNode* d = new DecisionTreeNode;
    DecisionTreeNode* e = new DecisionTreeNode;
    DecisionTreeNode* f = new DecisionTreeNode;
    DecisionTreeNode* g = new DecisionTreeNode;
    DecisionTreeNode* h = new DecisionTreeNode;
    DecisionTreeNode* i = new DecisionTreeNode;
    DecisionTreeNode* j = new DecisionTreeNode;
    DecisionTreeNode* k = new DecisionTreeNode;
    DecisionTreeNode* l = new DecisionTreeNode;
    DecisionTreeNode* m = new DecisionTreeNode;
    DecisionTreeNode* n = new DecisionTreeNode;

    // fill decision structure
    root.set_categorical_split(3, 2);
    a->set_numerical_split(2, 4.12);
    b->set_numerical_split(1, 7.1);
    c->set_numerical_split(0, 1.7);
    d->set_categorical_split(0, 1);
    e->set_leaf(1);
    f->set_numerical_split(1, 2.1);
    g->set_leaf(1);
    h->set_leaf(4);
    i->set_leaf(2);
    j->set_leaf(0);
    k->set_categorical_split(2, 5);
    l->set_leaf(7);
    m->set_leaf(5);
    n->set_leaf(3);

    // pose tree structure
    root.set_children(a, b);
    a->set_children(c, d);
    c->set_children(g, h);
    d->set_children(i, j);
    b->set_children(e, f);
    f->set_children(k, l);
    k->set_children(m, n);

    //    root
    //   |    |
    //   a    b
    //  |  |  | |
    //  c  d  e f
    // | || |  | |
    // g hi j  k l
    //        | |
    //        m n

    // expect tree size
    const int tree_size = root.vectorize_from_root();
    EXPECT_EQ(tree_size, 15);

    // expect tree indexes
    EXPECT_EQ(root.get_vectorized_index(), 0); // b, 3, ~, 2, ~
    EXPECT_EQ(a->get_vectorized_index(), 1); // d, 2, 4.12, ~, ~
    EXPECT_EQ(c->get_vectorized_index(), 2); // h, 0, 1.7, ~, ~
    EXPECT_EQ(g->get_vectorized_index(), 3); // ~, ~, ~, ~, 1
    EXPECT_EQ(h->get_vectorized_index(), 4); // ~, ~, ~, ~, 4
    EXPECT_EQ(d->get_vectorized_index(), 5); // j, 0, ~, 1, ~
    EXPECT_EQ(i->get_vectorized_index(), 6); // ~, ~, ~, ~, 2
    EXPECT_EQ(j->get_vectorized_index(), 7); // ~, ~, ~, ~, 0
    EXPECT_EQ(b->get_vectorized_index(), 8); // f, 1, 7.1, ~, ~
    EXPECT_EQ(e->get_vectorized_index(), 9); // ~, ~, ~, ~, 1
    EXPECT_EQ(f->get_vectorized_index(), 10); // l, 1, 2.1, ~, ~
    EXPECT_EQ(k->get_vectorized_index(), 11); // n, 2, ~, 5, ~
    EXPECT_EQ(m->get_vectorized_index(), 12); // ~, ~, ~, ~, 5
    EXPECT_EQ(n->get_vectorized_index(), 13); // ~, ~, ~, ~, 3
    EXPECT_EQ(l->get_vectorized_index(), 14); // ~, ~, ~, ~, 7


    // compute vectorized representation
    std::vector<int> computed_right_child_index;
    std::vector<int> computed_split_attribute_index;
    std::vector<double> computed_numerical_split_value;
    std::vector<int> computed_categorical_split_value;
    std::vector<int> computed_classification_if_leaf;
    root.build_vectorization_from_root(computed_right_child_index,
                                       computed_split_attribute_index,
                                       computed_numerical_split_value,
                                       computed_categorical_split_value,
                                       computed_classification_if_leaf);

    // establish expectations
    std::vector<int> expected_right_child_index = {8, 5, 4, -1, -1, 7, -1, -1, 10, -1, 14, 13, -1, -1, -1};
    std::vector<int> expected_split_attribute_index = {3, 2, 0, -1, -1, 0, -1, -1, 1, -1, 1, 2, -1, -1, -1};
    std::vector<double> expected_numerical_split_value = {0., 4.12, 1.7, 0., 0., 0., 0., 0., 7.1, 0., 2.1, 0., 0., 0., 0.};
    std::vector<int> expected_categorical_split_value = {2, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1, 5, -1, -1, -1};
    std::vector<int> expected_classification_if_leaf = {-1, -1, -1, 1, 4, -1, 2 , 0, -1, 1, -1, -1, 5, 3, 7};

    // compare to expectations
    expect_equal_vectors(computed_right_child_index, expected_right_child_index);
    expect_equal_vectors(computed_split_attribute_index, expected_split_attribute_index);
    expect_equal_float_vectors(computed_numerical_split_value, expected_numerical_split_value);
    expect_equal_vectors(computed_categorical_split_value, expected_categorical_split_value);
    expect_equal_vectors(computed_classification_if_leaf, expected_classification_if_leaf);

    // rebuild tree from vectorizations
    DecisionTreeNode new_root;
    new_root.build_tree_from_vectorizations_as_root(computed_right_child_index,
                                                    computed_split_attribute_index,
                                                    computed_numerical_split_value,
                                                    computed_categorical_split_value,
                                                    computed_classification_if_leaf);

    // compute vectorized representation of new tree
    std::vector<int> new_computed_right_child_index;
    std::vector<int> new_computed_split_attribute_index;
    std::vector<double> new_computed_numerical_split_value;
    std::vector<int> new_computed_categorical_split_value;
    std::vector<int> new_computed_classification_if_leaf;
    new_root.build_vectorization_from_root(new_computed_right_child_index,
                                           new_computed_split_attribute_index,
                                           new_computed_numerical_split_value,
                                           new_computed_categorical_split_value,
                                           new_computed_classification_if_leaf);

    // compare to original expectations
    expect_equal_vectors(new_computed_right_child_index, expected_right_child_index);
    expect_equal_vectors(new_computed_split_attribute_index, expected_split_attribute_index);
    expect_equal_float_vectors(new_computed_numerical_split_value, expected_numerical_split_value);
    expect_equal_vectors(new_computed_categorical_split_value, expected_categorical_split_value);
    expect_equal_vectors(new_computed_classification_if_leaf, expected_classification_if_leaf);
}

}
}
