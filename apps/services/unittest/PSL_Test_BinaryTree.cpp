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
