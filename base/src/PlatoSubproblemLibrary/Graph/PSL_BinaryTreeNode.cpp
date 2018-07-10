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

