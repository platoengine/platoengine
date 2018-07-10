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
