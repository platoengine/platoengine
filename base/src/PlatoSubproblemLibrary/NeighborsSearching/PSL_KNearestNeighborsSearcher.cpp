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

#include "PSL_KNearestNeighborsSearcher.hpp"

#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_DataFlow.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_DataSequence.hpp"

#include <vector>
#include <iostream>

namespace PlatoSubproblemLibrary
{

KNearestNeighborsSearcher::KNearestNeighborsSearcher(AbstractAuthority* authority,
                                                     const k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t& type) :
        DataFlow(authority),
        m_searcher_type(type),
        m_answer_class_outputs(NULL),
        m_answer_points(NULL),
        m_num_answers(-1)
{
}
KNearestNeighborsSearcher::~KNearestNeighborsSearcher()
{
    m_answer_class_outputs = NULL;
    m_answer_points = NULL;
}

k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t KNearestNeighborsSearcher::get_searcher_type()
{
    return m_searcher_type;
}

// for data flow
void KNearestNeighborsSearcher::before_allocate(const int& num_answers)
{
    m_num_answers = num_answers;
}
void KNearestNeighborsSearcher::allocate_data()
{
    m_data_sequence->enqueue_matrix();
    m_data_sequence->enqueue_int_vector();
}
void KNearestNeighborsSearcher::unpack_data()
{
    m_answer_class_outputs = m_data_sequence->get_int_vector(m_data_sequence->dequeue_int_vector());
    m_answer_points = m_data_sequence->get_matrix_ptr(m_data_sequence->dequeue_matrix());
}
void KNearestNeighborsSearcher::after_unpack(AbstractInterface::DenseMatrix* answer_points,
                                             const std::vector<int>& answer_class_outputs)
{
    *m_answer_class_outputs = answer_class_outputs;
    *m_answer_points = answer_points;
    m_num_answers = (*m_answer_points)->get_num_rows();
}

}
