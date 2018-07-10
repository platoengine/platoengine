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
