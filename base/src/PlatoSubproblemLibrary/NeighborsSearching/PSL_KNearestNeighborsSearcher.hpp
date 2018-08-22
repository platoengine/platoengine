#pragma once

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_DataFlow.hpp"
#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class DenseMatrix;
}
class AbstractAuthority;
class ParameterData;

class KNearestNeighborsSearcher : public DataFlow
{
public:
    KNearestNeighborsSearcher(AbstractAuthority* authority,
                              const k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t& type);
    virtual ~KNearestNeighborsSearcher();

    k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t get_searcher_type();
    virtual bool is_exact() = 0;
    virtual void extract_parameters(ParameterData* parameters) = 0;

    // for data flow
    void before_allocate(const int& num_answers);
    virtual void allocate_data();
    virtual void unpack_data();
    void after_unpack(AbstractInterface::DenseMatrix* answer_points, const std::vector<int>& answer_class_outputs);

    // for searching
    virtual void initialize() = 0;
    virtual void get_neighbors(const int& num_neighbors,
                               const std::vector<double>& input,
                               std::vector<double>& worst_to_best_distances,
                               std::vector<int>& worst_to_best_class_outputs) = 0;

protected:
    k_nearest_neighbors_searchers_t::k_nearest_neighbors_searchers_t m_searcher_type;
    std::vector<int>* m_answer_class_outputs;
    AbstractInterface::DenseMatrix** m_answer_points;
    int m_num_answers;

};

}
