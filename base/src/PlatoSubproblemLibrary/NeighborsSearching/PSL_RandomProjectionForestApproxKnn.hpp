#pragma once

#include "PSL_KNearestNeighborsSearcher.hpp"
#include <vector>
#include <queue>
#include <functional>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;

class RandomProjectionForestApproxKnn : public KNearestNeighborsSearcher
{
public:
    RandomProjectionForestApproxKnn(AbstractAuthority* authority);
    virtual ~RandomProjectionForestApproxKnn();

    virtual bool is_exact();
    virtual void extract_parameters(ParameterData* parameters);

    // for data flow
    virtual void unpack_data();
    virtual void allocate_data();

    // for searching
    virtual void initialize();
    virtual void get_neighbors(const int& num_neighbors,
                               const std::vector<double>& input_to_search,
                               std::vector<double>& worst_to_best_distances,
                               std::vector<int>& worst_to_best_class_outputs);

protected:

    void recursively_build_project_forest(const int& tree,
                                          const int& this_level,
                                          const int& this_cell,
                                          const std::vector<std::vector<std::vector<double> > >& row_tree_level,
                                          const std::vector<int>& active_rows);
    void recursively_follow_project_forest(const int& tree,
                                           const int& this_level,
                                           const int& this_cell,
                                           const int& num_neighbors,
                                           const std::vector<double>& query_point,
                                           std::priority_queue<std::pair<double, int> >& distance_then_outputClass,
                                           std::vector<bool>& have_searched,
                                           const std::vector<double>& projected_values);

    int m_initial_num_trees;
    int m_initial_approx_leaf_size;

    int* m_num_trees;
    int* m_num_levels;
    std::vector<std::vector<std::vector<int>*> > m_tree_then_leaf_then_rows;
    std::vector<std::vector<double>*> m_tree_then_split_points;
    std::vector<AbstractInterface::DenseMatrix**> m_random_projections;

};

}
