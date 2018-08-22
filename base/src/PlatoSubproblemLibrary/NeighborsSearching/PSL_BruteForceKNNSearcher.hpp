#pragma once

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_KNearestNeighborsSearcher.hpp"
#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;

class BruteForceKNNSearcher : public KNearestNeighborsSearcher
{
public:
    BruteForceKNNSearcher(AbstractAuthority* authority);
    virtual ~BruteForceKNNSearcher();

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

};

}
